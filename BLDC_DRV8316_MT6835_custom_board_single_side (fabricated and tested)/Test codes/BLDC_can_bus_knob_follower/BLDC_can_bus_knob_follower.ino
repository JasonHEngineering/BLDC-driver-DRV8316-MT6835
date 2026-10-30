/*
  ================================================================
  BLDC "follow the encoder" over CAN  -  sender / receiver
  ESP32-C6 + DRV8316CR + MT6835, two identical boards on one CAN bus
  ================================================================
  Arduino IDE: "esp32 by Espressif Systems" >= 3.0.0, board "ESP32C6 Dev Module".
  Motor: GB2806, 7 pole pairs.

  ONE sketch, TWO roles. Set IS_SENDER below, then flash each board:
     IS_SENDER = true   -> board A: reads the MT6835 at 1 kHz and broadcasts the
                           angle on CAN. Its motor drivers are left Hi-Z (coasting),
                           so you can turn its shaft by hand like a "master dial".
     IS_SENDER = false  -> board B: calibrates, then drives its motor to follow the
                           angle received from board A.

  CAN frame (standard 11-bit ID 0x100, 8 bytes, 500 kbit/s, sent every 1 ms):
     byte 0..2  raw 21-bit MT6835 angle (little endian)
     byte 3     MT6835 status bits
     byte 4..5  angular velocity, int16, units of 0.01 rad/s (little endian)
     byte 6     sequence counter
     byte 7     reserved

  ---------------------------------------------------------------
  CHANGES vs. previous version (review fixes)
  ---------------------------------------------------------------
  1. Encoder integrity
     * MT6835 reads now include the CRC byte and are rejected on mismatch
       (ENC_CHECK_CRC). A CRC self-test runs at start-up.
     * Sender skips a frame when its encoder read fails.
     * Receiver rejects frames whose status reports weak field / undervoltage,
       and frames whose angle step is physically implausible.
     * Sequence gaps > RX_MAX_GAP (or a streak of implausible frames) cause a
       clean resync instead of a possibly wrong multi-turn unwrap.
     * Receiver's own encoder: bad reads are skipped; ENC_BAD_LIMIT bad reads in
       a row -> brake until ENC_RECOVER_GOOD good reads in a row.
  2. Leash no longer slides the offset on every overshoot
     * The error is only saturated for the controller. In RELATIVE mode the
       target is re-anchored only after the leash has been saturated for
       LEASH_REANCHOR_US (follower genuinely blocked or sender too fast).
       In ABSOLUTE mode the offset is never modified, so alignment is kept.
     * Stall protection: large error + no motion for STALL_US -> voltage
       limited to STALL_UQ_LIMIT until the rotor moves again.
  3. DRV8316 fault recovery
     * CTRL2 is written read-modify-write (only PWM_MODE / CLR_FLT changed).
     * While faulted, CLR_FLT is issued every FAULT_RETRY_US so latched faults
       can clear. Too many clear attempts, or too many fault events within
       FAULT_WINDOW_US -> permanent lockout (braked) until reset.
  4. Smaller fixes
     * Extrapolation age clamped to >= 0 (frame newer than loop timestamp).
     * Status bit names corrected in the start-up printout (verify vs. datasheet).
     * Control loop execution time measured and printed (loopMax) - the C6 has
       no FPU, so check this stays well below CTRL_PERIOD_US (200 us).

  ---------------------------------------------------------------
  v3: RESPONSIVENESS
  ---------------------------------------------------------------
  * Stiffer default gains: velocity PI 0.12 V/(rad/s) + 3.0 (was 0.05 + 0.5),
    position P 30 (was 15), speed limit 60 rad/s (was 30), Uq limit 5 V (was 3).
  * Faster velocity estimates: receiver filter ~1 ms (was ~4 ms), sender ~1 ms
    (was ~5 ms), so the feed-forward and the damping react sooner.
  * Back-EMF voltage feed-forward (KFF): the voltage needed to spin at the
    target speed is applied immediately instead of waiting for the integrator
    to build it up. Starts at 0; while the follower spins steadily the RX
    printout shows "u/vel" - set kff to about 80 % of that value.
  * Live tuning over Serial (115200 baud, newline): type "help". Changes are
    not saved - copy the values you like into the *_DEFAULT constants.

  Hardware needed: a CAN transceiver on each board (GPIO4 = TX, GPIO5 = RX),
  CANH/CANL wired between boards, common GND, 120 ohm termination at each END of
  the bus (so 2 x 120 ohm for two nodes).
  ================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "driver/mcpwm_prelude.h"
#include "driver/twai.h"
#include "esp_timer.h"

// ---------------------------------------------------------------
// ROLE  <<<<<<<<<<<<<<  set per board
// ---------------------------------------------------------------
constexpr bool IS_SENDER = false;     // true = sender board, false = receiver board

// ---------------------------------------------------------------
// Pin assignment
// ---------------------------------------------------------------
constexpr int PIN_INHA = 2,  PIN_INLA = 3;
constexpr int PIN_INHB = 0,  PIN_INLB = 1;
constexpr int PIN_INHC = 7,  PIN_INLC = 14;
constexpr int PIN_NFAULT  = 15;      // open-drain, active low
constexpr int PIN_DRV_CS  = 23;
constexpr int PIN_ENC_CS  = 22;
constexpr int PIN_SCLK    = 19;
constexpr int PIN_MOSI    = 20;
constexpr int PIN_MISO    = 21;
constexpr int PIN_CAN_TX  = 4;
constexpr int PIN_CAN_RX  = 5;

// ---------------------------------------------------------------
// User configuration  (TUNE THESE)
// ---------------------------------------------------------------
// Motor / power (receiver)
constexpr int      POLE_PAIRS        = 7;
constexpr float    VBUS              = 24.0f;   // supply voltage [V]  <-- set to your real supply
constexpr float    UQ_LIMIT_DEFAULT  = 5.0f;    // max phase voltage [V] (was 3) - watch motor temperature
constexpr float    ALIGN_VOLTAGE     = 2.0f;    // first calibration voltage [V]
constexpr float    ALIGN_VOLTAGE_MAX = 5.0f;    // calibration retries x1.5 up to this [V]

// Control (receiver)
// The *_DEFAULT values (and UQ_LIMIT_DEFAULT) can be changed live over Serial - type "help".
constexpr float    KP_DEFAULT        = 0.12f;   // velocity loop P  [V per rad/s]          (was 0.05)
constexpr float    KI_DEFAULT        = 3.0f;    // velocity loop I  [V per rad/s per s]    (was 0.5)
constexpr float    KP_POS_DEFAULT    = 30.0f;   // position loop P  [rad/s per rad]        (was 15)
constexpr float    KFF_DEFAULT       = 0.0f;    // back-EMF feed-forward [V per rad/s] - set from "u/vel"
constexpr float    VEL_LIMIT_DEFAULT = 60.0f;   // max follower speed [rad/s]              (was 30)
constexpr float    VEL_ALPHA         = 0.2f;    // receiver speed filter, per 200 us (~1 ms) (was 0.05)
constexpr float    SENDER_VEL_ALPHA  = 0.5f;    // sender speed filter, per 1 ms   (~1 ms) (was 0.2)
constexpr float    LEASH_RAD         = 1.0f;    // max error used by the controller [rad]
constexpr int64_t  LEASH_REANCHOR_US = 300000;  // relative mode: re-anchor after leash saturated this long
constexpr int      FOLLOW_SIGN       = +1;      // +1 same direction, -1 mirrored
constexpr bool     ABSOLUTE_FOLLOW   = false;   // false = relative (recommended), true = absolute
constexpr int32_t  ABS_ZERO_COUNTS   = 0;       // absolute mode: receiver zero offset [encoder counts]
constexpr int64_t  RX_TIMEOUT_US     = 50000;   // no frame for this long -> hold position

// Stall protection (receiver)
constexpr float    STALL_ERR_RAD     = 0.5f;    // error above this ...
constexpr float    STALL_VEL         = 0.5f;    // ... while slower than this [rad/s] ...
constexpr int64_t  STALL_US          = 500000;  // ... for this long = stalled
constexpr float    STALL_UQ_LIMIT    = 1.0f;    // voltage limit while stalled [V]

// Encoder integrity
constexpr bool     ENC_CHECK_CRC     = true;    // set false only if the start-up CRC test always fails
constexpr float    SENDER_MAX_SPEED  = 300.0f;  // plausibility limit for received angle steps [rad/s]
constexpr float    OWN_MAX_SPEED     = 300.0f;  // plausibility limit for own encoder steps [rad/s]
constexpr uint8_t  RX_MAX_GAP        = 5;       // seq gap above this -> resync instead of unwrap
constexpr int      RX_MAX_BAD_STREAK = 20;      // implausible frames in a row -> resync
constexpr int      ENC_BAD_LIMIT     = 10;      // own bad reads in a row (2 ms) -> brake
constexpr int      ENC_RECOVER_GOOD  = 50;      // own good reads in a row (10 ms) -> resume

// DRV8316 fault handling (receiver)
constexpr int64_t  FAULT_RETRY_US    = 500000;  // CLR_FLT interval / required fault-free time
constexpr int      FAULT_MAX_CLEARS  = 10;      // CLR_FLT attempts per fault event before lockout
constexpr int      FAULT_MAX_EVENTS  = 5;       // fault events within the window before lockout
constexpr int64_t  FAULT_WINDOW_US   = 10000000;

// CAN / timing
constexpr uint32_t CAN_ID_ANGLE      = 0x100;
constexpr uint32_t SEND_PERIOD_US    = 1000;    // sender rate: 1 kHz
constexpr uint32_t CTRL_PERIOD_US    = 200;     // receiver control loop: 5 kHz

// PWM
constexpr uint32_t TIMER_RES_HZ      = 20000000;
constexpr uint32_t PWM_FREQ_HZ       = 25000;
constexpr uint32_t PERIOD_TICKS      = TIMER_RES_HZ / PWM_FREQ_HZ;  // 800
constexpr uint32_t PEAK_TICKS        = PERIOD_TICKS / 2;
constexpr uint32_t DEADTIME_TICKS    = 10;                          // 500 ns

// ---------------------------------------------------------------
// Globals
// ---------------------------------------------------------------
constexpr float    SQRT3_2        = 0.8660254f;
constexpr uint32_t ENC_MASK       = 0x1FFFFF;                   // 21 bit
constexpr float    RAD_PER_COUNT  = TWO_PI / 2097152.0f;

// MT6835 status bits (STATUS[2:0]) - verify against your datasheet revision
constexpr uint8_t  ENC_ST_OVERSPEED = 0x01;
constexpr uint8_t  ENC_ST_WEAK_MAG  = 0x02;
constexpr uint8_t  ENC_ST_UV        = 0x04;
constexpr uint8_t  ENC_ST_BAD       = ENC_ST_WEAK_MAG | ENC_ST_UV;   // angle not trustworthy

static mcpwm_timer_handle_t gTimer;
static mcpwm_oper_handle_t  gOper[3];
static mcpwm_cmpr_handle_t  gCmpr[3];
static mcpwm_gen_handle_t   gGenH[3];
static mcpwm_gen_handle_t   gGenL[3];

static TaskHandle_t gCtrlTask = nullptr;     // receiver control task / sender task
static float   gSensorDir = 1.0f;
static float   gZeroElec  = 0.0f;
static uint8_t gCtrl2     = 0;               // DRV8316 CTRL2 value (without CLR_FLT)

// Latest data from CAN (written by canRxTask, read by controlTask)
struct RxSnap {
  int64_t  counts;      // unwrapped sender angle [counts]
  uint32_t raw;         // sender single-turn angle [counts]
  float    vel;         // sender velocity [rad/s]
  int64_t  tUs;         // local arrival time
  uint32_t epoch;       // incremented whenever the unwrap had to be restarted
  bool     valid;
};
static RxSnap     gRx = {0, 0, 0.0f, 0, 0, false};
static portMUX_TYPE gRxMux = portMUX_INITIALIZER_UNLOCKED;

// Telemetry for loop()
static volatile float    gTelPos = 0, gTelTarget = 0, gTelErr = 0, gTelVel = 0, gTelUq = 0;
static volatile bool     gTelLink = false, gTelStall = false, gTelEncLost = false;
static volatile uint32_t gRxCount = 0, gRxLost = 0, gRxRejected = 0, gTxCount = 0, gTxFail = 0;
static volatile uint32_t gEncErr = 0, gLoopMaxUs = 0;
static volatile float    gTelAngle = 0;
static volatile bool     gFaultActive = false, gFaultReport = false, gFaultLockout = false;
static volatile bool     gEncLostReport = false;
static volatile uint8_t  gFaultRegs[3] = {0, 0, 0};
static volatile float    gTelErrMax = 0, gTelUperVel = 0;
static volatile bool     gPrintOn = true;

// Live-tunable control parameters (written by loop(), read by controlTask)
static volatile float gKp = KP_DEFAULT, gKi = KI_DEFAULT, gKpPos = KP_POS_DEFAULT;
static volatile float gKff = KFF_DEFAULT, gVelLimit = VEL_LIMIT_DEFAULT, gUqLimit = UQ_LIMIT_DEFAULT;

// ---------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------
static inline float wrapPi(float a) {
  while (a >  PI) a -= TWO_PI;
  while (a <= -PI) a += TWO_PI;
  return a;
}
static inline float wrap2Pi(float a) {
  a = fmodf(a, TWO_PI);
  if (a < 0) a += TWO_PI;
  return a;
}
// signed difference of two 21-bit angles, exact integer arithmetic
static inline int32_t deltaCounts(uint32_t now, uint32_t prev) {
  uint32_t d = (now - prev) & ENC_MASK;
  return (d & 0x100000) ? (int32_t)d - 0x200000 : (int32_t)d;
}

// ---------------------------------------------------------------
// MT6835 encoder (burst read 0xA003: ANGLE[20:13], ANGLE[12:5],
// ANGLE[4:0]|STATUS[2:0], CRC).  SPI mode 3.
// CRC-8, polynomial x^8 + x^2 + x + 1 (0x07), init 0x00, over the three
// angle/status bytes. If the start-up self-test reports 100 % CRC errors,
// check this definition against your datasheet revision.
// ---------------------------------------------------------------
static bool encRead(uint32_t *raw, uint8_t *status) {
  uint8_t tx[6] = {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
  uint8_t rx[6] = {0};
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ENC_CS, LOW);
  SPI.transferBytes(tx, rx, 6);
  digitalWrite(PIN_ENC_CS, HIGH);
  SPI.endTransaction();

  if (ENC_CHECK_CRC) {
    uint8_t crc = 0;
    for (int i = 2; i <= 4; i++) {
      crc ^= rx[i];
      for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    if (crc != rx[5]) return false;
  }
  if (status) *status = rx[4] & 0x07;
  *raw = (((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | (rx[4] >> 3)) & ENC_MASK;
  return true;
}

// For non-time-critical use (setup, calibration): retries, falls back to last good value.
static uint32_t encReadRetry(uint8_t *status) {
  static uint32_t lastGood = 0;
  uint32_t raw = 0;
  uint8_t st = 0;
  for (int i = 0; i < 3; i++) {
    if (encRead(&raw, &st)) {
      lastGood = raw;
      if (status) *status = st;
      return raw;
    }
  }
  gEncErr++;
  if (status) *status = 0xFF;      // 0xFF = read failed
  return lastGood;
}
static float encReadAngle() { return encReadRetry(nullptr) * RAD_PER_COUNT; }

// ---------------------------------------------------------------
// DRV8316 SPI
// ---------------------------------------------------------------
static uint8_t drvReadReg(uint8_t addr) {
  uint16_t cmd = 0x8000 | ((uint16_t)(addr & 0x3F) << 9);
  if (__builtin_parity(cmd)) cmd |= 0x0100;      // make total parity even

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_DRV_CS, LOW);
  delayMicroseconds(1);
  uint16_t r = SPI.transfer16(cmd);
  delayMicroseconds(1);
  digitalWrite(PIN_DRV_CS, HIGH);
  SPI.endTransaction();
  return r & 0xFF;
}

static void drvWriteReg(uint8_t addr, uint8_t data) {
  uint16_t cmd = ((uint16_t)(addr & 0x3F) << 9) | data;     // R/W bit = 0 (write)
  if (__builtin_parity(cmd)) cmd |= 0x0100;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_DRV_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer16(cmd);
  delayMicroseconds(1);
  digitalWrite(PIN_DRV_CS, HIGH);
  SPI.endTransaction();
}

static void drvPrintStatus() {
  // Read one at a time (argument evaluation order in printf is unspecified)
  uint8_t ic = drvReadReg(0x00), s1 = drvReadReg(0x01), s2 = drvReadReg(0x02);
  uint8_t c2 = drvReadReg(0x04), c4 = drvReadReg(0x06);
  Serial.printf("  IC_STAT=0x%02X STAT1=0x%02X STAT2=0x%02X CTRL2=0x%02X CTRL4=0x%02X\n", ic, s1, s2, c2, c4);
  Serial.printf("  IC_STAT: FAULT=%d OT=%d OVP=%d NPOR(1=ok)=%d OCP=%d SPI_FLT=%d BK_FLT=%d\n",
                ic & 1, (ic >> 1) & 1, (ic >> 2) & 1, (ic >> 3) & 1, (ic >> 4) & 1, (ic >> 5) & 1, (ic >> 6) & 1);
  Serial.printf("  STAT2: VCP_UV=%d BUCK_UV=%d BUCK_OCP=%d OTP_ERR=%d | CTRL4.DRV_OFF=%d | PWM_MODE=%d (0 = 6x)\n",
                (s2 >> 3) & 1, (s2 >> 4) & 1, (s2 >> 5) & 1, (s2 >> 6) & 1, (c4 >> 7) & 1, (c2 >> 1) & 3);
}

static void drvClearFault() {
  drvWriteReg(0x03, 0x03);                // CTRL1: unlock (in case it was re-locked)
  drvWriteReg(0x04, gCtrl2 | 0x01);       // CTRL2: CLR_FLT (self-clearing)
}

static void drvInit() {
  Serial.println("DRV8316 before init:");
  drvPrintStatus();

  drvWriteReg(0x03, 0x03);                // CTRL1: unlock registers
  uint8_t c2 = drvReadReg(0x04);
  gCtrl2 = c2 & ~0x07;                    // PWM_MODE = 00 (6x), CLR_FLT = 0, keep SLEW / SDO_MODE
  drvWriteReg(0x04, gCtrl2 | 0x01);       // + CLR_FLT
  delay(5);

  Serial.printf("DRV8316 after unlock + CLR_FLT + 6x mode (CTRL2 was 0x%02X):\n", c2);
  drvPrintStatus();
}

// ---------------------------------------------------------------
// MCPWM
// ---------------------------------------------------------------
static void pwmInit() {
  const int pinsH[3] = {PIN_INHA, PIN_INHB, PIN_INHC};
  const int pinsL[3] = {PIN_INLA, PIN_INLB, PIN_INLC};

  mcpwm_timer_config_t tc = {};
  tc.group_id      = 0;
  tc.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
  tc.resolution_hz = TIMER_RES_HZ;
  tc.count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN;   // center aligned
  tc.period_ticks  = PERIOD_TICKS;
  ESP_ERROR_CHECK(mcpwm_new_timer(&tc, &gTimer));

  for (int i = 0; i < 3; i++) {
    mcpwm_operator_config_t oc = {};
    oc.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oc, &gOper[i]));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(gOper[i], gTimer));

    mcpwm_comparator_config_t cc = {};
    cc.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(gOper[i], &cc, &gCmpr[i]));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(gCmpr[i], PEAK_TICKS / 2));

    mcpwm_generator_config_t gc = {};
    gc.gen_gpio_num = pinsH[i];
    ESP_ERROR_CHECK(mcpwm_new_generator(gOper[i], &gc, &gGenH[i]));
    gc.gen_gpio_num = pinsL[i];
    ESP_ERROR_CHECK(mcpwm_new_generator(gOper[i], &gc, &gGenL[i]));

    // High-side reference waveform: HIGH at counter=0, LOW on compare-up,
    // HIGH again on compare-down  =>  duty = compare / PEAK_TICKS
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(gGenH[i],
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(gGenH[i],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,   gCmpr[i], MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, gCmpr[i], MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    // Dead time. NOTE: API order is (in_generator, out_generator, config).
    // Low side = inverted copy of the high-side signal, with delay on the falling edge.
    mcpwm_dead_time_config_t dt = {};
    dt.posedge_delay_ticks = DEADTIME_TICKS;
    dt.negedge_delay_ticks = 0;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gGenH[i], gGenH[i], &dt));

    dt = {};
    dt.posedge_delay_ticks = 0;
    dt.negedge_delay_ticks = DEADTIME_TICKS;
    dt.flags.invert_output = true;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gGenH[i], gGenL[i], &dt));
  }

  // Start in "brake" (all high sides off, all low sides on) before running
  for (int i = 0; i < 3; i++) mcpwm_generator_set_force_level(gGenH[i], 0, true);

  ESP_ERROR_CHECK(mcpwm_timer_enable(gTimer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(gTimer, MCPWM_TIMER_START_NO_STOP));
}

// brake = true : INH = 0, INL = 1 on all phases (low-side short)
// brake = false: normal PWM
static void setBrake(bool brake) {
  for (int i = 0; i < 3; i++)
    mcpwm_generator_set_force_level(gGenH[i], brake ? 0 : -1, true);
}

static inline void setDuty(int ch, float d) {
  if (d < 0.02f) d = 0.02f;
  if (d > 0.98f) d = 0.98f;
  mcpwm_comparator_set_compare_value(gCmpr[ch], (uint32_t)(d * PEAK_TICKS));
}

// Inverse Park + inverse Clarke + zero-sequence centering (SVPWM equivalent)
static void setPhaseVoltages(float ud, float uq, float theta_e) {
  float s = sinf(theta_e), c = cosf(theta_e);
  float ua_ = ud * c - uq * s;       // alpha
  float ub_ = ud * s + uq * c;       // beta

  float a = ua_;
  float b = -0.5f * ua_ + SQRT3_2 * ub_;
  float cph = -0.5f * ua_ - SQRT3_2 * ub_;

  float vmax = fmaxf(a, fmaxf(b, cph));
  float vmin = fminf(a, fminf(b, cph));
  float mid  = 0.5f * (vmax + vmin);

  setDuty(0, (a   - mid) / VBUS + 0.5f);
  setDuty(1, (b   - mid) / VBUS + 0.5f);
  setDuty(2, (cph - mid) / VBUS + 0.5f);
}

// ---------------------------------------------------------------
// Sensor calibration
// ---------------------------------------------------------------
static bool calibrateSensor(float uAlign) {
  Serial.printf("Calibrating sensor with %.1f V - motor will move a little...\n", uAlign);

  setPhaseVoltages(uAlign, 0, 0);
  setBrake(false);
  delay(1500);                                   // let rotor settle at 0 deg

  float prev  = encReadAngle();
  float total = 0.0f;                            // signed mech. displacement [rad]

  const int STEPS = 24;                          // one electrical revolution, 15 deg steps
  for (int k = 1; k <= STEPS; k++) {
    setPhaseVoltages(uAlign, 0, k * (TWO_PI / STEPS));
    delay(40);
    float a = encReadAngle();
    total += wrapPi(a - prev);
    prev = a;
  }
  delay(500);
  float fin = encReadAngle();
  total += wrapPi(fin - prev);

  float expected = TWO_PI / POLE_PAIRS;
  Serial.printf("  mech. move per electrical rev: %.3f rad (expected %.3f for %d pole pairs)\n",
                fabsf(total), expected, POLE_PAIRS);
  if (fabsf(total) > 0.01f)
    Serial.printf("  -> estimated pole pairs: %.1f\n", TWO_PI / fabsf(total));

  if (fabsf(total) < 0.3f * expected) {
    Serial.println("  ERROR: rotor did not follow. Check VBUS, align voltage, wiring, DRV state.");
    return false;
  }
  if (fabsf(fabsf(total) - expected) > 0.15f * expected)
    Serial.println("  WARNING: pole-pair count looks wrong - fix POLE_PAIRS before running!");

  gSensorDir = (total >= 0.0f) ? 1.0f : -1.0f;
  gZeroElec  = wrap2Pi(gSensorDir * POLE_PAIRS * fin);   // vector is at 0 rad now
  Serial.printf("  sensor dir = %+.0f, zero electrical = %.3f rad\n", gSensorDir, gZeroElec);
  return true;
}

// ---------------------------------------------------------------
// CAN (TWAI) helpers
// ---------------------------------------------------------------
static bool canInit() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g.tx_queue_len = 4;
  g.rx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) { Serial.println("CAN: driver install failed"); return false; }
  if (twai_start() != ESP_OK)                    { Serial.println("CAN: start failed");          return false; }
  Serial.println("CAN: started at 500 kbit/s");
  return true;
}

// Called periodically from loop(): recover from bus-off, print status
static void canSupervise(bool print) {
  twai_status_info_t si;
  if (twai_get_status_info(&si) != ESP_OK) return;
  if (si.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
  else if (si.state == TWAI_STATE_STOPPED) twai_start();
  if (print)
    Serial.printf("  CAN state=%d txErr=%u rxErr=%u busErrs=%u\n",
                  (int)si.state, (unsigned)si.tx_error_counter, (unsigned)si.rx_error_counter,
                  (unsigned)si.bus_error_count);
}

// ---------------------------------------------------------------
// SENDER: encoder -> CAN at 1 kHz.  Motor stays Hi-Z (INH = INL = 0).
// ---------------------------------------------------------------
static void motorCoast() {
  const int pins[6] = {PIN_INHA, PIN_INLA, PIN_INHB, PIN_INLB, PIN_INHC, PIN_INLC};
  for (int i = 0; i < 6; i++) { pinMode(pins[i], OUTPUT); digitalWrite(pins[i], LOW); }
}

static void senderTask(void *) {
  uint32_t prevRaw = encReadRetry(nullptr);
  float vel = 0.0f;
  uint8_t seq = 0;
  int64_t tPrev = esp_timer_get_time();

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t now = esp_timer_get_time();

    uint8_t st = 0;
    uint32_t raw = 0;
    if (!encRead(&raw, &st)) { gEncErr++; continue; }   // skip this frame; receiver extrapolates

    float dt = (now - tPrev) * 1e-6f;                   // time since last GOOD sample
    tPrev = now;
    if (dt <= 0.0f) continue;

    int32_t dC = deltaCounts(raw, prevRaw);
    prevRaw = raw;
    vel += SENDER_VEL_ALPHA * ((dC * RAD_PER_COUNT) / dt - vel);

    int32_t v100 = (int32_t)(vel * 100.0f);
    v100 = constrain(v100, -32768, 32767);

    twai_message_t m = {};
    m.identifier = CAN_ID_ANGLE;
    m.data_length_code = 8;
    m.ss = 1;                        // single shot: never queue stale angles if nobody ACKs
    m.data[0] = raw & 0xFF;
    m.data[1] = (raw >> 8) & 0xFF;
    m.data[2] = (raw >> 16) & 0xFF;
    m.data[3] = st;
    m.data[4] = v100 & 0xFF;
    m.data[5] = (v100 >> 8) & 0xFF;
    m.data[6] = seq++;
    m.data[7] = 0;

    if (twai_transmit(&m, 0) == ESP_OK) gTxCount++; else gTxFail++;
    gTelAngle = raw * RAD_PER_COUNT;
    gTelVel   = vel;
  }
}

// ---------------------------------------------------------------
// RECEIVER: CAN rx task - unwraps the sender angle exactly (integer counts)
// Rejects untrustworthy / implausible frames; restarts the unwrap (epoch++)
// when continuity can no longer be guaranteed.
// ---------------------------------------------------------------
static void canRxTask(void *) {
  bool first = true;
  uint32_t lastRaw = 0;
  uint8_t lastSeq = 0;
  int64_t counts = 0;
  uint32_t epoch = 0;
  int badStreak = 0;
  const float maxStepPerFrameC = SENDER_MAX_SPEED * (SEND_PERIOD_US * 1e-6f) / RAD_PER_COUNT;

  for (;;) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(50)) != ESP_OK) continue;
    if (m.extd || m.rtr || m.identifier != CAN_ID_ANGLE || m.data_length_code != 8) continue;

    uint32_t raw = ((uint32_t)m.data[0] | ((uint32_t)m.data[1] << 8) | ((uint32_t)m.data[2] << 16)) & ENC_MASK;
    uint8_t  st  = m.data[3];
    int16_t v100 = (int16_t)((uint16_t)m.data[4] | ((uint16_t)m.data[5] << 8));
    uint8_t seq  = m.data[6];

    // Sender says its angle is not trustworthy -> ignore (counts as a gap later)
    if (st & ENC_ST_BAD) { gRxRejected++; continue; }

    bool resync = first;
    int32_t d = 0;
    if (!first) {
      uint8_t gap = (uint8_t)(seq - lastSeq);
      if (gap == 0) continue;                                  // duplicate
      d = deltaCounts(raw, lastRaw);
      if (gap > RX_MAX_GAP) {
        resync = true;                                         // too long to unwrap safely
      } else if (fabsf((float)d) > maxStepPerFrameC * gap) {
        gRxRejected++;                                         // implausible jump
        if (++badStreak < RX_MAX_BAD_STREAK) continue;
        resync = true;                                         // persistent -> accept as new reference
      } else if (gap > 1) {
        gRxLost += gap - 1;
      }
    }
    badStreak = 0;
    first = false;
    if (resync) epoch++; else counts += d;
    lastRaw = raw;
    lastSeq = seq;

    portENTER_CRITICAL(&gRxMux);
    gRx.counts = counts;
    gRx.raw    = raw;
    gRx.vel    = v100 * 0.01f;
    gRx.tUs    = esp_timer_get_time();
    gRx.epoch  = epoch;
    gRx.valid  = true;
    portEXIT_CRITICAL(&gRxMux);
    gRxCount++;
  }
}

// ---------------------------------------------------------------
// RECEIVER: control task (5 kHz)
//   CAN target -> position P -> (+ velocity feed-forward) -> velocity PI -> Uq
//   All position/velocity values are in the ENCODER frame; the sign needed to
//   turn that into a motor-frame voltage is gSensorDir (found by calibration).
// ---------------------------------------------------------------
static void controlTask(void *) {
  uint32_t prevRaw = encReadRetry(nullptr);
  int64_t posCounts = 0, targetCounts = 0, offset = 0;
  bool synced = false;
  uint32_t syncedEpoch = 0;
  float vel = 0.0f, integ = 0.0f;
  int64_t tPrev = esp_timer_get_time();
  const int64_t LEASH_C = (int64_t)(LEASH_RAD / RAD_PER_COUNT);
  const float   NOMINAL_DT = CTRL_PERIOD_US * 1e-6f;

  // DRV8316 fault state
  int faultCnt = 0, clearAttempts = 0, faultEvents = 0;
  bool faulted = false, lockout = false;
  int64_t faultHighSince = 0, lastClrT = 0, faultWindowStart = 0;

  // own encoder health
  int encBad = 0, encGood = 0;
  bool encLost = false;

  // leash / stall timers
  int64_t satSince = 0, stallSince = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t now = esp_timer_get_time();

    // ================= nFAULT supervision =================
    bool nf = (digitalRead(PIN_NFAULT) == LOW);
    faultCnt = nf ? faultCnt + 1 : 0;

    if (!faulted && faultCnt >= 3) {                 // new fault event
      faulted = true;
      setBrake(true);
      integ = 0;
      gFaultRegs[0] = drvReadReg(0x00);
      gFaultRegs[1] = drvReadReg(0x01);
      gFaultRegs[2] = drvReadReg(0x02);
      gFaultActive = true;
      gFaultReport = true;

      if (faultWindowStart == 0 || now - faultWindowStart > FAULT_WINDOW_US) {
        faultWindowStart = now;
        faultEvents = 0;
      }
      if (++faultEvents > FAULT_MAX_EVENTS) lockout = true;
      clearAttempts = 0;
      lastClrT = now;
      faultHighSince = 0;
    }

    if (faulted) {
      if (lockout) { gFaultLockout = true; continue; }   // stays braked until reset

      if (nf) {
        faultHighSince = 0;
        if (now - lastClrT > FAULT_RETRY_US) {           // latched fault: try CLR_FLT
          if (++clearAttempts > FAULT_MAX_CLEARS) { lockout = true; gFaultLockout = true; continue; }
          drvClearFault();
          lastClrT = now;
        }
      } else {
        if (faultHighSince == 0) faultHighSince = now;
        if (now - faultHighSince > FAULT_RETRY_US) {     // fault-free long enough -> resume
          faulted = false;
          gFaultActive = false;
          prevRaw = encReadRetry(nullptr);
          tPrev = now;
          vel = 0; integ = 0;
          synced = false;
          satSince = stallSince = 0;
          if (!encLost) setBrake(false);
        }
      }
      continue;
    }

    // ================= own encoder: checked read =================
    float dt = (now - tPrev) * 1e-6f;                    // time since last GOOD sample
    if (dt <= 0.0f) continue;

    uint32_t raw = 0;
    uint8_t  st  = 0;
    bool ok = encRead(&raw, &st) && !(st & ENC_ST_BAD);
    int32_t dC = 0;
    if (ok && !encLost) {
      dC = deltaCounts(raw, prevRaw);
      float maxC = OWN_MAX_SPEED * dt / RAD_PER_COUNT + 2000.0f;
      if (fabsf((float)dC) > maxC) ok = false;           // implausible jump
    }

    if (!ok) {
      gEncErr++;
      encGood = 0;
      if (!encLost && ++encBad >= ENC_BAD_LIMIT) {
        encLost = true;
        integ = 0;
        setBrake(true);
        gEncLostReport = true;
        gTelEncLost = true;
      }
      continue;                                          // keep last duties for this cycle
    }
    encBad = 0;

    if (encLost) {                                       // need a run of good reads to resume
      if (++encGood < ENC_RECOVER_GOOD) continue;
      encLost = false;
      gTelEncLost = false;
      encGood = 0;
      dC = 0;
      dt = NOMINAL_DT;
      vel = 0; integ = 0;
      synced = false;                                    // movement during the outage is unknown
      satSince = stallSince = 0;
      setBrake(false);
    }

    prevRaw = raw;
    tPrev = now;
    posCounts += dC;
    vel += VEL_ALPHA * ((dC * RAD_PER_COUNT) / dt - vel);

    // ================= target from CAN =================
    RxSnap s;
    portENTER_CRITICAL(&gRxMux);
    s = gRx;
    portEXIT_CRITICAL(&gRxMux);

    int64_t ageUs = now - s.tUs;                         // may be negative (frame newer than 'now')
    bool fresh = s.valid && ageUs < RX_TIMEOUT_US;
    float vff = 0.0f, ext = 0.0f;

    if (fresh) {
      if (!synced || s.epoch != syncedEpoch) {           // (re)synchronise
        if (ABSOLUTE_FOLLOW) {
          uint32_t tgtRaw = (uint32_t)((int64_t)FOLLOW_SIGN * s.raw + ABS_ZERO_COUNTS) & ENC_MASK;
          offset = (posCounts + deltaCounts(tgtRaw, raw)) - (int64_t)FOLLOW_SIGN * s.counts;
        } else {
          offset = posCounts - (int64_t)FOLLOW_SIGN * s.counts;
        }
        synced = true;
        syncedEpoch = s.epoch;
        integ = 0;
        satSince = 0;
      }
      targetCounts = offset + (int64_t)FOLLOW_SIGN * s.counts;
      float age = ageUs * 1e-6f;
      if (age < 0.0f)   age = 0.0f;
      if (age > 0.005f) age = 0.005f;                    // extrapolate at most 5 ms
      vff = FOLLOW_SIGN * s.vel;
      ext = vff * age;
    } else {
      synced = false;                                    // hold last target, resync later
    }

    // ================= leash =================
    // Saturate the error for the controller only. Re-anchor (relative mode
    // only) when it stays saturated, i.e. follower blocked / sender too fast.
    int64_t errC = targetCounts - posCounts;
    if (errC > LEASH_C || errC < -LEASH_C) {
      int64_t lim = (errC > 0) ? LEASH_C : -LEASH_C;
      if (satSince == 0) satSince = now;
      if (!ABSOLUTE_FOLLOW && now - satSince > LEASH_REANCHOR_US) {
        int64_t sh = errC - lim;
        offset -= sh;
        targetCounts -= sh;
        satSince = 0;
      }
      errC = lim;
    } else {
      satSince = 0;
    }
    float errRad = (float)errC * RAD_PER_COUNT + ext;

    // ================= stall protection =================
    if (fabsf(errRad) > STALL_ERR_RAD && fabsf(vel) < STALL_VEL) {
      if (stallSince == 0) stallSince = now;
    } else {
      stallSince = 0;
    }
    bool stalled = stallSince != 0 && (now - stallSince > STALL_US);
    const float uqMax = gUqLimit;
    float uLim = stalled ? fminf(STALL_UQ_LIMIT, uqMax) : uqMax;

    // ================= cascade: position P (+FF) -> velocity PI =================
    const float vLim = gVelLimit;
    float velSp = constrain(gKpPos * errRad + vff, -vLim, vLim);
    float verr  = velSp - vel;
    float uff   = gKff * velSp;                          // back-EMF feed-forward: voltage for target speed
    integ += gKi * verr * dt;
    integ = constrain(integ, -uLim, uLim);
    float u = constrain(uff + gKp * verr + integ, -uLim, uLim);   // + = want encoder angle to increase

    // ================= commutation =================
    float angle  = raw * RAD_PER_COUNT;
    float thetaE = wrap2Pi(gSensorDir * POLE_PAIRS * angle - gZeroElec);
    setPhaseVoltages(0.0f, gSensorDir * u, thetaE);

    // ================= telemetry =================
    gTelPos = (float)posCounts * RAD_PER_COUNT;
    gTelTarget = (float)targetCounts * RAD_PER_COUNT;
    gTelErr = errRad; gTelVel = vel; gTelUq = u; gTelLink = fresh; gTelStall = stalled;
    if (fabsf(errRad) > gTelErrMax) gTelErrMax = fabsf(errRad);
    gTelUperVel = (fabsf(vel) > 3.0f) ? u / vel : 0.0f;  // for choosing kff

    uint32_t loopUs = (uint32_t)(esp_timer_get_time() - now);
    if (loopUs > gLoopMaxUs) gLoopMaxUs = loopUs;
  }
}

// ---------------------------------------------------------------
// Live tuning over Serial (receiver). One command per line, e.g. "vkp 0.15"
// ---------------------------------------------------------------
static void printGains() {
  Serial.printf("GAINS  vkp=%.3f  vki=%.2f  pkp=%.1f  kff=%.3f  vlim=%.1f  uq=%.2f\n",
                gKp, gKi, gKpPos, gKff, gVelLimit, gUqLimit);
}

static void printHelp() {
  Serial.println("Commands (receiver), one per line:");
  Serial.println("  vkp <V per rad/s>     velocity loop P   (more = stiffer, too much = buzz)");
  Serial.println("  vki <V per rad/s/s>   velocity loop I   (more = less lag, too much = overshoot)");
  Serial.println("  pkp <rad/s per rad>   position loop P   (more = snappier, too much = ringing)");
  Serial.println("  kff <V per rad/s>     back-EMF feed-forward (~0.8 x u/vel while spinning)");
  Serial.println("  vlim <rad/s>          max follower speed");
  Serial.println("  uq <V>                max phase voltage (watch motor temperature)");
  Serial.println("  log 0|1               periodic status printout off/on");
  Serial.println("  show                  print current values");
}

static void handleSerial() {
  static char buf[48];
  static int n = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (n < (int)sizeof(buf) - 1) buf[n++] = c; continue; }
    buf[n] = 0;
    n = 0;

    char *cmd = strtok(buf, " \t");
    if (!cmd) continue;
    char *arg = strtok(nullptr, " \t");
    char *end = nullptr;
    float v = arg ? strtof(arg, &end) : NAN;
    bool hasVal = arg && end != arg && isfinite(v);

    if (!strcmp(cmd, "help")) { printHelp();  continue; }
    if (!strcmp(cmd, "show")) { printGains(); continue; }
    if (!strcmp(cmd, "log") && hasVal) { gPrintOn = (v != 0.0f); continue; }
    if (!hasVal) { Serial.println("? expected: <command> <number>   (type \"help\")"); continue; }

    if      (!strcmp(cmd, "vkp"))  gKp       = constrain(v, 0.0f, 2.0f);
    else if (!strcmp(cmd, "vki"))  gKi       = constrain(v, 0.0f, 100.0f);
    else if (!strcmp(cmd, "pkp"))  gKpPos    = constrain(v, 0.0f, 200.0f);
    else if (!strcmp(cmd, "kff"))  gKff      = constrain(v, 0.0f, 1.0f);
    else if (!strcmp(cmd, "vlim")) gVelLimit = constrain(v, 1.0f, 200.0f);
    else if (!strcmp(cmd, "uq"))   gUqLimit  = constrain(v, 0.5f, 0.55f * VBUS);   // SVPWM linear range
    else { Serial.println("? unknown command (type \"help\")"); continue; }
    printGains();
  }
}

// ---------------------------------------------------------------
// Start-up for each role
// ---------------------------------------------------------------
static void startTimerFor(esp_timer_cb_t cb, uint32_t periodUs, const char *name) {
  esp_timer_create_args_t ta = {};
  ta.callback = cb;
  ta.name = name;
  esp_timer_handle_t h;
  ESP_ERROR_CHECK(esp_timer_create(&ta, &h));
  ESP_ERROR_CHECK(esp_timer_start_periodic(h, periodUs));
}

static void startSender() {
  Serial.println("ROLE: SENDER (motor outputs Hi-Z, shaft can be turned by hand)");
  motorCoast();
  if (!canInit()) { while (true) delay(1000); }
  xTaskCreate(senderTask, "sender", 4096, nullptr, configMAX_PRIORITIES - 2, &gCtrlTask);
  startTimerFor([](void *) { xTaskNotifyGive(gCtrlTask); }, SEND_PERIOD_US, "send_tick");
  Serial.println("Sending angle frames.");
}

static void startReceiver() {
  Serial.println("ROLE: RECEIVER (motor follows the angle received on CAN)");
  pwmInit();                                   // starts braked
  drvInit();
  Serial.printf("nFAULT pin = %s\n", digitalRead(PIN_NFAULT) ? "OK (high)" : "ACTIVE (low)");

  bool calOk = false;
  if (digitalRead(PIN_NFAULT) == HIGH) {
    for (float u = ALIGN_VOLTAGE; u <= ALIGN_VOLTAGE_MAX + 0.01f && !calOk; u *= 1.5f)
      calOk = calibrateSensor(u);
  }
  if (!calOk) {
    setBrake(true);
    Serial.println("Calibration failed - halting (braked).");
    drvPrintStatus();
    while (true) delay(1000);
  }

  if (!canInit()) { setBrake(true); while (true) delay(1000); }
  xTaskCreate(canRxTask, "can_rx", 3072, nullptr, configMAX_PRIORITIES - 3, nullptr);
  xTaskCreate(controlTask, "ctrl", 4096, nullptr, configMAX_PRIORITIES - 2, &gCtrlTask);
  startTimerFor([](void *) { xTaskNotifyGive(gCtrlTask); }, CTRL_PERIOD_US, "ctrl_tick");
  Serial.println("Running - waiting for CAN frames; will hold position until they arrive.");
  printGains();
  Serial.println("Type \"help\" for live tuning commands.");
}

// ---------------------------------------------------------------
// Encoder self-test (CRC + status) at start-up
// ---------------------------------------------------------------
static void encSelfTest() {
  int crcFails = 0;
  uint32_t raw = 0;
  uint8_t st = 0;
  for (int i = 0; i < 20; i++) {
    if (!encRead(&raw, &st)) crcFails++;
    delay(1);
  }
  Serial.printf("Encoder angle %.3f rad, status 0x%X (bit0=overspeed, bit1=weak mag, bit2=undervoltage)\n",
                raw * RAD_PER_COUNT, st);
  Serial.printf("Encoder CRC check: %d / 20 failed%s\n", crcFails, ENC_CHECK_CRC ? "" : " (CRC check disabled)");
  if (ENC_CHECK_CRC && crcFails == 20)
    Serial.println("  WARNING: every read failed CRC. Check the CRC definition vs. your MT6835 datasheet,\n"
                   "           or set ENC_CHECK_CRC = false. The receiver will stay braked otherwise.");
  else if (crcFails > 0)
    Serial.println("  NOTE: occasional CRC errors - check SPI wiring / noise from motor leads.");
  if (st & ENC_ST_BAD)
    Serial.println("  WARNING: encoder reports weak magnet or undervoltage.");
}

// ---------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nBLDC CAN follow - ESP32-C6 / DRV8316 / MT6835");

  pinMode(PIN_DRV_CS, OUTPUT);  digitalWrite(PIN_DRV_CS, HIGH);
  pinMode(PIN_ENC_CS, OUTPUT);  digitalWrite(PIN_ENC_CS, HIGH);
  pinMode(PIN_NFAULT, INPUT_PULLUP);          // GPIO15 is a strapping pin on the C6
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

  encSelfTest();

  if (IS_SENDER) startSender(); else startReceiver();
}

void loop() {
  static uint32_t tPrint = millis();
  static bool lockoutReported = false;
  uint32_t now = millis();

  if (!IS_SENDER) {
    handleSerial();
    if (gFaultReport) {
      gFaultReport = false;
      Serial.printf("!!! nFAULT: IC_STAT=0x%02X STATUS1=0x%02X STATUS2=0x%02X (braked, will try CLR_FLT)\n",
                    gFaultRegs[0], gFaultRegs[1], gFaultRegs[2]);
    }
    if (gFaultLockout && !lockoutReported) {
      lockoutReported = true;
      Serial.println("!!! DRV8316 fault did not clear / repeated too often - LOCKED OUT (braked). Reset to retry.");
    }
    if (gEncLostReport) {
      gEncLostReport = false;
      Serial.println("!!! own encoder lost (CRC / status / implausible) - braked until it recovers");
    }
  }

  if (now - tPrint >= 250) {
    tPrint = now;
    if (!gPrintOn) {
      // quiet
    } else if (IS_SENDER) {
      Serial.printf("TX angle=%.3f rad  vel=%+7.2f rad/s  sent=%u  failed=%u  encErr=%u\n",
                    gTelAngle, gTelVel, (unsigned)gTxCount, (unsigned)gTxFail, (unsigned)gEncErr);
    } else {
      float upv = gTelUperVel;
      Serial.printf("RX link=%d target=%+8.3f pos=%+8.3f err=%+6.3f errMax=%5.3f vel=%+6.2f Uq=%+5.2f u/vel=",
                    gTelLink, gTelTarget, gTelPos, gTelErr, gTelErrMax, gTelVel, gTelUq);
      if (upv != 0.0f) Serial.printf("%.3f", upv); else Serial.print("-");
      Serial.printf(" frames=%u lost=%u rej=%u encErr=%u loopMax=%uus%s%s\n",
                    (unsigned)gRxCount, (unsigned)gRxLost, (unsigned)gRxRejected,
                    (unsigned)gEncErr, (unsigned)gLoopMaxUs,
                    gTelStall ? "  STALL" : "", gTelEncLost ? "  ENC-LOST" : "");
      gLoopMaxUs = 0;
      gTelErrMax = 0;
    }
    canSupervise(false);
  }
  delay(5);
}
