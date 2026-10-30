/*
  ================================================================
  BLDC "spring-return knob -> speed command" over CAN
  ESP32-C6 + DRV8316CR + MT6835, two identical boards on one CAN bus
  ================================================================
  Variant of bldc_can_follow.ino (v3). Same hardware, same pins.
  Arduino IDE: "esp32 by Espressif Systems" >= 3.0.0, board "ESP32C6 Dev Module".
  Motor: GB2806, 7 pole pairs.

  ONE sketch, TWO roles. Set IS_SENDER below, then flash each board.

  SENDER (IS_SENDER = true) - motorised spring-return knob ("jog wheel"):
     * Calibrates its own motor at start-up (the knob turns ~50 deg by
       itself - don't touch it), then takes its resting position as CENTRE.
     * Drives its motor as a virtual spring that pulls the knob back to
       centre: stiffness SPRING_K, damping SPRING_D. Past +-KNOB_RANGE a much
       stiffer "end stop" spring kicks in.
     * Displacement from centre -> speed command:
         direction = which way the knob is turned (CW / CCW)
         magnitude = grows with the displacement: 0 inside the dead band,
                     100 % at KNOB_RANGE, shaped by CMD_CURVE.
     * Sends the command on CAN at 1 kHz.

  RECEIVER (IS_SENDER = false) - speed follower:
     * Runs at  speed = command x VMAX  [rad/s], ramped with ACCEL.
     * The speed is integrated into a moving target position that is held by
       the same position -> velocity cascade as the follower sketch, so at
       zero command the motor HOLDS its position instead of slowly drifting.
     * Safety:
         - link lost, sender fault, sender encoder lost -> ramps to a stop
           (STOP_DECEL) and holds.
         - "arming": after start-up and after any interruption the knob must be
           seen at centre once before the motor will move again, so a reboot
           while the knob is held turned does not start the motor by surprise.
         - blocked motor: target cannot run ahead more than LEASH_RAD, and the
           voltage is reduced after STALL_US.

  CAN frame (standard ID 0x110, 8 bytes, 500 kbit/s, every 1 ms):
     byte 0..1  speed command, int16, -32767..+32767 = -100..+100 %  (LE)
     byte 2..3  knob displacement, int16, milliradians                (LE)
     byte 4     flags: bit0 = command valid, bit1 = sender DRV fault,
                       bit2 = sender encoder lost, bit3 = knob over-travel
     byte 5     sequence counter
     byte 6..7  reserved
  Different ID from the follower sketch (0x100), so a board still running the
  other variant is ignored rather than misinterpreted.

  Live tuning over Serial (115200 baud, newline) on BOTH boards - type "help".
  Changes are not saved; copy values you like into the *_DEFAULT constants.

  Hardware needed: a CAN transceiver on each board (GPIO4 = TX, GPIO5 = RX),
  CANH/CANL wired between boards, common GND, 120 ohm termination at each END of
  the bus (so 2 x 120 ohm for two nodes).
  ================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include "driver/mcpwm_prelude.h"
#include "driver/twai.h"
#include "esp_timer.h"

// ---------------------------------------------------------------
// ROLE  <<<<<<<<<<<<<<  set per board
// ---------------------------------------------------------------
constexpr bool IS_SENDER = false;     // true = knob board, false = driven motor board

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
// Motor / power (both boards)
constexpr int      POLE_PAIRS        = 7;
constexpr float    VBUS              = 24.0f;   // supply voltage [V]  <-- set to your real supply
constexpr float    ALIGN_VOLTAGE     = 2.0f;    // first calibration voltage [V]
constexpr float    ALIGN_VOLTAGE_MAX = 5.0f;    // calibration retries x1.5 up to this [V]

// ---- SENDER: knob feel and command mapping ----
// (*_DEFAULT values can be changed live over Serial)
constexpr float    SPRING_K_DEFAULT   = 3.0f;   // centring spring [V per rad]   (more = stiffer knob)
constexpr float    SPRING_D_DEFAULT   = 0.03f;  // damping [V per rad/s]         (more = less wobble on release)
constexpr float    SPRING_UQ_MAX      = 2.0f;   // max spring voltage inside the range [V]
constexpr float    END_K              = 15.0f;  // end-stop stiffness beyond the range [V per rad]
constexpr float    KNOB_UQ_LIMIT      = 4.0f;   // total voltage limit on the knob motor [V]
constexpr float    KNOB_RANGE_DEFAULT = 1.0f;   // displacement for 100 % command [rad] (~57 deg)
constexpr float    SPRING_U0_DEFAULT  = 0.8f;   // constant centring push [V] - beats cogging/friction so the
                                                //   knob really returns to centre when released
constexpr float    SPRING_U0_ZONE     = 0.015f; // push fades out within +-this of centre [rad] (~0.9 deg, no buzz)
constexpr float    DEADBAND_DEFAULT   = 0.08f;  // no command within +-this of centre [rad] (~4.6 deg)
constexpr float    DEADBAND_HYST      = 0.03f;  // command restarts only beyond deadband + this [rad]
constexpr float    CMD_CURVE_DEFAULT  = 1.5f;   // 1 = linear, >1 = finer speed control near centre
constexpr float    KNOB_RUNAWAY_RAD   = 3.0f;   // |displacement| beyond this -> brake, command invalid
constexpr float    KNOB_VEL_ALPHA     = 0.1f;   // knob speed filter, per 200 us
constexpr int      CMD_SIGN           = +1;     // -1 reverses which knob direction means which motor direction

// ---- RECEIVER: speed control ----
constexpr float    VMAX_DEFAULT      = 1000.0f;   // motor speed at 100 % command [rad/s]
constexpr float    ACCEL_DEFAULT     = 250.0f;  // acceleration / deceleration limit [rad/s^2]
constexpr float    STOP_DECEL        = 200.0f;  // deceleration when link / sender is lost [rad/s^2]
constexpr float    UQ_LIMIT_DEFAULT  = 5.0f;    // max phase voltage [V] - watch motor temperature
constexpr float    KP_DEFAULT        = 0.12f;   // velocity loop P  [V per rad/s]
constexpr float    KI_DEFAULT        = 3.0f;    // velocity loop I  [V per rad/s per s]
constexpr float    KP_POS_DEFAULT    = 30.0f;   // position hold P  [rad/s per rad]
constexpr float    KFF_DEFAULT       = 0.0f;    // back-EMF feed-forward [V per rad/s] - set from "u/vel"
constexpr float    VEL_ALPHA         = 0.2f;    // receiver speed filter, per 200 us (~1 ms)
constexpr float    LEASH_RAD         = 0.5f;    // target may run ahead of a blocked motor by at most this [rad]
constexpr int64_t  RX_TIMEOUT_US     = 50000;   // no frame for this long -> stop

// Stall protection (receiver)
constexpr float    STALL_ERR_RAD     = 0.3f;    // error above this ...
constexpr float    STALL_VEL         = 0.5f;    // ... while slower than this [rad/s] ...
constexpr int64_t  STALL_US          = 500000;  // ... for this long = stalled
constexpr float    STALL_UQ_LIMIT    = 1.0f;    // voltage limit while stalled [V]

// Encoder integrity (both boards)
constexpr bool     ENC_CHECK_CRC     = true;    // set false only if the start-up CRC test always fails
constexpr float    OWN_MAX_SPEED     = 300.0f;  // plausibility limit for own encoder steps [rad/s]
constexpr int      ENC_BAD_LIMIT     = 10;      // bad reads in a row (2 ms) -> brake
constexpr int      ENC_RECOVER_GOOD  = 50;      // good reads in a row (10 ms) -> resume

// DRV8316 fault handling (both boards)
constexpr int64_t  FAULT_RETRY_US    = 500000;  // CLR_FLT interval / required fault-free time
constexpr int      FAULT_MAX_CLEARS  = 10;      // CLR_FLT attempts per fault event before lockout
constexpr int      FAULT_MAX_EVENTS  = 5;       // fault events within the window before lockout
constexpr int64_t  FAULT_WINDOW_US   = 10000000;

// CAN / timing
constexpr uint32_t CAN_ID_CMD        = 0x110;
constexpr uint32_t CTRL_PERIOD_US    = 200;     // control loop on both boards: 5 kHz
constexpr int      SEND_EVERY        = 5;       // sender: one CAN frame every 5 cycles = 1 kHz

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
constexpr float    NOMINAL_DT     = CTRL_PERIOD_US * 1e-6f;

// MT6835 status bits (STATUS[2:0]) - verify against your datasheet revision
constexpr uint8_t  ENC_ST_OVERSPEED = 0x01;
constexpr uint8_t  ENC_ST_WEAK_MAG  = 0x02;
constexpr uint8_t  ENC_ST_UV        = 0x04;
constexpr uint8_t  ENC_ST_BAD       = ENC_ST_WEAK_MAG | ENC_ST_UV;   // angle not trustworthy

// CAN flag bits
constexpr uint8_t  FLAG_VALID       = 0x01;
constexpr uint8_t  FLAG_DRV_FAULT   = 0x02;
constexpr uint8_t  FLAG_ENC_LOST    = 0x04;
constexpr uint8_t  FLAG_OVERTRAVEL  = 0x08;

static mcpwm_timer_handle_t gTimer;
static mcpwm_oper_handle_t  gOper[3];
static mcpwm_cmpr_handle_t  gCmpr[3];
static mcpwm_gen_handle_t   gGenH[3];
static mcpwm_gen_handle_t   gGenL[3];

static TaskHandle_t gCtrlTask = nullptr;
static float   gSensorDir = 1.0f;
static float   gZeroElec  = 0.0f;
static uint8_t gCtrl2     = 0;               // DRV8316 CTRL2 value (without CLR_FLT)

// Latest command from CAN (written by canRxTask, read by controlTask)
struct RxSnap {
  float    cmd;         // -1..+1, already 0 if the sender marked it invalid
  float    disp;        // sender knob displacement [rad] (telemetry)
  uint8_t  flags;
  int64_t  tUs;         // local arrival time
  bool     valid;
};
// NOTE: all structs used as function parameters must be defined here, above the
// first function - the Arduino IDE inserts auto-generated prototypes there.
struct FaultSup {                 // DRV8316 nFAULT supervisor state
  int cnt = 0, clears = 0, events = 0;
  bool faulted = false, lockout = false;
  int64_t highSince = 0, lastClr = 0, winStart = 0;
};
struct EncHealth { int bad = 0, good = 0; bool lost = false; };   // own-encoder health
struct Param { const char *name; volatile float *v; float lo, hi; const char *help; };   // Serial tuning

static RxSnap       gRx = {0.0f, 0.0f, 0, 0, false};
static portMUX_TYPE gRxMux = portMUX_INITIALIZER_UNLOCKED;

// Telemetry for loop()
static volatile float    gTelDisp = 0, gTelCmd = 0, gTelVcmd = 0, gTelVel = 0, gTelUq = 0, gTelErr = 0;
static volatile float    gTelUperVel = 0;
static volatile bool     gTelLink = false, gTelArmed = false, gTelValid = false;
static volatile bool     gTelStall = false, gTelEncLost = false;
static volatile uint8_t  gTelFlags = 0;
static volatile uint32_t gRxCount = 0, gRxLost = 0, gTxCount = 0, gTxFail = 0;
static volatile uint32_t gEncErr = 0, gLoopMaxUs = 0;
static volatile bool     gFaultActive = false, gFaultReport = false, gFaultLockout = false;
static volatile bool     gEncLostReport = false, gOverTravelReport = false;
static volatile uint8_t  gFaultRegs[3] = {0, 0, 0};
static volatile bool     gPrintOn = true;
static volatile bool     gReZero = false;

// Live-tunable parameters (written by loop(), read by the control task)
static volatile float gSpringK = SPRING_K_DEFAULT, gSpringD = SPRING_D_DEFAULT, gSpringU0 = SPRING_U0_DEFAULT;
static volatile float gKnobRange = KNOB_RANGE_DEFAULT, gDeadband = DEADBAND_DEFAULT, gCurve = CMD_CURVE_DEFAULT;
static volatile float gVMax = VMAX_DEFAULT, gAccel = ACCEL_DEFAULT;
static volatile float gKp = KP_DEFAULT, gKi = KI_DEFAULT, gKpPos = KP_POS_DEFAULT;
static volatile float gKff = KFF_DEFAULT, gUqLimit = UQ_LIMIT_DEFAULT;

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
// CRC-8, polynomial 0x07, init 0x00, over the three angle/status bytes.
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

// u > 0 always means "push the encoder angle up", for either sensor direction
static inline void applyTorqueVoltage(float u, uint32_t raw) {
  float thetaE = wrap2Pi(gSensorDir * POLE_PAIRS * (raw * RAD_PER_COUNT) - gZeroElec);
  setPhaseVoltages(0.0f, gSensorDir * u, thetaE);
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
// Shared supervision: DRV8316 nFAULT and own-encoder health
// ---------------------------------------------------------------
// Call once per control cycle. Returns true while the bridge must stay off
// (braked). *recovered is set on the single cycle where the fault has cleared;
// the caller then resets its state and re-enables the outputs.
static bool faultStep(FaultSup &f, int64_t now, bool *recovered) {
  *recovered = false;
  bool nf = (digitalRead(PIN_NFAULT) == LOW);
  f.cnt = nf ? f.cnt + 1 : 0;

  if (!f.faulted && f.cnt >= 3) {                  // new fault event
    f.faulted = true;
    setBrake(true);
    gFaultRegs[0] = drvReadReg(0x00);
    gFaultRegs[1] = drvReadReg(0x01);
    gFaultRegs[2] = drvReadReg(0x02);
    gFaultActive = true;
    gFaultReport = true;
    if (f.winStart == 0 || now - f.winStart > FAULT_WINDOW_US) { f.winStart = now; f.events = 0; }
    if (++f.events > FAULT_MAX_EVENTS) f.lockout = true;
    f.clears = 0;
    f.lastClr = now;
    f.highSince = 0;
  }
  if (!f.faulted) return false;
  if (f.lockout) { gFaultLockout = true; return true; }   // braked until reset

  if (nf) {
    f.highSince = 0;
    if (now - f.lastClr > FAULT_RETRY_US) {        // latched fault: try CLR_FLT
      if (++f.clears > FAULT_MAX_CLEARS) { f.lockout = true; gFaultLockout = true; return true; }
      drvClearFault();
      f.lastClr = now;
    }
  } else {
    if (f.highSince == 0) f.highSince = now;
    if (now - f.highSince > FAULT_RETRY_US) {      // fault-free long enough
      f.faulted = false;
      gFaultActive = false;
      *recovered = true;
      return false;
    }
  }
  return true;
}

// Checked encoder sample. Returns true with *raw / *dC valid; false = skip
// this cycle. Too many bad reads in a row -> brake ("lost") until a run of
// good reads. *resumed is set on the first sample after an outage (dC = 0;
// the movement during the outage is unknown, the caller resets its state).
static bool encStep(EncHealth &h, uint32_t &prevRaw, float dt, uint32_t *raw, int32_t *dC, bool *resumed) {
  *resumed = false;
  *dC = 0;
  uint8_t st = 0;
  bool ok = encRead(raw, &st) && !(st & ENC_ST_BAD);
  if (ok && !h.lost) {
    *dC = deltaCounts(*raw, prevRaw);
    float maxC = OWN_MAX_SPEED * dt / RAD_PER_COUNT + 2000.0f;
    if (fabsf((float)*dC) > maxC) ok = false;      // implausible jump
  }
  if (!ok) {
    gEncErr++;
    h.good = 0;
    if (!h.lost && ++h.bad >= ENC_BAD_LIMIT) {
      h.lost = true;
      setBrake(true);
      gEncLostReport = true;
      gTelEncLost = true;
    }
    return false;
  }
  h.bad = 0;
  if (h.lost) {
    if (++h.good < ENC_RECOVER_GOOD) return false;
    h.lost = false;
    h.good = 0;
    gTelEncLost = false;
    *dC = 0;
    *resumed = true;
    setBrake(false);
  }
  prevRaw = *raw;
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

// Called periodically from loop(): recover from bus-off
static void canSupervise() {
  twai_status_info_t si;
  if (twai_get_status_info(&si) != ESP_OK) return;
  if (si.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
  else if (si.state == TWAI_STATE_STOPPED) twai_start();
}

// ---------------------------------------------------------------
// SENDER: knob displacement -> command (-1..+1)
// ---------------------------------------------------------------
// Dead band with hysteresis: once the command is zero, the knob has to go past
// deadband + DEADBAND_HYST to start again, so a knob that settles right at the
// edge of the dead band cannot keep flickering a small command on and off.
static float knobToCommand(float disp) {
  static bool active = false;
  float db = gDeadband, range = gKnobRange;
  float a = fabsf(disp);
  if (active) { if (a <= db) active = false; }
  else        { if (a > db + DEADBAND_HYST) active = true; }
  if (!active || range <= db) return 0.0f;
  float n = (a - db) / (range - db);              // 0..1 between dead band and range
  if (n > 1.0f) n = 1.0f;
  n = powf(n, gCurve);
  return (disp > 0.0f ? 1.0f : -1.0f) * (float)CMD_SIGN * n;
}

// Spring torque voltage (u > 0 pushes the encoder angle up)
static float knobSpring(float disp, float vel) {
  float range = gKnobRange;
  float u = -gSpringK * disp;
  // constant centring push (fades to 0 right at centre to avoid buzzing):
  // the linear spring alone is too weak near centre to beat cogging + friction
  u -= gSpringU0 * constrain(disp / SPRING_U0_ZONE, -1.0f, 1.0f);
  u = constrain(u, -SPRING_UQ_MAX, SPRING_UQ_MAX);
  float over = fabsf(disp) - range;
  if (over > 0.0f) u -= (disp > 0.0f ? 1.0f : -1.0f) * END_K * over;   // end stop
  u -= gSpringD * vel;                                                  // damping
  return constrain(u, -KNOB_UQ_LIMIT, KNOB_UQ_LIMIT);
}

// ---------------------------------------------------------------
// SENDER: 5 kHz task - spring-return knob, CAN command at 1 kHz
// Displacement is kept relative to the centre's ABSOLUTE encoder angle, so it
// stays correct even if the knob is moved while the motor was off (fault /
// encoder outage).
// ---------------------------------------------------------------
static void senderTask(void *) {
  uint32_t prevRaw = encReadRetry(nullptr);
  uint32_t centerRaw = prevRaw;                    // start-up position = centre
  int64_t dispC = 0;                               // unwrapped displacement from centre [counts]
  float vel = 0.0f, disp = 0.0f, uKnob = 0.0f;
  bool overTravel = false;
  int64_t tPrev = esp_timer_get_time();
  uint8_t seq = 0;
  int div = 0;
  FaultSup fs;
  EncHealth eh;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t now = esp_timer_get_time();

    // ---------- knob motor ----------
    bool recovered = false;
    bool faulted = faultStep(fs, now, &recovered);
    if (recovered) {
      prevRaw = encReadRetry(nullptr);
      dispC = deltaCounts(prevRaw, centerRaw);     // knob may have moved while off
      tPrev = now;
      vel = 0.0f;
      overTravel = false;                          // re-evaluated next cycle
      if (!eh.lost) setBrake(false);
    } else if (!faulted) {
      float dt = (now - tPrev) * 1e-6f;            // time since last GOOD sample
      uint32_t raw = 0;
      int32_t dC = 0;
      bool resumed = false;
      if (dt > 0.0f && encStep(eh, prevRaw, dt, &raw, &dC, &resumed)) {
        tPrev = now;
        if (resumed) {
          dispC = deltaCounts(raw, centerRaw);
          vel = 0.0f;
          overTravel = false;
        } else {
          dispC += dC;
          vel += KNOB_VEL_ALPHA * ((dC * RAD_PER_COUNT) / dt - vel);
        }
        if (gReZero) { gReZero = false; centerRaw = raw; dispC = 0; }

        disp = (float)dispC * RAD_PER_COUNT;

        // over-travel: forced far past the end stop, or spring pushing the wrong way
        bool ot = fabsf(disp) > KNOB_RUNAWAY_RAD;
        if (ot != overTravel) {
          overTravel = ot;
          setBrake(ot);
          if (ot) gOverTravelReport = true;
        }

        if (!overTravel) {
          uKnob = knobSpring(disp, vel);
          applyTorqueVoltage(uKnob, raw);
        } else {
          uKnob = 0.0f;
        }

        uint32_t loopUs = (uint32_t)(esp_timer_get_time() - now);
        if (loopUs > gLoopMaxUs) gLoopMaxUs = loopUs;
      }
    }

    // ---------- CAN command, 1 kHz ----------
    if (++div < SEND_EVERY) continue;
    div = 0;

    uint8_t flags = 0;
    if (fs.faulted)  flags |= FLAG_DRV_FAULT;
    if (eh.lost)     flags |= FLAG_ENC_LOST;
    if (overTravel)  flags |= FLAG_OVERTRAVEL;
    bool valid = (flags == 0);
    if (valid) flags |= FLAG_VALID;

    float cmd = valid ? knobToCommand(disp) : 0.0f;
    int16_t c16 = (int16_t)lrintf(constrain(cmd, -1.0f, 1.0f) * 32767.0f);
    int16_t d16 = (int16_t)constrain(lrintf(disp * 1000.0f), -32767L, 32767L);

    twai_message_t m = {};
    m.identifier = CAN_ID_CMD;
    m.data_length_code = 8;
    m.ss = 1;                        // single shot: never queue stale commands if nobody ACKs
    m.data[0] = (uint16_t)c16 & 0xFF;
    m.data[1] = ((uint16_t)c16 >> 8) & 0xFF;
    m.data[2] = (uint16_t)d16 & 0xFF;
    m.data[3] = ((uint16_t)d16 >> 8) & 0xFF;
    m.data[4] = flags;
    m.data[5] = seq++;
    m.data[6] = 0;
    m.data[7] = 0;
    if (twai_transmit(&m, 0) == ESP_OK) gTxCount++; else gTxFail++;

    gTelDisp = disp; gTelCmd = cmd; gTelVel = vel; gTelUq = uKnob;
    gTelValid = valid; gTelFlags = flags;
  }
}

// ---------------------------------------------------------------
// RECEIVER: CAN rx task
// ---------------------------------------------------------------
static void canRxTask(void *) {
  bool first = true;
  uint8_t lastSeq = 0;

  for (;;) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(50)) != ESP_OK) continue;
    if (m.extd || m.rtr || m.identifier != CAN_ID_CMD || m.data_length_code != 8) continue;

    int16_t c16  = (int16_t)((uint16_t)m.data[0] | ((uint16_t)m.data[1] << 8));
    int16_t d16  = (int16_t)((uint16_t)m.data[2] | ((uint16_t)m.data[3] << 8));
    uint8_t flags = m.data[4];
    uint8_t seq   = m.data[5];

    if (!first) {
      uint8_t gap = (uint8_t)(seq - lastSeq);
      if (gap == 0) continue;                      // duplicate
      if (gap > 1) gRxLost += gap - 1;
    }
    first = false;
    lastSeq = seq;

    float cmd = (flags & FLAG_VALID) ? constrain(c16 / 32767.0f, -1.0f, 1.0f) : 0.0f;

    portENTER_CRITICAL(&gRxMux);
    gRx.cmd   = cmd;
    gRx.disp  = d16 * 0.001f;
    gRx.flags = flags;
    gRx.tUs   = esp_timer_get_time();
    gRx.valid = true;
    portEXIT_CRITICAL(&gRxMux);
    gRxCount++;
  }
}

// ---------------------------------------------------------------
// RECEIVER: control task (5 kHz)
//   command -> ramped speed vcmd -> integrated into a moving target position
//   -> position P (+ vcmd feed-forward) -> velocity PI (+ back-EMF FF) -> Uq
// ---------------------------------------------------------------
static void controlTask(void *) {
  uint32_t prevRaw = encReadRetry(nullptr);
  int64_t posCounts = 0, targetCounts = 0;
  float tgtFrac = 0.0f;                            // sub-count part of the target
  float vcmd = 0.0f, vel = 0.0f, integ = 0.0f;
  bool armed = false;
  int64_t tPrev = esp_timer_get_time();
  int64_t stallSince = 0;
  const int64_t LEASH_C = (int64_t)(LEASH_RAD / RAD_PER_COUNT);
  FaultSup fs;
  EncHealth eh;

  auto resetMotion = [&]() {                       // stop, hold where we are, re-arm needed
    targetCounts = posCounts;
    tgtFrac = 0.0f;
    vcmd = 0.0f; vel = 0.0f; integ = 0.0f;
    armed = false;
    stallSince = 0;
  };

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t now = esp_timer_get_time();

    // ---------- DRV8316 fault ----------
    bool recovered = false;
    if (faultStep(fs, now, &recovered)) { gTelArmed = false; continue; }
    if (recovered) {
      prevRaw = encReadRetry(nullptr);
      tPrev = now;
      resetMotion();
      if (!eh.lost) setBrake(false);
      continue;
    }

    // ---------- own encoder ----------
    float dt = (now - tPrev) * 1e-6f;              // time since last GOOD sample
    if (dt <= 0.0f) continue;
    uint32_t raw = 0;
    int32_t dC = 0;
    bool resumed = false;
    if (!encStep(eh, prevRaw, dt, &raw, &dC, &resumed)) {
      if (eh.lost) { integ = 0.0f; gTelArmed = false; }
      continue;                                    // keep last duties for this cycle
    }
    if (resumed) { dt = NOMINAL_DT; resetMotion(); }
    tPrev = now;
    posCounts += dC;
    vel += VEL_ALPHA * ((dC * RAD_PER_COUNT) / dt - vel);

    // ---------- command from CAN ----------
    RxSnap s;
    portENTER_CRITICAL(&gRxMux);
    s = gRx;
    portEXIT_CRITICAL(&gRxMux);

    bool fresh    = s.valid && (now - s.tUs) < RX_TIMEOUT_US;
    bool senderOk = fresh && (s.flags & FLAG_VALID);
    if (!senderOk)                   armed = false;
    else if (!armed && s.cmd == 0.0f) armed = true;   // knob seen at centre -> arm

    float vTarget = armed ? s.cmd * gVMax : 0.0f;
    float rate    = senderOk ? gAccel : fmaxf(gAccel, STOP_DECEL);
    float step    = rate * dt;
    vcmd += constrain(vTarget - vcmd, -step, step);

    // ---------- moving target ----------
    tgtFrac += vcmd * dt / RAD_PER_COUNT;
    int32_t whole = (int32_t)tgtFrac;
    targetCounts += whole;
    tgtFrac -= (float)whole;

    // leash: the target never runs away from a blocked / slow motor
    int64_t errC = targetCounts - posCounts;
    if (errC > LEASH_C)       { targetCounts = posCounts + LEASH_C; tgtFrac = 0.0f; errC = LEASH_C; }
    else if (errC < -LEASH_C) { targetCounts = posCounts - LEASH_C; tgtFrac = 0.0f; errC = -LEASH_C; }
    float errRad = (float)errC * RAD_PER_COUNT;

    // ---------- stall protection ----------
    if (fabsf(errRad) > STALL_ERR_RAD && fabsf(vel) < STALL_VEL) {
      if (stallSince == 0) stallSince = now;
    } else {
      stallSince = 0;
    }
    bool stalled = stallSince != 0 && (now - stallSince > STALL_US);
    const float uqMax = gUqLimit;
    float uLim = stalled ? fminf(STALL_UQ_LIMIT, uqMax) : uqMax;

    // ---------- cascade ----------
    const float vLim = 1.2f * gVMax + 5.0f;        // headroom for position correction
    float velSp = constrain(gKpPos * errRad + vcmd, -vLim, vLim);
    float verr  = velSp - vel;
    float uff   = gKff * velSp;
    integ += gKi * verr * dt;
    integ = constrain(integ, -uLim, uLim);
    float u = constrain(uff + gKp * verr + integ, -uLim, uLim);

    applyTorqueVoltage(u, raw);

    // ---------- telemetry ----------
    gTelCmd = s.cmd; gTelDisp = s.disp; gTelFlags = s.flags;
    gTelVcmd = vcmd; gTelVel = vel; gTelErr = errRad; gTelUq = u;
    gTelLink = fresh; gTelArmed = armed; gTelStall = stalled;
    gTelUperVel = (fabsf(vel) > 3.0f) ? u / vel : 0.0f;

    uint32_t loopUs = (uint32_t)(esp_timer_get_time() - now);
    if (loopUs > gLoopMaxUs) gLoopMaxUs = loopUs;
  }
}

// ---------------------------------------------------------------
// Live tuning over Serial (both roles). One command per line, e.g. "k 4"
// ---------------------------------------------------------------
static const Param kSenderParams[] = {
  {"k",     &gSpringK,   0.0f, 10.0f, "centring spring [V/rad]        (more = stiffer knob)"},
  {"u0",    &gSpringU0,  0.0f, 2.0f,  "centring push [V]              (raise if knob stops off centre)"},
  {"d",     &gSpringD,   0.0f, 0.5f,  "knob damping [V per rad/s]     (more = less wobble on release)"},
  {"range", &gKnobRange, 0.2f, 2.5f,  "displacement for 100 % [rad]   (less = shorter throw)"},
  {"db",    &gDeadband,  0.0f, 0.5f,  "dead band around centre [rad]"},
  {"curve", &gCurve,     0.5f, 4.0f,  "1 = linear, >1 = finer control near centre"},
};
static const Param kReceiverParams[] = {
  {"vmax",  &gVMax,      1.0f, 150.0f, "speed at 100 % command [rad/s]"},
  {"acc",   &gAccel,     5.0f, 5000.0f,"acceleration limit [rad/s^2]  (more = snappier)"},
  {"vkp",   &gKp,        0.0f, 2.0f,   "velocity loop P [V per rad/s]"},
  {"vki",   &gKi,        0.0f, 100.0f, "velocity loop I [V per rad/s/s]"},
  {"pkp",   &gKpPos,     0.0f, 200.0f, "position hold P [rad/s per rad]"},
  {"kff",   &gKff,       0.0f, 1.0f,   "back-EMF feed-forward (~0.8 x u/vel while spinning)"},
  {"uq",    &gUqLimit,   0.5f, 0.55f * VBUS, "max phase voltage [V] (watch motor temperature)"},
};

static const Param *params(int *n) {
  if (IS_SENDER) { *n = sizeof(kSenderParams) / sizeof(Param);   return kSenderParams; }
  *n = sizeof(kReceiverParams) / sizeof(Param);
  return kReceiverParams;
}

static void printGains() {
  int n;
  const Param *p = params(&n);
  Serial.print("PARAMS ");
  for (int i = 0; i < n; i++) Serial.printf(" %s=%.3g", p[i].name, *p[i].v);
  Serial.println();
}

static void printHelp() {
  int n;
  const Param *p = params(&n);
  Serial.println("Commands, one per line:  <name> <value>");
  for (int i = 0; i < n; i++) Serial.printf("  %-6s %s\n", p[i].name, p[i].help);
  if (IS_SENDER) Serial.println("  zero   take the current knob position as the new centre");
  Serial.println("  log 0|1  periodic status printout off/on");
  Serial.println("  show     print current values");
}

static void handleSerial() {
  static char buf[48];
  static int len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (len < (int)sizeof(buf) - 1) buf[len++] = c; continue; }
    buf[len] = 0;
    len = 0;

    char *cmd = strtok(buf, " \t");
    if (!cmd) continue;
    char *arg = strtok(nullptr, " \t");
    char *end = nullptr;
    float v = arg ? strtof(arg, &end) : NAN;
    bool hasVal = arg && end != arg && isfinite(v);

    if (!strcmp(cmd, "help")) { printHelp();  continue; }
    if (!strcmp(cmd, "show")) { printGains(); continue; }
    if (!strcmp(cmd, "log") && hasVal) { gPrintOn = (v != 0.0f); continue; }
    if (IS_SENDER && !strcmp(cmd, "zero")) { gReZero = true; Serial.println("centre set to current position"); continue; }

    int n;
    const Param *p = params(&n);
    int i = 0;
    while (i < n && strcmp(cmd, p[i].name)) i++;
    if (i == n)  { Serial.println("? unknown command (type \"help\")"); continue; }
    if (!hasVal) { Serial.println("? expected: <name> <number>");      continue; }
    *p[i].v = constrain(v, p[i].lo, p[i].hi);
    printGains();
  }
}

// ---------------------------------------------------------------
// Start-up
// ---------------------------------------------------------------
static void startTimerFor(esp_timer_cb_t cb, uint32_t periodUs, const char *name) {
  esp_timer_create_args_t ta = {};
  ta.callback = cb;
  ta.name = name;
  esp_timer_handle_t h;
  ESP_ERROR_CHECK(esp_timer_create(&ta, &h));
  ESP_ERROR_CHECK(esp_timer_start_periodic(h, periodUs));
}

// PWM + DRV8316 + sensor calibration; halts (braked) on failure
static void motorBringUp() {
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
}

static void startSender() {
  Serial.println("ROLE: SENDER (spring-return knob). Don't touch the knob until calibration is done.");
  motorBringUp();
  if (!canInit()) { setBrake(true); while (true) delay(1000); }
  xTaskCreate(senderTask, "knob", 4096, nullptr, configMAX_PRIORITIES - 2, &gCtrlTask);
  startTimerFor([](void *) { xTaskNotifyGive(gCtrlTask); }, CTRL_PERIOD_US, "knob_tick");
  Serial.println("Knob active - current position is centre. Sending speed commands.");
}

static void startReceiver() {
  Serial.println("ROLE: RECEIVER (motor speed set by the knob on CAN)");
  motorBringUp();
  if (!canInit()) { setBrake(true); while (true) delay(1000); }
  xTaskCreate(canRxTask, "can_rx", 3072, nullptr, configMAX_PRIORITIES - 3, nullptr);
  xTaskCreate(controlTask, "ctrl", 4096, nullptr, configMAX_PRIORITIES - 2, &gCtrlTask);
  startTimerFor([](void *) { xTaskNotifyGive(gCtrlTask); }, CTRL_PERIOD_US, "ctrl_tick");
  Serial.println("Running - holding position; will move once the knob has been seen at centre.");
}

// Encoder self-test (CRC + status) at start-up
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
                   "           or set ENC_CHECK_CRC = false. The motor will stay braked otherwise.");
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
  Serial.println("\nBLDC CAN knob -> speed - ESP32-C6 / DRV8316 / MT6835");

  pinMode(PIN_DRV_CS, OUTPUT);  digitalWrite(PIN_DRV_CS, HIGH);
  pinMode(PIN_ENC_CS, OUTPUT);  digitalWrite(PIN_ENC_CS, HIGH);
  pinMode(PIN_NFAULT, INPUT_PULLUP);          // GPIO15 is a strapping pin on the C6
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

  encSelfTest();

  if (IS_SENDER) startSender(); else startReceiver();
  printGains();
  Serial.println("Type \"help\" for live tuning commands.");
}

void loop() {
  static uint32_t tPrint = millis();
  static bool lockoutReported = false;
  uint32_t now = millis();

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
  if (gOverTravelReport) {
    gOverTravelReport = false;
    Serial.println("!!! knob over-travel (forced past the end stop, or spring pushing the wrong way) - braked");
  }

  if (now - tPrint >= 250) {
    tPrint = now;
    if (gPrintOn) {
      if (IS_SENDER) {
        Serial.printf("KNOB disp=%+6.3f rad (%+6.1f deg) cmd=%+6.1f%% vel=%+6.2f Uq=%+5.2f valid=%d "
                      "sent=%u failed=%u encErr=%u loopMax=%uus\n",
                      gTelDisp, gTelDisp * 57.2958f, gTelCmd * 100.0f, gTelVel, gTelUq, gTelValid,
                      (unsigned)gTxCount, (unsigned)gTxFail, (unsigned)gEncErr, (unsigned)gLoopMaxUs);
      } else {
        float upv = gTelUperVel;
        uint8_t fl = gTelFlags;
        Serial.printf("RX link=%d armed=%d cmd=%+6.1f%% vcmd=%+6.2f vel=%+6.2f err=%+6.3f Uq=%+5.2f u/vel=",
                      gTelLink, gTelArmed, gTelCmd * 100.0f, gTelVcmd, gTelVel, gTelErr, gTelUq);
        if (upv != 0.0f) Serial.printf("%.3f", upv); else Serial.print("-");
        Serial.printf(" frames=%u lost=%u encErr=%u loopMax=%uus%s%s%s%s\n",
                      (unsigned)gRxCount, (unsigned)gRxLost, (unsigned)gEncErr, (unsigned)gLoopMaxUs,
                      gTelStall ? "  STALL" : "", gTelEncLost ? "  ENC-LOST" : "",
                      (gTelLink && (fl & (FLAG_DRV_FAULT | FLAG_ENC_LOST | FLAG_OVERTRAVEL))) ? "  SENDER-FAULT" : "",
                      (gTelLink && !gTelArmed && gTelCmd != 0.0f) ? "  (return knob to centre to arm)" : "");
      }
      gLoopMaxUs = 0;
    }
    canSupervise();
  }
  delay(5);
}
