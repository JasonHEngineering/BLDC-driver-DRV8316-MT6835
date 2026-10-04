/*
  ================================================================
  BLDC forward / reverse demo
  ESP32-C6  +  DRV8316CR (6x PWM mode)  +  MT6835 (21-bit SPI encoder)
  ================================================================
  Arduino IDE: install "esp32 by Espressif Systems" >= 3.0.0, board "ESP32C6 Dev Module".
  Enable "USB CDC On Boot" if you use the native USB port for Serial.
  Motor: GB2806, 7 pole pairs.

  What it does
   1. Starts 3-phase complementary PWM on the MCPWM peripheral (25 kHz,
      center aligned, hardware dead time).
   2. Reads the MT6835 over SPI.
   3. Calibrates the sensor: finds electrical zero offset, sensor direction,
      and prints an estimate of the pole-pair count so you can sanity check it.
   4. Runs a velocity PI loop (5 kHz) that outputs a q-axis voltage
      (voltage-mode FOC without current loop, SVPWM-style modulation).
   5. Flips the speed target between +TARGET_SPEED and -TARGET_SPEED every
      REVERSE_PERIOD_MS, with a slew-rate limit (ACCEL) so the reversal is smooth.

  Current-sense pins (GPIO6 / GPIO18) are not used yet - this version is
  voltage mode. They are the natural next step for a real FOC current loop.

  ---- SAFETY / FIRST RUN ----
   * Use a current-limited bench supply (start ~1 A) and a free-running motor.
   * Start with a low VBUS, small UQ_LIMIT and ALIGN_VOLTAGE.
   * Set POLE_PAIRS and VBUS correctly for your hardware.
  ================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"

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

// ---------------------------------------------------------------
// User configuration  (TUNE THESE)
// ---------------------------------------------------------------
constexpr int      POLE_PAIRS        = 7;      // motor pole pairs (magnets / 2)
constexpr float    VBUS              = 12.0f;  // supply voltage [V]  <-- set to your real supply
constexpr float    UQ_LIMIT          = 3.0f;   // max phase voltage during run [V] (GB2806 is high-resistance: 3 V is only a few 100 mA)
constexpr float    ALIGN_VOLTAGE     = 2.0f;   // first voltage tried for sensor calibration [V]
constexpr float    ALIGN_VOLTAGE_MAX = 5.0f;   // calibration retries with x1.5 steps up to this [V]

constexpr float    TARGET_SPEED      = 20.0f;  // mechanical speed [rad/s]  (~190 rpm)
constexpr float    ACCEL             = 40.0f;  // setpoint slew [rad/s^2]
constexpr uint32_t REVERSE_PERIOD_MS = 4000;   // direction flips every N ms

constexpr float    KP                = 0.05f;  // [V per rad/s]
constexpr float    KI                = 0.50f;  // [V per rad/s per s]

constexpr bool     FLIP_FORWARD      = false;  // true = swap what "forward" means

// PWM / timing
constexpr uint32_t TIMER_RES_HZ      = 20000000;            // 20 MHz tick
constexpr uint32_t PWM_FREQ_HZ       = 25000;
constexpr uint32_t PERIOD_TICKS      = TIMER_RES_HZ / PWM_FREQ_HZ;  // 800
constexpr uint32_t PEAK_TICKS        = PERIOD_TICKS / 2;    // up/down counter peak
constexpr uint32_t DEADTIME_TICKS    = 10;                  // 10 / 20 MHz = 500 ns
constexpr uint32_t CTRL_PERIOD_US    = 200;                 // 5 kHz control loop

// ---------------------------------------------------------------
// Globals
// ---------------------------------------------------------------
constexpr float SQRT3_2 = 0.8660254f;

static mcpwm_timer_handle_t gTimer;
static mcpwm_oper_handle_t  gOper[3];
static mcpwm_cmpr_handle_t  gCmpr[3];
static mcpwm_gen_handle_t   gGenH[3];   // high side (INHx)
static mcpwm_gen_handle_t   gGenL[3];   // low side  (INLx)

static TaskHandle_t gCtrlTask = nullptr;

static float gSensorDir  = 1.0f;   // +1 / -1, found during calibration
static float gZeroElec   = 0.0f;   // electrical zero offset [rad]

// telemetry (written by control task, read by loop())
static volatile float    gTargetSpeed = 0.0f;
static volatile float    gSpeedSet    = 0.0f;
static volatile float    gSpeedMeas   = 0.0f;
static volatile float    gUqOut       = 0.0f;
static volatile float    gAngleMech   = 0.0f;
static volatile bool     gFaultActive = false;
static volatile bool     gFaultReport = false;
static volatile uint8_t  gFaultRegs[3] = {0, 0, 0};

// ---------------------------------------------------------------
// Math helpers
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

// ---------------------------------------------------------------
// MT6835 encoder
// ---------------------------------------------------------------
// Burst angle read: command 0xA003, then 3 data bytes:
//   [20:13] [12:5] [4:0 | status(3)]
static float encReadAngleStatus(uint8_t *status) {
  uint8_t tx[5] = {0xA0, 0x03, 0x00, 0x00, 0x00};
  uint8_t rx[5] = {0};

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ENC_CS, LOW);
  SPI.transferBytes(tx, rx, 5);
  digitalWrite(PIN_ENC_CS, HIGH);
  SPI.endTransaction();

  uint32_t raw = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | (rx[4] >> 3);
  if (status) *status = rx[4] & 0x07;
  return (float)raw * (TWO_PI / 2097152.0f);   // 2^21
}
static float encReadAngle() { return encReadAngleStatus(nullptr); }

// ---------------------------------------------------------------
// DRV8316 SPI (diagnostics only: read status registers)
// Frame assumed: [R/W | ADDR(6) | PARITY | DATA(8)], SPI mode 1, even parity.
// -> Verify against the DRV8316 datasheet SPI section for your variant.
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

static void drvInit() {
  Serial.println("DRV8316 before init:");
  drvPrintStatus();

  drvWriteReg(0x03, 0x03);          // CTRL1: unlock registers
  drvWriteReg(0x04, 0x60 | 0x01);   // CTRL2: keep defaults, PWM_MODE = 6x, CLR_FLT = 1
  delay(5);

  Serial.println("DRV8316 after unlock + CLR_FLT + 6x mode:");
  drvPrintStatus();
}

// ---------------------------------------------------------------
// MCPWM: 3 operators, each = complementary pair with hardware dead time
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
// Sensor calibration: electrical zero + direction (+ pole-pair estimate)
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
// Control task (5 kHz): encoder -> speed -> PI -> Uq -> PWM
// ---------------------------------------------------------------
static void controlTask(void *) {
  float prevAngle = encReadAngle();
  float vel = 0.0f, integ = 0.0f, setpoint = 0.0f;
  int64_t tPrev = esp_timer_get_time();

  int      faultCnt = 0;
  bool     faulted  = false;
  int64_t  faultClearT = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    int64_t now = esp_timer_get_time();
    float dt = (now - tPrev) * 1e-6f;
    tPrev = now;
    if (dt <= 0.0f) continue;

    // ---- nFAULT supervision ----
    if (digitalRead(PIN_NFAULT) == LOW) {
      faultClearT = 0;
      if (!faulted && ++faultCnt >= 3) {
        faulted = true;
        setBrake(true);
        integ = 0; setpoint = 0;
        gFaultRegs[0] = drvReadReg(0x00);       // IC_STAT
        gFaultRegs[1] = drvReadReg(0x01);       // STATUS1
        gFaultRegs[2] = drvReadReg(0x02);       // STATUS2
        gFaultActive = true;
        gFaultReport = true;
      }
    } else {
      faultCnt = 0;
      if (faulted) {
        if (faultClearT == 0) faultClearT = now;
        if (now - faultClearT > 500000) {        // fault line released for 500 ms
          faulted = false;
          gFaultActive = false;
          prevAngle = encReadAngle();
          vel = 0;
          setBrake(false);
        }
      }
    }
    if (faulted) continue;

    // ---- encoder & speed estimate ----
    float angle = encReadAngle();
    float dA = wrapPi(angle - prevAngle);
    prevAngle = angle;
    float vRaw = gSensorDir * dA / dt;           // speed in "motor frame"
    vel += 0.05f * (vRaw - vel);                 // 1st order LPF (~4 ms)

    // ---- setpoint ramp ----
    float target = gTargetSpeed;
    float step = ACCEL * dt;
    if      (target > setpoint + step) setpoint += step;
    else if (target < setpoint - step) setpoint -= step;
    else                               setpoint  = target;

    // ---- velocity PI ----
    float err = setpoint - vel;
    integ += KI * err * dt;
    integ = constrain(integ, -UQ_LIMIT, UQ_LIMIT);
    float uq = constrain(KP * err + integ, -UQ_LIMIT, UQ_LIMIT);

    // ---- commutation ----
    float thetaE = wrap2Pi(gSensorDir * POLE_PAIRS * angle - gZeroElec);
    setPhaseVoltages(0.0f, uq, thetaE);

    gSpeedSet = setpoint; gSpeedMeas = vel; gUqOut = uq; gAngleMech = angle;
  }
}

// ---------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nBLDC forward/reverse demo - ESP32-C6 / DRV8316 / MT6835");

  pinMode(PIN_DRV_CS, OUTPUT);  digitalWrite(PIN_DRV_CS, HIGH);
  pinMode(PIN_ENC_CS, OUTPUT);  digitalWrite(PIN_ENC_CS, HIGH);
  pinMode(PIN_NFAULT, INPUT_PULLUP);   // note: GPIO15 is a strapping pin on the C6
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

  pwmInit();                           // starts braked

  // Encoder sanity check
  uint8_t st = 0;
  float a = encReadAngleStatus(&st);
  Serial.printf("Encoder angle %.3f rad, status 0x%X (bit0=UV, bit1=weak mag, bit2=overspeed)\n", a, st);

  // DRV8316: clear flags, force 6x PWM mode, print decoded status
  drvInit();
  Serial.printf("nFAULT pin = %s\n", digitalRead(PIN_NFAULT) ? "OK (high)" : "ACTIVE (low)");

  bool calOk = false;
  if (digitalRead(PIN_NFAULT) == HIGH) {
    for (float u = ALIGN_VOLTAGE; u <= ALIGN_VOLTAGE_MAX + 0.01f && !calOk; u *= 1.5f)
      calOk = calibrateSensor(u);
  }
  if (!calOk) {
    setBrake(true);
    Serial.println("Calibration failed. Final DRV8316 status:");
    drvPrintStatus();
    Serial.println("Checklist: VM really present at the chip? DRVOFF pin low? nSLEEP high? motor phases");
    Serial.println("connected to OUTA/B/C? Scope INHx/INLx - do they show PWM during calibration?");
    Serial.println("Halting (braked).");
    while (true) delay(1000);
  }

  // Start control loop
  xTaskCreate(controlTask, "ctrl", 4096, nullptr, configMAX_PRIORITIES - 2, &gCtrlTask);

  esp_timer_create_args_t ta = {};
  ta.callback = [](void *) { xTaskNotifyGive(gCtrlTask); };
  ta.name = "ctrl_tick";
  esp_timer_handle_t h;
  ESP_ERROR_CHECK(esp_timer_create(&ta, &h));
  ESP_ERROR_CHECK(esp_timer_start_periodic(h, CTRL_PERIOD_US));

  gTargetSpeed = FLIP_FORWARD ? -TARGET_SPEED : TARGET_SPEED;
  Serial.println("Running.");
}

void loop() {
  static uint32_t tFlip = millis();
  static uint32_t tPrint = millis();

  uint32_t now = millis();

  if (now - tFlip >= REVERSE_PERIOD_MS) {
    tFlip = now;
    gTargetSpeed = -gTargetSpeed;
    Serial.printf(">>> %s\n", gTargetSpeed * (FLIP_FORWARD ? -1 : 1) > 0 ? "FORWARD" : "BACKWARD");
  }

  if (gFaultReport) {
    gFaultReport = false;
    Serial.printf("!!! nFAULT: IC_STAT=0x%02X STATUS1=0x%02X STATUS2=0x%02X (braked)\n",
                  gFaultRegs[0], gFaultRegs[1], gFaultRegs[2]);
  }

  if (now - tPrint >= 250) {
    tPrint = now;
    Serial.printf("set=%7.1f  meas=%7.1f rad/s  Uq=%5.2f V  angle=%.3f\n",
                  gSpeedSet, gSpeedMeas, gUqOut, gAngleMech);
  }
  delay(5);
}
