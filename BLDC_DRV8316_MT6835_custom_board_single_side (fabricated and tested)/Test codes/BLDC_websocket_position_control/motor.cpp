#include "motor.h"
#include "config.h"
#include "encoder.h"
#include <SPI.h>
#include <math.h>
#include "driver/gptimer.h"
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"

// ---------------------------------------------------------------
// Globals
// ---------------------------------------------------------------
constexpr float SQRT3_2      = 0.8660254f;
constexpr float RAD_PER_DEG  = PI / 180.0f;

static mcpwm_timer_handle_t gTimer;
static mcpwm_oper_handle_t  gOper[3];
static mcpwm_cmpr_handle_t  gCmpr[3];
static mcpwm_gen_handle_t   gGenH[3];
static mcpwm_gen_handle_t   gGenL[3];

static TaskHandle_t gCtrlTask = nullptr;
static float    gSensorDir  = 1.0f;
static float    gZeroElec   = 0.0f;
static uint8_t  gCtrl2      = 0;             // DRV8316 CTRL2 value (without CLR_FLT)
static uint32_t gZeroCounts = 0;             // raw encoder angle at kinematic zero

// Target from the websocket task. Value first, then flag: both are single words.
static volatile float gCmdDeg = 0.0f;
static volatile bool  gCmdPending = false;

// Telemetry for loop()
static volatile int32_t  gTelAngleCounts = 0;
static volatile float    gTelTargetDeg = 0;
static volatile float    gTelPos = 0, gTelRef = 0, gTelErr = 0, gTelVel = 0, gTelUq = 0;
static volatile bool     gTelEncOk = false, gTelMoving = false, gTelStall = false, gTelEncLost = false;
static volatile uint32_t gEncErr = 0, gLoopMaxUs = 0;
static volatile bool     gFaultActive = false, gFaultReport = false, gFaultLockout = false;
static volatile bool     gEncLostReport = false;
static volatile uint8_t  gFaultRegs[3] = {0, 0, 0};
static volatile float    gTelErrMax = 0, gTelUperVel = 0;
static volatile bool     gPrintOn = true;

// Live-tunable control parameters (written by loop(), read by controlTask)
static volatile float gKp = KP_DEFAULT, gKi = KI_DEFAULT, gKpPos = KP_POS_DEFAULT;
static volatile float gKff = KFF_DEFAULT, gVelLimit = VEL_LIMIT_DEFAULT, gUqLimit = UQ_LIMIT_DEFAULT;
static volatile float gMoveSpeed = MOVE_SPEED_DEFAULT * RAD_PER_DEG;   // [rad/s]
static volatile float gMoveAccel = MOVE_ACCEL_DEFAULT * RAD_PER_DEG;   // [rad/s^2]

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
static float encReadAngle() {
  uint8_t st = 0;
  uint32_t raw = encoder_read_retry(&st);
  if (st == 0xFF) gEncErr++;
  return raw * RAD_PER_COUNT;
}

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
// Motion profile: moves the reference towards the target.
//   rem = signed distance still to go [rad], v = reference speed [rad/s]
// Kept as a distance rather than an absolute position so a long multi-turn
// move loses no float resolution as it nears the end. Velocity is continuous,
// so a new target during a move blends into it.
// ---------------------------------------------------------------
static inline void profileStep(float &rem, float &v, float vMax, float accel, float dt) {
  if (rem == 0.0f && v == 0.0f) return;
  float vWant = copysignf(fminf(vMax, sqrtf(2.0f * accel * fabsf(rem))), rem);
  float dv = accel * dt;
  v += constrain(vWant - v, -dv, dv);
  rem -= v * dt;
  if (fabsf(rem) < 2.0f * RAD_PER_COUNT && fabsf(v) < 4.0f * dv) { rem = 0.0f; v = 0.0f; }
}

// ---------------------------------------------------------------
// Control task (5 kHz)
//   target -> profile -> position P (+ profile speed) -> velocity PI -> Uq
//   All position/velocity values are in the ENCODER frame, relative to the
//   kinematic zero; the sign needed to turn that into a motor-frame voltage
//   is gSensorDir (found by calibration).
// ---------------------------------------------------------------
static void controlTask(void *arg) {
  bool lockout = !(bool)(intptr_t)arg;             // failed calibration: never drive

  uint32_t prevRaw = encoder_read_retry(nullptr);
  int64_t posCounts = encoder_delta_counts(prevRaw, gZeroCounts);   // starts in +/- half a turn
  int64_t targetCounts = posCounts;
  float rem = 0.0f, profVel = 0.0f;
  float vel = 0.0f, integ = 0.0f;
  bool needResync = true;                          // start by holding the present position
  int64_t tPrev = esp_timer_get_time();
  const float NOMINAL_DT = CTRL_PERIOD_US * 1e-6f;
  const float MAX_DT = 0.005f;                     // WiFi start-up can delay a tick

  // DRV8316 fault state
  int faultCnt = 0, clearAttempts = 0, faultEvents = 0;
  bool faulted = lockout;
  int64_t faultHighSince = 0, lastClrT = 0, faultWindowStart = 0;

  // encoder health
  int encBad = 0, encGood = 0;
  bool encLost = false;

  // timers
  int64_t stallSince = 0, lastBusy = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t now = esp_timer_get_time();

    // ================= nFAULT supervision =================
    bool nf = (digitalRead(PIN_NFAULT) == LOW);
    faultCnt = nf ? faultCnt + 1 : 0;

    if (!faulted && faultCnt >= 3) {                 // new fault event
      faulted = true;
      needResync = true;
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

    if (lockout) {
      gFaultLockout = true;                          // stays braked until reset
    } else if (faulted) {
      if (nf) {
        faultHighSince = 0;
        if (now - lastClrT > FAULT_RETRY_US) {       // latched fault: try CLR_FLT
          if (++clearAttempts > FAULT_MAX_CLEARS) {
            lockout = true;
            gFaultLockout = true;
          } else {
            drvClearFault();
            lastClrT = now;
          }
        }
      } else {
        if (faultHighSince == 0) faultHighSince = now;
        if (now - faultHighSince > FAULT_RETRY_US) { // fault-free long enough -> resume
          faulted = false;
          gFaultActive = false;
        }
      }
    }

    // ================= encoder: checked read =================
    // Keeps running while braked, so the page still shows the real angle.
    float dt = (now - tPrev) * 1e-6f;                // time since last GOOD sample
    if (dt <= 0.0f) continue;

    uint32_t raw = 0;
    uint8_t  st  = 0;
    bool ok = encoder_read(&raw, &st) && !(st & ENC_ST_BAD);
    int32_t dC = 0;
    if (ok) {
      dC = encoder_delta_counts(raw, prevRaw);
      float maxC = OWN_MAX_SPEED * dt / RAD_PER_COUNT + 2000.0f;
      if (!encLost && fabsf((float)dC) > maxC) ok = false;   // implausible jump
    }

    if (!ok) {
      gEncErr++;
      encGood = 0;
      gTelEncOk = false;
      if (!encLost && ++encBad >= ENC_BAD_LIMIT) {
        encLost = true;
        needResync = true;
        integ = 0;
        setBrake(true);
        gEncLostReport = true;
        gTelEncLost = true;
      }
      continue;                                      // keep last duties for this cycle
    }
    encBad = 0;

    if (encLost) {                                   // need a run of good reads to resume
      if (++encGood < ENC_RECOVER_GOOD) continue;
      encLost = false;
      gTelEncLost = false;
      encGood = 0;
      dt = NOMINAL_DT;                               // dC: shortest way round, keeps the angle absolute
    }

    prevRaw = raw;
    tPrev = now;
    posCounts += dC;
    vel += VEL_ALPHA * ((dC * RAD_PER_COUNT) / dt - vel);
    if (dt > MAX_DT) dt = MAX_DT;

    gTelEncOk = true;
    gTelAngleCounts = encoder_delta_counts(raw, gZeroCounts);

    // ================= braked: report only =================
    if (faulted) {
      gCmdPending = false;                           // a stale target must not run on resume
      gTelVel = vel;
      continue;
    }
    if (needResync) {                                // hold here; the page shows the new target
      needResync = false;
      targetCounts = posCounts;
      rem = 0; profVel = 0;
      vel = 0; integ = 0;
      stallSince = 0;
      gCmdPending = false;
      gTelTargetDeg = MAGNET_DIRECTION * (float)targetCounts * DEG_PER_COUNT;
      setBrake(false);
    }

    // ================= new target =================
    if (gCmdPending) {
      float deg = gCmdDeg;
      gCmdPending = false;
      int64_t newTarget = lroundf(MAGNET_DIRECTION * deg / DEG_PER_COUNT);
      rem += (float)(newTarget - targetCounts) * RAD_PER_COUNT;
      targetCounts = newTarget;
      gTelTargetDeg = deg;
    }

    // ================= profile + leash =================
    // The reference is (target - rem). If the rotor is held back the reference
    // waits for it, so releasing the shaft does not cause a lunge.
    float toTarget = (float)(targetCounts - posCounts) * RAD_PER_COUNT;
    float errRad = toTarget - rem;
    if (fabsf(errRad) > LEASH_RAD) {
      profVel = 0.0f;
      errRad = copysignf(LEASH_RAD, errRad);
    } else {
      profileStep(rem, profVel, gMoveSpeed, gMoveAccel, dt);
      errRad = toTarget - rem;
    }

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
    float velSp = constrain(gKpPos * errRad + profVel, -vLim, vLim);
    float verr  = velSp - vel;
    float uff   = gKff * velSp;                      // back-EMF feed-forward: voltage for target speed
    integ += gKi * verr * dt;
    integ = constrain(integ, -uLim, uLim);
    float u = constrain(uff + gKp * verr + integ, -uLim, uLim);   // + = want encoder angle to increase

    // ================= commutation =================
    float angle  = raw * RAD_PER_COUNT;
    float thetaE = wrap2Pi(gSensorDir * POLE_PAIRS * angle - gZeroElec);
    setPhaseVoltages(0.0f, gSensorDir * u, thetaE);

    // ================= telemetry =================
    if (rem != 0.0f || fabsf(toTarget) > MOVING_BAND_DEG * RAD_PER_DEG) lastBusy = now;
    gTelMoving = lastBusy != 0 && (now - lastBusy < MOVING_SETTLE_US);
    gTelPos = (float)posCounts * RAD_PER_COUNT;
    gTelRef = (float)targetCounts * RAD_PER_COUNT - rem;
    gTelErr = errRad; gTelVel = vel; gTelUq = u; gTelStall = stalled;
    if (fabsf(errRad) > gTelErrMax) gTelErrMax = fabsf(errRad);
    gTelUperVel = (fabsf(vel) > 3.0f) ? u / vel : 0.0f;  // for choosing kff

    uint32_t loopUs = (uint32_t)(esp_timer_get_time() - now);
    if (loopUs > gLoopMaxUs) gLoopMaxUs = loopUs;
  }
}

// Hardware timer, not esp_timer: the esp_timer task runs below the WiFi task,
// so its ticks would wait for WiFi on this single-core chip.
static bool onCtrlTick(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *) {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(gCtrlTask, &woken);
  return woken == pdTRUE;
}

static void startCtrlTimer() {
  gptimer_handle_t h;
  gptimer_config_t tc = {};
  tc.clk_src       = GPTIMER_CLK_SRC_DEFAULT;
  tc.direction     = GPTIMER_COUNT_UP;
  tc.resolution_hz = 1000000;
  ESP_ERROR_CHECK(gptimer_new_timer(&tc, &h));

  gptimer_event_callbacks_t cb = {};
  cb.on_alarm = onCtrlTick;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(h, &cb, nullptr));

  gptimer_alarm_config_t ac = {};
  ac.alarm_count = CTRL_PERIOD_US;
  ac.flags.auto_reload_on_alarm = true;
  ESP_ERROR_CHECK(gptimer_set_alarm_action(h, &ac));
  ESP_ERROR_CHECK(gptimer_enable(h));
  ESP_ERROR_CHECK(gptimer_start(h));
}

// ---------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------
bool motor_init() {
  // Same arithmetic as the stepper page: displayed = direction * raw - START_AXIS_DEG_ANGLE
  gZeroCounts = (uint32_t)lroundf(MAGNET_DIRECTION * START_AXIS_DEG_ANGLE / DEG_PER_COUNT) & ENC_MASK;

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
    Serial.println("Calibration failed - staying braked. The page will show the fault.");
    drvPrintStatus();
  }
  return calOk;
}

void motor_start(bool calibrationOk) {
  // Above the WiFi task (configMAX_PRIORITIES - 2): see onCtrlTick
  xTaskCreate(controlTask, "ctrl", 4096, (void *)(intptr_t)calibrationOk,
              configMAX_PRIORITIES - 1, &gCtrlTask);
  startCtrlTimer();
}

void motor_set_target_angle(float angleDeg) {
  angleDeg = constrain(angleDeg, -TARGET_LIMIT_DEG, TARGET_LIMIT_DEG);
  gCmdDeg = angleDeg;
  gCmdPending = true;
  Serial.printf("New target: %.2f deg\n", angleDeg);
}

float motor_get_angle_deg()        { return MAGNET_DIRECTION * (float)gTelAngleCounts * DEG_PER_COUNT; }
bool  motor_encoder_is_valid()     { return gTelEncOk; }
float motor_get_target_angle_deg() { return gTelTargetDeg; }
bool  motor_is_moving()            { return gTelMoving; }

uint8_t motor_get_status_flags() {
  uint8_t f = 0;
  if (gTelStall)     f |= MOTOR_FLAG_STALL;
  if (gFaultActive)  f |= MOTOR_FLAG_FAULT;
  if (gFaultLockout) f |= MOTOR_FLAG_LOCKOUT;
  return f;
}

// ---------------------------------------------------------------
// Live tuning over Serial. One command per line, e.g. "vkp 0.15"
// ---------------------------------------------------------------
static void printGains() {
  Serial.printf("GAINS  vkp=%.3f  vki=%.2f  pkp=%.1f  kff=%.3f  vlim=%.1f  uq=%.2f  speed=%.1f  accel=%.1f\n",
                gKp, gKi, gKpPos, gKff, gVelLimit, gUqLimit,
                gMoveSpeed / RAD_PER_DEG, gMoveAccel / RAD_PER_DEG);
}

static void printHelp() {
  Serial.println("Commands, one per line:");
  Serial.println("  move <deg>            go to a target angle (same as the web page)");
  Serial.println("  speed <deg/s>         move cruising speed");
  Serial.println("  accel <deg/s/s>       move acceleration");
  Serial.println("  vkp <V per rad/s>     velocity loop P   (more = stiffer, too much = buzz)");
  Serial.println("  vki <V per rad/s/s>   velocity loop I   (more = less lag, too much = overshoot)");
  Serial.println("  pkp <rad/s per rad>   position loop P   (more = snappier, too much = ringing)");
  Serial.println("  kff <V per rad/s>     back-EMF feed-forward (~0.8 x u/vel while spinning)");
  Serial.println("  vlim <rad/s>          max speed");
  Serial.println("  uq <V>                max phase voltage (watch motor temperature)");
  Serial.println("  log 0|1               periodic status printout off/on");
  Serial.println("  show                  print current values");
}

void motor_handle_serial() {
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

    if (!strcmp(cmd, "move")) { motor_set_target_angle(v); continue; }
    if      (!strcmp(cmd, "speed")) gMoveSpeed = constrain(v, 1.0f, 3600.0f) * RAD_PER_DEG;
    else if (!strcmp(cmd, "accel")) gMoveAccel = constrain(v, 10.0f, 36000.0f) * RAD_PER_DEG;
    else if (!strcmp(cmd, "vkp"))   gKp        = constrain(v, 0.0f, 2.0f);
    else if (!strcmp(cmd, "vki"))   gKi        = constrain(v, 0.0f, 100.0f);
    else if (!strcmp(cmd, "pkp"))   gKpPos     = constrain(v, 0.0f, 200.0f);
    else if (!strcmp(cmd, "kff"))   gKff       = constrain(v, 0.0f, 1.0f);
    else if (!strcmp(cmd, "vlim"))  gVelLimit  = constrain(v, 1.0f, 200.0f);
    else if (!strcmp(cmd, "uq"))    gUqLimit   = constrain(v, 0.5f, 0.55f * VBUS);   // SVPWM linear range
    else { Serial.println("? unknown command (type \"help\")"); continue; }
    printGains();
  }
}

void motor_report_events() {
  static bool lockoutReported = false;
  if (gFaultReport) {
    gFaultReport = false;
    Serial.printf("!!! nFAULT: IC_STAT=0x%02X STATUS1=0x%02X STATUS2=0x%02X (braked, will try CLR_FLT)\n",
                  gFaultRegs[0], gFaultRegs[1], gFaultRegs[2]);
  }
  if (gFaultLockout && !lockoutReported) {
    lockoutReported = true;
    Serial.println("!!! Motor LOCKED OUT (braked): calibration failed, or the DRV8316 fault did not clear. Reset to retry.");
  }
  if (gEncLostReport) {
    gEncLostReport = false;
    Serial.println("!!! encoder lost (CRC / status / implausible) - braked until it recovers, then holds there");
  }
}

void motor_print_status() {
  if (!gPrintOn) return;
  float upv = gTelUperVel;
  Serial.printf("target=%+9.2f ref=%+9.2f pos=%+9.2f deg  err=%+6.3f errMax=%5.3f rad  vel=%+6.2f Uq=%+5.2f u/vel=",
                gTelTargetDeg, gTelRef / RAD_PER_DEG, gTelPos / RAD_PER_DEG,
                gTelErr, gTelErrMax, gTelVel, gTelUq);
  if (upv != 0.0f) Serial.printf("%.3f", upv); else Serial.print("-");
  Serial.printf(" encErr=%u loopMax=%uus%s%s\n", (unsigned)gEncErr, (unsigned)gLoopMaxUs,
                gTelStall ? "  STALL" : "", gTelEncLost ? "  ENC-LOST" : "");
  gLoopMaxUs = 0;
  gTelErrMax = 0;
}
