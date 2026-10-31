#pragma once
#include <Arduino.h>

// ============================================================
//  WiFi
// ============================================================

constexpr const char* WIFI_SSID     = "StarHub_DB58";
constexpr const char* WIFI_PASSWORD = "rH4nz8UN4u";

// ============================================================
//  Pin assignment (ESP32-C6-MINI-1 board, same as bldc_can_bus_knob_follower)
// ============================================================

constexpr int PIN_INHA = 2,  PIN_INLA = 3;
constexpr int PIN_INHB = 0,  PIN_INLB = 1;
constexpr int PIN_INHC = 7,  PIN_INLC = 14;
constexpr int PIN_NFAULT  = 15;      // open-drain, active low
constexpr int PIN_DRV_CS  = 23;
constexpr int PIN_ENC_CS  = 22;
constexpr int PIN_SCLK    = 19;
constexpr int PIN_MOSI    = 20;
constexpr int PIN_MISO    = 21;
constexpr int PIN_CAN_TX  = 4;       // CAN is not used here: TX is held recessive
constexpr int PIN_CAN_RX  = 5;

// ============================================================
//  Encoder (MT6835)
// ============================================================

constexpr int   MAGNET_DIRECTION     = 1;     // +1 or -1, flip if angle increases the wrong way
constexpr float START_AXIS_DEG_ANGLE = 0.0f;  // raw encoder angle at kinematic zero

constexpr bool    ENC_CHECK_CRC    = true;    // set false only if the start-up CRC test always fails
constexpr float   OWN_MAX_SPEED    = 300.0f;  // plausibility limit for encoder steps [rad/s]
constexpr int     ENC_BAD_LIMIT    = 10;      // bad reads in a row (2 ms) -> brake
constexpr int     ENC_RECOVER_GOOD = 50;      // good reads in a row (10 ms) -> resume

constexpr uint32_t ENC_MASK      = 0x1FFFFF;                   // 21 bit
constexpr float    RAD_PER_COUNT = TWO_PI / 2097152.0f;
constexpr float    DEG_PER_COUNT = 360.0f / 2097152.0f;

// ============================================================
//  Motor (GB2806) / power
// ============================================================

constexpr int   POLE_PAIRS        = 7;
constexpr float VBUS              = 24.0f;   // supply voltage [V]  <-- set to your real supply
constexpr float UQ_LIMIT_DEFAULT  = 5.0f;    // max phase voltage [V] - watch motor temperature
constexpr float ALIGN_VOLTAGE     = 2.0f;    // first calibration voltage [V]
constexpr float ALIGN_VOLTAGE_MAX = 5.0f;    // calibration retries x1.5 up to this [V]

// ============================================================
//  Motion profile (trapezoidal move to each new target)
// ============================================================
// Defaults reproduce the stepper's motion: 112.5 deg/s is FULL_SPEED (2000 steps/s
// at 1/32 microstepping), and the ramps span about the same as its LAUNCH_STEPS.
// Both can be changed live over Serial ("speed", "accel").

constexpr float MOVE_SPEED_DEFAULT = 2000.0f;  // cruising speed [deg/s]
constexpr float MOVE_ACCEL_DEFAULT = 4000.0f;  // acceleration   [deg/s^2]

constexpr float TARGET_LIMIT_DEG   = 36000.0f; // commands beyond +/- this are clamped (100 turns)
constexpr float MOVING_BAND_DEG    = 0.2f;     // "moving" until within this of the target ...
constexpr int64_t MOVING_SETTLE_US = 100000;   // ... for this long

// ============================================================
//  Position / velocity control
// ============================================================
// The *_DEFAULT values can be changed live over Serial - type "help".

constexpr float KP_DEFAULT        = 0.12f;   // velocity loop P  [V per rad/s]
constexpr float KI_DEFAULT        = 3.0f;    // velocity loop I  [V per rad/s per s]
constexpr float KP_POS_DEFAULT    = 30.0f;   // position loop P  [rad/s per rad]
constexpr float KFF_DEFAULT       = 0.0f;    // back-EMF feed-forward [V per rad/s] - set from "u/vel"
constexpr float VEL_LIMIT_DEFAULT = 4000.0f;   // max speed [rad/s]
constexpr float VEL_ALPHA         = 0.2f;    // speed filter, per 200 us (~1 ms)
constexpr float LEASH_RAD         = 1.0f;    // max error used by the controller [rad];
                                             // the profile waits while the rotor is further behind

// Stall protection
constexpr float   STALL_ERR_RAD   = 0.5f;    // error above this ...
constexpr float   STALL_VEL       = 0.5f;    // ... while slower than this [rad/s] ...
constexpr int64_t STALL_US        = 500000;  // ... for this long = stalled
constexpr float   STALL_UQ_LIMIT  = 1.0f;    // voltage limit while stalled [V]

// DRV8316 fault handling
constexpr int64_t FAULT_RETRY_US   = 500000;  // CLR_FLT interval / required fault-free time
constexpr int     FAULT_MAX_CLEARS = 10;      // CLR_FLT attempts per fault event before lockout
constexpr int     FAULT_MAX_EVENTS = 5;       // fault events within the window before lockout
constexpr int64_t FAULT_WINDOW_US  = 10000000;

// ============================================================
//  Timing / PWM
// ============================================================

constexpr uint32_t CTRL_PERIOD_US = 200;     // control loop: 5 kHz

constexpr uint32_t TIMER_RES_HZ   = 20000000;
constexpr uint32_t PWM_FREQ_HZ    = 25000;
constexpr uint32_t PERIOD_TICKS   = TIMER_RES_HZ / PWM_FREQ_HZ;  // 800
constexpr uint32_t PEAK_TICKS     = PERIOD_TICKS / 2;
constexpr uint32_t DEADTIME_TICKS = 10;                          // 500 ns

// ============================================================
//  Telemetry
// ============================================================

constexpr uint32_t TX_PERIOD_US = 20000; // 50 Hz broadcast. The C6 has one core shared with
                                         // the control loop and WiFi, so this is half the
                                         // stepper's rate; the page looks the same.
