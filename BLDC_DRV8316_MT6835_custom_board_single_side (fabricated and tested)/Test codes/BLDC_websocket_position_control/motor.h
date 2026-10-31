#pragma once
#include <Arduino.h>

// motor_get_status_flags() bits - also sent in the telemetry packet
constexpr uint8_t MOTOR_FLAG_STALL   = 0x04;   // blocked, voltage reduced
constexpr uint8_t MOTOR_FLAG_FAULT   = 0x08;   // DRV8316 fault, braked, retrying
constexpr uint8_t MOTOR_FLAG_LOCKOUT = 0x10;   // braked until reset

// Sets up PWM + DRV8316 and calibrates the encoder against the rotor (the motor
// moves a little). Returns false if calibration failed. Call once from setup().
bool motor_init();

// Starts the 5 kHz control task; the motor holds its position until a target
// arrives. If calibrationOk is false it stays braked and only reports the angle.
void motor_start(bool calibrationOk);

// Commands a new target angle (degrees), absolute and multi-turn like the
// stepper: 720 is two turns from zero. Safe to call from any task.
// Called from the websocket text-message handler when the user submits a value.
void motor_set_target_angle(float angleDeg);

// Zero-referenced, wrapped-to-[-180,180) angle in degrees, from the last good read.
float motor_get_angle_deg();

// True if the control task's last encoder read was good.
bool motor_encoder_is_valid();

// The last accepted target angle, in degrees.
float motor_get_target_angle_deg();

// True during a move and until the rotor has settled on the target.
bool motor_is_moving();

// MOTOR_FLAG_* bits.
uint8_t motor_get_status_flags();

// Live tuning + status over Serial. Call all three from loop();
// motor_print_status at a low rate.
void motor_handle_serial();
void motor_print_status();
void motor_report_events();
