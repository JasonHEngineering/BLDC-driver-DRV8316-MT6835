#pragma once
#include <Arduino.h>

// MT6835 status bits (STATUS[2:0]) - verify against your datasheet revision
constexpr uint8_t ENC_ST_OVERSPEED = 0x01;
constexpr uint8_t ENC_ST_WEAK_MAG  = 0x02;
constexpr uint8_t ENC_ST_UV        = 0x04;
constexpr uint8_t ENC_ST_BAD       = ENC_ST_WEAK_MAG | ENC_ST_UV;   // angle not trustworthy

// One burst read of angle + status. Returns false if the CRC does not match.
// Time-critical path: only the control task calls this once it is running.
bool encoder_read(uint32_t *raw, uint8_t *status);

// For non-time-critical use (setup, calibration): retries, falls back to the
// last good value. *status = 0xFF if every retry failed.
uint32_t encoder_read_retry(uint8_t *status);

// Reads the encoder 20 times and prints CRC + magnet health. Call from setup().
void encoder_self_test();

// Signed difference of two 21-bit angles, exact integer arithmetic.
static inline int32_t encoder_delta_counts(uint32_t now, uint32_t prev)
{
    uint32_t d = (now - prev) & 0x1FFFFF;
    return (d & 0x100000) ? (int32_t)d - 0x200000 : (int32_t)d;
}
