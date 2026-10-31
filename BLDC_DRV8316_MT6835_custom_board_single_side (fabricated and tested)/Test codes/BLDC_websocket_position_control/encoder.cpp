#include "encoder.h"
#include "config.h"
#include <SPI.h>

// ---------------------------------------------------------------
// MT6835 encoder (burst read 0xA003: ANGLE[20:13], ANGLE[12:5],
// ANGLE[4:0]|STATUS[2:0], CRC).  SPI mode 3.
// CRC-8, polynomial x^8 + x^2 + x + 1 (0x07), init 0x00, over the three
// angle/status bytes. If the start-up self-test reports 100 % CRC errors,
// check this definition against your datasheet revision.
// ---------------------------------------------------------------
bool encoder_read(uint32_t *raw, uint8_t *status) {
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

uint32_t encoder_read_retry(uint8_t *status) {
  static uint32_t lastGood = 0;
  uint32_t raw = 0;
  uint8_t st = 0;
  for (int i = 0; i < 3; i++) {
    if (encoder_read(&raw, &st)) {
      lastGood = raw;
      if (status) *status = st;
      return raw;
    }
  }
  if (status) *status = 0xFF;      // 0xFF = read failed
  return lastGood;
}

void encoder_self_test() {
  int crcFails = 0;
  uint32_t raw = 0;
  uint8_t st = 0;
  for (int i = 0; i < 20; i++) {
    if (!encoder_read(&raw, &st)) crcFails++;
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
