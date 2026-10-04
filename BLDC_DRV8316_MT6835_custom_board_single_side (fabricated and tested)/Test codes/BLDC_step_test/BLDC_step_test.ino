/*
  Six-step hardware test: ESP32-C6 + DRV8316 (6x PWM mode) + MT6835
  --------------------------------------------------------------
  Purpose: find out whether the driver + motor wiring work, WITHOUT the MCPWM
  peripheral, dead-time logic, or FOC maths from the main sketch.

  It energizes two phases at a time (6 states = 1 electrical revolution),
  using plain LEDC PWM (analogWrite) on the high-side pin and a constant-on
  low-side pin on the return phase. Unused phases are left Hi-Z (INH=INL=0).
  Every step prints the encoder angle. If the motor works, the angle moves
  by about 2*pi/(7*6) = 0.15 rad per step.

  WATCH YOUR BENCH SUPPLY CURRENT DISPLAY while this runs.
  Use a current-limited supply (~0.5 A to start).
*/

#include <Arduino.h>
#include <SPI.h>

constexpr int PIN_INHA = 2,  PIN_INLA = 3;
constexpr int PIN_INHB = 0,  PIN_INLB = 1;
constexpr int PIN_INHC = 7,  PIN_INLC = 14;
constexpr int PIN_NFAULT = 15;
constexpr int PIN_DRV_CS = 23, PIN_ENC_CS = 22;
constexpr int PIN_SCLK = 19, PIN_MOSI = 20, PIN_MISO = 21;

// Effective voltage ~= VBUS * TEST_DUTY/255 applied across two windings.
// 64/255 = 25% -> about 2 V at 8 V supply. Raise (e.g. 100) only if nothing moves.
constexpr int TEST_DUTY = 64;
constexpr int STEP_MS   = 400;

const int PINS[6] = {PIN_INHA, PIN_INLA, PIN_INHB, PIN_INLB, PIN_INHC, PIN_INLC};
// index: 0=INHA 1=INLA 2=INHB 3=INLB 4=INHC 5=INLC

// For each of the 6 states: {high-side pin index (PWM), low-side pin index (on)}
const int STATES[6][2] = {
  {0, 3},   // A+ B-
  {0, 5},   // A+ C-
  {2, 5},   // B+ C-
  {2, 1},   // B+ A-
  {4, 1},   // C+ A-
  {4, 3},   // C+ B-
};

static float encAngle() {
  uint8_t tx[5] = {0xA0, 0x03, 0, 0, 0}, rx[5] = {0};
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_ENC_CS, LOW);
  SPI.transferBytes(tx, rx, 5);
  digitalWrite(PIN_ENC_CS, HIGH);
  SPI.endTransaction();
  uint32_t raw = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | (rx[4] >> 3);
  return raw * (TWO_PI / 2097152.0f);
}

static uint8_t drvRead(uint8_t addr) {
  uint16_t cmd = 0x8000 | ((uint16_t)addr << 9);
  if (__builtin_parity(cmd)) cmd |= 0x0100;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_DRV_CS, LOW);
  delayMicroseconds(1);
  uint16_t r = SPI.transfer16(cmd);
  delayMicroseconds(1);
  digitalWrite(PIN_DRV_CS, HIGH);
  SPI.endTransaction();
  return r & 0xFF;
}

static void drvWrite(uint8_t addr, uint8_t data) {
  uint16_t cmd = ((uint16_t)addr << 9) | data;
  if (__builtin_parity(cmd)) cmd |= 0x0100;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_DRV_CS, LOW);
  delayMicroseconds(1);
  SPI.transfer16(cmd);
  delayMicroseconds(1);
  digitalWrite(PIN_DRV_CS, HIGH);
  SPI.endTransaction();
}

static void allOff() {
  for (int i = 0; i < 6; i++) analogWrite(PINS[i], 0);
}

static void applyState(int s) {
  allOff();
  delayMicroseconds(50);                 // never overlap old and new states
  analogWrite(PINS[STATES[s][1]], 255);  // low side: constantly on
  analogWrite(PINS[STATES[s][0]], TEST_DUTY);  // high side: PWM
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nSix-step test");

  pinMode(PIN_DRV_CS, OUTPUT); digitalWrite(PIN_DRV_CS, HIGH);
  pinMode(PIN_ENC_CS, OUTPUT); digitalWrite(PIN_ENC_CS, HIGH);
  pinMode(PIN_NFAULT, INPUT_PULLUP);
  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

  for (int i = 0; i < 6; i++) {
    pinMode(PINS[i], OUTPUT);
    analogWriteFrequency(PINS[i], 20000);
  }
  allOff();

  drvWrite(0x03, 0x03);          // unlock
  drvWrite(0x04, 0x61);          // 6x mode + clear faults
  delay(5);
  Serial.printf("IC_STAT=0x%02X (expect 0x08), nFAULT=%d\n", drvRead(0x00), digitalRead(PIN_NFAULT));
  Serial.println("Starting in 2 s - watch supply current and the motor shaft.");
  delay(2000);
}

void loop() {
  static int s = 0;
  static float prev = encAngle();

  applyState(s);
  delay(STEP_MS);

  float a = encAngle();
  float d = a - prev;
  if (d >  PI) d -= TWO_PI;
  if (d < -PI) d += TWO_PI;
  Serial.printf("state %d  angle=%.3f rad  delta=%+.3f  nFAULT=%d  IC_STAT=0x%02X\n",
                s, a, d, digitalRead(PIN_NFAULT), drvRead(0x00));
  prev = a;

  s = (s + 1) % 6;
}
