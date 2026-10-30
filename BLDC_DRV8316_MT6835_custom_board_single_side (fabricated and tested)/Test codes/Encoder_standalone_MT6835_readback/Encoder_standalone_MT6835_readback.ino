#include <Arduino.h>
#include <SPI.h>

// ============================================================
// MT6835 + ESP32-C6
// ============================================================
//
// ESP32-C6 custom board
//
// GPIO22 -> MT6835 CSN
// GPIO21 -> MT6835 MISO
// GPIO19 -> MT6835 SCK
// GPIO20 -> MT6835 MOSI
//
// SPI Mode 3
// MSB first
// ============================================================

#define MT6835_CS      22
#define MT6835_MISO    21
#define MT6835_SCK     19
#define MT6835_MOSI    20

#define MT6835_SPI_SPEED 1000000UL

SPIClass *mtSPI = &SPI;


// ============================================================
// MT6835 Burst Read
//
// Command:
//
// C3 C2 C1 C0 A11 A10 ... A0
//
// Burst command = 1010
// Starting address = 0x003
//
// After command/address, MT6835 outputs:
//
// DATA 1 = register 0x003
// DATA 2 = register 0x004
// DATA 3 = register 0x005
// DATA 4 = register 0x006
// ============================================================

bool mt6835ReadAngle(
    uint32_t &angleRaw,
    uint8_t &status,
    uint8_t &crc
)
{
    uint8_t angleHigh;
    uint8_t angleMid;
    uint8_t angleLowStatus;
    uint8_t crcByte;

    // --------------------------------------------------------
    // Burst command
    //
    // 4-bit command = 1010
    // 12-bit address = 0x003
    //
    // 0xA003
    // --------------------------------------------------------

    const uint16_t command = 0xA003;

    mtSPI->beginTransaction(
        SPISettings(
            MT6835_SPI_SPEED,
            MSBFIRST,
            SPI_MODE3
        )
    );

    // CSN LOW starts transaction and latches angle
    digitalWrite(MT6835_CS, LOW);

    // --------------------------------------------------------
    // Send command + address
    // --------------------------------------------------------

    mtSPI->transfer((command >> 8) & 0xFF);
    mtSPI->transfer(command & 0xFF);

    // --------------------------------------------------------
    // Read four consecutive registers
    //
    // 0x003
    // 0x004
    // 0x005
    // 0x006
    // --------------------------------------------------------

    angleHigh = mtSPI->transfer(0x00);
    angleMid = mtSPI->transfer(0x00);
    angleLowStatus = mtSPI->transfer(0x00);
    crcByte = mtSPI->transfer(0x00);

    // CSN HIGH terminates burst read
    digitalWrite(MT6835_CS, HIGH);

    mtSPI->endTransaction();

    // --------------------------------------------------------
    // Assemble 21-bit angle
    //
    // 0x003:
    //   ANGLE[20:13]
    //
    // 0x004:
    //   ANGLE[12:5]
    //
    // 0x005:
    //   ANGLE[4:0] in bits 7:3
    //   STATUS[2:0] in bits 2:0
    // --------------------------------------------------------

    angleRaw =
        ((uint32_t)angleHigh << 13) |
        ((uint32_t)angleMid << 5) |
        ((uint32_t)(angleLowStatus >> 3) & 0x1F);

    // --------------------------------------------------------
    // Extract status
    // --------------------------------------------------------

    status = angleLowStatus & 0x07;

    // --------------------------------------------------------
    // CRC
    // --------------------------------------------------------

    crc = crcByte;

    return true;
}


// ============================================================
// Convert 21-bit raw angle to degrees
// ============================================================

float mt6835RawToDegrees(uint32_t raw)
{
    return ((float)raw * 360.0f) / 2097152.0f;
}


// ============================================================
// CRC-8
//
// Polynomial:
// x^8 + x^2 + x + 1
//
// Polynomial = 0x07
//
// CRC covers:
//
// ANGLE[20:0] + STATUS[2:0]
//
// Total = 24 bits
// ============================================================

uint8_t mt6835CalculateCRC(
    uint8_t angleHigh,
    uint8_t angleMid,
    uint8_t angleLowStatus
)
{
    uint8_t crc = 0x00;

    uint8_t data[3] =
    {
        angleHigh,
        angleMid,
        angleLowStatus
    };

    for (int byteIndex = 0; byteIndex < 3; byteIndex++)
    {
        uint8_t dataByte = data[byteIndex];

        for (int bit = 7; bit >= 0; bit--)
        {
            uint8_t inputBit =
                (dataByte >> bit) & 0x01;

            uint8_t crcMSB =
                (crc >> 7) & 0x01;

            crc <<= 1;

            if (crcMSB ^ inputBit)
            {
                crc ^= 0x07;
            }
        }
    }

    return crc;
}


// ============================================================
// More complete diagnostic burst read
//
// This version returns all four bytes so that we can verify
// CRC as well as angle.
// ============================================================

bool mt6835ReadRaw(
    uint8_t &angleHigh,
    uint8_t &angleMid,
    uint8_t &angleLowStatus,
    uint8_t &crcByte
)
{
    const uint16_t command = 0xA003;

    mtSPI->beginTransaction(
        SPISettings(
            MT6835_SPI_SPEED,
            MSBFIRST,
            SPI_MODE3
        )
    );

    digitalWrite(MT6835_CS, LOW);

    // Command + address
    mtSPI->transfer((command >> 8) & 0xFF);
    mtSPI->transfer(command & 0xFF);

    // Burst data
    angleHigh = mtSPI->transfer(0x00);
    angleMid = mtSPI->transfer(0x00);
    angleLowStatus = mtSPI->transfer(0x00);
    crcByte = mtSPI->transfer(0x00);

    digitalWrite(MT6835_CS, HIGH);

    mtSPI->endTransaction();

    return true;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("MT6835 SPI Burst Read Test");
    Serial.println("ESP32-C6 Custom Board");
    Serial.println("========================================");

    // --------------------------------------------------------
    // Configure CS
    // --------------------------------------------------------

    pinMode(MT6835_CS, OUTPUT);

    // MT6835 requires CSN HIGH when idle
    digitalWrite(MT6835_CS, HIGH);

    // --------------------------------------------------------
    // Start SPI using custom PCB pinout
    // --------------------------------------------------------

    mtSPI->begin(
        MT6835_SCK,
        MT6835_MISO,
        MT6835_MOSI,
        MT6835_CS
    );

    delay(100);

    Serial.println("SPI configuration:");
    Serial.printf("CS   = GPIO%d\n", MT6835_CS);
    Serial.printf("MISO = GPIO%d\n", MT6835_MISO);
    Serial.printf("SCK  = GPIO%d\n", MT6835_SCK);
    Serial.printf("MOSI = GPIO%d\n", MT6835_MOSI);
    Serial.println();

    Serial.println("SPI Mode: 3");
    Serial.println("SPI Speed: 1 MHz");
    Serial.println();

    Serial.println("Raw angle readback:");
    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    uint8_t angleHigh;
    uint8_t angleMid;
    uint8_t angleLowStatus;
    uint8_t crcReceived;

    // --------------------------------------------------------
    // Read MT6835
    // --------------------------------------------------------

    mt6835ReadRaw(
        angleHigh,
        angleMid,
        angleLowStatus,
        crcReceived
    );

    // --------------------------------------------------------
    // Assemble angle
    // --------------------------------------------------------

    uint32_t angleRaw =
        ((uint32_t)angleHigh << 13) |
        ((uint32_t)angleMid << 5) |
        ((uint32_t)(angleLowStatus >> 3) & 0x1F);

    // --------------------------------------------------------
    // Status
    // --------------------------------------------------------

    uint8_t status =
        angleLowStatus & 0x07;

    // --------------------------------------------------------
    // Convert to degrees
    // --------------------------------------------------------

    float angleDegrees =
        mt6835RawToDegrees(angleRaw);

    // --------------------------------------------------------
    // Calculate CRC
    // --------------------------------------------------------

    uint8_t crcCalculated =
        mt6835CalculateCRC(
            angleHigh,
            angleMid,
            angleLowStatus
        );

    bool crcOK =
        (crcCalculated == crcReceived);

    // --------------------------------------------------------
    // Print
    // --------------------------------------------------------

    Serial.print("Raw: ");
    Serial.print(angleRaw);

    Serial.print("   Angle: ");
    Serial.print(angleDegrees, 4);
    Serial.print(" deg");

    Serial.print("   Status: 0x");
    if (status < 0x10)
        Serial.print("0");

    Serial.print(status, HEX);

    Serial.print("   CRC RX: 0x");
    if (crcReceived < 0x10)
        Serial.print("0");

    Serial.print(crcReceived, HEX);

    Serial.print("   CRC CALC: 0x");
    if (crcCalculated < 0x10)
        Serial.print("0");

    Serial.print(crcCalculated, HEX);

    Serial.print("   ");

    if (crcOK)
        Serial.print("CRC OK");
    else
        Serial.print("!!! CRC ERROR !!!");

    Serial.println();

    delay(20);
}