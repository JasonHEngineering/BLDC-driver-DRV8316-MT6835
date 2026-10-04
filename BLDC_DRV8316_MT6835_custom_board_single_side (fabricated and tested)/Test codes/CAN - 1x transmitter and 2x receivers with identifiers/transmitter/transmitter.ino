#include <Arduino.h>
#include "driver/twai.h"

// ============================================================
// CAN GPIO
// ============================================================

#define CAN_TX_PIN  4
#define CAN_RX_PIN  5

// ============================================================
// CAN IDs
// ============================================================

#define CAN_ID_RECEIVER_1  0x100
#define CAN_ID_RECEIVER_2  0x200

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32-S3 CAN TRANSMITTER");
    Serial.println("================================");

    // --------------------------------------------------------
    // TWAI general configuration
    // --------------------------------------------------------

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN,
            (gpio_num_t)CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    // --------------------------------------------------------
    // CAN speed
    // --------------------------------------------------------

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_500KBITS();

    // --------------------------------------------------------
    // Accept all messages
    //
    // Filtering will be done in software on receivers.
    // --------------------------------------------------------

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // --------------------------------------------------------
    // Install TWAI driver
    // --------------------------------------------------------

    esp_err_t result =
        twai_driver_install(
            &g_config,
            &t_config,
            &f_config
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "TWAI driver install failed: %s\n",
            esp_err_to_name(result)
        );

        return;
    }

    Serial.println("TWAI driver installed");

    // --------------------------------------------------------
    // Start TWAI
    // --------------------------------------------------------

    result = twai_start();

    if (result != ESP_OK)
    {
        Serial.printf(
            "TWAI start failed: %s\n",
            esp_err_to_name(result)
        );

        return;
    }

    Serial.println("TWAI started");
    Serial.println("CAN speed: 500 kbit/s");
    Serial.println();
}


// ============================================================
// Send CAN message
// ============================================================

void sendCANMessage(
    uint32_t canID,
    uint8_t data0,
    uint8_t data1,
    uint8_t data2,
    uint8_t data3
)
{
    twai_message_t message = {};

    // --------------------------------------------------------
    // CAN identifier
    // --------------------------------------------------------

    message.identifier = canID;

    // Standard 11-bit CAN ID
    message.extd = 0;

    // Normal data frame
    message.rtr = 0;

    // Four data bytes
    message.data_length_code = 4;

    message.data[0] = data0;
    message.data[1] = data1;
    message.data[2] = data2;
    message.data[3] = data3;

    // --------------------------------------------------------
    // Transmit
    // --------------------------------------------------------

    esp_err_t result =
        twai_transmit(
            &message,
            pdMS_TO_TICKS(1000)
        );

    if (result == ESP_OK)
    {
        Serial.printf(
            "TX  ID=0x%03X  DATA=%02X %02X %02X %02X\n",
            canID,
            data0,
            data1,
            data2,
            data3
        );
    }
    else
    {
        Serial.printf(
            "CAN transmit failed: %s\n",
            esp_err_to_name(result)
        );
    }
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // Message intended for Receiver 1
    // --------------------------------------------------------

    sendCANMessage(
        CAN_ID_RECEIVER_1,
        0x11,
        0x22,
        0x33,
        0x44
    );

    delay(1000);

    // --------------------------------------------------------
    // Message intended for Receiver 2
    // --------------------------------------------------------

    sendCANMessage(
        CAN_ID_RECEIVER_2,
        0xAA,
        0xBB,
        0xCC,
        0xDD
    );

    delay(1000);
}