#include <Arduino.h>
#include "driver/twai.h"

// ============================================================
// CAN GPIO
// ============================================================

#define CAN_TX_PIN  4
#define CAN_RX_PIN  5

// ============================================================
// CAN ID for this receiver
// ============================================================

#define MY_CAN_ID 0x100


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32-S3 CAN RECEIVER 1");
    Serial.println("================================");

    // --------------------------------------------------------
    // TWAI configuration
    // --------------------------------------------------------

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN,
            (gpio_num_t)CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_500KBITS();

    // Receive everything.
    // Filtering is performed below in software.
    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // --------------------------------------------------------
    // Install driver
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
    Serial.println("Listening for CAN ID 0x100");
    Serial.println();
}


// ============================================================
// Main loop
// ============================================================

void loop()
{
    twai_message_t message;

    // --------------------------------------------------------
    // Wait for CAN message
    // --------------------------------------------------------

    esp_err_t result =
        twai_receive(
            &message,
            pdMS_TO_TICKS(1000)
        );

    if (result == ESP_OK)
    {
        // ----------------------------------------------------
        // Display every received CAN ID
        // ----------------------------------------------------

        Serial.printf(
            "RX  ID=0x%03X",
            message.identifier
        );

        Serial.print("  DATA=");

        for (int i = 0;
             i < message.data_length_code;
             i++)
        {
            Serial.printf(
                "%02X ",
                message.data[i]
            );
        }

        Serial.println();

        // ----------------------------------------------------
        // SOFTWARE FILTER
        // ----------------------------------------------------

        if (message.identifier == MY_CAN_ID)
        {
            Serial.println(
                ">>> Message intended for RECEIVER 1"
            );

            // ------------------------------------------------
            // Process the data here
            // ------------------------------------------------

            uint8_t value0 = message.data[0];
            uint8_t value1 = message.data[1];
            uint8_t value2 = message.data[2];
            uint8_t value3 = message.data[3];

            Serial.printf(
                "    Processed data: %02X %02X %02X %02X\n",
                value0,
                value1,
                value2,
                value3
            );
        }
        else
        {
            Serial.println(
                "    Ignored - not my CAN ID"
            );
        }

        Serial.println();
    }
}