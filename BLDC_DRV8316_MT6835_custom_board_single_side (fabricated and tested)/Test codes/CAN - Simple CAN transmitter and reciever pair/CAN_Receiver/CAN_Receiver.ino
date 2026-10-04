#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN  4
#define CAN_RX_PIN  5

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("ESP32-S3 CAN Receiver");

    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN,
            (gpio_num_t)CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_500KBITS();

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

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

    result = twai_start();

    if (result != ESP_OK)
    {
        Serial.printf(
            "TWAI start failed: %s\n",
            esp_err_to_name(result)
        );
        return;
    }

    Serial.println("CAN started at 500 kbit/s");
}


void loop()
{
    twai_message_t message;

    // Wait up to 1 second for a CAN message
    esp_err_t result =
        twai_receive(
            &message,
            pdMS_TO_TICKS(1000)
        );

    if (result == ESP_OK)
    {
        Serial.println();
        Serial.println("CAN message received");

        // Standard or extended?
        if (message.extd)
        {
            Serial.println("Format: Extended (29-bit)");
        }
        else
        {
            Serial.println("Format: Standard (11-bit)");
        }

        Serial.printf(
            "ID: 0x%03X\n",
            message.identifier
        );

        Serial.printf(
            "DLC: %d\n",
            message.data_length_code
        );

        Serial.print("Data: ");

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
    }
}