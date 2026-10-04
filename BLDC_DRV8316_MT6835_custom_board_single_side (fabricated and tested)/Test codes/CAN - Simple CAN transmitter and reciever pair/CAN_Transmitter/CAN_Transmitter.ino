#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN  4
#define CAN_RX_PIN  5

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("ESP32-S3 CAN Transmitter");

    // General TWAI configuration
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)CAN_TX_PIN,
            (gpio_num_t)CAN_RX_PIN,
            TWAI_MODE_NORMAL
        );

    // 500 kbit/s
    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_500KBITS();

    // Accept all CAN IDs
    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install driver
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

    // Start CAN controller
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
    twai_message_t message = {};

    // Standard 11-bit CAN identifier
    message.identifier = 0x100;

    // Standard frame
    message.extd = 0;

    // Data frame
    message.rtr = 0;

    // Four data bytes
    message.data_length_code = 4;

    message.data[0] = 0x11;
    message.data[1] = 0x22;
    message.data[2] = 0x33;
    message.data[3] = 0x44;

    // Transmit
    esp_err_t result =
        twai_transmit(
            &message,
            pdMS_TO_TICKS(1000)
        );

    if (result == ESP_OK)
    {
        Serial.println("CAN message transmitted");
    }
    else
    {
        Serial.printf(
            "CAN transmit failed: %s\n",
            esp_err_to_name(result)
        );
    }

    delay(1000);
}