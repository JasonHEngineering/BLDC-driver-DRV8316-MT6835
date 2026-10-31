// ============================================================
//  BLDC angular position control over WebSocket
//  MT6835 encoder + closed-loop GB2806 BLDC through a DRV8316
//  + WebSocket UI (live angle display + target-angle entry)
//  ESP32-C6
//
//  Arduino IDE: "esp32 by Espressif Systems" >= 3.0.0, board
//  "ESP32C6 Dev Module". Upload the data/ folder to LittleFS.
//
//  See config.h for pin assignments and tunable constants.
//  Type "help" in the Serial Monitor (115200) for live tuning.
// ============================================================

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <SPI.h>

#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "telemetry.h"

AsyncWebServer server(80);

uint32_t lastTransmit = 0;
uint32_t lastPrint    = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nBLDC WebSocket position control - ESP32-C6 / DRV8316 / MT6835");

    // The SN65HVD230 stays on the board: keep its driver recessive
    pinMode(PIN_CAN_TX, OUTPUT);  digitalWrite(PIN_CAN_TX, HIGH);

    pinMode(PIN_DRV_CS, OUTPUT);  digitalWrite(PIN_DRV_CS, HIGH);
    pinMode(PIN_ENC_CS, OUTPUT);  digitalWrite(PIN_ENC_CS, HIGH);
    pinMode(PIN_NFAULT, INPUT_PULLUP);          // GPIO15 is a strapping pin on the C6
    SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

    encoder_self_test();

    // The motor is up and holding before WiFi starts, so nothing here depends on
    // the network. A missing page is reported but does not stop the motor.
    if (!LittleFS.begin())
    {
        Serial.println("LittleFS Mount Failed - upload the data folder");
    }

    motor_start(motor_init());

    WiFi.persistent(false);     // no flash writes while the control loop runs
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);       // modem sleep would add 100 ms+ to every command
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println(WiFi.localIP());

    if (MDNS.begin("JH_Engineering"))
    {
        Serial.println("mDNS started");
    }

    telemetry_init(server);

    server.serveStatic("/", LittleFS, "/")
        .setDefaultFile("index.html");

    server.begin();

    lastTransmit = micros();

    Serial.println("Ready. Type \"help\" for live tuning commands.");
}

void loop()
{
    uint32_t now = micros();

    // --- Telemetry broadcast. The control itself runs in its own task (motor.cpp) ---
    if (now - lastTransmit >= TX_PERIOD_US)
    {
        lastTransmit = now;

        telemetry_broadcast(
            motor_get_angle_deg(),
            motor_encoder_is_valid(),
            motor_get_target_angle_deg(),
            motor_is_moving(),
            motor_get_status_flags());
    }

    motor_handle_serial();
    motor_report_events();

    if (now - lastPrint >= 1000000)
    {
        lastPrint = now;
        motor_print_status();
    }

    delay(2);
}
