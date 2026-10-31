#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>

#pragma pack(push, 1)
struct TelemetryPacket
{
    uint8_t  board_id;          // 0
    uint8_t  packet_version;    // 1
    uint16_t reserved;          // 2-3

    uint32_t sequence;          // 4-7
    uint32_t timestamp_us;      // 8-11

    float angle_deg;            // 12-15  current encoder angle
    float target_angle_deg;     // 16-19  commanded target angle

    uint8_t status_flags;       // 20     bit0 = encoder valid, bit1 = motor moving,
                                //        bit2 = stalled, bit3 = driver fault, bit4 = locked out
};
#pragma pack(pop)

// The websocket instance, shared so main.ino can attach it if needed.
extern AsyncWebSocket ws;

// Registers the websocket handler on the given server and initializes the
// packet. Call once from setup(), after server construction.
void telemetry_init(AsyncWebServer &server);

// Broadcasts one telemetry packet to all connected clients. Call at the
// telemetry cadence (see TX_PERIOD_US). motorFlags = motor_get_status_flags().
void telemetry_broadcast(float encoderAngle, bool encoderValid, float targetAngle, bool motorMoving, uint8_t motorFlags);
