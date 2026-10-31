#include "telemetry.h"
#include "motor.h"
#include <math.h>

AsyncWebSocket ws("/ws");

static TelemetryPacket packet;

static void onWsEvent(
    AsyncWebSocket *server,
    AsyncWebSocketClient *client,
    AwsEventType type,
    void *arg,
    uint8_t *data,
    size_t len)
{
    switch (type)
    {
        case WS_EVT_CONNECT:
            Serial.printf("Client %u connected\n", client->id());
            break;

        case WS_EVT_DISCONNECT:
            Serial.printf("Client %u disconnected\n", client->id());
            break;

        case WS_EVT_DATA:
        {
            AwsFrameInfo *info = (AwsFrameInfo *)arg;
            // Single-frame text message only (fine for a short number string)
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
            {
                char text[32];
                if (len >= sizeof(text)) break;
                memcpy(text, data, len);
                text[len] = 0;

                // Anything that is not a number is ignored: atof() would read it
                // as 0 and send the motor to zero.
                char *end = nullptr;
                float newTarget = strtof(text, &end);
                if (end != text && *end == 0 && isfinite(newTarget))
                {
                    motor_set_target_angle(newTarget);
                }
            }
            break;
        }

        default:
            break;
    }
}

void telemetry_init(AsyncWebServer &server)
{
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    packet.board_id         = 1;
    packet.packet_version   = 3;
    packet.reserved         = 0;
    packet.sequence         = 0;
    packet.status_flags     = 0;
    packet.target_angle_deg = 0.0f;
}

void telemetry_broadcast(float encoderAngle, bool encoderValid, float targetAngle, bool motorMoving, uint8_t motorFlags)
{
    packet.sequence++;
    packet.timestamp_us     = micros();
    packet.angle_deg        = encoderAngle;
    packet.target_angle_deg = targetAngle;

    packet.status_flags = motorFlags;
    if (encoderValid) packet.status_flags |= 0x01;
    if (motorMoving)  packet.status_flags |= 0x02;

    // Skip a frame rather than queue up behind a slow client
    if (ws.count() > 0 && ws.availableForWriteAll())
    {
        ws.binaryAll((uint8_t *)&packet, sizeof(packet));
    }

    static uint8_t cleanupDivider = 0;
    if (++cleanupDivider >= 50)
    {
        cleanupDivider = 0;
        ws.cleanupClients();
    }
}
