#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>


// ============================================================
// AS5048A SPI
// ESP32-S3-DevKitC-1
// ============================================================

#define SPI_CS      21
#define SPI_MISO    12
#define SPI_SCLK    13
#define SPI_MOSI    11


// ============================================================
// DRV8313
// ============================================================

#define PWM_A_PIN   15
#define PWM_B_PIN   16
#define PWM_C_PIN   17
#define DRV_ENABLE  18


// ============================================================
// MOTOR
// ============================================================

// GB2806
// 14 poles = 7 pole pairs
#define POLE_PAIRS  7


// ============================================================
// POWER
// ============================================================

// DRV8313 supply voltage
#define MOTOR_VOLTAGE  8.0f


// FOC voltage limit
//
// Start conservatively.
// Increase later if necessary.
#define MOTOR_VOLTAGE_LIMIT  1.0f


// ============================================================
// POSITION TEST
// ============================================================

// One complete mechanical revolution
#define MOVE_ANGLE  _2PI


// How long to hold each position
#define HOLD_TIME_MS  4000


// Maximum commanded velocity
#define MAX_VELOCITY  12.0f


// ============================================================
// OBJECTS
// ============================================================


// ------------------------------------------------------------
// AS5048A
// ------------------------------------------------------------

MagneticSensorSPI sensor = MagneticSensorSPI(
    SPI_CS,
    14,
    0x3FFF,
    1000000
);


// ------------------------------------------------------------
// DRV8313
// ------------------------------------------------------------

BLDCDriver3PWM driver = BLDCDriver3PWM(
    PWM_A_PIN,
    PWM_B_PIN,
    PWM_C_PIN,
    DRV_ENABLE
);


// ------------------------------------------------------------
// Motor
// ------------------------------------------------------------

BLDCMotor motor = BLDCMotor(POLE_PAIRS);


// ============================================================
// TEST STATE
// ============================================================

enum TestState
{
    MOVE_FORWARD,
    HOLD_FORWARD,
    MOVE_BACKWARD,
    HOLD_BACKWARD
};

TestState state = MOVE_FORWARD;


// ============================================================
// POSITION VARIABLES
// ============================================================

float start_angle = 0.0f;

float forward_target = 0.0f;

float backward_target = 0.0f;

unsigned long stateStartTime = 0;


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("==============================================");
    Serial.println(" GB2806 + DRV8313 + AS5048A");
    Serial.println(" Closed-loop position FOC test");
    Serial.println("==============================================");
    Serial.println();


    // --------------------------------------------------------
    // SimpleFOC debugging
    // --------------------------------------------------------

    SimpleFOCDebug::enable(&Serial);


    // --------------------------------------------------------
    // SPI
    // --------------------------------------------------------

    Serial.println("Initializing SPI...");

    SPI.begin(
        SPI_SCLK,
        SPI_MISO,
        SPI_MOSI,
        SPI_CS
    );

    pinMode(SPI_CS, OUTPUT);
    digitalWrite(SPI_CS, HIGH);

    Serial.println("SPI initialized.");
    Serial.println();


    // --------------------------------------------------------
    // AS5048A
    // --------------------------------------------------------

    Serial.println("Initializing AS5048A...");

    sensor.spi_mode = SPI_MODE1;
    sensor.clock_speed = 1000000;

    sensor.init(&SPI);

    delay(100);

    sensor.update();

    Serial.print("Initial sensor angle: ");
    Serial.print(sensor.getAngle(), 4);
    Serial.println(" rad");

    Serial.println();


    // --------------------------------------------------------
    // DRV8313
    // --------------------------------------------------------

    Serial.println("Initializing DRV8313...");

    driver.voltage_power_supply = MOTOR_VOLTAGE;

    driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;

    driver.pwm_frequency = 20000;


    if (!driver.init())
    {
        Serial.println();
        Serial.println("ERROR: DRV8313 initialization FAILED!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("DRV8313 initialization successful.");
    Serial.println();


    // --------------------------------------------------------
    // Link sensor and driver
    // --------------------------------------------------------

    motor.linkSensor(&sensor);

    motor.linkDriver(&driver);


    // --------------------------------------------------------
    // Position control
    // --------------------------------------------------------

    motor.controller = MotionControlType::angle;


    // --------------------------------------------------------
    // Voltage limit
    // --------------------------------------------------------

    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;


    // --------------------------------------------------------
    // Maximum velocity
    // --------------------------------------------------------

    motor.velocity_limit = MAX_VELOCITY;


    // --------------------------------------------------------
    // Velocity PID
    //
    // The angle controller internally uses velocity control.
    // --------------------------------------------------------

    motor.PID_velocity.P = 0.05f;

    motor.PID_velocity.I = 2.0f;

    motor.PID_velocity.D = 0.0f;


    // --------------------------------------------------------
    // Velocity filtering
    // --------------------------------------------------------

    motor.LPF_velocity.Tf = 0.01f;


    // --------------------------------------------------------
    // Position P controller
    // --------------------------------------------------------

    motor.P_angle.P = 3.0f;


    // --------------------------------------------------------
    // Motor initialization
    // --------------------------------------------------------

    Serial.println("Initializing motor...");

    if (!motor.init())
    {
        Serial.println();
        Serial.println("ERROR: Motor initialization FAILED!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("Motor initialization successful.");
    Serial.println();


    // --------------------------------------------------------
    // FOC alignment
    // --------------------------------------------------------

    Serial.println("----------------------------------------------");
    Serial.println("Starting FOC alignment.");
    Serial.println("Motor may move during alignment.");
    Serial.println("----------------------------------------------");

    if (!motor.initFOC())
    {
        Serial.println();
        Serial.println("ERROR: FOC initialization FAILED!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println();
    Serial.println("----------------------------------------------");
    Serial.println("FOC initialization SUCCESSFUL");
    Serial.println("----------------------------------------------");
    Serial.println();


    // --------------------------------------------------------
    // Establish starting position
    // --------------------------------------------------------

    sensor.update();

    start_angle = sensor.getAngle();

    forward_target = start_angle + MOVE_ANGLE;

    backward_target = start_angle;


    Serial.print("Starting angle: ");
    Serial.print(start_angle, 4);

    Serial.println(" rad");

    Serial.print("Forward target: ");
    Serial.print(forward_target, 4);

    Serial.println(" rad");

    Serial.print("Backward target: ");
    Serial.print(backward_target, 4);

    Serial.println(" rad");

    Serial.println();


    // --------------------------------------------------------
    // Start test
    // --------------------------------------------------------

    state = MOVE_FORWARD;

    stateStartTime = millis();

    Serial.println("Moving FORWARD...");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // FOC
    // ========================================================

    motor.loopFOC();


    // ========================================================
    // TEST STATE MACHINE
    // ========================================================

    switch (state)
    {
        // ----------------------------------------------------
        // MOVE FORWARD
        // ----------------------------------------------------

        case MOVE_FORWARD:

            motor.target = forward_target;

            motor.move();


            // Check whether target is reached
            if (fabs(motor.shaft_angle - forward_target) < 0.05f)
            {
                state = HOLD_FORWARD;

                stateStartTime = millis();

                Serial.println();
                Serial.println("FORWARD position reached.");
                Serial.println("Holding...");
            }

            break;


        // ----------------------------------------------------
        // HOLD FORWARD
        // ----------------------------------------------------

        case HOLD_FORWARD:

            motor.target = forward_target;

            motor.move();


            if (millis() - stateStartTime >= HOLD_TIME_MS)
            {
                state = MOVE_BACKWARD;

                Serial.println();
                Serial.println("Moving BACKWARD...");
            }

            break;


        // ----------------------------------------------------
        // MOVE BACKWARD
        // ----------------------------------------------------

        case MOVE_BACKWARD:

            motor.target = backward_target;

            motor.move();


            if (fabs(motor.shaft_angle - backward_target) < 0.05f)
            {
                state = HOLD_BACKWARD;

                stateStartTime = millis();

                Serial.println();
                Serial.println("BACKWARD position reached.");
                Serial.println("Holding...");
            }

            break;


        // ----------------------------------------------------
        // HOLD BACKWARD
        // ----------------------------------------------------

        case HOLD_BACKWARD:

            motor.target = backward_target;

            motor.move();


            if (millis() - stateStartTime >= HOLD_TIME_MS)
            {
                state = MOVE_FORWARD;

                Serial.println();
                Serial.println("Moving FORWARD...");
            }

            break;
    }


    // ========================================================
    // SERIAL MONITOR
    // ========================================================

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= 250)
    {
        lastPrint = millis();


        Serial.print("Sensor: ");
        Serial.print(sensor.getAngle(), 4);

        Serial.print(" rad | Shaft: ");
        Serial.print(motor.shaft_angle, 4);

        Serial.print(" rad | Target: ");
        Serial.print(motor.target, 4);

        Serial.print(" rad | Error: ");
        Serial.print(
            motor.target - motor.shaft_angle,
            4
        );

        Serial.print(" rad | Velocity: ");
        Serial.print(
            motor.shaft_velocity,
            4
        );

        Serial.println(" rad/s");
    }
}