/**
 * ============================================================
 *  SEMI-AUTONOMOUS FLOOR SCRUBBING ROBOT
 *  Arduino UNO R3 — SLAVE CONTROLLER
 * ============================================================
 *  Responsibilities:
 *    - Liquid Cleaning Control (S3003 Servo x2 → Spray Nozzles)
 *    - Mop/Scrubber Motor Control (DC Motor via L298N)
 *    - Vacuum Motor Control (DC Motor via relay or L298N)
 *    - Auxiliary sensor feedback
 *    - I2C Slave — receives commands from Arduino Mega (Master)
 *
 *  I2C Address : 0x08
 *  Target      : Arduino UNO R3
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// ============================================================
//  I2C SLAVE ADDRESS  (must match Master's SLAVE_ADDRESS)
// ============================================================
#define SLAVE_ADDRESS        0x08

// ============================================================
//  SERVO PINS  (S3003 × 2 — Spray Nozzle Angle Control)
//  Each servo controls one spray nozzle valve/angle
// ============================================================
#define SERVO_SPRAY_LEFT_PIN   9
#define SERVO_SPRAY_RIGHT_PIN  10

// Servo angle positions
#define SPRAY_OPEN_ANGLE       90    // degrees — nozzle open
#define SPRAY_CLOSED_ANGLE     0     // degrees — nozzle closed

// ============================================================
//  MOP / SCRUBBER MOTOR PINS  (L298N — single channel)
// ============================================================
#define MOP_MOTOR_IN1          2
#define MOP_MOTOR_IN2          3
#define MOP_MOTOR_EN           5    // PWM
#define MOP_SPEED              200  // 0–255 PWM

// ============================================================
//  VACUUM MOTOR CONTROL
//  Using a relay module (HIGH = ON) or L298N
// ============================================================
#define VACUUM_RELAY_PIN       6    // Relay module IN pin (active HIGH)
// If using L298N for vacuum motor:
#define VACUUM_MOTOR_IN1       7
#define VACUUM_MOTOR_IN2       8
#define VACUUM_MOTOR_EN        11   // PWM
#define VACUUM_SPEED           255

// ============================================================
//  STATUS LED  (optional visual feedback)
// ============================================================
#define LED_IDLE_PIN           A0
#define LED_VACUUM_PIN         A1
#define LED_MOP_PIN            A2
#define LED_SPRAY_PIN          A3

// ============================================================
//  LIQUID LEVEL SENSOR (optional)
//  Monitors tank level — sends warning to master if low
// ============================================================
#define LIQUID_LEVEL_PIN       A4
#define LIQUID_LOW_THRESHOLD   200   // analog threshold (0–1023)

// ============================================================
//  COMMAND BYTE DEFINITIONS  (mirror of Master defines)
// ============================================================
#define CMD_ALL_OFF          0x00
#define CMD_VACUUM_ON        0x01
#define CMD_VACUUM_OFF       0x02
#define CMD_SPRAY_ON         0x03
#define CMD_SPRAY_OFF        0x04
#define CMD_MOP_ON           0x05
#define CMD_MOP_OFF          0x06
#define CMD_WET_CLEAN        0x07   // Spray + Mop
#define CMD_DRY_CLEAN        0x08   // Vacuum only
#define CMD_STATUS_REQUEST   0x09

// ============================================================
//  STATUS BYTE BITS  (returned on I2C status request)
//  Bit 0 = Vacuum active
//  Bit 1 = Mop active
//  Bit 2 = Spray active
//  Bit 3 = Liquid low warning
//  Bit 4 = Error flag
// ============================================================
#define STATUS_VACUUM_BIT    0
#define STATUS_MOP_BIT       1
#define STATUS_SPRAY_BIT     2
#define STATUS_LIQUID_BIT    3
#define STATUS_ERROR_BIT     4

// ============================================================
//  GLOBAL STATE
// ============================================================
Servo servoLeft;
Servo servoRight;

volatile uint8_t receivedCommand = CMD_ALL_OFF;
volatile bool    newCommandFlag  = false;

bool vacuumOn   = false;
bool mopOn      = false;
bool sprayOn    = false;
bool liquidLow  = false;

uint8_t statusByte = 0x00;

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
void initPins();
void initServos();
void initI2C();

// Actuator control
void activateVacuum();
void deactivateVacuum();
void activateMop();
void deactivateMop();
void activateSpray();
void deactivateSpray();
void activateWetCleaning();
void activateDryCleaning();
void deactivateAll();

// Motor helpers
void setMopMotor(bool on, uint8_t speed);
void setVacuumMotor(bool on);

// Servo helpers
void setSprayServos(bool open);

// I2C callbacks
void onI2CReceive(int numBytes);
void onI2CRequest();

// Status
void updateStatusByte();
void checkLiquidLevel();
void updateLEDs();

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(9600);
    initPins();
    initServos();
    initI2C();

    Serial.println(F("=== Slave (Arduino UNO R3) Initialised ==="));
    Serial.println(F("Awaiting commands from Master..."));

    deactivateAll();   // Safe starting state
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
    // Process received I2C command (set in ISR)
    if (newCommandFlag) {
        newCommandFlag = false;
        processCommand(receivedCommand);
    }

    // Periodic liquid level check
    static uint32_t lastLiquidCheck = 0;
    if (millis() - lastLiquidCheck >= 2000) {
        checkLiquidLevel();
        lastLiquidCheck = millis();
    }

    // Update status byte and LEDs
    updateStatusByte();
    updateLEDs();

    delay(20);
}

// ============================================================
//  INITIALISATION
// ============================================================
void initPins() {
    // Mop motor
    pinMode(MOP_MOTOR_IN1, OUTPUT);
    pinMode(MOP_MOTOR_IN2, OUTPUT);
    pinMode(MOP_MOTOR_EN,  OUTPUT);

    // Vacuum relay
    pinMode(VACUUM_RELAY_PIN, OUTPUT);
    digitalWrite(VACUUM_RELAY_PIN, LOW);   // Relay OFF at start

    // Vacuum motor (if using L298N)
    pinMode(VACUUM_MOTOR_IN1, OUTPUT);
    pinMode(VACUUM_MOTOR_IN2, OUTPUT);
    pinMode(VACUUM_MOTOR_EN,  OUTPUT);

    // Status LEDs
    pinMode(LED_IDLE_PIN,   OUTPUT);
    pinMode(LED_VACUUM_PIN, OUTPUT);
    pinMode(LED_MOP_PIN,    OUTPUT);
    pinMode(LED_SPRAY_PIN,  OUTPUT);

    // Liquid level (analog — INPUT by default)
    pinMode(LIQUID_LEVEL_PIN, INPUT);

    // Default all outputs LOW
    digitalWrite(MOP_MOTOR_IN1,   LOW);
    digitalWrite(MOP_MOTOR_IN2,   LOW);
    analogWrite(MOP_MOTOR_EN,     0);
    digitalWrite(VACUUM_MOTOR_IN1, LOW);
    digitalWrite(VACUUM_MOTOR_IN2, LOW);
    analogWrite(VACUUM_MOTOR_EN,   0);
}

void initServos() {
    servoLeft.attach(SERVO_SPRAY_LEFT_PIN);
    servoRight.attach(SERVO_SPRAY_RIGHT_PIN);
    servoLeft.write(SPRAY_CLOSED_ANGLE);    // Close nozzles at startup
    servoRight.write(SPRAY_CLOSED_ANGLE);
}

void initI2C() {
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(onI2CReceive);   // Register receive callback
    Wire.onRequest(onI2CRequest);   // Register request callback
}

// ============================================================
//  COMMAND PROCESSOR
// ============================================================
void processCommand(uint8_t cmd) {
    Serial.print(F("CMD received: 0x"));
    Serial.println(cmd, HEX);

    switch (cmd) {
        case CMD_ALL_OFF:      deactivateAll();         break;
        case CMD_VACUUM_ON:    activateVacuum();        break;
        case CMD_VACUUM_OFF:   deactivateVacuum();      break;
        case CMD_SPRAY_ON:     activateSpray();         break;
        case CMD_SPRAY_OFF:    deactivateSpray();       break;
        case CMD_MOP_ON:       activateMop();           break;
        case CMD_MOP_OFF:      deactivateMop();         break;
        case CMD_WET_CLEAN:    activateWetCleaning();   break;
        case CMD_DRY_CLEAN:    activateDryCleaning();   break;
        case CMD_STATUS_REQUEST: /* handled in onI2CRequest */ break;
        default:
            Serial.println(F("Unknown command — ignoring"));
            break;
    }
}

// ============================================================
//  ACTUATOR CONTROL
// ============================================================

/** Activate vacuum motor only (Dry Cleaning Mode) */
void activateVacuum() {
    if (!vacuumOn) {
        setVacuumMotor(true);
        vacuumOn = true;
        Serial.println(F("Vacuum: ON"));
    }
}

/** Deactivate vacuum motor */
void deactivateVacuum() {
    if (vacuumOn) {
        setVacuumMotor(false);
        vacuumOn = false;
        Serial.println(F("Vacuum: OFF"));
    }
}

/** Activate rotating mop/scrubber motor */
void activateMop() {
    if (!mopOn) {
        setMopMotor(true, MOP_SPEED);
        mopOn = true;
        Serial.println(F("Mop: ON"));
    }
}

/** Deactivate mop motor */
void deactivateMop() {
    if (mopOn) {
        setMopMotor(false, 0);
        mopOn = false;
        Serial.println(F("Mop: OFF"));
    }
}

/** Open spray nozzle servos → spray cleaning solution */
void activateSpray() {
    if (!sprayOn) {
        // Check liquid level first
        if (liquidLow) {
            Serial.println(F("WARNING: Liquid tank LOW — spray aborted"));
            return;
        }
        setSprayServos(true);   // Open nozzles
        sprayOn = true;
        Serial.println(F("Spray: ON"));
    }
}

/** Close spray nozzle servos */
void deactivateSpray() {
    if (sprayOn) {
        setSprayServos(false);  // Close nozzles
        sprayOn = false;
        Serial.println(F("Spray: OFF"));
    }
}

/** Wet/Deep Cleaning: Spray nozzles + Mop motor */
void activateWetCleaning() {
    Serial.println(F("WET CLEANING mode: Spray + Mop"));
    deactivateVacuum();   // Vacuum off in wet mode
    activateSpray();
    delay(300);           // Allow spray to saturate surface
    activateMop();
}

/** Dry Cleaning: Vacuum only */
void activateDryCleaning() {
    Serial.println(F("DRY CLEANING mode: Vacuum only"));
    deactivateSpray();
    deactivateMop();
    activateVacuum();
}

/** Turn off all actuators */
void deactivateAll() {
    deactivateVacuum();
    deactivateSpray();
    deactivateMop();
    Serial.println(F("All actuators: OFF"));
}

// ============================================================
//  LOW-LEVEL MOTOR HELPERS
// ============================================================

/**
 * Controls the mop/scrubber DC motor.
 * Rotates in one direction (no reverse needed for scrubbing).
 */
void setMopMotor(bool on, uint8_t speed) {
    if (on) {
        digitalWrite(MOP_MOTOR_IN1, HIGH);
        digitalWrite(MOP_MOTOR_IN2, LOW);
        analogWrite(MOP_MOTOR_EN, speed);
    } else {
        digitalWrite(MOP_MOTOR_IN1, LOW);
        digitalWrite(MOP_MOTOR_IN2, LOW);
        analogWrite(MOP_MOTOR_EN, 0);
    }
}

/**
 * Controls the vacuum motor via relay module.
 * Also drives through L298N as fallback if uncommented.
 */
void setVacuumMotor(bool on) {
    // --- Method 1: Relay module (simplest, recommended for high-current vacuum) ---
    digitalWrite(VACUUM_RELAY_PIN, on ? HIGH : LOW);

    // --- Method 2: L298N motor driver (uncomment if using L298N) ---
    // if (on) {
    //     digitalWrite(VACUUM_MOTOR_IN1, HIGH);
    //     digitalWrite(VACUUM_MOTOR_IN2, LOW);
    //     analogWrite(VACUUM_MOTOR_EN, VACUUM_SPEED);
    // } else {
    //     digitalWrite(VACUUM_MOTOR_IN1, LOW);
    //     digitalWrite(VACUUM_MOTOR_IN2, LOW);
    //     analogWrite(VACUUM_MOTOR_EN, 0);
    // }
}

/**
 * Controls spray nozzle servos (S3003 x2).
 * openNozzle = true  → rotate servo to SPRAY_OPEN_ANGLE
 * openNozzle = false → rotate servo to SPRAY_CLOSED_ANGLE
 */
void setSprayServos(bool openNozzle) {
    int angle = openNozzle ? SPRAY_OPEN_ANGLE : SPRAY_CLOSED_ANGLE;
    servoLeft.write(angle);
    servoRight.write(angle);
}

// ============================================================
//  I2C CALLBACKS
// ============================================================

/** Called when Master sends data via I2C */
void onI2CReceive(int numBytes) {
    while (Wire.available()) {
        receivedCommand = Wire.read();
    }
    newCommandFlag = true;
}

/** Called when Master requests data via I2C */
void onI2CRequest() {
    updateStatusByte();
    Wire.write(statusByte);
}

// ============================================================
//  STATUS & MONITORING
// ============================================================

/** Build status byte from current actuator states */
void updateStatusByte() {
    statusByte = 0x00;
    if (vacuumOn)  statusByte |= (1 << STATUS_VACUUM_BIT);
    if (mopOn)     statusByte |= (1 << STATUS_MOP_BIT);
    if (sprayOn)   statusByte |= (1 << STATUS_SPRAY_BIT);
    if (liquidLow) statusByte |= (1 << STATUS_LIQUID_BIT);
}

/** Read liquid level analog sensor and set liquidLow flag */
void checkLiquidLevel() {
    int level = analogRead(LIQUID_LEVEL_PIN);
    liquidLow = (level < LIQUID_LOW_THRESHOLD);
    if (liquidLow) {
        Serial.println(F("ALERT: Cleaning liquid tank is LOW!"));
        // Deactivate spray automatically if liquid is empty
        if (sprayOn) {
            deactivateSpray();
        }
    }
}

/** Visual feedback via status LEDs */
void updateLEDs() {
    digitalWrite(LED_IDLE_PIN,   (!vacuumOn && !mopOn && !sprayOn) ? HIGH : LOW);
    digitalWrite(LED_VACUUM_PIN, vacuumOn  ? HIGH : LOW);
    digitalWrite(LED_MOP_PIN,    mopOn     ? HIGH : LOW);
    digitalWrite(LED_SPRAY_PIN,  sprayOn   ? HIGH : LOW);
}
