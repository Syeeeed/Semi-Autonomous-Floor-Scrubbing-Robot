/**
 * ============================================================
 *  SEMI-AUTONOMOUS FLOOR SCRUBBING ROBOT
 *  Arduino Mega 2560 — MASTER CONTROLLER
 * ============================================================
 *  Responsibilities:
 *    - Navigation (4x DC Drive Motors via L298N drivers)
 *    - Environment Scan (Ultrasonic + IR Sensors)
 *    - Surface Analysis & Decision Making
 *    - I2C/Serial communication with Arduino UNO R3 (Slave)
 *    - Serial communication with ESP32 CAM (Wi-Fi bridge)
 *
 *  Cleaning Logic:
 *    DUST detected       → Vacuum ON
 *    STICKY/HARD DIRT    → Spray + Mop ON
 *    NOTHING (SAFE)      → Continue Navigation
 *
 *  Author  : Semi-Autonomous Robot Team
 *  Target  : Arduino Mega 2560
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <NewPing.h>   // Library for HC-SR04 ultrasonic sensor

// ============================================================
//  SLAVE ADDRESS (I2C to Arduino UNO R3)
// ============================================================
#define SLAVE_ADDRESS        0x08

// ============================================================
//  SERIAL PORT DEFINITIONS
//  Serial  (USB)     → Debug Monitor
//  Serial1 (pins 18/19) → ESP32 CAM communication
// ============================================================
#define ESP32_SERIAL         Serial1
#define ESP32_BAUD           115200
#define DEBUG_BAUD           9600

// ============================================================
//  ULTRASONIC SENSOR PINS  (HC-SR04)
//  Front, Left, Right sensors for obstacle avoidance
// ============================================================
#define US_FRONT_TRIG        22
#define US_FRONT_ECHO        23
#define US_LEFT_TRIG         24
#define US_LEFT_ECHO         25
#define US_RIGHT_TRIG        26
#define US_RIGHT_ECHO        27
#define US_MAX_DISTANCE      200   // cm
#define OBSTACLE_THRESHOLD   20    // cm — stop distance

// ============================================================
//  IR SENSOR PINS (surface dirt/dust detection)
//  IR1 = primary dirt sensor (centre, analog)
//  IR2 = secondary confirmation sensor (analog)
// ============================================================
#define IR_SENSOR_1          A0
#define IR_SENSOR_2          A1
#define IR_DUST_THRESHOLD    400   // analog value (0–1023)
#define IR_DIRT_THRESHOLD    600   // sticky/hard dirt — higher reflectance

// ============================================================
//  MOTOR DRIVER PINS  (L298N — 4 Drive Wheels)
//  Motor A = Left Front
//  Motor B = Left Rear
//  Motor C = Right Front
//  Motor D = Right Rear
// ============================================================
// Motor A — Left Front
#define MOTOR_A_IN1          2
#define MOTOR_A_IN2          3
#define MOTOR_A_EN           4    // PWM

// Motor B — Left Rear
#define MOTOR_B_IN1          5
#define MOTOR_B_IN2          6
#define MOTOR_B_EN           7    // PWM (use hardware PWM pin on Mega)

// Motor C — Right Front
#define MOTOR_C_IN1          8
#define MOTOR_C_IN2          9
#define MOTOR_C_EN           10   // PWM

// Motor D — Right Rear
#define MOTOR_D_IN1          11
#define MOTOR_D_IN2          12
#define MOTOR_D_EN           13   // PWM

#define MOTOR_SPEED_NORMAL   180  // 0–255 PWM
#define MOTOR_SPEED_SLOW     100
#define MOTOR_SPEED_STOP     0

// ============================================================
//  SLAVE COMMAND BYTES (sent via I2C to UNO R3)
// ============================================================
#define CMD_ALL_OFF          0x00
#define CMD_VACUUM_ON        0x01
#define CMD_VACUUM_OFF       0x02
#define CMD_SPRAY_ON         0x03
#define CMD_SPRAY_OFF        0x04
#define CMD_MOP_ON           0x05
#define CMD_MOP_OFF          0x06
#define CMD_WET_CLEAN        0x07   // Spray + Mop together
#define CMD_DRY_CLEAN        0x08   // Vacuum only
#define CMD_STATUS_REQUEST   0x09

// ============================================================
//  SYSTEM STATE MACHINE
// ============================================================
enum RobotState {
    STATE_IDLE,
    STATE_SCANNING,
    STATE_DRY_CLEANING,    // Dust detected  → Vacuum
    STATE_WET_CLEANING,    // Dirt detected  → Spray + Mop
    STATE_NAVIGATING,      // Clear path
    STATE_OBSTACLE_AVOID,
    STATE_RC_MANUAL,       // Manual override from RC App
    STATE_ERROR
};

// ============================================================
//  SURFACE DETECTION RESULT
// ============================================================
enum SurfaceType {
    SURFACE_CLEAN,
    SURFACE_DUST,
    SURFACE_STICKY_DIRT
};

// ============================================================
//  RC COMMAND STRUCTURE  (received from ESP32)
// ============================================================
enum RCCommand {
    RC_NONE,
    RC_FORWARD,
    RC_BACKWARD,
    RC_LEFT,
    RC_RIGHT,
    RC_STOP,
    RC_AUTO_MODE,
    RC_MANUAL_MODE,
    RC_VACUUM_TOGGLE,
    RC_MOP_TOGGLE,
    RC_SPRAY_TOGGLE
};

// ============================================================
//  GLOBAL VARIABLES
// ============================================================
RobotState  currentState      = STATE_IDLE;
RobotState  previousState     = STATE_IDLE;
SurfaceType detectedSurface   = SURFACE_CLEAN;
RCCommand   lastRCCommand     = RC_NONE;

bool autoMode       = true;   // false = manual RC control
bool vacuumActive   = false;
bool mopActive      = false;
bool sprayActive    = false;
bool obstacleDetected = false;

uint8_t  slaveStatus         = 0x00;
uint32_t lastScanTime        = 0;
uint32_t lastNavUpdateTime   = 0;
uint32_t cleaningDuration    = 0;
uint32_t stateEnterTime      = 0;

const uint32_t SCAN_INTERVAL          = 500;   // ms
const uint32_t NAV_UPDATE_INTERVAL    = 100;   // ms
const uint32_t CLEANING_TIMEOUT       = 5000;  // ms — max clean time per spot
const uint32_t OBSTACLE_STOP_DELAY    = 300;   // ms

// NewPing objects for each ultrasonic sensor
NewPing sonarFront(US_FRONT_TRIG, US_FRONT_ECHO, US_MAX_DISTANCE);
NewPing sonarLeft (US_LEFT_TRIG,  US_LEFT_ECHO,  US_MAX_DISTANCE);
NewPing sonarRight(US_RIGHT_TRIG, US_RIGHT_ECHO, US_MAX_DISTANCE);

// ============================================================
//  FUNCTION PROTOTYPES
// ============================================================
void     initPins();
void     initSerial();
void     initI2C();

// Motor Control
void     motorForward(uint8_t speed);
void     motorBackward(uint8_t speed);
void     motorTurnLeft(uint8_t speed);
void     motorTurnRight(uint8_t speed);
void     motorStop();
void     setMotorA(bool forward, uint8_t speed);
void     setMotorB(bool forward, uint8_t speed);
void     setMotorC(bool forward, uint8_t speed);
void     setMotorD(bool forward, uint8_t speed);

// Sensor Reading
uint16_t readUltrasonicFront();
uint16_t readUltrasonicLeft();
uint16_t readUltrasonicRight();
SurfaceType scanSurface();
bool     isObstacleAhead();

// State Machine
void     updateStateMachine();
void     enterState(RobotState newState);
void     handleIdleState();
void     handleScanningState();
void     handleDryCleaningState();
void     handleWetCleaningState();
void     handleNavigatingState();
void     handleObstacleAvoidState();
void     handleRCManualState();

// Slave Communication (I2C)
void     sendSlaveCommand(uint8_t cmd);
uint8_t  requestSlaveStatus();

// ESP32 Communication
void     processESP32Data();
RCCommand parseRCCommand(const String& msg);
void     sendStatusToESP32();

// Utilities
void     debugPrint(const String& msg);
void     debugPrintState(RobotState s);
String   stateToString(RobotState s);

// ============================================================
//  SETUP
// ============================================================
void setup() {
    initSerial();
    initPins();
    initI2C();

    delay(1000);  // Power-on stabilise

    debugPrint(F("=== Semi-Autonomous Floor Scrubbing Robot ==="));
    debugPrint(F("Master (Arduino Mega) — Initialised"));
    debugPrint(F("Entering IDLE state..."));

    enterState(STATE_IDLE);
    delay(500);
    enterState(STATE_SCANNING);   // Begin scanning on startup
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
    // 1. Read incoming RC commands from ESP32
    processESP32Data();

    // 2. Check for obstacles (always active)
    obstacleDetected = isObstacleAhead();

    // 3. Run state machine
    updateStateMachine();

    // 4. Periodically report status to ESP32
    static uint32_t lastStatusTime = 0;
    if (millis() - lastStatusTime >= 1000) {
        sendStatusToESP32();
        lastStatusTime = millis();
    }

    delay(10);  // Small loop delay
}

// ============================================================
//  INITIALISATION
// ============================================================
void initSerial() {
    Serial.begin(DEBUG_BAUD);
    ESP32_SERIAL.begin(ESP32_BAUD);
}

void initPins() {
    // Motor A
    pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_A_EN,  OUTPUT);
    // Motor B
    pinMode(MOTOR_B_IN1, OUTPUT); pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(MOTOR_B_EN,  OUTPUT);
    // Motor C
    pinMode(MOTOR_C_IN1, OUTPUT); pinMode(MOTOR_C_IN2, OUTPUT);
    pinMode(MOTOR_C_EN,  OUTPUT);
    // Motor D
    pinMode(MOTOR_D_IN1, OUTPUT); pinMode(MOTOR_D_IN2, OUTPUT);
    pinMode(MOTOR_D_EN,  OUTPUT);

    // IR Sensors (analog — no pinMode needed, but set to INPUT for clarity)
    pinMode(IR_SENSOR_1, INPUT);
    pinMode(IR_SENSOR_2, INPUT);

    // Stop all motors on init
    motorStop();
}

void initI2C() {
    Wire.begin();   // Master mode
    debugPrint(F("I2C Master initialised"));
}

// ============================================================
//  STATE MACHINE
// ============================================================
void updateStateMachine() {
    switch (currentState) {
        case STATE_IDLE:            handleIdleState();          break;
        case STATE_SCANNING:        handleScanningState();      break;
        case STATE_DRY_CLEANING:    handleDryCleaningState();   break;
        case STATE_WET_CLEANING:    handleWetCleaningState();   break;
        case STATE_NAVIGATING:      handleNavigatingState();    break;
        case STATE_OBSTACLE_AVOID:  handleObstacleAvoidState(); break;
        case STATE_RC_MANUAL:       handleRCManualState();      break;
        default:                    enterState(STATE_IDLE);     break;
    }
}

void enterState(RobotState newState) {
    if (newState != currentState) {
        previousState = currentState;
        currentState  = newState;
        stateEnterTime = millis();
        debugPrint("State: " + stateToString(previousState) +
                   " → " + stateToString(currentState));
    }
}

// -------  IDLE  -------
void handleIdleState() {
    motorStop();
    sendSlaveCommand(CMD_ALL_OFF);
    // Transition to scanning after 1 second
    if (millis() - stateEnterTime > 1000) {
        enterState(STATE_SCANNING);
    }
}

// -------  SCANNING  -------
void handleScanningState() {
    motorStop();   // Pause to scan

    if (millis() - lastScanTime >= SCAN_INTERVAL) {
        lastScanTime = millis();

        detectedSurface = scanSurface();

        switch (detectedSurface) {
            case SURFACE_DUST:
                debugPrint(F(">> DUST detected → Dry Cleaning Mode"));
                enterState(STATE_DRY_CLEANING);
                break;
            case SURFACE_STICKY_DIRT:
                debugPrint(F(">> STICKY/HARD DIRT detected → Wet Cleaning Mode"));
                enterState(STATE_WET_CLEANING);
                break;
            case SURFACE_CLEAN:
                debugPrint(F(">> Surface CLEAN → Navigating"));
                enterState(STATE_NAVIGATING);
                break;
        }
    }
}

// -------  DRY CLEANING  (Vacuum ON)  -------
void handleDryCleaningState() {
    if (!vacuumActive) {
        sendSlaveCommand(CMD_VACUUM_ON);
        vacuumActive = true;
        mopActive    = false;
        sprayActive  = false;
        debugPrint(F("Vacuum ACTIVATED"));
    }

    // Move slowly while vacuuming
    if (!obstacleDetected) {
        motorForward(MOTOR_SPEED_SLOW);
    } else {
        motorStop();
    }

    // Re-scan after cleaning timeout
    if (millis() - stateEnterTime >= CLEANING_TIMEOUT) {
        sendSlaveCommand(CMD_VACUUM_OFF);
        vacuumActive = false;
        debugPrint(F("Vacuum OFF — re-scanning"));
        enterState(STATE_SCANNING);
    }
}

// -------  WET CLEANING  (Spray + Mop ON)  -------
void handleWetCleaningState() {
    if (!sprayActive || !mopActive) {
        sendSlaveCommand(CMD_WET_CLEAN);
        sprayActive  = true;
        mopActive    = true;
        vacuumActive = false;
        debugPrint(F("Spray + Mop ACTIVATED"));
    }

    // Move slowly while scrubbing
    if (!obstacleDetected) {
        motorForward(MOTOR_SPEED_SLOW);
    } else {
        motorStop();
    }

    // Re-scan after cleaning timeout
    if (millis() - stateEnterTime >= CLEANING_TIMEOUT) {
        sendSlaveCommand(CMD_SPRAY_OFF);
        sendSlaveCommand(CMD_MOP_OFF);
        sprayActive = false;
        mopActive   = false;
        debugPrint(F("Spray + Mop OFF — re-scanning"));
        enterState(STATE_SCANNING);
    }
}

// -------  NAVIGATING  -------
void handleNavigatingState() {
    // Turn off all cleaning tools
    if (vacuumActive || mopActive || sprayActive) {
        sendSlaveCommand(CMD_ALL_OFF);
        vacuumActive = false;
        mopActive    = false;
        sprayActive  = false;
    }

    if (obstacleDetected) {
        enterState(STATE_OBSTACLE_AVOID);
        return;
    }

    motorForward(MOTOR_SPEED_NORMAL);

    // Periodically scan while navigating
    if (millis() - lastScanTime >= SCAN_INTERVAL * 2) {
        lastScanTime = millis();
        enterState(STATE_SCANNING);
    }
}

// -------  OBSTACLE AVOIDANCE  -------
void handleObstacleAvoidState() {
    motorStop();
    delay(OBSTACLE_STOP_DELAY);

    uint16_t distLeft  = readUltrasonicLeft();
    uint16_t distRight = readUltrasonicRight();

    debugPrint("Obstacle! Left=" + String(distLeft) +
               "cm  Right=" + String(distRight) + "cm");

    // Back up slightly
    motorBackward(MOTOR_SPEED_SLOW);
    delay(400);
    motorStop();

    // Turn toward more space
    if (distLeft > distRight) {
        debugPrint(F("Turning LEFT to avoid obstacle"));
        motorTurnLeft(MOTOR_SPEED_NORMAL);
    } else {
        debugPrint(F("Turning RIGHT to avoid obstacle"));
        motorTurnRight(MOTOR_SPEED_NORMAL);
    }
    delay(600);
    motorStop();

    enterState(STATE_SCANNING);
}

// -------  RC MANUAL MODE  -------
void handleRCManualState() {
    switch (lastRCCommand) {
        case RC_FORWARD:         motorForward(MOTOR_SPEED_NORMAL);  break;
        case RC_BACKWARD:        motorBackward(MOTOR_SPEED_NORMAL); break;
        case RC_LEFT:            motorTurnLeft(MOTOR_SPEED_NORMAL); break;
        case RC_RIGHT:           motorTurnRight(MOTOR_SPEED_NORMAL);break;
        case RC_STOP:            motorStop();                        break;
        case RC_AUTO_MODE:
            autoMode = true;
            debugPrint(F("Switching to AUTO mode"));
            enterState(STATE_SCANNING);
            return;
        case RC_VACUUM_TOGGLE:
            vacuumActive = !vacuumActive;
            sendSlaveCommand(vacuumActive ? CMD_VACUUM_ON : CMD_VACUUM_OFF);
            break;
        case RC_MOP_TOGGLE:
            mopActive = !mopActive;
            sendSlaveCommand(mopActive ? CMD_MOP_ON : CMD_MOP_OFF);
            break;
        case RC_SPRAY_TOGGLE:
            sprayActive = !sprayActive;
            sendSlaveCommand(sprayActive ? CMD_SPRAY_ON : CMD_SPRAY_OFF);
            break;
        default: break;
    }
    lastRCCommand = RC_NONE;
}

// ============================================================
//  SENSOR FUNCTIONS
// ============================================================
uint16_t readUltrasonicFront() {
    uint16_t d = sonarFront.ping_cm();
    return (d == 0) ? US_MAX_DISTANCE : d;
}

uint16_t readUltrasonicLeft() {
    uint16_t d = sonarLeft.ping_cm();
    return (d == 0) ? US_MAX_DISTANCE : d;
}

uint16_t readUltrasonicRight() {
    uint16_t d = sonarRight.ping_cm();
    return (d == 0) ? US_MAX_DISTANCE : d;
}

bool isObstacleAhead() {
    uint16_t dist = readUltrasonicFront();
    return (dist < OBSTACLE_THRESHOLD);
}

/**
 * Reads both IR sensors and classifies the surface type.
 *
 * IR sensor output interpretation:
 *  Low analog value  → high reflectance → lighter surface (may be dust)
 *  Medium value      → normal surface (clean)
 *  High value        → dark/wet/dirty surface (sticky dirt)
 *
 * Both sensors must agree to reduce false positives.
 */
SurfaceType scanSurface() {
    int ir1 = analogRead(IR_SENSOR_1);
    int ir2 = analogRead(IR_SENSOR_2);
    int avg  = (ir1 + ir2) / 2;

    debugPrint("IR avg=" + String(avg) +
               "  IR1=" + String(ir1) +
               "  IR2=" + String(ir2));

    if (avg < IR_DUST_THRESHOLD) {
        return SURFACE_DUST;          // Dust: high reflectance → low value
    } else if (avg >= IR_DIRT_THRESHOLD) {
        return SURFACE_STICKY_DIRT;   // Dark sticky dirt
    } else {
        return SURFACE_CLEAN;
    }
}

// ============================================================
//  MOTOR CONTROL
// ============================================================
void setMotorA(bool forward, uint8_t speed) {
    digitalWrite(MOTOR_A_IN1, forward ? HIGH : LOW);
    digitalWrite(MOTOR_A_IN2, forward ? LOW  : HIGH);
    analogWrite(MOTOR_A_EN, speed);
}

void setMotorB(bool forward, uint8_t speed) {
    digitalWrite(MOTOR_B_IN1, forward ? HIGH : LOW);
    digitalWrite(MOTOR_B_IN2, forward ? LOW  : HIGH);
    analogWrite(MOTOR_B_EN, speed);
}

void setMotorC(bool forward, uint8_t speed) {
    digitalWrite(MOTOR_C_IN1, forward ? HIGH : LOW);
    digitalWrite(MOTOR_C_IN2, forward ? LOW  : HIGH);
    analogWrite(MOTOR_C_EN, speed);
}

void setMotorD(bool forward, uint8_t speed) {
    digitalWrite(MOTOR_D_IN1, forward ? HIGH : LOW);
    digitalWrite(MOTOR_D_IN2, forward ? LOW  : HIGH);
    analogWrite(MOTOR_D_EN, speed);
}

void motorForward(uint8_t speed) {
    setMotorA(true,  speed);   // Left Front  → forward
    setMotorB(true,  speed);   // Left Rear   → forward
    setMotorC(true,  speed);   // Right Front → forward
    setMotorD(true,  speed);   // Right Rear  → forward
}

void motorBackward(uint8_t speed) {
    setMotorA(false, speed);
    setMotorB(false, speed);
    setMotorC(false, speed);
    setMotorD(false, speed);
}

void motorTurnLeft(uint8_t speed) {
    // Left side backward, Right side forward
    setMotorA(false, speed);
    setMotorB(false, speed);
    setMotorC(true,  speed);
    setMotorD(true,  speed);
}

void motorTurnRight(uint8_t speed) {
    // Left side forward, Right side backward
    setMotorA(true,  speed);
    setMotorB(true,  speed);
    setMotorC(false, speed);
    setMotorD(false, speed);
}

void motorStop() {
    setMotorA(true, MOTOR_SPEED_STOP);
    setMotorB(true, MOTOR_SPEED_STOP);
    setMotorC(true, MOTOR_SPEED_STOP);
    setMotorD(true, MOTOR_SPEED_STOP);
}

// ============================================================
//  I2C SLAVE COMMUNICATION
// ============================================================
void sendSlaveCommand(uint8_t cmd) {
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write(cmd);
    Wire.endTransmission();
    debugPrint("Slave CMD sent: 0x" + String(cmd, HEX));
}

uint8_t requestSlaveStatus() {
    Wire.requestFrom((uint8_t)SLAVE_ADDRESS, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;   // Error
}

// ============================================================
//  ESP32 COMMUNICATION
// ============================================================
void processESP32Data() {
    if (!ESP32_SERIAL.available()) return;

    String msg = ESP32_SERIAL.readStringUntil('\n');
    msg.trim();
    if (msg.length() == 0) return;

    debugPrint("ESP32 → Mega: " + msg);

    RCCommand cmd = parseRCCommand(msg);

    if (cmd == RC_MANUAL_MODE) {
        autoMode = false;
        enterState(STATE_RC_MANUAL);
        debugPrint(F("Manual mode activated"));
    } else if (cmd == RC_AUTO_MODE) {
        autoMode = true;
        enterState(STATE_SCANNING);
        debugPrint(F("Auto mode activated"));
    } else if (!autoMode) {
        lastRCCommand = cmd;
    }
}

RCCommand parseRCCommand(const String& msg) {
    if (msg == "FWD")    return RC_FORWARD;
    if (msg == "BWD")    return RC_BACKWARD;
    if (msg == "LFT")    return RC_LEFT;
    if (msg == "RGT")    return RC_RIGHT;
    if (msg == "STP")    return RC_STOP;
    if (msg == "AUTO")   return RC_AUTO_MODE;
    if (msg == "MAN")    return RC_MANUAL_MODE;
    if (msg == "VAC")    return RC_VACUUM_TOGGLE;
    if (msg == "MOP")    return RC_MOP_TOGGLE;
    if (msg == "SPR")    return RC_SPRAY_TOGGLE;
    return RC_NONE;
}

void sendStatusToESP32() {
    // JSON-like status string
    String status = "{";
    status += "\"state\":\"" + stateToString(currentState) + "\",";
    status += "\"auto\":"    + String(autoMode    ? "true" : "false") + ",";
    status += "\"vacuum\":"  + String(vacuumActive ? "true" : "false") + ",";
    status += "\"mop\":"     + String(mopActive    ? "true" : "false") + ",";
    status += "\"spray\":"   + String(sprayActive  ? "true" : "false") + ",";
    status += "\"obstacle\":"+ String(obstacleDetected ? "true" : "false");
    status += "}";

    ESP32_SERIAL.println(status);
}

// ============================================================
//  UTILITIES
// ============================================================
void debugPrint(const String& msg) {
    Serial.println(msg);
}

String stateToString(RobotState s) {
    switch (s) {
        case STATE_IDLE:           return "IDLE";
        case STATE_SCANNING:       return "SCANNING";
        case STATE_DRY_CLEANING:   return "DRY_CLEANING";
        case STATE_WET_CLEANING:   return "WET_CLEANING";
        case STATE_NAVIGATING:     return "NAVIGATING";
        case STATE_OBSTACLE_AVOID: return "OBSTACLE_AVOID";
        case STATE_RC_MANUAL:      return "RC_MANUAL";
        case STATE_ERROR:          return "ERROR";
        default:                   return "UNKNOWN";
    }
}
