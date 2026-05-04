# Semi-Autonomous Floor Scrubbing Robot — System Documentation

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      MOBILE RC APP (Browser)                    │
│                   http://192.168.4.1  (Wi-Fi)                   │
└───────────────────────────┬─────────────────────────────────────┘
                            │ Wi-Fi (AP Mode)
┌───────────────────────────▼─────────────────────────────────────┐
│                       ESP32-CAM                                 │
│  • MJPEG Live Stream (port 81)   • RC Dashboard (port 80)       │
│  • Parses RC commands            • Streams robot status         │
└───────────────────────────┬─────────────────────────────────────┘
                            │ UART Serial (115200 baud)
┌───────────────────────────▼─────────────────────────────────────┐
│                   ARDUINO MEGA 2560  (MASTER)                   │
│  • Navigation (4x DC motors)    • State Machine                 │
│  • Ultrasonic front/left/right  • IR Surface Analysis           │
└───────────────────────────┬─────────────────────────────────────┘
                            │ I2C  (SDA=20, SCL=21, Address 0x08)
┌───────────────────────────▼─────────────────────────────────────┐
│                   ARDUINO UNO R3  (SLAVE)                       │
│  • S3003 Servo x2 (Spray)       • Mop Motor (L298N)             │
│  • Vacuum Motor (Relay)         • Liquid Level Sensor           │
└─────────────────────────────────────────────────────────────────┘
```

---

## Source Files

| File | Controller | Description |
|------|-----------|-------------|
| `arduino_mega_master.cpp` | Arduino Mega 2560 | Navigation, sensors, state machine |
| `arduino_uno_slave.cpp`   | Arduino UNO R3    | Cleaning actuators, servos, vacuum |
| `esp32_cam_controller.cpp`| ESP32-CAM         | Wi-Fi, video stream, RC dashboard |

---

## Arduino Mega — Pin Assignments

### Drive Motors (L298N x 2)
| Signal | Pin | Description |
|--------|-----|-------------|
| MOTOR_A_IN1 | 2  | Left Front direction |
| MOTOR_A_IN2 | 3  | Left Front direction |
| MOTOR_A_EN  | 4  | Left Front PWM speed |
| MOTOR_B_IN1 | 5  | Left Rear direction |
| MOTOR_B_IN2 | 6  | Left Rear direction |
| MOTOR_B_EN  | 7  | Left Rear PWM speed |
| MOTOR_C_IN1 | 8  | Right Front direction |
| MOTOR_C_IN2 | 9  | Right Front direction |
| MOTOR_C_EN  | 10 | Right Front PWM speed |
| MOTOR_D_IN1 | 11 | Right Rear direction |
| MOTOR_D_IN2 | 12 | Right Rear direction |
| MOTOR_D_EN  | 13 | Right Rear PWM speed |

### Ultrasonic Sensors (HC-SR04 x 3)
| Sensor | TRIG | ECHO |
|--------|------|------|
| Front  | 22   | 23   |
| Left   | 24   | 25   |
| Right  | 26   | 27   |

### IR Sensors
| Sensor | Pin |
|--------|-----|
| IR1 (primary) | A0 |
| IR2 (confirm) | A1 |

### Communication
| Interface | Pins | Target |
|-----------|------|--------|
| I2C SDA | 20 | UNO Slave |
| I2C SCL | 21 | UNO Slave |
| Serial1 TX | 18 | ESP32 RX |
| Serial1 RX | 19 | ESP32 TX |

---

## Arduino UNO R3 — Pin Assignments

| Signal | Pin | Description |
|--------|-----|-------------|
| Servo Left   | 9  | S3003 left spray nozzle |
| Servo Right  | 10 | S3003 right spray nozzle |
| MOP_IN1      | 2  | Mop motor direction |
| MOP_IN2      | 3  | Mop motor direction |
| MOP_EN       | 5  | Mop motor PWM |
| VACUUM_RELAY | 6  | Vacuum relay IN pin |
| LED_IDLE     | A0 | Status LED |
| LED_VACUUM   | A1 | Status LED |
| LED_MOP      | A2 | Status LED |
| LED_SPRAY    | A3 | Status LED |
| LIQUID_LEVEL | A4 | Tank level sensor |

---

## ESP32-CAM Configuration

| Parameter | Value |
|-----------|-------|
| Wi-Fi Mode | Access Point |
| SSID | `RobotControl` |
| Password | `robot1234` |
| IP Address | `192.168.4.1` |
| Dashboard URL | `http://192.168.4.1` |
| Video Stream | `http://192.168.4.1:81/stream` |
| Camera Resolution | VGA 640x480 |

---

## State Machine (Arduino Mega)

| State | Motors | Cleaning | Transition |
|-------|--------|----------|------------|
| IDLE | STOP | ALL OFF | → SCANNING after 1s |
| SCANNING | STOP | — | IR reads → branch |
| DRY_CLEANING | SLOW | Vacuum ON | 5s timeout → SCAN |
| WET_CLEANING | SLOW | Spray+Mop ON | 5s timeout → SCAN |
| NAVIGATING | NORMAL | ALL OFF | obstacle → AVOID, periodic → SCAN |
| OBSTACLE_AVOID | STOP/TURN | — | → SCANNING |
| RC_MANUAL | RC CMD | manual toggle | AUTO cmd → SCANNING |

---

## Surface Detection (IR Sensors)

| IR Average | Surface | Action |
|-----------|---------|--------|
| < 400 | Dust | Vacuum ON |
| 400–599 | Clean | Navigate |
| >= 600 | Sticky/hard dirt | Spray + Mop ON |

---

## I2C Commands (Mega → UNO)

| Byte | Meaning |
|------|---------|
| 0x00 | All OFF |
| 0x01 | Vacuum ON |
| 0x02 | Vacuum OFF |
| 0x03 | Spray ON |
| 0x04 | Spray OFF |
| 0x05 | Mop ON |
| 0x06 | Mop OFF |
| 0x07 | Wet Clean (Spray+Mop) |
| 0x08 | Dry Clean (Vacuum) |
| 0x09 | Status Request |

---

## RC Commands (App → ESP32 → Mega)

| String | Action |
|--------|--------|
| FWD | Forward |
| BWD | Backward |
| LFT | Turn Left |
| RGT | Turn Right |
| STP | Stop |
| AUTO | Auto mode |
| MAN | Manual mode |
| VAC | Toggle vacuum |
| MOP | Toggle mop |
| SPR | Toggle spray |

---

## Required Libraries

### Arduino Mega
- `Wire.h` (built-in)
- **NewPing** by Tim Eckel — install via Library Manager

### Arduino UNO R3
- `Wire.h` (built-in)
- `Servo.h` (built-in)

### ESP32-CAM
- **ESP32 by Espressif Systems** board package (Board Manager)
- `WebServer.h`, `esp_camera.h` — included in ESP32 core

---

## Upload Instructions

### Arduino Mega
1. Open `arduino_mega_master.cpp`
2. Tools → Board → **Arduino Mega or Mega 2560**
3. Install NewPing library → Upload

### Arduino UNO R3
1. Open `arduino_uno_slave.cpp`
2. Tools → Board → **Arduino Uno** → Upload

### ESP32-CAM
1. Open `esp32_cam_controller.cpp`
2. Tools → Board → **AI Thinker ESP32-CAM**
3. **Short GPIO0 to GND** before upload (flash mode)
4. Upload → Disconnect GPIO0 → Press RESET

---

## Cleaning Mode Summary

| Mode | Vacuum | Spray | Mop | Triggered By |
|------|--------|-------|-----|-------------|
| Dry Cleaning | ON | OFF | OFF | IR: Dust |
| Wet/Deep Cleaning | OFF | ON | ON | IR: Sticky dirt |
| Navigation | OFF | OFF | OFF | IR: Clean |

---

## Safety Features

- Brownout detector disabled on ESP32-CAM (motor noise tolerance)
- Spray auto-disables when liquid tank sensor reads LOW
- Obstacle avoidance active in all autonomous states
- Cleaning timeout (5s max per spot) prevents over-scrubbing
- Status byte on I2C lets Master monitor Slave health
