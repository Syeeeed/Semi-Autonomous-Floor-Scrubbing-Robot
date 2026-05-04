# Semi-Autonomous Floor Scrubbing Robot

## 🧠 Overview
A semi-autonomous floor scrubbing robot designed for integrated **dry (dust) and wet (liquid) cleaning**. The system combines vacuum suction, liquid spraying, and rotating mop-based scrubbing with sensor-driven navigation and dual-controller architecture.

---

## ⚙️ System Concept

The robot intelligently switches between:
- **Vacuum cleaning (dust removal)**
- **Liquid spraying + scrubbing (deep cleaning)**

---

## 🧩 Mechanical System

- **Locomotion:** 4 DC motors (wheel drive)  
- **Scrubbing Unit:** 1 motor-driven rotating mop  
- **Cleaning Medium:** Mop capable of both dry and wet cleaning  

---

## 💧 Cleaning Mechanism

### Dust Cleaning
- Vacuum system activated when dust is detected  
- Continuous suction during motion  

### Liquid Cleaning
- 2 × **S3003 servo motors** control spray system  
- Cleaning solution sprayed as foam  
- Mop motor rotates for deep scrubbing  

---

## 🧠 Intelligent Cleaning Logic

---

## 🧠 Control Architecture

### Master–Slave System
- **Arduino Mega (Master):**
  - Main control & navigation logic  
  - Decision making  

- **Arduino UNO R3 (Slave):**
  - Liquid cleaning system control  
  - Servo and auxiliary control  

---

## 📡 Sensors & Navigation

- **Ultrasonic Sensors:** Obstacle detection  
- **IR Sensors:** Auxiliary navigation  
- Semi-autonomous movement with obstacle avoidance  

---

## 📷 Vision & Communication

- **ESP32 Camera Module**
  - Real-time video streaming  
  - Wi-Fi-based communication  

- **Mobile Control**
  - Controlled via **RC Car mobile app**  
  - Live visual feedback through camera  

---

## 🔌 System Architecture

---

## 🖼️ System Design

### 🔹 Prototype
![Robot Prototype](./assets/prototype.jpg)

### 🔹 CAD Model
![CAD Design](./assets/design1.png)
![CAD Design](./assets/design2.png)

---

## 🎥 Demonstration
(Add your video link here)

---

## ⚙️ Tech Stack
- **Microcontrollers:** Arduino Mega, Arduino UNO R3  
- **Sensors:** Ultrasonic, IR Sensors  
- **Vision:** ESP32 Camera (Wi-Fi enabled)  
- **Actuators:** DC Motors, Servo Motors (S3003)  
- **Control:** Arduino IDE (Embedded C)  

---

## 🚀 Features
- Dual-mode cleaning (**dry + wet**)  
- Semi-autonomous navigation  
- Intelligent cleaning decision logic  
- Real-time camera feedback  
- Mobile app control  
- Modular and low-cost design  

---

## 🚀 Applications
- Indoor floor cleaning  
- Smart cleaning systems  
- Educational robotics platform  

---

## ⚡ Future Improvements
- Full SLAM-based autonomy  
- AI-based dirt classification  
- Path optimization algorithms  
- LiDAR / vision-based navigation  
- Automated water management system  

---

## 🧠 Key Insight
This system demonstrates a practical integration of:
- **Mechanical design (scrubbing + vacuum)**
- **Embedded control (multi-controller architecture)**
- **Sensor-based intelligence (decision-driven cleaning)**

→ enabling a low-cost semi-autonomous cleaning robot.
