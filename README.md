# Semi-Autonomous Floor Scrubbing Robot
🚧 **Status:** Complete Project

<img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/c03ff30b-c7d6-48f1-b188-fbfb570f6bcc" />

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

## 📐 Mechanism & Kinematic Analysis

The cleaning mechanism was analyzed using the **Complex Algebra Method** (Theory of Machines).

### Objective
- Evaluate motion of scrubbing mechanism  
- Ensure stable and efficient operation  

### Analysis
- **Link 3:** Angular displacement, velocity, acceleration  
- **Link 4:** Linear displacement, velocity, acceleration  

### Governing Equations

---

## 🖼️ System Design

### 🔹 Prototype
<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/ce2e42f9-c2ee-48c6-9868-4d5a9191a377" />

### 🔹 CAD Model
<img width="778" height="613" alt="image" src="https://github.com/user-attachments/assets/d811c9d1-765b-4604-be23-4c3fae4a91b8" />

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
- vision-based navigation  
- Automated water management system  

---

## 🧠 Key Insight
This system demonstrates a practical integration of:
- **Mechanical design (scrubbing + vacuum)**
- **Embedded control (multi-controller architecture)**
- **Sensor-based intelligence (decision-driven cleaning)**

→ enabling a low-cost semi-autonomous cleaning robot.

