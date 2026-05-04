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

## ⚙️ System Flow (Animated)

<p align="center">
<svg width="700" height="160" xmlns="http://www.w3.org/2000/svg">

  <rect width="100%" height="100%" fill="#0d1117"/>

  <!-- Boxes -->
  <rect x="30" y="50" width="100" height="40" stroke="#00F7FF" fill="none"/>
  <rect x="170" y="50" width="100" height="40" stroke="#00F7FF" fill="none"/>
  <rect x="310" y="50" width="100" height="40" stroke="#00F7FF" fill="none"/>
  <rect x="450" y="50" width="100" height="40" stroke="#00F7FF" fill="none"/>
  <rect x="590" y="50" width="100" height="40" stroke="#00F7FF" fill="none"/>

  <!-- Labels -->
  <text x="50" y="75" fill="#00F7FF">Scan</text>
  <text x="190" y="75" fill="#00F7FF">Detect</text>
  <text x="330" y="75" fill="#00F7FF">Vacuum</text>
  <text x="470" y="75" fill="#00F7FF">Spray</text>
  <text x="610" y="75" fill="#00F7FF">Scrub</text>

  <!-- Moving Dot -->
  <circle r="5" fill="#00F7FF">
    <animateMotion dur="4s" repeatCount="indefinite"
      path="M80,90 L220,90 L360,90 L500,90 L640,90"/>
  </circle>

</svg>
</p>
