
# ESP32–STM32 Wireless Touch-Controlled Car with Obstacle Detection

A dual-mode embedded robotic car that can be controlled using a touch-based interface and can switch into an autonomous mode for real-time obstacle awareness.

---

## 🧩 System Architecture

### Controller Module
- ESP32 + TSC2046 + resistive touchpad  
- ESP-NOW for low-latency wireless transmission  

### Car Module
- ESP32 receiver → forwards data to STM32 via I²C  
- STM32 handles:
  - Touch interpretation  
  - PWM motor control  
  - Ultrasonic distance measurement  
  - Mode switching  
- Four DC motors driven using dual IBT-2 motor drivers  
- Three HC-SR04 sensors for ~180° obstacle detection  

---

## 🛠 Key Features

- Dual-mode operation (manual touch control + autonomous obstacle awareness)  
- Real-time wireless control using ESP-NOW  
- Smooth motor actuation using PWM   
- Touch input stabilization using filtering
- Automatic safety stop feature when obstacles are too close  

---

This project includes three main firmware components:

---

### **1️⃣ STM32 Main Firmware (`STM32_Main/`)**
Runs on the STM32F401RE and handles:
- I²C communication with ESP32  
- Touch command decoding  
- PWM motor control  
- Ultrasonic sensing (front/left/right)  
- Mode switching + UART output  

---

### **2️⃣ ESP32 Trackpad Firmware (`ESP32_Trackpad/`)**
Runs on the handheld controller and handles:
- Reading touchpad X/Y via TSC2046 (SPI)  
- Filtering and packaging coordinates  
- Sending data to the car using **ESP-NOW**  

---

### **3️⃣ ESP32 Car Firmware (`ESP32_to_STM32/`)**
Runs on the car ESP32 and handles:
- Receiving ESP-NOW touch packets  
- Validating/unpacking data  
- Forwarding coordinates to STM32 via **I²C**  

---

## 📁 Media
All diagrams, wiring layouts, and demo images are stored in the `Media/` folder for quick reference and visualization.




 




