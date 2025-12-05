
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

## 🔧 Technical Implementation

### Touch Input Flow

1. Touchpad → TSC2046 digitizes X/Y coordinates  
2. ESP32 reads touch data using SPI  
3. ESP-NOW transmits X/Y coordinates wirelessly to the car  
4. Receiving ESP32 forwards data to STM32 via I²C  
5. STM32 maps coordinates to:
   - Forward  
   - Backward  
   - Left  
   - Right  
   - Stop  

### Autonomous Mode Logic

- STM32 polls all three ultrasonic sensors (front, left, right)  
- Implements timeout handling + multiple echo validation  
- Stops all motors when any distance < 30 cm threshold  
- Sends distance readings periodically over UART for monitoring  





 




