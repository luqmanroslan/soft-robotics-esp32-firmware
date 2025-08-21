
# Soft Robot Firmware – ESP32 Low-Level Controller  

This repository contains the **ESP32 firmware** for the soft robotic actuator control system.  
It implements **high-level length control, low-level pressure regulation, and safety mechanisms**, providing a reliable interface between the robot hardware and a PC-side GUI.  

---

## Features  
- **Sensor integration**:  
  - BNO08x IMU (orientation → roll, pitch, yaw)  
  - Potentiometers (actuator length feedback)  
  - Pressure sensors (real-time chamber pressures)  
- **Control modes**:  
  - Length-to-pressure PID control (FK/IK loop)  
  - Direct pressure control  
  - Dance/trajectory playback mode  
- **Communication**:  
  - USB serial @ **115200 baud**  
  - **JSON protocol** for commands & telemetry (ArduinoJson)  
- **Safety mechanisms**:  
  - `emergency_stop` command (closes valves immediately)  
  - Pressure limits enforced with `constrain()`  
  - Fail-safe defaults on boot  

---

## Repository Structure  
- `Low_Level_Controller_Updated.ino` — main firmware  
- `README.md` — documentation (this file)  

---

## Dependencies  
Install via Arduino Library Manager or manually:  
- [Adafruit BNO08x](https://github.com/adafruit/Adafruit_BNO08x)  
- [ArduinoJson](https://arduinojson.org/)  

---

## How to Build & Flash  
1. Open the `.ino` file in the **Arduino IDE** (or PlatformIO).  
2. Select **ESP32 Dev Module** as the board.  
3. Set the baud rate to **115200**.  
4. Connect ESP32 via USB and click **Upload**.  

---

## Serial Protocol  

### Commands (PC → ESP32)  
- **Length control**  
  ```json
  { "command":"lengths_control", "target_lengths":[L1,L2,L3], "current_lengths":[C1,C2,C3] }
  ```

- **Direct length set** 
```json
{ "command": "length", "values": [L1, L2, L3] }
```

- **Direct pressure set** 
```json
{ "command": "set_pressure", "values": [P1, P2, P3] }
```
- **Emergency Stop** 
```json
{ "command": "emergency_stop" }
```

- **Dance mode** 
```json
{ "command": "start_dance" }
{ "command": "stop_dance" }
```

---

## Integration with the Full System  
This firmware is one part of the **Soft Robot Control System**, which also includes:  
- **PC-side GUI & Communication Layer** – for visualization, joystick/voice control, and serial communication with the ESP32.  
- **Machine Learning Models** – for mapping sensor readings to actuator poses and supporting high-level control strategies.  

👉 For the full project, see the companion repositories:  
- [soft-robotics-gui](https://github.com/yourusername/soft-robotics-gui)  
- [ml-training](https://github.com/luqmanroslan/ml-training)  






