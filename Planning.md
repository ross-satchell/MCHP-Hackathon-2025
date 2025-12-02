# # 🛠️ Self-Balancing Robot - Planning

## Goals Day 1

HW Concept from [Piddybot](https://www.instructables.com/PIDDYBOT-DIY-Arduino-Balancing-Robot/)

HW Side

- Fasten ruler to frame for HW bot
- determine where to put Battery?
- 3d printed vs. Frame bound motor components.
- Motor encoders

SW side

- GIThub project repo setup.
- Develop SW - first simple balance, then moving forwards, line following...
- Use old Academic baselines - IMU gyro control, display, etc.
- add BT Connectivity to adjust drift component from phone

ACA side

- anything we can add to help support a coursework edit.
- Video demonstration / promo?

## POC Checklist (Streamlined)

## 1. Hardware Build

### Chassis & Mounts

- [X] Design 3D-printed mount for PCB, motors, wheels (CAD model)
- [X] 3D print prototype and verify fit for all hardware
- [ ] Refine mount design (if required for wiring/stability)
- [ ] Assemble: securely mount PCB, motors, wheels

### Final Assembly & Connections

- [ ] Wire motors to driver/PCB
- [ ] Connect power supply to PCB
- [ ] Physical checks: secure mounts, check alignment
- [ ] Power-up and confirm voltages, component operation

---

## 2. Software Development (MicroPython/CircuitPython)

### Dev Environment & Libraries

- [X] Set up development IDE (VS Code, Mu)
- [X] Install/verify MicroPython/CircuitPython firmware on MCU
- [X] Include/validate necessary Python libraries for IMU
- [ ] Libraries for motor driver
- [ ] Libraries for BLE - built in _bleio or BLE 10.1.0 ( [Download release here](https://github.com/adafruit/Adafruit_CircuitPython_BLE/releases) )

### Sensor & Motor Verification

- [X] Write/test MicroPython code to read IMU angle data - Same as Serial Monitor demo code...
- [ ] Calibrate IMU readings (if/as needed)
- [ ] Test basic motor control (direction, speed) with driver

### PID Control System in Python

- [ ] Implement or adapt PID controller in Python (MicroPython or CircuitPython)
- [ ] Integrate IMU data, PID logic, and motor actuation in main loop
- [ ] Tune PID gains for balancing performance

### Integration & Testing

- [ ] Combine sensor reading, PID, motor actuation in main control program
- [ ] Test loop timing and sample rate
- [ ] Implement basic safety/fail-safes (cut motors if out of range)

### Debugging & Optimization

- [ ] Implement serial data logging (angle, PID values, motor commands)
- [ ] Refine/tune PID parameters for robust balancing

---

## 3. Documentation & Demo

### Mechanical & Software Documentation

- [ ] Document 3D mount (files, CAD screenshots, notes)
- [ ] Document software architecture, list dependencies, usage notes

### POC Demo Preparation

- [ ] Plan and perform controlled tests (safe area)
- [ ] Record video and summarize/demo results

---
