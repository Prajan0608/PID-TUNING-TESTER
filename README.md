# PID-TUNING-TESTER #
# ESP32 PID Distance Control Using Quadrature Encoder

## 📌 Project Overview
This project implements a **closed-loop distance control system** using an **ESP32**, 
**DC motors**, and a **quadrature encoder**.  
A **PID (Proportional–Integral–Derivative) controller** is used to move the robot
to a specified target distance accurately with **smooth motion and zero overshoot**.

The system continuously measures the actual distance using encoder feedback,
compares it with the target distance, and adjusts the motor speed in real time.

---

## 🎯 Objective
- To achieve **precise distance control** in a mobile robot
- To eliminate errors caused by load variation and wheel slip
- To demonstrate practical implementation of PID control using ESP32

---

## ⚙️ System Architecture

**Input**
- Target distance (in centimeters)

**Feedback**
- Encoder pulse count

**Controller**
- PID control algorithm

**Output**
- Motor PWM and direction signals

This forms a **closed-loop feedback control system**.

---

## 🧠 Key Concepts Used
- Closed-loop control system
- Quadrature encoder feedback
- PID control algorithm
- Anti-windup technique
- Soft stopping logic
- Real-time parameter tuning

---

## 🔧 Hardware Requirements

| Component | Description |
|--------|------------|
| ESP32 | Main controller |
| DC Motors | Differential drive motors |
| Quadrature Encoder | Distance feedback |
| Motor Driver | L298N / BTS7960 / equivalent |
| Battery | External motor power supply |

---

## 🔌 Pin Configuration

### Motor Driver Connections
| ESP32 Pin | Function |
|--------|---------|
| GPIO 14 | Left Motor PWM |
| GPIO 22 | Left Motor Direction |
| GPIO 13 | Right Motor PWM |
| GPIO 23 | Right Motor Direction |

### Encoder Connections
| Encoder Pin | ESP32 Pin |
|-----------|-----------|
| Channel A | GPIO 25 |
| Channel B | GPIO 26 |
| VCC | 3.3V |
| GND | GND |

> ⚠️ **Common ground between ESP32 and motor driver is mandatory**

---

## 🔄 Encoder Working
- Full quadrature decoding is used
- Direction and speed are detected accurately
- Internal pull-up resistors are enabled
- Encoder counts increase or decrease based on direction

Distance calculation:

Distance (cm) = Encoder_Count / COUNTS_PER_CM


---

## 🧮 PID Control Logic

### Error Calculation


error = target_distance − actual_distance


### PID Equation


output = (Kp × error) + (Ki × integral) + (Kd × derivative)


---

### 🔵 Proportional (Kp)
- Controls speed based on present error
- Higher Kp → faster response
- Too high → oscillation

---

### 🟢 Integral (Ki)
- Accumulates past errors
- Removes steady-state error

In this project:
- Integral is active **only near the target**
- Prevents overshoot and windup

---

### 🔴 Derivative (Kd)
- Predicts future error
- Slows down motion before reaching target
- Responsible for smooth stopping

---

## 🛡️ Stability Enhancements

### ✅ Anti-Windup
Limits excessive integral accumulation.

### ✅ Minimum PWM Compensation
Prevents motor stalling at low speeds.

### ✅ Speed Limiting Near Target
Ensures smooth deceleration and accurate stop.

---

## 🏁 Stop Condition
The motor automatically stops when:
- Distance error < **0.05 cm**
- Change in error is nearly zero

This guarantees:
✔ Accurate positioning  
✔ No oscillation  
✔ Stable behavior  

---

## ⌨️ Serial Commands (Live Tuning)

| Command | Description |
|------|-------------|
| `p<value>` | Set proportional gain |
| `i<value>` | Set integral gain |
| `d<value>` | Set derivative gain |
| `t<value>` | Set target distance (cm) |
| `s` | Start motion |
| `r` | Stop motor |

### Example


p12
i0.7
d600
t30
s


---

## 📊 Serial Plotter Output
Data format:


Target_Distance,Actual_Distance


Used for real-time performance visualization.

---

## 🧪 Experimental Results
- Fast initial acceleration
- Smooth deceleration near target
- Zero overshoot
- Final position error < **0.05 cm**

---

## ⚠️ Safety Notes
- Do not power motors from ESP32
- Use external motor power supply
- Ensure proper grounding
- Secure robot during testing

---

## 🚀 Applications
- Autonomous mobile robots
- AGVs (Automated Guided Vehicles)
- Line-following robots
- Precision motion control systems
- Academic and research projects

---

## 📌 Conclusion
This project demonstrates a **robust and reliable PID-based distance control system**
using ESP32 and encoder feedback.  
The implementation is suitable for **real-world robotics applications** as well as
academic and industrial learning purposes.

---

## 🔮 Future Enhancements
- Adaptive PID tuning
- Velocity profiling
- IMU + encoder sensor fusion
- ROS integration
