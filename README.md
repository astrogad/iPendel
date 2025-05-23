# iPendel

Inverted pendulums are no longer rare; today, there are numerous variations explored across academic and hobbyist communities. This project focuses on the **from-scratch development** of a **reaction wheel type inverted pendulum** using off-the-shelf components.

What sets this project apart is the use of **Simscape/SimMechanics** for modeling and simulating the controller. Unlike many similar projects that rely on simplified or purely mathematical models, this implementation leverages physical modeling tools to more accurately capture the system dynamics.

The project title "iPendel" is inspired by an article (authored by Jean-Sébastien Gonsette) published in *Elektor* magazine.

---

## Features

- 🚀 Full physical modeling using Simscape/SimMechanics
- 🎯 Custom LQR controller implementation
- 🧠 Sensor fusion combining MPU6050 IMU and AS5047P encoder
- 🛞 Reaction wheel stabilization with BLDC gimbal motor
- 🧰 Real-time control with Teensy 4.0 and SimpleFOC
- 🖨️ Lightweight, 3D-printed pendulum body
- 📊 Optional Python-based data visualization for tuning

---

## Hardware Components

- Teensy 4.0 microcontroller  
- ZS-X12H brushless driver with hall sensor
- Nanotec BLDC Motor DF45S024050-A2  
- Stainless steel flywheel (~73g)  
- MPU6050 IMU  
- AS5047P magnetic rotary encoder  
- Custom 3D-printed PETG frame

---

## Software Stack

- Arduino IDE with SimpleFOC  
- MATLAB/Simulink + Simscape/SimMechanics  
- Python (for visualization and tuning support)

---

## Control Strategy

The stabilization is achieved using an LQR (Linear Quadratic Regulator) approach. The physical model in Simscape allows for precise derivation of state-space dynamics, which improves tuning accuracy and performance.

Sensor fusion combines data from the MPU6050 (for angle estimation) and the AS5047P (for angular velocity and motor shaft position).

---

## Project Status

🔧 **In Progress**

The core hardware has been assembled, and a basic LQR controller has been implemented. Stabilization is _marginally_ functional - the pendulum can balance at its unstable equilibrium point when manually positioned and released, at which point the control loop begins to take over. **However, the pendulum is very sensitive to disturbances** - even a slight touch or vibration of the tabletop (on which it stands) can cause it to topple. Further optimization (especially in tuning and modeling) is ongoing.

---

## Future Work

- Implement hardware and software improvements to increase robustness against external disturbances.
- Go from the breadboard-wired prototype to a proper custom PCB design, and possibly tweak the controller to incorporate the resulting changes in the dynamics.
- Incorporate stand-up control into the simulation and the code.

---

## License

MIT License. See `LICENSE` file for details.
