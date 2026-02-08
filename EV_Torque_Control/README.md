# EV Torque Control System (Simulink / Stateflow)

This project implements a simplified Electric Vehicle (EV) torque control system
using MATLAB Simulink and Stateflow (R2026a version). The focus is on clean architecture, control
logic clarity, and ASIL-style safety separation rather than production-level
complexity.

The system handles drive torque, braking torque, and fault conditions while
demonstrating realistic control strategies used in automotive applications.

---

## System Overview

The model is structured into clearly separated subsystems:

- Input Conditioning
- Safety / Plausibility Checks
- Driving Mode State Machine (Stateflow)
- Torque Calculation
- Outputs (Actuator Interface)

This separation follows common automotive control and functional safety practices.

---

## Features

- Drive / Braking / Fault mode management using Stateflow
- ASIL-style safety plausibility checks on raw sensor inputs
- Normalization and conditioning of driver inputs
- Realistic torque calculation for:
  - Drive torque
  - Regenerative braking
  - Friction braking
- Regenerative braking limited by vehicle speed
- Torque blending between regenerative and friction braking
- Actuator-level torque saturation
- Fail-safe behavior on detected faults

---

## Safety and Fault Handling

Safety plausibility checks are implemented on raw input signals
(accelerator pedal, brake pedal, vehicle speed) to avoid fault masking.

Detected faults generate a `Fault_Flag`, which forces the system into a Fault
state via Stateflow. In the Fault state, all torque requests are set to zero.

This architecture reflects ASIL-style design principles commonly used in
automotive systems.

---

## Tools and Technologies

- MATLAB
- Simulink
- Stateflow

---

## Project Scope and Limitations

This project is intended for learning and demonstration purposes.
It does not represent production-certified automotive software
and does not include redundancy, diagnostics, or ISO 26262 certification artifacts.

---

## Example Test Scenarios

- Normal driving (acceleration only)
- Braking with regenerative and friction torque blending
- Pedal conflict detection
- Out-of-range sensor fault injection
- Fault recovery behavior

---

## Author

Ghazal Ghorbani
