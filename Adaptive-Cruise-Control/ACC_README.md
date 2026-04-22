# Adaptive Cruise Control (ACC) — Model-Based Design in MATLAB/Simulink

**Author:** Ghazal Ghorbani  
**Tools:** MATLAB, Simulink, Stateflow, Embedded Coder, Simulink Test  
**Domain:** Automotive ADAS — Longitudinal Control  
**Status:** 🟡 In Progress — Phase 1 Complete

---

## Project Overview

A complete Model-Based Design (MBD) implementation of an Adaptive Cruise Control (ACC) system for highway driving scenarios. The system regulates ego vehicle speed and maintains a safe following distance to a lead vehicle using a multi-mode Stateflow state machine, PID-based control laws, and auto-generated production C code via Embedded Coder.

This project demonstrates the full MBD workflow: plant modelling → sensor modelling → control logic (Stateflow) → controller design (PID) → C code generation → software-in-the-loop (SIL) verification → test scenario reporting.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   ACC_System.slx (Top Level)            │
│                                                         │
│  ┌──────────────┐    ┌──────────────┐                  │
│  │ ACC_Plant    │───▶│ ACC_Sensor   │                  │
│  │ (Phase 1)    │    │ (Phase 2)    │                  │
│  └──────────────┘    └──────┬───────┘                  │
│         ▲                   │ d_rel_meas               │
│         │ a_demand          │ delta_v_meas             │
│         │            ┌──────▼───────┐                  │
│         │            │ ACC_Control  │                  │
│         └────────────│ (Phase 3+4) │                  │
│                      └──────────────┘                  │
└─────────────────────────────────────────────────────────┘
```

---

## Repository Structure

```
ACC_Simulink_Project/
├── ACC_Plant.slx           ← Phase 1: Vehicle plant model
├── ACC_Sensor.slx          ← Phase 2: Radar sensor model
├── ACC_StateMachine.slx    ← Phase 3: Stateflow ACC logic
├── ACC_Controller.slx      ← Phase 4: PID controller
├── ACC_System.slx          ← Full integrated system
├── docs/
│   ├── requirements.md     ← System requirements
│   └── test_report.pdf     ← Phase 6 test results
└── README.md
```

---

## Project Phases

| Phase | Description | MATLAB Tools | Status |
|-------|-------------|--------------|--------|
| 1 | Vehicle Plant Model | Simulink | ✅ Complete |
| 2 | Radar Sensor Model | Simulink | ⬜ Pending |
| 3 | ACC State Machine | Stateflow | ⬜ Pending |
| 4 | Controller Design (PID) | Simulink | ⬜ Pending |
| 5 | C Code Generation & SIL | Embedded Coder | ⬜ Pending |
| 6 | Test Scenarios & Report | Simulink Test | ⬜ Pending |

---

## Phase 1 — Vehicle Plant Model (`ACC_Plant.slx`)

### Purpose

Models the physical longitudinal dynamics of two vehicles on a straight highway. This is the **ground-truth simulation environment** — it represents the real world that the ACC controller will interact with. No control logic is present in this model; it is intentionally passive and physics-only.

Separating the plant from the controller is a fundamental principle of Model-Based Design, mirroring how real automotive teams divide vehicle modelling from ECU software development.

### Subsystems

#### `Ego_Vehicle`
The car equipped with ACC — the vehicle being controlled.

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Input | `a_demand` [m/s²] | Acceleration command (from ACC controller in Phase 4) |
| Saturation upper limit | +3 m/s² | Comfortable max acceleration for a passenger car |
| Saturation lower limit | −8 m/s² | Hard braking limit (~0.8g), consistent with real ACC specs |
| Integrator 1 IC | 20 m/s | Ego starts at highway speed (72 km/h) |
| Integrator 2 IC | 0 m | Ego defined as positional origin |
| Output 1 | `v_ego` [m/s] | Ego vehicle longitudinal speed |
| Output 2 | `x_ego` [m] | Ego vehicle longitudinal position |

**Signal flow:**  
`a_demand → Saturation → ∫(a→v_ego) → ∫(v_ego→x_ego)`

#### `Lead_Vehicle`
The car ahead driving at constant speed. Open-loop, no driver input. Will be upgraded to a dynamic profile in Phase 6 (braking, cut-in scenarios).

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Speed | 15 m/s (54 km/h) | Slower than ego → immediate closing scenario to stress-test ACC |
| Integrator IC | 50 m | Lead starts 50 m ahead — realistic highway gap at these speeds |
| Output 1 | `x_lead` [m] | Lead vehicle longitudinal position |
| Output 2 | `v_lead` [m/s] | Lead vehicle speed (constant) |

### Computed Signals (Top-Level)

| Signal | Formula | Unit | Description |
|--------|---------|------|-------------|
| `d_rel` | `x_lead − x_ego` | m | Gap between vehicles. Positive = lead is ahead. Negative = collision. |
| `delta_v` | `v_lead − v_ego` | m/s | Closing speed. Negative = ego approaching lead. Positive = gap increasing. |

> ⚠️ These are **ground-truth values**. Phase 2 will add realistic radar noise to produce measured versions for the controller.

### Solver Configuration

| Parameter | Value |
|-----------|-------|
| Solver type | Fixed-step |
| Algorithm | ode4 (Runge-Kutta) |
| Step size | 0.01 s (10 ms) |
| Stop time | 30 s |

### Assumptions & Simplifications

- 1D longitudinal motion only (no lateral dynamics or lane changes)
- Flat road assumed (no gradient forces)
- Vehicle mass effects abstracted into acceleration demand signal
- Tyre slip, actuator delays, and aerodynamic drag not modelled
- These simplifications are standard for ACC control design at this level and are documented for future extension

### Validation

**Sanity test:** `a_demand = 0` (ego coasts at constant speed, no controller).

**Expected behaviour:**
- `v_ego` constant at 20 m/s ✅
- `v_lead` constant at 15 m/s ✅
- `d_rel` decreases linearly at −5 m/s (ego faster than lead by 5 m/s) ✅
- At t = 10 s → `d_rel = 0` m (ego reaches lead position) ✅
- `d_rel` goes negative after t = 10 s (expected — no controller active yet) ✅

**Scope result:**

`d_rel` starts at +50 m and decreases to −100 m over 30 seconds — a perfectly linear slope of −5 m/s confirming correct plant dynamics.

---

## How to Run

1. Open MATLAB (R2022b or later recommended)
2. Open `ACC_Plant.slx`
3. Set solver to Fixed-step, ode4, step size 0.01
4. Connect a `Constant = 0` block to `a_demand`
5. Add a Scope to `d_rel`
6. Press **Run** (Ctrl+T)
7. Verify `d_rel` decreases linearly from 50 m ✅

---

## Requirements

- MATLAB R2022b or later
- Simulink
- Stateflow (Phase 3)
- Embedded Coder (Phase 5)
- Simulink Test (Phase 6)

---

## Portfolio Context

This project is the third in a series of Model-Based Design projects:

| # | Project | Topics Covered |
|---|---------|----------------|
| 1 | One-Pedal Vehicle | Regenerative braking, driver input logic |
| 2 | EV Torque Control | Powertrain control, torque demand management |
| 3 | **Adaptive Cruise Control** | **ADAS, longitudinal control, Stateflow, Embedded Coder** |

---

*Documentation is maintained phase-by-phase as the project progresses.*
