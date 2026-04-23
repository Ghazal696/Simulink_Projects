# Adaptive Cruise Control (ACC) — Model-Based Design in MATLAB/Simulink

**Author:** Ghazal Ghorbani  
**Tools:** MATLAB, Simulink, Stateflow, Embedded Coder, Simulink Test  
**Domain:** Automotive ADAS — Longitudinal Control  
**Status:** 🟡 In Progress — Phase 3 Complete

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
│         │            │ACC_Controller│                  │
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
├── ACC_Controller.slx      ← Phase 3: Stateflow state machine
├── ACC_PID.slx             ← Phase 4: PID controller (pending)
├── ACC_System.slx          ← Full integrated system (pending)
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
| 2 | Radar Sensor Model | Simulink | ✅ Complete |
| 3 | ACC State Machine | Stateflow | ✅ Complete |
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

> ⚠️ These are **ground-truth values**. Phase 2 adds realistic radar noise to produce measured versions for the controller.

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

## Phase 2 — Radar Sensor Model (`ACC_Sensor.slx`)

### Purpose

Models the ACC system's front-facing radar sensor. Takes ground-truth values from the plant and adds realistic measurement noise, producing the **measured signals** that the controller will actually use. No sensor in the real world is perfect — this model captures that imperfection faithfully.

This phase sits between the physical world (Phase 1) and the decision logic (Phase 3), reflecting the standard automotive signal chain: plant → sensor → controller.

### Signal Chain

```
ACC_Plant (ground truth)     ACC_Sensor (noisy measured)     ACC_Controller
  d_rel  [m]      ──────►   d_rel_meas  [m]      ──────►   (Phase 3+4)
  delta_v [m/s]   ──────►   delta_v_meas [m/s]   ──────►
```

### Subsystem: `Radar_Sensor`

#### Distance Channel (`d_rel_meas`)

| Block | Parameter | Value | Rationale |
|-------|-----------|-------|-----------|
| Band-Limited White Noise | Noise power | 0.0009 | Produces ±0.3 m RMS error — matches real 77 GHz automotive radar specs |
| Band-Limited White Noise | Sample time | 0.01 s | Aligned with plant solver step (10 ms) |
| Add | — | `d_rel + noise` | Combines true distance with sensor noise |
| Saturation | Upper limit | 200 m | Physical radar detection range limit |
| Saturation | Lower limit | 0 m | Distance cannot be negative |

#### Closing Speed Channel (`delta_v_meas`)

| Block | Parameter | Value | Rationale |
|-------|-----------|-------|-----------|
| Band-Limited White Noise | Noise power | 0.0001 | Produces ±0.1 m/s RMS error — matches real radar Doppler accuracy |
| Band-Limited White Noise | Sample time | 0.01 s | Aligned with plant solver step |
| Add | — | `delta_v + noise` | Combines true closing speed with sensor noise |
| Saturation | Upper limit | +50 m/s | Physical radar velocity range limit |
| Saturation | Lower limit | −50 m/s | Physical radar velocity range limit |

### Design Parameters

| Parameter | Value | Basis |
|-----------|-------|-------|
| Distance noise (RMS) | ±0.3 m | Typical 77 GHz automotive radar range accuracy |
| Velocity noise (RMS) | ±0.1 m/s | Typical 77 GHz automotive radar Doppler accuracy |
| Detection range | 0–200 m | Standard long-range ACC radar detection envelope |
| Velocity range | ±50 m/s | Covers all realistic highway speed differentials |

### Validation

**Sanity test:** Connect Phase 1 plant outputs directly. Run 10 s simulation with ego coasting.

**Expected scope behaviour:**
- `d_rel_meas` (yellow): decreases from ~50 m to ~0 m with tight ±0.3 m noise jitter ✅
- `delta_v_meas` (blue): flat line at ~−5 m/s with tight ±0.1 m/s noise jitter ✅
- Overall trend follows ground truth closely — noise visible but not dominant ✅

---

## Phase 3 — ACC State Machine (`ACC_Controller.slx`)

### Purpose

Implements the **operating mode logic** of the ACC system as a Stateflow hierarchical state machine. Reads sensor measurements and determines which control law the PID controller (Phase 4) should apply at every instant.

This is the decision-making brain of the system — it answers the question: *"What should the car do right now?"*

### Stateflow Chart: `ACC_StateMachine`

#### Inputs

| Signal | Unit | Source | Description |
|--------|------|--------|-------------|
| `d_rel_meas` | m | ACC_Sensor | Measured distance to lead vehicle |
| `delta_v_meas` | m/s | ACC_Sensor | Measured closing speed *(reserved for Phase 4)* |
| `v_ego` | m/s | ACC_Plant | Ego vehicle speed *(reserved for Phase 4)* |
| `ACC_enable` | — | Driver | ACC on/off switch (1 = on, 0 = off) |

#### Outputs

| Signal | Unit | Description |
|--------|------|-------------|
| `ACC_mode` | — | Current operating state (0–4) |
| `v_set` | m/s | Desired cruising speed reference for PID |
| `d_set` | m | Desired following distance reference for PID |

#### States

| State | ACC_mode | Condition to Enter | Behaviour |
|-------|----------|--------------------|-----------|
| `OFF` | 0 | Default / `ACC_enable == 0` | System inactive. All outputs zero. |
| `STANDBY` | 1 | `ACC_enable == 1` | ACC on, assessing road condition. |
| `SPEED_CONTROL` | 2 | `d_rel_meas > 80 m` | Road clear. Maintain `v_set = 30 m/s`. |
| `FOLLOW_MODE` | 3 | `10 m < d_rel_meas ≤ 80 m` | Lead detected. Maintain `d_set = 50 m`. |
| `EMERGENCY_BRAKE` | 4 | `d_rel_meas ≤ 10 m` | Critical gap. Maximum braking. |

#### Transition Table

| From | To | Condition |
|------|----|-----------|
| `OFF` | `STANDBY` | `[ACC_enable == 1]` |
| `STANDBY` | `OFF` | `[ACC_enable == 0]` |
| `STANDBY` | `SPEED_CONTROL` | `[d_rel_meas > 80]` |
| `STANDBY` | `FOLLOW_MODE` | `[d_rel_meas <= 80 && d_rel_meas > 10]` |
| `SPEED_CONTROL` | `STANDBY` | `[ACC_enable == 0]` |
| `SPEED_CONTROL` | `FOLLOW_MODE` | `[d_rel_meas <= 80 && d_rel_meas > 10]` |
| `FOLLOW_MODE` | `SPEED_CONTROL` | `[d_rel_meas > 80]` |
| `FOLLOW_MODE` | `EMERGENCY_BRAKE` | `[d_rel_meas <= 10]` |
| `EMERGENCY_BRAKE` | `FOLLOW_MODE` | `[d_rel_meas > 10 && d_rel_meas <= 80]` |
| `EMERGENCY_BRAKE` | `STANDBY` | `[ACC_enable == 0]` |

#### Distance Threshold Rationale

| Threshold | Value | Rationale |
|-----------|-------|-----------|
| Road clear boundary | 80 m | At 20 m/s, 80 m = 4 s lookahead — sufficient time to react before a lead vehicle becomes relevant |
| Follow mode boundary | 10 m | Minimum safe gap before human-level reaction is insufficient |
| Emergency brake trigger | ≤ 10 m | At 20 m/s, 10 m = 0.5 s to collision — well below human reaction time (~1.5 s). Automated emergency braking is the only option |

#### Design Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `v_set` | 30 m/s (108 km/h) | Realistic European highway cruising speed. Faster than both vehicles to create meaningful controller behaviour. |
| `d_set` | 50 m | ISO 15622 time-gap law: `d_set = 5 + 2.5 × v_ego ≈ 50 m` at following speed ~18 m/s. Becomes dynamic in Phase 4. |

> 📝 **Note on unused inputs:** `delta_v_meas` and `v_ego` are declared but currently unused in the Stateflow transition logic. They are reserved for Phase 4 where: `delta_v_meas` will enable combined distance+velocity emergency brake conditions, and `v_ego` will be used for dynamic `d_set` computation and speed error in the PID controller.

### Validation

**Test constants used for isolated Phase 3 testing:**

| Input | Value | Expected result |
|-------|-------|-----------------|
| `d_rel_meas` | 45 m | Between 10–80 m → FOLLOW_MODE |
| `delta_v_meas` | −5 m/s | Closing in on lead |
| `v_ego` | 30 m/s | At set speed |
| `ACC_enable` | 1 | ACC switched ON |

**Expected `ACC_mode` sequence:** `0 → 1 → 3` (OFF → STANDBY → FOLLOW_MODE) ✅

---

## How to Run

### Phase 1 (Plant only)
1. Open `ACC_Plant.slx`
2. Connect `Constant = 0` to `a_demand`
3. Add Scope to `d_rel` and `delta_v`
4. Run (Ctrl+T) — verify linear decrease of `d_rel` at −5 m/s ✅

### Phase 2 (Sensor only)
1. Open `ACC_Sensor.slx`
2. Connect Phase 1 outputs to sensor inputs (or use test constants)
3. Add Scope to `d_rel_meas` and `delta_v_meas`
4. Run — verify noisy signals track ground truth with ±0.3 m / ±0.1 m/s jitter ✅

### Phase 3 (State machine only)
1. Open `ACC_Controller.slx`
2. Test constants already wired: `d_rel_meas=45`, `delta_v_meas=-5`, `v_ego=30`, `ACC_enable=1`
3. Add Scope to `ACC_mode`
4. Run — verify `ACC_mode` settles at 3 (FOLLOW_MODE) ✅

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
