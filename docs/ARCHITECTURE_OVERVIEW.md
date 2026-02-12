# AeroPerception MVP: Architecture Overview

> **Document status:** PoC / MVP scope — v0.1
> **Relationship to full architecture:** This document is a scoped subset of [ARCHITECTURE_OVERVIEW.md](./ARCHITECTURE_OVERVIEW.md). Every decision made here is intentionally compatible with that north-star document. Where we cut scope, we say so explicitly and leave the growth hook visible.
> **Goal:** Get a flying, perceiving, autonomously navigating vehicle into SITL as fast as possible, with clean code that does not need to be thrown away.

---

## Table of Contents

- [AeroPerception MVP: Architecture Overview](#aeroperception-mvp-architecture-overview)
  - [Table of Contents](#table-of-contents)
  - [1. What the MVP Proves](#1-what-the-mvp-proves)
  - [2. What the MVP Does Not Try to Do](#2-what-the-mvp-does-not-try-to-do)
  - [3. Subsystem Map](#3-subsystem-map)
  - [4. The Minimal Viable Stack](#4-the-minimal-viable-stack)
    - [Why uXRCE-DDS, not MAVROS?](#why-uxrce-dds-not-mavros)
  - [5. Communication Architecture](#5-communication-architecture)
  - [6. Onboard vs Offboard for MVP](#6-onboard-vs-offboard-for-mvp)
  - [7. Design Principles for Fast Execution](#7-design-principles-for-fast-execution)
    - [7.1 One Interface Package, Zero Shortcuts](#71-one-interface-package-zero-shortcuts)
    - [7.2 Every Node is a Lifecycle Node](#72-every-node-is-a-lifecycle-node)
    - [7.3 Keep the Hot Path Explicit](#73-keep-the-hot-path-explicit)
    - [7.4 SITL is Not Hardware — Know the Difference](#74-sitl-is-not-hardware--know-the-difference)
    - [7.5 Commit Working States](#75-commit-working-states)
  - [8. MVP Block Diagram](#8-mvp-block-diagram)
  - [9. Data Flow](#9-data-flow)
    - [Upward (Sensors → Commands → FC)](#upward-sensors--commands--fc)
    - [Downward (FC State → ROS 2 → Operator)](#downward-fc-state--ros-2--operator)
  - [10. Key Assumptions and Accepted Risks](#10-key-assumptions-and-accepted-risks)
  - [11. Phase-to-Phase Growth Path](#11-phase-to-phase-growth-path)

---

## 1. What the MVP Proves

The MVP (Minimum Viable Product / Proof of Concept) answers five questions with running code:

| Question                                                   | Demonstrated By                                                    |
| ---------------------------------------------------------- | ------------------------------------------------------------------ |
| Can PX4 be controlled programmatically from ROS 2?         | Bridge node arms, takes off, flies to waypoints, lands             |
| Can monocular perception produce a usable pose estimate?   | OpenVINS VIO running on simulated camera stream                    |
| Can a learned detector run in the ROS 2 graph?             | YOLOv8-nano publishing detections on a live image topic            |
| Can a simple planner close the perception-to-control loop? | Trajectory planner consuming VIO pose, publishing setpoints to PX4 |
| Can we record and replay a complete mission?               | rosbag2 + MCAP captures everything; Foxglove plays it back         |

If all five are demonstrable in simulation by end of Phase 4, the MVP is complete.

---

## 2. What the MVP Does Not Try to Do

Explicitly out of scope for the PoC. These are not forgotten — they are deferred to the full architecture.

| Out of Scope                           | Reason for Deferral                             |
| -------------------------------------- | ----------------------------------------------- |
| Hardware flight                        | SITL-only until stack is validated              |
| Thermal / night perception             | Requires specialized hardware                   |
| Encrypted comms / VPN                  | Adds setup complexity; not needed in simulation |
| Precision landing / dock return        | Phase 2+ feature                                |
| Multi-UAV fleet                        | Single-vehicle scope                            |
| Terrain-aware DEM planning             | Flat-world assumption acceptable in SITL        |
| Forensic / tamper-evident logging      | Not needed for PoC                              |
| Fully hardened failsafes               | PX4 native failsafes handle SITL safety         |
| Security / surveillance behaviors      | Full use case is Phase 2+                       |
| GPU inference at production throughput | SITL tolerates lower frame rates                |

---

## 3. Subsystem Map

The MVP uses the same six-subsystem decomposition as the full architecture, but each subsystem is its leanest viable implementation.

```
┌─────────────────────────────────────────────────────────────────┐
│                   AEROPERCEPTION MVP STACK                      │
│                                                                 │
│  ┌────────────┐  ┌────────────┐  ┌──────────────────────────┐   │
│  │  PX4 SITL  │  │  Bridge    │  │  Perception              │   │
│  │  (Gazebo   │◄─│  (uXRCE-   │◄─│  OpenVINS VIO            │   │
│  │   Gz)      │  │   DDS)     │  │  YOLOv8-nano detector    │   │
│  └────────────┘  └────────────┘  └──────────────────────────┘   │
│                                             │                   │
│  ┌────────────┐  ┌────────────┐  ┌──────────▼───────────────┐   │
│  │  QGC GCS   │  │  rosbag2   │  │  Waypoint Planner +      │   │
│  │  (monitor  │  │  + MCAP    │  │  Mission Manager         │   │
│  │   only)    │  │  logging   │  │  (5-state machine)       │   │
│  └────────────┘  └────────────┘  └──────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. The Minimal Viable Stack

One concrete choice per subsystem. No optionality for now — optionality comes in Phase 5+.

| Subsystem          | MVP Implementation                            | Full Architecture Equivalent                  |
| ------------------ | --------------------------------------------- | --------------------------------------------- |
| Flight controller  | PX4 SITL (Gazebo Gz, X500 model)              | PX4 on Pixhawk hardware                       |
| Bridge             | uXRCE-DDS + Micro-XRCE-DDS Agent              | Same (uXRCE preferred)                        |
| VIO / odometry     | OpenVINS (monocular, no loop closure)         | Pluggable: OpenVINS / ORB-SLAM3 / VINS-Fusion |
| Object detection   | YOLOv8-nano (ONNX Runtime, CPU)               | YOLOv8 / RT-DETR / Grounding DINO             |
| Trajectory planner | Linear waypoint interpolator                  | Min-snap / MPPI                               |
| Mission manager    | 5-state FSM (IDLE→TAKEOFF→MISSION→HOLD→LAND)  | Full 9-state FSM with degraded modes          |
| Logging            | rosbag2 + MCAP, essentials preset             | Dual-stream + forensic chaining               |
| GCS                | QGroundControl (monitoring + manual override) | QGC + custom dashboard                        |
| Simulation         | Gazebo Gz Harmonic + ros_gz_bridge            | Isaac Sim for photorealistic future work      |

### Why uXRCE-DDS, not MAVROS?

The original roadmap draft listed MAVROS. We are using **uXRCE-DDS** instead, because:

1. It is PX4's native bridge and carries less latency (~2–5 ms vs ~10–15 ms)
2. It exposes the full uORB topic set, not just the MAVLink subset
3. It is the direction PX4 is investing in; MAVROS investment has diminishing returns
4. The setup delta is small (one extra `MicroXRCEAgent` process)

MAVROS remains available as a fallback and is documented in the integration guide for anyone who needs it.

---

## 5. Communication Architecture

Three communication planes, all in software for SITL:

```
┌───────────────────────────────────────────────────────────────────┐
│  Plane 1: FC ↔ Companion (uXRCE-DDS over UDP loopback in SITL)   │
│                                                                    │
│  PX4 SITL process ──UDP:8888──► MicroXRCEAgent ──DDS──► ROS 2    │
│  /fmu/out/* topics visible; /fmu/in/* subscriptions active        │
│                                                                    │
├───────────────────────────────────────────────────────────────────┤
│  Plane 2: Intra-ROS 2 (DDS, CycloneDDS)                          │
│                                                                    │
│  All nodes in the same ROS 2 domain (domain ID 0 for SITL)       │
│  Hot path: Bridge → Planner → Bridge at 50 Hz, same process       │
│                                                                    │
├───────────────────────────────────────────────────────────────────┤
│  Plane 3: Companion ↔ GCS (MAVLink/UDP, via mavlink-router)      │
│                                                                    │
│  QGC connects on UDP 14550                                        │
│  Monitor only in MVP (operator does not command via QGC)          │
└───────────────────────────────────────────────────────────────────┘
```

**DDS middleware:** `rmw_cyclonedds_cpp`. Set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in all terminal sessions. This is non-negotiable for reliable multi-process operation.

---

## 6. Onboard vs Offboard for MVP

In SITL everything runs on one machine. The logical boundary still matters because it determines what must be preserved when we move to hardware.

| Component            | Logical Location       | Notes                                |
| -------------------- | ---------------------- | ------------------------------------ |
| PX4 SITL             | Onboard (simulated FC) | Will be hardware FC on vehicle       |
| MicroXRCEAgent       | Onboard (companion)    | Stays on companion                   |
| OpenVINS VIO         | Onboard (companion)    | Latency-sensitive; stays onboard     |
| YOLOv8-nano detector | Onboard (companion)    | CPU inference; acceptable at 5–10 Hz |
| Waypoint planner     | Onboard (companion)    | Must survive link loss               |
| Mission manager      | Onboard (companion)    | Must survive link loss               |
| rosbag2 recorder     | Onboard (companion)    | Records to local storage             |
| QGC                  | Offboard (GCS machine) | Monitoring only                      |
| Foxglove playback    | Offboard (developer)   | Post-flight analysis                 |

**The rule that must not be broken even in SITL:** The vehicle must be able to complete its current mission segment, hold, and land safely with QGC closed. Never build logic that makes GCS connectivity a safety dependency.

---

## 7. Design Principles for Fast Execution

### 7.1 One Interface Package, Zero Shortcuts

All custom messages, services, and actions live in `aeroperception_interfaces` from day one. Do not publish `std_msgs/String` as a workaround for missing types. The cost of defining a proper interface once is less than the cost of refactoring later.

### 7.2 Every Node is a Lifecycle Node

Use `rclcpp_lifecycle::LifecycleNode` for all nodes. This costs ~30 minutes extra upfront and saves hours of "why is my node starting before the bridge is ready" debugging.

### 7.3 Keep the Hot Path Explicit

The critical path is: VIO → Planner → Bridge → PX4. Every node on this path publishes its timestamp. Latency on this path is logged to `/diagnostics` from the first day. If you can't measure it, you can't fix it.

### 7.4 SITL is Not Hardware — Know the Difference

Keep a running list of SITL simplifications (perfect GPS, no IMU noise, no USB latency). This list becomes the hardware validation checklist in Phase 5.

### 7.5 Commit Working States

Every phase ends with a tagged git commit that launches cleanly from scratch. "It worked on my machine" is not a deliverable.

---

## 8. MVP Block Diagram

```
                    ┌─────────────────────────────────────────────────┐
                    │         DEVELOPER WORKSTATION (SITL)             │
                    │                                                   │
┌──────────────┐   │  ┌──────────────┐    ┌──────────────────────┐   │
│  Gazebo Gz   │   │  │              │    │  Perception Stack     │   │
│  Physics +   │◄──┼──│  PX4 SITL   │    │                      │   │
│  Camera      │   │  │  (process)  │    │  OpenVINS VIO        │   │
│  sensor      ├───┼──►             │    │  /state/visual_odom  │   │
└──────────────┘   │  │  uXRCE-DDS  │    │                      │   │
                    │  │  Client     │    │  YOLOv8-nano         │   │
                    │  └──────┬──────┘    │  /perception/detects │   │
                    │         │ UDP:8888   └──────────┬───────────┘   │
                    │  ┌──────▼──────┐               │               │
                    │  │  XRCE       │               │ /state/        │
                    │  │  Agent      │               │ visual_odom    │
                    │  └──────┬──────┘               │               │
                    │         │ DDS                   │               │
                    │  ┌──────▼──────────────────────▼─────────────┐ │
                    │  │          ROS 2 DDS Domain                  │ │
                    │  │                                            │ │
                    │  │  ┌──────────────┐  ┌──────────────────┐   │ │
                    │  │  │  Bridge Node │  │  Mission Manager │   │ │
                    │  │  │  - NED↔ENU  │  │  5-state FSM    │   │ │
                    │  │  │  - mode svc │  │  - arm/takeoff  │   │ │
                    │  │  │  - heartbeat│  │  - execute plan │   │ │
                    │  │  └──────┬───────┘  │  - hold/land   │   │ │
                    │  │         │           └────────┬─────────┘   │ │
                    │  │  ┌──────▼───────────────────▼──────────┐   │ │
                    │  │  │  Waypoint Planner                   │   │ │
                    │  │  │  - Linear interpolation             │   │ │
                    │  │  │  - Velocity-limited setpoints       │   │ │
                    │  │  │  - Publishes /fmu/in/trajectory_    │   │ │
                    │  │  │    setpoint @ 50 Hz                 │   │ │
                    │  │  └─────────────────────────────────────┘   │ │
                    │  │                                            │ │
                    │  │  ┌──────────────────────────────────────┐  │ │
                    │  │  │  rosbag2 Recorder (MCAP, essentials) │  │ │
                    │  │  └──────────────────────────────────────┘  │ │
                    │  └────────────────────────────────────────────┘ │
                    └─────────────────────────────────────────────────┘
                                          │ UDP:14550
                                   ┌──────▼──────┐
                                   │  QGC (GCS)  │
                                   │  monitoring │
                                   └─────────────┘
```

---

## 9. Data Flow

### Upward (Sensors → Commands → FC)

```
Gazebo camera plugin
  → ros_gz_bridge
  → /camera/image_raw          (30 Hz)

PX4 IMU (via bridge)
  → /fmu/out/sensor_combined   (100 Hz, used for VIO IMU)

/camera/image_raw  ┐
/imu/data          ├──► OpenVINS ──► /state/visual_odometry (50 Hz)

/camera/image_raw ──► YOLOv8-nano ──► /perception/detections (10 Hz)

/state/visual_odometry ──► Waypoint Planner ──► /fmu/in/trajectory_setpoint (50 Hz)
                                              → /fmu/in/offboard_control_mode (50 Hz)
```

### Downward (FC State → ROS 2 → Operator)

```
PX4 SITL
  → uXRCE-DDS
  → /fmu/out/vehicle_local_position  (50 Hz)
  → /fmu/out/vehicle_status          (5 Hz)
  → /fmu/out/battery_status          (1 Hz)

All of the above ──► rosbag2 writer ──► flight_YYYYMMDD_HHMMSS.mcap

/fmu/out/* ──► mavlink-router ──► QGC (UDP:14550)
```

---

## 10. Key Assumptions and Accepted Risks

| #   | Assumption                                                         | Accepted Risk                                        |
| --- | ------------------------------------------------------------------ | ---------------------------------------------------- |
| A1  | SITL camera produces clean frames (no motion blur, ideal exposure) | VIO will need re-tuning for real cameras             |
| A2  | Flat world in simulation; no terrain                               | Terrain-aware planning deferred to Phase 5+          |
| A3  | GPS always available in SITL                                       | GPS-denied scenarios not tested until hardware phase |
| A4  | Single vehicle; no fleet                                           | Coordination logic not developed                     |
| A5  | CPU inference acceptable at 5–10 Hz                                | Higher frame rates require GPU on hardware           |
| A6  | No network security in SITL                                        | Encrypted comms added before any hardware field test |
| A7  | Developer machine has ≥16 GB RAM, 4+ cores                         | SITL + Gazebo + full ROS 2 stack is memory-hungry    |

---

## 11. Phase-to-Phase Growth Path

The MVP phases correspond directly to the roadmap. Each phase builds on the last without breaking existing interfaces.

```
Phase 0  Repository + CI
   │     ─────────────────────────────────────────────────────
   │     Deliverable: Clean workspace, linting CI, base README
   ▼
Phase 1  PX4 SITL + Bridge + Basic Commands
   │     ─────────────────────────────────────────────────────
   │     Deliverable: Arm → Takeoff → Hover → Land from ROS 2
   ▼
Phase 2  Perception Pipeline (VIO + Detector)
   │     ─────────────────────────────────────────────────────
   │     Deliverable: /state/visual_odometry + /perception/detections
   │     publishing on simulated camera stream
   ▼
Phase 3  Planning + Waypoint Navigation
   │     ─────────────────────────────────────────────────────
   │     Deliverable: Fly to N waypoints autonomously, avoiding
   │     static obstacles in Gazebo scene
   ▼
Phase 4  Mission Logic + Full Loop
   │     ─────────────────────────────────────────────────────
   │     Deliverable: Mission manager drives full mission;
   │     rosbag2 records and replays in Foxglove
   ▼
Phase 5  Simulation Hardening (Deferred)
         ─────────────────────────────────────────────────────
         Lighting variation, sensor noise, benchmark suite
         Gateway to hardware validation
```

**After Phase 4, the full architecture documents take over.** Every component built in the MVP has been designed to slot directly into the full system without replacement.

---

*Companion documents: [MVP_COMPONENTS.md](./MVP_COMPONENTS.md) | [MVP_DESIGN_DECISIONS.md](./MVP_DESIGN_DECISIONS.md) | [MVP_INTEGRATION_GUIDE.md](./MVP_INTEGRATION_GUIDE.md) | [MVP_ROADMAP.md](./MVP_ROADMAP.md)*
