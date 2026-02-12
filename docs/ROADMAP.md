# AeroPerception MVP: Roadmap

> **Document status:** PoC / MVP scope — v0.1
> **Purpose:** The definitive execution plan for the AeroPerception MVP (Phases 0–4), plus Phase 5 deferred scope.
> **Relationship to docs:** This is the *what* and *when*. [MVP_INTEGRATION_GUIDE.md](./MVP_INTEGRATION_GUIDE.md) is the *how*.
> **Relationship to full architecture:** Each phase output slots directly into the full architecture without replacement. The MVP is the foundation, not a throwaway.

---

## The Core Commitment

> Build the smallest possible system that proves the full perception–planning–control loop works, in simulation, with clean code that does not get thrown away.

Every task in this roadmap exists to advance that goal. If a task doesn't, it doesn't belong here yet.

---

## Phase Summary

```
Phase 0   Repository + CI                    ~1–2 days
   │
Phase 1   PX4 SITL + Bridge + Commands       ~3–5 days
   │
Phase 2   Perception (VIO + Detector)        ~5–7 days
   │
Phase 3   Waypoint Planning                  ~3–4 days
   │
Phase 4   Mission Loop + Logging             ~3–4 days
   │
  ─┼─ MVP COMPLETE ─────────────────────────────────────────
   │
Phase 5   Simulation Hardening (deferred)    TBD
```

---

## Phase 0 — Repository and Environment Bootstrapping

**Duration:** 1–2 days
**Owner:** Project lead (solo or first contributor)

### Goal

A clean, buildable, lintable repository that any contributor can clone and build from scratch in under 30 minutes on a fresh Ubuntu 22.04 machine.

### Tasks

| # | Task | Notes |
|---|------|-------|
| 0.1 | Initialize GitHub repository with MIT or Apache 2.0 license | Choose license before first commit |
| 0.2 | Create ROS 2 workspace structure | All packages under `src/`; see MVP_COMPONENTS.md Section 1 |
| 0.3 | Create stub packages (empty but buildable) | All 7 packages: `aeroperception_interfaces`, `bringup`, `bridge`, `perception`, `planning`, `sensors`, `logging` |
| 0.4 | Define all MVP message/service types in `aeroperception_interfaces` | Do not use std_msgs hacks; define proper types now |
| 0.5 | Add `px4_msgs` as a git submodule; pin to PX4 firmware version | **Version pin is mandatory**; document it in README |
| 0.6 | Write `rosdep` dependency files for all packages | `package.xml` with all apt and pip dependencies listed |
| 0.7 | Set up GitHub Actions CI | `colcon build`, `colcon test`, `ament_flake8`, `ament_cppcheck` |
| 0.8 | Write root `README.md` | Setup instructions, architecture overview link, phase status badges |
| 0.9 | Add this roadmap and the MVP architecture docs to `/docs` | |
| 0.10 | Tag `v0.0.1-phase0` on passing CI | |

### Deliverables

- Public GitHub repository, CI green, colcon builds clean
- All MVP interface types defined and building
- README sufficient for a new contributor to set up the environment
- `/docs` directory contains all MVP architecture documents

### Definition of Done

A colleague with no prior knowledge of the project can `git clone`, follow the README, and run `colcon build` with zero errors.

---

## Phase 1 — PX4 SITL + ROS 2 Bridge + Basic Commands

**Duration:** 3–5 days
**Depends on:** Phase 0 complete

### Goal

Demonstrate programmatic flight control from ROS 2: arm → takeoff → hover → land, fully from code, no RC transmitter. This validates the entire PX4–ROS 2 communication path.

### Tasks

| # | Task | Notes |
|---|------|-------|
| 1.1 | Install PX4-Autopilot and verify `make px4_sitl gz_x500` builds | See Integration Guide Section 1.1 |
| 1.2 | Install Micro-XRCE-DDS Agent | See Integration Guide Section 1.2 |
| 1.3 | Verify `/fmu/out/*` topics appear in ROS 2 after agent connects | `ros2 topic list \| grep fmu` |
| 1.4 | Implement `bridge_node` (C++) | Frame transforms, vehicle state publisher, arm/mode services |
| 1.5 | Implement `imu_adapter_node` (C++) | `SensorCombined` → `sensor_msgs/Imu` |
| 1.6 | Write unit tests for NED↔ENU transforms | Test against known poses; CI must pass |
| 1.7 | Implement `aeroperception_bringup/launch/bridge.launch.py` | Launches XRCE agent + bridge_node |
| 1.8 | Write Phase 1 validation script: arm → takeoff → hover → land | `scripts/test_takeoff.py` |
| 1.9 | Connect QGC on UDP 14550; confirm vehicle telemetry visible | |
| 1.10 | Record Phase 1 demo bag | Replay in Foxglove; confirm altitude profile |
| 1.11 | Tag `v0.1.0-phase1` | |

### Deliverables

- `aeroperception_bridge` package: fully built, tested, CI green
- Launch file: `bridge.launch.py`
- Demo: screen recording of Gazebo + QGC showing takeoff → hover → land, triggered from ROS 2 script
- rosbag2 replay in Foxglove confirming correct ENU position values

### Definition of Done

Running `ros2 launch aeroperception_bringup bridge.launch.py` followed by `ros2 run aeroperception_bringup test_takeoff` causes the simulated vehicle to take off to 3 m, hold for 5 seconds, and land. All without touching a keyboard after the launch command.

### Key Risks

| Risk | Mitigation |
|------|-----------|
| px4_msgs version mismatch | Pin submodule hash; document in README; add CI check |
| OFFBOARD rejection | Pre-publish setpoints for ≥1 s before mode change (documented in bridge_node) |
| Gazebo Gz camera topic name changes between PX4 versions | Use `gz topic -l` to verify; update bridge config |

---

## Phase 2 — Perception Pipeline: VIO + Detector

**Duration:** 5–7 days
**Depends on:** Phase 1 complete (camera stream must be available)

### Goal

Two independent perception outputs on live simulated data:
1. `OpenVINS` → `/state/visual_odometry` at ≥30 Hz, with VIO health OK
2. `YOLOv8-nano` → `/perception/detections` at ≥5 Hz

VIO output is injected into PX4's EKF2 (EKF2 fuses it, though GPS is also available in SITL).

### Tasks

| # | Task | Notes |
|---|------|-------|
| 2.1 | Set up Gazebo camera bridge via `ros_gz_bridge` | See Integration Guide Section 2.1; verify `/camera/image_raw` topic |
| 2.2 | Get exact camera intrinsics from `/camera/camera_info` | Log `fx, fy, cx, cy` for OpenVINS config |
| 2.3 | Install OpenVINS (`ov_msckf`) as a workspace package | See Integration Guide Section 2.1 |
| 2.4 | Write OpenVINS config YAML for Gazebo X500 camera | `sim_x500.yaml`; tune IMU noise params |
| 2.5 | Write `vio_node` wrapper (C++) | Launches OpenVINS; remaps output to `/state/visual_odometry` |
| 2.6 | Write `perception_coordinator_node` (Python) | Aggregates VIO + detector health |
| 2.7 | Set PX4 EKF2 parameters for VIO fusion | Via QGC or param file |
| 2.8 | Verify EKF2 fuses VIO (check `ekf2 status` in PX4 shell) | |
| 2.9 | Export YOLOv8-nano to ONNX | `yolov8n.onnx`; store in `aeroperception_perception/models/` |
| 2.10 | Write `detector_node` (Python) | ONNX Runtime CPU inference; publishes `Detection2DArray` |
| 2.11 | Write `aeroperception_bringup/launch/perception.launch.py` | Launches camera bridge + vio + detector + coordinator |
| 2.12 | Record Phase 2 demo bag | VIO pose + detection overlay in Foxglove |
| 2.13 | Tag `v0.2.0-phase2` | |

### Deliverables

- `aeroperception_perception` package: vio_node, detector_node, perception_coordinator_node
- Launch file: `perception.launch.py`
- Demo: Foxglove replay showing VIO pose track and YOLOv8 detections on camera feed
- OpenVINS config file committed to repository

### Definition of Done

After `ros2 launch aeroperception_bringup perception.launch.py`:
- `ros2 topic hz /state/visual_odometry` reports ≥30 Hz
- `/state/vio_health` shows `level: 0` (OK) within 30 s of launch
- `ros2 topic hz /perception/detections` reports ≥5 Hz
- VIO position tracks vehicle movement (command a hover, then yaw — the odom pose should change)

### Key Risks

| Risk | Mitigation |
|------|-----------|
| OpenVINS fails to initialize in simulation | Check camera info matches config; try gentle yaw rotation command to give parallax |
| EKF2 rejects VIO (bad covariance) | Tune `EKF2_EV_NOISE_MD` and VIO output covariance in OpenVINS config |
| YOLOv8 runs too slowly | Reduce input resolution to 416×416; reduce inference rate to 5 Hz |
| Gazebo camera topic name differs from expected | `gz topic -l` to find actual name; update bridge YAML |

---

## Phase 3 — Waypoint Planning

**Duration:** 3–4 days
**Depends on:** Phase 1 complete (Phase 2 running in parallel is fine; planner works with GPS-only state too)

### Goal

The vehicle flies to a sequence of 4+ waypoints autonomously using the waypoint planner. No operator intervention after mission start.

### Tasks

| # | Task | Notes |
|---|------|-------|
| 3.1 | Implement `waypoint_planner_node` (C++) | Linear interpolation + velocity capping; see Components doc Section 5.2 |
| 3.2 | Verify planner publishes at 50 Hz with correct NED setpoints via bridge | Check `/fmu/in/trajectory_setpoint` values |
| 3.3 | Test single-waypoint flight (manual goal publish) | `ros2 topic pub /mission/active_goal ...` |
| 3.4 | Add waypoint acceptance radius logic | Vehicle must stay within 0.5 m of waypoint before "reached" |
| 3.5 | Add a static obstacle box to Gazebo world | See Integration Guide Section 3.3; confirm vehicle clears it by altitude |
| 3.6 | Write `aeroperception_bringup/launch/planning.launch.py` | Launches bridge + planner |
| 3.7 | Record Phase 3 demo bag | 4-waypoint flight path visible in Foxglove 3D panel |
| 3.8 | Tag `v0.3.0-phase3` | |

### Deliverables

- `aeroperception_planning` package: `waypoint_planner_node`
- Launch file: `planning.launch.py`
- Demo: 4-waypoint mission in Gazebo, Foxglove path visualization confirming accuracy

### Definition of Done

Vehicle reaches all 4 waypoints in a pre-defined test route, with each waypoint reached within the acceptance radius, at constant altitude, in <60 seconds total flight time.

### Key Risks

| Risk | Mitigation |
|------|-----------|
| Velocity overshoot past waypoints | Tune `kp` (proportional gain) and `max_velocity` in planner config |
| Yaw control oscillation | Limit yaw rate; P controller on yaw error |
| Setpoint integration drift | Planner must use fresh `vehicle_state` each cycle; no integrating old positions |

---

## Phase 4 — Mission Logic + Full Loop Integration

**Duration:** 3–4 days
**Depends on:** Phases 1, 2, 3 all complete

### Goal

The Mission Manager FSM drives a complete mission from a single `ros2 service call`. The full pipeline — from PX4 state, through perception health gating, to trajectory planning, to setpoints — works end-to-end. Everything is logged and replayable.

### Tasks

| # | Task | Notes |
|---|------|-------|
| 4.1 | Implement `mission_manager_node` (Python) | 5-state FSM; see Components doc Section 5.1 |
| 4.2 | Implement `/mission/load` and `/mission/start` services | JSON-deserialized waypoint list |
| 4.3 | Wire perception health gate into TAKEOFF → EXECUTING transition | Do not enter EXECUTING until `overall_healthy = true` |
| 4.4 | Write `aeroperception_bringup/launch/simulation.launch.py` | One command launches everything in correct order |
| 4.5 | Configure rosbag2 MCAP recorder with essentials preset | Auto-starts with simulation.launch.py |
| 4.6 | Run end-to-end: `simulation.launch.py` → `load mission` → `start` → fly → land | |
| 4.7 | Replay bag in Foxglove; verify all key topics present and correct | See Integration Guide Section 4.3 |
| 4.8 | Write MVP acceptance test (automated or documented checklist) | See Integration Guide Section 4.4 |
| 4.9 | Record Phase 4 demo video | Screen capture: Gazebo + QGC + Foxglove replay |
| 4.10 | Write post-MVP retrospective (1 page): what worked, what surprised us, what changes for Phase 5 | |
| 4.11 | Tag `v0.4.0-mvp` | This is the portfolio-ready deliverable |

### Deliverables

- `aeroperception_planning` package: `mission_manager_node`
- `aeroperception_bringup/launch/simulation.launch.py` — single-command launch
- rosbag2 bag of a complete Phase 4 demo mission
- Foxglove layout file (`.json`) configured for mission replay
- Demo video
- Post-MVP retrospective document
- GitHub Release: `v0.4.0-mvp` with release notes

### Definition of Done

**The Phase 4 Acceptance Test** (from Integration Guide Section 4.4):

From a fresh terminal (workspace already built):
```bash
# Terminal 1
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2
MicroXRCEAgent udp4 -p 8888

# Terminal 3
ros2 launch aeroperception_bringup simulation.launch.py

# Terminal 4 (after ~30 s for initialization)
ros2 service call /mission/load ...  # phase4_demo mission
ros2 service call /mission/start ...
```

Pass criteria:
- [ ] Vehicle arms, takes off to 5 m
- [ ] VIO health is OK before EXECUTING state is entered
- [ ] Vehicle reaches all 5 waypoints within acceptance radius
- [ ] Vehicle lands and disarms
- [ ] `/mission/status` transitions through: IDLE → TAKEOFF → EXECUTING → LAND
- [ ] rosbag2 bag written without errors
- [ ] Foxglove replay shows correct position, VIO pose, and mission status timeline

---

## Phase 5 — Simulation Hardening (Deferred)

**Duration:** TBD — begins after Phase 4 tag
**This phase is not MVP scope.** It is documented here to show the growth path.

### Goals

1. Make SITL more representative of hardware conditions
2. Evaluate robustness of VIO and detector under varied conditions
3. Validate architectural assumptions made in MVP
4. Open the door to hardware flight

### Planned Tasks

| Area | Task |
|------|------|
| **Perception hardening** | Add camera distortion, lens noise, rolling shutter to Gazebo camera model |
| **IMU hardening** | Add realistic IMU noise (Brownian noise, bias instability) to Gazebo IMU |
| **Lighting variation** | Simulate dawn, midday, dusk, artificial lighting in Gazebo world |
| **VIO benchmark** | Run OpenVINS and ORB-SLAM3 against EuRoC dataset; compare to SITL performance |
| **Detection benchmark** | Measure YOLOv8-nano FPS under CPU load; evaluate on-Jetson inference path |
| **Planner upgrade** | Replace linear interpolator with A* or RRT* for static obstacle avoidance |
| **FSM upgrade** | Extend to full 9-state FSM with degraded-mode recovery |
| **MAVROS fallback** | Document and test MAVROS bridge as an alternative |
| **ORB-SLAM3 backend** | Add as pluggable VIO backend for loop-closure evaluation |
| **Hardware checklist** | Document SITL assumptions; create hardware validation plan |

---

## What This Roadmap Deliberately Excludes

These are not forgotten. They are deferred to the full architecture phase after a successful MVP.

| Feature | When |
|---------|------|
| Hardware flight | After Phase 5 simulation hardening |
| Encrypted communications | Before any hardware field test |
| Terrain-aware planning | Full architecture Phase 1 |
| Thermal / night perception | Full architecture Phase 2 |
| Precision landing | Full architecture Phase 2 |
| Multi-UAV fleet | Full architecture Phase 3 |
| Security / surveillance behaviors | Full architecture Phase 2–3 |
| Forensic logging | Full architecture Phase 2 |

---

## Tracking Progress

Each phase has a corresponding GitHub Project board column. Issues are labeled by phase (`phase-0`, `phase-1`, etc.). Milestones are set to phase completion tags.

**Current status:** Phase 0 — in progress

---

*This roadmap is a living document. It will be updated at the end of each phase with lessons learned and any scope adjustments.*

*Companion documents: [MVP_ARCHITECTURE_OVERVIEW.md](./MVP_ARCHITECTURE_OVERVIEW.md) | [MVP_COMPONENTS.md](./MVP_COMPONENTS.md) | [MVP_DESIGN_DECISIONS.md](./MVP_DESIGN_DECISIONS.md) | [MVP_INTEGRATION_GUIDE.md](./MVP_INTEGRATION_GUIDE.md)*
