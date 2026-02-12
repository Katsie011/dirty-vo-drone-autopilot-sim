# AeroPerception MVP: Design Decisions

> **Document status:** PoC / MVP scope — v0.1
> **Relationship to full architecture:** Each ADR here is either a scoped version of a full ADR, a new PoC-specific decision, or an explicit statement of what we are deferring and why.
> **Format:** Context → Options → Decision → Rationale → Consequences → Deferred

---

## Table of Contents

| ADR | Title |
|-----|-------|
| [MVP-ADR-001](#mvp-adr-001) | Programming Language per Node |
| [MVP-ADR-002](#mvp-adr-002) | Bridge: uXRCE-DDS vs MAVROS (Reconciling the Roadmap) |
| [MVP-ADR-003](#mvp-adr-003) | VIO Backend: OpenVINS as Concrete Default |
| [MVP-ADR-004](#mvp-adr-004) | Detector: YOLOv8-nano on CPU (ONNX) |
| [MVP-ADR-005](#mvp-adr-005) | Planner Complexity: Linear Interpolation First |
| [MVP-ADR-006](#mvp-adr-006) | Mission FSM: 5 States, Not 9 |
| [MVP-ADR-007](#mvp-adr-007) | Depth Estimation: Deferred from MVP |
| [MVP-ADR-008](#mvp-adr-008) | Camera: Simulated Gazebo Camera, Not MiDaS/ZoeDepth |
| [MVP-ADR-009](#mvp-adr-009) | State Machine Implementation: Python, Not SMACH or BehaviorTree.CPP |
| [MVP-ADR-010](#mvp-adr-010) | Coordinate Frame Contract: ENU Everywhere in ROS 2 |
| [MVP-ADR-011](#mvp-adr-011) | Test Strategy for PoC |

---

## MVP-ADR-001

### Programming Language per Node

**Status:** Accepted

#### Context

ROS 2 supports C++ (`rclcpp`) and Python (`rclpy`). Both are first-class. The choice per node affects development speed, runtime performance, and maintainability.

#### Options Considered

- All C++: maximum performance; slower to write and debug; harder for prototyping logic
- All Python: fast prototyping; insufficient for 50 Hz hard-real-time-ish loops
- Mixed by node responsibility: use the right tool per role

#### Decision

**Mixed, by node role:**

| Role | Language | Reason |
|------|----------|--------|
| Bridge node | C++ | On 50 Hz hot path; zero-copy DDS messages matter |
| IMU adapter | C++ | 200 Hz; Python is too slow |
| Waypoint planner | C++ | 50 Hz publish loop |
| VIO (OpenVINS) | C++ | Upstream is C++; thin wrapper only |
| Detector node | Python | 10 Hz is comfortable for Python; ONNX Runtime Python API is mature |
| Mission manager | Python | State machine logic; readability > performance; 2–10 Hz |
| Perception coordinator | Python | 1 Hz trivial aggregation |

#### Rationale

Python nodes that publish at low rate (≤10 Hz) and contain logic rather than number-crunching are faster to develop and easier to read. C++ nodes on the control-critical path avoid GIL and Python overhead. This split is common in production robotics stacks (Nav2, MoveIt 2 use this pattern).

#### Consequences

- Two toolchains to manage in CI (ament_cmake for C++, ament_python for Python)
- Python nodes require explicit dependency management in `package.xml` (`<exec_depend>`)
- Python `asyncio` is not compatible with `rclpy` executors; use `rclpy.spin_once()` or callbacks only

---

## MVP-ADR-002

### Bridge: uXRCE-DDS vs MAVROS (Reconciling the Roadmap)

**Status:** Accepted

#### Context

The original roadmap draft specified MAVROS2. The full architecture document chose uXRCE-DDS. This ADR explains the reconciliation explicitly so there is no confusion.

#### The Roadmap's MAVROS Choice

The roadmap was written by the project lead based on familiarity with MAVROS from prior work. MAVROS is a completely valid choice and has served the robotics community for years. The ADR is not a criticism of that choice.

#### Why We Move to uXRCE-DDS

| Factor | MAVROS | uXRCE-DDS |
|--------|--------|----------|
| Setup complexity | `sudo apt install ros-humble-mavros` | One extra process (`MicroXRCEAgent`); 2 extra lines in PX4 build |
| PX4 version required | Any PX4 | PX4 ≥1.14 |
| Latency (setpoint path) | ~10–15 ms | ~2–5 ms |
| Topic richness | MAVLink message subset | Full uORB topic set |
| Long-term maintainability | MAVROS maintenance reduced | PX4's active investment |
| Debug experience | `rostopic echo /mavros/*` | `ros2 topic echo /fmu/out/*` — same DX |

The extra setup step for uXRCE-DDS (running `MicroXRCEAgent`) is a one-time cost documented in the integration guide. The benefits compound over the life of the project.

#### Decision

**uXRCE-DDS is the bridge for the MVP and all future phases.** MAVROS is documented as a fallback in the integration guide for community members who need it, but all tutorial content and default launch files use uXRCE-DDS.

#### Deferred

The MAVROS fallback path in `aeroperception_bridge` (`use_mavros: true` configuration flag) is deferred to Phase 5.

---

## MVP-ADR-003

### VIO Backend: OpenVINS as Concrete Default

**Status:** Accepted

#### Context

The full architecture leaves VIO pluggable (OpenVINS, ORB-SLAM3, VINS-Fusion, etc.). The MVP needs one concrete, working implementation.

#### Options for MVP Default

| Option | ROS 2 Native | Monocular | Complexity | Documentation |
|--------|-------------|-----------|------------|---------------|
| OpenVINS | Yes | Yes | Medium | Excellent |
| ORB-SLAM3 | Via wrapper (needs work) | Yes | High | Medium |
| VINS-Fusion | ROS 2 port available | Yes | Medium | Medium |
| DSO | Via wrapper | Yes | High | Low |

#### Decision

**OpenVINS** is the MVP VIO backend.

#### Rationale

- OpenVINS has the best ROS 2 native support of the options; it was written with ROS 2 in mind
- The RPNG lab actively maintains it with clear changelogs
- OpenVINS has excellent parameter documentation — IMU noise tuning (the main failure point in VIO deployment) is well-explained
- ORB-SLAM3 would be the upgrade for loop closure (important for large indoor spaces); this is a Phase 5 option

#### Note on the Roadmap

The roadmap listed "ORB-SLAM3 or VINS-Fusion." Both remain valid options and will work with the VIO interface contract. OpenVINS is the concrete choice for Phase 2 because it is the fastest path to a working, well-documented VIO in ROS 2 Humble.

#### Deferred

ORB-SLAM3 wrapper as alternative backend: deferred to Phase 5 (simulation hardening / research use case).

---

## MVP-ADR-004

### Detector: YOLOv8-nano on CPU (ONNX)

**Status:** Accepted

#### Context

The roadmap listed "YOLOv8/YOLOv9." The MVP needs one concrete, dependency-light implementation.

#### Decision

**YOLOv8-nano exported to ONNX, run with ONNX Runtime on CPU.**

#### Rationale

- YOLOv8-nano is 6 MB; YOLOv9 is larger and more recently released but adds no MVP value
- ONNX Runtime CPU has zero NVIDIA dependency — works on any Linux machine including developer laptops without GPUs
- At 640×640 input, YOLOv8-nano runs at 8–15 FPS on a modern CPU, which is sufficient for Phase 2 (detections are logged, not yet acted on)
- The ONNX path is the same path used on Jetson (via TensorRT EP) — no code change required when moving to GPU hardware

#### Note on the Roadmap

The roadmap mentioned MiDaS/ZoeDepth/Leres for monocular depth estimation alongside the detector. See MVP-ADR-008 for why those are deferred.

#### Deferred

- GPU inference (TensorRT): hardware phase
- YOLOv9 / RT-DETR: Phase 5 evaluation
- Custom-trained model: domain-specific, post-MVP

---

## MVP-ADR-005

### Planner Complexity: Linear Interpolation First

**Status:** Accepted

#### Context

The roadmap listed "RRT*, A*, DWA" for planning. These are the right algorithms for the full system. They are not the right starting point.

#### Options

| Planner | When Appropriate |
|---------|-----------------|
| Linear interpolation (P-controller on position error) | MVP: no obstacles, known waypoints |
| A* on 2D grid | Phase 3: static obstacle map available |
| RRT* | Phase 3+: dynamic or 3D environments |
| DWA (Dynamic Window Approach) | Phase 3+: real-time local avoidance |
| MPPI | Phase 5: model-predictive; best in class |

#### Decision

**Linear waypoint interpolation with velocity capping** for the MVP. Specifically: a proportional controller that generates a velocity setpoint pointing toward the current goal waypoint, scaled to not exceed `max_velocity` and to decelerate as the waypoint is approached.

#### Rationale

- The goal of Phase 2–3 is to prove the perception → control loop works. A complex planner obscures whether problems are in perception, planning, or control.
- Linear interpolation is fully observable: if the vehicle doesn't go where expected, the bug is in the bridge or PX4 parameters, not the planner.
- The planner interface (consume goal pose, publish trajectory setpoints) is identical regardless of algorithm. Replacing it with A* in Phase 3 is a drop-in swap.

#### Deferred

RRT*, A* on 3D voxel map, DWA: Phase 3 implementation. The planner node interface contract does not change.

---

## MVP-ADR-006

### Mission FSM: 5 States, Not 9

**Status:** Accepted

#### Context

The full architecture Mission Manager has 9 states with nuanced degraded modes. The MVP uses 5.

#### Decision

**5 states:** IDLE → TAKEOFF → EXECUTING → HOLDING → LAND

States removed from MVP:

| Full Architecture State | MVP Handling |
|------------------------|-------------|
| DEGRADED (partial perception) | Folded into HOLDING |
| ERROR | HOLDING + log; no automated recovery attempt |
| RTL (autonomous) | Handled by PX4 native RTL; not a separate FSM state |
| DOCKING (precision land) | Not implemented |

#### Rationale

- HOLDING covers all "something is wrong, stop and wait" cases in the MVP
- PX4 handles battery-critical RTL autonomously; no FSM state needed
- Adding states before the basic loop works adds testing surface area with no MVP benefit

#### Deferred

Full 9-state FSM with automated degraded-mode recovery: Phase 4+.

---

## MVP-ADR-007

### Depth Estimation: Deferred from MVP

**Status:** Deferred

#### Context

The roadmap listed monocular depth estimation (MiDaS, ZoeDepth, Leres) as a Phase 2 module. These produce **depth maps** from a single image using learned models.

#### Why Deferred

1. **Depth maps ≠ VIO.** MiDaS produces a relative depth map. It does not produce a metric 3D position estimate suitable for flight control. VIO (OpenVINS) produces that.
2. **Depth maps for obstacle avoidance** require integration with an occupancy map and a local planner. Neither exists in MVP Phase 2.
3. **Relative depth** from MiDaS without metric calibration is not actionable for the MVP planner.
4. **Inference cost:** ZoeDepth runs at 2–4 Hz on CPU; this budget is better spent on VIO reliability in the MVP.

#### What Replaces It in MVP

The detector node (YOLOv8-nano) provides 2D bounding boxes. The planner in Phase 3+ will use VIO + simple repulsion from detections for basic avoidance, without needing a depth map.

#### When to Add

Phase 3 (if stereo depth from Gazebo stereo camera is available) or Phase 5 (learned monocular depth with metric calibration for hardware deployment).

---

## MVP-ADR-008

### Gazebo Camera: Use Native Plugin, Not a Separate Camera Model

**Status:** Accepted

#### Context

The PX4 Gazebo Gz x500 model includes a camera. We could either use this stock camera or create a custom SDF model with more realistic camera parameters.

#### Decision

**Use the stock x500 camera plugin for Phase 1–2.** Tweak only the resolution (to 640×480) and FPS (30) via SDF parameter overrides in `aeroperception_simulation`.

Custom camera SDF (distortion, realistic noise, rolling shutter) is deferred to Phase 5.

#### Rationale

- The stock camera produces a clean, distortion-free image — ideal for validating VIO and detection algorithms before adding noise
- Phase 5 adds noise and distortion as part of the "simulation hardening" work
- This matches the roadmap's Phase 5 goal: "improve camera plugin models"

---

## MVP-ADR-009

### State Machine Implementation: Python, Not SMACH or BehaviorTree.CPP

**Status:** Accepted

#### Context

ROS 2 has ecosystem packages for state machines (SMACH ROS 2 port) and behavior trees (BehaviorTree.CPP + Nav2 BT). The Mission Manager FSM could use either.

#### Decision

**Hand-coded Python FSM** with an explicit state enum and transition functions for the MVP.

#### Rationale

- A 5-state FSM does not need a framework. Adding BehaviorTree.CPP for 5 states is over-engineering.
- SMACH's ROS 2 port has lagged behind; the API is unfamiliar to most ROS 2 developers
- A hand-coded FSM in ~150 lines of Python is readable, debuggable, and easily extended
- BehaviorTree.CPP is the right choice for complex hierarchical behaviors in Phase 4+; starting there before the basic loop works adds unnecessary complexity

#### Deferred

BehaviorTree.CPP (via Nav2 BT executor or standalone): Phase 4, when multi-behavior orchestration and preemption become necessary.

---

## MVP-ADR-010

### Coordinate Frame Contract: ENU Everywhere in ROS 2

**Status:** Accepted — this is a non-negotiable architectural rule

#### Context

PX4 uses NED (North-East-Down). ROS 2 REP-103 specifies ENU (East-North-Up). These are incompatible. Mixing them is the single most common source of bugs in PX4–ROS 2 integration.

#### Decision

**ENU is the universal frame inside the ROS 2 graph.** The bridge node is the only component that touches NED, and only at the PX4 interface boundary.

**Rules:**
- All poses, velocities, and accelerations published on any `/aeroperception/*` or `/state/*` or `/perception/*` topic are in ENU
- The bridge transforms NED→ENU on inbound data and ENU→NED on outbound data
- No node except `bridge_node` imports or uses PX4 frame convention
- All `geometry_msgs/Pose`, `nav_msgs/Odometry`, `geometry_msgs/Twist` are ENU

**Body frame convention:** FLU (Forward-Left-Up) per REP-103 inside ROS 2. PX4 uses FRD (Forward-Right-Down). The bridge handles the body frame transform as part of the attitude quaternion conversion.

#### Rationale

- Violating this rule has caused crashes in real systems. It is worth enforcing from the first commit.
- All downstream code (planner, mission manager, visualization) works with standard ROS 2 spatial conventions, making it compatible with Nav2, MoveIt 2, and every ROS 2 tool out of the box.

#### Consequences

- The transform in the bridge must be correct. It is unit-tested against known poses before any flight.
- TF2 tree: `map` → `odom` → `base_link` (all ENU frames)

---

## MVP-ADR-011

### Test Strategy for PoC

**Status:** Accepted

#### Context

What testing is appropriate for a fast-moving PoC?

#### Decision

**Minimal but non-zero.** Three testing tiers:

| Tier | What | When |
|------|------|------|
| **Unit tests** | Frame transform math (NED↔ENU); waypoint acceptance logic; FSM transitions | Per commit via CI |
| **Integration smoke tests** | Launch SITL + stack; verify topics are publishing; assert VIO health becomes OK within 30 s | Per merge to main |
| **Visual demo test** | Record a bag of a complete mission; Foxglove replay confirms vehicle flew the expected path | Per phase completion |

**What we do not write for MVP:** Exhaustive unit tests for every function, hardware-in-the-loop tests, performance benchmarks. These belong in Phase 5.

**CI pipeline (Phase 0):** `ament_flake8`, `ament_pep8`, `ament_cppcheck`, `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`, and the frame-transform unit tests. These run on every push.

---

*Companion documents: [MVP_ARCHITECTURE_OVERVIEW.md](./MVP_ARCHITECTURE_OVERVIEW.md) | [MVP_COMPONENTS.md](./MVP_COMPONENTS.md) | [MVP_INTEGRATION_GUIDE.md](./MVP_INTEGRATION_GUIDE.md) | [MVP_ROADMAP.md](./MVP_ROADMAP.md)*
