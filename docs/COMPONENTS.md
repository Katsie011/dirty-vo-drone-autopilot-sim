# AeroPerception MVP: Component Specifications

> **Document status:** PoC / MVP scope — v0.1
> **Relationship to full architecture:** Scoped subset of [COMPONENTS.md](./COMPONENTS.md). Where a component is simplified from the full spec, the delta is noted explicitly.
> **Purpose:** Precise enough to implement. Not more complex than necessary.

---

## Table of Contents

1. [Package Structure](#1-package-structure)
2. [Flight Control Stack (PX4 SITL)](#2-flight-control-stack-px4-sitl)
3. [Bridge Layer](#3-bridge-layer)
4. [Perception Pipeline](#4-perception-pipeline)
5. [Planning and Mission Management](#5-planning-and-mission-management)
6. [Sensor Driver Package (Simulation)](#6-sensor-driver-package-simulation)
7. [Logging](#7-logging)
8. [Interface Package Reference](#8-interface-package-reference)
9. [Node Summary Table](#9-node-summary-table)

---

## 1. Package Structure

```
aeroperception_ws/src/
├── aeroperception_interfaces/    # ALL custom msg/srv/action — do not skip this
├── aeroperception_bringup/       # Launch files; sim and vehicle profiles
├── aeroperception_bridge/        # Bridge node + mode management
├── aeroperception_perception/    # OpenVINS wrapper + YOLOv8 detector node
├── aeroperception_planning/      # Waypoint planner + mission manager
├── aeroperception_sensors/       # Sim camera config + (later) hardware drivers
└── aeroperception_logging/       # rosbag2 config + diagnostics aggregator

External (git submodules / apt):
├── px4_msgs/                     # Pinned to PX4 firmware version
├── OpenVINS (ov_msckf)           # VIO backend
└── depthai-ros / realsense2      # Sensor drivers (hardware phase only)
```

**Naming convention:** All ROS 2 packages use the `aeroperception_` prefix. All custom interface types live exclusively in `aeroperception_interfaces`. This is enforced from day one.

---

## 2. Flight Control Stack (PX4 SITL)

### 2.1 What We Use from PX4

For the MVP, we treat PX4 as a black box with a clean interface. We do not modify PX4 internals.

The only PX4 parameters we touch:

| Parameter | Value | Reason |
|-----------|-------|--------|
| `EKF2_AID_MASK` | Enable vision pose fusion | Accept VIO from companion |
| `EKF2_EV_DELAY` | ~50 ms | Expected VIO latency |
| `EKF2_HGT_REF` | vision (when GPS-denied testing) or baro+GPS | Height reference |
| `COM_RCL_EXCEPT` | 4 (offboard) | Don't failsafe on RC loss during OFFBOARD |
| `NAV_RCL_ACT` | 3 (RTL) | RC loss action |
| `GF_ACTION` | 1 (warn) | Geofence action during development |
| `MC_YAWRATE_MAX` | 60 deg/s | Reasonable yaw rate for smooth VIO |

### 2.2 SITL Launch Target

```
# In PX4-Autopilot directory:
make px4_sitl gz_x500
```

The `gz_x500` model provides:
- Forward-facing RGB camera (used for VIO and detection)
- Downward-facing camera (available; not used in MVP)
- Standard IMU + barometer + GPS (simulated)

### 2.3 What PX4 Gives Us (Topics Available After Bridge)

| Topic | Type | Rate | Used By |
|-------|------|------|---------|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | 50 Hz | Bridge, Planner |
| `/fmu/out/vehicle_status` | `VehicleStatus` | 5 Hz | Mission Manager |
| `/fmu/out/battery_status` | `BatteryStatus` | 1 Hz | Mission Manager, Logging |
| `/fmu/out/sensor_combined` | `SensorCombined` | 100 Hz | VIO (IMU source) |

**What we do not use in MVP:** GPS position topics (VIO is primary), vehicle_attitude (planner uses VIO pose directly), actuator outputs.

---

## 3. Bridge Layer

### 3.1 The `bridge_node`

**Package:** `aeroperception_bridge`
**Node:** `bridge_node` (Lifecycle node)
**Language:** C++

**Single responsibility:** Be the boundary between the PX4 uXRCE-DDS world and the ROS 2 autonomy world. Nothing else belongs here.

#### Inputs (subscribing from PX4 via uXRCE-DDS)

| Topic | Type | Action |
|-------|------|--------|
| `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | Transform NED→ENU; republish as `/aeroperception/vehicle_state` |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | Parse nav_state, arming_state; republish as `/aeroperception/flight_mode` |
| `/fmu/out/battery_status` | `px4_msgs/BatteryStatus` | Pass through to `/aeroperception/battery` |

#### Outputs (publishing to PX4 via uXRCE-DDS)

| Topic | Type | Source | Rate |
|-------|------|--------|------|
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Waypoint planner (via bridge) | 50 Hz |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Bridge heartbeat | 50 Hz |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | Mission manager (arm/mode) | On demand |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | VIO → bridge transform | 30 Hz |

#### Services Provided

| Service | Type | Description |
|---------|------|-------------|
| `/bridge/arm` | `aeroperception_interfaces/srv/ArmVehicle` | Send arm command to PX4 |
| `/bridge/set_mode` | `aeroperception_interfaces/srv/SetFlightMode` | Switch PX4 flight mode |

#### Frame Transforms

The bridge performs **all** NED↔ENU and FRD↔FLU conversions. No other node in the stack should ever see NED coordinates. This is a hard rule. The ROS 2 ecosystem speaks ENU; PX4 speaks NED; the bridge translates at the boundary.

```
PX4 NED (x=North, y=East, z=Down)
    ↕  bridge_node transform
ROS 2 ENU (x=East, y=North, z=Up)
```

Quaternion transform: `q_enu = q_rotation_ned_to_enu * q_ned`
Position transform: `[x_enu, y_enu, z_enu] = [y_ned, x_ned, -z_ned]`

#### Offboard Heartbeat

The bridge maintains the 50 Hz `OffboardControlMode` publish regardless of whether the planner is sending setpoints. When the planner is not active, the bridge publishes a **hold-position setpoint** (current EKF2 position as the target) to keep PX4 in OFFBOARD without drifting.

#### Failure Modes (MVP)

| Failure | Detection | Action |
|---------|-----------|--------|
| XRCE Agent disconnects | Stale `/fmu/out/vehicle_status` > 500 ms | Log ERROR; transition to UNCONFIGURED lifecycle state; supervisor restarts |
| Mode change rejected by PX4 | Service returns `success=false` | Log ERROR; Mission Manager FSM transitions to ERROR state |
| VIO not available | `/state/visual_odometry` stale > 1 s | Do not inject VIO into PX4 EKF2; log WARN |

---

## 4. Perception Pipeline

### 4.1 VIO Node: OpenVINS Wrapper

**Package:** `aeroperception_perception`
**Node:** `vio_node`
**Backend:** OpenVINS (`ov_msckf`)
**Language:** C++ (wrapper), OpenVINS is C++

OpenVINS is launched as a ROS 2 component; its output topics are remapped to the standard perception contract.

#### Inputs

| Topic | Type | Notes |
|-------|------|-------|
| `/camera/image_raw` | `sensor_msgs/Image` | Mono8 or RGB8, 30 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Latched, calibration |
| `/imu/data` | `sensor_msgs/Imu` | 100–200 Hz; sourced from `/fmu/out/sensor_combined` via adapter |

**IMU adapter node:** A thin node converts `px4_msgs/SensorCombined` → `sensor_msgs/Imu` with proper header stamps and frame_id. Lives in `aeroperception_bridge`.

#### Outputs

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/state/visual_odometry` | `nav_msgs/Odometry` | 30–50 Hz | ENU frame, `frame_id: "odom"`, `child_frame_id: "base_link"` |
| `/state/vio_health` | `diagnostic_msgs/DiagnosticStatus` | 1 Hz | TRACKING / LOST / INITIALIZING |

#### OpenVINS Configuration Essentials

OpenVINS is configured via a YAML file in `aeroperception_perception/config/openvins/`. For Gazebo Gz simulated camera, key parameters:

```yaml
# Intrinsics from Gazebo camera plugin (check gz topic for actual values)
cam0_wh: [640, 480]
cam0_k: [fx, fy, cx, cy]
cam0_d: [0, 0, 0, 0]  # Simulated camera has no distortion

# IMU noise (these are the most critical parameters for VIO quality)
# For simulation, start with these; reduce noise for faster sim init
gyro_noise_density: 0.005
accel_noise_density: 0.01
gyro_random_walk: 4.0e-6
accel_random_walk: 2.0e-4

# Feature tracking
num_features: 150        # Reduce if CPU-limited; increase for accuracy
fast_threshold: 15
```

#### VIO Initialization

OpenVINS requires ~2–3 seconds of motion to initialize. The Mission Manager waits for `vio_health.level == OK` before commanding OFFBOARD mode. During initialization, PX4 holds position using its own GPS-based estimate.

#### Failure Mode

If `vio_health` transitions to LOST during a mission:
- Planner immediately publishes zero-velocity setpoint
- Mission Manager transitions to HOLD state
- If VIO recovers within 10 s, mission resumes
- If not, Mission Manager requests RTL via bridge service

### 4.2 Object Detector Node: YOLOv8-nano

**Package:** `aeroperception_perception`
**Node:** `detector_node`
**Backend:** YOLOv8-nano, ONNX Runtime (CPU)
**Language:** Python (acceptable for inference node at this rate)

#### Why YOLOv8-nano for MVP

- Runs at 5–15 Hz on a modern CPU without GPU
- ONNX export means zero dependency on NVIDIA toolchain for SITL
- Produces the same `vision_msgs/Detection2DArray` output as the full-architecture GPU variant
- Model weights are 6 MB; trivially version-controlled or fetched at build time

#### Inputs

| Topic | Type | Notes |
|-------|------|-------|
| `/camera/image_raw` | `sensor_msgs/Image` | Subscribed at 10 Hz max (detector rate-limits itself) |

#### Outputs

| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/perception/detections` | `vision_msgs/Detection2DArray` | ≤10 Hz | Bounding boxes, class labels, confidence scores |
| `/perception/detector_health` | `diagnostic_msgs/DiagnosticStatus` | 1 Hz | OK / WARN (slow inference) / ERROR (model load fail) |

#### Model Configuration

```yaml
# aeroperception_perception/config/detector/yolov8_nano.yaml
model_path: "$(find aeroperception_perception)/models/yolov8n.onnx"
confidence_threshold: 0.40
nms_threshold: 0.45
input_width: 640
input_height: 640
inference_frequency_hz: 10
classes_file: "$(find aeroperception_perception)/config/detector/coco_classes.txt"
```

The default COCO model is used for MVP. Fine-tuned models can be hot-swapped by changing `model_path`.

#### Note on Detections in MVP

In Phase 2, detections are published and logged but **not yet consumed by the planner**. The planner responds to detections in Phase 3+ (obstacle avoidance integration). The detector is built and wired into the graph in Phase 2 so the interface is in place.

### 4.3 Perception Coordinator Node

**Package:** `aeroperception_perception`
**Node:** `perception_coordinator_node`
**Language:** Python

A 10-line aggregator. Subscribes to `vio_health` and `detector_health`. Publishes `/perception/health` as `aeroperception_interfaces/msg/PerceptionHealth`. The Mission Manager subscribes to this single topic rather than individual health topics.

```
/state/vio_health       ─┐
/perception/detector_    ├──► perception_coordinator ──► /perception/health
  health                ─┘
```

---

## 5. Planning and Mission Management

### 5.1 Mission Manager Node

**Package:** `aeroperception_planning`
**Node:** `mission_manager_node`
**Language:** Python (state machine; logic clarity > performance here)

The Mission Manager is the only node that calls bridge services. Everything else publishes topics.

#### 5-State FSM

```
              ┌─────────────────────────────────────────────┐
              │                                             │
  power-on    │        /bridge/arm called                  │
    ──────► [IDLE] ──────────────────────────► [TAKEOFF]   │
              │                                   │         │
   land cmd   │                        VIO ready  │         │
   or error   │                        + altitude │         │
              │                        reached    │         │
              │                                   ▼         │
           [LAND] ◄──── land cmd ──── [EXECUTING] ◄── goal  │
              │                           │       ──────────┘
              │           perc fail       │ link loss /
              │           or error        ▼ operator cmd
              │                       [HOLDING]
              │                           │
              │                    recovery or
              └──────── land cmd ──── RTL cmd
```

| State | Entry Condition | Behavior | Exit Condition |
|-------|----------------|----------|---------------|
| IDLE | Startup; or post-land | No setpoints published | Operator arm + takeoff command |
| TAKEOFF | Arm confirmed; takeoff commanded | Fly to takeoff altitude (default: 3 m); wait for VIO | VIO healthy + altitude reached |
| EXECUTING | VIO healthy; goals loaded | Feed waypoints to planner | All waypoints reached, or HOLD command, or failure |
| HOLDING | Perception degraded, or operator command, or link loss | Zero-velocity setpoint to planner; wait | Recovery confirmed, or LAND command |
| LAND | Land commanded or RTL complete | Command land altitude | Landed confirmed by `vehicle_status.landed_state` |

#### Inputs

| Topic / Service | Type | Notes |
|----------------|------|-------|
| `/aeroperception/vehicle_state` | `VehicleStateENU` | Current pose, mode, arming state |
| `/aeroperception/flight_mode` | `std_msgs/String` | Current PX4 nav_state |
| `/perception/health` | `PerceptionHealth` | Gate EXECUTING state |
| `/operator/command` | `MissionCommand` | Arm, takeoff, load mission, start, hold, land |

#### Outputs

| Topic / Service | Type | Notes |
|----------------|------|-------|
| `/mission/active_goal` | `geometry_msgs/PoseStamped` | Current target for planner |
| `/mission/status` | `MissionStatus` | Published at 2 Hz; logged and telemetered |
| `/bridge/arm` | Service call | Via service client |
| `/bridge/set_mode` | Service call | Via service client |

#### Mission Format (MVP)

A mission is a JSON-serializable list of waypoints with altitude in ENU. No curves, no speed changes, no conditions:

```json
{
  "mission_id": "survey_01",
  "takeoff_altitude_m": 5.0,
  "waypoints": [
    {"x": 0.0, "y": 0.0, "z": 5.0, "yaw_deg": 0.0},
    {"x": 10.0, "y": 0.0, "z": 5.0, "yaw_deg": 0.0},
    {"x": 10.0, "y": 10.0, "z": 5.0, "yaw_deg": 0.0},
    {"x": 0.0, "y": 0.0, "z": 5.0, "yaw_deg": 0.0}
  ],
  "waypoint_acceptance_radius_m": 0.5
}
```

Missions are loaded via the `/mission/load` service or from a YAML file at launch.

### 5.2 Waypoint Planner Node

**Package:** `aeroperception_planning`
**Node:** `waypoint_planner_node`
**Language:** C++ (this is on the hot path; must publish at 50 Hz reliably)

The simplest viable planner: constant velocity interpolation between waypoints with a velocity cap.

#### Algorithm

```
Given: current_pose (from /aeroperception/vehicle_state)
       current_goal (from /mission/active_goal)
       max_velocity: 2.0 m/s (configurable)
       acceptance_radius: 0.5 m (configurable)

Each cycle (50 Hz):
  direction = normalize(goal.position - current_pose.position)
  distance = |goal.position - current_pose.position|

  if distance < acceptance_radius:
    publish "waypoint_reached" event
    request next goal from mission manager

  velocity_cmd = direction * min(max_velocity, distance * kp)
  # kp = 0.8 (proportional gain; slows near waypoint)

  Publish TrajectorySetpoint:
    position = goal.position  (feedforward)
    velocity = velocity_cmd   (primary control)
    yaw = goal.yaw
```

This is a P-controller on position error, implemented as a velocity setpoint. PX4's velocity controller handles the inner loop. Simple, predictable, testable.

#### Inputs

| Topic | Type | Rate |
|-------|------|------|
| `/aeroperception/vehicle_state` | `VehicleStateENU` | 50 Hz |
| `/mission/active_goal` | `geometry_msgs/PoseStamped` | On change |

#### Outputs

| Topic | Type | Rate |
|-------|------|------|
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | 50 Hz (always, even in HOLD) |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | 50 Hz (heartbeat) |

#### Growth Hook

In Phase 3+, this node is extended to subscribe to `/perception/obstacle_cloud` and apply simple velocity scaling when obstacles are within a threshold radius. The waypoint goal interface does not change.

---

## 6. Sensor Driver Package (Simulation)

### 6.1 Gazebo Camera Bridge

In SITL, the camera is a Gazebo Gz sensor. It is bridged to ROS 2 via `ros_gz_bridge`.

**Bridge configuration** (`aeroperception_sensors/config/gz_bridge_sim.yaml`):

```yaml
- ros_topic_name: "/camera/image_raw"
  gz_topic_name: "/world/default/model/x500_0/link/camera_link/sensor/camera_front/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/world/default/model/x500_0/link/camera_link/sensor/camera_front/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
```

**Note:** Exact Gazebo topic names depend on the world and model SDF. These must be verified against the actual PX4 x500 Gazebo Gz model. Use `gz topic -l` to enumerate available topics.

### 6.2 IMU Adapter

The Gazebo simulation exposes IMU through PX4's uXRCE bridge as `px4_msgs/SensorCombined`. OpenVINS expects `sensor_msgs/Imu`. A thin adapter node in `aeroperception_bridge` performs the conversion:

```
/fmu/out/sensor_combined  →  imu_adapter_node  →  /imu/data  (sensor_msgs/Imu)
```

This adapter also logs the timestamp delta to verify time alignment between camera and IMU frames.

---

## 7. Logging

### 7.1 rosbag2 Recorder

**Preset:** `essentials` — records state, mission, key perception, battery. Does not record raw images by default (adds ~2 GB/min). Raw image recording is enabled via launch argument `record_images:=true`.

**Topics recorded in `essentials` preset:**

```
/aeroperception/vehicle_state
/aeroperception/flight_mode
/aeroperception/battery
/state/visual_odometry
/state/vio_health
/perception/detections
/perception/health
/mission/status
/mission/active_goal
/fmu/in/trajectory_setpoint
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status
/diagnostics_agg
```

**Storage config** (`aeroperception_logging/config/record_config.yaml`):

```yaml
storage_id: mcap
output_path: "~/aeroperception_bags"
max_bag_size: 1073741824  # 1 GB per bag file
compression_mode: file
compression_format: zstd
```

### 7.2 Diagnostics

Each node publishes to `/diagnostics` at 1 Hz. The `diagnostic_aggregator` node rolls this up to `/diagnostics_agg`. In MVP, this is consumed only by the rosbag and by RViz2 during development. No forwarding to GCS in the PoC.

---

## 8. Interface Package Reference

The MVP subset of `aeroperception_interfaces`. These are all the types needed for Phase 0–4. Types are defined once and never duplicated.

### 8.1 Messages

**`aeroperception_interfaces/msg/VehicleStateENU.msg`**
```
# Vehicle state in ENU (East-North-Up) frame
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
string flight_mode      # "OFFBOARD", "HOLD", "POSCTL", etc.
bool armed
float32 battery_remaining  # 0.0 to 1.0
```

**`aeroperception_interfaces/msg/MissionStatus.msg`**
```
std_msgs/Header header
string state            # "IDLE", "TAKEOFF", "EXECUTING", "HOLDING", "LAND"
int32 active_waypoint
int32 total_waypoints
float32 elapsed_time_s
bool perception_healthy
```

**`aeroperception_interfaces/msg/MissionCommand.msg`**
```
std_msgs/Header header
string command          # "ARM", "TAKEOFF", "START", "HOLD", "LAND", "RTL"
string mission_json     # Used with LOAD command; empty otherwise
```

**`aeroperception_interfaces/msg/PerceptionHealth.msg`**
```
std_msgs/Header header
uint8 vio_status        # 0=UNKNOWN, 1=INITIALIZING, 2=HEALTHY, 3=DEGRADED, 4=FAILED
uint8 detector_status   # same enum
bool overall_healthy    # true if vio_status == HEALTHY
```

### 8.2 Services

**`aeroperception_interfaces/srv/ArmVehicle.srv`**
```
bool arm
---
bool success
string message
```

**`aeroperception_interfaces/srv/SetFlightMode.srv`**
```
string mode
---
bool success
string current_mode
```

**`aeroperception_interfaces/srv/LoadMission.srv`**
```
string mission_json
---
bool success
int32 num_waypoints
string error_message
```

---

## 9. Node Summary Table

Complete inventory of all ROS 2 nodes in the MVP stack.

| Node | Package | Language | Rate | Purpose |
|------|---------|----------|------|---------|
| `bridge_node` | `aeroperception_bridge` | C++ | 50 Hz | FC↔ROS 2 translation, frame transforms |
| `imu_adapter_node` | `aeroperception_bridge` | C++ | 200 Hz | SensorCombined → sensor_msgs/Imu |
| `vio_node` | `aeroperception_perception` | C++ (OpenVINS) | 30–50 Hz | Visual-Inertial Odometry |
| `detector_node` | `aeroperception_perception` | Python | ≤10 Hz | YOLOv8-nano object detection |
| `perception_coordinator_node` | `aeroperception_perception` | Python | 1 Hz | Health aggregator |
| `mission_manager_node` | `aeroperception_planning` | Python | 2–10 Hz | 5-state FSM, mission execution |
| `waypoint_planner_node` | `aeroperception_planning` | C++ | 50 Hz | Setpoint generation |
| `gz_camera_bridge` | (ros_gz_bridge, configured) | — | 30 Hz | Gazebo camera → ROS 2 |
| `diagnostic_aggregator` | (standard package) | — | 1 Hz | Roll up /diagnostics |
| `rosbag2_recorder` | (standard package, configured) | — | — | Record essentials preset |

**Process note:** For MVP, each node runs as a separate process. If latency on the hot path (bridge → planner → bridge) exceeds 20 ms during development, migrate bridge + planner to a single component container.

---

*Companion documents: [MVP_ARCHITECTURE_OVERVIEW.md](./MVP_ARCHITECTURE_OVERVIEW.md) | [MVP_DESIGN_DECISIONS.md](./MVP_DESIGN_DECISIONS.md) | [MVP_INTEGRATION_GUIDE.md](./MVP_INTEGRATION_GUIDE.md) | [MVP_ROADMAP.md](./MVP_ROADMAP.md)*
