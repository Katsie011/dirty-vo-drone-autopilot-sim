# AeroPerception MVP: Integration Guide

> **Document status:** PoC / MVP scope — v0.1
> **Purpose:** Concrete, executable instructions for each phase of the MVP. Follow these in order. Each phase ends with a defined, verifiable deliverable.
> **Companion to:** [MVP_ROADMAP.md](./MVP_ROADMAP.md) (what), this document (how)

---

## Table of Contents

1. [Phase 0 — Repository and Environment Setup](#phase-0--repository-and-environment-setup)
2. [Phase 1 — PX4 SITL + Bridge + Basic Commands](#phase-1--px4-sitl--bridge--basic-commands)
3. [Phase 2 — Perception Pipeline](#phase-2--perception-pipeline)
4. [Phase 3 — Waypoint Planning](#phase-3--waypoint-planning)
5. [Phase 4 — Mission Loop + Logging](#phase-4--mission-loop--logging)
6. [Common Debugging Reference](#common-debugging-reference)
7. [Message Flow Diagrams](#message-flow-diagrams)
8. [Failsafe Flows (MVP)](#failsafe-flows-mvp)

---

## Phase 0 — Repository and Environment Setup

**Goal:** Clean workspace, all dependencies resolvable, CI passing on an empty build.

### 0.1 Host Machine Requirements

- Ubuntu 22.04 LTS (bare metal or VM with ≥4 CPU cores allocated)
- ≥16 GB RAM (PX4 SITL + Gazebo + ROS 2 stack together consume ~6–10 GB)
- ≥50 GB free disk (PX4, Gazebo worlds, bags)
- Internet access for initial dependency fetch

### 0.2 Install ROS 2 Humble

Follow the official binary install:

```bash
# Locale setup
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools

# Source in your .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 0.3 Install CycloneDDS

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp

# Add to .bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### 0.4 Install Gazebo Gz Harmonic

```bash
sudo apt install curl
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable \
  $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install gz-harmonic

# ROS-Gazebo bridge
sudo apt install ros-humble-ros-gzharmonic
```

### 0.5 Set Up the ROS 2 Workspace

```bash
mkdir -p ~/aeroperception_ws/src
cd ~/aeroperception_ws/src

# Initialize the AeroPerception repository here
# (create the packages listed in MVP_COMPONENTS.md Section 1)
# After packages exist:
cd ~/aeroperception_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 0.6 Install rosbag2 MCAP Plugin

```bash
sudo apt install ros-humble-rosbag2-storage-mcap
```

### 0.7 CI Configuration (GitHub Actions)

Create `.github/workflows/ci.yml`:

```yaml
name: CI
on: [push, pull_request]
jobs:
  build-and-lint:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: humble
      - run: |
          export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
          rosdep update
          rosdep install --from-paths src --ignore-src -r -y
          colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
          colcon test --return-code-on-test-failure
```

**Phase 0 Deliverable:** `colcon build` succeeds on empty (stub) packages. CI passes. README exists with setup instructions.

---

## Phase 1 — PX4 SITL + Bridge + Basic Commands

**Goal:** A ROS 2 node can arm the vehicle, command takeoff, hover, and land — fully from code, no RC required. QGC confirms the flight in monitor mode.

### 1.1 Install PX4

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# Install PX4 dev dependencies
bash ./Tools/setup/ubuntu.sh

# Build PX4 SITL with Gazebo Gz
make px4_sitl gz_x500
```

This downloads all dependencies and produces the SITL binary. If it builds cleanly, PX4 is ready.

**Verify the uXRCE-DDS client is enabled:**

```bash
# In the PX4 build, check:
grep -r "MICROXRCE_DDS_CLIENT" PX4-Autopilot/boards/px4/sitl/
# Should see: CONFIG_MODULES_MICROXRCE_DDS_CLIENT=y
```

If not present, add it to the board configuration (consult PX4 documentation for your exact version).

### 1.2 Install Micro-XRCE-DDS Agent

```bash
pip install pyserial  # dependency
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig /usr/local/lib/
```

Verify:

```bash
MicroXRCEAgent --help
# Should show usage for serial and udp modes
```

### 1.3 Pin px4_msgs to Your PX4 Version

```bash
cd ~/aeroperception_ws/src
git clone https://github.com/PX4/px4_msgs.git

# CRITICAL: Check the PX4 firmware release tag and match px4_msgs tag
cd px4_msgs
git tag --list  # find the tag matching your PX4 version
git checkout <matching_tag>
```

**Version mismatch is the most common Phase 1 failure.** If `px4_msgs` and PX4 firmware are mismatched, messages will not deserialize correctly and topics will appear to publish garbage. Always pin them together.

### 1.4 Launch SITL for the First Time

Open four terminals, all sourced with `source ~/.bashrc` and `source ~/aeroperception_ws/install/setup.bash`:

**Terminal 1: PX4 SITL**
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
Wait for the Gazebo Gz window to open and PX4 to print `[commander] Ready for takeoff!`

**Terminal 2: XRCE Agent**
```bash
MicroXRCEAgent udp4 -p 8888
```
Wait for PX4 to connect. You will see: `[UXRCEClient|INFO] ... connected`

**Terminal 3: Verify topics**
```bash
ros2 topic list | grep fmu
```
You should see:
```
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status
/fmu/out/battery_status
/fmu/out/sensor_combined
# ... and more
```

**Terminal 4: QGC**
```bash
# Download QGC AppImage from qgroundcontrol.com, make executable, launch
./QGroundControl.AppImage
```
QGC should connect to the simulated vehicle on UDP 14550 (PX4 SITL listens on this port by default).

### 1.5 Build and Launch the Bridge Node

With the `aeroperception_bridge` package implemented (see MVP_COMPONENTS.md Section 3):

```bash
cd ~/aeroperception_ws
colcon build --packages-select aeroperception_bridge aeroperception_interfaces
source install/setup.bash

ros2 launch aeroperception_bringup bridge.launch.py
```

Verify bridge is working:

```bash
# Should publish at ~50 Hz
ros2 topic hz /aeroperception/vehicle_state

# Should show ENU position (not NED)
ros2 topic echo /aeroperception/vehicle_state --once
```

### 1.6 Arm, Takeoff, Hover, Land

With a minimal test script (this lives in `aeroperception_bringup/scripts/test_takeoff.py`):

```python
#!/usr/bin/env python3
"""
Phase 1 validation script: arm → takeoff → hover 5s → land
"""
import rclpy
from rclpy.node import Node
from aeroperception_interfaces.srv import ArmVehicle, SetFlightMode, LoadMission
from aeroperception_interfaces.msg import MissionCommand
import time

# Test sequence:
# 1. Call /bridge/arm (arm=True)
# 2. Call /bridge/set_mode "OFFBOARD"
# 3. Publish MissionCommand TAKEOFF to /operator/command
# 4. Wait 10 seconds (hover)
# 5. Publish MissionCommand LAND to /operator/command
```

**Phase 1 Deliverable:** Record a short `ros2 bag record` of the flight. Play it back and confirm in Foxglove that `/aeroperception/vehicle_state.pose.position.z` rises to ~3 m, holds, then returns to 0. Record a screen capture of QGC showing the vehicle track.

### 1.7 Common Phase 1 Problems

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| `/fmu/out/*` topics empty | XRCE Agent not connected | Check Agent terminal for errors; verify PX4 has uXRCE module |
| `px4_msgs` deserialization errors | Version mismatch | Align git tags; rebuild |
| Vehicle immediately disarms after OFFBOARD | Setpoints not pre-publishing | Bridge must publish setpoints for ≥1 s before requesting OFFBOARD |
| QGC shows "No GPS Fix" | Normal for indoor SITL | Check PX4 `GNSS_PITCH` and `GNSS_CONFIG` params; or ignore for now |
| Bridge node crashes with TF error | NED/ENU transform bug | Unit-test transforms against known poses |

---

## Phase 2 — Perception Pipeline

**Goal:** OpenVINS publishing `/state/visual_odometry` and YOLOv8-nano publishing `/perception/detections`, both on the live simulated camera stream.

### 2.1 Install OpenVINS

```bash
cd ~/aeroperception_ws/src
git clone https://github.com/rpng/open_vins.git

# Humble-compatible branch — check OpenVINS GitHub for current recommendation
cd open_vins
git checkout master  # or the ros2 branch if separated
```

Install dependencies:

```bash
sudo apt install libsuitesparse-dev libeigen3-dev libopencv-dev
cd ~/aeroperception_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ov_core ov_msckf ov_init ov_eval
```

### 2.2 Configure OpenVINS for Gazebo Simulated Camera

Copy the base config template to `aeroperception_perception/config/openvins/sim_x500.yaml` and set:

```yaml
# Camera intrinsics (get from Gazebo camera SDF or /camera/camera_info)
# Typical Gazebo default: 640x480, FOV 1.3962 rad (~80 deg)
# fx = fy = width / (2 * tan(fov/2)) ≈ 206
cam0_wh: [640, 480]
cam0_k: [206.0, 206.0, 320.0, 240.0]  # [fx, fy, cx, cy]
cam0_d: [0.0, 0.0, 0.0, 0.0]           # No distortion in simulation

# IMU noise — start generous for simulation; tighten for hardware
gyro_noise_density: 0.005
accel_noise_density: 0.01
gyro_random_walk: 4.0e-6
accel_random_walk: 2.0e-4

# VIO settings
num_features: 150       # Reduce to 100 if CPU-limited
fast_threshold: 15
min_parallax: 1.0
max_camera_time_diff: 0.02  # 20 ms

# Display (disable in headless mode)
use_pangolin: false
```

**Getting exact intrinsics from Gazebo:** After Gazebo is running:
```bash
ros2 topic echo /camera/camera_info --once
# Read fx, fy, cx, cy from the K matrix (K[0], K[4], K[2], K[5])
```

### 2.3 Launch Perception Stack

```bash
ros2 launch aeroperception_perception perception.launch.py \
  vio_backend:=openvins \
  config_file:=$(ros2 pkg prefix aeroperception_perception)/config/openvins/sim_x500.yaml
```

Verify VIO is running:

```bash
# Should publish at 30-50 Hz after initialization
ros2 topic hz /state/visual_odometry

# Check VIO health (wait up to 30 s for initialization)
ros2 topic echo /state/vio_health
# level: 0 = OK, 1 = WARN (initializing), 2 = ERROR
```

**VIO initialization takes 3–10 seconds** as features accumulate. During SITL testing, the vehicle can be commanded to hover in place; OpenVINS will initialize once enough parallax is available. Yaw rotation helps.

### 2.4 Inject VIO into PX4 EKF2

Set these PX4 parameters (via QGC Parameters panel or `param set` in the PX4 shell):

```
EKF2_AID_MASK: set bit 3 (vision position fusion) = value 8
EKF2_EV_DELAY: 50 (ms)
EKF2_EV_NOISE_MD: 1 (use external noise values)
```

The bridge node then publishes VIO to `/fmu/in/vehicle_visual_odometry` whenever VIO health is OK.

Verify fusion is working:
```bash
# In PX4 shell (type 'status' in the SITL terminal):
> ekf2 status
# Should show "vision position" in the fusion sources
```

### 2.5 Install and Launch YOLOv8 Detector

```bash
# Python dependencies (in venv or system Python)
pip3 install onnxruntime opencv-python ultralytics

# Download YOLOv8-nano ONNX model
# Via Ultralytics (requires ultralytics pip package):
python3 -c "from ultralytics import YOLO; m=YOLO('yolov8n.pt'); m.export(format='onnx')"
# Moves yolov8n.onnx to aeroperception_perception/models/
```

Launch:
```bash
ros2 launch aeroperception_perception detector.launch.py \
  model_path:=$(ros2 pkg prefix aeroperception_perception)/models/yolov8n.onnx
```

Verify:
```bash
ros2 topic hz /perception/detections
# Expect: ~5-10 Hz (CPU inference; depends on machine)

ros2 topic echo /perception/detections --once
# Should see Detection2DArray with bounding boxes if anything is in the camera view
```

**Phase 2 Deliverable:** Record a bag with `ros2 bag record -a -o phase2_demo`. Replay in Foxglove and confirm:
- `/state/visual_odometry` publishing at ≥30 Hz with pose values that change as the simulated vehicle moves
- `/perception/detections` publishing (even if empty; confirms node is alive)
- `/state/vio_health` shows level=0 (OK) after initialization

---

## Phase 3 — Waypoint Planning

**Goal:** Vehicle flies to a sequence of 4 waypoints autonomously, driven entirely by the perception-planning-control pipeline.

### 3.1 Launch the Waypoint Planner

With `aeroperception_planning` built:

```bash
ros2 launch aeroperception_bringup planning.launch.py
```

Verify planner topics:
```bash
# Should publish at 50 Hz (even with no goal, publishes hold setpoint)
ros2 topic hz /fmu/in/trajectory_setpoint

# Should publish OffboardControlMode heartbeat
ros2 topic hz /fmu/in/offboard_control_mode
```

### 3.2 Send a Manual Waypoint Goal

Before the Mission Manager is wired up, test the planner directly:

```bash
# Publish a single goal pose (ENU: x=East, y=North, z=Up from origin)
ros2 topic pub /mission/active_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 0.0, z: 5.0}, \
  orientation: {w: 1.0}}}" --once
```

Switch to OFFBOARD mode (vehicle must already be armed and airborne):
```bash
ros2 service call /bridge/set_mode aeroperception_interfaces/srv/SetFlightMode \
  "{mode: 'OFFBOARD'}"
```

The vehicle should fly toward (5, 0, 5) in ENU coordinates. Watch in Gazebo and QGC.

### 3.3 Add a Simple Obstacle in Gazebo

Add a box model to the Gazebo world to test that the planner can fly around it when Phase 3 obstacle avoidance is added:

In `aeroperception_simulation/worlds/test_world.sdf`:
```xml
<model name="obstacle_box">
  <pose>3.0 0.0 1.0 0 0 0</pose>
  <link name="box_link">
    <collision name="box_collision">
      <geometry><box><size>1.0 1.0 2.0</size></box></geometry>
    </collision>
    <visual name="box_visual">
      <geometry><box><size>1.0 1.0 2.0</size></box></geometry>
    </visual>
  </link>
  <static>true</static>
</model>
```

In Phase 3 MVP, the obstacle is visible in Gazebo but **not avoided** (no depth source yet). The waypoint at (5, 0, 5) passes above the obstacle (box height is 2 m, waypoint is at 5 m altitude). This is intentional — Phase 3 validates path following, not avoidance.

**Phase 3 Deliverable:** Record a bag. Foxglove plot of vehicle position vs. target waypoints shows the vehicle reaching each waypoint within acceptance radius. Flight path in QGC shows the expected route.

---

## Phase 4 — Mission Loop + Logging

**Goal:** The full pipeline runs end-to-end from a single launch command. Mission Manager drives the flight. rosbag2 records everything. Foxglove can replay the mission.

### 4.1 Full Stack Launch File

`aeroperception_bringup/launch/simulation.launch.py` launches everything in the correct order:

```
Launch order (with lifecycle management):
1. ros_gz_bridge (camera bridge) — must be ready before perception
2. MicroXRCEAgent — must connect before bridge node
3. bridge_node — must connect to PX4 before mission manager
4. imu_adapter_node
5. vio_node — waits for camera and IMU
6. detector_node
7. perception_coordinator_node
8. waypoint_planner_node
9. mission_manager_node
10. rosbag2_recorder — starts last to catch all initialization messages
```

**Lifecycle dependency management:** The Mission Manager waits for `/perception/health.overall_healthy` before transitioning to EXECUTING. This naturally handles the startup ordering.

### 4.2 Load and Execute a Mission

```bash
# Terminal 1: SITL
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2: XRCE Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Full AeroPerception stack
ros2 launch aeroperception_bringup simulation.launch.py

# Terminal 4: Send mission
ros2 service call /mission/load aeroperception_interfaces/srv/LoadMission \
  "{mission_json: '{
    \"mission_id\": \"phase4_demo\",
    \"takeoff_altitude_m\": 5.0,
    \"waypoints\": [
      {\"x\": 0.0, \"y\": 0.0, \"z\": 5.0, \"yaw_deg\": 0.0},
      {\"x\": 10.0, \"y\": 0.0, \"z\": 5.0, \"yaw_deg\": 0.0},
      {\"x\": 10.0, \"y\": 10.0, \"z\": 5.0, \"yaw_deg\": 90.0},
      {\"x\": 0.0, \"y\": 10.0, \"z\": 5.0, \"yaw_deg\": 180.0},
      {\"x\": 0.0, \"y\": 0.0, \"z\": 5.0, \"yaw_deg\": 0.0}
    ],
    \"waypoint_acceptance_radius_m\": 0.5
  }'}"

ros2 service call /mission/start aeroperception_interfaces/srv/StartMission "{}"
```

Monitor mission progress:
```bash
ros2 topic echo /mission/status
```

### 4.3 Recording and Replay

The rosbag2 recorder starts automatically with the `simulation.launch.py`. Bags are written to `~/aeroperception_bags/`.

**Replaying in Foxglove Studio:**
1. Open Foxglove Studio (download from foxglove.dev)
2. File → Open file → select the `.mcap` bag
3. Drag `/aeroperception/vehicle_state` to a 3D panel — confirm the flight path
4. Drag `/perception/detections` to an image panel with `/camera/image_raw` — confirm detections overlay
5. Drag `/mission/status` to a state panel — confirm state transitions

### 4.4 Phase 4 Definition of Done

The MVP is complete when, from a **fresh repository clone** on a clean Ubuntu 22.04 machine, following the Phase 0–4 integration guide, a developer can:

1. Build the workspace in <30 minutes
2. Launch SITL + full AeroPerception stack with one command
3. Load and execute the Phase 4 demo mission
4. Observe the vehicle fly the 5-waypoint route in Gazebo and QGC
5. Replay the recorded bag in Foxglove and see all key topics
6. Confirm VIO health is OK for the duration of the mission
7. Confirm detections are publishing on the camera stream

This is the MVP acceptance test.

---

## Common Debugging Reference

### Topic Health Checklist

Run this after launching to confirm the pipeline is alive before flying:

```bash
#!/bin/bash
# health_check.sh — run after launching simulation.launch.py

echo "=== FC Bridge ==="
ros2 topic hz /aeroperception/vehicle_state --window 5 2>&1 | grep "average rate"
ros2 topic hz /fmu/in/offboard_control_mode --window 5 2>&1 | grep "average rate"

echo "=== Perception ==="
ros2 topic hz /state/visual_odometry --window 5 2>&1 | grep "average rate"
ros2 topic hz /perception/detections --window 5 2>&1 | grep "average rate"

echo "=== Planner ==="
ros2 topic hz /fmu/in/trajectory_setpoint --window 5 2>&1 | grep "average rate"

echo "=== Mission ==="
ros2 topic echo /mission/status --once
ros2 topic echo /perception/health --once
```

### Diagnosis Decision Tree

```
Vehicle not responding to OFFBOARD commands?
  └── Check: ros2 topic hz /fmu/in/offboard_control_mode
        └── <10 Hz: planner or bridge not running → check node list
        └── OK: check /fmu/out/vehicle_status.nav_state == 14 (OFFBOARD)?
              └── No: PX4 rejected OFFBOARD → was it pre-published for 1 s?

VIO not initializing?
  └── Check: ros2 topic hz /camera/image_raw
        └── 0 Hz: Gazebo bridge not running → check ros_gz_bridge
        └── OK: ros2 topic echo /state/vio_health
              └── INITIALIZING: move the vehicle; yaw rotation helps
              └── ERROR: check OpenVINS launch log for camera_info mismatch

Setpoints publishing but vehicle not moving?
  └── Check: is vehicle armed? → ros2 topic echo /aeroperception/flight_mode
  └── Check: is vehicle in OFFBOARD? → same topic
  └── Check: setpoint values → ros2 topic echo /fmu/in/trajectory_setpoint --once
        └── All NaN: planner is in HOLD mode → check mission/active_goal
        └── NED position instead of ENU: bridge frame transform bug
```

---

## Message Flow Diagrams

### Phase 1 Flow (Bridge Validation)

```
PX4 SITL ──uXRCE──► XRCE Agent ──DDS──► /fmu/out/vehicle_status
                                          /fmu/out/vehicle_local_position
                                                    │
                                         bridge_node (NED→ENU)
                                                    │
                                         /aeroperception/vehicle_state  ──► QGC monitor
                                                    │
                              /bridge/arm ◄──── test_takeoff.py
                              /bridge/set_mode ◄─┘
                                                    │
                                         /fmu/in/trajectory_setpoint ──► PX4
                                         /fmu/in/offboard_control_mode ──► PX4
```

### Phase 2 Flow (Perception Online)

```
Gazebo camera ──gz_bridge──► /camera/image_raw ──────────────────────────┐
                                                                          │
/fmu/out/sensor_combined ──► imu_adapter ──► /imu/data                  │
                                                          │               │
                                                          ▼               ▼
                                                    OpenVINS VIO     YOLOv8-nano
                                                          │               │
                                         /state/visual_odometry   /perception/detections
                                                          │
                                           (also injected to EKF2)
                                           /fmu/in/vehicle_visual_odometry
```

### Phase 4 Flow (Full Mission)

```
┌────────────────────────────────────────────────────────────────────┐
│  Mission Manager                                                    │
│  State: IDLE ──► TAKEOFF ──► EXECUTING ──► LAND                   │
│                                                                     │
│  Reads: /perception/health, /aeroperception/vehicle_state          │
│  Writes: /mission/active_goal (one waypoint at a time)             │
│  Calls: /bridge/arm, /bridge/set_mode                              │
└───────────────────────────┬────────────────────────────────────────┘
                            │ /mission/active_goal
                            ▼
                  Waypoint Planner
                  Reads: vehicle_state + active_goal
                  Writes: /fmu/in/trajectory_setpoint @ 50 Hz
                            │
                            ▼
                  Bridge Node (ENU→NED)
                            │
                            ▼
                  PX4 FC (position + velocity controller)
                            │
                            ▼
                     Motors → Flight
```

---

## Failsafe Flows (MVP)

### OFFBOARD Heartbeat Loss

```
Bridge or planner process dies
        │
PX4 watchdog: no setpoints for > 500 ms
        │
PX4 exits OFFBOARD → HOLD mode (configured via NAV_RCL_ACT)
        │
Mission Manager: detects nav_state != OFFBOARD
        │
Mission Manager: transitions to HOLDING state
        │
Mission Manager: attempts re-entry (publish setpoints → set_mode OFFBOARD)
        │
If re-entry fails 3x: Mission Manager requests RTL via /bridge/set_mode "AUTO.RTL"
```

### VIO Health Lost During Mission

```
OpenVINS: tracking LOST (textureless surface or fast motion)
        │
/state/vio_health → DEGRADED or FAILED
        │
perception_coordinator: /perception/health.overall_healthy = false
        │
Mission Manager: transitions to HOLDING state
        │
Planner: receives HOLD goal (current position) → publishes zero-velocity setpoint
        │
Wait up to 10 s for VIO recovery
        │
VIO recovers → Mission Manager resumes from last waypoint
        │
VIO does not recover → Mission Manager requests RTL
```

### Battery Critical (PX4-Owned)

```
PX4 battery_status.remaining < COM_LOW_BAT_ACT threshold
        │
PX4 commander: forces LAND (no ROS 2 involvement needed)
        │
Mission Manager: observes vehicle_status.nav_state == AUTO_LAND
        │
Mission Manager: transitions to LAND state; logs event
```

---

*Companion documents: [MVP_ARCHITECTURE_OVERVIEW.md](./MVP_ARCHITECTURE_OVERVIEW.md) | [MVP_COMPONENTS.md](./MVP_COMPONENTS.md) | [MVP_DESIGN_DECISIONS.md](./MVP_DESIGN_DECISIONS.md) | [MVP_ROADMAP.md](./MVP_ROADMAP.md)*
