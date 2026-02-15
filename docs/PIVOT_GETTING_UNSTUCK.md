# AeroPerception: Getting Unstuck — The Mac M1 Pivot

> **Context:** PX4 SITL has no native arm64 Docker image and is extremely painful to build on Mac M1/M2/M3. OpenVINS has fragile ROS 2 build dependencies. One day lost already. This document resets the plan.
> **Goal:** Flying, perceiving, autonomously navigating loop — running on your Mac — by the end of today's session.

---


## The New MVP Stack

```
┌─────────────────────────────────────────────────────────────────┐
│  Everything runs natively on Mac M1. No Docker. No compilation. │
│                                                                 │
│  ┌─────────────────┐     MAVLink/UDP      ┌──────────────────┐  │
│  │  Python Drone   │◄────────────────────►│  MAVSDK-Python   │  │
│  │  Simulator      │      port 14550      │  or pymavlink    │  │
│  │  (~120 lines)   │                      │  bridge          │  │
│  └─────────────────┘                      └────────┬─────────┘  │
│                                                    │            │
│  ┌─────────────────┐                      ┌────────▼─────────┐  │
│  │  Video Source   │                      │  Mission Manager │  │
│  │  (webcam, file, │──► /camera/image_raw │  + Planner       │  │
│  │   or generated) │                      └────────┬─────────┘  │
│  └─────────────────┘                               │            │
│                                                    │            │
│  ┌─────────────────┐                      ┌────────▼─────────┐  │
│  │  LK Optical     │◄── /camera/image_raw │  Waypoint goals  │  │
│  │  Flow VO        │──► /state/visual_odom│  → velocity cmds │  │
│  │  (~80 lines)    │                      └──────────────────┘  │
│  └─────────────────┘                                            │
│                                                                 │
│  ┌─────────────────┐                                            │
│  │  YOLOv8-nano    │◄── /camera/image_raw                       │
│  │  Detector       │──► /perception/detections                  │
│  └─────────────────┘                                            │
└─────────────────────────────────────────────────────────────────┘
```

### What Each Component Is

| Component | What | Install |
|-----------|------|---------|
| Python drone sim | 120-line Python script; fake MAVLink vehicle | `pip install pymavlink` |
| MAVSDK-Python bridge | Async Python; talks MAVLink, publishes ROS 2 topics | `pip install mavsdk` |
| LK Optical Flow VO | OpenCV Lucas-Kanade tracker; produces relative pose | `pip install opencv-python` |
| YOLOv8-nano detector | Already chosen; ONNX CPU | `pip install ultralytics` |
| ROS 2 (optional for PoC) | Can be replaced with pure Python pub/sub | macOS binary via conda or brew |

**The critical realisation:** For Phase 0–2, you don't even need ROS 2 running. The perception pipeline and the drone sim can talk via plain Python queues or ZeroMQ until the interfaces are proven. Add ROS 2 wrapping once the algorithms work.

---

## Three Tiers of Complexity — Pick Where to Start

### Tier 1: Zero dependencies, running in 30 minutes

**No ROS 2. No Docker. Pure Python.**

```
conda activate aeroperception
pip install pymavlink mavsdk opencv-python ultralytics numpy
```

Five Python scripts, all in one folder:
1. `drone_sim.py` — fake drone, responds to MAVLink
2. `lk_vo.py` — optical flow, outputs pose
3. `detector.py` — YOLOv8, outputs detections
4. `planner.py` — P-controller, outputs velocity commands
5. `mission.py` — state machine, sends goals

They share state via a `SharedState` dataclass passed by reference. No message bus needed at this scale. This proves the algorithm loop works. **This is your Phase 0–2.**

### Tier 2: Add ArduPilot SITL via Docker (~1 hour setup)

Once Tier 1 is working, add a real autopilot in a container:

```bash
docker run -it --rm \
  -p 5760:5760 \
  -p 14550:14550 \
  --platform linux/amd64 \
  --env VEHICLE=ArduCopter \
  orthuk/ardupilot-sitl \
  ./Tools/autotest/sim_vehicle.py -v ArduCopter --no-rebuild \
  --out udp:host.docker.internal:14550
```

Your Python bridge connects to `udp:127.0.0.1:14550` instead of the fake sim. **Zero code changes in the perception or planning layers.** This is your Phase 3.

### Tier 3: Add ROS 2 wrapper (~2–3 hours)

Wrap each Python script as a ROS 2 node. The core logic doesn't change — you just add `rclpy` pub/sub around the existing functions. **This is your Phase 4.**

ROS 2 on Mac M1 is installable via:
```bash
conda install -c conda-forge ros-humble-desktop  # experimental but works
# OR: run a Ubuntu 22.04 Docker container with ROS 2 and mount your src
```

---

## Replacing OpenVINS: Lucas-Kanade Optical Flow VO

OpenVINS is a fantastic library. It is also a >500MB compile-from-source C++ project with 15 ROS 2 dependencies. For the PoC, replace it entirely.

### What LK Flow VO Gives You

Lucas-Kanade sparse optical flow tracks feature points frame-to-frame. Combined with the known camera intrinsics and a simple homography or essential matrix decomposition, it produces:
- Relative rotation (roll, pitch, yaw delta)
- Relative translation direction (scale is ambiguous without depth — same as monocular ORB-SLAM3 at init)
- A quality/confidence score (ratio of tracked features)

For the drone simulator, scale can be resolved by fusing with the simulated barometer altitude (which the fake drone publishes). For the hardware phase, you fuse with GPS or a pressure altimeter — exactly what OpenVINS does internally.

### The 80-Line VO

```python
# lk_vo.py — Lucas-Kanade Visual Odometry
import cv2
import numpy as np

class LKVisualOdometry:
    """
    Minimal monocular VO using Lucas-Kanade sparse optical flow.
    
    Inputs:  camera frames (numpy arrays)
    Outputs: pose delta (dx, dy, dz_relative, dyaw) per frame
    
    This is not as robust as OpenVINS or ORB-SLAM3.
    It is good enough to prove the perception → control loop.
    It is what gets replaced with OpenVINS in Phase 5.
    """
    
    def __init__(self, camera_matrix: np.ndarray, scale_from_baro: bool = True):
        self.K = camera_matrix
        self.scale_from_baro = scale_from_baro
        
        # LK parameters
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        self.feature_params = dict(
            maxCorners=200,
            qualityLevel=0.01,
            minDistance=10,
            blockSize=3
        )
        
        self.prev_frame = None
        self.prev_pts = None
        
        # Accumulated pose (ENU convention)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0  # overridden by baro when available
        self.yaw = 0.0
        
        self.tracking_quality = 0.0  # 0.0 (lost) to 1.0 (excellent)
    
    def update(self, frame: np.ndarray, baro_altitude_m: float = None) -> dict:
        """
        Process one frame. Returns pose estimate.
        Call at camera rate (30 Hz).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.prev_frame is None or self.prev_pts is None or len(self.prev_pts) < 20:
            # (Re)initialize: detect features
            self.prev_pts = cv2.goodFeaturesToTrack(gray, **self.feature_params)
            self.prev_frame = gray
            return self._make_pose(baro_altitude_m, quality=0.0, status="INITIALIZING")
        
        # Track features frame-to-frame
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_frame, gray, self.prev_pts, None, **self.lk_params
        )
        
        # Keep only successfully tracked points
        good_new = next_pts[status == 1]
        good_old = self.prev_pts[status == 1]
        
        self.tracking_quality = len(good_new) / max(len(self.prev_pts), 1)
        
        if len(good_new) < 8:
            # Not enough points for essential matrix — reinitialize
            self.prev_pts = None
            return self._make_pose(baro_altitude_m, quality=0.0, status="LOST")
        
        # Estimate relative camera motion via essential matrix
        E, mask = cv2.findEssentialMat(
            good_new, good_old, self.K,
            method=cv2.RANSAC, prob=0.999, threshold=1.0
        )
        
        if E is None:
            return self._make_pose(baro_altitude_m, quality=self.tracking_quality, status="DEGRADED")
        
        _, R, t, _ = cv2.recoverPose(E, good_new, good_old, self.K)
        
        # Extract yaw from rotation matrix
        yaw_delta = np.arctan2(R[1, 0], R[0, 0])
        self.yaw += yaw_delta
        
        # Translation: direction only (monocular — scale from baro or set to 1)
        scale = 1.0
        if baro_altitude_m is not None and self.scale_from_baro:
            # Simple scale from altitude change — imperfect but functional
            dz = baro_altitude_m - self.z
            t_norm = np.linalg.norm(t)
            if t_norm > 1e-6:
                scale = abs(dz) / t_norm if abs(dz) > 0.05 else 0.5
        
        # Update position (camera frame → ENU: rough approximation)
        self.x += scale * float(t[0])
        self.y += scale * float(t[1])
        
        if baro_altitude_m is not None:
            self.z = baro_altitude_m
        
        # Refresh features periodically
        if self.tracking_quality < 0.5:
            new_pts = cv2.goodFeaturesToTrack(gray, **self.feature_params)
            if new_pts is not None:
                self.prev_pts = np.vstack([good_new.reshape(-1, 1, 2), new_pts])
            else:
                self.prev_pts = good_new.reshape(-1, 1, 2)
        else:
            self.prev_pts = good_new.reshape(-1, 1, 2)
        
        self.prev_frame = gray
        
        return self._make_pose(baro_altitude_m, quality=self.tracking_quality, status="HEALTHY")
    
    def _make_pose(self, baro_alt, quality, status):
        return {
            "x": self.x,
            "y": self.y, 
            "z": baro_alt if baro_alt is not None else self.z,
            "yaw": self.yaw,
            "tracking_quality": quality,
            "status": status,
            "num_features": len(self.prev_pts) if self.prev_pts is not None else 0
        }
```

### Honest Assessment vs OpenVINS

| Property | LK Flow VO | OpenVINS |
|----------|-----------|---------|
| Drift | ~2–5% of distance | ~0.1–0.5% |
| IMU fusion | No | Yes (tight coupling) |
| Loop closure | No | No (but SLAM variants do) |
| Mac M1 install | `pip install opencv-python` | Hours of C++ compilation |
| Good enough for PoC? | **Yes** | Yes but overkill |
| Replace with in Phase 5? | Yes | Stays for hardware |

**The drift doesn't matter for the PoC.** You're proving the loop architecture works, not claiming centimetre accuracy.

---

## The Python Drone Simulator

This is the piece that replaces PX4 entirely for Tiers 1–2. It's a MAVLink-speaking process that:
- Accepts arm, mode, velocity setpoints
- Integrates velocity into position (simple Euler integration)
- Publishes HEARTBEAT, LOCAL_POSITION_NED, ATTITUDE, BATTERY_STATUS
- Obeys a basic gravity model (descends if no throttle)

```python
# drone_sim.py — minimal MAVLink drone simulator
"""
Exposes a UDP MAVLink interface on port 14550.
Responds correctly to arm/disarm, GUIDED mode, velocity setpoints.
Integrates physics at 100 Hz. Publishes state at 50 Hz.

Connect QGC to udp:localhost:14550 to see the vehicle on the map.
Connect your bridge code to the same port.
"""

import time
import math
import threading
import numpy as np
from pymavlink import mavutil

# --- Simulated vehicle state ---
class DroneState:
    def __init__(self):
        # Position in NED (meters from home)
        self.north = 0.0
        self.east = 0.0
        self.down = -0.5  # starts slightly above ground
        
        # Velocity in NED (m/s)
        self.vn = 0.0
        self.ve = 0.0
        self.vd = 0.0
        
        # Attitude (radians)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Mode and status
        self.armed = False
        self.mode = "STABILIZE"  # ArduCopter modes
        self.landed = True
        self.battery_remaining = 100  # percent
        
        # Setpoints (from companion)
        self.cmd_vn = 0.0
        self.cmd_ve = 0.0
        self.cmd_vd = 0.0
        self.cmd_yawrate = 0.0
        self.last_setpoint_time = 0.0

state = DroneState()

def physics_loop(dt=0.01):
    """Simple physics: integrate velocity, apply drag, gravity."""
    DRAG = 0.3       # velocity drag coefficient
    GRAVITY = 9.81
    SETPOINT_TIMEOUT = 0.5  # seconds
    
    while True:
        t = time.time()
        
        # If armed and in GUIDED mode and setpoints fresh
        if state.armed and state.mode == "GUIDED":
            if (t - state.last_setpoint_time) < SETPOINT_TIMEOUT:
                # Respond to velocity commands
                state.vn += (state.cmd_vn - state.vn) * DRAG * dt * 10
                state.ve += (state.cmd_ve - state.ve) * DRAG * dt * 10
                state.vd += (state.cmd_vd - state.vd) * DRAG * dt * 10
                state.yaw += state.cmd_yawrate * dt
            else:
                # Setpoint timeout: hover (zero velocity)
                state.vn *= (1 - DRAG * dt * 5)
                state.ve *= (1 - DRAG * dt * 5)
                state.vd *= (1 - DRAG * dt * 5)
        else:
            # Not in guided mode: drag to stop
            state.vn *= (1 - DRAG * dt * 3)
            state.ve *= (1 - DRAG * dt * 3)
            state.vd *= (1 - DRAG * dt * 3)
            
            # Gravity when not powered
            if not state.armed:
                state.vd += GRAVITY * 0.1 * dt
        
        # Integrate position
        state.north += state.vn * dt
        state.east  += state.ve * dt
        state.down  += state.vd * dt
        
        # Ground clamp
        if state.down > 0:
            state.down = 0
            state.vd = 0
            if state.armed and abs(state.vn) < 0.1 and abs(state.ve) < 0.1:
                state.landed = True
        else:
            state.landed = False
        
        # Battery drain (slow)
        if state.armed:
            state.battery_remaining -= 0.001 * dt
        
        time.sleep(dt)

def mavlink_loop():
    """MAVLink message handler: parse commands, publish state."""
    mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550', source_system=1)
    boot_time = time.time()
    last_heartbeat = 0
    last_state_pub = 0
    
    def send_heartbeat():
        mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0,  # base_mode
            0,  # custom_mode (0 = STABILIZE)
            mavutil.mavlink.MAV_STATE_ACTIVE if state.armed else mavutil.mavlink.MAV_STATE_STANDBY
        )
    
    def send_local_position():
        t_ms = int((time.time() - boot_time) * 1000)
        mav.mav.local_position_ned_send(
            t_ms,
            state.north, state.east, state.down,
            state.vn, state.ve, state.vd
        )
    
    def send_attitude():
        t_ms = int((time.time() - boot_time) * 1000)
        mav.mav.attitude_send(
            t_ms,
            state.roll, state.pitch, state.yaw,
            0.0, 0.0, 0.0  # angular rates
        )
    
    def send_battery():
        mav.mav.battery_status_send(
            0,  # id
            mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
            mavutil.mavlink.MAV_BATTERY_TYPE_LIPO,
            int(25 * 100),  # temperature (not used)
            [int(state.battery_remaining * 42),  # cell voltage (fake)
             65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535],
            -1,  # current
            int(state.battery_remaining * 100),  # battery_remaining %
            -1, -1, 0
        )
    
    while True:
        now = time.time()
        
        # Publish heartbeat at 1 Hz
        if now - last_heartbeat > 1.0:
            send_heartbeat()
            last_heartbeat = now
        
        # Publish state at 50 Hz
        if now - last_state_pub > 0.02:
            send_local_position()
            send_attitude()
            if int(now) % 5 == 0:  # battery at 0.2 Hz
                send_battery()
            last_state_pub = now
        
        # Handle incoming messages (non-blocking)
        msg = mav.recv_match(blocking=False)
        if msg:
            mtype = msg.get_type()
            
            if mtype == 'COMMAND_LONG':
                cmd = msg.command
                
                # ARM/DISARM
                if cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    state.armed = bool(msg.param1)
                    print(f"[SIM] {'ARMED' if state.armed else 'DISARMED'}")
                    mav.mav.command_ack_send(cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0, 0, 0)
                
                # TAKEOFF
                elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    target_alt = msg.param7  # altitude in meters
                    state.cmd_vd = -2.0  # climb at 2 m/s
                    state.mode = "GUIDED"
                    print(f"[SIM] TAKEOFF to {target_alt}m")
                    mav.mav.command_ack_send(cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0, 0, 0)
                
                # LAND  
                elif cmd == mavutil.mavlink.MAV_CMD_NAV_LAND:
                    state.cmd_vd = 1.0  # descend
                    print("[SIM] LANDING")
                    mav.mav.command_ack_send(cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0, 0, 0)
                
                # DO_SET_MODE
                elif cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    mode_map = {4: "GUIDED", 6: "RTL", 9: "LAND"}
                    state.mode = mode_map.get(int(msg.param2), state.mode)
                    print(f"[SIM] Mode: {state.mode}")
                    mav.mav.command_ack_send(cmd, mavutil.mavlink.MAV_RESULT_ACCEPTED, 0, 0, 0, 0)
            
            # VELOCITY SETPOINTS (offboard control)
            elif mtype == 'SET_POSITION_TARGET_LOCAL_NED':
                # type_mask: 0b0000111111000111 = velocity only
                state.cmd_vn = msg.vx
                state.cmd_ve = msg.vy
                state.cmd_vd = msg.vz
                state.cmd_yawrate = msg.yaw_rate
                state.last_setpoint_time = now
        
        time.sleep(0.002)  # 500 Hz loop

if __name__ == "__main__":
    print("[SIM] Starting drone simulator on UDP:14550")
    print("[SIM] Connect QGC to udp:localhost:14550")
    
    threading.Thread(target=physics_loop, daemon=True).start()
    mavlink_loop()  # blocks
```

---

## Revised Phase Plan

### Today's Goal (Tier 1 — no Docker, no ROS 2)

```bash
# 1. Install dependencies (~5 minutes)
conda activate aeroperception
pip install pymavlink mavsdk opencv-python ultralytics numpy

# 2. Start drone sim (Terminal 1)
python drone_sim.py

# 3. Run perception + planner (Terminal 2)
python perception_demo.py  # LK VO + YOLOv8 on webcam or video file

# 4. Connect QGC to udp:localhost:14550 and watch vehicle telemetry
```

You should have a "drone" responding to commands with a camera feed being processed within 30 minutes.

### This Week's Goal (Tier 2 — real autopilot)

```bash
# Replace drone_sim.py with ArduPilot SITL in Docker
docker run -it --rm -p 14550:14550 \
  --platform linux/amd64 \
  orthuk/ardupilot-sitl \
  ./Tools/autotest/sim_vehicle.py \
  -v ArduCopter --no-rebuild \
  --out udp:host.docker.internal:14550 --speedup=1
```

Everything else stays identical.

### Next Week's Goal (Tier 3 — ROS 2 wrappers)

Wrap the working Python scripts as ROS 2 nodes. The bridge to ArduPilot is MAVROS (well-supported, no uXRCE complexity). When you eventually move to PX4 hardware, you adapt the bridge layer only.

---

## What to Do Right Now

1. **Run the drone_sim.py script above.** It should start immediately on your conda env.
2. **Open QGC** and connect to `udp:localhost:14550`. You should see a vehicle on the map.
3. **Send it an arm+takeoff command** using QGC's Actions menu. Watch the position readout change.
4. **That's Phase 1 complete.** No PX4, no Docker, no compilation.

Then come back and we'll build the perception loop on top of it.

---

## What This Changes in the Architecture

The architecture documents stay valid. The only thing that changes is the bridge layer:

| | Original Plan | New Plan |
|---|---|---|
| SITL | PX4 + Gazebo | `drone_sim.py` → ArduPilot in Docker |
| Bridge | uXRCE-DDS | MAVSDK-Python / pymavlink |
| VIO | OpenVINS | LK Optical Flow (Phase 0–4) → OpenVINS (Phase 5 hardware) |
| Everything else | Unchanged | Unchanged |

The bridge swap (uXRCE → MAVROS/pymavlink) is the difference between one ADR. The perception pipeline, mission manager, planner, and logging are completely unaffected.

---

*The goal is momentum. Get something flying today. Harden it this week. Wrap it properly next week.*
