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