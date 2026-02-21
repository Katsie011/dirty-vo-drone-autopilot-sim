"""
Physics-Based State Estimator with α-β Filter
==============================================

Integrates physics prediction with visual odometry measurements.

Architecture:
  PREDICT step: propagate position using current velocity estimate
  UPDATE step:  fuse VO measurement with prediction using α-β gains

Why α-β over EKF for the MVP:
  - No covariance matrices to tune (simpler)
  - Computationally cheap (critical for MacBook Air CPU)
  - Good enough for constant-velocity assumption
  - Upgrades cleanly to EKF when we add IMU

The filter state:
  position    (x, y, z)         — metres in ENU frame
  velocity    (vx, vy, vz)      — m/s in ENU frame
  orientation (qw, qx, qy, qz)  — quaternion (world frame)
  ang_vel     (wx, wy, wz)      — rad/s body rates (future: from gyro)

Timestamp handling:
  Uses time.perf_counter() for sub-millisecond precision.
  Computes dt per frame — handles variable frame rate gracefully.
"""

import time
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass


@dataclass
class FilterGains:
    """α-β filter tuning parameters."""
    
    # Position gains
    alpha: float = 0.6   # How much to trust measurements (0=ignore, 1=full trust)
    beta:  float = 0.4   # How much to adjust velocity from residual
    
    # Orientation gain (simpler — just blend)
    gamma: float = 0.7   # Quaternion measurement weight
    
    # Sanity check gates
    max_position_jump_m:  float = 2.0   # Reject measurements > 2m from prediction
    max_velocity_m_s:     float = 5.0   # Reject induced velocities > 5 m/s
    max_yaw_rate_rad_s:   float = 1.5   # ~86 deg/s — reasonable for handheld


class PhysicsState:
    """
    6-DOF state estimator with α-β filtering.
    
    Separates PREDICT (physics) from UPDATE (vision).
    """
    
    def __init__(
        self,
        initial_pos: np.ndarray = None,
        initial_vel: np.ndarray = None,
        gains: FilterGains = None,
    ):
        # State vector
        self.p = initial_pos if initial_pos is not None else np.zeros(3)  # [x, y, z]
        self.v = initial_vel if initial_vel is not None else np.zeros(3)  # [vx, vy, vz]
        
        # Orientation as quaternion [w, x, y, z] (unit quaternion)
        # Start at identity (no rotation from world frame)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Angular velocity (body rates) — not used until we have IMU
        self.w = np.zeros(3)
        
        # Timing
        self._last_time: Optional[float] = None
        self._dt = 0.0
        
        # Filter gains
        self.gains = gains if gains is not None else FilterGains()
        
        # Diagnostics
        self.rejected_count = 0      # measurements rejected by sanity gate
        self.prediction_error = 0.0  # |measured - predicted| in metres
        self.velocity_smoothed = True
        
    # ──────────────────────────────────────────────────────────────────
    def reset(self, pos: np.ndarray = None):
        """Reset to origin or specified position."""
        self.p = pos if pos is not None else np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros(3)
        self._last_time = None
        self._dt = 0.0
        self.rejected_count = 0
    
    # ──────────────────────────────────────────────────────────────────
    def predict(self, current_time: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Physics prediction step: propagate state forward using current velocity.
        
        Returns:
            (predicted_position, predicted_quaternion)
        """
        if self._last_time is None:
            self._last_time = current_time
            self._dt = 0.0
            return self.p.copy(), self.q.copy()
        
        # Compute time delta (handles variable frame rate)
        self._dt = current_time - self._last_time
        self._last_time = current_time
        
        # Clamp dt to avoid explosions on paused frames
        if self._dt > 0.5:  # 500ms max (2 fps minimum)
            self._dt = 0.5
        
        if self._dt < 1e-6:  # avoid division by zero
            return self.p.copy(), self.q.copy()
        
        # ── Position prediction: constant velocity model ──────────────
        # p_predicted = p + v * dt
        p_pred = self.p + self.v * self._dt
        
        # ── Orientation prediction: integrate angular velocity ───────
        # For now, we don't have gyro data, so we hold orientation constant.
        # When IMU is added, this becomes:
        #   q_pred = q ⊗ exp(0.5 * w * dt)
        # where ⊗ is quaternion multiplication and exp is the exponential map.
        q_pred = self.q.copy()
        
        return p_pred, q_pred
    
    # ──────────────────────────────────────────────────────────────────
    def update(
        self,
        measured_pos: np.ndarray,
        measured_yaw: float,
        current_time: float,
    ) -> bool:
        """
        Measurement update step: fuse VO measurement with prediction.
        
        Args:
            measured_pos:  [x, y, z] from VO (ENU frame)
            measured_yaw:  yaw angle in radians
            current_time:  timestamp from time.perf_counter()
        
        Returns:
            True if measurement was accepted, False if rejected by sanity gate
        """
        # ── PREDICT ────────────────────────────────────────────────────
        p_pred, q_pred = self.predict(current_time)
        
        # ── SANITY GATE ────────────────────────────────────────────────
        # Compute prediction error (residual)
        residual = measured_pos - p_pred
        self.prediction_error = float(np.linalg.norm(residual))
        
        # Gate 1: Position jump check
        if self.prediction_error > self.gains.max_position_jump_m:
            self.rejected_count += 1
            # Do NOT update state — hold prediction
            return False
        
        # Compute induced velocity from measurement
        if self._dt > 1e-6:
            v_induced = residual / self._dt
            v_induced_mag = np.linalg.norm(v_induced)
            
            # Gate 2: Velocity sanity check
            if v_induced_mag > self.gains.max_velocity_m_s:
                self.rejected_count += 1
                return False
        else:
            v_induced = np.zeros(3)
        
        # ── α-β UPDATE ─────────────────────────────────────────────────
        # Position update: blend prediction with measurement
        # p_new = p_pred + α * residual
        self.p = p_pred + self.gains.alpha * residual
        
        # Velocity update: adjust velocity estimate from residual
        # v_new = v + β * residual / dt
        if self._dt > 1e-6:
            self.v = self.v + (self.gains.beta / self._dt) * residual
        
        # ── Orientation update ─────────────────────────────────────────
        # Convert measured yaw to quaternion (rotation around z-axis)
        # q = [cos(θ/2), 0, 0, sin(θ/2)]
        half_yaw = measured_yaw / 2.0
        q_meas = np.array([
            np.cos(half_yaw),  # w
            0.0,               # x
            0.0,               # y
            np.sin(half_yaw),  # z
        ])
        
        # Blend quaternions using SLERP (Spherical Linear Interpolation)
        # This is geometrically correct — simple average is not!
        self.q = self._slerp(q_pred, q_meas, self.gains.gamma)
        
        # Normalize to maintain unit quaternion (numerical stability)
        self.q /= np.linalg.norm(self.q)
        
        return True
    
    # ──────────────────────────────────────────────────────────────────
    def get_state(self) -> dict:
        """Return current state as a dict (for visualization / logging)."""
        # Extract yaw from quaternion for easier interpretation
        # yaw = atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y^2 + q_z^2))
        qw, qx, qy, qz = self.q
        yaw = np.arctan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0*(qy**2 + qz**2))
        
        return {
            "x":        float(self.p[0]),
            "y":        float(self.p[1]),
            "z":        float(self.p[2]),
            "vx":       float(self.v[0]),
            "vy":       float(self.v[1]),
            "vz":       float(self.v[2]),
            "yaw":      float(yaw),
            "yaw_deg":  float(np.degrees(yaw)),
            "quat":     self.q.tolist(),
            "dt":       float(self._dt),
            "pred_err": float(self.prediction_error),
        }
    
    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """
        Spherical Linear Interpolation between two quaternions.
        
        Args:
            q1, q2: Unit quaternions [w, x, y, z]
            t:      Interpolation parameter (0 = q1, 1 = q2)
        
        Returns:
            Interpolated quaternion
        """
        # Compute dot product (cosine of angle between quaternions)
        dot = np.dot(q1, q2)
        
        # If dot < 0, quaternions are on opposite hemispheres
        # Negate q2 to take the shorter path
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # Clamp dot to avoid numerical issues with arccos
        dot = np.clip(dot, -1.0, 1.0)
        
        # If quaternions are very close, use linear interpolation
        # (avoids division by zero in slerp formula)
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # Standard SLERP formula
        theta = np.arccos(dot)
        sin_theta = np.sin(theta)
        w1 = np.sin((1.0 - t) * theta) / sin_theta
        w2 = np.sin(t * theta) / sin_theta
        
        return w1 * q1 + w2 * q2


# ──────────────────────────────────────────────────────────────────────────────
# HELPER: Quaternion to Euler (for debugging / visualization)
# ──────────────────────────────────────────────────────────────────────────────

def quaternion_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """
    Convert quaternion [w, x, y, z] to Euler angles [roll, pitch, yaw] in radians.
    
    Returns:
        (roll, pitch, yaw) in radians
    """
    qw, qx, qy, qz = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx**2 + qy**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)  # handle numerical issues
    pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy**2 + qz**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles [roll, pitch, yaw] in radians to quaternion [w, x, y, z].
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return np.array([qw, qx, qy, qz])
