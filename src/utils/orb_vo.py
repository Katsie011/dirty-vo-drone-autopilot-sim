import math
import time
import cv2
from typing import Optional
import numpy as np

from .config import Config
from .physics_state import PhysicsState, FilterGains

# ─────────────────────────────────────────────────────────────────────────────
# VISUAL ODOMETRY (ORB + Essential Matrix + Physics Filter)
# ─────────────────────────────────────────────────────────────────────────────


class VisualOdometry:
    """
    Feature-based monocular VO with α-β physics filter integration.
    
    Architecture:
      Frame N → ORB features → match → Essential Matrix → R, t (raw measurement)
                ↓
      Physics Filter: PREDICT (constant velocity) → UPDATE (α-β fusion)
                ↓
      Smoothed state estimate (position, velocity, orientation)
    """

    STATUS_INITIALIZING = "INITIALIZING"
    STATUS_STATIONARY = "STATIONARY"
    STATUS_HEALTHY = "HEALTHY"
    STATUS_DEGRADED = "DEGRADED"
    STATUS_LOST = "LOST"

    def __init__(self, K: np.ndarray, use_physics: bool = True):
        self.K = K
        self.inv_K = np.linalg.inv(K)

        self._orb = cv2.ORB_create(
            nfeatures=Config.ORB_N_FEATURES,
            scaleFactor=Config.ORB_SCALE_FACTOR,
            nlevels=Config.ORB_N_LEVELS,
            edgeThreshold=Config.ORB_EDGE_THRESH,
            firstLevel=0,
            WTA_K=2,
            scoreType=cv2.ORB_HARRIS_SCORE,
            patchSize=31,
            fastThreshold=Config.ORB_FAST_THRESH,
        )

        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self._prev_gray: Optional[np.ndarray] = None
        self._prev_kp: Optional[tuple] = None
        self._prev_desc: Optional[np.ndarray] = None

        # Raw VO state (before physics filter)
        self._x_raw = 0.0
        self._y_raw = 0.0
        self._yaw_raw = 0.0

        # Physics filter
        self._use_physics = use_physics
        if use_physics:
            gains = FilterGains(
                alpha=0.6,   # Position measurement trust
                beta=0.4,    # Velocity adjustment gain
                gamma=0.7,   # Orientation measurement trust
                max_position_jump_m=2.0,
                max_velocity_m_s=5.0,
            )
            self._physics = PhysicsState(
                initial_pos=np.array([0.0, 0.0, Config.ASSUMED_HEIGHT_M]),
                gains=gains,
            )
        else:
            self._physics = None

        # Published state (from physics filter if enabled, else raw)
        self.x = 0.0
        self.y = 0.0
        self.z = Config.ASSUMED_HEIGHT_M
        self.yaw = 0.0
        self.vx = 0.0   # NEW: velocity estimates
        self.vy = 0.0
        self.vz = 0.0

        self.status = self.STATUS_INITIALIZING
        self.tracking_quality = 0.0
        self.num_features = 0
        self.num_matches = 0
        self.pixel_disp = 0.0
        self.frame_count = 0
        self.measurement_rejected = False  # NEW: physics gate indicator

        self.trajectory: list[tuple[float, float]] = [(0.0, 0.0)]

    def reset(self):
        self._x_raw = self._y_raw = self._yaw_raw = 0.0
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.trajectory = [(0.0, 0.0)]
        self._prev_gray = None
        self._prev_kp = None
        self._prev_desc = None
        self.status = self.STATUS_INITIALIZING
        self.pixel_disp = 0.0
        self.measurement_rejected = False
        if self._physics:
            self._physics.reset(pos=np.array([0.0, 0.0, Config.ASSUMED_HEIGHT_M]))

    def update(self, frame: np.ndarray) -> dict:
        self.frame_count += 1
        current_time = time.perf_counter()  # High-res timestamp

        blurred = cv2.GaussianBlur(frame, Config.BLUR_KERNEL, 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

        kp, desc = self._orb.detectAndCompute(gray, None)

        if desc is None or len(kp) < Config.MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_desc = desc
            self.status = self.STATUS_LOST
            self.num_features = len(kp) if kp else 0
            self.num_matches = 0
            self.pixel_disp = 0.0
            self.measurement_rejected = False
            # Physics prediction only (no measurement update)
            if self._physics:
                self._update_from_physics()
            return self._make_pose(viz_kp=kp)

        if self._prev_desc is None or len(self._prev_kp) < Config.MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_desc = desc
            self.status = self.STATUS_INITIALIZING
            self.num_features = len(kp)
            self.num_matches = 0
            self.pixel_disp = 0.0
            self.measurement_rejected = False
            if self._physics:
                self._update_from_physics()
            return self._make_pose(viz_kp=kp)

        matches = self._matcher.knnMatch(self._prev_desc, desc, k=2)

        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < Config.MATCH_RATIO_THRESH * n.distance:
                    good_matches.append(m)

        self.num_matches = len(good_matches)
        self.tracking_quality = len(good_matches) / max(len(self._prev_kp), 1)

        if len(good_matches) < Config.MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_desc = desc
            self.status = self.STATUS_LOST
            self.num_features = len(kp)
            self.pixel_disp = 0.0
            self.measurement_rejected = False
            if self._physics:
                self._update_from_physics()
            return self._make_pose(viz_kp=kp)

        pts_prev = np.float32([self._prev_kp[m.queryIdx].pt for m in good_matches])
        pts_curr = np.float32([kp[m.trainIdx].pt for m in good_matches])

        displacements = np.linalg.norm(pts_curr - pts_prev, axis=1)
        self.pixel_disp = float(np.median(displacements))

        if self.pixel_disp < Config.MIN_PIXEL_DISP:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_desc = desc
            self.status = self.STATUS_STATIONARY
            self.num_features = len(kp)
            self.measurement_rejected = False
            # Stationary — prediction only
            if self._physics:
                self._update_from_physics()
            return self._make_pose(viz_kp=kp, matches=good_matches)

        try:
            E, e_mask = cv2.findEssentialMat(
                pts_curr,
                pts_prev,
                self.K,
                method=cv2.RANSAC,
                prob=0.999,
                threshold=1.0,
            )
            if E is None or E.shape != (3, 3):
                raise ValueError("Degenerate E matrix")

            mask_flat = e_mask.ravel().astype(bool)
            inlier_prev = pts_prev[mask_flat]
            inlier_curr = pts_curr[mask_flat]
            inlier_ratio = len(inlier_curr) / max(len(pts_curr), 1)

            if inlier_ratio < Config.MIN_INLIER_RATIO or len(inlier_curr) < 8:
                raise ValueError(f"Inlier ratio too low: {inlier_ratio:.2f}")

            _, R, t, pose_mask = cv2.recoverPose(E, inlier_curr, inlier_prev, self.K)

        except (cv2.error, ValueError) as ex:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_desc = desc
            self.status = self.STATUS_DEGRADED
            self.num_features = len(kp)
            self.measurement_rejected = False
            if self._physics:
                self._update_from_physics()
            return self._make_pose(viz_kp=kp, matches=good_matches)

        # ── Extract yaw and translation from R, t ────────────────────
        rvec, _ = cv2.Rodrigues(R)
        yaw_delta = float(rvec[2, 0])
        yaw_delta = float(np.clip(yaw_delta, -0.15, 0.15))

        motion_ratio = min(self.pixel_disp / 5.0, 1.0)
        effective_scale = Config.ASSUMED_HEIGHT_M * Config.SCALE_FACTOR * motion_ratio

        if np.linalg.norm(t) < 1e-8:
            effective_scale = 0.0

        # ── Update raw VO state ───────────────────────────────────────
        self._yaw_raw += yaw_delta
        cos_y = math.cos(self._yaw_raw)
        sin_y = math.sin(self._yaw_raw)
        tx, ty = float(t[0, 0]), float(t[1, 0])
        dx_raw = effective_scale * (cos_y * tx - sin_y * ty)
        dy_raw = effective_scale * (sin_y * tx + cos_y * ty)
        self._x_raw += dx_raw
        self._y_raw += dy_raw

        # ── Fuse with physics filter ──────────────────────────────────
        if self._physics:
            measured_pos = np.array([self._x_raw, self._y_raw, Config.ASSUMED_HEIGHT_M])
            accepted = self._physics.update(measured_pos, self._yaw_raw, current_time)
            self.measurement_rejected = not accepted
            self._update_from_physics()
        else:
            # No physics — use raw VO
            self.x = self._x_raw
            self.y = self._y_raw
            self.yaw = self._yaw_raw
            self.measurement_rejected = False

        # ── Update trajectory (always use filtered position) ─────────
        self.trajectory.append((self.x, self.y))
        if len(self.trajectory) > 2000:
            self.trajectory.pop(0)

        # ── Update reference frame ────────────────────────────────────
        self._prev_gray = gray
        self._prev_kp = kp
        self._prev_desc = desc
        self.num_features = len(kp)

        self.status = (
            self.STATUS_HEALTHY if inlier_ratio > 0.65 else self.STATUS_DEGRADED
        )
        return self._make_pose(viz_kp=kp, matches=good_matches)

    def _update_from_physics(self):
        """Pull state from physics filter into published VO state."""
        if not self._physics:
            return
        state = self._physics.get_state()
        self.x = state["x"]
        self.y = state["y"]
        self.z = state["z"]
        self.yaw = state["yaw"]
        self.vx = state["vx"]
        self.vy = state["vy"]
        self.vz = state["vz"]

    def _make_pose(self, viz_kp, matches=None) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw_deg": math.degrees(self.yaw),
            "vx": self.vx,
            "vy": self.vy,
            "vz": self.vz,
            "tracking_quality": self.tracking_quality,
            "num_features": self.num_features,
            "num_matches": self.num_matches,
            "status": self.status,
            "pixel_disp": self.pixel_disp,
            "measurement_rejected": self.measurement_rejected,
            "keypoints": viz_kp,
            "matches": matches,
        }
