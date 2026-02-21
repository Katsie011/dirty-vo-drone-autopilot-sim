import math
import cv2
from typing import Optional
import numpy as np

from .config import Config

# ─────────────────────────────────────────────────────────────────────────────
# VISUAL ODOMETRY (ORB + Essential Matrix)
# ─────────────────────────────────────────────────────────────────────────────


class VisualOdometry:
    """
    Feature-based monocular VO: ORB → descriptor matching → Essential Matrix.
    """

    STATUS_INITIALIZING = "INITIALIZING"
    STATUS_STATIONARY = "STATIONARY"
    STATUS_HEALTHY = "HEALTHY"
    STATUS_DEGRADED = "DEGRADED"
    STATUS_LOST = "LOST"

    def __init__(self, K: np.ndarray):
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

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = Config.ASSUMED_HEIGHT_M
        self.yaw = 0.0

        self.status = self.STATUS_INITIALIZING
        self.tracking_quality = 0.0
        self.num_features = 0
        self.num_matches = 0
        self.pixel_disp = 0.0
        self.frame_count = 0

        self.trajectory: list[tuple[float, float]] = [(0.0, 0.0)]

    def reset(self):
        self._x = self._y = self._yaw = 0.0
        self.x = self.y = self.yaw = 0.0
        self.trajectory = [(0.0, 0.0)]
        self._prev_gray = None
        self._prev_kp = None
        self._prev_desc = None
        self.status = self.STATUS_INITIALIZING
        self.pixel_disp = 0.0

    def update(self, frame: np.ndarray) -> dict:
        self.frame_count += 1

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
            return self._make_pose(viz_kp=kp)

        if self._prev_desc is None or len(self._prev_kp) < Config.MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_desc = desc
            self.status = self.STATUS_INITIALIZING
            self.num_features = len(kp)
            self.num_matches = 0
            self.pixel_disp = 0.0
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
            self._ema()
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
            self._ema()
            return self._make_pose(viz_kp=kp, matches=good_matches)

        rvec, _ = cv2.Rodrigues(R)
        yaw_delta = float(rvec[2, 0])
        yaw_delta = float(np.clip(yaw_delta, -0.15, 0.15))

        motion_ratio = min(self.pixel_disp / 5.0, 1.0)
        effective_scale = Config.ASSUMED_HEIGHT_M * Config.SCALE_FACTOR * motion_ratio

        if np.linalg.norm(t) < 1e-8:
            effective_scale = 0.0

        self._yaw += yaw_delta
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        tx, ty = float(t[0, 0]), float(t[1, 0])
        self._x += effective_scale * (cos_y * tx - sin_y * ty)
        self._y += effective_scale * (sin_y * tx + cos_y * ty)

        self.trajectory.append((self._x, self._y))
        if len(self.trajectory) > 2000:
            self.trajectory.pop(0)

        self._prev_gray = gray
        self._prev_kp = kp
        self._prev_desc = desc
        self.num_features = len(kp)

        self.status = (
            self.STATUS_HEALTHY if inlier_ratio > 0.65 else self.STATUS_DEGRADED
        )
        self._ema()
        return self._make_pose(viz_kp=kp, matches=good_matches)

    def _ema(self):
        a = Config.POSE_EMA_ALPHA
        self.x = a * self.x + (1 - a) * self._x
        self.y = a * self.y + (1 - a) * self._y
        self.yaw = a * self.yaw + (1 - a) * self._yaw

    def _make_pose(self, viz_kp, matches=None) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw_deg": math.degrees(self.yaw),
            "tracking_quality": self.tracking_quality,
            "num_features": self.num_features,
            "num_matches": self.num_matches,
            "status": self.status,
            "pixel_disp": self.pixel_disp,
            "keypoints": viz_kp,
            "matches": matches,
        }
