# lk_vo.py — Lucas-Kanade Visual Odometry
import cv2
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# VISUAL ODOMETRY
# ─────────────────────────────────────────────────────────────────────────────


class VisualOdometry:
    """
    Monocular VO: Lucas-Kanade optical flow → Essential Matrix → pose.

    Internal pose  (_x, _y, _yaw)  accumulates only on confirmed motion.
    Displayed pose (x, y, yaw)     is EMA-smoothed for clean rendering.
    """

    STATUS_INITIALIZING = "INITIALIZING"
    STATUS_STATIONARY = "STATIONARY"
    STATUS_HEALTHY = "HEALTHY"
    STATUS_DEGRADED = "DEGRADED"
    STATUS_LOST = "LOST"

    def __init__(self, K: np.ndarray):
        self.K = K
        self.inv_K = np.linalg.inv(K)

        self._lk_params = dict(
            winSize=Config.LK_WIN_SIZE,
            maxLevel=Config.LK_MAX_LEVEL,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        self._feat_params = dict(
            maxCorners=Config.LK_MAX_CORNERS,
            qualityLevel=Config.LK_QUALITY,
            minDistance=Config.LK_MIN_DISTANCE,
            blockSize=3,
        )

        # Internal integrated pose
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # Displayed (EMA-smoothed) pose — these are what the UI reads
        self.x = 0.0
        self.y = 0.0
        self.z = Config.ASSUMED_HEIGHT_M
        self.yaw = 0.0

        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None

        self.status = self.STATUS_INITIALIZING
        self.tracking_quality = 0.0
        self.num_features = 0
        self.pixel_disp = 0.0
        self.frame_count = 0

        self.trajectory: list[tuple[float, float]] = [(0.0, 0.0)]

    # ─────────────────────────────────────────────────────────────────
    def reset(self):
        self._x = self._y = self._yaw = 0.0
        self.x = self.y = self.yaw = 0.0
        self.trajectory = [(0.0, 0.0)]
        self._prev_gray = None
        self._prev_pts = None
        self.status = self.STATUS_INITIALIZING
        self.pixel_disp = 0.0

    # ─────────────────────────────────────────────────────────────────
    def update(self, frame: np.ndarray) -> dict:
        self.frame_count += 1

        # Pre-blur to reduce sensor noise before tracking
        blurred = cv2.GaussianBlur(frame, Config.BLUR_KERNEL, 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

        # ── Init / reinit feature set ──────────────────────────────
        needs_init = (
            self._prev_gray is None
            or self._prev_pts is None
            or len(self._prev_pts) < Config.MIN_FEATURES
        )
        if needs_init:
            self._prev_pts = self._detect(gray)
            self._prev_gray = gray
            self.status = self.STATUS_INITIALIZING
            self.num_features = 0 if self._prev_pts is None else len(self._prev_pts)
            self.pixel_disp = 0.0
            return self._make_pose(viz_pts=None)

        # ── Forward LK track ───────────────────────────────────────
        next_pts, st_fwd, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **self._lk_params
        )

        # ── Backward LK check (bidirectional consistency) ──────────
        # Tracks points *back* from the new frame to the old frame.
        # If a point doesn't round-trip cleanly, it's unreliable — reject it.
        back_pts, st_bwd, _ = cv2.calcOpticalFlowPyrLK(
            gray, self._prev_gray, next_pts, None, **self._lk_params
        )
        fb_err = np.linalg.norm(
            self._prev_pts.reshape(-1, 2) - back_pts.reshape(-1, 2), axis=1
        )
        valid = (
            (st_fwd.reshape(-1) == 1)
            & (st_bwd.reshape(-1) == 1)
            & (fb_err < Config.FB_CHECK_THRESHOLD)
        )
        good_new = next_pts.reshape(-1, 2)[valid]
        good_old = self._prev_pts.reshape(-1, 2)[valid]

        self.tracking_quality = len(good_new) / max(len(self._prev_pts), 1)
        self.num_features = len(good_new)

        if len(good_new) < Config.MIN_FEATURES:
            self._prev_pts = None
            self.status = self.STATUS_LOST
            self.pixel_disp = 0.0
            return self._make_pose(viz_pts=None)

        # ── Stillness gate — PRIMARY JITTER FIX ───────────────────
        # Median pixel displacement tells us how much the scene actually moved.
        # Sub-pixel noise should NOT produce trajectory movement.
        displacements = np.linalg.norm(good_new - good_old, axis=1)
        self.pixel_disp = float(np.median(displacements))

        if self.pixel_disp < Config.MIN_PIXEL_DISP:
            # Camera is not moving — hold pose, just update tracking
            self._prev_gray = gray
            self._prev_pts = good_new.reshape(-1, 1, 2)
            self.status = self.STATUS_STATIONARY
            self._ema()
            return self._make_pose(viz_pts=good_new)

        # ── Essential matrix (RANSAC) ──────────────────────────────
        try:
            E, e_mask = cv2.findEssentialMat(
                good_new,
                good_old,
                self.K,
                method=cv2.RANSAC,
                prob=0.999,
                threshold=1.0,
            )
            if E is None or E.shape != (3, 3):
                raise ValueError("Degenerate E matrix")

            # BUG FIX: apply RANSAC inlier mask BEFORE recoverPose
            mask_flat = e_mask.ravel().astype(bool)
            inlier_new = good_new[mask_flat]
            inlier_old = good_old[mask_flat]
            inlier_ratio = len(inlier_new) / max(len(good_new), 1)

            if inlier_ratio < Config.MIN_INLIER_RATIO or len(inlier_new) < 8:
                raise ValueError(f"Inlier ratio too low: {inlier_ratio:.2f}")

            _, R, t, _ = cv2.recoverPose(E, inlier_new, inlier_old, self.K)

        except (cv2.error, ValueError):
            self.status = self.STATUS_DEGRADED
            self._prev_gray = gray
            self._prev_pts = good_new.reshape(-1, 1, 2)
            self._ema()
            return self._make_pose(viz_pts=good_new)

        # ── Yaw extraction — BUG FIX ──────────────────────────────
        # v1 used atan2(R[1,0], R[0,0]) which extracts the *full planar*
        # rotation including pitch/roll components → corrupted yaw.
        # Rodrigues gives us the rotation *vector*; its z-component is
        # the around-camera-axis rotation (our proxy for yaw).
        rvec, _ = cv2.Rodrigues(R)
        yaw_delta = float(rvec[2, 0])
        # Clamp: more than ~8° per frame at 30fps is almost certainly noise
        yaw_delta = float(np.clip(yaw_delta, -0.15, 0.15))

        # ── Scale → metres ─────────────────────────────────────────
        # Scale is proportional to pixel displacement, saturating at 5px.
        # This means sub-threshold noise produces near-zero position change.
        motion_ratio = min(self.pixel_disp / 5.0, 1.0)
        effective_scale = Config.ASSUMED_HEIGHT_M * Config.SCALE_FACTOR * motion_ratio

        if np.linalg.norm(t) < 1e-8:
            effective_scale = 0.0

        # ── Integrate pose ─────────────────────────────────────────
        self._yaw += yaw_delta
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        tx, ty = float(t[0, 0]), float(t[1, 0])
        self._x += effective_scale * (cos_y * tx - sin_y * ty)
        self._y += effective_scale * (sin_y * tx + cos_y * ty)

        # Append to trajectory only when we actually moved
        self.trajectory.append((self._x, self._y))
        if len(self.trajectory) > 2000:
            self.trajectory.pop(0)

        # ── Refresh feature set if degraded ───────────────────────
        if self.tracking_quality < Config.REINIT_THRESHOLD:
            new_pts = self._detect(gray)
            if new_pts is not None and len(new_pts) > 0:
                self._prev_pts = np.vstack([good_new.reshape(-1, 1, 2), new_pts])[
                    : Config.LK_MAX_CORNERS
                ]
            else:
                self._prev_pts = good_new.reshape(-1, 1, 2)
        else:
            self._prev_pts = good_new.reshape(-1, 1, 2)

        self._prev_gray = gray
        self.status = (
            self.STATUS_HEALTHY if inlier_ratio > 0.65 else self.STATUS_DEGRADED
        )
        self._ema()
        return self._make_pose(viz_pts=good_new)

    # ─────────────────────────────────────────────────────────────────
    def _ema(self):
        """Apply EMA to displayed pose. Smooths rendering only."""
        a = Config.POSE_EMA_ALPHA
        self.x = a * self.x + (1 - a) * self._x
        self.y = a * self.y + (1 - a) * self._y
        self.yaw = a * self.yaw + (1 - a) * self._yaw

    def _detect(self, gray: np.ndarray) -> Optional[np.ndarray]:
        return cv2.goodFeaturesToTrack(gray, **self._feat_params)

    def _make_pose(self, viz_pts) -> dict:
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw_deg": math.degrees(self.yaw),
            "tracking_quality": self.tracking_quality,
            "num_features": self.num_features,
            "status": self.status,
            "pixel_disp": self.pixel_disp,
            "tracked_points": viz_pts,
        }
