#!/usr/bin/env python3
"""
AeroPerception — Visual Odometry Demo  v2
==========================================
Run with:
    python main.py                   # default webcam
    python main.py --source 1        # external webcam
    python main.py --source video.mp4

Controls:
    R — reset pose  |  D — detections  |  F — features
    S — snapshot    |  Q / ESC — quit

Tuning flags (useful when calibrating):
    --scale N       larger → bigger movements on map (default 0.08)
    --min-disp N    larger → harder to trigger motion (default 0.8 px)
    --no-detector   skip YOLOv8 for faster startup

v2 bug-fixes vs v1:
  BUG 1 — Pose updated unconditionally even when stationary.
           Fixed: median pixel displacement gate; if Δpx < MIN_PIXEL_DISP,
           skip the pose update entirely (primary jitter fix).
  BUG 2 — Yaw extracted via atan2(R[1,0],R[0,0]) which conflates pitch/roll
           into the yaw estimate.
           Fixed: Rodrigues decomposition → z-axis component only.
  BUG 3 — recoverPose received ALL tracked points, not just RANSAC inliers.
           Fixed: apply e_mask from findEssentialMat before recoverPose.
  BUG 4 — Trajectory point appended before quality/inlier check.
           Fixed: append only after confirmed motion.
  NEW  — Bidirectional LK consistency check removes unreliable correspondences.
  NEW  — Gaussian pre-blur reduces pixel noise before feature detection.
  NEW  — EMA applied to *displayed* pose only (smooths render, not trajectory).
  NEW  — --scale and --min-disp CLI flags for interactive tuning.

Dependencies:  pip install opencv-python numpy ultralytics
"""

import argparse
import math
import queue
import sys
import threading
import time
import warnings
from typing import Optional

import cv2
import numpy as np

warnings.filterwarnings("ignore")

from config import Config
from obj_detector import DetectorThread
from visualizer import Visualiser


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


# ─────────────────────────────────────────────────────────────────────────────
# CAMERA INTRINSICS
# ─────────────────────────────────────────────────────────────────────────────


def estimate_K(w: int, h: int) -> np.ndarray:
    fx = (w / 2.0) / math.tan(math.radians(70.0) / 2.0)
    return np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1.0]], dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────


def main():
    ap = argparse.ArgumentParser(description="AeroPerception VO Demo v2")
    ap.add_argument("--source", default="0")
    ap.add_argument("--no-detector", action="store_true")
    ap.add_argument("--width", type=int, default=Config.FRAME_WIDTH)
    ap.add_argument("--height", type=int, default=Config.FRAME_HEIGHT)
    ap.add_argument(
        "--scale",
        type=float,
        default=None,
        help="Scale factor: bigger = larger map moves (default 0.08)",
    )
    ap.add_argument(
        "--min-disp",
        type=float,
        default=None,
        help="Stillness gate in pixels (default 0.8)",
    )
    args = ap.parse_args()

    if args.no_detector:
        Config.DETECTOR_ENABLED = False
    if args.scale is not None:
        Config.SCALE_FACTOR = args.scale
    if args.min_disp is not None:
        Config.MIN_PIXEL_DISP = args.min_disp

    src = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open: {args.source}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, Config.TARGET_FPS)

    ret, probe = cap.read()
    if not ret:
        print("[ERROR] No first frame")
        sys.exit(1)

    H, W = probe.shape[:2]
    print(f"[CAM]  {W}×{H}")

    K = estimate_K(W, H)
    vo = VisualOdometry(K)
    viz = Visualiser(W, H)

    print(
        f"[VO]   stillness gate={Config.MIN_PIXEL_DISP}px  scale={Config.SCALE_FACTOR}"
    )
    print(f"       Tune with:  --scale 0.12  --min-disp 1.2  etc.")

    det = DetectorThread() if Config.DETECTOR_ENABLED else None
    if det:
        print("[DET]  YOLOv8-nano loading in background…")

    win = "AeroPerception — VO Demo v2"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, viz._cw, viz._ch)

    sh = sd = True
    fi = si = 0

    print("\n  R:reset  D:dets  F:features  S:snap  Q:quit")
    print("  Watch 'Dpx' — near 0 when still, >1 when moving.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            if not isinstance(src, int):
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            break
        fi += 1
        pose = vo.update(frame)
        if det:
            det.submit(frame)
        dets = det.detections if det else []

        display = viz.render(frame, pose, dets, sh, sd, vo.trajectory, fi)
        cv2.imshow(win, display)

        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), ord("Q"), 27):
            break
        elif k in (ord("r"), ord("R")):
            vo.reset()
            print("[VO]  reset")
        elif k in (ord("d"), ord("D")):
            if det:
                det.toggle()
                sd = det.enabled
            else:
                sd = not sd
        elif k in (ord("f"), ord("F")):
            sh = not sh
        elif k in (ord("s"), ord("S")):
            p = f"snap_{si:04d}.png"
            cv2.imwrite(p, display)
            print(f"[SNAP] {p}")
            si += 1

    cap.release()
    cv2.destroyAllWindows()
    print("\n[DONE]")
    if len(vo.trajectory) > 1:
        xs, ys = zip(*vo.trajectory)
        dist = sum(
            math.hypot(
                vo.trajectory[i][0] - vo.trajectory[i - 1][0],
                vo.trajectory[i][1] - vo.trajectory[i - 1][1],
            )
            for i in range(1, len(vo.trajectory))
        )
        print(
            f"  Frames: {fi}  |  Path: {dist:.2f} m  |  "
            f"X: {min(xs):.2f}→{max(xs):.2f}  Y: {min(ys):.2f}→{max(ys):.2f}"
        )


if __name__ == "__main__":
    main()
