#!/usr/bin/env python3
"""
AeroPerception — Visual Odometry Demo
======================================
Run with:
    python main.py                  # uses default webcam
    python main.py --source 1       # alternate camera index
    python main.py --source video.mp4

Controls:
    R — reset pose to origin
    D — toggle detections on/off
    F — toggle feature points on/off
    S — save snapshot
    Q / ESC — quit

What this demonstrates:
    • Lucas-Kanade sparse optical flow tracking
    • Monocular VO producing a 2D trajectory (x/y ground plane)
    • YOLOv8-nano object detection running on a background thread
    • Live telemetry overlay

This is Phase 2 of the AeroPerception MVP. No ROS 2, no autopilot, no
compilation required. It is the perception loop in isolation — the piece
that will be wrapped as a ROS 2 node and connected to the drone bridge
in Phase 3.

Dependencies:
    pip install opencv-python numpy ultralytics
"""

import argparse
import sys
import time
import math
import warnings
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from config import Config
from obj_detector import DetectorThread


# Suppress ultralytics output spam
warnings.filterwarnings("ignore")



# ─────────────────────────────────────────────────────────────────────────────
# VISUAL ODOMETRY
# ─────────────────────────────────────────────────────────────────────────────

class VisualOdometry:
    """
    Monocular Visual Odometry using Lucas-Kanade sparse optical flow.

    Produces a 2D trajectory estimate in the x-y plane (ground plane
    approximation) plus a yaw estimate. Z is held at ASSUMED_HEIGHT_M
    since we have no barometer on the laptop webcam.

    This is deliberately simple — it is not production VO. It is the
    minimal implementation that proves the perception→control loop can
    work. It will be replaced by OpenVINS on hardware.
    """

    STATUS_INITIALIZING = "INITIALIZING"
    STATUS_HEALTHY      = "HEALTHY"
    STATUS_DEGRADED     = "DEGRADED"
    STATUS_LOST         = "LOST"

    def __init__(self, camera_matrix: np.ndarray):
        self.K = camera_matrix
        self.inv_K = np.linalg.inv(camera_matrix)

        # LK parameters
        self._lk_params = dict(
            winSize=Config.LK_WIN_SIZE,
            maxLevel=Config.LK_MAX_LEVEL,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                30, 0.01
            )
        )
        self._feat_params = dict(
            maxCorners=Config.LK_MAX_CORNERS,
            qualityLevel=Config.LK_QUALITY,
            minDistance=Config.LK_MIN_DISTANCE,
            blockSize=3
        )

        # State
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts:  Optional[np.ndarray] = None

        # Accumulated pose (ENU: x=East, y=North, z=Up)
        self.x   = 0.0
        self.y   = 0.0
        self.z   = Config.ASSUMED_HEIGHT_M
        self.yaw = 0.0

        # Diagnostics
        self.status            = self.STATUS_INITIALIZING
        self.tracking_quality  = 0.0     # 0–1
        self.num_features      = 0
        self.frame_count       = 0

        # Trajectory history (for plotting)
        self.trajectory: list[tuple[float, float]] = [(0.0, 0.0)]
        self._max_traj_len = 2000

    # ------------------------------------------------------------------
    def reset(self):
        """Reset pose to origin (press R)."""
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.trajectory = [(0.0, 0.0)]
        self._prev_gray = None
        self._prev_pts  = None
        self.status = self.STATUS_INITIALIZING

    # ------------------------------------------------------------------
    def update(self, frame: np.ndarray) -> dict:
        """
        Process one frame. Returns current pose estimate.
        Should be called at camera rate (≤60 Hz).
        """
        self.frame_count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── Initialise / reinitialise feature set ──────────────────────
        needs_init = (
            self._prev_gray is None
            or self._prev_pts is None
            or len(self._prev_pts) < Config.MIN_FEATURES
        )
        if needs_init:
            self._prev_pts  = self._detect_features(gray)
            self._prev_gray = gray
            self.status = self.STATUS_INITIALIZING
            self.num_features = 0 if self._prev_pts is None else len(self._prev_pts)
            return self._pose_dict(good_new=None)

        # ── Track features ──────────────────────────────────────────────
        next_pts, st, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **self._lk_params
        )

        good_new = next_pts[st == 1]
        good_old = self._prev_pts[st == 1]

        total = max(len(self._prev_pts), 1)
        self.tracking_quality = len(good_new) / total
        self.num_features     = len(good_new)

        # ── Quality check ────────────────────────────────────────────────
        if len(good_new) < Config.MIN_FEATURES:
            self._prev_pts  = None   # will reinit next frame
            self.status = self.STATUS_LOST
            return self._pose_dict(good_new=None)

        # ── Estimate motion via Essential Matrix ────────────────────────
        try:
            E, mask = cv2.findEssentialMat(
                good_new, good_old, self.K,
                method=cv2.RANSAC, prob=0.999, threshold=1.0
            )
            if E is None or E.shape != (3, 3):
                raise ValueError("Bad E matrix")

            _, R, t, inlier_mask = cv2.recoverPose(E, good_new, good_old, self.K)

            # Inlier ratio
            inlier_ratio = float(np.sum(inlier_mask)) / max(len(good_new), 1)

        except (cv2.error, ValueError):
            self.status = self.STATUS_DEGRADED
            self._prev_gray = gray
            self._prev_pts  = good_new.reshape(-1, 1, 2)
            return self._pose_dict(good_new=good_new)

        # ── Scale recovery (height heuristic) ───────────────────────────
        # recoverPose returns unit translation. We scale by assumed height.
        # On the real vehicle this is replaced by barometer or GPS altitude.
        t_norm = float(np.linalg.norm(t))
        if t_norm > 1e-6:
            scale = Config.ASSUMED_HEIGHT_M * 0.05   # conservative scale factor
        else:
            scale = 0.0

        # ── Update pose ─────────────────────────────────────────────────
        # Extract yaw from rotation (small angle approx)
        yaw_delta = math.atan2(float(R[1, 0]), float(R[0, 0]))
        self.yaw += yaw_delta

        # Rotate translation into world frame
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        # print(f"T has shape:{t.shape}")
        dx =  scale * (cos_y * float(t[0, 0]) - sin_y * float(t[1, 0]))
        dy =  scale * (sin_y * float(t[0, 0]) + cos_y * float(t[1, 0]))

        self.x += dx
        self.y += dy

        # Record trajectory
        self.trajectory.append((self.x, self.y))
        if len(self.trajectory) > self._max_traj_len:
            self.trajectory.pop(0)

        # ── Refresh features ─────────────────────────────────────────────
        if self.tracking_quality < Config.REINIT_THRESHOLD:
            new_pts = self._detect_features(gray)
            if new_pts is not None and len(new_pts) > 0:
                self._prev_pts = np.vstack([
                    good_new.reshape(-1, 1, 2),
                    new_pts
                ])[:Config.LK_MAX_CORNERS]
        else:
            self._prev_pts = good_new.reshape(-1, 1, 2)

        self._prev_gray = gray

        self.status = (
            self.STATUS_HEALTHY  if inlier_ratio > 0.6
            else self.STATUS_DEGRADED
        )

        return self._pose_dict(good_new=good_new)

    # ------------------------------------------------------------------
    def _detect_features(self, gray: np.ndarray) -> Optional[np.ndarray]:
        pts = cv2.goodFeaturesToTrack(gray, **self._feat_params)
        return pts

    def _pose_dict(self, good_new) -> dict:
        return {
            "x":               self.x,
            "y":               self.y,
            "z":               self.z,
            "yaw_deg":         math.degrees(self.yaw),
            "tracking_quality": self.tracking_quality,
            "num_features":    self.num_features,
            "status":          self.status,
            "tracked_points":  good_new,
        }



# ─────────────────────────────────────────────────────────────────────────────
# CAMERA INTRINSICS
# ─────────────────────────────────────────────────────────────────────────────

def estimate_camera_matrix(width: int, height: int) -> np.ndarray:
    """
    Estimate camera intrinsics from image dimensions.

    A proper calibration (using a checkerboard + cv2.calibrateCamera) will
    give better VO results. For the demo, a standard approximation works.

    Typical MacBook webcam: ~70° horizontal FOV.
    """
    fov_h_deg = 70.0
    fov_h_rad = math.radians(fov_h_deg)
    fx = (width / 2.0) / math.tan(fov_h_rad / 2.0)
    fy = fx  # square pixels assumed
    cx = width  / 2.0
    cy = height / 2.0
    return np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────────────
# VISUALISER
# ─────────────────────────────────────────────────────────────────────────────

class Visualiser:
    """
    Composes the display frame from the camera feed + overlays + telemetry.

    Layout:
    ┌──────────────────────────┬─────────────────┐
    │  Camera feed             │  Trajectory map │
    │  + feature points        │  (top-down)     │
    │  + detections            │                 │
    ├──────────────────────────┴─────────────────┤
    │  Telemetry bar                              │
    └─────────────────────────────────────────────┘
    """

    TELEMETRY_H = 80
    TRAJ_W = Config.TRAJ_SIZE
    TRAJ_H = Config.TRAJ_SIZE

    def __init__(self, frame_w: int, frame_h: int):
        self._fw = frame_w
        self._fh = frame_h

        # Scale factor to fit camera into left panel
        total_w      = Config.VIZ_WIDTH
        left_w       = total_w - self.TRAJ_W
        self._scale  = min(left_w / frame_w, Config.VIZ_HEIGHT / frame_h)
        self._disp_w = int(frame_w * self._scale)
        self._disp_h = int(frame_h * self._scale)

        self._canvas_w = self._disp_w + self.TRAJ_W
        self._canvas_h = self._disp_h + self.TELEMETRY_H

        # Font
        self._font      = cv2.FONT_HERSHEY_SIMPLEX
        self._font_mono = cv2.FONT_HERSHEY_PLAIN

        # FPS tracking
        self._fps_times: list[float] = []

    # ------------------------------------------------------------------
    def render(
        self,
        frame:      np.ndarray,
        pose:       dict,
        detections: list,
        show_feats: bool,
        show_dets:  bool,
        traj:       list,
        frame_idx:  int,
    ) -> np.ndarray:
        """Build and return the full display frame."""
        canvas = np.full(
            (self._canvas_h, self._canvas_w, 3),
            Config.COL_BG, dtype=np.uint8
        )

        # ── Camera panel ──────────────────────────────────────────────
        cam = cv2.resize(frame, (self._disp_w, self._disp_h))

        # Feature points overlay
        if show_feats and pose["tracked_points"] is not None:
            pts = pose["tracked_points"]
            for pt in pts:
                px = int(pt[0] * self._scale)
                py = int(pt[1] * self._scale)
                cv2.circle(cam, (px, py), 3, Config.COL_GREEN, -1)
                cv2.circle(cam, (px, py), 5, Config.COL_GREEN,  1)

        # Detection bounding boxes
        if show_dets:
            scale_x = self._disp_w / self._fw
            scale_y = self._disp_h / self._fh
            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                x1 = int(x1 * scale_x); y1 = int(y1 * scale_y)
                x2 = int(x2 * scale_x); y2 = int(y2 * scale_y)
                cv2.rectangle(cam, (x1, y1), (x2, y2), Config.COL_ORANGE, 2)
                label = f"{det['label']} {det['conf']:.0%}"
                lw, lh = self._text_size(label, 0.55, 1)
                cv2.rectangle(cam, (x1, y1 - lh - 6), (x1 + lw + 6, y1),
                              Config.COL_ORANGE, -1)
                cv2.putText(cam, label, (x1 + 3, y1 - 4),
                            self._font, 0.55, Config.COL_BG, 1, cv2.LINE_AA)

        # Status chip
        status   = pose["status"]
        stat_col = {
            "HEALTHY":     Config.COL_GREEN,
            "DEGRADED":    Config.COL_ORANGE,
            "LOST":        Config.COL_RED,
            "INITIALIZING": Config.COL_GREY,
        }.get(status, Config.COL_GREY)

        self._chip(cam, status, (12, 12), stat_col)

        canvas[:self._disp_h, :self._disp_w] = cam

        # ── Trajectory panel ──────────────────────────────────────────
        traj_panel = self._draw_trajectory(traj, pose)
        canvas[:self.TRAJ_H, self._disp_w:self._disp_w + self.TRAJ_W] = traj_panel

        # ── Telemetry bar ──────────────────────────────────────────────
        tele_y = self._disp_h
        self._draw_telemetry(canvas, tele_y, pose, frame_idx)

        return canvas

    # ------------------------------------------------------------------
    def _draw_trajectory(self, traj: list, pose: dict) -> np.ndarray:
        """Top-down trajectory plot."""
        panel = np.full((self.TRAJ_H, self.TRAJ_W, 3), Config.COL_PANEL, dtype=np.uint8)

        # Grid
        cx = self.TRAJ_W // 2
        cy = self.TRAJ_H // 2
        for d in range(1, 6):
            r = int(d * Config.TRAJ_SCALE)
            cv2.circle(panel, (cx, cy), r, (40, 42, 50), 1)
        cv2.line(panel, (0, cy), (self.TRAJ_W, cy), (40, 42, 50), 1)
        cv2.line(panel, (cx, 0), (cx, self.TRAJ_H), (40, 42, 50), 1)

        # Axis labels
        cv2.putText(panel, "N", (cx + 4, 14),
                    self._font, 0.4, Config.COL_GREY, 1, cv2.LINE_AA)
        cv2.putText(panel, "E", (self.TRAJ_W - 16, cy - 4),
                    self._font, 0.4, Config.COL_GREY, 1, cv2.LINE_AA)

        # Title
        cv2.putText(panel, "TRAJECTORY", (8, 18),
                    self._font, 0.45, Config.COL_ACCENT, 1, cv2.LINE_AA)

        # Draw path
        if len(traj) > 1:
            pts_px = []
            for (tx, ty) in traj:
                px = cx + int(tx * Config.TRAJ_SCALE)
                py = cy - int(ty * Config.TRAJ_SCALE)
                px = max(0, min(self.TRAJ_W - 1, px))
                py = max(0, min(self.TRAJ_H - 1, py))
                pts_px.append((px, py))

            # Fade older path segments
            n = len(pts_px)
            for i in range(1, n):
                alpha = i / n
                col = tuple(int(c * alpha) for c in Config.COL_TRACK)
                cv2.line(panel, pts_px[i-1], pts_px[i], col, 1)

            # Bright recent portion
            recent = pts_px[max(0, n-60):]
            for i in range(1, len(recent)):
                cv2.line(panel, recent[i-1], recent[i], Config.COL_ACCENT, 2)

        # Current position marker
        cur_px = cx + int(pose["x"] * Config.TRAJ_SCALE)
        cur_py = cy - int(pose["y"] * Config.TRAJ_SCALE)
        cur_px = max(2, min(self.TRAJ_W - 3, cur_px))
        cur_py = max(2, min(self.TRAJ_H - 3, cur_py))

        # Heading arrow
        yaw = math.radians(pose["yaw_deg"])
        arrow_len = 16
        ax = cur_px + int(arrow_len * math.sin(yaw))
        ay = cur_py - int(arrow_len * math.cos(yaw))
        cv2.arrowedLine(panel, (cur_px, cur_py), (ax, ay),
                        Config.COL_WHITE, 2, tipLength=0.35)
        cv2.circle(panel, (cur_px, cur_py), 5, Config.COL_WHITE, -1)
        cv2.circle(panel, (cur_px, cur_py), 7, Config.COL_ACCENT,  2)

        # Scale indicator
        scale_px = Config.TRAJ_SCALE
        bar_x = 8; bar_y = self.TRAJ_H - 16
        cv2.line(panel, (bar_x, bar_y), (bar_x + scale_px, bar_y),
                 Config.COL_GREY, 1)
        cv2.line(panel, (bar_x, bar_y - 3), (bar_x, bar_y + 3),
                 Config.COL_GREY, 1)
        cv2.line(panel, (bar_x + scale_px, bar_y - 3),
                 (bar_x + scale_px, bar_y + 3), Config.COL_GREY, 1)
        cv2.putText(panel, "1 m", (bar_x + scale_px // 2 - 10, bar_y - 5),
                    self._font, 0.35, Config.COL_GREY, 1, cv2.LINE_AA)

        return panel

    # ------------------------------------------------------------------
    def _draw_telemetry(
        self, canvas: np.ndarray, y: int, pose: dict, frame_idx: int
    ):
        """Bottom telemetry bar."""
        now = time.time()
        self._fps_times.append(now)
        self._fps_times = [t for t in self._fps_times if now - t < 1.0]
        fps = len(self._fps_times)

        # Background
        cv2.rectangle(canvas,
                      (0, y), (self._canvas_w, y + self.TELEMETRY_H),
                      Config.COL_PANEL, -1)
        cv2.line(canvas, (0, y), (self._canvas_w, y), (50, 50, 60), 1)

        # Left block: position
        fields = [
            ("X",    f"{pose['x']:+7.2f} m"),
            ("Y",    f"{pose['y']:+7.2f} m"),
            ("Z",    f"{pose['z']:+7.2f} m"),
            ("YAW",  f"{pose['yaw_deg']:+7.1f}°"),
        ]
        col_w = 130
        for i, (key, val) in enumerate(fields):
            bx = 12 + i * col_w
            by = y + 24
            cv2.putText(canvas, key, (bx, by),
                        self._font, 0.38, Config.COL_GREY, 1, cv2.LINE_AA)
            cv2.putText(canvas, val, (bx, by + 24),
                        self._font, 0.55, Config.COL_WHITE, 1, cv2.LINE_AA)

        # Middle block: quality bar
        qx = 12 + 4 * col_w + 20
        qy = y + 20
        cv2.putText(canvas, "TRACK QUALITY", (qx, qy),
                    self._font, 0.38, Config.COL_GREY, 1, cv2.LINE_AA)
        bar_w = 160; bar_h = 12
        cv2.rectangle(canvas, (qx, qy + 8), (qx + bar_w, qy + 8 + bar_h),
                      (50, 50, 60), -1)
        q = max(0.0, min(1.0, pose["tracking_quality"]))
        q_col = (
            Config.COL_GREEN  if q > 0.6 else
            Config.COL_ORANGE if q > 0.3 else
            Config.COL_RED
        )
        cv2.rectangle(canvas,
                      (qx, qy + 8),
                      (qx + int(bar_w * q), qy + 8 + bar_h),
                      q_col, -1)
        cv2.putText(canvas, f"{q:.0%}  {pose['num_features']} pts",
                    (qx, qy + 38),
                    self._font, 0.45, Config.COL_WHITE, 1, cv2.LINE_AA)

        # Right block: fps + frame
        rx = self._canvas_w - 120
        cv2.putText(canvas, f"{fps:2d} FPS", (rx, y + 28),
                    self._font, 0.55, Config.COL_ACCENT, 1, cv2.LINE_AA)
        cv2.putText(canvas, f"#{frame_idx:06d}", (rx, y + 54),
                    self._font, 0.4, Config.COL_GREY, 1, cv2.LINE_AA)

        # Key hints
        hints = "R:reset  D:dets  F:feats  S:snap  Q:quit"
        cv2.putText(canvas, hints,
                    (12, y + self.TELEMETRY_H - 8),
                    self._font, 0.35, Config.COL_GREY, 1, cv2.LINE_AA)

    # ------------------------------------------------------------------
    @staticmethod
    def _text_size(text: str, scale: float, thickness: int) -> tuple[int, int]:
        (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        return w, h

    @staticmethod
    def _chip(img: np.ndarray, text: str, pos: tuple, color: tuple):
        """Rounded status chip."""
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        x, y = pos
        pad = 6
        cv2.rectangle(img, (x, y), (x + tw + pad*2, y + th + pad*2),
                      color, -1, cv2.LINE_AA)
        cv2.putText(img, text, (x + pad, y + th + pad - 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="AeroPerception — Visual Odometry Demo"
    )
    parser.add_argument(
        "--source", default="0",
        help="Camera index (0,1,2…) or path to video file (default: 0)"
    )
    parser.add_argument(
        "--no-detector", action="store_true",
        help="Skip loading YOLOv8 (faster startup, no detections)"
    )
    parser.add_argument(
        "--width", type=int, default=Config.FRAME_WIDTH,
        help="Capture width (default 1280)"
    )
    parser.add_argument(
        "--height", type=int, default=Config.FRAME_HEIGHT,
        help="Capture height (default 720)"
    )
    args = parser.parse_args()

    if args.no_detector:
        Config.DETECTOR_ENABLED = False

    # ── Open capture ────────────────────────────────────────────────────
    source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(source)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera/video: {args.source}")
        print("  Try: python main.py --source 0   (built-in webcam)")
        print("       python main.py --source 1   (external webcam)")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS,          Config.TARGET_FPS)

    # Read actual dimensions (may differ from requested)
    ret, probe = cap.read()
    if not ret:
        print("[ERROR] Could not read first frame.")
        sys.exit(1)

    actual_h, actual_w = probe.shape[:2]
    print(f"[CAM]  {actual_w}×{actual_h} @ {Config.TARGET_FPS} fps")

    # ── Init subsystems ─────────────────────────────────────────────────
    K   = estimate_camera_matrix(actual_w, actual_h)
    vo  = VisualOdometry(K)
    viz = Visualiser(actual_w, actual_h)

    print("[VO]   Lucas-Kanade optical flow ready")
    print(f"       K = fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

    if Config.DETECTOR_ENABLED:
        detector = DetectorThread()
        print(f"[DET]  YOLOv8-nano loading in background (model: {Config.DETECTOR_MODEL})")
    else:
        detector = None
        print("[DET]  Detector disabled (--no-detector)")

    # ── Window ──────────────────────────────────────────────────────────
    win_name = "AeroPerception — VO Demo"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, viz._canvas_w, viz._canvas_h)

    show_features  = True
    show_detections = True
    frame_idx = 0
    snap_idx  = 0

    print()
    print("━" * 50)
    print("  CONTROLS")
    print("  R — reset pose to origin")
    print("  D — toggle object detections")
    print("  F — toggle feature point overlay")
    print("  S — save snapshot")
    print("  Q / ESC — quit")
    print("━" * 50)
    print()
    print("  Move the camera slowly to build a trajectory.")
    print("  Fast motion → tracking LOST (expected — slow down and")
    print("  features will reinitialise).")
    print()

    # ── Main loop ───────────────────────────────────────────────────────
    while True:
        ret, frame = cap.read()
        if not ret:
            if isinstance(source, str):
                # End of video file — loop
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            break

        frame_idx += 1

        # Run VO
        pose = vo.update(frame)

        # Submit to detector (non-blocking, throttled)
        if detector is not None:
            detector.submit(frame)

        # Get latest detections
        dets = detector.detections if detector is not None else []

        # Render
        display = viz.render(
            frame=frame,
            pose=pose,
            detections=dets,
            show_feats=show_features,
            show_dets=show_detections,
            traj=vo.trajectory,
            frame_idx=frame_idx,
        )

        cv2.imshow(win_name, display)

        # Key handling
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('q'), ord('Q'), 27):   # Q or ESC
            break

        elif key in (ord('r'), ord('R')):
            vo.reset()
            print("[VO]   Pose reset to origin")

        elif key in (ord('d'), ord('D')):
            if detector is not None:
                detector.toggle()
                show_detections = detector.enabled
            else:
                show_detections = not show_detections
            print(f"[DET]  {'ON' if show_detections else 'OFF'}")

        elif key in (ord('f'), ord('F')):
            show_features = not show_features
            print(f"[FEAT] {'ON' if show_features else 'OFF'}")

        elif key in (ord('s'), ord('S')):
            snap_path = f"snap_{snap_idx:04d}.png"
            cv2.imwrite(snap_path, display)
            print(f"[SNAP] Saved {snap_path}")
            snap_idx += 1

    # ── Cleanup ──────────────────────────────────────────────────────────
    cap.release()
    cv2.destroyAllWindows()
    print("\n[DONE] Session ended.")
    if len(vo.trajectory) > 1:
        xs = [p[0] for p in vo.trajectory]
        ys = [p[1] for p in vo.trajectory]
        total_dist = sum(
            math.hypot(vo.trajectory[i][0] - vo.trajectory[i-1][0],
                       vo.trajectory[i][1] - vo.trajectory[i-1][1])
            for i in range(1, len(vo.trajectory))
        )
        print(f"  Frames processed : {frame_idx}")
        print(f"  Path length      : {total_dist:.2f} m (estimated)")
        print(f"  X range          : {min(xs):.2f} → {max(xs):.2f} m")
        print(f"  Y range          : {min(ys):.2f} → {max(ys):.2f} m")


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    main()
