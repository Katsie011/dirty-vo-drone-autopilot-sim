#!/usr/bin/env python3
"""
AeroPerception — Visual Odometry Demo  v3 (ORB + Essential Matrix)
====================================================================
Run with:
    python main.py                   # default webcam
    python main.py --source 1        # external webcam
    python main.py --source video.mp4

Controls:
    R — reset pose  |  D — detections  |  F — features
    S — snapshot    |  Q / ESC — quit

Tuning flags:
    --scale N       movement scale (default 0.08)
    --min-disp N    stillness gate in pixels (default 1.2)
    --no-detector   skip YOLOv8

v3 architecture (feature-based):
  REPLACED: Lucas-Kanade optical flow tracking
  WITH:     ORB feature detection + descriptor matching

  WHY:
    - Rotation invariant (LK breaks on fast yaw)
    - Re-localization after tracking loss (descriptors can be re-matched)
    - Cleaner correspondences → better Essential Matrix estimation
    - Same RANSAC outlier rejection, now on descriptor matches

  KEPT:
    - Essential Matrix decomposition → R, t
    - Median pixel displacement stillness gate
    - EMA smoothing on display pose
    - All visualization and telemetry

  ALGORITHM:
    Frame N:   detect ORB keypoints + compute 256-bit descriptors
    Frame N+1: detect ORB keypoints + compute descriptors
    Match:     Brute-force Hamming distance (binary descriptors)
    Filter:    Lowe's ratio test (reject ambiguous matches)
    Gate:      Median pixel displacement < threshold → stationary
    Solve:     cv2.findEssentialMat with RANSAC on matched keypoints
    Decompose: cv2.recoverPose → R, t
    Integrate: Update pose in world frame

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

from utils.config import Config
from utils.visualizer import Visualiser
from utils.orb_vo import VisualOdometry
from utils.obj_detector import DetectorThread


warnings.filterwarnings("ignore")


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
    ap = argparse.ArgumentParser(description="AeroPerception VO Demo v3 (ORB)")
    ap.add_argument("--source", default="0")
    ap.add_argument("--no-detector", action="store_true")
    ap.add_argument("--width", type=int, default=Config.FRAME_WIDTH)
    ap.add_argument("--height", type=int, default=Config.FRAME_HEIGHT)
    ap.add_argument("--scale", type=float, default=None)
    ap.add_argument("--min-disp", type=float, default=None)
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

    print(f"[VO]   ORB feature-based")
    print(f"       {Config.ORB_N_FEATURES} features, ratio={Config.MATCH_RATIO_THRESH}")
    print(f"       stillness={Config.MIN_PIXEL_DISP}px  scale={Config.SCALE_FACTOR}")

    det = DetectorThread() if Config.DETECTOR_ENABLED else None
    if det:
        print("[DET]  YOLOv8-nano loading…")

    win = "AeroPerception — VO Demo v3 (ORB)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, viz._cw, viz._ch)

    sh = sd = True
    fi = si = 0

    print("\n  R:reset  D:dets  F:features  S:snap  Q:quit\n")

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
        print(f"  Frames: {fi}  |  Path: {dist:.2f} m")
        print(f"  X: {min(xs):.2f}→{max(xs):.2f}  Y: {min(ys):.2f}→{max(ys):.2f}")


if __name__ == "__main__":
    main()
