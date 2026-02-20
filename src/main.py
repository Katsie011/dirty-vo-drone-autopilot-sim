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
from utils.obj_detector import DetectorThread
from visualizer import Visualiser
from utils import lk_vo


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
    vo = lk_vo.VisualOdometry(K)
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
