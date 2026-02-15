# AeroPerception — VO Demo Quick Start

## Install

```bash
conda activate aeroperception
pip install opencv-python numpy ultralytics
```

## Run

```bash
# Built-in MacBook webcam
python main.py

# External webcam
python main.py --source 1

# Video file (useful for repeatable testing)
python main.py --source my_video.mp4

# Skip detector for faster startup
python main.py --no-detector
```

## Controls

| Key | Action |
|-----|--------|
| `R` | Reset pose to origin |
| `D` | Toggle object detections |
| `F` | Toggle feature point overlay |
| `S` | Save snapshot |
| `Q` / `ESC` | Quit |

## What you're looking at

```
┌──────────────────────────────┬─────────────────┐
│  Camera feed                 │  Trajectory map │
│  Green dots = tracked feats  │  Top-down view  │
│  Orange boxes = detections   │  Arrow = heading│
├──────────────────────────────┴─────────────────┤
│  X  Y  Z  YAW  |  Track quality bar  |  FPS    │
└────────────────────────────────────────────────┘
```

## Tips for best results

- **Move slowly** — fast motion causes tracking loss (expected for monocular LK flow)
- **Textured surfaces** work much better than blank walls (the algorithm needs features to track)
- After tracking loss, hold still for ~1 second — features will reinitialise automatically
- Press `R` to zero the trajectory whenever you want a fresh start

## What this is NOT (yet)

- Not metric-accurate (scale is estimated from an assumed camera height of 1.5 m)
- Not loop-closing (trajectory drifts over long distances — expected for LK VO)
- Not connected to a drone or ROS 2 (that's Phase 3)

This is the perception loop in isolation: the piece that becomes a
ROS 2 node in the next phase.
