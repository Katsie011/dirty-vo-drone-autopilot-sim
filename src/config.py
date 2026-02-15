# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────


class Config:
    # Camera
    FRAME_WIDTH = 1280
    FRAME_HEIGHT = 720
    TARGET_FPS = 30

    # Pre-processing: small Gaussian blur reduces pixel noise
    BLUR_KERNEL = (3, 3)

    # LK Optical Flow
    LK_WIN_SIZE = (21, 21)
    LK_MAX_LEVEL = 3
    LK_MAX_CORNERS = 250
    LK_QUALITY = 0.02  # raised from 0.01 → fewer, stronger features
    LK_MIN_DISTANCE = 12
    REINIT_THRESHOLD = 0.45
    MIN_FEATURES = 15

    # Bidirectional LK check: reject points where |fwd - bwd| > N pixels
    FB_CHECK_THRESHOLD = 1.0

    # ── PRIMARY JITTER FIX ──────────────────────────────────────────
    # If the median pixel displacement of tracked points is below this,
    # treat the frame as stationary and skip the pose update entirely.
    MIN_PIXEL_DISP = 0.8  # pixels
    MIN_INLIER_RATIO = 0.55  # reject frames with too few RANSAC inliers
    # ────────────────────────────────────────────────────────────────

    # Scale: maps pixel motion to metres. Tune with --scale flag.
    ASSUMED_HEIGHT_M = 1.5
    SCALE_FACTOR = 0.05

    # EMA on *displayed* pose only — smooths rendering without affecting
    # the integrated trajectory. 1.0 = instant (no smooth), 0.0 = frozen.
    POSE_EMA_ALPHA = 0.75

    # Detector
    DETECTOR_MODEL = "yolov8n.pt"
    DETECTOR_CONF = 0.40
    DETECTOR_INTERVAL = 0.12
    DETECTOR_ENABLED = True

    # Display
    VIZ_WIDTH = 1400
    VIZ_HEIGHT = 720
    TRAJ_SIZE = 500
    TRAJ_SCALE = 80  # pixels per metre in trajectory view

    # Colours (BGR)
    COL_GREEN = (80, 220, 80)
    COL_CYAN = (200, 220, 40)
    COL_ORANGE = (40, 160, 220)
    COL_RED = (60, 60, 220)
    COL_WHITE = (240, 240, 240)
    COL_GREY = (100, 100, 100)
    COL_BG = (18, 18, 22)
    COL_PANEL = (28, 28, 34)
    COL_ACCENT = (60, 180, 255)
    COL_TRACK = (60, 120, 200)
