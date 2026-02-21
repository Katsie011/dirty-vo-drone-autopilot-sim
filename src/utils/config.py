# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────


class Config:
    # Camera
    FRAME_WIDTH = 1280
    FRAME_HEIGHT = 720
    TARGET_FPS = 30

    # Pre-processing
    BLUR_KERNEL = (3, 3)

    # ORB Feature Detector
    ORB_N_FEATURES = 800
    ORB_SCALE_FACTOR = 1.2
    ORB_N_LEVELS = 8
    ORB_EDGE_THRESH = 15
    ORB_FAST_THRESH = 20

    # Descriptor Matching
    MATCH_RATIO_THRESH = 0.75
    MIN_MATCHES = 20

    # Motion gate
    MIN_PIXEL_DISP = 1.2
    MIN_INLIER_RATIO = 0.55

    # Scale
    ASSUMED_HEIGHT_M = 1.5
    SCALE_FACTOR = 0.08

    # EMA smoothing
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
    TRAJ_SCALE = 80

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
