
# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

class Config:
    # Camera / capture
    FRAME_WIDTH  = 1280
    FRAME_HEIGHT = 720
    TARGET_FPS   = 30

    # LK Optical Flow
    LK_WIN_SIZE      = (21, 21)
    LK_MAX_LEVEL     = 3
    LK_MAX_CORNERS   = 300
    LK_QUALITY       = 0.01
    LK_MIN_DISTANCE  = 10
    REINIT_THRESHOLD = 0.40   # reinit features if tracking quality drops below this
    MIN_FEATURES     = 12     # minimum tracked points before declaring LOST

    # VO scale heuristic
    # In a real system this comes from barometer or GPS altitude.
    # For the desktop demo we use a fixed assumed height (m) so the
    # trajectory is in roughly metric units rather than pixels.
    ASSUMED_HEIGHT_M = 1.5

    # Detector
    DETECTOR_MODEL    = "yolov8n.pt"   # auto-downloaded on first run
    DETECTOR_CONF     = 0.40
    DETECTOR_INTERVAL = 0.10           # run detector every N seconds
    DETECTOR_ENABLED  = True

    # Display
    VIZ_WIDTH  = 1400   # total window width
    VIZ_HEIGHT = 720    # total window height
    TRAJ_SIZE  = 500    # trajectory panel (square)
    TRAJ_SCALE = 80     # pixels per metre in trajectory view

    # Colours (BGR)
    COL_GREEN   = (80,  220, 80)
    COL_CYAN    = (200, 220, 40)
    COL_ORANGE  = (40,  160, 220)
    COL_RED     = (60,  60,  220)
    COL_WHITE   = (240, 240, 240)
    COL_GREY    = (100, 100, 100)
    COL_BG      = (18,  18,  22)
    COL_PANEL   = (28,  28,  34)
    COL_ACCENT  = (60,  180, 255)
    COL_TRACK   = (60,  120, 200)
