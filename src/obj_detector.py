import threading
import queue
from config import Config
import time
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# OBJECT DETECTOR  (background thread)
# ─────────────────────────────────────────────────────────────────────────────

class DetectorThread:
    """
    Runs YOLOv8-nano in a background thread so it never blocks the VO loop.
    Caller reads `self.detections` for the latest results.
    """

    def __init__(self):
        self._frame_queue:     queue.Queue = queue.Queue(maxsize=1)
        self._result_lock:     threading.Lock = threading.Lock()
        self._latest_dets:     list = []
        self._model            = None
        self._enabled          = Config.DETECTOR_ENABLED
        self._last_run_time:   float = 0.0
        self._thread:          threading.Thread = threading.Thread(
                                    target=self._loop, daemon=True
                               )
        self._thread.start()

    def submit(self, frame: np.ndarray):
        """Non-blocking — drops frames if detector is busy."""
        if not self._enabled:
            return
        now = time.time()
        if now - self._last_run_time < Config.DETECTOR_INTERVAL:
            return
        try:
            # Replace old pending frame rather than queuing
            self._frame_queue.put_nowait(frame.copy())
        except queue.Full:
            pass

    @property
    def detections(self) -> list:
        with self._result_lock:
            return list(self._latest_dets)

    def toggle(self):
        self._enabled = not self._enabled
        if not self._enabled:
            with self._result_lock:
                self._latest_dets = []

    @property
    def enabled(self) -> bool:
        return self._enabled

    # ------------------------------------------------------------------
    def _loop(self):
        """Worker thread: load model once, then process frames."""
        try:
            from ultralytics import YOLO
            self._model = YOLO(Config.DETECTOR_MODEL, verbose=False)
            # Warm-up pass
            dummy = np.zeros((64, 64, 3), dtype=np.uint8)
            self._model(dummy, verbose=False)
        except Exception as e:
            print(f"[DETECTOR] Failed to load model: {e}")
            self._enabled = False
            return

        while True:
            try:
                frame = self._frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            self._last_run_time = time.time()
            try:
                results = self._model(frame, conf=Config.DETECTOR_CONF,
                                      verbose=False)[0]
                dets = []
                if results.boxes is not None:
                    for box in results.boxes:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        conf  = float(box.conf[0])
                        cls   = int(box.cls[0])
                        label = results.names[cls]
                        dets.append({
                            "bbox":  (int(x1), int(y1), int(x2), int(y2)),
                            "label": label,
                            "conf":  conf,
                        })
                with self._result_lock:
                    self._latest_dets = dets
            except Exception:
                pass
