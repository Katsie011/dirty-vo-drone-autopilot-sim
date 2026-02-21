import threading
import queue
from utils.config import Config
import time
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# DETECTOR
# ─────────────────────────────────────────────────────────────────────────────


class DetectorThread:
    def __init__(self):
        self._q = queue.Queue(maxsize=1)
        self._lock = threading.Lock()
        self._dets: list = []
        self._on = Config.DETECTOR_ENABLED
        self._t_last = 0.0
        threading.Thread(target=self._loop, daemon=True).start()

    def submit(self, frame: np.ndarray):
        if not self._on or time.time() - self._t_last < Config.DETECTOR_INTERVAL:
            return
        try:
            self._q.put_nowait(frame.copy())
        except queue.Full:
            pass

    @property
    def detections(self) -> list:
        with self._lock:
            return list(self._dets)

    def toggle(self):
        self._on = not self._on
        if not self._on:
            with self._lock:
                self._dets = []

    @property
    def enabled(self) -> bool:
        return self._on

    def _loop(self):
        try:
            from ultralytics import YOLO

            model = YOLO(Config.DETECTOR_MODEL, verbose=False)
            model(np.zeros((64, 64, 3), dtype=np.uint8), verbose=False)
        except Exception as e:
            print(f"[DET] Load failed: {e}")
            self._on = False
            return
        while True:
            try:
                frame = self._q.get(timeout=1.0)
            except queue.Empty:
                continue
            self._t_last = time.time()
            try:
                res = model(frame, conf=Config.DETECTOR_CONF, verbose=False)[0]
                dets = []
                if res.boxes is not None:
                    for b in res.boxes:
                        x1, y1, x2, y2 = b.xyxy[0].tolist()
                        dets.append(
                            {
                                "bbox": (int(x1), int(y1), int(x2), int(y2)),
                                "label": res.names[int(b.cls[0])],
                                "conf": float(b.conf[0]),
                            }
                        )
                with self._lock:
                    self._dets = dets
            except Exception:
                pass
