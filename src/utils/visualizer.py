import cv2
import numpy as np
import math
import time
from .config import Config


# ─────────────────────────────────────────────────────────────────────────────
# VISUALISER
# ─────────────────────────────────────────────────────────────────────────────


class Visualiser:
    TH = 92
    TW = Config.TRAJ_SIZE
    TH_P = Config.TRAJ_SIZE

    def __init__(self, fw: int, fh: int):
        self._fw, self._fh = fw, fh
        lw = Config.VIZ_WIDTH - self.TW
        sc = min(lw / fw, Config.VIZ_HEIGHT / fh)
        self._dw = int(fw * sc)
        self._dh = int(fh * sc)
        self._sc = sc
        self._cw = self._dw + self.TW
        self._ch = self._dh + self.TH
        self._f = cv2.FONT_HERSHEY_SIMPLEX
        self._fps_t: list[float] = []

    def render(self, frame, pose, dets, feats, det_on, traj, idx):
        canvas = np.full((self._ch, self._cw, 3), Config.COL_BG, dtype=np.uint8)
        cam = cv2.resize(frame, (self._dw, self._dh))

        if feats and pose["keypoints"] is not None and len(pose["keypoints"]) > 0:
            for kp in pose["keypoints"]:
                px, py = int(kp.pt[0] * self._sc), int(kp.pt[1] * self._sc)
                angle = kp.angle
                r = int(kp.size / 2 * self._sc)
                if r < 2:
                    r = 3
                cv2.circle(cam, (px, py), r, Config.COL_GREEN, 1)
                if angle >= 0:
                    rad = math.radians(angle)
                    ex = int(px + r * math.cos(rad))
                    ey = int(py + r * math.sin(rad))
                    cv2.line(cam, (px, py), (ex, ey), Config.COL_GREEN, 1)

        if det_on:
            sx, sy = self._dw / self._fw, self._dh / self._fh
            for d in dets:
                x1, y1, x2, y2 = d["bbox"]
                x1, y1, x2, y2 = int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy)
                cv2.rectangle(cam, (x1, y1), (x2, y2), Config.COL_ORANGE, 2)
                lbl = f"{d['label']} {d['conf']:.0%}"
                (tw, th), _ = cv2.getTextSize(lbl, self._f, 0.5, 1)
                cv2.rectangle(
                    cam, (x1, y1 - th - 8), (x1 + tw + 6, y1), Config.COL_ORANGE, -1
                )
                cv2.putText(
                    cam,
                    lbl,
                    (x1 + 3, y1 - 4),
                    self._f,
                    0.5,
                    Config.COL_BG,
                    1,
                    cv2.LINE_AA,
                )

        sc = {
            "HEALTHY": Config.COL_GREEN,
            "STATIONARY": Config.COL_CYAN,
            "DEGRADED": Config.COL_ORANGE,
            "LOST": Config.COL_RED,
            "INITIALIZING": Config.COL_GREY,
        }.get(pose["status"], Config.COL_GREY)
        self._chip(cam, pose["status"], (10, 10), sc)
        cv2.putText(
            cam,
            f"Dpx:{pose['pixel_disp']:.1f}  M:{pose['num_matches']}",
            (10, self._dh - 12),
            self._f,
            0.4,
            Config.COL_GREY,
            1,
            cv2.LINE_AA,
        )

        canvas[: self._dh, : self._dw] = cam
        canvas[: self.TH_P, self._dw :] = self._traj_panel(traj, pose)
        self._telemetry(canvas, self._dh, pose, idx)
        return canvas

    def _traj_panel(self, traj, pose):
        p = np.full((self.TH_P, self.TW, 3), Config.COL_PANEL, dtype=np.uint8)
        cx, cy = self.TW // 2, self.TH_P // 2
        for d in range(1, 6):
            cv2.circle(p, (cx, cy), int(d * Config.TRAJ_SCALE), (40, 42, 52), 1)
        cv2.line(p, (0, cy), (self.TW, cy), (40, 42, 52), 1)
        cv2.line(p, (cx, 0), (cx, self.TH_P), (40, 42, 52), 1)
        cv2.putText(
            p, "N", (cx + 4, 14), self._f, 0.38, Config.COL_GREY, 1, cv2.LINE_AA
        )
        cv2.putText(
            p,
            "E",
            (self.TW - 16, cy - 4),
            self._f,
            0.38,
            Config.COL_GREY,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            p, "TRAJECTORY", (8, 18), self._f, 0.45, Config.COL_ACCENT, 1, cv2.LINE_AA
        )

        if len(traj) > 1:

            def to_px(tx, ty):
                return (
                    max(0, min(self.TW - 1, cx + int(tx * Config.TRAJ_SCALE))),
                    max(0, min(self.TH_P - 1, cy - int(ty * Config.TRAJ_SCALE))),
                )

            pts = [to_px(tx, ty) for tx, ty in traj]
            n = len(pts)
            for i in range(1, n):
                a = (i / n) ** 1.5
                col = tuple(int(c * a) for c in Config.COL_TRACK)
                cv2.line(p, pts[i - 1], pts[i], col, 1)
            tail = pts[max(0, n - 80) :]
            for i in range(1, len(tail)):
                cv2.line(p, tail[i - 1], tail[i], Config.COL_ACCENT, 2)

        cpx = max(4, min(self.TW - 5, cx + int(pose["x"] * Config.TRAJ_SCALE)))
        cpy = max(4, min(self.TH_P - 5, cy - int(pose["y"] * Config.TRAJ_SCALE)))
        yr = math.radians(pose["yaw_deg"])
        ax, ay = cpx + int(18 * math.sin(yr)), cpy - int(18 * math.cos(yr))
        cv2.arrowedLine(p, (cpx, cpy), (ax, ay), Config.COL_WHITE, 2, tipLength=0.3)
        cv2.circle(p, (cpx, cpy), 5, Config.COL_WHITE, -1)
        cv2.circle(p, (cpx, cpy), 7, Config.COL_ACCENT, 2)

        bx, by, sp = 8, self.TH_P - 16, Config.TRAJ_SCALE
        cv2.line(p, (bx, by), (bx + sp, by), Config.COL_GREY, 1)
        for ex in (bx, bx + sp):
            cv2.line(p, (ex, by - 3), (ex, by + 3), Config.COL_GREY, 1)
        cv2.putText(
            p,
            "1 m",
            (bx + sp // 2 - 10, by - 5),
            self._f,
            0.35,
            Config.COL_GREY,
            1,
            cv2.LINE_AA,
        )
        return p

    def _telemetry(self, canvas, y, pose, idx):
        now = time.time()
        self._fps_t.append(now)
        self._fps_t = [t for t in self._fps_t if now - t < 1.0]

        cv2.rectangle(canvas, (0, y), (self._cw, y + self.TH), Config.COL_PANEL, -1)
        cv2.line(canvas, (0, y), (self._cw, y), (50, 50, 62), 1)

        fields = [
            ("X", f"{pose['x']:+7.2f} m"),
            ("Y", f"{pose['y']:+7.2f} m"),
            ("Z", f"{pose['z']:+7.2f} m"),
            ("YAW", f"{pose['yaw_deg']:+7.1f}°"),
        ]
        cw = 130
        for i, (k, v) in enumerate(fields):
            bx = 12 + i * cw
            cv2.putText(
                canvas, k, (bx, y + 22), self._f, 0.38, Config.COL_GREY, 1, cv2.LINE_AA
            )
            cv2.putText(
                canvas, v, (bx, y + 46), self._f, 0.55, Config.COL_WHITE, 1, cv2.LINE_AA
            )

        qx = 12 + 4 * cw + 24
        cv2.putText(
            canvas,
            "TRACK QUALITY",
            (qx, y + 22),
            self._f,
            0.38,
            Config.COL_GREY,
            1,
            cv2.LINE_AA,
        )
        q = max(0.0, min(1.0, pose["tracking_quality"]))
        qc = (
            Config.COL_GREEN
            if q > 0.6
            else (Config.COL_ORANGE if q > 0.3 else Config.COL_RED)
        )
        cv2.rectangle(canvas, (qx, y + 28), (qx + 160, y + 40), (50, 50, 62), -1)
        cv2.rectangle(canvas, (qx, y + 28), (qx + int(160 * q), y + 40), qc, -1)
        cv2.putText(
            canvas,
            f"{q:.0%}  {pose['num_features']} feats",
            (qx, y + 54),
            self._f,
            0.42,
            Config.COL_WHITE,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"{pose['num_matches']} matches  Dpx:{pose['pixel_disp']:.1f}",
            (qx, y + 72),
            self._f,
            0.38,
            Config.COL_GREY,
            1,
            cv2.LINE_AA,
        )

        rx = self._cw - 110
        cv2.putText(
            canvas,
            f"{len(self._fps_t):2d} FPS",
            (rx, y + 30),
            self._f,
            0.6,
            Config.COL_ACCENT,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"#{idx:06d}",
            (rx, y + 54),
            self._f,
            0.38,
            Config.COL_GREY,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "R:reset  D:dets  F:features  S:snap  Q:quit",
            (12, y + self.TH - 8),
            self._f,
            0.35,
            Config.COL_GREY,
            1,
            cv2.LINE_AA,
        )

    @staticmethod
    def _chip(img, text, pos, color):
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        x, y = pos
        p = 6
        cv2.rectangle(img, (x, y), (x + tw + p * 2, y + th + p * 2), color, -1)
        cv2.putText(
            img,
            text,
            (x + p, y + th + p - 1),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
