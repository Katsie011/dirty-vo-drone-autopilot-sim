# lk_vo.py — Lucas-Kanade Visual Odometry
import cv2
import numpy as np

class LKVisualOdometry:
    """
    Minimal monocular VO using Lucas-Kanade sparse optical flow.
    
    Inputs:  camera frames (numpy arrays)
    Outputs: pose delta (dx, dy, dz_relative, dyaw) per frame
    
    This is not as robust as OpenVINS or ORB-SLAM3.
    It is good enough to prove the perception → control loop.
    It is what gets replaced with OpenVINS in Phase 5.
    """
    
    def __init__(self, camera_matrix: np.ndarray, scale_from_baro: bool = True):
        self.K = camera_matrix  
        self.scale_from_baro = scale_from_baro
        
        # LK parameters
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        self.feature_params = dict(
            maxCorners=200,
            qualityLevel=0.01,
            minDistance=10,
            blockSize=3
        )
        
        self.prev_frame = None
        self.prev_pts = None
        
        # Accumulated pose (ENU convention)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0  # overridden by baro when available
        self.yaw = 0.0
        
        self.tracking_quality = 0.0  # 0.0 (lost) to 1.0 (excellent)
    
    def update(self, frame: np.ndarray, baro_altitude_m: float = None) -> dict:
        """
        Process one frame. Returns pose estimate.
        Call at camera rate (30 Hz).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.prev_frame is None or self.prev_pts is None or len(self.prev_pts) < 20:
            # (Re)initialize: detect features
            self.prev_pts = cv2.goodFeaturesToTrack(gray, **self.feature_params)
            self.prev_frame = gray
            return self._make_pose(baro_altitude_m, quality=0.0, status="INITIALIZING")
        
        # Track features frame-to-frame
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_frame, gray, self.prev_pts, None, **self.lk_params
        )
        
        # Keep only successfully tracked points
        good_new = next_pts[status == 1]
        good_old = self.prev_pts[status == 1]
        
        self.tracking_quality = len(good_new) / max(len(self.prev_pts), 1)
        
        if len(good_new) < 8:
            # Not enough points for essential matrix — reinitialize
            self.prev_pts = None
            return self._make_pose(baro_altitude_m, quality=0.0, status="LOST")
        
        # Estimate relative camera motion via essential matrix
        E, mask = cv2.findEssentialMat(
            good_new, good_old, self.K,
            method=cv2.RANSAC, prob=0.999, threshold=1.0
        )
        
        if E is None:
            return self._make_pose(baro_altitude_m, quality=self.tracking_quality, status="DEGRADED")
        
        _, R, t, _ = cv2.recoverPose(E, good_new, good_old, self.K)
        
        # Extract yaw from rotation matrix
        yaw_delta = np.arctan2(R[1, 0], R[0, 0])
        self.yaw += yaw_delta
        
        # Translation: direction only (monocular — scale from baro or set to 1)
        scale = 1.0
        if baro_altitude_m is not None and self.scale_from_baro:
            # Simple scale from altitude change — imperfect but functional
            dz = baro_altitude_m - self.z
            t_norm = np.linalg.norm(t)
            if t_norm > 1e-6:
                scale = abs(dz) / t_norm if abs(dz) > 0.05 else 0.5
        
        # Update position (camera frame → ENU: rough approximation)
        self.x += scale * float(t[0])
        self.y += scale * float(t[1])
        
        if baro_altitude_m is not None:
            self.z = baro_altitude_m
        
        # Refresh features periodically
        if self.tracking_quality < 0.5:
            new_pts = cv2.goodFeaturesToTrack(gray, **self.feature_params)
            if new_pts is not None:
                self.prev_pts = np.vstack([good_new.reshape(-1, 1, 2), new_pts])
            else:
                self.prev_pts = good_new.reshape(-1, 1, 2)
        else:
            self.prev_pts = good_new.reshape(-1, 1, 2)
        
        self.prev_frame = gray
        
        return self._make_pose(baro_altitude_m, quality=self.tracking_quality, status="HEALTHY")
    
    def _make_pose(self, baro_alt, quality, status):
        return {
            "x": self.x,
            "y": self.y, 
            "z": baro_alt if baro_alt is not None else self.z,
            "yaw": self.yaw,
            "tracking_quality": quality,
            "status": status,
            "num_features": len(self.prev_pts) if self.prev_pts is not None else 0
        }