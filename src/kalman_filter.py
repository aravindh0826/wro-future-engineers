"""
Kalman Filter
Tracks pillar position across frames, handles brief occlusions,
smooths noisy detections.
State vector: [x, y, vx, vy]
"""

import numpy as np
import logging

logger = logging.getLogger(__name__)


class KalmanFilter:
    def __init__(self, process_noise=1e-2, measurement_noise=1e-1):
        """
        Args:
            process_noise     : how much we trust motion model (lower = smoother)
            measurement_noise : how much we trust camera detections (lower = trust camera more)
        """
        # State: [x, y, vx, vy]
        self.state     = None   # (4,1) array
        self.P         = None   # covariance matrix (4,4)
        self.initialized = False

        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=float)

        # Measurement matrix (we only observe x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # Process noise covariance
        self.Q = np.eye(4) * process_noise

        # Measurement noise covariance
        self.R = np.eye(2) * measurement_noise

        # Identity
        self.I = np.eye(4)

        self._missed_frames = 0
        self.MAX_MISSED     = 10   # forget after 10 missed frames

    # ── Public API ────────────────────────────────────────────────────────────
    def update(self, detection):
        """
        Feed a new detection.
        Args:
            detection: (cx, cy, area, dist) tuple from CVPipeline
        """
        if detection is None:
            self._missed_frames += 1
            if self._missed_frames > self.MAX_MISSED:
                self.initialized = False
            return

        cx, cy = detection[0], detection[1]
        self._missed_frames = 0

        if not self.initialized:
            self._init(cx, cy)
            return

        # ── Predict ───────────────────────────────────────────────────────
        self.state = self.F @ self.state
        self.P     = self.F @ self.P @ self.F.T + self.Q

        # ── Update ────────────────────────────────────────────────────────
        z   = np.array([[cx], [cy]], dtype=float)
        y   = z - self.H @ self.state                   # innovation
        S   = self.H @ self.P @ self.H.T + self.R      # innovation covariance
        K   = self.P @ self.H.T @ np.linalg.inv(S)     # Kalman gain

        self.state = self.state + K @ y
        self.P     = (self.I - K @ self.H) @ self.P

    def predict(self):
        """
        Returns best estimate of pillar position (cx, cy) or None.
        """
        if not self.initialized:
            return None
        x = float(self.state[0][0])
        y = float(self.state[1][0])
        return (x, y)

    def reset(self):
        self.initialized    = False
        self._missed_frames = 0

    # ── Internal ──────────────────────────────────────────────────────────────
    def _init(self, cx, cy):
        self.state = np.array([[cx], [cy], [0.0], [0.0]], dtype=float)
        self.P     = np.eye(4) * 1.0
        self.initialized = True
        logger.debug(f"Kalman initialised at ({cx}, {cy})")
