"""
Kalman Filter
Tracks pillar position across frames, handles brief occlusions,
smooths noisy detections.
State vector: [x, y, vx, vy]

Rotation compensation: the constant-velocity motion model (F) assumes the
pillar moves in a straight line across the image between frames. That
assumption breaks specifically when the car itself is turning (corners) --
the pillar's apparent pixel motion is dominated by the car's own yaw, not
the pillar moving. Without correction this is exactly where prediction
through a missed/occluded frame is worst, which is also where the speed
controller is already most conservative (corner_near slowdown), compounding
the problem. compensate_rotation() rotates the tracked velocity vector by
the car's yaw delta each frame (from ImuTracker.yaw_rate_dps) so a missed
frame during a turn still predicts a physically plausible pillar position
instead of extrapolating in a straight line through a curve.
"""

import math
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
        self._last_area     = None
        self._last_dist     = None

    # ── Public API ────────────────────────────────────────────────────────────
    def update(self, detection, yaw_rate_dps=0.0, dt=0.033):
        """
        Feed a new detection.
        Args:
            detection    : (cx, cy, area, dist) tuple from CVPipeline, or None
            yaw_rate_dps : ImuTracker.yaw_rate_dps for this frame -- used to
                           rotate the tracked velocity vector so blind
                           prediction through a missed/occluded frame stays
                           physically plausible during a turn instead of
                           extrapolating in a straight line. Defaults to 0.0
                           so callers without an IMU wired up behave exactly
                           as before.
            dt           : seconds since the last update() call (only used
                           for the rotation compensation math above).
        """
        if detection is None:
            self._missed_frames += 1
            if self._missed_frames > self.MAX_MISSED:
                self.initialized = False
                self._last_area = None
                self._last_dist = None
                return
            # Blind prediction through the missed frame: still advance the
            # motion model so predict() returns a moving estimate instead of
            # a frozen one, with velocity rotation-compensated for yaw.
            if self.initialized:
                self._compensate_rotation(yaw_rate_dps, dt)
                self.state = self.F @ self.state
                self.P     = self.F @ self.P @ self.F.T + self.Q
            return

        cx, cy = detection[0], detection[1]
        self._missed_frames = 0
        # area/dist have no motion model -- pass the latest measurement
        # straight through alongside the Kalman-filtered x, y.
        self._last_area = detection[2] if len(detection) > 2 else None
        self._last_dist = detection[3] if len(detection) > 3 else None

        if not self.initialized:
            self._init(cx, cy)
            return

        # ── Predict ───────────────────────────────────────────────────────
        self._compensate_rotation(yaw_rate_dps, dt)
        self.state = self.F @ self.state
        self.P     = self.F @ self.P @ self.F.T + self.Q

        # ── Update ────────────────────────────────────────────────────────
        z   = np.array([[cx], [cy]], dtype=float)
        y   = z - self.H @ self.state                   # innovation
        S   = self.H @ self.P @ self.H.T + self.R      # innovation covariance
        K   = self.P @ self.H.T @ np.linalg.inv(S)     # Kalman gain

        self.state = self.state + K @ y
        self.P     = (self.I - K @ self.H) @ self.P

    def _compensate_rotation(self, yaw_rate_dps, dt):
        """
        Rotates the tracked velocity vector (vx, vy) by the car's yaw delta
        this frame, so the constant-velocity model accounts for ego-rotation
        instead of assuming the pillar itself is moving in a straight line.

        Approximation, not exact optics: a true correction would need the
        pillar's depth and the camera's focal length to reproject correctly.
        This rotates velocity directly, which is close enough at the yaw
        rates/frame times involved here (a few degrees per frame) to keep
        predictions in the right direction through a turn -- TUNE_ME: sign
        and magnitude should be checked against real turn footage once the
        car exists; flip the sign of delta_theta below if compensation
        visibly predicts the wrong direction on real corner footage.
        """
        if not self.initialized or yaw_rate_dps == 0.0:
            return
        delta_theta = math.radians(yaw_rate_dps * dt)
        cos_t, sin_t = math.cos(delta_theta), math.sin(delta_theta)
        vx, vy = float(self.state[2][0]), float(self.state[3][0])
        self.state[2][0] = vx * cos_t - vy * sin_t
        self.state[3][0] = vx * sin_t + vy * cos_t

    def predict(self):
        """
        Returns best estimate of pillar position as (cx, cy, area, dist),
        matching CVPipeline's raw detection tuple shape so downstream code
        (e.g. distance-based pillar avoidance) works the same whether the
        position came from a live detection, a Kalman prediction across a
        missed frame, or PillarMemory recall. area/dist are the latest
        known measurement (not motion-predicted) and may be None if a
        pillar was never actually detected (e.g. right after init).
        Returns None if nothing is currently tracked.
        """
        if not self.initialized:
            return None
        x = float(self.state[0][0])
        y = float(self.state[1][0])
        return (x, y, self._last_area, self._last_dist)

    def reset(self):
        self.initialized    = False
        self._missed_frames = 0
        self._last_area     = None
        self._last_dist     = None

    # ── Internal ──────────────────────────────────────────────────────────────
    def _init(self, cx, cy):
        self.state = np.array([[cx], [cy], [0.0], [0.0]], dtype=float)
        self.P     = np.eye(4) * 1.0
        self.initialized = True
        logger.debug(f"Kalman initialised at ({cx}, {cy})")
