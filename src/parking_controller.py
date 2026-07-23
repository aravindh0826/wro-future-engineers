"""
Parking Controller
Vision-guided parallel-parking state machine. Triggered after the 3 laps
are complete and the vehicle is back in the starting section (rule: robot
must find the parking lot and perform parallel parking, appendix A #6).

The lot is bounded by two magenta blocks (rule 13.25, 200x20x100mm).
CVPipeline._detect_parking_markers returns up to 2 of them, closest first,
as (cx, cy, w, h, distance_mm).

Timings below are open-loop (no wheel encoders in this codebase) and MUST
be tuned on the real vehicle/track before competition — treat every
TUNE_ME constant as a starting point, not a final value.
"""

import time
import logging

logger = logging.getLogger(__name__)

FRAME_WIDTH  = 320
FRAME_CENTER = FRAME_WIDTH // 2

DRIVE       = "drive"
REVERSE_OUT = "reverse_out"
REVERSE_IN  = "reverse_in"
STRAIGHTEN  = "straighten"
DONE        = "done"

# TUNE_ME — cy (px) at which both markers are considered "alongside" the car
ENTRY_CY_THRESHOLD = 170
# TUNE_ME — open-loop durations (seconds) and speeds (mm/s)
REVERSE_OUT_TIME = 1.0
REVERSE_IN_TIME  = 1.3
STRAIGHTEN_TIME  = 0.6
PARK_SPEED       = -120
STEER_LOCK       = 1.0
MARKER_SYMMETRY_TOL_PX = 15


class ParkingController:
    def __init__(self):
        self.state = DRIVE
        self._state_start = None
        self._lot_side = None   # "left" or "right" relative to frame center

    def reset(self):
        self.state = DRIVE
        self._state_start = None
        self._lot_side = None

    def update(self, detections):
        """
        Args:
            detections : full CVPipeline.process() output for this frame
        Returns:
            None                                    — not yet triggered, caller should
                                                        keep using normal wall-follow output
            {"steering": f, "speed": i, "done": bool} — controller has taken over
        """
        markers = detections.get("parking_markers", [])

        if self.state == DRIVE:
            if len(markers) >= 2:
                avg_cy = sum(m[1] for m in markers) / len(markers)
                if avg_cy >= ENTRY_CY_THRESHOLD:
                    avg_cx = sum(m[0] for m in markers) / len(markers)
                    self._lot_side = "right" if avg_cx >= FRAME_CENTER else "left"
                    self._enter(REVERSE_OUT)
                    logger.info(f"Parking triggered, lot on {self._lot_side}")
            return None

        elapsed = time.time() - self._state_start
        away_lock = STEER_LOCK if self._lot_side == "left" else -STEER_LOCK
        toward_lock = -away_lock

        if self.state == REVERSE_OUT:
            if elapsed >= REVERSE_OUT_TIME:
                self._enter(REVERSE_IN)
            return {"steering": away_lock, "speed": PARK_SPEED, "done": False}

        if self.state == REVERSE_IN:
            if elapsed >= REVERSE_IN_TIME:
                self._enter(STRAIGHTEN)
            return {"steering": toward_lock, "speed": PARK_SPEED, "done": False}

        if self.state == STRAIGHTEN:
            aligned = self._markers_symmetric(markers)
            if aligned or elapsed >= STRAIGHTEN_TIME:
                self._enter(DONE)
            return {"steering": 0.0, "speed": PARK_SPEED * 0.5, "done": False}

        # DONE
        return {"steering": 0.0, "speed": 0, "done": True}

    # ── Internal ──────────────────────────────────────────────────────────────
    def _enter(self, state):
        self.state = state
        self._state_start = time.time()
        logger.info(f"Parking state -> {state}")

    def _markers_symmetric(self, markers):
        if len(markers) < 2:
            return False
        left_x  = min(m[0] for m in markers[:2])
        right_x = max(m[0] for m in markers[:2])
        mid = (left_x + right_x) / 2
        return abs(mid - FRAME_CENTER) <= MARKER_SYMMETRY_TOL_PX
