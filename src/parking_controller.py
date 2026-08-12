"""
Parking Controller (vision-only, SUPERSEDED by ultrasonic_parking_controller.py)
main.py now imports UltrasonicParkingController, which does the same
vision-triggered lot approach but adds rear ultrasonic sensors for the
blind-spot reverse/parallelism part (the forward camera loses the magenta
markers mid-manoeuvre -- see that file's docstring). This file is kept
only as a fallback reference: if the ultrasonic sensors aren't mounted
and wired in time for competition, swap the import in main.py back to
ParkingController from here and the car still parks (with less accurate,
open-loop-only contact/parallelism handling).

Vision-guided parallel-parking state machine. Triggered after the 3 laps
are complete and the vehicle is back in the starting section (rule: robot
must find the parking lot and perform parallel parking, appendix A #6).

The lot is bounded by two magenta blocks (rule 13.25, 200x20x100mm).
CVPipeline._detect_parking_markers returns up to 2 of them, closest first,
as (cx, cy, w, h, distance_mm).

Timings below are open-loop (no wheel encoders in this codebase) and MUST
be tuned on the real vehicle/track before competition — treat every
TUNE_ME constant as a starting point, not a final value.

Safety: any contact with the magenta parking-lot blocks zeroes the entire
parking score (rule 13.25-13.27), so reverse speed is eased and then
hard-stopped as the nearest marker gets close, using live distance from
CVPipeline rather than open-loop timing alone.
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
# TUNE_ME — open-loop durations (seconds) and speeds (mm/s), used as a
# fallback ceiling — the live distance guard below is expected to end
# REVERSE_IN earlier than this timeout in most real approaches.
REVERSE_OUT_TIME = 1.0
REVERSE_IN_TIME  = 1.3
STRAIGHTEN_TIME  = 0.6
PARK_SPEED       = -120
STEER_LOCK       = 1.0
MARKER_SYMMETRY_TOL_PX = 15

# ── Contact-avoidance guard (TUNE_ME on real hardware) ───────────────────────
# Distances are to the NEAREST detected marker (mm), from CVPipeline's
# height-based estimate. These must be tuned once the physical vehicle's
# rear overhang / camera mount offset is known.
CONTACT_EASE_DISTANCE_MM = 150   # start slowing reverse speed inside this range
CONTACT_STOP_DISTANCE_MM = 60    # force a full stop inside this range
CONTACT_MIN_SPEED_SCALE  = 0.35  # eased floor before hard stop kicks in

# ── Parallelism proxy (TUNE_ME) ──────────────────────────────────────────────
# True parallelism (wheel-to-wall distance within 2cm both ends) needs an
# IMU/gyro heading reading, which this codebase doesn't have. As a vision-only
# proxy, check that the two markers are at similar DISTANCE from the camera
# (not just similar x-position) — if the car is angled into/away from the
# lot, the near marker reads noticeably closer than the far one even when
# horizontally centered.
PARALLELISM_DIST_TOL_MM = 40


class ParkingController:
    def __init__(self):
        self.state = DRIVE
        self._state_start = None
        self._lot_side = None   # "left" or "right" relative to frame center
        self.result = None      # "full" or "partial" once DONE, else None

    def reset(self):
        self.state = DRIVE
        self._state_start = None
        self._lot_side = None
        self.result = None

    def update(self, detections):
        """
        Args:
            detections : full CVPipeline.process() output for this frame
        Returns:
            None — not yet triggered, caller should keep using normal
                   wall-follow output
            {"steering": f, "speed": i, "done": bool, "result": str|None}
                   — controller has taken over. "result" is "full",
                     "partial", or None (only meaningful once done=True).
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
            speed = self._guarded_reverse_speed(markers, PARK_SPEED)
            if elapsed >= REVERSE_OUT_TIME:
                self._enter(REVERSE_IN)
            return {"steering": away_lock, "speed": speed, "done": False, "result": None}

        if self.state == REVERSE_IN:
            speed = self._guarded_reverse_speed(markers, PARK_SPEED)
            hit_wall = speed == 0
            if elapsed >= REVERSE_IN_TIME or hit_wall:
                self._enter(STRAIGHTEN)
            return {"steering": toward_lock, "speed": speed, "done": False, "result": None}

        if self.state == STRAIGHTEN:
            speed = self._guarded_reverse_speed(markers, PARK_SPEED * 0.5)
            aligned = self._markers_symmetric(markers)
            if aligned or elapsed >= STRAIGHTEN_TIME or speed == 0:
                self._finish(markers)
            return {"steering": 0.0, "speed": speed, "done": False, "result": None}

        # DONE
        return {"steering": 0.0, "speed": 0, "done": True, "result": self.result}

    # ── Internal ──────────────────────────────────────────────────────────────
    def _enter(self, state):
        self.state = state
        self._state_start = time.time()
        logger.info(f"Parking state -> {state}")

    def _finish(self, markers):
        self.state = DONE
        self._state_start = time.time()
        centered = self._markers_symmetric(markers)
        parallel = self._markers_parallel(markers)
        self.result = "full" if (centered and parallel) else "partial"
        logger.info(f"Parking DONE — result={self.result} "
                    f"(centered={centered}, parallel_proxy={parallel})")

    def _guarded_reverse_speed(self, markers, base_speed):
        """
        Scales/stops reverse speed based on live distance to the nearest
        marker, so the vehicle eases off and stops before contact rather
        than relying only on open-loop timing. Returns 0 if inside the
        hard-stop distance.
        """
        if not markers:
            return base_speed

        nearest_dist = min(m[4] for m in markers)

        if nearest_dist <= CONTACT_STOP_DISTANCE_MM:
            logger.warning(f"Parking contact guard: nearest marker {nearest_dist:.0f}mm "
                            f"<= stop threshold {CONTACT_STOP_DISTANCE_MM}mm — stopping")
            return 0

        if nearest_dist <= CONTACT_EASE_DISTANCE_MM:
            span = CONTACT_EASE_DISTANCE_MM - CONTACT_STOP_DISTANCE_MM
            frac = (nearest_dist - CONTACT_STOP_DISTANCE_MM) / span
            scale = CONTACT_MIN_SPEED_SCALE + (1.0 - CONTACT_MIN_SPEED_SCALE) * frac
            return base_speed * scale

        return base_speed

    def _markers_symmetric(self, markers):
        """Horizontal centering proxy — position, not heading."""
        if len(markers) < 2:
            return False
        left_x  = min(m[0] for m in markers[:2])
        right_x = max(m[0] for m in markers[:2])
        mid = (left_x + right_x) / 2
        return abs(mid - FRAME_CENTER) <= MARKER_SYMMETRY_TOL_PX

    def _markers_parallel(self, markers):
        """
        Vision-only proxy for "parallel to the wall": if the two markers
        are at similar distance from the camera, the car isn't angled
        sharply into or away from the lot. Not equivalent to the rule's
        actual wheel-to-wall <=2cm criterion (that needs IMU/gyro heading),
        but catches the coarse case of a visibly skewed park.
        """
        if len(markers) < 2:
            return False
        d0, d1 = markers[0][4], markers[1][4]
        return abs(d0 - d1) <= PARALLELISM_DIST_TOL_MM
