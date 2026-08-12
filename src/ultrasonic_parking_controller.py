"""
Ultrasonic-Guided Parking Controller
Vision-triggered, ultrasonic-guided parallel-parking state machine.
Triggered after the 3 laps are complete and the vehicle is back in the
starting section (Obstacle Challenge, "Parking in the parking lot", rules
13.25-13.27 + parking section on pages 41-42).

WHY A SEPARATE FILE FROM parking_controller.py:
parking_controller.py is vision-only: it estimates distance-to-marker and
parallelism from the magenta-marker size/position in the forward camera
frame. That works while the markers are still in frame, but once the
vehicle is mid-reverse and turning into the lot, the markers routinely
leave the forward camera's field of view -- the camera cannot see behind
the car. That blind spot is exactly the part of the manoeuvre where
contact with the parking-lot limitations must be avoided (touching them
zeroes the parking score, rules 13.25-13.27) and where final parallelism
is judged.

This controller keeps vision for what it's good at (finding the lot,
picking a side, checking the car ends up centred over the lot -- Rule
9.11 "Parking in the parking lot") and switches to two rear ultrasonic
sensors (Rule 11.11: any sensor, any number, no restriction) for the
blind-spot part:
  - Contact avoidance while reversing (distance to whatever is directly
    behind each rear corner -- lot limitation or outer wall).
  - Parallelism, measured the same way the rules measure it: distance
    from the wheels on one side to the wall, difference <= 2cm
    ("Parking in the parking lot", pages 41-42) -- not a vision proxy.

Steering uses the single Ackermann front actuator only (Rule 11.3); the
rear axle is drive-only (single motor, Rules 11.3/11.5). Full lock is
used for the swing-in, matching how a single-steered-axle vehicle must
parallel-park (front wheels determine the arc; the rear axle cannot
steer independently, so there is no other way to swing the rear end
sideways).

FALLBACK: motors.read_ultrasonic() returns None until the sensors are
wired to the Arduino and confirmed working (this codebase has not been
run on the finished chassis yet). Every state below degrades to the same
open-loop-timing behaviour as parking_controller.py when ultrasonic data
is unavailable or stale, so the car does not stall mid-manoeuvre waiting
for a sensor that isn't there. TUNE_ME constants below are starting
points, not final values -- verify on the real vehicle before relying on
them in competition.
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
ALIGN       = "align"
DONE        = "done"

# TUNE_ME — cy (px) at which both markers are considered "alongside" the car
ENTRY_CY_THRESHOLD = 170

# TUNE_ME — open-loop durations (seconds), used both as the behaviour when
# ultrasonic is unavailable AND as a fallback ceiling when it is (so a
# stuck/misread sensor can't wedge the state machine forever).
REVERSE_OUT_TIME = 1.0
REVERSE_IN_TIME  = 1.5
STRAIGHTEN_TIME  = 0.8
ALIGN_TIME       = 1.5
PARK_SPEED       = -120
ALIGN_SPEED      = -70
STEER_LOCK       = 1.0
ALIGN_STEER_NUDGE = 0.35
MARKER_SYMMETRY_TOL_PX = 15

# ── Contact-avoidance guard (TUNE_ME on real hardware) ───────────────────────
# Distances are to the nearer of the two rear ultrasonic sensors (mm).
# Mounting position/angle of the sensors at the rear corners changes these
# numbers directly -- must be re-tuned once physically mounted.
CONTACT_EASE_DISTANCE_MM = 150   # start slowing reverse speed inside this range
CONTACT_STOP_DISTANCE_MM = 60    # force a full stop inside this range
CONTACT_MIN_SPEED_SCALE  = 0.35  # eased floor before hard stop kicks in

# ── Parallelism (rule-accurate, not a proxy) ─────────────────────────────────
# Rule text: "parked parallel" if wheel-to-wall distance on one side does
# not differ from the other by more than 2cm. Rear ultrasonic sensors
# mounted near each rear wheel measure exactly this, so use the rule's own
# 20mm tolerance rather than an arbitrary vision-based number.
PARALLEL_TOLERANCE_MM = 20
# Max ultrasonic reading (mm) trusted as "seeing the wall" -- beyond this,
# treat the sensor as reading open space / no wall in range yet.
ULTRA_MAX_TRUST_MM = 800
# How many discrete ALIGN correction pulses to attempt before giving up and
# scoring whatever alignment currently exists as "partial". Each pulse is
# ALIGN_PULSE_TIME seconds of steer+reverse, not a per-frame nudge -- the
# car needs the wheels to actually move between ultrasonic re-checks.
ALIGN_MAX_ATTEMPTS = 4
ALIGN_PULSE_TIME = 0.35

# ── Vision-only fallback (mirrors parking_controller.py) ────────────────────
FALLBACK_PARALLELISM_DIST_TOL_MM = 40


class UltrasonicParkingController:
    def __init__(self, motors):
        """
        Args:
            motors: the active MotorController instance (Arduino, GPIO, or
                    mock). Must expose read_ultrasonic(max_age_s) -> 
                    (left_mm, right_mm) | None -- see arduino_motor_controller.py.
        """
        self._motors = motors
        self.state = DRIVE
        self._state_start = None
        self._lot_side = None    # "left" or "right" relative to frame center
        self._align_attempts = 0
        self._align_pulse_start = None
        self._align_pulse_cmd = (0.0, 0)   # (steering, speed) held for the current pulse
        self.result = None       # "full" or "partial" once DONE, else None

    def reset(self):
        self.state = DRIVE
        self._state_start = None
        self._lot_side = None
        self._align_attempts = 0
        self._align_pulse_start = None
        self._align_pulse_cmd = (0.0, 0)
        self.result = None

    def update(self, detections):
        """
        Args:
            detections : full CVPipeline.process() output for this frame
        Returns:
            None — not yet triggered, caller should keep using normal
                   wall-follow output
            {"steering": f, "speed": i, "done": bool, "result": str|None}
        """
        markers = detections.get("parking_markers", [])
        ultra = self._motors.read_ultrasonic()

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
            speed = self._guarded_reverse_speed(ultra, markers, PARK_SPEED)
            if elapsed >= REVERSE_OUT_TIME:
                self._enter(REVERSE_IN)
            return {"steering": away_lock, "speed": speed, "done": False, "result": None}

        if self.state == REVERSE_IN:
            speed = self._guarded_reverse_speed(ultra, markers, PARK_SPEED)
            hit_wall = speed == 0
            if elapsed >= REVERSE_IN_TIME or hit_wall:
                self._enter(STRAIGHTEN)
            return {"steering": toward_lock, "speed": speed, "done": False, "result": None}

        if self.state == STRAIGHTEN:
            speed = self._guarded_reverse_speed(ultra, markers, PARK_SPEED * 0.5)
            parallel_known = self._parallel_known(ultra)
            aligned = parallel_known and self._is_parallel(ultra)
            if aligned or elapsed >= STRAIGHTEN_TIME or speed == 0:
                self._enter(ALIGN)
            return {"steering": 0.0, "speed": speed, "done": False, "result": None}

        if self.state == ALIGN:
            # Closed-loop fine correction using ultrasonic parallelism --
            # only meaningful path this state can take, since vision can't
            # see the wall the car is now backed up against. If ultrasonic
            # is unavailable, skip straight to finishing (nothing to
            # correct against).
            if not self._parallel_known(ultra) or elapsed >= ALIGN_TIME:
                self._finish(ultra, markers)
                return {"steering": 0.0, "speed": 0, "done": False, "result": None}

            if self._is_parallel(ultra):
                self._finish(ultra, markers)
                return {"steering": 0.0, "speed": 0, "done": False, "result": None}

            # Mid-pulse: hold the current pulse's command until it completes
            # rather than re-deciding every frame, so each correction is a
            # discrete, measurable movement.
            if self._align_pulse_start is not None:
                pulse_elapsed = time.time() - self._align_pulse_start
                if pulse_elapsed < ALIGN_PULSE_TIME:
                    steer, speed = self._align_pulse_cmd
                    return {"steering": steer, "speed": speed, "done": False, "result": None}
                self._align_pulse_start = None   # pulse finished, fall through to start next

            if self._align_attempts >= ALIGN_MAX_ATTEMPTS:
                self._finish(ultra, markers)
                return {"steering": 0.0, "speed": 0, "done": False, "result": None}

            left_mm, right_mm = ultra
            # Positive diff => left corner farther from wall than right =>
            # rear-left is proud of the wall => nudge steer so this pulse's
            # short reverse pulls the left corner in (toward the wall).
            diff = left_mm - right_mm
            nudge = ALIGN_STEER_NUDGE if diff > 0 else -ALIGN_STEER_NUDGE
            speed = self._guarded_reverse_speed(ultra, None, ALIGN_SPEED)
            self._align_attempts += 1
            self._align_pulse_start = time.time()
            self._align_pulse_cmd = (nudge, speed)
            return {"steering": nudge, "speed": speed, "done": False, "result": None}

        # DONE
        return {"steering": 0.0, "speed": 0, "done": True, "result": self.result}

    # ── Internal ──────────────────────────────────────────────────────────────
    def _enter(self, state):
        self.state = state
        self._state_start = time.time()
        if state == ALIGN:
            self._align_attempts = 0
            self._align_pulse_start = None
        logger.info(f"Parking state -> {state}")

    def _finish(self, ultra, markers):
        self.state = DONE
        self._state_start = time.time()
        centered = self._markers_symmetric(markers)
        if self._parallel_known(ultra):
            parallel = self._is_parallel(ultra)
            source = "ultrasonic"
        else:
            parallel = self._markers_parallel_fallback(markers)
            source = "vision-fallback"
        self.result = "full" if (centered and parallel) else "partial"
        logger.info(f"Parking DONE — result={self.result} "
                    f"(centered={centered}, parallel={parallel}, source={source})")

    def _guarded_reverse_speed(self, ultra, markers, base_speed):
        """
        Scales/stops reverse speed based on live distance to the nearer
        rear ultrasonic sensor, so the vehicle eases off and hard-stops
        before contact instead of relying on open-loop timing alone.
        Falls back to the vision-based marker distance (parking_controller.py's
        original approach) only if ultrasonic is unavailable, and to
        unguarded base_speed if neither sensor has data (e.g. very first
        frames, or PC/mock testing).
        """
        if ultra is not None:
            valid = [d for d in ultra if d >= 0]
            if valid:
                nearest = min(valid)
                return self._scale_for_distance(nearest, base_speed, "ultrasonic")

        if markers:
            nearest = min(m[4] for m in markers)
            return self._scale_for_distance(nearest, base_speed, "vision-fallback")

        return base_speed

    def _scale_for_distance(self, nearest_mm, base_speed, source):
        if nearest_mm <= CONTACT_STOP_DISTANCE_MM:
            logger.warning(f"Parking contact guard ({source}): nearest {nearest_mm:.0f}mm "
                            f"<= stop threshold {CONTACT_STOP_DISTANCE_MM}mm — stopping")
            return 0
        if nearest_mm <= CONTACT_EASE_DISTANCE_MM:
            span = CONTACT_EASE_DISTANCE_MM - CONTACT_STOP_DISTANCE_MM
            frac = (nearest_mm - CONTACT_STOP_DISTANCE_MM) / span
            scale = CONTACT_MIN_SPEED_SCALE + (1.0 - CONTACT_MIN_SPEED_SCALE) * frac
            return base_speed * scale
        return base_speed

    def _parallel_known(self, ultra):
        if ultra is None:
            return False
        left_mm, right_mm = ultra
        if left_mm < 0 or right_mm < 0:
            return False
        if left_mm > ULTRA_MAX_TRUST_MM or right_mm > ULTRA_MAX_TRUST_MM:
            return False
        return True

    def _is_parallel(self, ultra):
        left_mm, right_mm = ultra
        return abs(left_mm - right_mm) <= PARALLEL_TOLERANCE_MM

    def _markers_symmetric(self, markers):
        """Horizontal centering proxy — position, not heading. Vision is
        still the right tool for this: it's about being centred over the
        lot lengthwise, which the rear ultrasonic sensors can't see."""
        if len(markers) < 2:
            return False
        left_x  = min(m[0] for m in markers[:2])
        right_x = max(m[0] for m in markers[:2])
        mid = (left_x + right_x) / 2
        return abs(mid - FRAME_CENTER) <= MARKER_SYMMETRY_TOL_PX

    def _markers_parallel_fallback(self, markers):
        """Same vision-distance proxy as parking_controller.py, used only
        when ultrasonic never became available during this parking run."""
        if len(markers) < 2:
            return False
        d0, d1 = markers[0][4], markers[1][4]
        return abs(d0 - d1) <= FALLBACK_PARALLELISM_DIST_TOL_MM
