"""
Wall Follower
Computes the steering error needed to keep the car
centered between the left and right walls.
Works from wall detection data provided by CVPipeline.

WRO 2025 corridor widths (rule 9.19):
  Wide section  : 1000 mm
  Narrow section:  600 mm
The follower detects which corridor it is in from the
measured pixel width and normalises accordingly so the
same PID gains work in both sections.
"""

import logging

logger = logging.getLogger(__name__)

FRAME_WIDTH  = 320
FRAME_CENTER = FRAME_WIDTH // 2

# ── Corridor width calibration ────────────────────────────────────────────────
# Pixel widths for each real corridor at 320 px frame width.
# Tune these against real camera footage.
WIDE_CORRIDOR_PX   = 240   # ~1000 mm corridor in pixels
NARROW_CORRIDOR_PX = 144   # ~600 mm corridor in pixels  (600/1000 * 240)

# If measured width is within this many px of a known size, lock to that size.
CORRIDOR_SNAP_TOLERANCE = 40   # px

# Minimum believable corridor width — below this = bad detection
MIN_VALID_CORRIDOR_PX = 30


class WallFollower:
    def __init__(self):
        self._last_error      = 0.0
        self._corridor_px     = WIDE_CORRIDOR_PX   # start assuming wide
        self._narrow_count    = 0                  # consecutive narrow readings
        self._wide_count      = 0
        logger.info("WallFollower initialised (variable corridor support)")

    # ── Public API ────────────────────────────────────────────────────────────
    def get_error(self, walls):
        """
        Compute steering error from wall detections.
        Positive error = car is too far left  → steer right
        Negative error = car is too far right → steer left

        Handles both 1000 mm (wide) and 600 mm (narrow) WRO corridors by
        dynamically tracking which corridor the car is currently in and
        normalising the center error against that corridor's half-width.

        Args:
            walls : dict with keys left, right, center_error from CVPipeline
        Returns:
            float : normalised error in [-1, 1]
        """
        if not walls:
            return self._last_error * 0.8   # decay last known error

        left  = walls.get("left",  0)
        right = walls.get("right", FRAME_WIDTH)

        corridor_px = right - left

        # ── Sanity check ──────────────────────────────────────────────────
        if corridor_px < MIN_VALID_CORRIDOR_PX:
            logger.warning(f"Suspicious corridor width: {corridor_px}px — using last error")
            return self._last_error * 0.5

        # ── Corridor width tracking (hysteresis to avoid flapping) ────────
        self._update_corridor_estimate(corridor_px)

        # ── Compute center error ──────────────────────────────────────────
        # Use the pipeline's pre-computed center error if available,
        # otherwise compute from raw left/right.
        center_error = walls.get("center_error", FRAME_CENTER - ((left + right) // 2))

        # Normalise against HALF the current corridor width so the output
        # range stays [-1, 1] regardless of whether we're in a wide or
        # narrow section.
        half_corridor = self._corridor_px / 2.0
        normalised = center_error / half_corridor
        normalised = max(-1.0, min(1.0, normalised))

        self._last_error = normalised
        return normalised

    @property
    def corridor_type(self):
        """Returns 'wide' (1000 mm) or 'narrow' (600 mm)."""
        if abs(self._corridor_px - NARROW_CORRIDOR_PX) < CORRIDOR_SNAP_TOLERANCE:
            return "narrow"
        return "wide"

    def reset(self):
        self._last_error   = 0.0
        self._corridor_px  = WIDE_CORRIDOR_PX
        self._narrow_count = 0
        self._wide_count   = 0

    # ── Internal ──────────────────────────────────────────────────────────────
    def _update_corridor_estimate(self, measured_px):
        """
        Hysteresis filter: require 3 consecutive readings near a known
        corridor width before switching, to avoid noise-driven flapping.
        """
        near_narrow = abs(measured_px - NARROW_CORRIDOR_PX) < CORRIDOR_SNAP_TOLERANCE
        near_wide   = abs(measured_px - WIDE_CORRIDOR_PX)   < CORRIDOR_SNAP_TOLERANCE

        if near_narrow:
            self._narrow_count += 1
            self._wide_count    = 0
        elif near_wide:
            self._wide_count   += 1
            self._narrow_count  = 0
        else:
            # Ambiguous — use raw measurement, reset counters
            self._corridor_px  = measured_px
            self._narrow_count = 0
            self._wide_count   = 0
            return

        HYSTERESIS = 3
        if self._narrow_count >= HYSTERESIS:
            if self._corridor_px != NARROW_CORRIDOR_PX:
                logger.info("Corridor: wide → narrow (600 mm)")
            self._corridor_px = NARROW_CORRIDOR_PX

        elif self._wide_count >= HYSTERESIS:
            if self._corridor_px != WIDE_CORRIDOR_PX:
                logger.info("Corridor: narrow → wide (1000 mm)")
            self._corridor_px = WIDE_CORRIDOR_PX