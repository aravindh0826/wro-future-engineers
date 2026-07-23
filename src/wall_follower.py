"""
Wall Follower
Computes the steering error needed to keep the car centered between walls.

Corridor width depends on challenge type (WRO 2026 rules, Section 8):
  Open Challenge     : 1000 mm or 600 mm, varies per section
  Obstacle Challenge : always 1000 mm (+/-10 mm) — corridor never narrows
In "obstacle" mode the corridor is locked wide; narrow-corridor snapping
only runs in "open" mode.

Corridor pixel widths are loaded from config/corridor_calibration.json if
present (see corridor_calibrator.py) — the hardcoded defaults below are
geometric guesses only, same as focal_length_px was before calibration.
"""

import json
import os
import logging

logger = logging.getLogger(__name__)

_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CORRIDOR_CONFIG_PATH = os.path.join(_BASE_DIR, "..", "config", "corridor_calibration.json")

FRAME_WIDTH  = 320
FRAME_CENTER = FRAME_WIDTH // 2

# ── Corridor width defaults (overridden by config/corridor_calibration.json) ──
DEFAULT_WIDE_CORRIDOR_PX   = 240   # ~1000 mm corridor in pixels — UNCALIBRATED GUESS
DEFAULT_NARROW_CORRIDOR_PX = 144   # ~600 mm corridor in pixels  — UNCALIBRATED GUESS

# If measured width is within this many px of a known size, lock to that size.
CORRIDOR_SNAP_TOLERANCE = 40   # px

# Minimum believable corridor width — below this = bad detection
MIN_VALID_CORRIDOR_PX = 30


def _load_corridor_config():
    if os.path.exists(CORRIDOR_CONFIG_PATH):
        try:
            with open(CORRIDOR_CONFIG_PATH) as f:
                return json.load(f)
        except Exception as e:
            logger.warning(f"Could not load {CORRIDOR_CONFIG_PATH}: {e}")
    return {}


class WallFollower:
    def __init__(self, mode="obstacle"):
        self.mode = mode

        cfg = _load_corridor_config()
        self.WIDE_CORRIDOR_PX   = cfg.get("wide_corridor_px",   DEFAULT_WIDE_CORRIDOR_PX)
        self.NARROW_CORRIDOR_PX = cfg.get("narrow_corridor_px", DEFAULT_NARROW_CORRIDOR_PX)

        self._last_error      = 0.0
        self._corridor_px     = self.WIDE_CORRIDOR_PX
        self._narrow_count    = 0
        self._wide_count      = 0
        logger.info(f"WallFollower initialised (mode={mode}, "
                    f"wide_px={self.WIDE_CORRIDOR_PX}, narrow_px={self.NARROW_CORRIDOR_PX})")

    # ── Public API ────────────────────────────────────────────────────────────
    def get_error(self, walls):
        """
        Compute steering error from wall detections.
        Positive error = car is too far left  → steer right
        Negative error = car is too far right → steer left

        Args:
            walls : dict with keys left, right, center_error from CVPipeline
        Returns:
            float : normalised error in [-1, 1]
        """
        if not walls:
            return self._last_error * 0.8

        left  = walls.get("left",  0)
        right = walls.get("right", FRAME_WIDTH)

        corridor_px = right - left

        if corridor_px < MIN_VALID_CORRIDOR_PX:
            logger.warning(f"Suspicious corridor width: {corridor_px}px — using last error")
            return self._last_error * 0.5

        if self.mode == "open":
            self._update_corridor_estimate(corridor_px)

        center_error = walls.get("center_error", FRAME_CENTER - ((left + right) // 2))

        half_corridor = self._corridor_px / 2.0
        normalised = center_error / half_corridor
        normalised = max(-1.0, min(1.0, normalised))

        self._last_error = normalised
        return normalised

    @property
    def corridor_type(self):
        """Returns 'wide' (1000 mm) or 'narrow' (600 mm). Always 'wide' in obstacle mode."""
        if self.mode == "obstacle":
            return "wide"
        if abs(self._corridor_px - self.NARROW_CORRIDOR_PX) < CORRIDOR_SNAP_TOLERANCE:
            return "narrow"
        return "wide"

    def reset(self):
        self._last_error   = 0.0
        self._corridor_px  = self.WIDE_CORRIDOR_PX
        self._narrow_count = 0
        self._wide_count   = 0

    # ── Internal ──────────────────────────────────────────────────────────────
    def _update_corridor_estimate(self, measured_px):
        near_narrow = abs(measured_px - self.NARROW_CORRIDOR_PX) < CORRIDOR_SNAP_TOLERANCE
        near_wide   = abs(measured_px - self.WIDE_CORRIDOR_PX)   < CORRIDOR_SNAP_TOLERANCE

        if near_narrow:
            self._narrow_count += 1
            self._wide_count    = 0
        elif near_wide:
            self._wide_count   += 1
            self._narrow_count  = 0
        else:
            self._corridor_px  = measured_px
            self._narrow_count = 0
            self._wide_count   = 0
            return

        HYSTERESIS = 3
        if self._narrow_count >= HYSTERESIS:
            if self._corridor_px != self.NARROW_CORRIDOR_PX:
                logger.info("Corridor: wide → narrow (600 mm)")
            self._corridor_px = self.NARROW_CORRIDOR_PX

        elif self._wide_count >= HYSTERESIS:
            if self._corridor_px != self.WIDE_CORRIDOR_PX:
                logger.info("Corridor: narrow → wide (1000 mm)")
            self._corridor_px = self.WIDE_CORRIDOR_PX