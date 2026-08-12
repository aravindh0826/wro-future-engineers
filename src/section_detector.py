"""
Section Detector
Tracks which of the 8 track sections (4 straight + 4 corner) the car is in
and counts laps from orange (corner) / blue (straight) line crossings.

Driving direction (CW/CCW) is fixed and known before the round starts
(rule 9.8 — the team physically orients the front axle to the announced
direction), so it is passed in, not inferred from vision.
"""

import time
import logging

logger = logging.getLogger(__name__)

STRAIGHT = "straight"
CORNER   = "corner"

SECTION_TYPES = [STRAIGHT, CORNER, STRAIGHT, CORNER, STRAIGHT, CORNER, STRAIGHT, CORNER]

MIN_SECTION_TIME = 0.5   # seconds, debounce


class SectionDetector:
    def __init__(self, direction="CW"):
        """
        Args:
            direction : "CW" or "CCW" — the round's driving direction,
                        set once per round before the vehicle starts.
        """
        if direction not in ("CW", "CCW"):
            raise ValueError("direction must be 'CW' or 'CCW'")
        self.direction = direction

        self.current_section  = 0
        self.sections_passed  = 0
        self._last_transition = 0.0

        self._orange_seen = False
        self._blue_seen   = False

        logger.info(f"SectionDetector initialised (direction={direction})")

    # ── Public API ────────────────────────────────────────────────────────────
    def update(self, lines):
        """
        Args:
            lines : {"orange": bool, "blue": bool} from CVPipeline
        Returns:
            dict or None: {"section", "type", "direction", "lap_complete"}
        """
        if not lines:
            return None

        now = time.time()

        if now - self._last_transition < MIN_SECTION_TIME:
            if not lines.get("orange"):
                self._orange_seen = False
            if not lines.get("blue"):
                self._blue_seen = False
            return None

        orange = lines.get("orange", False)
        blue   = lines.get("blue", False)
        event = None

        if orange and not self._orange_seen:
            self._orange_seen = True
            self._last_transition = now
            event = self._advance_section("orange")
        elif blue and not self._blue_seen:
            self._blue_seen = True
            self._last_transition = now
            event = self._advance_section("blue")

        if not orange:
            self._orange_seen = False
        if not blue:
            self._blue_seen = False

        return event

    @property
    def section_type(self):
        return SECTION_TYPES[self.current_section % len(SECTION_TYPES)]

    def reset(self, direction=None):
        if direction is not None:
            if direction not in ("CW", "CCW"):
                raise ValueError("direction must be 'CW' or 'CCW'")
            self.direction = direction
        self.current_section  = 0
        self.sections_passed  = 0
        self._orange_seen     = False
        self._blue_seen       = False

    # ── Internal ──────────────────────────────────────────────────────────────
    def _advance_section(self, line_color):
        prev_section = self.current_section
        self.current_section = (self.current_section + 1) % 8
        self.sections_passed += 1

        lap_complete = self.sections_passed > 0 and self.sections_passed % 8 == 0
        if lap_complete:
            logger.info(f"Lap complete at section {self.current_section}")

        event = {
            "section":      self.current_section,
            "type":         self.section_type,
            "direction":    self.direction,
            "lap_complete": lap_complete,
        }
        logger.debug(f"Section {prev_section} -> {self.current_section} "
                     f"({self.section_type}) via {line_color}")
        return event
