"""
Section Detector
Detects orange and blue lines to determine:
- Which section (0-7) the car is currently in
- When a lap is complete
- Direction of travel (CW or CCW)

WRO Future Engineers track layout:
- 4 straight sections (between corners)
- 4 corner sections
- Orange lines mark corners
- Blue lines mark straight midpoints
- Direction determined by which corner is hit first
"""

import time
import logging

logger = logging.getLogger(__name__)

# Section types
STRAIGHT = "straight"
CORNER   = "corner"

SECTION_TYPES = [
    STRAIGHT, CORNER, STRAIGHT, CORNER,
    STRAIGHT, CORNER, STRAIGHT, CORNER
]

# Minimum time between section transitions (debounce)
MIN_SECTION_TIME = 0.5   # seconds


class SectionDetector:
    def __init__(self):
        self.current_section  = 0
        self.sections_passed  = 0
        self.direction        = None   # "CW" or "CCW"
        self._last_transition = 0.0
        self._direction_set   = False

        # Line state machine — prevent multiple triggers on same line
        self._orange_seen     = False
        self._blue_seen       = False

        logger.info("SectionDetector initialised")

    # ── Public API ────────────────────────────────────────────────────────────
    def update(self, lines):
        """
        Process line detections and update section tracking.
        Args:
            lines : {"orange": bool, "blue": bool} from CVPipeline
        Returns:
            dict or None:
                {
                  "section"     : int,
                  "type"        : "straight" | "corner",
                  "direction"   : "CW" | "CCW" | None,
                  "lap_complete": bool
                }
        """
        if not lines:
            return None

        now = time.time()

        # Debounce
        if now - self._last_transition < MIN_SECTION_TIME:
            # Reset seen flags when line clears
            if not lines.get("orange"):
                self._orange_seen = False
            if not lines.get("blue"):
                self._blue_seen = False
            return None

        orange = lines.get("orange", False)
        blue   = lines.get("blue",   False)

        event = None

        # Orange line → entering a corner section
        if orange and not self._orange_seen:
            self._orange_seen   = True
            self._last_transition = now
            event = self._advance_section("orange")

        # Blue line → entering a straight section
        elif blue and not self._blue_seen:
            self._blue_seen     = True
            self._last_transition = now
            event = self._advance_section("blue")

        # Clear flags when lines disappear
        if not orange:
            self._orange_seen = False
        if not blue:
            self._blue_seen = False

        return event

    @property
    def section_type(self):
        return SECTION_TYPES[self.current_section % len(SECTION_TYPES)]

    def reset(self):
        self.current_section  = 0
        self.sections_passed  = 0
        self.direction        = None
        self._direction_set   = False
        self._orange_seen     = False
        self._blue_seen       = False

    # ── Internal ──────────────────────────────────────────────────────────────
    def _advance_section(self, line_color):
        prev_section      = self.current_section
        self.current_section = (self.current_section + 1) % 8
        self.sections_passed += 1

        # Determine direction from first corner hit
        if not self._direction_set and line_color == "orange":
            # First orange line seen — direction depends on which side
            # This is a placeholder; refine based on your track orientation
            self.direction     = "CW"
            self._direction_set = True
            logger.info(f"Direction set: {self.direction}")

        # Lap complete every 8 sections
        lap_complete = (self.sections_passed > 0 and
                        self.sections_passed % 8 == 0)

        if lap_complete:
            logger.info(f"Lap complete at section {self.current_section}")

        event = {
            "section":      self.current_section,
            "type":         self.section_type,
            "direction":    self.direction,
            "lap_complete": lap_complete,
        }

        logger.debug(f"Section {prev_section} → {self.current_section} "
                     f"({self.section_type}) via {line_color}")
        return event
