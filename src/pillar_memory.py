"""
Pillar Memory Map
Stores pillar positions detected during lap 1.
On laps 2 and 3, recalled memory supplements live CV
allowing the car to react earlier and drive faster.

Each section entry is also tagged with the IMU heading at the moment
it was first recorded (heading relative to that section's start, from
ImuTracker.reset_heading() at the last line crossing). On recall, the
caller can check that against the current in-section heading before
trusting the memory -- if the car's heading through this section looks
different from lap 1 (e.g. a missed/extra line-crossing put it out of
sync), the memory is stale and should be ignored rather than trusted
at higher speed. This is a relative check, not a global position check
-- there are no wheel encoders, so absolute distance still isn't
trustworthy, but heading-since-section-start is cheap and reliable.
"""

import logging
import json
import os

logger = logging.getLogger(__name__)

# WRO Future Engineers track has 8 sections:
# 4 straight sections + 4 corner sections
TOTAL_SECTIONS = 8

CONFIDENCE_TARGET   = 5     # samples to reach full (1.0) confidence
HEADING_TOLERANCE_DEG = 20.0  # max heading deviation from lap-1 to still trust recall


class PillarMemory:
    def __init__(self, save_path="../logs/pillar_map.json"):
        """
        Args:
            save_path : persist map to disk so it survives restarts (optional)
        """
        self.save_path = save_path
        self.current_lap = 1

        # Map structure: {section_id: {"red": (cx,cy), "green": (cx,cy)}}
        self._map = {i: {"red": None, "green": None} for i in range(TOTAL_SECTIONS)}

        # Confidence counter per section
        self._confidence = {i: 0 for i in range(TOTAL_SECTIONS)}

        # Heading (deg, relative to section start) recorded the first time
        # each section got a detection during lap 1.
        self._entry_heading = {i: None for i in range(TOTAL_SECTIONS)}

        # Grid-classified column (0..4, see sign_grid.py) for each pillar,
        # if the lap-1 reading confidently snapped to a known card
        # position. This is the PRIMARY recall target on lap 2/3 when
        # present -- exact, precomputed geometry instead of a re-averaged
        # live estimate. None means lap 1 never got a confident
        # classification for that pillar (ambiguous readings, or the
        # pillar was never in an unobstructed enough view); the caller
        # should fall back to _wall_offset_mm below in that case.
        self._grid_column = {i: {"red": None, "green": None} for i in range(TOTAL_SECTIONS)}

        # Distance from the LEFT wall to the obstacle, in mm (from
        # WallFollower.lateral_offset_mm), recorded alongside each pillar.
        # This is what lets lap 2/3 steer to a known wall-gap and hold a
        # straight line past the obstacle, instead of only reacting to the
        # pillar's raw on-screen position (which shifts frame to frame).
        self._wall_offset_mm = {i: {"red": None, "green": None} for i in range(TOTAL_SECTIONS)}

        logger.info("PillarMemory initialised")

    # ── Public API ────────────────────────────────────────────────────────────
    def update(self, section, red, green, heading_deg=0.0, wall_offset_mm=None,
               grid_column=None):
        """
        Update memory with current detections.
        Only stores during lap 1, uses higher confidence on repeated detections.
        Args:
            section        : current section index (0-7)
            red            : (cx, cy, area, dist) from Kalman.predict(), or None
            green          : (cx, cy, area, dist) from Kalman.predict(), or None
            heading_deg    : IMU heading relative to this section's start
                             (ImuTracker.heading_deg since the last
                             reset_heading() call). Defaults to 0.0 so callers
                             without an IMU wired up behave exactly as before.
            wall_offset_mm : {"red": mm or None, "green": mm or None} — distance
                             from the left wall to each pillar, from
                             WallFollower.lateral_offset_mm(). Optional; if
                             omitted, no wall-relative alignment data is stored.
            grid_column    : {"red": int or None, "green": int or None} — the
                             SignGrid-classified column (0..4) for each
                             pillar, if lap 1's reading confidently snapped
                             to a known card position. Optional; when a
                             color's entry is None (ambiguous classification
                             that frame), the existing wall_offset_mm value
                             remains the fallback for that color.
        """
        if self.current_lap != 1:
            return
        if section is None:
            return

        # Only store if confidence threshold not yet reached
        if self._confidence[section] < CONFIDENCE_TARGET:
            if red:
                self._map[section]["red"] = self._merge(self._map[section]["red"], red)
            if green:
                self._map[section]["green"] = self._merge(self._map[section]["green"], green)

            if red or green:
                if self._entry_heading[section] is None:
                    self._entry_heading[section] = heading_deg
                if wall_offset_mm:
                    if red and wall_offset_mm.get("red") is not None \
                            and self._wall_offset_mm[section]["red"] is None:
                        self._wall_offset_mm[section]["red"] = wall_offset_mm["red"]
                    if green and wall_offset_mm.get("green") is not None \
                            and self._wall_offset_mm[section]["green"] is None:
                        self._wall_offset_mm[section]["green"] = wall_offset_mm["green"]
                if grid_column:
                    if red and grid_column.get("red") is not None \
                            and self._grid_column[section]["red"] is None:
                        self._grid_column[section]["red"] = grid_column["red"]
                    if green and grid_column.get("green") is not None \
                            and self._grid_column[section]["green"] is None:
                        self._grid_column[section]["green"] = grid_column["green"]
                self._confidence[section] += 1
                logger.debug(f"Section {section} confidence: {self._confidence[section]}")

    @staticmethod
    def _merge(existing, new):
        """Element-wise average of two (cx, cy, area, dist)-shaped tuples,
        for stability across repeated lap-1 detections. Falls back to
        whichever side has a value when the other is missing/shorter."""
        if existing is None:
            return new
        if new is None:
            return existing
        length = max(len(existing), len(new))
        merged = []
        for i in range(length):
            e = existing[i] if i < len(existing) else None
            n = new[i]      if i < len(new)      else None
            if e is None:
                merged.append(n)
            elif n is None:
                merged.append(e)
            else:
                merged.append((e + n) / 2)
        return tuple(merged)

    def recall(self, section, heading_deg=None):
        """
        Returns stored pillar positions for a section.
        Returns None if nothing stored, not past lap 1, or (when
        heading_deg is given) the car's current in-section heading has
        drifted too far from what was recorded during lap 1 -- e.g. a
        missed line-crossing put section counting out of sync with the
        real track, so trusting stale positional memory would be worse
        than falling back to live CV alone.
        Args:
            section     : current section index
            heading_deg : optional current ImuTracker.heading_deg since
                          this section's reset_heading(). If provided and
                          it deviates from the recorded entry heading by
                          more than HEADING_TOLERANCE_DEG, recall is
                          suppressed for this section.
        Returns:
            {"red": (cx,cy,area,dist) or None, "green": (cx,cy,area,dist) or None} or None
        """
        if self.current_lap == 1:
            return None
        if section is None:
            return None

        if heading_deg is not None:
            entry = self._entry_heading.get(section)
            if entry is not None and abs(heading_deg - entry) > HEADING_TOLERANCE_DEG:
                logger.debug(f"Section {section}: heading mismatch "
                             f"(now={heading_deg:.1f} vs lap1={entry:.1f}) -- recall suppressed")
                return None

        return self._map.get(section)

    def recall_wall_offset(self, section):
        """
        Returns {"red": mm or None, "green": mm or None} — the stored
        distance from the left wall to each pillar in this section, for
        computing a steering target that holds a straight line past a
        known obstacle instead of only reacting to its live on-screen
        position. Returns None if not past lap 1 or section is None.

        This is the FALLBACK target -- prefer recall_grid_column() when it
        has a value for this section/color; only use this continuous
        estimate when the grid classifier couldn't confidently place lap
        1's reading on a known card position (see sign_grid.py).
        """
        if self.current_lap == 1 or section is None:
            return None
        return self._wall_offset_mm.get(section)

    def recall_grid_column(self, section):
        """
        Returns {"red": int or None, "green": int or None} — the
        SignGrid-classified column (0..4) recorded for each pillar in
        this section during lap 1, if any. This is the PRIMARY recall
        target: exact precomputed geometry (via
        SignGrid.target_mm_for_cell()) rather than a re-averaged live mm
        estimate. Returns None if not past lap 1 or section is None.
        """
        if self.current_lap == 1 or section is None:
            return None
        return self._grid_column.get(section)

    def confidence(self, section):
        """
        Returns this section's memory confidence as a fraction [0.0, 1.0],
        for callers (e.g. SpeedController) that want to scale a speed
        boost by how trustworthy the recalled data actually is, rather
        than assuming every section is equally reliable just because the
        lap number is 2 or 3.
        """
        if section is None:
            return 0.0
        return min(1.0, self._confidence.get(section, 0) / CONFIDENCE_TARGET)

    def next_lap(self):
        """Called when a lap is completed."""
        self.current_lap += 1
        if self.current_lap == 2:
            self._save()
            logger.info("Lap 1 complete — pillar map saved")
            self._log_map()

    def reset(self):
        """Full reset for new round."""
        self.current_lap = 1
        self._map        = {i: {"red": None, "green": None} for i in range(TOTAL_SECTIONS)}
        self._confidence = {i: 0 for i in range(TOTAL_SECTIONS)}
        self._entry_heading = {i: None for i in range(TOTAL_SECTIONS)}
        self._wall_offset_mm = {i: {"red": None, "green": None} for i in range(TOTAL_SECTIONS)}
        self._grid_column = {i: {"red": None, "green": None} for i in range(TOTAL_SECTIONS)}

    def coverage(self):
        """Returns % of sections with at least one pillar mapped."""
        mapped = sum(
            1 for v in self._map.values()
            if v["red"] is not None or v["green"] is not None
        )
        return (mapped / TOTAL_SECTIONS) * 100

    # ── Persistence ───────────────────────────────────────────────────────────
    def _save(self):
        try:
            os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
            serializable = {
                str(k): {
                    "red":   list(v["red"])   if v["red"]   else None,
                    "green": list(v["green"]) if v["green"] else None,
                }
                for k, v in self._map.items()
            }
            with open(self.save_path, "w") as f:
                json.dump(serializable, f, indent=2)
        except Exception as e:
            logger.warning(f"Could not save pillar map: {e}")

    def _log_map(self):
        logger.info(f"Pillar map coverage: {self.coverage():.0f}%")
        for i, v in self._map.items():
            logger.info(f"  Section {i}: red={v['red']} green={v['green']}")
