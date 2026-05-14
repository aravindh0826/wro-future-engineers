"""
Pillar Memory Map
Stores pillar positions detected during lap 1.
On laps 2 and 3, recalled memory supplements live CV
allowing the car to react earlier and drive faster.
"""

import logging
import json
import os

logger = logging.getLogger(__name__)

# WRO Future Engineers track has 8 sections:
# 4 straight sections + 4 corner sections
TOTAL_SECTIONS = 8


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

        logger.info("PillarMemory initialised")

    # ── Public API ────────────────────────────────────────────────────────────
    def update(self, section, red, green):
        """
        Update memory with current detections.
        Only stores during lap 1, uses higher confidence on repeated detections.
        Args:
            section : current section index (0-7)
            red     : (cx, cy) from Kalman or None
            green   : (cx, cy) from Kalman or None
        """
        if self.current_lap != 1:
            return
        if section is None:
            return

        # Only store if confidence threshold not yet reached
        if self._confidence[section] < 5:
            if red:
                existing = self._map[section]["red"]
                if existing:
                    # Average with existing for stability
                    self._map[section]["red"] = (
                        (existing[0] + red[0]) / 2,
                        (existing[1] + red[1]) / 2
                    )
                else:
                    self._map[section]["red"] = red

            if green:
                existing = self._map[section]["green"]
                if existing:
                    self._map[section]["green"] = (
                        (existing[0] + green[0]) / 2,
                        (existing[1] + green[1]) / 2
                    )
                else:
                    self._map[section]["green"] = green

            if red or green:
                self._confidence[section] += 1
                logger.debug(f"Section {section} confidence: {self._confidence[section]}")

    def recall(self, section):
        """
        Returns stored pillar positions for a section.
        Returns None if nothing stored or not past lap 1.
        Args:
            section : current section index
        Returns:
            {"red": (cx,cy) or None, "green": (cx,cy) or None} or None
        """
        if self.current_lap == 1:
            return None
        if section is None:
            return None
        return self._map.get(section)

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
