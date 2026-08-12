"""
Speed Controller
Adaptive speed based on:
- Wall error (large error = slow down)
- Pillar proximity (pillar near = slow down)
- Corner proximity (corner ahead = slow down)
- Wall directly ahead (approaching a corner/turn = slow down)
- Clear straight (speed up)
- Section memory confidence (lap 2/3 boost is scaled by how trustworthy
  PillarMemory's recall is for the CURRENT section, not a flat per-lap
  multiplier). A section with 0/5 confidence gets no boost even on lap 3;
  a section at full 5/5 confidence gets the full boost as soon as it's
  reached on lap 2. This keeps lap 1 at full reactive speed (never
  artificially slowed) while making the lap 2/3 speedup track what the
  car actually knows, section by section, instead of assuming the whole
  lap is equally safe to rush.
"""

import logging

logger = logging.getLogger(__name__)


class SpeedController:
    def __init__(self, base=200, max_s=400, min_s=100):
        """
        Args:
            base  : normal cruising speed mm/s
            max_s : maximum speed on clear straights
            min_s : minimum speed in tight situations
        """
        self.base  = base
        self.max_s = max_s
        self.min_s = min_s
        self._current_speed = base
        logger.info(f"SpeedController: base={base} max={max_s} min={min_s}")

    # Maximum boost multiplier at full (1.0) section confidence, per lap.
    # Lap 1 always gets 1.0 (no memory exists yet to justify a boost).
    MAX_BOOST_BY_LAP = {1: 1.0, 2: 1.2, 3: 1.4}

    # ── Public API ────────────────────────────────────────────────────────────
    def compute(self, wall_error=0.0, pillar_near=False,
                corner_near=False, wall_ahead=False, lap=1,
                section_confidence=0.0):
        """
        Compute target speed.
        Args:
            wall_error          : normalised [-1,1] from WallFollower
            pillar_near         : bool from CVPipeline
            corner_near         : bool from CVPipeline
            wall_ahead          : bool from CVPipeline — wall detected close ahead
            lap                 : current lap number
            section_confidence  : PillarMemory.confidence(current_section),
                                   a float in [0.0, 1.0]. 0.0 (default) means
                                   "no memory for this section" and yields no
                                   boost even on lap 2/3 -- callers without
                                   pillar memory wired up behave exactly as
                                   before (base speed, capped at min_s/max_s).
        Returns:
            int : target speed in mm/s
        """
        speed = self.base

        # Confidence-gated lap boost — scales linearly from 1.0 (no
        # confidence) up to this lap's max boost (full confidence), instead
        # of assuming every section is equally safe to rush just because
        # the lap number went up.
        max_boost = self.MAX_BOOST_BY_LAP.get(lap, self.MAX_BOOST_BY_LAP[3] if lap > 3 else 1.0)
        confidence = max(0.0, min(1.0, section_confidence))
        boost = 1.0 + (max_boost - 1.0) * confidence
        speed *= boost

        # Slow down for large wall error (car is drifting)
        wall_penalty = abs(wall_error) * 0.5   # up to 50% speed reduction
        speed *= (1.0 - wall_penalty)

        # Slow down for pillars
        if pillar_near:
            speed *= 0.7

        # Slow down for corners
        if corner_near:
            speed *= 0.6

        # Slow down for a wall directly ahead (approaching a corner/turn)
        if wall_ahead:
            speed *= 0.75

        # Clamp
        speed = max(self.min_s, min(self.max_s, speed))
        self._current_speed = int(speed)

        return self._current_speed

    @property
    def current_speed(self):
        return self._current_speed