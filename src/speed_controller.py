"""
Speed Controller
Adaptive speed based on:
- Wall error (large error = slow down)
- Pillar proximity (pillar near = slow down)
- Corner proximity (corner ahead = slow down)
- Wall directly ahead (approaching a corner/turn = slow down)
- Clear straight (speed up)
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

    # ── Public API ────────────────────────────────────────────────────────────
    def compute(self, wall_error=0.0, pillar_near=False,
                corner_near=False, wall_ahead=False, lap=1):
        """
        Compute target speed.
        Args:
            wall_error  : normalised [-1,1] from WallFollower
            pillar_near : bool from CVPipeline
            corner_near : bool from CVPipeline
            wall_ahead  : bool from CVPipeline — wall detected close ahead
            lap         : current lap number (faster on later laps)
        Returns:
            int : target speed in mm/s
        """
        speed = self.base

        # Lap multiplier — laps 2 & 3 are faster due to pillar memory
        if lap == 2:
            speed *= 1.2
        elif lap >= 3:
            speed *= 1.4

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