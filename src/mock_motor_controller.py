"""
Mock Motor Controller (PC Development)
Simulates motor and servo output for testing on PC.
Prints values to console so you can verify the logic.
"""

import logging

logger = logging.getLogger(__name__)


class MotorController:
    def __init__(self):
        self._steering = 0.0
        self._speed    = 0
        logger.info("MockMotorController initialised (PC mode)")

    def start(self):
        print("[MOCK MOTORS] Started")

    def stop(self):
        print("[MOCK MOTORS] Stopped")

    def set_steering(self, value):
        self._steering = max(-1.0, min(1.0, value))
        logger.debug(f"[MOCK] Steering: {self._steering:.3f}")

    def set_speed(self, speed_mm_s):
        self._speed = speed_mm_s
        logger.debug(f"[MOCK] Speed: {self._speed} mm/s")

    @property
    def steering(self):
        return self._steering

    @property
    def speed(self):
        return self._speed
