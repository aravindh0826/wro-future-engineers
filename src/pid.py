"""
PID Controller
Generic PID for steering and any other control loop.
Includes anti-windup, output clamping, derivative smoothing.
"""

import time
import logging

logger = logging.getLogger(__name__)


class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0,
                 output_min=-1.0, output_max=1.0,
                 integral_limit=10.0):
        """
        Args:
            kp, ki, kd      : PID gains
            output_min/max  : clamp output to this range
            integral_limit  : anti-windup — limits integral accumulation
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min    = output_min
        self.output_max    = output_max
        self.integral_limit = integral_limit

        self._integral     = 0.0
        self._prev_error   = 0.0
        self._prev_time    = None

        # Derivative low-pass filter coefficient (0=no filter, 1=max filter)
        self._d_filter     = 0.7
        self._prev_d       = 0.0

        logger.debug(f"PID init: kp={kp} ki={ki} kd={kd}")

    # ── Public API ────────────────────────────────────────────────────────────
    def compute(self, error):
        """
        Compute PID output for given error.
        Args:
            error : setpoint - measured value
        Returns:
            float : control output clamped to [output_min, output_max]
        """
        now = time.time()

        if self._prev_time is None:
            dt = 0.033   # assume ~30fps on first call
        else:
            dt = now - self._prev_time
            dt = max(dt, 1e-6)   # prevent division by zero

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral  = max(-self.integral_limit,
                              min(self.integral_limit, self._integral))
        i = self.ki * self._integral

        # Derivative with low-pass filter
        raw_d = (error - self._prev_error) / dt
        d_filtered = self._d_filter * self._prev_d + (1 - self._d_filter) * raw_d
        d = self.kd * d_filtered

        output = p + i + d
        output = max(self.output_min, min(self.output_max, output))

        self._prev_error = error
        self._prev_time  = now
        self._prev_d     = d_filtered

        return output

    def reset(self):
        """Reset integral and derivative state."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = None
        self._prev_d     = 0.0

    def set_gains(self, kp=None, ki=None, kd=None):
        """Update gains at runtime."""
        if kp is not None: self.kp = kp
        if ki is not None: self.ki = ki
        if kd is not None: self.kd = kd
        self.reset()
