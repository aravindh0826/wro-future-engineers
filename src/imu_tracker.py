"""
IMU Tracker (MPU6050)
Tracks RELATIVE heading only -- not a global x,y position.

Why relative, not global: there are no wheel encoders on this vehicle,
so there is no trustworthy distance-traveled signal. A gyro integrated
over a full 3-lap run would drift into garbage regardless of how good
the bias calibration is. Instead, heading is zeroed at every section
(line-crossing) boundary via reset_heading(), so drift never accumulates
past a few seconds of one section -- MIN_SECTION_TIME/typical section
transit time is small, so the integration window is short enough that
raw gyro-Z integration (no complementary/Kalman fusion) is accurate
enough for two jobs:
  1. Gating PillarMemory recall (does current heading-in-section match
     what lap 1 saw at this point? if not, don't trust the recall).
  2. Feeding yaw-rate into the pillar KalmanFilter's motion model so
     predicted pillar position accounts for the car's own turning
     during occlusion (see kalman_filter.py compensate_rotation()).

On PC (dev/testing), MockImuTracker returns zeros so the rest of the
pipeline runs unchanged without real I2C hardware attached.
"""

import time
import logging

from camera import IS_PI

logger = logging.getLogger(__name__)

MPU6050_ADDR   = 0x68
PWR_MGMT_1     = 0x6B
GYRO_CONFIG    = 0x1B
GYRO_ZOUT_H    = 0x47

# ±250 deg/s range (GYRO_CONFIG = 0x00) -> 131 LSB per deg/s
GYRO_SCALE_DPS = 131.0

CALIBRATION_SAMPLES = 200   # ~1s at typical I2C read rate, car must be still


class ImuTracker:
    def __init__(self, bus_num=1, address=MPU6050_ADDR):
        self.address   = address
        self._bus_num  = bus_num
        self._bus      = None

        self._gyro_bias_dps = 0.0
        self._heading_deg   = 0.0
        self._yaw_rate_dps  = 0.0
        self._last_t        = None
        self._calibrated    = False

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def start(self):
        import smbus2
        self._bus = smbus2.SMBus(self._bus_num)
        # Wake the chip (default power-on state is sleep)
        self._bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # ±250 deg/s, most sensitive range -- fine for section-relative heading
        self._bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)
        time.sleep(0.05)
        self._calibrate_bias()
        self._last_t = time.time()
        logger.info(f"ImuTracker started (bias={self._gyro_bias_dps:.3f} dps)")

    def stop(self):
        if self._bus is not None:
            self._bus.close()

    # ── Public API ────────────────────────────────────────────────────────────
    def update(self):
        """Call once per main-loop iteration. Integrates gyro-Z into the
        running relative heading. Cheap (single I2C read) -- safe to call
        every frame."""
        now = time.time()
        dt  = now - self._last_t if self._last_t else 0.0
        self._last_t = now

        raw = self._read_gyro_z_raw()
        dps = (raw / GYRO_SCALE_DPS) - self._gyro_bias_dps
        self._yaw_rate_dps = dps

        if dt > 0:
            self._heading_deg += dps * dt

    def reset_heading(self):
        """Zero the relative heading. Call this at every section
        (line-crossing) boundary so gyro drift never accumulates past a
        single section's transit time."""
        self._heading_deg = 0.0

    @property
    def heading_deg(self):
        """Heading relative to the last reset_heading() call, degrees."""
        return self._heading_deg

    @property
    def yaw_rate_dps(self):
        """Instantaneous yaw rate, degrees/sec (signed)."""
        return self._yaw_rate_dps

    # ── Internal ──────────────────────────────────────────────────────────────
    def _calibrate_bias(self):
        """Average gyro-Z while assumed stationary at startup to remove
        constant bias drift. Vehicle MUST be still during this call --
        it runs once, right after motors/camera init, before driving starts."""
        total = 0.0
        for _ in range(CALIBRATION_SAMPLES):
            total += self._read_gyro_z_raw()
            time.sleep(0.002)
        self._gyro_bias_dps = (total / CALIBRATION_SAMPLES) / GYRO_SCALE_DPS
        self._calibrated = True

    def _read_gyro_z_raw(self):
        high = self._bus.read_byte_data(self.address, GYRO_ZOUT_H)
        low  = self._bus.read_byte_data(self.address, GYRO_ZOUT_H + 1)
        val  = (high << 8) | low
        if val >= 0x8000:
            val -= 0x10000
        return val


class MockImuTracker:
    """PC-mode stand-in. Always reports zero heading/yaw-rate so the rest
    of the pipeline (PillarMemory gating, Kalman rotation compensation)
    runs with no-op behaviour identical to before the IMU existed."""

    def __init__(self, *args, **kwargs):
        pass

    def start(self):
        print("[MOCK IMU] Started (heading always 0.0)")

    def stop(self):
        pass

    def update(self):
        pass

    def reset_heading(self):
        pass

    @property
    def heading_deg(self):
        return 0.0

    @property
    def yaw_rate_dps(self):
        return 0.0


def make_imu_tracker():
    """Factory mirroring camera.py's IS_PI pattern -- use this from main.py
    instead of constructing ImuTracker/MockImuTracker directly."""
    return ImuTracker() if IS_PI else MockImuTracker()
