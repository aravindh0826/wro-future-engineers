"""
Motor Controller (Raspberry Pi)
Controls:
- DC drive motor via PWM (L298N or similar driver)
- Servo steering via PWM
Adjust GPIO pins and PWM frequencies to match your hardware.
"""

import logging
import time

logger = logging.getLogger(__name__)

# ── GPIO Pin Configuration ────────────────────────────────────────────────────
# Change these to match your wiring
SERVO_PIN    = 12   # PWM capable pin
MOTOR_IN1    = 20   # Motor direction pin 1
MOTOR_IN2    = 21   # Motor direction pin 2
MOTOR_EN     = 16   # Motor enable (PWM)

# ── Servo calibration ─────────────────────────────────────────────────────────
SERVO_CENTER  = 7.5   # duty cycle % for straight
SERVO_LEFT    = 5.0   # duty cycle % for full left
SERVO_RIGHT   = 10.0  # duty cycle % for full right
SERVO_FREQ    = 50    # Hz

# ── Motor calibration ─────────────────────────────────────────────────────────
MOTOR_FREQ    = 1000  # Hz
MAX_SPEED_MM  = 400   # mm/s at 100% duty cycle


class MotorController:
    def __init__(self):
        import RPi.GPIO as GPIO
        self.GPIO = GPIO
        self._setup()
        logger.info("MotorController initialised (Pi)")

    def start(self):
        self.set_steering(0)
        self.set_speed(0)
        logger.info("Motors started")

    def stop(self):
        self.set_speed(0)
        self.set_steering(0)
        self._motor_pwm.stop()
        self._servo_pwm.stop()
        self.GPIO.cleanup()
        logger.info("Motors stopped")

    # ── Steering ──────────────────────────────────────────────────────────────
    def set_steering(self, value):
        """
        Args:
            value : float [-1.0, 1.0] where -1=full left, 1=full right
        """
        value = max(-1.0, min(1.0, value))
        if value >= 0:
            duty = SERVO_CENTER + (SERVO_RIGHT - SERVO_CENTER) * value
        else:
            duty = SERVO_CENTER + (SERVO_CENTER - SERVO_LEFT) * value
        self._servo_pwm.ChangeDutyCycle(duty)

    # ── Speed ─────────────────────────────────────────────────────────────────
    def set_speed(self, speed_mm_s):
        """
        Args:
            speed_mm_s : target speed in mm/s (positive = forward)
        """
        duty = abs(speed_mm_s) / MAX_SPEED_MM * 100
        duty = max(0, min(100, duty))

        if speed_mm_s > 0:
            self.GPIO.output(MOTOR_IN1, self.GPIO.HIGH)
            self.GPIO.output(MOTOR_IN2, self.GPIO.LOW)
        elif speed_mm_s < 0:
            self.GPIO.output(MOTOR_IN1, self.GPIO.LOW)
            self.GPIO.output(MOTOR_IN2, self.GPIO.HIGH)
        else:
            self.GPIO.output(MOTOR_IN1, self.GPIO.LOW)
            self.GPIO.output(MOTOR_IN2, self.GPIO.LOW)

        self._motor_pwm.ChangeDutyCycle(duty)

    # ── Setup ─────────────────────────────────────────────────────────────────
    def _setup(self):
        GPIO = self.GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(SERVO_PIN,  GPIO.OUT)
        GPIO.setup(MOTOR_IN1,  GPIO.OUT)
        GPIO.setup(MOTOR_IN2,  GPIO.OUT)
        GPIO.setup(MOTOR_EN,   GPIO.OUT)

        self._servo_pwm = GPIO.PWM(SERVO_PIN, SERVO_FREQ)
        self._motor_pwm = GPIO.PWM(MOTOR_EN,  MOTOR_FREQ)

        self._servo_pwm.start(SERVO_CENTER)
        self._motor_pwm.start(0)
