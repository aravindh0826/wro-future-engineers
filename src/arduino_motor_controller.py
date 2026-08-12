"""
Arduino Motor Controller (Raspberry Pi side)
Drop-in replacement for motor_controller.MotorController that offloads
servo/motor PWM generation to an Arduino (SBM) over a wired serial link,
instead of driving RPi.GPIO directly.

Same public interface as the direct-GPIO MotorController (start, stop,
set_steering, set_speed, steering_duty, steering, speed) so main.py needs
no changes beyond the import at the top of the file -- see the IS_PI
branch there.

WHY: keeps real-time PWM generation off the Pi, which reduces both Pi
CPU load and actuation jitter (PWM refresh no longer competes with the
CV pipeline for CPU/scheduling time), and decouples actuation from CV
frame rate since the Arduino continuously holds/outputs the last
setpoint between updates (see arduino/motor_controller/motor_controller.ino
for the receiving side and full rationale).

RULES:
    - Rule 11.8 / 11.9 : more than one SBC/SBM is explicitly permitted.
    - Rule 11.10 / 11.17 : communication between the Pi and the Arduino
      must be a physical wire (this uses USB-serial / UART) -- never
      Bluetooth/WiFi/RF.
    - Rule 11.6 : the vehicle must remain fully autonomous -- if the
      serial link drops, the Arduino's own watchdog (not this file) is
      what brings the vehicle to a safe stop. This class does not need to
      replicate that logic, but see reconnect handling below for why we
      don't silently swallow a dead link either.

PROTOCOL: identical to what the Arduino sketch expects --
    "C,<steer>,<speed>\n"
    steer : float in [-1.0, 1.0]
    speed : int, mm/s, signed

TELEMETRY (Arduino -> Pi, read via read_ultrasonic()):
    "U,<left_mm>,<right_mm>\n" -- rear-left / rear-right ultrasonic
    distance, sent unsolicited by the Arduino every ~120ms (Rule 11.11,
    any sensor permitted). -1 means "no echo" for that sensor. Used by
    ultrasonic_parking_controller.py for the blind-spot reverse manoeuvre,
    where the forward camera can no longer see the magenta parking
    markers -- see arduino/motor_controller/motor_controller.ino.
"""

import logging
import time

logger = logging.getLogger(__name__)

# ── Serial configuration ────────────────────────────────────────────────────
# Change to match your wiring. Common Arduino serial-over-USB device names:
#   Arduino Uno/Nano (CH340 or similar): /dev/ttyUSB0
#   Arduino Uno/Nano/Micro (native USB, e.g. Leonardo): /dev/ttyACM0
# Check with `ls /dev/tty*` before/after plugging in the Arduino.
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 115200
# How long to wait for the Arduino bootloader to finish resetting after the
# serial port opens, before we trust the link enough to send commands.
BOOT_SETTLE_S = 2.0


def steering_value_to_duty(value):
    """
    Kept for interface/logging parity with motor_controller.py's version
    (used by debug_visualizer.py) even though actual duty-cycle computation
    now happens on the Arduino. Mirrors the same convention so HUD/logging
    code doesn't need to know which controller backend is active.

    Args:
        value : float [-1.0, 1.0], -1=full left, 1=full right (front axle
                only -- Rule 11.3, single-servo Ackermann front steering;
                rear axle is drive-only, Rules 11.3/11.5).
    """
    value = max(-1.0, min(1.0, value))
    label = "STRAIGHT" if value == 0 else ("RIGHT" if value > 0 else "LEFT")
    # Nominal duty-cycle-equivalent for display only; actual PWM is
    # generated on the Arduino from the same [-1, 1] value.
    duty = 7.5 + 2.5 * value
    return duty, label


class ArduinoMotorController:
    def __init__(self, port=SERIAL_PORT, baud=BAUD_RATE):
        import serial   # pyserial -- add to requirements.txt
        self._serial_module = serial
        self._port = port
        self._baud = baud
        self._ser = None
        self._steering = 0.0
        self._speed = 0
        self._ultrasonic = None       # (left_mm, right_mm) or None if never seen
        self._ultrasonic_t = 0.0
        self._rx_buffer = ""
        logger.info(f"ArduinoMotorController initialised (port={port}, baud={baud})")

    def start(self):
        self._ser = self._serial_module.Serial(self._port, self._baud, timeout=0.05)
        # Opening the port resets most Arduino boards; give the bootloader
        # time to finish before sending real commands, or the first few
        # setpoints get eaten.
        time.sleep(BOOT_SETTLE_S)
        self.set_steering(0.0)
        self.set_speed(0)
        logger.info("Motors started (Arduino link up)")

    def stop(self):
        try:
            self.set_speed(0)
            self.set_steering(0.0)
        finally:
            if self._ser is not None:
                self._ser.close()
        logger.info("Motors stopped (Arduino link closed)")

    # ── Steering ──────────────────────────────────────────────────────────────
    def set_steering(self, value):
        """
        Args:
            value : float [-1.0, 1.0] where -1=full left, 1=full right
        """
        self._steering = max(-1.0, min(1.0, value))
        # Sent combined with speed below -- main.py always calls
        # set_steering() immediately followed by set_speed() each loop
        # iteration, so we only actually write to serial once per iteration
        # rather than twice.

    @property
    def steering_duty(self):
        """(duty_percent, direction_label) for logging/HUD parity with the
        direct-GPIO controller (Rule 11.3 — single-servo Ackermann front
        steering)."""
        return steering_value_to_duty(self._steering)

    # ── Speed ─────────────────────────────────────────────────────────────────
    def set_speed(self, speed_mm_s):
        """
        Args:
            speed_mm_s : target speed in mm/s (positive = forward)
        """
        self._speed = speed_mm_s
        self._send_command()

    @property
    def steering(self):
        return self._steering

    @property
    def speed(self):
        return self._speed

    # ── Ultrasonic telemetry (rear-left/right, parking use only) ───────────────
    def read_ultrasonic(self, max_age_s=0.5):
        """
        Returns (left_mm, right_mm) from the rear-corner ultrasonic sensors,
        or None if no reading has arrived yet or the last one is stale
        (link dropped/parking module not wired). -1 in either field means
        that sensor got no echo (out of range). Callers (e.g.
        ultrasonic_parking_controller.py) MUST handle None by falling back
        to a vision-only / open-loop behaviour -- this is a defensive
        requirement since the sensors haven't been mounted/tested on the
        real chassis yet.
        """
        self._poll_telemetry()
        if self._ultrasonic is None:
            return None
        if time.time() - self._ultrasonic_t > max_age_s:
            return None
        return self._ultrasonic

    def _poll_telemetry(self):
        if self._ser is None:
            return
        try:
            n = self._ser.in_waiting
            if n:
                self._rx_buffer += self._ser.read(n).decode("ascii", errors="ignore")
        except (OSError, self._serial_module.SerialException) as exc:
            logger.error(f"Serial read failed: {exc}")
            return

        while "\n" in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split("\n", 1)
            line = line.strip()
            if not line.startswith("U,"):
                continue
            parts = line.split(",")
            if len(parts) != 3:
                continue
            try:
                left_mm = float(parts[1])
                right_mm = float(parts[2])
            except ValueError:
                continue
            self._ultrasonic = (left_mm, right_mm)
            self._ultrasonic_t = time.time()

    # ── Serial transport ─────────────────────────────────────────────────────
    def _send_command(self):
        if self._ser is None:
            return
        line = f"C,{self._steering:.4f},{int(self._speed)}\n"
        try:
            self._ser.write(line.encode("ascii"))
        except (OSError, self._serial_module.SerialException) as exc:
            # Don't crash the control loop over a flaky USB link -- the
            # Arduino's own watchdog (WATCHDOG_MS in the .ino) will bring
            # the vehicle to a safe stop if commands stop arriving. Just
            # log it loudly so it's visible during development/testing.
            logger.error(f"Serial write failed: {exc}")
