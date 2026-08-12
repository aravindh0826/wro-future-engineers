"""
Start Switch / Start Button (Raspberry Pi side)

WRO 2026 rules on starting the vehicle (rules ~9.8-9.14, "Starting
Procedure"):
    - The vehicle is placed in the starting zone totally SWITCHED OFF.
    - It is then switched ON via exactly one switch. That switch is a
      physical, hardware, in-line power switch (battery -> SBC/SBM) --
      NOT something this file controls. Wire it in series with your main
      battery per docs/ARDUINO_OFFLOAD.md; there is nothing to code for
      it, it just cuts power.
    - After being switched on, "the vehicle should then be in a waiting
      state. Waiting for a Start button to be pressed. The Start button
      could be on the main SBC/SBM or a separately installed Push
      Button. Only one Start button is allowed." Pressing it "must start
      the vehicle action to attempt the challenge round."

So there are TWO separate things, not one:
    1. Power switch -- hardware only, in series with the battery. Turns
       the Pi/Arduino on. main.py (this whole process) doesn't exist
       until this happens, by definition.
    2. Start button -- THIS file. Once main.py has booted and finished
       initialising the camera/motors/IMU, it must sit in a "waiting
       state" and do nothing else until this button is pressed. Only
       then does driving begin. This satisfies "no external/remote
       control" (Rule 11.6) too -- the vehicle only ever reacts to a
       button physically on itself, never a remote signal.

WIRING: a momentary push-button between START_BUTTON_PIN (BCM numbering)
and GND. Uses the Pi's internal pull-up, so the pin reads HIGH when not
pressed and LOW when pressed -- no external resistor needed. Change
START_BUTTON_PIN to match wherever you actually wire it.

PC / mock mode: there's no real button, so wait_for_press() waits for
Enter on the keyboard instead, so debug_visualizer.py / PC testing isn't
blocked forever.
"""

import logging
import time

from camera import IS_PI

logger = logging.getLogger(__name__)

# BCM pin number -- change to match your wiring. Must not collide with any
# pin already used by camera/IMU (I2C: GPIO2/3) or anything else on the Pi
# -- this build doesn't drive the servo/motor GPIO directly (that's on the
# Arduino now), so the main constraint is just I2C (GPIO2/3, used by the
# MPU6050) and whatever the Arduino USB-serial link uses (not GPIO at all).
START_BUTTON_PIN = 26

DEBOUNCE_S = 0.05


class StartSwitch:
    """
    Call wait_for_press() once, after all other init (camera, motors,
    IMU calibration) is complete and the vehicle is sitting stationary in
    the starting zone. Blocks until the physical start button is
    pressed, then returns. Intended to be called exactly once per round.
    """

    def __init__(self, pin=START_BUTTON_PIN):
        self._pin = pin
        self._gpio = None
        if IS_PI:
            import RPi.GPIO as GPIO
            self._gpio = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self._pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        logger.info(f"StartSwitch initialised (pin={pin}, IS_PI={IS_PI})")

    def wait_for_press(self):
        if IS_PI:
            self._wait_for_press_pi()
        else:
            self._wait_for_press_pc()

    def _wait_for_press_pi(self):
        GPIO = self._gpio
        print(f"Vehicle ready. Waiting for start button (GPIO{self._pin})...")
        logger.info("Waiting for start button press")
        # Wait for a clean press: pin reads LOW while held (pull-up +
        # button-to-GND), require it to be HIGH first (not already held
        # down from a previous round / stuck wiring) before arming.
        while GPIO.input(self._pin) == GPIO.LOW:
            time.sleep(0.05)
        while GPIO.input(self._pin) == GPIO.HIGH:
            time.sleep(0.02)
        time.sleep(DEBOUNCE_S)   # debounce
        if GPIO.input(self._pin) != GPIO.LOW:
            # Bounced back up before debounce settled -- treat as noise,
            # not a real press, and keep waiting.
            return self._wait_for_press_pi()
        print("Start button pressed. Go!")
        logger.info("Start button pressed -- beginning round")

    def _wait_for_press_pc(self):
        print("[PC MODE] No physical start button -- press Enter to start.")
        input()
        logger.info("[PC MODE] Start (Enter) pressed -- beginning round")

    def cleanup(self):
        if IS_PI and self._gpio is not None:
            self._gpio.cleanup(self._pin)
