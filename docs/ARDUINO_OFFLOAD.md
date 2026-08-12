# Pi + Arduino split architecture

## Why

The Raspberry Pi 4B (4GB) runs the CV pipeline (`cv_pipeline.py`), Kalman
filtering, pillar memory, sign grid, and the rest of the vision/mission
logic. Direct-GPIO PWM generation for the steering servo and drive motor
(the old `motor_controller.py`) competed with that for CPU and scheduling
time, showing up as actuation jitter and added Pi load.

Motor/servo PWM generation is now offloaded to an Arduino (SBM). The Pi
sends steering + speed **setpoints**; the Arduino owns the actual PWM
output continuously via its own hardware timers, so it doesn't need
babysitting every control-loop iteration the way software PWM on the Pi
did.

## What moved, what stayed

| Stays on Pi (needs vision data) | Moved to Arduino |
|---|---|
| `cv_pipeline.py`, `wall_follower.py`, `sign_grid.py`, `pillar_memory.py`, `section_detector.py`, `ultrasonic_parking_controller.py`, `parking_controller.py`, `kalman_filter.py`, `pid.py`, `speed_controller.py` | Servo PWM (steering), DRV8825 stepper pulses (drive motor), watchdog failsafe, rear-left/rear-right ultrasonic sensor reads |

The steering PID and speed target computation stay on the Pi exactly as
before -- they need the vision-derived wall error. Only the final
`(steering, speed)` setpoint crosses the wire each control-loop
iteration; `main.py` itself is unchanged except for which
`MotorController` class it imports. The Arduino also does its own
ultrasonic reads and reports them back unsolicited -- see Protocol below
-- since Rule 11.17 requires that link to be a physical wire too, and
the two rear sensors are cheap to read from whichever board they're
wired to.

## Protocol

Pi -> Arduino, one ASCII line per setpoint update:

```
C,<steer>,<speed>\n
```

- `steer`: float in `[-1.0, 1.0]`, same convention as
  `steering_value_to_duty()` (-1 = full left, 1 = full right)
- `speed`: signed int, mm/s (positive = forward)

Example: `C,0.35,220\n`

Sent from `arduino_motor_controller.py`'s `set_speed()` (called right
after `set_steering()` every loop iteration in `main.py`, so one write
covers both per iteration).

Arduino -> Pi, unsolicited telemetry (Rule 11.11, any sensor/any number
permitted):

```
U,<left_mm>,<right_mm>\n
```

- rear-left / rear-right ultrasonic distance in mm; `-1` means no echo
  (out of range for that sensor)
- sent at most every `ULTRA_REPORT_MS` (120ms default) once both sensors
  have a fresh reading
- read on the Pi side via `ArduinoMotorController.read_ultrasonic()`,
  used only by `ultrasonic_parking_controller.py` for the blind-spot
  reverse-parking manoeuvre (see that file's docstring for why the
  forward camera alone isn't enough there)

## Watchdog / autonomy compliance

**Rule 11.6** requires the vehicle to be fully autonomous with no
external/remote control while running. If the Pi<->Arduino serial link
drops, the Arduino must not keep coasting on the last command forever --
that would effectively be uncommanded, unsupervised motion. The `.ino`
sketch stops the motor and centers the steering if it hasn't received a
valid command within `WATCHDOG_MS` (default 200ms).

## Wiring / rules this design is built around

- **Rule 11.8**: controller can be SBC or SBM, no restriction on brand.
- **Rule 11.9**: "There could be more than one SBC/SBM on the vehicle" --
  explicit permission for the Pi+Arduino combo.
- **Rule 11.10**: no RF/Bluetooth/WiFi/wireless communication components
  during competition rounds -- the Pi<->Arduino link **must** stay a
  physical wire (USB-serial in this implementation).
- **Rule 11.17**: "Only wire connections are permitted for communication
  between vehicle electromechanical components." Same constraint,
  applies directly to this link.
- **Rule 12.6**: at vehicle check, *all* controllers must be powered off.
  With two boards now on the vehicle, remember both -- not just the Pi.

## Open variables (not fixed by rule, not fixed by this change)

Same status as noted elsewhere in this project -- nothing below should be
assumed, all require empirical calibration on the physical car:

- `SERVO_CENTER_DEG` / `SERVO_LEFT_DEG` / `SERVO_RIGHT_DEG` in the `.ino`
  sketch -- depends on physical servo horn mounting/travel.
- `MAX_SPEED_MM` in both the `.ino` sketch and
  `arduino_motor_controller.py` -- depends on motor/gearing/wheel
  diameter, keep these two in sync if you recalibrate.
- Which Arduino board/pins you actually use, and the exact serial device
  path (`/dev/ttyACM0` vs `/dev/ttyUSB0`) -- depends on which Arduino and
  USB-serial chip you wire up.
- Wire gauge/connector choice for the Pi<->Arduino link and for
  DRV8825/servo power -- pending physical assembly, same as battery/Pi
  positioning already flagged as open.
- `WHEEL_DIAMETER_MM` in the `.ino` sketch -- the drive motor is now a
  stepper (STH-39D219) + DRV8825 + GT2 pulley/belt, see
  `CHANGES_STEPPER_DRIVE.md` and the STEPPER DRIVE MATH block at the top
  of the `.ino` file for the full mm/s -> step-rate conversion. This
  replaces `MAX_SPEED_MM`/duty-cycle math the L298N version used.
- Rear ultrasonic sensor mounting position/angle (`ULTRA_LEFT_TRIG` /
  `ULTRA_LEFT_ECHO` / `ULTRA_RIGHT_TRIG` / `ULTRA_RIGHT_ECHO` pins in the
  `.ino` sketch, and `CONTACT_STOP_DISTANCE_MM` /
  `CONTACT_EASE_DISTANCE_MM` / `PARALLEL_TOLERANCE_MM` in
  `ultrasonic_parking_controller.py`) -- entirely unverified until the
  sensors are physically bolted to the rear corners and read against a
  real wall at a measured distance.

## Files

- `arduino/motor_controller/motor_controller.ino` -- Arduino-side sketch
  (servo/motor PWM + rear ultrasonic reads/telemetry)
- `src/arduino_motor_controller.py` -- Pi-side serial client (drop-in
  replacement for `motor_controller.MotorController`), also exposes
  `read_ultrasonic()`
- `src/motor_controller.py` -- kept in the repo as the original
  direct-GPIO version, for reference/fallback. Not used by default now;
  `main.py` imports `arduino_motor_controller` when `IS_PI`.
- `src/ultrasonic_parking_controller.py` -- the parking state machine
  that consumes `read_ultrasonic()`; see its docstring for the full
  vision+ultrasonic design rationale.
