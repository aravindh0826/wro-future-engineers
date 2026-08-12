# Changes: DC+L298N -> stepper+DRV8825 drive motor, WRO start-button gate

## Motivation
Physical chassis swap: rear drive motor changed from a DC gearmotor +
L298N driver to a NEMA-14 stepper (STH-39D219) driven by a DRV8825,
coupled to the rear axle through a GT2 pulley/belt reduction (20T motor
pulley, 60T axle pulley, 202mm belt -> 3:1 reduction). Also added the
WRO-required "waiting state + start button" step, which the codebase
didn't have before (main.py previously started driving immediately on
process launch).

## Rules caveat -- read before competition
The rulebook's motor clause is worded "Teams can use any electrical DC
motors and/or servo motors of their choice." It does not explicitly say
"any motor type," so a stepper drive axle is not unambiguously covered
by the letter of that clause, even though steppers are commonly used in
practice. **Confirm with your organizer/judges before relying on this at
competition.** See the caveat block at the top of
`arduino/motor_controller/motor_controller.ino`.

## Why the Python side (Pi) needed almost no changes
`arduino_motor_controller.py` and everything upstream of it
(`main.py`, `speed_controller.py`, `wall_follower.py`,
`ultrasonic_parking_controller.py`, `parking_controller.py`, `pid.py`,
`sign_grid.py`, `pillar_memory.py`, corridor/distance detection in
`cv_pipeline.py`) only ever deal in abstract `mm/s` speed and `[-1,1]`
steering setpoints sent over the same `C,<steer>,<speed>\n` serial
protocol as before. The motor-type-specific math (mm/s -> step rate)
lives entirely in the `.ino` file. This is exactly why the earlier
Pi/Arduino split (see `CHANGES_ARDUINO_OFFLOAD.md`) was worth doing --
swapping the physical drive motor turned into a firmware-only change.

## Changed
- `arduino/motor_controller/motor_controller.ino`:
  - `MOTOR_IN1`/`MOTOR_IN2`/`MOTOR_EN` (L298N) -> `STEPPER_STEP`/
    `STEPPER_DIR`/`STEPPER_EN` (DRV8825), same pin numbers (5/6/7)
    reused where sensible to minimise rewiring.
  - `applySpeed()` no longer computes a PWM duty cycle. It now sets
    `STEPPER_DIR` for direction and computes a STEP pulse interval via
    the new `speedToStepIntervalUs()` (mm/s -> wheel rev/s -> motor
    rev/s via `GEAR_RATIO` -> steps/s via `STEPS_PER_REV`*`MICROSTEPPING`).
  - New `updateStepper()`, called every `loop()` iteration, emits STEP
    pulses via non-blocking `micros()` comparison (NOT a hardware timer
    -- Timer1 is already owned by the `Servo` library on an ATmega328,
    so a competing Timer1-based stepper ISR would have broken steering).
    Verified in Python (mirroring the same formula) that the required
    step rate stays under 1.2kHz across the full 100-400mm/s
    `speed_controller.py` range, comfortably under the new
    `MAX_STEP_RATE_HZ` (4000) safety cap.
  - `MAX_SPEED_MM` (L298N duty-cycle reference) removed, replaced by
    `WHEEL_DIAMETER_MM` + `GEAR_RATIO` + `STEPS_PER_REV` +
    `MICROSTEPPING` -- all placeholders except `GEAR_RATIO` (known from
    your 20T/60T pulleys), same "measure on the real car" status as
    `SERVO_CENTER_DEG`/etc. always had.
  - Servo section unchanged in logic; comment updated to note the
    +/-30deg range is a deliberate standard placeholder (per your
    instruction that Ackermann geometry isn't finalised yet), not a
    finished calibration.
- `src/main.py`: added the WRO starting-procedure "waiting state" --
  after camera/motor/IMU init, the vehicle now blocks on
  `start_switch.wait_for_press()` before entering the driving loop.
  Nothing else in the driving/parking loop changed.
- `docs/ARDUINO_OFFLOAD.md`, `README.md`: updated to describe the
  stepper drivetrain and the start button instead of L298N/no-button.
- `requirements.txt`: added `RPi.GPIO` (only imported on real Pi
  hardware, same convention as `smbus2`/`pyserial`).

## Added
- `src/start_switch.py`: implements the two-part WRO starting
  procedure --
  1. Power switch: hardware-only, in series with the battery, not
     code. Documented in this file's docstring so it isn't confused
     with the button below.
  2. Start button: `StartSwitch.wait_for_press()`, a Pi GPIO26
     (BCM, pull-up, active-low) momentary push-button. `main.py` blocks
     on this after all init and before driving starts. PC/mock mode
     waits for Enter instead, so PC testing isn't blocked forever.

## Not changed
- `speed_controller.py`, `wall_follower.py`, `sign_grid.py`,
  `pillar_memory.py`, `cv_pipeline.py`, corridor-width detection
  (`config/corridor_calibration.json`,
  `wall_follower.lateral_offset_mm()`), distance detection
  (`kalman_filter.py`, focal-length math in `cv_pipeline.py`/
  `config/camera_calibration.json`) -- all of this is camera geometry
  and vision logic with no coupling to drivetrain type. Verified by
  inspection that none of these files reference motor/servo specifics.
- `arduino_motor_controller.py` (Pi-side serial client) -- protocol and
  public interface identical, no code changes needed.
- `src/motor_controller.py` (old direct-GPIO L298N fallback) -- left
  as-is, already unused by default per `CHANGES_ARDUINO_OFFLOAD.md`.

## Still open -- must be measured on the real vehicle, not assumed
Same status as everything already flagged elsewhere in this project:
- `WHEEL_DIAMETER_MM` in the `.ino` -- placeholder (64mm), directly
  scales every mm/s target. Wrong value = car drives at the wrong real
  speed even though the Pi-side number looks right.
- `STEPS_PER_REV` -- assumed 200 (1.8deg/step) for the STH-39D219;
  confirm against the datasheet/nameplate, a few STH-39D variants are
  0.9deg/step (400 steps/rev).
- `MICROSTEPPING` -- assumed 1 (full step, DRV8825 MS1-3 default). If
  you wire MS1-3 for microstepping, this constant must match or the
  car's real speed will be off by that factor.
- DIR polarity (`applySpeed()`'s `HIGH`/`LOW` for forward) -- may need
  swapping depending on motor wiring orientation; there's no way to know
  this without spinning the real motor.
- `MAX_STEP_RATE_HZ` (4000, conservative) -- lower it if the motor
  stalls/skips audibly at `MAX_SPEED_MM`-equivalent speeds on the bench;
  a stepper has no encoder here to detect a stall automatically.
- `START_BUTTON_PIN` (GPIO26 BCM) in `start_switch.py` -- change to
  match wherever you actually wire the push-button.
- Servo `SERVO_CENTER_DEG`/`LEFT`/`RIGHT`, ultrasonic mounting -- same
  unresolved status as before, unrelated to this change.
