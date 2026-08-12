# Changes: Pi + Arduino split (motor/servo offload)

## Motivation
Reduce Raspberry Pi 4B (4GB) CPU load and actuation latency/jitter by
moving real-time PWM generation (steering servo + L298N drive motor) off
the Pi and onto an Arduino, communicating over wired serial.

## Rule basis
- Rule 11.8/11.9: multiple SBC/SBM explicitly permitted, no brand restriction.
- Rule 11.10/11.17: Pi<->Arduino link must be a physical wire, never wireless.
- Rule 11.6: watchdog on the Arduino side ensures a dropped link fails to a
  safe stop, keeping the vehicle's autonomy compliant even during a comms fault.
- Rule 12.6: both controllers must be powered off at vehicle check, not just one.

## Added
- `arduino/motor_controller/motor_controller.ino` -- Arduino sketch:
  receives `C,<steer>,<speed>` setpoints over serial, drives servo + L298N
  continuously, 200ms watchdog failsafe.
- `src/arduino_motor_controller.py` -- Pi-side serial client, same public
  interface as the old `motor_controller.MotorController` (drop-in).
- `docs/ARDUINO_OFFLOAD.md` -- architecture, protocol, wiring, rule notes.
- `pyserial>=3.5` added to `requirements.txt`.

## Changed
- `src/main.py`: `IS_PI` branch now imports `ArduinoMotorController` from
  `arduino_motor_controller.py` instead of the direct-GPIO
  `motor_controller.MotorController`. No other changes to `main.py` --
  `pid.py`, `speed_controller.py`, and the rest of the control loop are
  untouched, since steering PID / speed target computation stay on the Pi.

## Fixed (unrelated rule-citation correction, found while editing these files)
`motor_controller.py`, `mock_motor_controller.py`, and
`debug_visualizer.py` cited **Rule 11.13** for single-servo Ackermann
steering. Rule 11.13 is actually about the max-two-driving-motors
constraint, not steering. The correct citation for "one steering actuator
of any type" is **Rule 11.3**. Fixed in all three files.

## Not changed / kept as-is
- `src/motor_controller.py` (original direct-GPIO version) is kept in the
  repo for reference and as a manual fallback -- not used by default.
- Steering PID (`pid.py`) and speed target computation
  (`speed_controller.py`) remain on the Pi; only the final setpoint
  crosses the wire to the Arduino.

## Still open (not resolved by this change)
- Servo angle calibration (`SERVO_CENTER_DEG`/`LEFT`/`RIGHT`) and
  `MAX_SPEED_MM` in the `.ino` sketch are placeholders -- require
  empirical calibration on the physical car, same status as camera
  height/tilt (via `focal_calibrator.py`/`hsv_calibrator.py`) and
  wheelbase/track width.
- Exact Arduino board/pin choice and serial device path
  (`/dev/ttyACM0` vs `/dev/ttyUSB0`) -- pending hardware selection.
