# Ackermann Steering — Why and How

## Why Ackermann steering
When a car turns, the front-left and front-right wheels trace circles of
different radii around the same turn center — the inner wheel's circle is
tighter than the outer wheel's. If both front wheels were forced to the same
steering angle (parallel steering, e.g. a single tie rod straight across),
at least one wheel is always dragged sideways instead of rolling freely.
That scrub:
- wastes drive torque and battery on friction instead of forward motion,
- wears the tires and adds unpredictable drag, and
- makes the car's turning radius less consistent — a problem for holding a
  line in the obstacle-round corridor widths and for repeatable parking.

Ackermann geometry fixes this by linking the two wheels so the **inner wheel
turns at a sharper angle than the outer wheel** whenever the car turns, so
both wheels roll along their own correctly-sized circle with (ideally) no
side-slip. It's the same geometry full-size cars use, scaled down.

Rule 11.3 permits a single steering actuator on the steering axle, so a
single servo is compliant — what we changed is the **linkage geometry**
between that one servo and the two front wheels, not the actuator count.

## How we implemented it
- **Servo horn → central steering (Pitman) arm**: the servo rotates a short
  arm at the center of the front axle.
- **Two tie rods**: the central arm connects via left and right tie rods to
  each front wheel's steering knuckle.
- **Steering knuckles on kingpins**: each front wheel pivots on its own
  kingpin/bearing, independent of the other wheel — this is what allows the
  two wheels to reach different angles from one input.
- **The Ackermann angle** comes from the *geometry*, not the servo: the tie
  rod pivot points on the knuckles are set inward (toward the rear axle
  centerline) rather than straight out from the wheel, so as the arm
  rotates, the inner-side tie rod pushes its knuckle through a larger angle
  than the outer-side tie rod. Getting this right means the tie-rod pivot
  points, the central arm length, and the wheelbase/track width all have to
  be sized together.
- This whole linkage was CAD'd and is 3D printed — see `docs/stl_files/`
  for the part files (steering knuckles, bell-crank/central arm, tie-rod
  ends).

## Current status
The linkage is designed and printed; the exact dimensions (arm length,
tie-rod length, and the resulting min turning radius and left/right servo
endpoints) are still being finalised on the physical chassis. Until that
measurement pass, `SERVO_CENTER_DEG` / `SERVO_LEFT_DEG` / `SERVO_RIGHT_DEG`
in `arduino/motor_controller/motor_controller.ino` remain a standard ±30°
placeholder — see the pre-competition checklist in the main `README.md`.
