# Changes — IMU / section-relative world model

## Why not a global x,y coordinate world model

There are no wheel encoders on this vehicle — only an MPU6050 IMU (gyro +
accel) and the camera. Without a distance-traveled signal, a global
"distance from origin to each obstacle" model has no reliable way to
measure how far the car has actually moved (wheel slip, battery-voltage
speed variance, and surface friction all corrupt any time-based or
duty-cycle-based distance estimate). Building a global coordinate map on
top of that would silently drift every lap and eventually be *worse* than
just trusting the camera every frame.

What's implemented instead is a **section-relative world model**:
`SectionDetector`'s 8 known track sections act as the coarse map (a plain
dict lookup by section id — `PillarMemory.recall(section)` is O(1), not a
history scan), and the IMU supplies short-horizon *relative* heading
within the current section only, reset to zero at every line-crossing so
gyro drift never compounds past one section's transit time.

## New: `imu_tracker.py`

- `ImuTracker` (real MPU6050 over I2C, Pi-only) / `MockImuTracker` (PC dev,
  always zeros) — same `IS_PI`-branching pattern as `camera.py` and
  `mock_motor_controller.py`. Use `make_imu_tracker()` from `main.py`
  rather than constructing either class directly.
- Gyro bias is calibrated once at startup (`start()`, 200 samples, vehicle
  MUST be stationary) — removes constant sensor offset before any driving
  begins.
- `update()` — call once per loop iteration, integrates gyro-Z into
  `heading_deg` (relative to the last reset).
- `reset_heading()` — called on every section transition in `main.py` so
  drift never accumulates across more than one section.
- `yaw_rate_dps` — instantaneous signed yaw rate, fed into the Kalman
  filter (see below).

## Changed: `kalman_filter.py`

- `update()` now accepts `yaw_rate_dps` and `dt`, and calls
  `_compensate_rotation()` before the motion-model predict step — this
  rotates the tracked `(vx, vy)` velocity vector by the car's own yaw
  delta each frame, so a missed/occluded detection during a turn predicts
  a physically plausible pillar position instead of extrapolating in a
  straight line through a curve.
- Also fixes a latent bug: `main.py` previously only called
  `kalman.update(detection)` when a detection existed, meaning
  `update(None)` — and therefore the whole missed-frame/forget-after-10
  path — was dead code. `main.py` now calls `update()` unconditionally
  every processed frame.
- `_compensate_rotation()` is a direct velocity-vector rotation, not a
  full reprojection (would need pillar depth + focal length to be exact).
  Flagged `TUNE_ME` in-code: sign/magnitude should be checked against real
  corner footage once the car exists.

## Changed: `pillar_memory.py`

- Each section's first lap-1 detection is now tagged with
  `heading_deg` (IMU heading since that section's last reset).
- `recall(section, heading_deg=None)` — if the current heading deviates
  from the recorded lap-1 entry heading by more than
  `HEADING_TOLERANCE_DEG` (20°), recall is suppressed for that section.
  This catches a missed/duplicate line-crossing putting section counting
  out of sync with the real track, so stale memory isn't trusted at
  higher speed.
- New `confidence(section)` — returns `[0.0, 1.0]`, how many of the
  `CONFIDENCE_TARGET` (5) lap-1 samples were actually collected for that
  section. Used by `SpeedController` instead of assuming every section is
  equally trustworthy just because the lap number went up.

## Changed: `speed_controller.py`

- Replaced the flat per-lap multiplier (which boosted the *entire* lap
  2/3 uniformly regardless of whether memory was any good) with a
  **confidence-gated** boost: `compute(..., section_confidence=...)`
  scales the lap's max boost (1.2x lap 2, 1.4x lap 3) linearly by that
  section's `PillarMemory.confidence()`. A section with no lap-1 data
  gets no boost even on lap 3; a fully-confident section gets the full
  boost as soon as it's reached on lap 2. Lap 1 is unaffected — always
  full reactive speed, never artificially slowed.
- All existing safety slowdowns (wall error, pillar-near, corner-near,
  wall-ahead) still apply multiplicatively on top of the boost.

## Changed: `main.py`

- Instantiates and starts `imu.start()` (gyro calibration — vehicle must
  be stationary at this point) alongside camera/motor start.
- Calls `imu.update()` every frame (cheap single I2C read).
- Calls `imu.reset_heading()` on every section transition.
- Passes `yaw_rate_dps`/`dt` into both Kalman filters, `heading_deg` into
  `PillarMemory.update()`/`recall()`, and `section_confidence` into
  `SpeedController.compute()`.
- `imu.stop()` added to the shutdown path.

## Verified (synthetic/mocked, no car yet)

- `imu_tracker.py` PC-mode mock: zeros returned, no crash.
- `kalman_filter.py`: init, normal update-with-yaw, and blind prediction
  through missed frames during simulated high yaw rate all produce
  sane, non-crashing position estimates.
- `pillar_memory.py`: confidence climbs correctly with repeated lap-1
  detections; `recall()` correctly passes on-heading-match and correctly
  suppresses on-heading-mismatch (12° vs 20° tolerance: passes; 90° vs
  20°: suppressed).
- `speed_controller.py`: lap 2 with zero section confidence stays at base
  speed (no blind boost); lap 2 at full confidence hits the full 1.2x;
  lap 3 at full confidence hits 1.4x; corner-near safety slowdown still
  applies on top regardless of confidence.
- Full synthetic 3-lap loop (all modules wired together, no real
  camera/motors): runs clean, speed increases lap-over-lap as memory
  confidence accumulates, no exceptions.

## Still open — needs real hardware

- Rotation-compensation sign/magnitude in `kalman_filter.py`
  (`TUNE_ME`) — needs real corner footage to verify direction.
- `HEADING_TOLERANCE_DEG` (20°) — reasonable starting guess, needs
  tuning against real line-crossing timing jitter.
- Gyro bias calibration currently assumes the vehicle is perfectly still
  during `imu.start()` — no operator confirmation step; worth adding a
  "press enter when stationary" style gate before competition use.
- `CONFIDENCE_TARGET=5` assumes a pillar stays in frame for several
  consecutive processed frames while the car transits a section during
  lap 1 — reasonable at ~30fps/2 (processed every other frame) but not
  yet checked against real transit speed once the car exists.
