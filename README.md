# WRO 2026 Future Engineers — Team Titan Grandmasters

## Team Introduction
- **Team Name:** Titan Grandmasters
- **Team Number:** 1091
- **Country:** India
- **Team Leader:** Aravindh Balaji
- **Members:** Abdul Hakeem K, Balaji Keerthi
- **Coach:** Saurav Kumar Topo
- **Season:** WRO 2026 Future Engineers

---

## Documentation Index
This README is the top-level reference. Deeper detail lives in `docs/`:
- **`docs/ACKERMANN_STEERING.md`** — why we use Ackermann steering and how the linkage works
- **`docs/COMPONENTS.md`** — every major component and why it was chosen
- **`docs/ARDUINO_OFFLOAD.md`** — Pi/Arduino split architecture and serial protocol
- **`docs/stl_files/`** — 3D-printed chassis STL/CAD files (see status below)
- **`docs/vehicle_photos/`** — required vehicle photos
- **`CHANGES_*.md`** (repo root) — dated technical change logs for each major redesign

---

## Vehicle Description
Our autonomous vehicle uses a Raspberry Pi 4B (4GB) as the main controller, paired with a Pi Camera Module Rev 1.3 (OV5647) for vision. Motor/servo PWM/step generation is offloaded to an Arduino Uno over a wired serial link (Rules 11.8/11.9 permit multiple SBC/SBM; Rules 11.10/11.17 require the link to be a physical wire). The vehicle is rear-wheel drive with a single drive motor on a shared shaft (Rule 11.13, single driving axle) and front-only Ackermann steering via a single servo (Rule 11.3, single steering actuator). A single physical power switch (battery -> SBC/SBM) plus a single physical start button on the Pi gate when the vehicle powers on vs. when it actually starts driving (rules ~9.8-9.14) — see `src/start_switch.py`.

**Rules note on the drive motor:** the rulebook's motor clause is worded "any electrical DC motors and/or servo motors" and doesn't explicitly name steppers. Confirm a stepper-driven axle is acceptable with your organizer/judges before competition — see the caveat at the top of `arduino/motor_controller/motor_controller.ino`.

### Hardware
| Component | Specification |
|-----------|--------------|
| Main controller (SBC) | Raspberry Pi 4B (4GB) |
| Secondary controller (SBM) | Arduino Uno — owns servo PWM + stepper pulses, wired serial to Pi |
| Camera | Pi Camera Module Rev 1.3 (OV5647) |
| Drive Motor | STH-39D219 NEMA-14 stepper + DRV8825 driver, rear axle via GT2 pulley/belt (20T motor / 60T axle, 202mm belt, 3:1 reduction) |
| Steering | Servo, front axle, Ackermann geometry (single actuator) |
| IMU | MPU6050 |
| Parking sensors | 2x HC-SR04 ultrasonic (rear-left/rear-right corners) — see `docs/ARDUINO_OFFLOAD.md` and `src/ultrasonic_parking_controller.py` |
| Start button | Momentary push-button, Pi GPIO26 (BCM) — `src/start_switch.py` |
| Power | 7.4V 2S LiPo battery, single in-line power switch |

Note: this vehicle has one drive motor on one driving axle (rear) and one steering actuator on one steering axle (front) — it is NOT differential drive.

See `docs/COMPONENTS.md` for why each component above was chosen, and
`docs/ACKERMANN_STEERING.md` for the steering linkage design.

### 3D-Printed Chassis (STL Files)
The chassis has been redesigned with a **better part orientation** for
printing and assembly. Parts are currently being printed and will all be
mounted and verified on the vehicle before competition. STL files (and
source CAD, where kept) go in **`docs/stl_files/`** as each part is
finalized — see that folder's README for the file list and checklist.

---

## Software Architecture

### CV Pipeline
Our vehicle uses a pure computer vision pipeline — no machine learning. This was a deliberate choice for reliability and speed.

```
Camera Frame
    ↓
CLAHE Preprocessing (lighting normalisation)
    ↓
HSV Color Masking
    ↓
Morphological Noise Removal
    ↓
Contour Detection + Shape Filtering
    ↓
Kalman Filter (smooth tracking)
    ↓
Pillar Memory Map (faster laps 2 & 3)
    ↓
Wall Centering PID + Pillar Avoidance
    ↓
Adaptive Speed Control
    ↓
Motor/Servo Output
```

### Key Modules
- **camera.py** — Threaded frame capture, works on Pi (picamera2) and PC (webcam/video)
- **cv_pipeline.py** — Full color detection, contour analysis, wall/pillar/parking-marker detection. Loads HSV ranges from `config/hsv_values.json` and focal length / lens calibration from `config/camera_calibration.json` if present.
- **kalman_filter.py** — Smooth pillar tracking across frames, handles brief occlusions
- **pillar_memory.py** — Stores lap 1 pillar positions, enables faster laps 2 & 3
- **wall_follower.py** — Keeps car centered in corridor; mode-aware (Obstacle Challenge corridor is fixed 1000mm, Open Challenge varies 1000/600mm)
- **section_detector.py** — Detects orange/blue lines for section and lap counting; driving direction (CW/CCW) is passed in per round, not inferred
- **speed_controller.py** — Adaptive speed based on surroundings
- **ultrasonic_parking_controller.py** — Vision-triggered parallel-parking state machine, triggered after lap 3. Uses the magenta parking markers (camera) to find and enter the lot, then switches to two rear ultrasonic sensors for the blind-spot reverse/align (the forward camera can't see the markers once the car is mid-reverse). Falls back to vision-only open-loop behaviour if ultrasonic data isn't available.
- **parking_controller.py** — Original vision-only parking state machine. Superseded by `ultrasonic_parking_controller.py` above; kept as a fallback reference in case the ultrasonic sensors aren't mounted/wired in time.
- **arduino_motor_controller.py** — Pi-side serial client to the Arduino (drop-in for `motor_controller.MotorController`). Sends `C,<steer>,<speed>` setpoints and reads back `U,<left_mm>,<right_mm>` ultrasonic telemetry.
- **pid.py** — Generic PID with anti-windup and derivative filtering
- **start_switch.py** — Pi-side WRO start-button gate; blocks after init until the physical start button is pressed, before any driving begins
- **focal_calibrator.py** — Empirically measures `focal_length_px` against a real pillar at a known distance
- **lens_calibrator.py** — Optional checkerboard lens-distortion calibration

### Color Detection
All colors detected using HSV color space with CLAHE preprocessing for lighting robustness:
| Color | Purpose |
|-------|---------|
| Red | Traffic pillar — pass on right |
| Green | Traffic pillar — pass on left |
| Orange | Corner section lines |
| Blue | Straight section lines |
| Black | Outer/inner walls |

---

## How To Run

### On PC (development)
```bash
pip install -r requirements.txt
cd src
python debug_visualizer.py              # webcam
python debug_visualizer.py --source video.mp4  # video file
```

### HSV Calibration
```bash
cd src
python hsv_calibrator.py --color red1
python hsv_calibrator.py --color green
python hsv_calibrator.py --color orange
python hsv_calibrator.py --color magenta
```
Saves to `config/hsv_values.json`, which `cv_pipeline.py` loads automatically.

### Camera Calibration
```bash
cd src
python focal_calibrator.py --distance 300 --color red   # empirical focal length
python lens_calibrator.py                                 # optional: lens distortion
```
Saves to `config/camera_calibration.json`, loaded automatically by `cv_pipeline.py`.

### On Raspberry Pi (competition)
```bash
pip install -r requirements.txt
pip install picamera2
cd src
python main.py
```
Before each round, set `CHALLENGE_MODE` ("obstacle"/"open") and `DIRECTION` ("CW"/"CCW") at the top of `main.py` to match that round's announced configuration (rules 9.3-9.8) — these are not detected automatically.

---

## Repository Structure
```
wro2025-future-engineers/
├── src/
│   ├── main.py                 # Entry point
│   ├── camera.py               # Camera capture (Pi + PC)
│   ├── cv_pipeline.py          # Full CV detection pipeline
│   ├── kalman_filter.py        # Pillar tracking
│   ├── pillar_memory.py        # Lap 1 map for faster laps
│   ├── wall_follower.py        # Wall centering
│   ├── section_detector.py     # Lap and section counting
│   ├── speed_controller.py     # Adaptive speed
│   ├── ultrasonic_parking_controller.py  # Vision + rear-ultrasonic parallel-parking state machine
│   ├── parking_controller.py   # Vision-only fallback parking state machine
│   ├── pid.py                  # PID controller
│   ├── motor_controller.py     # Pi GPIO motors (fallback, not used with Arduino offload)
│   ├── arduino_motor_controller.py # Pi-side serial client to the Arduino (used on the real vehicle)
│   ├── mock_motor_controller.py # PC development mock
│   ├── start_switch.py         # WRO start-button waiting-state gate
│   ├── hsv_calibrator.py       # Live HSV tuning tool
│   ├── focal_calibrator.py     # Empirical focal length calibration
│   ├── lens_calibrator.py      # Optional lens distortion calibration
│   └── debug_visualizer.py     # Full debug window for PC
├── arduino/
│   └── motor_controller/motor_controller.ino  # Arduino sketch: servo PWM + DRV8825 stepper pulses + rear ultrasonic telemetry
├── config/
│   ├── hsv_values.json         # Saved HSV calibration
│   └── camera_calibration.json # Focal length + lens distortion
├── docs/
│   ├── ARDUINO_OFFLOAD.md      # Pi/Arduino split architecture + protocol
│   ├── ACKERMANN_STEERING.md   # Why + how of the Ackermann linkage
│   ├── COMPONENTS.md           # Component choices and rationale
│   ├── stl_files/              # 3D-printed chassis STL/CAD files
│   └── vehicle_photos/
├── logs/
├── video/
├── requirements.txt
└── README.md
```

---

## Design Journey — Where We Started, Where We Are Now

This section is a running log of the vehicle's mechanical/electrical
evolution, kept deliberately separate from the technical reference
above so a judge (or a future version of us) can see the reasoning, not
just the current end state. Nothing above this section has been
rewritten to pretend we always had the current design — this is the
actual order things happened in.

### 1. Starting point
- Drive: DC gearmotor (37GB-520 class) on the rear axle, driven through
  an L298N H-bridge, speed set by PWM duty cycle.
- Steering: a single hobby servo on the front axle. Early builds didn't
  have a formally designed Ackermann linkage — steering geometry was
  effectively whatever the stock kit chassis provided.
- Compute: Raspberry Pi 4B running the full CV/control pipeline
  directly, then later split so the Pi handles vision/decision-making
  and an Arduino Uno owns real-time servo/motor PWM + ultrasonic
  polling over a wired serial link (see `CHANGES_ARDUINO_OFFLOAD.md`)
  — this split is why later hardware swaps (like the one below) only
  ever touch the Arduino sketch, not the Pi-side vision/control code.

### 2. Chassis redesign — proper Ackermann steering
We moved off the stock kit steering to a purpose-designed Ackermann
linkage (CAD'd and 3D printed): a steering rack/bell-crank driven by
the servo, connecting to tie rods on both front knuckles so the two
front wheels turn at correctly different angles in a turn (inner wheel
turns tighter than the outer one), rather than both wheels being forced
to the same angle. Mechanically this is:
- Servo horn → central steering arm → left and right tie rods → each
  front wheel's steering knuckle, with the knuckles pivoting on their
  own kingpin/bearing.
- The geometry (arm lengths, tie-rod pivot points) is what makes it a
  *true* Ackermann linkage instead of parallel steering — it's the same
  reason full-size cars don't turn both front wheels to the same angle.
- This satisfies Rule 11.3 (single steering actuator, front axle) more
  robustly than the stock geometry did, and should reduce front-tire
  scrub in tight turns, which matters for the obstacle-round corridor
  widths.
- The exact linkage dimensions (arm lengths, tie-rod length, resulting
  min turning radius) are still being finalised on the physical
  chassis, so `SERVO_CENTER_DEG`/`SERVO_LEFT_DEG`/`SERVO_RIGHT_DEG` in
  the Arduino sketch remain a standard +/-30° placeholder until that's
  measured — see the "Still open" list below.

### 3. Drive motor redesign — DC motor → stepper
The rear drive motor changed from the DC gearmotor + L298N to a
NEMA-14 stepper (STH-39D219) driven by a DRV8825, coupled to the rear
axle through a GT2 pulley/belt reduction (20-tooth motor pulley,
60-tooth axle pulley, 202mm belt → 3:1 reduction). Reasoning: more
predictable, repeatable low-speed behaviour than a small brushed DC
gearmotor, at the cost of losing torque headroom at high step rates
(steppers have no closed-loop feedback here, so there's no automatic
stall detection — see the open items below).

**Rules caveat carried over from the firmware changes:** the rulebook's
motor clause says "any electrical DC motors and/or servo motors,"
without explicitly naming steppers. We're treating this as a real
compliance question, not a formality — see `CHANGES_STEPPER_DRIVE.md`
and the caveat block at the top of `motor_controller.ino`. Confirming
this with organizers/judges is on the pre-competition checklist below.

### 4. Software/firmware updates made for the above (this repo, current state)
- `motor_controller.ino`: L298N PWM drive logic replaced with DRV8825
  STEP/DIR/EN stepper control — non-blocking pulse generation via
  `micros()` (not a hardware timer, since the Servo library already
  owns Timer1 on the Uno). New `speedToStepIntervalUs()` converts the
  same mm/s setpoint the Pi always sent into a step rate using the
  pulley ratio above.
- `main.py` / new `start_switch.py`: added the WRO starting-procedure
  "waiting state" — vehicle powers on (single physical switch,
  hardware only), initialises camera/motors/IMU, then blocks on a
  physical start-button press before driving begins. This didn't exist
  before; `main.py` previously started driving immediately on launch.
- `README.md`, `docs/ARDUINO_OFFLOAD.md`, `requirements.txt`: updated
  to describe the current hardware instead of the original DC/L298N
  setup.
- **What deliberately did not change:** `speed_controller.py`,
  `wall_follower.py`, corridor-width detection, distance/Kalman
  tracking, parking logic. All of this operates on abstract mm/s and
  camera geometry, with no coupling to which motor is spinning the
  axle — confirmed by inspection when the stepper swap was made. This
  is the payoff of the earlier Pi/Arduino split: a full drive-motor
  technology change ended up being a firmware-only change.

Full technical diff: `CHANGES_STEPPER_DRIVE.md`.

### 5. Pre-competition checklist — still open, must be verified on the real vehicle
None of these are guesses we're comfortable trusting untested; each one
directly affects whether the car drives at the right speed, turns the
right amount, or starts correctly at competition.
- [ ] **Confirm stepper-as-drive-motor is rules-compliant** with
      organizers/judges (see Rules caveat above). Highest priority —
      affects whether the whole drivetrain redesign is usable as-is.
- [ ] **Finalise Ackermann linkage geometry** on the physical chassis
      and update `SERVO_CENTER_DEG`/`SERVO_LEFT_DEG`/`SERVO_RIGHT_DEG`
      in `motor_controller.ino` from the placeholder ±30°.
- [ ] **Measure real drive wheel diameter** and update
      `WHEEL_DIAMETER_MM` in `motor_controller.ino` (currently a 64mm
      placeholder) — this scales every mm/s speed command.
- [ ] **Confirm STH-39D219 steps/rev** against its datasheet/nameplate
      (assumed 200, i.e. 1.8°/step — some STH-39D variants are 400,
      i.e. 0.9°/step) and `MICROSTEPPING` matches however DRV8825
      MS1/MS2/MS3 end up wired.
- [ ] **Bench-test the stepper alone** (car on blocks): confirm `DIR`
      polarity (does "forward" actually spin the wheels forward?) and
      listen for stalling/skipped steps at target top speed — there's
      no encoder feedback to catch a stall automatically.
- [ ] **Wire and test the physical start button** at whatever GPIO pin
      it ends up on (placeholder: GPIO26 BCM in `start_switch.py`) and
      confirm the single in-line power switch is wired correctly
      (battery → SBC/SBM, one switch only, per the WRO starting
      procedure).
- [ ] **Camera focal length / corridor calibration** (`focal_calibrator.py`,
      `corridor_calibrator.py`) — still needs to be run against the
      camera as actually mounted on the redesigned chassis; height/tilt
      changes from the redesign could shift these from their current
      values.
- [ ] **Full three-lap + parking run** on the real track, on the final
      hardware — not yet done end-to-end since these changes were made.
- [ ] **Finish printing and mount all re-oriented chassis parts**, add the
      STL/CAD files to `docs/stl_files/`, and re-test-fit the Ackermann
      linkage and motor mount on the physical chassis.