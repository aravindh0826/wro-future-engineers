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
This README is the top-level reference. It documents the current system, the final mechanical revision, and the team's design journey. Deeper detail lives in `docs/`:
- **`docs/ACKERMANN_STEERING.md`** — why we use Ackermann steering and how the linkage works
- **`docs/COMPONENTS.md`** — every major component and why it was chosen
- **`docs/ARDUINO_OFFLOAD.md`** — Pi/Arduino split architecture and serial protocol
- **`docs/stl_files/`** — 3D-printed chassis STL/CAD files (see status below)
- **`docs/vehicle_photos/`** — required vehicle photos
- **`CHANGES_*.md`** (repo root) — dated technical change logs for each major redesign

---

## Current Competition Status — Final Chassis Update

> **Important build-status note:** During the final stage of development, we identified an opportunity to improve the chassis design and made a last-minute mechanical redesign rather than locking in a version that could be improved. The revised chassis has a better part orientation for fabrication and assembly, and the updated parts are **currently being 3D printed**.
>
> The finalized chassis parts are expected to be **available and assembled for the final competition**. Because this redesign is being completed under the current time constraints, the repository intentionally documents the finalized CAD/STL design and the latest physical chassis state while the new parts are being manufactured and integrated.
>
> **Chassis files:** `docs/stl_files/`  
> **Current chassis photographs:** `docs/vehicle_photos/`
>
> This is a deliberate engineering decision: there is always room for further improvement, and we chose to implement a meaningful mechanical improvement when it was identified rather than preserve an older design solely for documentation timing.

### Working Video Status

The working video is **temporarily not included in this repository** because the vehicle is currently undergoing the final chassis modification, 3D printing, assembly, and re-verification. A video recorded before these changes would represent an earlier physical configuration and could therefore be misleading as evidence of the final vehicle.

The software architecture and control pipeline remain documented in this repository, while the physical vehicle is being brought to the updated final configuration. The priority at this stage is to complete the improved chassis, mount and verify the components, recalibrate where required, and perform the final end-to-end validation before competition.

**Documentation available now:**
- Updated chassis STL/CAD files: `docs/stl_files/`
- Current physical chassis photographs: `docs/vehicle_photos/`
- Software and control architecture: `src/`
- Hardware/architecture documentation: `docs/`
- Technical redesign history: `CHANGES_*.md`

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

### 3D-Printed Chassis — Final Mechanical Revision
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
python debug_visualizer.py              # webcam
python debug_visualizer.py --source video.mp4  # video file
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
python focal_calibrator.py --distance 300 --color red   # empirical focal length
python lens_calibrator.py                                 # optional: lens distortion
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
│   ├── main.py                 # Entry point
│   ├── camera.py               # Camera capture (Pi + PC)
│   ├── cv_pipeline.py          # Full CV detection pipeline
│   ├── kalman_filter.py        # Pillar tracking
│   ├── pillar_memory.py        # Lap 1 map for faster laps
│   ├── wall_follower.py        # Wall centering
│   ├── section_detector.py     # Lap and section counting
│   ├── speed_controller.py     # Adaptive speed
│   ├── ultrasonic_parking_controller.py  # Vision + rear-ultrasonic parallel-parking state machine
│   ├── parking_controller.py   # Vision-only fallback parking state machine
│   ├── pid.py                  # PID controller
│   ├── motor_controller.py     # Pi GPIO motors (fallback, not used with Arduino offload)
│   ├── arduino_motor_controller.py # Pi-side serial client to the Arduino (used on the real vehicle)
│   ├── mock_motor_controller.py # PC development mock
│   ├── start_switch.py         # WRO start-button waiting-state gate
│   ├── hsv_calibrator.py       # Live HSV tuning tool
│   ├── focal_calibrator.py     # Empirical focal length calibration
│   ├── lens_calibrator.py      # Optional lens distortion calibration
│   └── debug_visualizer.py     # Full debug window for PC
├── arduino/
│   └── motor_controller/motor_controller.ino  # Arduino sketch: servo PWM + DRV8825 stepper pulses + rear ultrasonic telemetry
├── config/
│   ├── hsv_values.json         # Saved HSV calibration
│   └── camera_calibration.json # Focal length + lens distortion
├── docs/
│   ├── ARDUINO_OFFLOAD.md      # Pi/Arduino split architecture + protocol
│   ├── ACKERMANN_STEERING.md   # Why + how of the Ackermann linkage
│   ├── COMPONENTS.md           # Component choices and rationale
│   ├── stl_files/              # 3D-printed chassis STL/CAD files
│   └── vehicle_photos/
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

**### 1. Starting point**
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

**### 2. Chassis redesign — proper Ackermann steering**
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
  **true** Ackermann linkage instead of parallel steering — it's the same
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

**### 3. Drive motor redesign — DC motor → stepper**
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

**### 4. Software/firmware updates made for the above (this repo, current state)**
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

**### 5. Competition-Day Final Verification Checklist — Must Be Verified on the Real Vehicle

The following items are the **final physical and operational checks to be completed and verified on the real vehicle on the day of the competition**, after the updated chassis has been printed, assembled, and integrated. These checks are intentionally listed here because the final vehicle configuration may require small adjustments during assembly, calibration, and track preparation.

- [ ] **Confirm stepper-as-drive-motor is rules-compliant** with organizers/judges (see Rules caveat above). This remains the highest-priority compliance check before the vehicle is used in competition.
- [ ] **Verify and finalise Ackermann steering geometry** on the assembled vehicle and update `SERVO_CENTER_DEG` / `SERVO_LEFT_DEG` / `SERVO_RIGHT_DEG` in `motor_controller.ino` according to the measured physical steering range.
- [ ] **Measure and confirm the real drive-wheel diameter** and ensure `WHEEL_DIAMETER_MM` in `motor_controller.ino` matches the final installed wheels.
- [ ] **Confirm STH-39D219 step count/revolution and DRV8825 microstepping configuration** against the actual motor and driver wiring before the final run.
- [ ] **Bench-test the stepper drivetrain** with the vehicle safely supported: confirm forward/reverse direction, verify smooth operation, and check for stalling or skipped steps at the intended operating speed.
- [ ] **Verify the physical start button and power-switch arrangement** on the final vehicle and confirm that the vehicle remains stationary until the required physical start action.
- [ ] **Perform final camera, focal-length, lens, and corridor calibration** with the camera mounted in its final position on the redesigned chassis.
- [ ] **Verify the complete CV and control pipeline** on the final assembled vehicle, including pillar detection, wall following, section detection, adaptive speed control, and parking behaviour.
- [ ] **Perform a final three-lap and parking validation run** on the real vehicle, using the final chassis and competition configuration, before the vehicle is entered into competition.
- [ ] **Complete final chassis assembly and inspection**: verify all newly 3D-printed parts are securely mounted, check the Ackermann linkage and motor mount, inspect fasteners and clearances, and confirm that the final chassis is mechanically ready for competition.

These checks are a normal part of the final engineering and competition-preparation process. The purpose is to ensure that the **physical vehicle presented at competition matches the documented design and is correctly calibrated, mechanically secure, and operationally verified**.
