# WRO 2026 Future Engineers — Team [Titan Grandmasters]

## Team Introduction
- **Team Name:** [Titan Grandmasters]
- **Country:** [India]
- **Members:** [Aravindh Balaji], [Balaji Keerthi], [Abdul Hakeem K]
- **Coach:** [Saurav Kumar Topo]
- **Season:** WRO 2026 Future Engineers

---

## Vehicle Description
Our autonomous vehicle uses a Raspberry Pi 5 as the main controller paired with a Pi Camera Module v1.3 for vision. The vehicle uses a differential drive system with a servo-controlled steering mechanism.

### Hardware
| Component | Specification |
|-----------|--------------|
| Controller | Raspberry Pi 5 (4GB) |
| Camera | Pi Camera Module v1.3 |
| Drive Motor | DC motor with L298N driver |
| Steering | Servo motor |
| Power | 7.4V LiPo battery |

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
    ↓
Parking Controller (vision-guided parallel parking, after lap 3)
```

### Key Modules
- **camera.py** — Threaded frame capture, works on Pi (picamera2) and PC (webcam/video)
- **cv_pipeline.py** — Full color detection, contour analysis, wall/pillar/parking-marker detection. Loads HSV ranges from `config/hsv_values.json` and focal length / lens calibration from `config/camera_calibration.json` if present.
- **kalman_filter.py** — Smooth pillar tracking across frames, handles brief occlusions
- **pillar_memory.py** — Stores lap 1 pillar positions, enables faster laps 2 & 3
- **wall_follower.py** — Keeps car centered in corridor; mode-aware (Obstacle Challenge corridor is fixed 1000mm, Open Challenge varies 1000/600mm)
- **section_detector.py** — Detects orange/blue lines for section and lap counting; driving direction (CW/CCW) is passed in per round, not inferred
- **speed_controller.py** — Adaptive speed based on surroundings
- **parking_controller.py** — Vision-guided parallel-parking state machine, triggered after lap 3. The parking lot is bounded by two magenta blocks (rule 13.25, 200×20×100mm)
- **pid.py** — Generic PID with anti-windup and derivative filtering
- **focal_calibrator.py** — Empirically measures `focal_length_px` against a real pillar at a known distance
- **lens_calibrator.py** — Optional checkerboard lens-distortion calibration
- **corridor_calibrator.py** — Measures the real corridor width (wide 1000mm / narrow 600mm) in pixels with the car centered in it, and saves the result to `config/corridor_calibration.json` for use by `wall_follower.py`

### Color Detection
All colors detected using HSV color space with CLAHE preprocessing for lighting robustness:
| Color | Purpose |
|-------|---------|
| Red | Traffic pillar — pass on right |
| Green | Traffic pillar — pass on left |
| Orange | Corner section lines |
| Blue | Straight section lines |
| Magenta | Parking-lot boundary markers (rule 13.25) |
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
python lens_calibrator.py                                # optional: lens distortion
```
Saves to `config/camera_calibration.json`, loaded automatically by `cv_pipeline.py`.

### Corridor Calibration
```bash
cd src
python corridor_calibrator.py --type wide     # 1000mm corridor
python corridor_calibrator.py --type narrow   # 600mm corridor
```
Center the car in the real corridor, then press `s` to save the measured pixel width. Saves to `config/corridor_calibration.json`, used by `wall_follower.py`.

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
│   ├── main.py                   # Entry point
│   ├── camera.py                 # Camera capture (Pi + PC)
│   ├── cv_pipeline.py            # Full CV detection pipeline
│   ├── kalman_filter.py          # Pillar tracking
│   ├── pillar_memory.py          # Lap 1 map for faster laps
│   ├── wall_follower.py          # Wall centering
│   ├── section_detector.py       # Lap and section counting
│   ├── speed_controller.py       # Adaptive speed
│   ├── parking_controller.py     # Parallel-parking state machine
│   ├── pid.py                    # PID controller
│   ├── motor_controller.py       # Pi GPIO motors
│   ├── mock_motor_controller.py  # PC development mock
│   ├── hsv_calibrator.py         # Live HSV tuning tool
│   ├── focal_calibrator.py       # Empirical focal length calibration
│   ├── lens_calibrator.py        # Optional lens distortion calibration
│   ├── corridor_calibrator.py    # Corridor width calibration
│   └── debug_visualizer.py       # Full debug window for PC
├── config/
│   ├── hsv_values.json           # Saved HSV calibration
│   ├── camera_calibration.json   # Focal length + lens distortion
│   └── corridor_calibration.json # Wide/narrow corridor pixel widths
├── docs/
│   └── vehicle_photos/
├── logs/
├── video/
├── requirements.txt
└── README.md
```
