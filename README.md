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
```

### Key Modules
- **camera.py** — Threaded frame capture, works on Pi (picamera2) and PC (webcam/video)
- **cv_pipeline.py** — Full color detection, contour analysis, wall/pillar/line detection
- **kalman_filter.py** — Smooth pillar tracking across frames, handles brief occlusions
- **pillar_memory.py** — Stores lap 1 pillar positions, enables faster laps 2 & 3
- **wall_follower.py** — Keeps car centered in corridor using wall distance
- **section_detector.py** — Detects orange/blue lines for section and lap counting
- **speed_controller.py** — Adaptive speed based on surroundings
- **pid.py** — Generic PID with anti-windup and derivative filtering

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
```

### On Raspberry Pi (competition)
```bash
pip install -r requirements.txt
pip install picamera2
cd src
python main.py
```

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
│   ├── pid.py                  # PID controller
│   ├── motor_controller.py     # Pi GPIO motors
│   ├── mock_motor_controller.py # PC development mock
│   ├── hsv_calibrator.py       # Live HSV tuning tool
│   └── debug_visualizer.py     # Full debug window for PC
├── config/
│   └── hsv_values.json         # Saved HSV calibration
├── docs/
│   └── vehicle_photos/
├── logs/
├── video/
├── requirements.txt
└── README.md
```