# Component Choices — What and Why

| Component | Role | Why this component |
|---|---|---|
| Raspberry Pi 4B (4GB) | Main SBC — runs the CV pipeline, wall-following/PID, speed control, section/lap logic, parking state machine | Enough CPU/RAM to run OpenCV in real time; large community support and a first-class camera interface (CSI), which a microcontroller alone can't provide |
| Arduino Uno | Secondary SBM — real-time servo PWM, DRV8825 stepper pulses, ultrasonic polling, serial link to Pi | Offloads latency-sensitive, hardware-timing work off the Pi so CV frame processing isn't jittered by actuation, and gives a 200ms watchdog fail-safe independent of the Pi (Rule 11.6). Split documented in `CHANGES_ARDUINO_OFFLOAD.md` |
| Pi Camera Module Rev 1.3 (OV5647) | Vision input for wall/pillar/parking-marker detection | Native CSI interface to the Pi (lower latency/CPU than USB), well-supported by `picamera2` |
| NEMA-14 stepper (STH-39D219) + DRV8825 | Rear drive motor | Open-loop step control gives repeatable, predictable low-speed behaviour lap-to-lap, which matters more for this vehicle than raw torque headroom — see `CHANGES_STEPPER_DRIVE.md` for the trade-offs (no stall feedback, step-rate ceiling) |
| GT2 pulley/belt (20T/60T, 3:1) | Reduction between stepper and rear axle | Converts the stepper's higher, lower-torque speed into the torque/speed range the axle actually needs, without the backlash of a gear train |
| Steering servo (single) | Front-axle steering actuator | Rule 11.3 permits one steering actuator of any type; a servo gives direct angle control, which the Ackermann linkage (see `docs/ACKERMANN_STEERING.md`) needs at its input arm |
| MPU6050 IMU | Heading/orientation tracking, drift correction between vision updates | Cheap, well-supported 6-axis sensor; smooths over frames where vision temporarily loses a clean reference (see `imu_tracker.py`) |
| 2x HC-SR04 ultrasonic | Blind-spot ranging during the reverse/align phase of parallel parking | The forward camera can't see the parking markers once the car is mid-reverse; ultrasonic covers exactly that blind spot (see `ultrasonic_parking_controller.py`) |
| 3D-printed chassis + Ackermann linkage | Structural frame, steering knuckles, tie rods, mounts | Off-the-shelf kit geometry didn't give a true Ackermann linkage or clean mounting for the re-oriented electronics layout; printing lets us iterate the geometry directly — see `docs/stl_files/` |
| 7.4V 2S LiPo + single in-line switch | Power source | Matches the voltage/current needs of the stepper + servo + Pi/Arduino combo; single switch keeps the "power off = fully off" rule (Rule 12.6) simple to satisfy |
