"""
Debug Visualizer
Runs the full CV pipeline on webcam or video file
and shows a live annotated debug window.
No hardware needed — runs entirely on PC.

Usage:
    python debug_visualizer.py
    python debug_visualizer.py --source path/to/video.mp4
"""
import numpy as np
import cv2
import time
import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from cv_pipeline     import CVPipeline, PILLAR_AVOID_TRIGGER_MM
from kalman_filter   import KalmanFilter
from pillar_memory   import PillarMemory
from wall_follower   import WallFollower
from section_detector import SectionDetector
from speed_controller import SpeedController
from ultrasonic_parking_controller import UltrasonicParkingController
from pid             import PID
from mock_motor_controller import MotorController


def run(source=0, mode="obstacle", direction="CW"):
    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print(f"Cannot open source: {source}")
        return

    # Init all modules
    cv_pipeline   = CVPipeline(mode=mode)
    kalman_red   = KalmanFilter(process_noise=1e-3, measurement_noise=1e-1)
    kalman_green = KalmanFilter(process_noise=1e-3, measurement_noise=1e-1)
    pillar_memory = PillarMemory()
    wall_follower = WallFollower(mode=mode)
    section_det   = SectionDetector(direction=direction)
    speed_ctrl    = SpeedController()
    steering_pid  = PID(kp=0.4, ki=0.01, kd=0.1)
    motors        = MotorController()
    # mock_motor_controller.read_ultrasonic() always returns None on PC, so
    # this exercises the vision-only fallback path -- the real ultrasonic
    # path only runs on the Pi against ArduinoMotorController.
    parking_ctrl  = UltrasonicParkingController(motors)

    laps        = 0
    frame_count = 0
    fps_timer   = time.time()
    fps         = 0
    parking_active = False   # toggled with 'p' -- lets you test parking logic
                              # standalone on PC without needing 3 real laps first

    print("Debug Visualizer running. Press Q to quit, P to toggle parking-phase test.")

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        frame = cv2.resize(frame, (320, 240))
        frame_count += 1

        # FPS counter
        if frame_count % 30 == 0:
            fps = 30 / (time.time() - fps_timer)
            fps_timer = time.time()

        # ── Full detection ────────────────────────────────────────────────
        detections = cv_pipeline.process(frame)

        kalman_red.update(detections["red_pillar"])
        kalman_green.update(detections["green_pillar"])

        section_event = section_det.update(detections["lines"])
        if section_event and section_event.get("lap_complete"):
            laps += 1
            pillar_memory.next_lap()

        red_pos   = kalman_red.predict()
        green_pos = kalman_green.predict()

        pillar_memory.update(
            section=section_det.current_section,
            red=red_pos,
            green=green_pos
        )

        recalled = pillar_memory.recall(section_det.current_section)

        # ── Control ───────────────────────────────────────────────────────
        park_out = None
        if parking_active:
            park_out = parking_ctrl.update(detections)

        pillar_info = None
        if park_out is not None:
            steering = park_out["steering"]
            speed    = park_out["speed"]
            wall_error = 0.0
            pillar_action = 0.0
            if park_out["done"]:
                print(f"[PARKING] done, result={park_out.get('result')}")
        else:
            wall_error    = wall_follower.get_error(detections.get("walls", {}))
            pillar_info   = cv_pipeline.get_pillar_action_info(red_pos, green_pos, recalled)
            pillar_action = pillar_info["action"] if pillar_info else 0.0
            steering      = steering_pid.compute(wall_error + pillar_action)
            speed         = speed_ctrl.compute(
                wall_error  = wall_error,
                pillar_near = detections.get("pillar_near", False),
                corner_near = detections.get("corner_near", False),
                wall_ahead  = detections.get("walls", {}).get("wall_ahead", False),
                lap         = laps + 1
            )

        motors.set_steering(steering)
        motors.set_speed(speed)
        servo_duty, servo_dir = motors.steering_duty

        # ── Debug overlay ─────────────────────────────────────────────────
        debug = cv_pipeline.draw_debug(frame, detections)

        # Kalman predictions
        if red_pos:
            cv2.circle(debug, (int(red_pos[0]), int(red_pos[1])), 8, (0,0,200), 2)
        if green_pos:
            cv2.circle(debug, (int(green_pos[0]), int(green_pos[1])), 8, (0,200,0), 2)

        # HUD
        h, w = debug.shape[:2]

        if pillar_info:
            pillar_line = (f"Pillar: {pillar_info['color'].upper()} @ "
                            f"{pillar_info['distance_mm']:.0f}mm -> "
                            f"{pillar_info['direction']} (Rule 9.19)")
        else:
            pillar_line = "Pillar: none in view/memory"

        hud_lines = [
            f"FPS: {fps:.0f}",
            f"Laps: {laps}",
            f"Section: {section_det.current_section} ({section_det.section_type})",
            f"Steering: {steering:.3f}  (wall {wall_error:+.3f} + pillar {pillar_action:+.3f})",
            f"Servo: {servo_duty:.1f}% duty -> {servo_dir} (front axle only, Rule 11.3)",
            f"Speed: {speed} mm/s",
            pillar_line,
            f"Avoid trigger: <{PILLAR_AVOID_TRIGGER_MM:.0f}mm (tuned value, not rule-specified)",
            f"Map cov: {pillar_memory.coverage():.0f}%",
            f"Parking: {'ON - ' + parking_ctrl.state if parking_active else 'off (press P)'}",
        ]
        for i, line in enumerate(hud_lines):
            cv2.putText(debug, line, (w + 5, 20 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Steering bar
        bar_x = int(w // 2 + steering * w // 2)
        cv2.line(debug, (w//2, h-5), (bar_x, h-5), (0, 255, 255), 3)

        # Expand frame for HUD panel
        PANEL_W = 300
        panel = np.zeros((h, PANEL_W, 3), dtype=np.uint8)
        combined = np.hstack([debug, panel])

        # Re-draw HUD on panel
        for i, line in enumerate(hud_lines):
            cv2.putText(combined, line, (w + 5, 20 + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        cv2.imshow("WRO Debug", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('p'):
            parking_active = not parking_active
            parking_ctrl.reset()
            print(f"[PARKING] test mode {'ENABLED' if parking_active else 'disabled'} "
                  f"-- hold 2 magenta markers alongside the car in frame to trigger")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=0,
                        help="0=webcam or path to video file")
    parser.add_argument("--mode", default="obstacle", choices=["obstacle", "open"])
    parser.add_argument("--direction", default="CW", choices=["CW", "CCW"])
    args = parser.parse_args()

    try:
        source = int(args.source)
    except ValueError:
        source = args.source

    run(source, mode=args.mode, direction=args.direction)
