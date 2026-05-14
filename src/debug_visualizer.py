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

from cv_pipeline     import CVPipeline
from kalman_filter   import KalmanFilter
from pillar_memory   import PillarMemory
from wall_follower   import WallFollower
from section_detector import SectionDetector
from speed_controller import SpeedController
from pid             import PID
from mock_motor_controller import MotorController


def run(source=0):
    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print(f"Cannot open source: {source}")
        return

    # Init all modules
    cv_pipeline   = CVPipeline()
    kalman_red   = KalmanFilter(process_noise=1e-3, measurement_noise=1e-1)
    kalman_green = KalmanFilter(process_noise=1e-3, measurement_noise=1e-1)
    pillar_memory = PillarMemory()
    wall_follower = WallFollower()
    section_det   = SectionDetector()
    speed_ctrl    = SpeedController()
    steering_pid  = PID(kp=0.4, ki=0.01, kd=0.1)
    motors        = MotorController()

    laps        = 0
    frame_count = 0
    fps_timer   = time.time()
    fps         = 0

    print("Debug Visualizer running. Press Q to quit.")

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
        wall_error    = wall_follower.get_error(detections.get("walls", {}))
        pillar_action = cv_pipeline.get_pillar_action(red_pos, green_pos, recalled)
        steering      = steering_pid.compute(wall_error + pillar_action)
        speed         = speed_ctrl.compute(
            wall_error  = wall_error,
            pillar_near = detections.get("pillar_near", False),
            corner_near = detections.get("corner_near", False),
            lap         = laps + 1
        )

        motors.set_steering(steering)
        motors.set_speed(speed)

        # ── Debug overlay ─────────────────────────────────────────────────
        debug = cv_pipeline.draw_debug(frame, detections)

        # Kalman predictions
        if red_pos:
            cv2.circle(debug, (int(red_pos[0]), int(red_pos[1])), 8, (0,0,200), 2)
        if green_pos:
            cv2.circle(debug, (int(green_pos[0]), int(green_pos[1])), 8, (0,200,0), 2)

        # HUD
        h, w = debug.shape[:2]
        hud_lines = [
            f"FPS: {fps:.0f}",
            f"Laps: {laps}",
            f"Section: {section_det.current_section} ({section_det.section_type})",
            f"Steering: {steering:.3f}",
            f"Speed: {speed} mm/s",
            f"Wall err: {wall_error:.3f}",
            f"Map cov: {pillar_memory.coverage():.0f}%",
        ]
        for i, line in enumerate(hud_lines):
            cv2.putText(debug, line, (w + 5, 20 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # Steering bar
        bar_x = int(w // 2 + steering * w // 2)
        cv2.line(debug, (w//2, h-5), (bar_x, h-5), (0, 255, 255), 3)

        # Expand frame for HUD panel
        panel = np.zeros((h, 160, 3), dtype=np.uint8)
        combined = np.hstack([debug, panel])

        # Re-draw HUD on panel
        for i, line in enumerate(hud_lines):
            cv2.putText(combined, line, (w + 5, 20 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.imshow("WRO Debug", combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import numpy as np

    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=0,
                        help="0=webcam or path to video file")
    args = parser.parse_args()

    try:
        source = int(args.source)
    except ValueError:
        source = args.source

    run(source)
