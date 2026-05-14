"""
WRO 2025 Future Engineers - Main Entry Point
Runs the full CV pipeline for autonomous driving
"""

import time
import platform
import logging
from camera import Camera
from cv_pipeline import CVPipeline
from kalman_filter import KalmanFilter
from pillar_memory import PillarMemory
from wall_follower import WallFollower
from section_detector import SectionDetector
from speed_controller import SpeedController
from pid import PID

# Setup logging
logging.basicConfig(
    filename='../logs/run.log',
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(message)s'
)
logger = logging.getLogger(__name__)

# ── Platform detection ───────────────────────────────────────────────────────
IS_PI = platform.system() == "Linux"

if IS_PI:
    from motor_controller import MotorController
else:
    from mock_motor_controller import MotorController
    print("[PC MODE] Running with mocked motors")

# ── Constants ────────────────────────────────────────────────────────────────
MAX_LAPS        = 3
TARGET_SPEED    = 200          # mm/s base speed
MAX_SPEED       = 400          # mm/s max speed
MIN_SPEED       = 100          # mm/s min speed (corners)
FRAME_WIDTH     = 320
FRAME_HEIGHT    = 240


def main():
    logger.info("Starting WRO 2025 Future Engineers CV Pipeline")
    print("Initialising WRO 2025 Future Engineers...")

    # ── Init all modules ─────────────────────────────────────────────────────
    camera          = Camera(width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=60)
    cv_pipeline     = CVPipeline()
    kalman_red      = KalmanFilter()
    kalman_green    = KalmanFilter()
    pillar_memory   = PillarMemory()
    wall_follower   = WallFollower()
    section_det     = SectionDetector()
    speed_ctrl      = SpeedController(base=TARGET_SPEED, max_s=MAX_SPEED, min_s=MIN_SPEED)
    steering_pid    = PID(kp=0.4, ki=0.01, kd=0.1)
    motors          = MotorController()

    camera.start()
    motors.start()

    laps        = 0
    direction   = None
    frame_count = 0
    detections  = {}

    print("Pipeline running. Press Ctrl+C to stop.")

    try:
        while laps < MAX_LAPS:
            frame = camera.get_frame()
            if frame is None:
                continue

            frame_count += 1

            # ── Full detection every 2nd frame ────────────────────────────
            if frame_count % 2 == 0:
                detections = cv_pipeline.process(frame)

                # Kalman update for pillars
                if detections["red_pillar"]:
                    kalman_red.update(detections["red_pillar"])
                if detections["green_pillar"]:
                    kalman_green.update(detections["green_pillar"])

                # Section detection → lap counting + direction
                section_event = section_det.update(detections["lines"])
                if section_event:
                    direction = section_event.get("direction", direction)
                    if section_event.get("lap_complete"):
                        laps += 1
                        logger.info(f"Lap {laps} complete")
                        print(f"Lap {laps} complete!")
                        pillar_memory.next_lap()

                # Parking marker detection — log for now; parking logic TBD
                if detections.get("parking_near"):
                    logger.info("Parking marker detected")

                # Pillar memory — store on lap 1, recall on laps 2 & 3
                red_pos   = kalman_red.predict()
                green_pos = kalman_green.predict()
                pillar_memory.update(
                    section=section_det.current_section,
                    red=red_pos,
                    green=green_pos
                )

            # ── Always: steering + speed update every frame ───────────────
            recalled = pillar_memory.recall(section_det.current_section)

            wall_error    = wall_follower.get_error(detections.get("walls", {}))
            pillar_action = cv_pipeline.get_pillar_action(
                red_pos   = kalman_red.predict(),
                green_pos = kalman_green.predict(),
                recalled  = recalled
            )

            steering = steering_pid.compute(wall_error + pillar_action)
            speed    = speed_ctrl.compute(
                wall_error    = wall_error,
                pillar_near   = detections.get("pillar_near", False),
                corner_near   = detections.get("corner_near", False)
            )

            motors.set_steering(steering)
            motors.set_speed(speed)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        camera.stop()
        motors.stop()
        logger.info("Pipeline stopped cleanly")
        print("Shutdown complete.")


if __name__ == "__main__":
    main()