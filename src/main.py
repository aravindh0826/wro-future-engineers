"""
WRO 2026 Future Engineers - Main Entry Point
Runs the full CV pipeline for autonomous driving + parking.
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
from parking_controller import ParkingController
from pid import PID

logging.basicConfig(
    filename='../logs/run.log',
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(message)s'
)
logger = logging.getLogger(__name__)

IS_PI = platform.system() == "Linux"

if IS_PI:
    from motor_controller import MotorController
else:
    from mock_motor_controller import MotorController
    print("[PC MODE] Running with mocked motors")

# ── Per-round configuration ──────────────────────────────────────────────────
# Set these before every round per the judge's coin toss / announcement
# (rules 9.3-9.5, 9.8). CV cannot and should not infer these.
CHALLENGE_MODE = "obstacle"   # "obstacle" or "open"
DIRECTION      = "CW"         # "CW" or "CCW"

MAX_LAPS        = 3
TARGET_SPEED    = 200
MAX_SPEED       = 400
MIN_SPEED       = 100
FRAME_WIDTH     = 320
FRAME_HEIGHT    = 240
PARKING_TIMEOUT = 15.0   # seconds, safety cap on the parking phase


def main():
    logger.info(f"Starting WRO 2026 Future Engineers ({CHALLENGE_MODE}, {DIRECTION})")
    print(f"Initialising... mode={CHALLENGE_MODE} direction={DIRECTION}")

    camera        = Camera(width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=60)
    cv_pipeline   = CVPipeline(mode=CHALLENGE_MODE)
    kalman_red    = KalmanFilter()
    kalman_green  = KalmanFilter()
    pillar_memory = PillarMemory()
    wall_follower = WallFollower(mode=CHALLENGE_MODE)
    section_det   = SectionDetector(direction=DIRECTION)
    speed_ctrl    = SpeedController(base=TARGET_SPEED, max_s=MAX_SPEED, min_s=MIN_SPEED)
    parking_ctrl  = ParkingController()
    steering_pid  = PID(kp=0.4, ki=0.01, kd=0.1)
    motors        = MotorController()

    camera.start()
    motors.start()

    laps        = 0
    frame_count = 0
    detections  = {}

    print("Pipeline running. Press Ctrl+C to stop.")

    try:
        # ── Driving phase: 3 laps ────────────────────────────────────────────
        while laps < MAX_LAPS:
            frame = camera.get_frame()
            if frame is None:
                continue
            frame_count += 1

            if frame_count % 2 == 0:
                detections = cv_pipeline.process(frame)

                if detections["red_pillar"]:
                    kalman_red.update(detections["red_pillar"])
                if detections["green_pillar"]:
                    kalman_green.update(detections["green_pillar"])

                section_event = section_det.update(detections["lines"])
                if section_event and section_event.get("lap_complete"):
                    laps += 1
                    logger.info(f"Lap {laps} complete")
                    print(f"Lap {laps} complete!")
                    pillar_memory.next_lap()

                red_pos   = kalman_red.predict()
                green_pos = kalman_green.predict()
                pillar_memory.update(section=section_det.current_section,
                                      red=red_pos, green=green_pos)

            recalled = pillar_memory.recall(section_det.current_section)
            wall_error = wall_follower.get_error(detections.get("walls", {}))
            pillar_action = cv_pipeline.get_pillar_action(
                red_pos=kalman_red.predict(), green_pos=kalman_green.predict(),
                recalled=recalled
            ) if CHALLENGE_MODE == "obstacle" else 0.0

            steering = steering_pid.compute(wall_error + pillar_action)
            speed = speed_ctrl.compute(
                wall_error=wall_error,
                pillar_near=detections.get("pillar_near", False),
                corner_near=detections.get("corner_near", False),
                wall_ahead=detections.get("walls", {}).get("wall_ahead", False)
            )
            motors.set_steering(steering)
            motors.set_speed(speed)

        # ── Parking phase (Obstacle Challenge only) ─────────────────────────
        if CHALLENGE_MODE == "obstacle":
            print("Laps complete. Searching for parking lot...")
            park_start = time.time()
            while time.time() - park_start < PARKING_TIMEOUT:
                frame = camera.get_frame()
                if frame is None:
                    continue
                detections = cv_pipeline.process(frame)

                park_out = parking_ctrl.update(detections)
                if park_out is None:
                    wall_error = wall_follower.get_error(detections.get("walls", {}))
                    steering = steering_pid.compute(wall_error)
                    speed = speed_ctrl.compute(wall_error=wall_error)
                    motors.set_steering(steering)
                    motors.set_speed(speed)
                else:
                    motors.set_steering(park_out["steering"])
                    motors.set_speed(park_out["speed"])
                    if park_out["done"]:
                        logger.info("Parking complete")
                        print("Parked.")
                        break

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        camera.stop()
        motors.stop()
        logger.info("Pipeline stopped cleanly")
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
