"""
WRO 2026 Future Engineers - Main Entry Point
Runs the full CV pipeline for autonomous driving + parking.
"""

import time
import logging
from camera import Camera, IS_PI
from cv_pipeline import CVPipeline
from kalman_filter import KalmanFilter
from pillar_memory import PillarMemory
from sign_grid import SignGrid
from wall_follower import WallFollower
from section_detector import SectionDetector
from speed_controller import SpeedController
from ultrasonic_parking_controller import UltrasonicParkingController
from pid import PID
from imu_tracker import make_imu_tracker
from start_switch import StartSwitch

logging.basicConfig(
    filename='../logs/run.log',
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(message)s'
)
logger = logging.getLogger(__name__)

if IS_PI:
    # Motor/servo PWM generation is offloaded to an Arduino over wired
    # serial (Rules 11.8/11.9 permit multiple SBC/SBM; Rules 11.10/11.17
    # require the link to be a physical wire). See
    # arduino_motor_controller.py and arduino/motor_controller/motor_controller.ino
    # for the protocol and full rationale. The old direct-GPIO
    # motor_controller.MotorController is kept in the repo for reference /
    # as a fallback if you ever want to drive the L298N/servo straight
    # from the Pi again -- swap the import back if so.
    from arduino_motor_controller import ArduinoMotorController as MotorController
else:
    from mock_motor_controller import MotorController
    print("[PC MODE] Running with mocked motors")

# ── Per-round configuration ──────────────────────────────────────────────────
# Set these before every round per the judge's coin toss / announcement
# (rules 9.3-9.5, 9.8). CV cannot and should not infer these.
CHALLENGE_MODE = "obstacle"   # "obstacle" or "open"
DIRECTION      = "CW"         # "CW" or "CCW"

# Straight-section length for this round, mm. NOT measured by current CV --
# per rules Appendix B (page 43), the interior wall is built per-round from
# one of three prepared segment sets, giving a straight section length of
# 1000, 1400, or 1800 mm. This is announced/determined by the same
# randomization procedure as CHALLENGE_MODE/DIRECTION above (rules 8, 8b),
# not inferred from the camera. Used by SignGrid to convert a lap-1
# classified grid column into a real mm steering target -- get this number
# wrong and every grid-classified target on lap 2/3 will be off by a fixed
# scale factor, so it MUST be set correctly before the round, same as
# DIRECTION. UNVERIFIED DEFAULT below -- confirm/update at the field.
SECTION_LENGTH_MM = 1400

MAX_LAPS        = 3
TARGET_SPEED    = 200
MAX_SPEED       = 400
MIN_SPEED       = 100
FRAME_WIDTH     = 320
FRAME_HEIGHT    = 240
PARKING_TIMEOUT   = 15.0   # seconds, safety cap on searching for + doing parking
STOP_HOLD_SECONDS = 3.0    # seconds to remain stationary after stopping, so the
                           # vehicle doesn't drift/creep while being judged as stopped.
                           # This is a conservative default, not a rule-specified
                           # number -- the rules require the vehicle to actually
                           # be stationary, not just touch zero speed momentarily.


def main():
    logger.info(f"Starting WRO 2026 Future Engineers ({CHALLENGE_MODE}, {DIRECTION})")
    print(f"Initialising... mode={CHALLENGE_MODE} direction={DIRECTION}")

    camera        = Camera(width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=60)
    cv_pipeline   = CVPipeline(mode=CHALLENGE_MODE)
    kalman_red    = KalmanFilter()
    kalman_green  = KalmanFilter()
    pillar_memory = PillarMemory()
    sign_grid     = SignGrid()
    wall_follower = WallFollower(mode=CHALLENGE_MODE)
    section_det   = SectionDetector(direction=DIRECTION)
    speed_ctrl    = SpeedController(base=TARGET_SPEED, max_s=MAX_SPEED, min_s=MIN_SPEED)
    steering_pid  = PID(kp=0.4, ki=0.01, kd=0.1)
    motors        = MotorController()
    # Needs the live motors instance -- it reads rear ultrasonic telemetry
    # via motors.read_ultrasonic() during the blind-spot reverse manoeuvre
    # (see ultrasonic_parking_controller.py for why vision alone isn't
    # enough here).
    parking_ctrl  = UltrasonicParkingController(motors)
    imu           = make_imu_tracker()
    start_switch  = StartSwitch()

    camera.start()
    motors.start()
    imu.start()   # calibrates gyro bias -- vehicle MUST be stationary here.
                   # Calibrating now (before the start-button wait below)
                   # rather than after gives it strictly more settle time,
                   # since the vehicle is required to be stationary in the
                   # starting zone for this entire window anyway.

    # ── Waiting state (WRO starting procedure) ───────────────────────────
    # Rules ~9.8-9.14: after the vehicle is switched on (single physical
    # power switch, hardware only -- see start_switch.py's docstring), it
    # must sit in a waiting state until a single Start button is pressed,
    # and only THEN may it begin moving. Everything above this point
    # (camera/motor/IMU init) happens automatically on power-on; nothing
    # below it happens until the button press. This is also the point
    # that satisfies "no external/remote control" (Rule 11.6) for how the
    # round is triggered -- a button physically on the vehicle, not a
    # signal from a laptop/remote.
    start_switch.wait_for_press()

    laps        = 0
    frame_count = 0
    detections  = {}

    print("Pipeline running. Press Ctrl+C to stop.")

    try:
        # ── Driving phase: 3 laps ────────────────────────────────────────────
        prev_t = time.time()
        while laps < MAX_LAPS:
            frame = camera.get_frame()
            if frame is None:
                continue
            frame_count += 1

            now = time.time()
            frame_dt = max(now - prev_t, 1e-3)
            prev_t = now
            imu.update()   # cheap single I2C read -- safe every frame

            if frame_count % 2 == 0:
                detections = cv_pipeline.process(frame)

                kalman_red.update(detections["red_pillar"],
                                   yaw_rate_dps=imu.yaw_rate_dps, dt=frame_dt)
                kalman_green.update(detections["green_pillar"],
                                     yaw_rate_dps=imu.yaw_rate_dps, dt=frame_dt)

                section_event = section_det.update(detections["lines"])
                if section_event:
                    imu.reset_heading()   # zero drift at every section boundary
                    if section_event.get("lap_complete"):
                        laps += 1
                        logger.info(f"Lap {laps} complete")
                        print(f"Lap {laps} complete!")
                        pillar_memory.next_lap()

                red_pos   = kalman_red.predict()
                green_pos = kalman_green.predict()
                walls_now = detections.get("walls", {})
                wall_offset_now = {
                    "red":   wall_follower.lateral_offset_mm(red_pos[0], walls_now)   if red_pos   else None,
                    "green": wall_follower.lateral_offset_mm(green_pos[0], walls_now) if green_pos else None,
                }
                grid_column_now = {
                    "red":   (sign_grid.classify_column(wall_offset_now["red"],   SECTION_LENGTH_MM) or (None, None))[0]
                             if wall_offset_now["red"]   is not None else None,
                    "green": (sign_grid.classify_column(wall_offset_now["green"], SECTION_LENGTH_MM) or (None, None))[0]
                             if wall_offset_now["green"] is not None else None,
                }
                pillar_memory.update(section=section_det.current_section,
                                      red=red_pos, green=green_pos,
                                      heading_deg=imu.heading_deg,
                                      wall_offset_mm=wall_offset_now,
                                      grid_column=grid_column_now)

            recalled = pillar_memory.recall(section_det.current_section,
                                             heading_deg=imu.heading_deg)
            recalled_wall_offset = pillar_memory.recall_wall_offset(section_det.current_section)
            recalled_grid_column = pillar_memory.recall_grid_column(section_det.current_section)
            section_confidence = pillar_memory.confidence(section_det.current_section)
            wall_error = wall_follower.get_error(detections.get("walls", {}))
            pillar_info = cv_pipeline.get_pillar_action_info(
                red_pos=kalman_red.predict(), green_pos=kalman_green.predict(),
                recalled=recalled
            ) if CHALLENGE_MODE == "obstacle" else None
            pillar_action = pillar_info["action"] if pillar_info else 0.0

            # Alignment bias: on lap 2/3, if this section's obstacle gap was
            # already measured on lap 1, nudge steering toward reproducing
            # that same wall-to-obstacle gap instead of only reacting to
            # the live pillar position -- holds a straighter line past a
            # known obstacle. Live detection (pillar_action above) still
            # always takes priority; this is a small correction on top,
            # not a replacement.
            #
            # Target source priority:
            #   1. Grid column (SignGrid) -- exact precomputed geometry,
            #      if lap 1's reading confidently classified to a known
            #      card position. Preferred: stable, not re-derived from
            #      noisy live pixels.
            #   2. Continuous wall-offset mm (recalled_wall_offset) --
            #      fallback for sections where lap 1's reading was
            #      ambiguous between two grid columns (see sign_grid.py
            #      SNAP_CONFIDENCE_FRACTION) and no grid target exists.
            alignment_bias = 0.0
            if pillar_info:
                color = pillar_info["color"]
                target_mm = None

                grid_col = recalled_grid_column.get(color) if recalled_grid_column else None
                if grid_col is not None:
                    target_mm = sign_grid.target_mm_for_cell(grid_col, SECTION_LENGTH_MM)

                if target_mm is None and recalled_wall_offset:
                    target_mm = recalled_wall_offset.get(color)

                live_pos = kalman_red.predict() if color == "red" else kalman_green.predict()
                if target_mm is not None and live_pos:
                    live_mm = wall_follower.lateral_offset_mm(live_pos[0], detections.get("walls", {}))
                    if live_mm is not None:
                        alignment_bias = max(-0.2, min(0.2, (target_mm - live_mm) / 1000.0))

            steering = steering_pid.compute(wall_error + pillar_action + alignment_bias)
            speed = speed_ctrl.compute(
                wall_error=wall_error,
                pillar_near=detections.get("pillar_near", False),
                corner_near=detections.get("corner_near", False),
                wall_ahead=detections.get("walls", {}).get("wall_ahead", False),
                lap=laps + 1,
                section_confidence=section_confidence
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
                        logger.info(f"Parking complete (result={park_out.get('result')})")
                        print(f"Parked ({park_out.get('result', 'unknown')}).")
                        break
        else:
            # ── Open Challenge: autonomous stop in the finish section ───────
            # Rule 9.24.2: after 3 laps the vehicle must stop, autonomously,
            # with its full projection inside the finish section. There is
            # no live "am I in the finish section" sensor here (no absolute
            # position reference beyond section/lap counting), so the
            # closest safe proxy is: stop as soon as lap 3 completes, since
            # the finish section is the section the car is in when the 3rd
            # lap completes (same as the starting section). This does not
            # replace verifying that behaviour on the real track — the
            # car's physical stopping distance after this point must be
            # measured so it actually settles inside the section, not past it.
            print("3 laps complete. Stopping in finish section.")
            motors.set_steering(0.0)
            motors.set_speed(0)
            logger.info("Open Challenge: stopped after lap 3 (finish-section proxy)")

        # ── Stop-and-hold: stay stationary once stopped ─────────────────────
        # Applies to both challenge types once driving/parking is done, so
        # the vehicle doesn't drift or get scored mid-motion.
        print(f"Holding position for {STOP_HOLD_SECONDS:.0f}s.")
        motors.set_steering(0.0)
        motors.set_speed(0)
        hold_start = time.time()
        while time.time() - hold_start < STOP_HOLD_SECONDS:
            motors.set_speed(0)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        camera.stop()
        motors.stop()
        imu.stop()
        start_switch.cleanup()
        logger.info("Pipeline stopped cleanly")
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
