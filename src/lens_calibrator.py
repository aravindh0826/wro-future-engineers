"""
Lens Distortion Calibrator (optional)
Standard OpenCV checkerboard calibration for the Pi Camera Module Rev 1.3
lens. Saves camera_matrix + dist_coeffs into config/camera_calibration.json
so CVPipeline can undistort frames before detection.

Print a 9x6 internal-corner checkerboard, hold it in front of the mounted
camera at several angles/distances, press 'c' to capture each pose
(aim for 15-20), then 'q' to compute and save.

Usage:
    python lens_calibrator.py --source 0
"""

import cv2
import numpy as np
import json
import os
import argparse
from cv_pipeline import CAMERA_CONFIG_PATH

BOARD_SIZE = (9, 6)


def run(source):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Cannot open source: {source}")
        return

    objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)

    objpoints, imgpoints = [], []
    frame_shape = None

    print("Press 'c' to capture a pose, 'q' to finish and calibrate.")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        frame = cv2.resize(frame, (320, 240))
        frame_shape = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

        disp = frame.copy()
        if found:
            cv2.drawChessboardCorners(disp, BOARD_SIZE, corners, found)
        cv2.putText(disp, f"captures={len(objpoints)}", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow("Lens Calibrator", disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') and found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners)
            print(f"Captured pose {len(objpoints)}")
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(objpoints) < 8:
        print(f"Only {len(objpoints)} captures — need at least 8-10 for a stable result. Aborting.")
        return

    ret, mtx, dist, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints, frame_shape[::-1], None, None
    )
    print(f"Calibration RMS error: {ret:.3f}")
    _save(mtx.tolist(), dist.tolist())


def _save(camera_matrix, dist_coeffs):
    cfg = {}
    if os.path.exists(CAMERA_CONFIG_PATH):
        with open(CAMERA_CONFIG_PATH) as fp:
            cfg = json.load(fp)
    cfg["camera_matrix"] = camera_matrix
    cfg["dist_coeffs"]   = dist_coeffs
    os.makedirs(os.path.dirname(CAMERA_CONFIG_PATH), exist_ok=True)
    with open(CAMERA_CONFIG_PATH, "w") as fp:
        json.dump(cfg, fp, indent=2)
    print(f"Saved camera_matrix/dist_coeffs to {CAMERA_CONFIG_PATH}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=0)
    args = parser.parse_args()
    try:
        source = int(args.source)
    except ValueError:
        source = args.source
    run(source)
