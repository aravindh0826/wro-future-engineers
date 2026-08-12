"""
Focal Length Calibrator
Place a red or green pillar at a known distance in front of the mounted
camera, run this, and press 's' to record a sample once the bounding box
looks clean. Each 's' press ADDS a sample rather than overwriting the
saved value immediately -- move the pillar to a different known distance
between presses (e.g. 200, 300, 400, 500mm) and press 's' again at each
one, then press 'a' to average all collected samples and save. A
single-point calibration is more sensitive to measurement noise (a few mm
of ruler error shifts the result more than it would with averaging).

f_px = (pixel_height * real_distance_mm) / real_height_mm

Run on the Pi with no --source to use the Pi Camera directly (via
frame_source.FrameSource / picamera2, same path as main.py). Pass
--source to calibrate against a webcam or a recorded video file instead.

Usage:
    python focal_calibrator.py --distance 300 --color red             # Pi Camera
    python focal_calibrator.py --distance 300 --color red --source video.mp4

Controls:
    s = record a sample at the --distance given on the command line
    a = average all recorded samples and save to config/camera_calibration.json
    q = quit without saving (unless 'a' was already pressed)
"""

import cv2
import json
import os
import argparse
from cv_pipeline import CVPipeline, PILLAR_REAL_HEIGHT_MM, CAMERA_CONFIG_PATH
from frame_source import FrameSource
import numpy as np


def largest_box(mask, min_area=800):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, min_area
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > best_area:
            best_area = area
            best = cv2.boundingRect(cnt)
    return best


def run(source, distance_mm, color):
    cap = FrameSource(source)
    if not cap.isOpened():
        print(f"Cannot open source: {source}")
        return

    pipeline = CVPipeline(mode="obstacle")
    samples = []   # list of f_px values recorded this session
    print(f"Measuring at {distance_mm}mm, color={color}.")
    print("Move the pillar to a few different known distances, pressing 's' at each.")
    print("Press 'a' when done to average all samples and save. Press 'q' to abort.")

    box, clipped, f_px = None, False, None

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        frame = cv2.resize(frame, (320, 240))

        pre = pipeline._preprocess(frame)
        hsv = cv2.cvtColor(pre, cv2.COLOR_BGR2HSV)
        mask = pipeline._red_mask(hsv) if color == "red" else pipeline._color_mask(hsv, pipeline.hsv_green)

        mask_pixels = cv2.countNonZero(mask)
        box = largest_box(mask)
        disp = frame.copy()

        if box is None:
            clipped = False
            cv2.putText(disp, f"NO OBJECT DETECTED (mask px={mask_pixels})", (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(disp, "Check color/lighting, or object is too small/far", (5, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
        else:
            x, y, w, h = box
            frame_h = frame.shape[0]
            clipped = y <= 1 or (y + h) >= frame_h - 1
            color_box = (0, 0, 255) if clipped else (0, 255, 255)
            cv2.rectangle(disp, (x, y), (x + w, y + h), color_box, 2)

            if clipped:
                cv2.putText(disp, "CLIPPED - move object back / further from camera", (5, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            else:
                f_px = (h * distance_mm) / PILLAR_REAL_HEIGHT_MM
                est_dist_mm = (PILLAR_REAL_HEIGHT_MM * pipeline.focal_length_px) / h if h > 0 else 0
                cv2.putText(disp, f"h={h}px  new_f={f_px:.1f}px", (5, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(disp, f"current calib says: {est_dist_mm:.0f}mm  (you placed it at {distance_mm:.0f}mm)",
                            (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)

        cv2.putText(disp, f"samples recorded: {len(samples)}  (press 's' to add, 'a' to average+save)",
                    (5, disp.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        mask_view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([disp, mask_view])
        cv2.imshow("Focal Calibrator (frame | mask)", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Quit without saving." if not samples else
                  f"Quit -- {len(samples)} sample(s) recorded but NOT saved (press 'a' next time to save).")
            break
        elif key == ord('s'):
            if box is not None and not clipped and f_px is not None:
                samples.append(f_px)
                print(f"Sample {len(samples)} recorded: f_px={f_px:.1f} at distance={distance_mm:.0f}mm")
            else:
                print("No clean (unclipped) detection right now -- sample not recorded.")
        elif key == ord('a'):
            if not samples:
                print("No samples recorded yet -- press 's' at 1+ distances first.")
                continue
            avg_f_px = sum(samples) / len(samples)
            _save(avg_f_px, n_samples=len(samples))
            print(f"Saved focal_length_px = {avg_f_px:.1f} (averaged over {len(samples)} sample(s))")
            break

    cap.release()
    cv2.destroyAllWindows()

def _save(f_px, n_samples=1):
    cfg = {}
    if os.path.exists(CAMERA_CONFIG_PATH):
        with open(CAMERA_CONFIG_PATH) as fp:
            cfg = json.load(fp)
    cfg["focal_length_px"] = round(f_px, 1)
    cfg["focal_length_calibration_samples"] = n_samples
    os.makedirs(os.path.dirname(CAMERA_CONFIG_PATH), exist_ok=True)
    with open(CAMERA_CONFIG_PATH, "w") as fp:
        json.dump(cfg, fp, indent=2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=None,
                        help="Omit to use the Pi Camera (picamera2) on a "
                             "real Pi; otherwise pass a webcam index or a "
                             "video file path")
    parser.add_argument("--distance", type=float, required=True, help="known distance to pillar in mm")
    parser.add_argument("--color", default="red", choices=["red", "green"])
    args = parser.parse_args()

    if args.source is None:
        source = None
    else:
        try:
            source = int(args.source)
        except ValueError:
            source = args.source

    run(source, args.distance, args.color)
