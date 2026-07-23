"""
Focal Length Calibrator
Place a red or green pillar at a known distance in front of the mounted
camera, run this, and press 's' once the bounding box looks clean to
compute and save focal_length_px into config/camera_calibration.json.

f_px = (pixel_height * real_distance_mm) / real_height_mm

Usage:
    python focal_calibrator.py --distance 300 --color red
    python focal_calibrator.py --distance 300 --color red --source video.mp4
"""

import cv2
import json
import os
import argparse
from cv_pipeline import CVPipeline, PILLAR_REAL_HEIGHT_MM, CAMERA_CONFIG_PATH
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
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Cannot open source: {source}")
        return

    pipeline = CVPipeline(mode="obstacle")
    print(f"Measuring at {distance_mm}mm, color={color}. Press 's' to save, 'q' to quit.")

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

        mask_view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([disp, mask_view])
        cv2.imshow("Focal Calibrator (frame | mask)", combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and box is not None and not clipped:
            _save(f_px)
            print(f"Saved focal_length_px = {f_px:.1f}")

    cap.release()
    cv2.destroyAllWindows()

def _save(f_px):
    cfg = {}
    if os.path.exists(CAMERA_CONFIG_PATH):
        with open(CAMERA_CONFIG_PATH) as fp:
            cfg = json.load(fp)
    cfg["focal_length_px"] = round(f_px, 1)
    os.makedirs(os.path.dirname(CAMERA_CONFIG_PATH), exist_ok=True)
    with open(CAMERA_CONFIG_PATH, "w") as fp:
        json.dump(cfg, fp, indent=2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=0)
    parser.add_argument("--distance", type=float, required=True, help="known distance to pillar in mm")
    parser.add_argument("--color", default="red", choices=["red", "green"])
    args = parser.parse_args()

    try:
        source = int(args.source)
    except ValueError:
        source = args.source

    run(source, args.distance, args.color)
