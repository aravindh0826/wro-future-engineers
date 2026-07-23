"""
Corridor Width Calibrator
Place the car centered in a real 1000mm (wide) or 600mm (narrow) corridor,
run this, and press 's' to save the measured pixel corridor width into
config/corridor_calibration.json.

Usage:
    python corridor_calibrator.py --type wide
    python corridor_calibrator.py --type narrow --source video.mp4
"""

import cv2
import json
import os
import argparse
from cv_pipeline import CVPipeline, CAMERA_CONFIG_PATH

CORRIDOR_CONFIG_PATH = os.path.join(os.path.dirname(CAMERA_CONFIG_PATH), "corridor_calibration.json")


def run(source, corridor_type):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Cannot open source: {source}")
        return

    pipeline = CVPipeline(mode="open")
    print(f"Calibrating '{corridor_type}' corridor. Press 's' to save, 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        frame = cv2.resize(frame, (320, 240))

        hsv = cv2.cvtColor(pipeline._preprocess(frame), cv2.COLOR_BGR2HSV)
        mask_black = pipeline._color_mask(hsv, pipeline.hsv_black, symmetric=True)
        h = frame.shape[0]
        walls = pipeline._detect_walls(mask_black[h // 2:, :], frame_w=320, frame_h=h // 2)

        corridor_px = walls["right"] - walls["left"]
        disp = frame.copy()
        cv2.line(disp, (walls["left"], h - 10), (walls["left"], h), (0, 255, 255), 2)
        cv2.line(disp, (walls["right"], h - 10), (walls["right"], h), (0, 255, 255), 2)
        cv2.putText(disp, f"corridor={corridor_px}px  left={walls['left']} right={walls['right']}",
                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow("Corridor Calibrator", disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            _save(corridor_type, corridor_px)
            print(f"Saved {corridor_type}_corridor_px = {corridor_px}")

    cap.release()
    cv2.destroyAllWindows()


def _save(corridor_type, px):
    cfg = {}
    if os.path.exists(CORRIDOR_CONFIG_PATH):
        with open(CORRIDOR_CONFIG_PATH) as fp:
            cfg = json.load(fp)
    key = "wide_corridor_px" if corridor_type == "wide" else "narrow_corridor_px"
    cfg[key] = px
    os.makedirs(os.path.dirname(CORRIDOR_CONFIG_PATH), exist_ok=True)
    with open(CORRIDOR_CONFIG_PATH, "w") as fp:
        json.dump(cfg, fp, indent=2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=0)
    parser.add_argument("--type", default="wide", choices=["wide", "narrow"])
    args = parser.parse_args()
    try:
        source = int(args.source)
    except ValueError:
        source = args.source
    run(source, args.type)