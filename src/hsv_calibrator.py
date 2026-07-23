"""
HSV Calibration Tool
Live trackbar tool to tune HSV ranges for each color.
Run this on PC with a webcam or video file to find
the right HSV values for your specific lighting conditions.

Usage:
    python hsv_calibrator.py
    python hsv_calibrator.py --source path/to/video.mp4
    python hsv_calibrator.py --color green
"""

import cv2
import numpy as np
import argparse
import json
import os


# Default starting values per color
DEFAULTS = {
    "red1":    {"hl": 0,   "sl": 120, "vl": 70,  "hu": 10,  "su": 255, "vu": 255},
    "red2":    {"hl": 170, "sl": 120, "vl": 70,  "hu": 180, "su": 255, "vu": 255},
    "green":   {"hl": 40,  "sl": 80,  "vl": 40,  "hu": 90,  "su": 255, "vu": 255},
    "orange":  {"hl": 10,  "sl": 150, "vl": 150, "hu": 25,  "su": 255, "vu": 255},
    "blue":    {"hl": 100, "sl": 100, "vl": 50,  "hu": 130, "su": 255, "vu": 255},
    "black":   {"hl": 0,   "sl": 0,   "vl": 0,   "hu": 180, "su": 255, "vu": 60},
    "magenta": {"hl": 135, "sl": 100, "vl": 100, "hu": 165, "su": 255, "vu": 255},
}

SAVE_PATH = "../config/hsv_values.json"


def nothing(x):
    pass


def create_trackbars(window, defaults):
    cv2.createTrackbar("H Low",  window, defaults["hl"], 180, nothing)
    cv2.createTrackbar("S Low",  window, defaults["sl"], 255, nothing)
    cv2.createTrackbar("V Low",  window, defaults["vl"], 255, nothing)
    cv2.createTrackbar("H High", window, defaults["hu"], 180, nothing)
    cv2.createTrackbar("S High", window, defaults["su"], 255, nothing)
    cv2.createTrackbar("V High", window, defaults["vu"], 255, nothing)


def get_trackbar_values(window):
    hl = cv2.getTrackbarPos("H Low",  window)
    sl = cv2.getTrackbarPos("S Low",  window)
    vl = cv2.getTrackbarPos("V Low",  window)
    hu = cv2.getTrackbarPos("H High", window)
    su = cv2.getTrackbarPos("S High", window)
    vu = cv2.getTrackbarPos("V High", window)
    return np.array([hl, sl, vl]), np.array([hu, su, vu])


def run_calibrator(source, color):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Cannot open source: {source}")
        return

    win_name = f"HSV Calibrator — {color}"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 800, 600)

    defaults = DEFAULTS.get(color, DEFAULTS["green"])
    create_trackbars(win_name, defaults)

    # CLAHE for preprocessing
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    print(f"\nCalibrating HSV for: {color}")
    print("Press 'S' to save values, 'Q' to quit\n")

    saved_values = {}

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        frame = cv2.resize(frame, (640, 480))

        # Preprocess
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = clahe.apply(l)
        lab = cv2.merge([l, a, b])
        preprocessed = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)

        # Get current trackbar values
        lower, upper = get_trackbar_values(win_name)

        # Apply mask
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask,  kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Result overlay
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Info overlay
        info = f"H:[{lower[0]}-{upper[0]}] S:[{lower[1]}-{upper[1]}] V:[{lower[2]}-{upper[2]}]"
        cv2.putText(result, info, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(result, "S=save  Q=quit", (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Side by side display
        combined = np.hstack([frame, result])
        cv2.imshow(win_name, combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            saved_values[color] = {
                "lower": lower.tolist(),
                "upper": upper.tolist()
            }
            _save(saved_values)
            print(f"Saved {color}: lower={lower.tolist()} upper={upper.tolist()}")

    cap.release()
    cv2.destroyAllWindows()
    return saved_values


def _save(values):
    os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
    existing = {}
    if os.path.exists(SAVE_PATH):
        with open(SAVE_PATH) as f:
            existing = json.load(f)
    existing.update(values)
    with open(SAVE_PATH, "w") as f:
        json.dump(existing, f, indent=2)
    print(f"Saved to {SAVE_PATH}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default=0,
                        help="Video source: 0=webcam, or path to video file")
    parser.add_argument("--color",  default="red1",
                        choices=list(DEFAULTS.keys()),
                        help="Color to calibrate")
    args = parser.parse_args()

    try:
        source = int(args.source)
    except ValueError:
        source = args.source

    run_calibrator(source, args.color)
