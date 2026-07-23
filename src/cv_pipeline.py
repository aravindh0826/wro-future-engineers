"""
CV Pipeline — HSV masking, contour detection, wall/pillar/parking distance estimation.
Loads HSV ranges from config/hsv_values.json if present (falls back to defaults).
Loads camera calibration (focal length, distortion) from config/camera_calibration.json.
"""

import cv2
import numpy as np
import json
import os
import logging

logger = logging.getLogger(__name__)

_BASE_DIR = os.path.dirname(os.path.abspath(__file__))
HSV_CONFIG_PATH    = os.path.join(_BASE_DIR, "..", "config", "hsv_values.json")
CAMERA_CONFIG_PATH = os.path.join(_BASE_DIR, "..", "config", "camera_calibration.json")

# ── Default HSV ranges (overridden by config/hsv_values.json if present) ────
DEFAULT_HSV = {
    "red1":    {"lower": [0,   150, 80],  "upper": [8,   255, 255]},
    "red2":    {"lower": [172, 150, 80],  "upper": [180, 255, 255]},
    "green":   {"lower": [40,  80,  40],  "upper": [90,  255, 255]},
    "orange":  {"lower": [9,   140, 150], "upper": [25,  255, 255]},
    "blue":    {"lower": [100, 80,  60],  "upper": [130, 255, 255]},
    "black":   {"lower": [0,   0,   0],   "upper": [180, 255, 60]},
    # True magenta RGB(255,0,255) -> hue 150. Kept narrow (135-165) so it
    # doesn't overlap red2 (172-180, red pillar hue is 178).
    "magenta": {"lower": [135, 100, 100], "upper": [165, 255, 255]},
}

# ── Size filters (px at 320x240) ─────────────────────────────────────────────
MIN_PILLAR_AREA  = 1200
MAX_PILLAR_AREA  = 50000
MIN_WALL_AREA    = 500
MIN_LINE_AREA    = 3000
MIN_PARKING_AREA = 300
MAX_PARKING_AREA = 20000

# Real-world dimensions (mm) — per WRO 2026 rules 13.19 / 13.25
PILLAR_REAL_WIDTH_MM   = 50.0
PILLAR_REAL_HEIGHT_MM  = 100.0   # used for distance — invariant to pillar yaw
PARKING_REAL_HEIGHT_MM = 100.0
WALL_REAL_HEIGHT_MM    = 100.0   # rule 13.3

# Default focal length for Pi Camera Module Rev 1.3 (OV5647, f=3.6mm) at 320px
# capture width: f_px = f_mm * width_px / sensor_width_mm = 3.6 * 320 / 3.63.
# This is a geometric estimate only — run focal_calibrator.py against the real
# camera/mount to get an empirical value and it will be picked up automatically.
DEFAULT_FOCAL_LENGTH_PX = 317.4

# Inner wall configs — only variable in the Open Challenge (rule 13.16).
# Obstacle Challenge is always fixed 1000x1000 (square).
INNER_WALL_CONFIGS_MM = {
    "square":    (1000, 1000),
    "rect_1400": (1400, 1000),
    "rect_1800": (1800, 1000),
}


def _load_json(path):
    if os.path.exists(path):
        try:
            with open(path) as f:
                return json.load(f)
        except Exception as e:
            logger.warning(f"Could not load {path}: {e}")
    return {}


class CVPipeline:
    def __init__(self, mode="obstacle"):
        """
        Args:
            mode : "obstacle" or "open" — controls which detectors run.
                   Obstacle Challenge: fixed 1000mm corridor, pillars + parking active.
                   Open Challenge: variable corridor/inner-wall, no pillars/parking.
        """
        self.mode = mode

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        cv2.ocl.setUseOpenCL(True)
        cv2.setNumThreads(4)

        self._load_hsv_config()
        self._load_camera_config()

        logger.info(f"CVPipeline initialised (mode={mode}, focal_px={self.focal_length_px})")

    # ── Config loading ───────────────────────────────────────────────────────
    def _load_hsv_config(self):
        cfg = _load_json(HSV_CONFIG_PATH)
        merged = {**DEFAULT_HSV, **cfg}

        def rng(key):
            v = merged[key]
            return (np.array(v["lower"], dtype=np.uint8), np.array(v["upper"], dtype=np.uint8))

        self.hsv_red1    = rng("red1")
        self.hsv_red2    = rng("red2")
        self.hsv_green   = rng("green")
        self.hsv_orange  = rng("orange")
        self.hsv_blue    = rng("blue")
        self.hsv_black   = rng("black")
        self.hsv_magenta = rng("magenta")

    def _load_camera_config(self):
        cfg = _load_json(CAMERA_CONFIG_PATH)
        self.focal_length_px = cfg.get("focal_length_px", DEFAULT_FOCAL_LENGTH_PX)

        self.camera_matrix = None
        self.dist_coeffs   = None
        if cfg.get("camera_matrix") and cfg.get("dist_coeffs"):
            self.camera_matrix = np.array(cfg["camera_matrix"], dtype=np.float64)
            self.dist_coeffs   = np.array(cfg["dist_coeffs"], dtype=np.float64)

    # ── Main process ──────────────────────────────────────────────────────────
    def process(self, frame):
        """
        Returns:
            {
                red_pillar   : (cx, cy, area, distance_mm) or None,
                green_pillar : (cx, cy, area, distance_mm) or None,
                walls        : {left, right, top, center_error, inner_wall_config, wall_ahead},
                lines        : {orange: bool, blue: bool, magenta: bool},
                parking_markers : [(cx, cy, w, h, distance_mm), ...]  (0-2 entries),
                pillar_near  : bool,
                corner_near  : bool,
                parking_near : bool,
            }
        """
        if self.camera_matrix is not None:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        h, w = frame.shape[:2]
        preprocessed = self._preprocess(frame)
        hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)

        mask_black   = self._color_mask(hsv, self.hsv_black, symmetric=True)
        mask_orange  = self._color_mask(hsv, self.hsv_orange)
        mask_blue    = self._color_mask(hsv, self.hsv_blue)
        mask_magenta = self._magenta_mask(hsv)

        walls = self._detect_walls(mask_black[h // 2:, :], frame_w=w, frame_h=h // 2)
        lines = self._detect_lines(mask_orange, mask_blue, mask_magenta)

        red_pillar, green_pillar = None, None
        if self.mode == "obstacle":
            mask_red   = self._red_mask(hsv)
            mask_green = self._color_mask(hsv, self.hsv_green)
            red_pillar   = self._detect_pillar(mask_red,   label="red")
            green_pillar = self._detect_pillar(mask_green, label="green")

        parking_markers = []
        if self.mode == "obstacle":
            parking_markers = self._detect_parking_markers(mask_magenta)

        pillar_near  = self._is_pillar_near(red_pillar, green_pillar, threshold_mm=400)
        corner_near  = lines["orange"] or lines["blue"]
        parking_near = lines["magenta"]

        return {
            "red_pillar":      red_pillar,
            "green_pillar":    green_pillar,
            "walls":           walls,
            "lines":           lines,
            "parking_markers": parking_markers,
            "pillar_near":     pillar_near,
            "corner_near":     corner_near,
            "parking_near":    parking_near,
        }

    # ── Pillar action ─────────────────────────────────────────────────────────
    def get_pillar_action(self, red_pos, green_pos, recalled):
        red   = red_pos   or (recalled.get("red")   if recalled else None)
        green = green_pos or (recalled.get("green") if recalled else None)

        if red is None and green is None:
            return 0.0

        frame_center = 160

        if red and not green:
            return (frame_center - red[0]) * 0.003
        if green and not red:
            return (frame_center - green[0]) * -0.003
        if red and green:
            mid = (red[0] + green[0]) / 2
            return (frame_center - mid) * 0.003
        return 0.0

    # ── Preprocessing ─────────────────────────────────────────────────────────
    def _preprocess(self, frame):
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self.clahe.apply(l)
        lab = cv2.merge([l, a, b])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # ── Color masking ─────────────────────────────────────────────────────────
    def _red_mask(self, hsv):
        m1 = cv2.inRange(hsv, self.hsv_red1[0], self.hsv_red1[1])
        m2 = cv2.inRange(hsv, self.hsv_red2[0], self.hsv_red2[1])
        return self._clean_mask(cv2.bitwise_or(m1, m2))

    def _magenta_mask(self, hsv):
        mask = cv2.inRange(hsv, self.hsv_magenta[0], self.hsv_magenta[1])
        return self._clean_mask(mask)

    def _color_mask(self, hsv, hsv_range, symmetric=False):
        mask = cv2.inRange(hsv, hsv_range[0], hsv_range[1])
        return self._clean_mask_symmetric(mask) if symmetric else self._clean_mask(mask)

    def _clean_mask(self, mask):
        mask = cv2.erode(mask,  self.kernel_small, iterations=1)
        mask = cv2.dilate(mask, self.kernel_large, iterations=2)
        return mask

    def _clean_mask_symmetric(self, mask):
        """
        Same denoise/gap-fill purpose as _clean_mask, but erode/dilate use
        matching kernel+iterations so they roughly cancel out in size rather
        than growing it. Needed for measurement-sensitive masks (walls) where
        the asymmetric erode1+dilate2 combo measurably inflates apparent
        pixel height on thin/distant objects, which corrupts the
        height-based distance formula (distance ~ 1/height).
        """
        mask = cv2.erode(mask,  self.kernel_small, iterations=1)
        mask = cv2.dilate(mask, self.kernel_small, iterations=1)
        return mask

    # ── Pillar detection ──────────────────────────────────────────────────────
    def _detect_pillar(self, mask, label=""):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_h = mask.shape[0]
        best, best_area = None, 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            min_area = 1500 if label == "green" else MIN_PILLAR_AREA
            if area < min_area or area > MAX_PILLAR_AREA:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect = h / max(w, 1)
            if aspect < 0.4:
                continue

            if area > best_area:
                best_area = area
                M  = cv2.moments(cnt)
                cx = int(M["m10"] / M["m00"]) if M["m00"] else x + w // 2
                cy = int(M["m01"] / M["m00"]) if M["m00"] else y + h // 2
                clipped = y <= 1 or (y + h) >= frame_h - 1
                dist = self._estimate_distance(w, h, clipped)
                best = (cx, cy, area, dist)

        return best

    # ── Parking marker detection ──────────────────────────────────────────────
    def _detect_parking_markers(self, mask):
        """Detect up to 2 magenta parking-lot blocks (200x20x100mm, rule 13.25)."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_h = mask.shape[0]
        candidates = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_PARKING_AREA or area > MAX_PARKING_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            M  = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"]) if M["m00"] else x + w // 2
            cy = int(M["m01"] / M["m00"]) if M["m00"] else y + h // 2
            clipped = y <= 1 or (y + h) >= frame_h - 1
            dist = self._estimate_distance(w, h, clipped, real_height_mm=PARKING_REAL_HEIGHT_MM)
            candidates.append((cx, cy, w, h, dist))

        candidates.sort(key=lambda c: c[4])
        return candidates[:2]

    # ── Wall detection ────────────────────────────────────────────────────────
    def _detect_walls(self, mask, frame_w, frame_h):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        left_x, right_x, top_y = 0, frame_w, 0
        inner_wall_config = "square"
        best_inner_width = 0
        frame_mid = frame_w / 2.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_WALL_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx_cnt = x + w / 2.0

            # Every wall blob is assigned left/right by centroid side — no dead
            # zone in the middle, so an inner-wall corner sitting dead-center
            # in frame (common when approaching it head-on) still bounds the
            # corridor correctly instead of being ignored.
            if cx_cnt < frame_mid:
                left_x = max(left_x, x + w)
            else:
                right_x = min(right_x, x)

            if y < frame_h * 0.3:
                top_y = max(top_y, y + h)

            if self.mode == "open":
                clipped = y <= 1 or (y + h) >= frame_h - 1
                if (not clipped and frame_w * 0.2 < cx_cnt < frame_w * 0.8 and
                        w > best_inner_width and w > 60):
                    best_inner_width = w
                    inner_wall_config = self._classify_wall_length(w, h)

        return {
            "left":              left_x,
            "right":             right_x,
            "top":               top_y,
            "center_error":      (frame_w // 2) - ((left_x + right_x) // 2),
            "inner_wall_config": inner_wall_config,
            "wall_ahead":        top_y > frame_h * 0.5,
        }

    def _classify_wall_length(self, pixel_width, pixel_height):
        """
        Classify a visible inner-wall segment as 1000/1400/1800mm (rule 13.16,
        Open Challenge only) using the same size-from-distance physics as
        pillar distance estimation: wall height is a fixed 100mm (rule 13.3),
        so distance = (100 * f_px) / pixel_height, then real_width follows
        from pixel_width and that distance. Far more reliable than an
        aspect-ratio guess, which varies with viewing angle, not real size.
        """
        if pixel_height <= 0:
            return "square"
        distance_mm = (WALL_REAL_HEIGHT_MM * self.focal_length_px) / pixel_height
        real_width_mm = (pixel_width * distance_mm) / self.focal_length_px

        candidates = {"square": 1000, "rect_1400": 1400, "rect_1800": 1800}
        return min(candidates, key=lambda k: abs(candidates[k] - real_width_mm))

    # ── Line detection ────────────────────────────────────────────────────────
    def _detect_lines(self, mask_orange, mask_blue, mask_magenta):
        orange_detected  = cv2.countNonZero(mask_orange)  > 1200
        blue_detected    = cv2.countNonZero(mask_blue)    > 1500
        magenta_detected = cv2.countNonZero(mask_magenta) > 800
        return {"orange": orange_detected, "blue": blue_detected, "magenta": magenta_detected}

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _estimate_distance(self, pixel_width, pixel_height, clipped=False, real_height_mm=None):
        """
        Distance from pixel height (yaw-invariant, real height fixed per rule
        13.19/13.25). Falls back to width if the box is clipped by the frame
        edge (height unreliable in that case).
        """
        real_h = real_height_mm if real_height_mm is not None else PILLAR_REAL_HEIGHT_MM
        if not clipped and pixel_height > 0:
            return (real_h * self.focal_length_px) / pixel_height
        if pixel_width > 0:
            return (PILLAR_REAL_WIDTH_MM * self.focal_length_px) / pixel_width
        return 9999

    def _is_pillar_near(self, red, green, threshold_mm=400):
        if red   and red[3]   < threshold_mm:
            return True
        if green and green[3] < threshold_mm:
            return True
        return False

    # ── Debug visualisation ───────────────────────────────────────────────────
    def draw_debug(self, frame, detections):
        h, w = frame.shape[:2]
        debug = frame.copy()

        for label, color, key in [
            ("RED",   (0, 0, 255), "red_pillar"),
            ("GREEN", (0, 255, 0), "green_pillar"),
        ]:
            det = detections.get(key)
            if det:
                cx, cy, area, dist = det
                cv2.circle(debug, (cx, cy), 10, color, 2)
                cv2.putText(debug, f"{label} {dist:.0f}mm", (cx - 20, cy - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        for (cx, cy, pw, ph, dist) in detections.get("parking_markers", []):
            cv2.rectangle(debug, (cx - pw // 2, cy - ph // 2), (cx + pw // 2, cy + ph // 2),
                          (255, 0, 255), 2)
            cv2.putText(debug, f"PARK {dist:.0f}mm", (cx - 20, cy - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

        walls = detections.get("walls", {})
        if walls:
            err = walls.get("center_error", 0)
            cv2.putText(debug, f"Wall err: {err}px", (5, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        lines = detections.get("lines", {})
        if lines.get("orange"):
            cv2.putText(debug, "ORANGE LINE", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        if lines.get("blue"):
            cv2.putText(debug, "BLUE LINE", (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)
        if lines.get("magenta"):
            cv2.putText(debug, "PARKING MARKER", (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        return debug