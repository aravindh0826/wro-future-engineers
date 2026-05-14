"""
CV Pipeline
Full OpenCV detection pipeline:
- CLAHE preprocessing (lighting normalisation)
- HSV color masking for red, green, orange, blue, black, magenta
- Morphological noise removal
- Contour detection with shape + aspect ratio filtering
- Wall distance estimation + inner wall config detection
- Pillar side and distance estimation
- Corner detection via orange/blue lines
- Parking marker detection via magenta lines
"""

import cv2
import numpy as np
import logging

logger = logging.getLogger(__name__)

# ── HSV Ranges (based on official WRO mat colors) ────────────────────────────
# Format: (lower, upper) in HSV

HSV_RED_1 = (np.array([0,  150, 80]), np.array([8,  255, 255]))
HSV_RED_2 = (np.array([172, 150, 80]), np.array([180, 255, 255]))
HSV_GREEN   = (np.array([40,  80,   40]),  np.array([90,  255, 255]))
HSV_ORANGE = (np.array([9, 140, 150]), np.array([25, 255, 255]))
HSV_BLUE = (np.array([100, 80, 60]), np.array([130, 255, 255]))
HSV_BLACK   = (np.array([0,   0,    0]),   np.array([180, 255,  60]))
# Magenta — parking zone markers (WRO spec: RGB 255,0,255)
# Magenta wraps the hue wheel so two ranges are needed (like red).
HSV_MAGENTA_1 = (np.array([140, 100, 100]), np.array([160, 255, 255]))
HSV_MAGENTA_2 = (np.array([160, 100, 100]), np.array([180, 255, 255]))

# ── Size filters (px at 320x240) ──────────────────────────────────────────────
MIN_PILLAR_AREA     = 1200
MAX_PILLAR_AREA = 50000
MIN_WALL_AREA       = 500
MIN_LINE_AREA = 3000

# Real world pillar dimensions (mm) — used for distance estimation
PILLAR_REAL_WIDTH_MM  = 50.0
FOCAL_LENGTH_PX       = 280.0   # calibrate with: f = (P * D) / W

# ── Inner wall configurations (WRO 2025 rule 9.19) ───────────────────────────
# Inner wall can be 1000×1000, 1400×1000 or 1800×1000 mm.
# We track the *aspect ratio* of the inner wall bounding box to figure out
# which configuration we're in.
INNER_WALL_CONFIGS_MM = {
    "square":     (1000, 1000),   # aspect ~1.00
    "rect_1400":  (1400, 1000),   # aspect ~1.40
    "rect_1800":  (1800, 1000),   # aspect ~1.80
}


class CVPipeline:
    def __init__(self):
        # CLAHE for lighting normalisation
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Morphological kernels
        self.kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # Enable OpenCL GPU acceleration if available
        cv2.ocl.setUseOpenCL(True)
        cv2.setNumThreads(4)

        logger.info("CVPipeline initialised")

    # ── Main process ──────────────────────────────────────────────────────────
    def process(self, frame):
        """
        Process a BGR frame and return detection dict.
        Returns:
            {
                red_pillar  : (cx, cy, area, distance_mm) or None,
                green_pillar: (cx, cy, area, distance_mm) or None,
                walls       : {left, right, top, center_error, inner_wall_config},
                lines       : {orange: bool, blue: bool, magenta: bool},
                pillar_near : bool,
                corner_near : bool,
                parking_near: bool,
            }
        """
        h, w = frame.shape[:2]

        # ── 1. Preprocess ─────────────────────────────────────────────────
        preprocessed = self._preprocess(frame)

        # ── 2. HSV conversion ─────────────────────────────────────────────
        hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)

        # ── 3. Color masks ────────────────────────────────────────────────
        mask_red     = self._red_mask(hsv)
        mask_green   = self._color_mask(hsv, HSV_GREEN)
        mask_orange  = self._color_mask(hsv, HSV_ORANGE)
        mask_blue    = self._color_mask(hsv, HSV_BLUE)
        mask_black   = self._color_mask(hsv, HSV_BLACK)
        mask_magenta = self._magenta_mask(hsv)

        # ── 4. Detections ─────────────────────────────────────────────────
        red_pillar   = self._detect_pillar(mask_red,   label="red")
        green_pillar = self._detect_pillar(mask_green, label="green")

        walls        = self._detect_walls(mask_black[h//2:, :], frame_w=w, frame_h=h//2)
        lines        = self._detect_lines(mask_orange, mask_blue, mask_magenta)

        pillar_near  = self._is_pillar_near(red_pillar, green_pillar, threshold_mm=400)
        corner_near  = lines["orange"] or lines["blue"]
        parking_near = lines["magenta"]

        return {
            "red_pillar":   red_pillar,
            "green_pillar": green_pillar,
            "walls":        walls,
            "lines":        lines,
            "pillar_near":  pillar_near,
            "corner_near":  corner_near,
            "parking_near": parking_near,
        }

    # ── Pillar action ─────────────────────────────────────────────────────────
    def get_pillar_action(self, red_pos, green_pos, recalled):
        """
        Returns a steering correction value based on pillar positions.
        Positive = steer right, Negative = steer left.
        Uses recalled memory from lap 1 if live detection is None.
        """
        red   = red_pos   or (recalled.get("red")   if recalled else None)
        green = green_pos or (recalled.get("green") if recalled else None)

        if red is None and green is None:
            return 0.0

        frame_center = 160  # half of 320px width

        if red and not green:
            # Red pillar → pass on RIGHT → steer right if pillar is left of center
            cx = red[0]
            return (frame_center - cx) * 0.003

        if green and not red:
            # Green pillar → pass on LEFT → steer left if pillar is right of center
            cx = green[0]
            return (frame_center - cx) * -0.003

        if red and green:
            # Both visible → steer between them toward the gap
            mid = (red[0] + green[0]) / 2
            return (frame_center - mid) * 0.003

        return 0.0

    # ── Preprocessing ─────────────────────────────────────────────────────────
    def _preprocess(self, frame):
        """Apply CLAHE to L channel for lighting normalisation."""
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self.clahe.apply(l)
        lab = cv2.merge([l, a, b])
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # ── Color masking ─────────────────────────────────────────────────────────
    def _red_mask(self, hsv):
        """Red wraps around 0° so needs two ranges."""
        m1 = cv2.inRange(hsv, HSV_RED_1[0], HSV_RED_1[1])
        m2 = cv2.inRange(hsv, HSV_RED_2[0], HSV_RED_2[1])
        mask = cv2.bitwise_or(m1, m2)
        return self._clean_mask(mask)

    def _magenta_mask(self, hsv):
        """Magenta also wraps (high hue end) so needs two ranges."""
        m1 = cv2.inRange(hsv, HSV_MAGENTA_1[0], HSV_MAGENTA_1[1])
        m2 = cv2.inRange(hsv, HSV_MAGENTA_2[0], HSV_MAGENTA_2[1])
        mask = cv2.bitwise_or(m1, m2)
        return self._clean_mask(mask)

    def _color_mask(self, hsv, hsv_range):
        mask = cv2.inRange(hsv, hsv_range[0], hsv_range[1])
        return self._clean_mask(mask)

    def _clean_mask(self, mask):
        """Erode then dilate to remove noise."""
        mask = cv2.erode(mask,  self.kernel_small, iterations=1)
        mask = cv2.dilate(mask, self.kernel_large, iterations=2)
        return mask

    # ── Pillar detection ──────────────────────────────────────────────────────
    def _detect_pillar(self, mask, label=""):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        best = None
        best_area = 0

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
                dist = self._estimate_distance(w)
                best = (cx, cy, area, dist)

        return best

    # ── Wall detection ────────────────────────────────────────────────────────
    def _detect_walls(self, mask, frame_w, frame_h):
        """
        Estimate left, right, top wall distances in pixels.

        Also estimates inner wall configuration by tracking the largest
        central black contour's aspect ratio, mapping to the three allowed
        WRO inner wall sizes:
          1000×1000 mm (aspect ~1.0)
          1400×1000 mm (aspect ~1.4)
          1800×1000 mm (aspect ~1.8)
        """
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        left_x  = 0
        right_x = frame_w
        top_y   = 0

        inner_wall_config = "square"   # default assumption
        best_inner_area   = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_WALL_AREA:
                continue
            x, y, w, h = cv2.boundingRect(cnt)

            # Left wall
            if x < frame_w * 0.3:
                left_x = max(left_x, x + w)

            # Right wall
            if x + w > frame_w * 0.7:
                right_x = min(right_x, x)

            # Top wall
            if y < frame_h * 0.3:
                top_y = max(top_y, y + h)

            # Inner wall — largest central contour
            cx_cnt = x + w / 2
            if (frame_w * 0.2 < cx_cnt < frame_w * 0.8 and
                    area > best_inner_area and area > MIN_WALL_AREA * 10):
                best_inner_area = area
                aspect = w / max(h, 1)
                if aspect < 1.2:
                    inner_wall_config = "square"
                elif aspect < 1.6:
                    inner_wall_config = "rect_1400"
                else:
                    inner_wall_config = "rect_1800"

        return {
            "left":              left_x,
            "right":             right_x,
            "top":               top_y,
            "center_error":      (frame_w // 2) - ((left_x + right_x) // 2),
            "inner_wall_config": inner_wall_config,
        }

    # ── Line detection ────────────────────────────────────────────────────────
    def _detect_lines(self, mask_orange, mask_blue, mask_magenta):
        """Detect orange, blue and magenta section/parking lines."""
        orange_detected  = cv2.countNonZero(mask_orange)  > 1200
        blue_detected    = cv2.countNonZero(mask_blue)    > 1500
        magenta_detected = cv2.countNonZero(mask_magenta) > 800
        return {"orange": orange_detected, "blue": blue_detected,
                "magenta": magenta_detected}

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _estimate_distance(self, pixel_width):
        """Estimate distance to pillar in mm using known real width."""
        if pixel_width <= 0:
            return 9999
        return (PILLAR_REAL_WIDTH_MM * FOCAL_LENGTH_PX) / pixel_width

    def _is_pillar_near(self, red, green, threshold_mm=400):
        if red   and red[3]   < threshold_mm:
            return True
        if green and green[3] < threshold_mm:
            return True
        return False

    # ── Debug visualisation ───────────────────────────────────────────────────
    def draw_debug(self, frame, detections):
        """Draw detection overlays onto frame for debugging."""
        h, w = frame.shape[:2]
        debug = frame.copy()

        for label, color, key in [
            ("RED",   (0, 0, 255),   "red_pillar"),
            ("GREEN", (0, 255, 0),   "green_pillar"),
        ]:
            det = detections.get(key)
            if det:
                cx, cy, area, dist = det
                cv2.circle(debug, (cx, cy), 10, color, 2)
                cv2.putText(debug, f"{label} {dist:.0f}mm",
                            (cx - 20, cy - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        walls = detections.get("walls", {})
        if walls:
            err = walls.get("center_error", 0)
            cv2.putText(debug, f"Wall err: {err}px",
                        (5, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

        lines = detections.get("lines", {})
        if lines.get("orange"):
            cv2.putText(debug, "ORANGE LINE", (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        if lines.get("blue"):
            cv2.putText(debug, "BLUE LINE", (5, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)
        if lines.get("magenta"):
            cv2.putText(debug, "PARKING MARKER", (5, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        return debug