"""
Camera Module
Handles Pi Camera (on Pi) and webcam/video file (on PC)
Runs capture in a separate thread for maximum speed
"""

import cv2
import threading
import platform
import time
import logging

logger = logging.getLogger(__name__)
IS_PI  = platform.system() == "Linux"


class Camera:
    def __init__(self, width=320, height=240, fps=60, source=0):
        """
        Args:
            width, height : capture resolution
            fps           : target frame rate
            source        : video file path (PC) or camera index
        """
        self.width   = width
        self.height  = height
        self.fps     = fps
        self.source  = source

        self._frame  = None
        self._lock   = threading.Lock()
        self._running = False
        self._thread = None

        logger.info(f"Camera init: {width}x{height} @ {fps}fps | PI={IS_PI}")

    # ── Start / Stop ──────────────────────────────────────────────────────────
    def start(self):
        self._running = True
        if IS_PI:
            self._start_pi()
        else:
            self._start_pc()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        # Wait for first frame
        timeout = 5.0
        start   = time.time()
        while self._frame is None and time.time() - start < timeout:
            time.sleep(0.05)
        if self._frame is None:
            raise RuntimeError("Camera failed to produce a frame within timeout.")
        logger.info("Camera started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if IS_PI:
            self._picam.stop()
        else:
            self._cap.release()
        logger.info("Camera stopped")

    # ── Frame access ──────────────────────────────────────────────────────────
    def get_frame(self):
        """Returns the latest frame (BGR numpy array) or None."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    # ── Internal init ─────────────────────────────────────────────────────────
    def _start_pi(self):
        from picamera2 import Picamera2
        self._picam = Picamera2()
        config = self._picam.create_video_configuration(
            main={"format": "BGR888", "size": (self.width, self.height)},
            controls={"FrameRate": self.fps}
        )
        self._picam.configure(config)
        self._picam.start()
        time.sleep(0.5)          # warm up

    def _start_pc(self):
        self._cap = cv2.VideoCapture(self.source)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS,          self.fps)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open video source: {self.source}")

    # ── Capture loops ─────────────────────────────────────────────────────────
    def _capture_loop(self):
        while self._running:
            if IS_PI:
                frame = self._picam.capture_array()
            else:
                ret, frame = self._cap.read()
                if not ret:
                    # Loop video file
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                frame = cv2.resize(frame, (self.width, self.height))

            with self._lock:
                self._frame = frame
