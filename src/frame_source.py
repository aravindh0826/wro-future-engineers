"""
Frame Source Helper (shared by the calibration tools)

hsv_calibrator.py, focal_calibrator.py, lens_calibrator.py, and
corridor_calibrator.py all previously opened their video source with
cv2.VideoCapture(source) directly. That works for a USB webcam or a
recorded video file, but NOT for a CSI-ribbon Pi Camera Module -- picamera2
doesn't expose it as /dev/videoN the way cv2.VideoCapture expects unless
you're on the legacy camera stack. main.py never hit this because
camera.py already branches to picamera2 on real Pi hardware (IS_PI).

FrameSource gives the calibrators the same picamera2-backed path as
main.py, while still falling back to cv2.VideoCapture when the caller
passes an explicit --source (webcam index or a recorded video file) --
that override works identically on the Pi or a dev machine, so you can
still calibrate against saved footage if you want to.

Usage from a calibrator script:
    from frame_source import FrameSource
    cap = FrameSource(source)     # source=None -> Pi Camera on a real Pi
    ...
    ret, frame = cap.read()       # same shape as cv2.VideoCapture.read()
    ...
    cap.release()
"""

import cv2
from camera import Camera, IS_PI


class FrameSource:
    """Minimal cv2.VideoCapture-like interface (read/set/release/isOpened)
    that transparently uses the Pi Camera (via camera.Camera/picamera2) on
    real Pi hardware when no explicit source is given, and cv2.VideoCapture
    otherwise."""

    def __init__(self, source=None, width=640, height=480, fps=30):
        self._use_picam = (source is None) and IS_PI

        if self._use_picam:
            print("[FrameSource] Using Pi Camera (picamera2) via camera.Camera")
            self._cam = Camera(width=width, height=height, fps=fps)
            self._cam.start()
            self._cap = None
        else:
            actual_source = source if source is not None else 0
            print(f"[FrameSource] Using cv2.VideoCapture(source={actual_source!r})")
            self._cam = None
            self._cap = cv2.VideoCapture(actual_source)

    def isOpened(self):
        if self._use_picam:
            return True   # Camera.start() already raises if it fails to get a frame
        return self._cap.isOpened()

    def read(self):
        if self._use_picam:
            frame = self._cam.get_frame()
            if frame is None:
                return False, None
            return True, frame
        return self._cap.read()

    def set(self, *args, **kwargs):
        # No-op on the Pi Camera path -- e.g. calibrators call this to loop
        # a video file back to frame 0, which is meaningless for a live
        # camera feed.
        if not self._use_picam:
            self._cap.set(*args, **kwargs)

    def release(self):
        if self._use_picam:
            self._cam.stop()
        else:
            self._cap.release()
