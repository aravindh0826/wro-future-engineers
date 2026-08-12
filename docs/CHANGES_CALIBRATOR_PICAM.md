# Changes: Calibrators now use the Pi Camera directly

## Problem
`hsv_calibrator.py`, `focal_calibrator.py`, `lens_calibrator.py`, and
`corridor_calibrator.py` all opened their video source with
`cv2.VideoCapture(source)` directly. That works for a USB webcam or a
recorded video file, but not for the CSI-ribbon Pi Camera Module --
picamera2 doesn't expose it as `/dev/videoN` the way `cv2.VideoCapture`
expects (unless the legacy camera stack is enabled). `main.py` never hit
this because `camera.py` already branches to picamera2 on real Pi
hardware (`IS_PI`); the calibrators just hadn't been updated to match.

## Fix
Added `src/frame_source.py`: a small `FrameSource` class with the same
`read()`/`set()`/`release()`/`isOpened()` shape as `cv2.VideoCapture`, so
each calibrator's capture loop didn't need restructuring. Internally:
- If no `--source` is given AND running on a real Pi (`camera.IS_PI`),
  it uses `camera.Camera` (the same picamera2 path as `main.py`).
- Otherwise (explicit `--source`, or not on a Pi), it uses
  `cv2.VideoCapture` exactly as before -- so calibrating against a
  webcam or a recorded video file still works unchanged, on the Pi or a
  dev machine.

All four calibrators now import `FrameSource` and default `--source` to
`None` instead of `0`, with updated `--help` text and usage docstrings.

## Files changed
- Added `src/frame_source.py`
- `src/hsv_calibrator.py`, `src/focal_calibrator.py`,
  `src/lens_calibrator.py`, `src/corridor_calibrator.py`: swapped
  `cv2.VideoCapture(source)` for `FrameSource(source)`, updated arg
  parsing and docstrings.

## Run commands (on the Pi, with the Pi Camera)
```bash
cd src
python3 hsv_calibrator.py --color red1
python3 focal_calibrator.py --distance 300 --color red
python3 lens_calibrator.py
python3 corridor_calibrator.py --type wide
```
No `--source` needed -- omitting it is what selects the Pi Camera path.
Pass `--source 0` or `--source path/to/video.mp4` to force the old
webcam/file behavior on any platform.
