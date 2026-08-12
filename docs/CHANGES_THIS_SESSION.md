# Changes — this session

Full line-by-line audit of every file in `src/`, cross-checked against the
codebase's own documented status. All 17 modules compile and were exercised
with synthetic-frame smoke tests (no real car yet, so this is the ceiling
of what's testable right now).

## Bugs fixed

1. **`main.py` — lap-based speed multiplier regression.**
   `speed_ctrl.compute()` was called without `lap=`, so speed silently
   defaulted to lap-1 pacing for the whole run (this bug had reappeared —
   `debug_visualizer.py` already had the fix, `main.py` didn't).
   Fixed: now passes `lap=laps + 1`. Verified with a smoke test showing
   200 → 240 → 280 mm/s across laps 1/2/3.

2. **`camera.py` / `main.py` — broken Raspberry Pi detection.**
   `IS_PI = platform.system() == "Linux"` is true on *any* Linux machine,
   not just a Pi — this would crash on any Linux dev/test machine trying to
   `import picamera2` / `RPi.GPIO`. Fixed: `camera.py` now checks
   `/proc/device-tree/model` for real Pi hardware, with a safe fallback.
   `main.py` now imports `IS_PI` from `camera.py` instead of duplicating
   the broken check. Verified: this sandbox now correctly reports
   `IS_PI=False` and runs in mock-motor mode.

3. **`parking_controller.py` — contact-avoidance guard was missing.**
   Despite earlier notes suggesting this existed, this codebase's parking
   state machine had pure open-loop timing with no live distance check —
   a real risk since any contact with the magenta parking-lot blocks
   zeroes the whole parking score. Added `_guarded_reverse_speed()`: eases
   reverse speed starting at 150mm from the nearest marker, hard-stops at
   60mm, using CVPipeline's live distance estimate. Verified with a smoke
   test forcing markers to 40mm (stops) and 100mm (eases).

4. **`parking_controller.py` — no full-vs-partial parking distinction, no
   parallelism check.** Added a `result` field ("full"/"partial") computed
   from horizontal centering plus a distance-symmetry proxy for
   "parallel to the wall" (true parallelism needs IMU/gyro heading, which
   this codebase doesn't have — this is a vision-only approximation,
   clearly labeled as such in the code comments).

5. **`main.py` — missing finish-section stop (Open Challenge) and
   stop-and-hold (both challenges).** This was the single biggest gap
   flagged previously. Added: an autonomous stop after lap 3 in Open
   Challenge mode, and a stop-and-hold phase after driving/parking
   completes in both modes. Honesty note: I could not verify an exact
   "hold for N seconds" number in the rules text (PDF wouldn't parse,
   no fixed duration turned up in search) — `STOP_HOLD_SECONDS` is
   labeled in-code as a conservative default, not a rule citation.
   **Worth double-checking against the official rules PDF directly.**

6. **`debug_visualizer.py` — no way to test parking on PC.** Since there's
   no car yet, this tool is one of the few ways to sanity-check logic
   before hardware exists. Added a `P` key to toggle a parking-test mode
   that wires in `ParkingController` the same way `main.py` does.

7. **Minor:** `requirements.txt` still said "WRO 2025" in its header
   comment, fixed to 2026. `focal_calibrator.py` now supports recording
   multiple distance samples and averaging them (`'s'` to add a sample,
   `'a'` to average + save) instead of a single point overwriting the
   config on each press — reduces sensitivity to measurement error, as
   previously flagged.

## Verified working end-to-end (mocked, paced synthetic frames)

- Full Obstacle Challenge run: 3 laps → parking search → parking completes
  with `result=full` → stop-and-hold → clean shutdown.
- Full Open Challenge run: 3 laps → finish-section stop → stop-and-hold →
  clean shutdown.
- Pillar detection, parking marker detection, Kalman filtering, pillar
  memory persistence, wall following, PID, and the lap speed multiplier
  all confirmed via synthetic-frame smoke tests.

## Still open / not yet done (unchanged from before this session)

- Kalman velocity (`vx, vy`) is tracked internally but never used for
  lookahead — `predict()` only returns position. Real accuracy upgrade,
  not a bug.
- Section counting has no drift-correction / anchor mechanism — a single
  missed or duplicate debounce over a 3-minute run has no self-correction.
- No wheel-encoder/IMU feedback anywhere — all speed control is open-loop
  duty-cycle, which is the single largest reliability limiter once real
  hardware exists (battery sag, surface friction, incline all go unseen).
- `PARKING_TIMEOUT` and `STOP_HOLD_SECONDS` are both safety-cap constants
  that will need real-world tuning once the car exists — no way to
  validate actual timing without hardware.
- Physical checklist items (vehicle footprint 300×200mm / height 300mm,
  focus ring glued after calibration, etc.) are still pending — code-only
  work is now caught up with everything that's testable without a car.
