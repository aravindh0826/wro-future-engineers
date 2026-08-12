# Changes — sign grid classification

Builds the grid-classification upgrade path we discussed previously:
"grid classification as primary, continuous estimation as fallback."
Wasn't built last session -- this session builds it, grounded directly in
the rules PDF figures rather than assumption.

## Rules PDF note

The uploaded "PDF" is actually a zip of page images (`unzip -l` confirms
it, `pdfplumber` correctly refuses it as not a real PDF). Extracted and
read the page images directly to digitize the actual geometry -- this
was not done last session ("PDF wouldn't parse" was the prior blocker
for the STOP_HOLD_SECONDS number, same underlying file format issue).

## Geometry correction (this session, after a user-supplied re-check image)

First pass digitized Figure 8c (page 15) alone and read it as 5 lateral
positions per row (columns 0..4), because each card's drawing shows 4
grid cells (5 gridlines) for visual reference. That was wrong about the
*seat count*, though it happened to still produce correct mm values by
coincidence (dots only ever landed on columns 0, 2, 4 -- i.e. 0%, 50%,
100% of the drawn width -- so `target_mm_for_cell()` output the right
numbers even though `classify_column()` could in principle have
"confidently" snapped a live reading onto column 1 or 3, positions that
can never actually occur).

Cross-checking against Figure 3 (page 7, "Zones and traffic signs'
seats in the straightforward section") caught this. Figure 3 draws the
seats directly -- 6 seats total, 3 columns x 2 rows -- and the
accompanying text confirms it: "Six internal zones... 4 T-intersections
and 2 X-intersections are used to position the traffic signs" (4+2=6).
Re-checking GRID_CARDS confirmed every one of the 36 cards' dots lands
only on 0/2/4 of the old 5-column scale -- i.e. only ever on
left/centre/right, never in between -- which is exactly what Figure 3's
3-seat-per-row layout predicts. The two figures agree once read
correctly; `sign_grid.py` now models 3 columns (`NUM_COLUMNS = 3`:
LEFT=0, CENTRE=1, RIGHT=2) instead of 5, which removes the two
positions (old columns 1, 3) that never actually occur and closes the
silent-misclassification risk.

Card-by-card re-verification: every one of the 36 cards was re-viewed
side-by-side against a second, cleaner copy of Figure 8c the user
supplied, cropped and zoomed row by row. All 36 matched the existing
digitization exactly (no dot-position errors found) -- the only
correction needed was the column-count/indexing model, not the
individual card contents.

## New: `src/sign_grid.py`

- `GRID_CARDS` -- all 36 cards from Figure 8c, digitized card-by-card,
  re-verified against a second copy of the source image, and
  cross-checked against Figure 3's independently-drawn seat layout.
  Each card: 1-2 `(color, row, column)` dots. `row` is
  `"near"`/`"far"` (two longitudinal seat rows along the section),
  `column` is `0..2` (three lateral seats across the section width;
  0=LEFT/inner-border side, 2=RIGHT/outer-border side, 1=CENTRE), per
  the rules text on page 14 stating the card's thick black line
  represents the inner border, and page 7's explicit 6-seat /
  4-T-intersection + 2-X-intersection description.
- `SignGrid.classify_column(live_mm, section_length_mm)` -- rounds a
  live `WallFollower.lateral_offset_mm()` reading to the nearest of the
  3 known seat columns. Returns `None` (ambiguous) if the reading isn't
  confidently close to a seat, so the caller falls back to the
  continuous estimate instead of snapping to a wrong cell.
  `SNAP_CONFIDENCE_FRACTION` (0.30) controls how strict this is --
  flagged `TUNE_ME`, not yet validated against real camera noise.
- `SignGrid.target_mm_for_cell(column, section_length_mm)` -- pure
  geometry lookup, no live measurement involved.
- `cards_with_color_at_column()` -- debug helper for re-verifying the
  digitized table against the source figure by eye later.

## Section length is NOT a fixed constant

Rules Appendix B (page 43) shows the interior wall is built per-round
from one of three prepared segment sets, giving straight-section length
of 1000, 1400, or 1800 mm depending on the corridor-width coin toss
(rules 8, Figure 7b). `classify_column()`/`target_mm_for_cell()` take
section length as a parameter rather than hardcoding one value.

`main.py` gets a new `SECTION_LENGTH_MM` constant, set alongside
`CHALLENGE_MODE`/`DIRECTION` in the per-round configuration block --
**this must be set correctly before each round** (same requirement as
direction), default value is an unverified placeholder, clearly
commented as such.

## Changed: `src/pillar_memory.py`

- New `_grid_column` storage alongside the existing `_wall_offset_mm`.
- `update()` gains an optional `grid_column={"red":.., "green":..}` arg.
- New `recall_grid_column(section)`, same lap-gating as
  `recall_wall_offset()`.
- Existing `_wall_offset_mm` storage/recall is unchanged -- still the
  fallback data source, not removed.

## Changed: `src/main.py`

- Imports and instantiates `SignGrid`.
- Lap-1 update path: classifies each pillar's live wall-offset to a grid
  column every processed frame, passes it into `pillar_memory.update()`
  alongside the existing continuous wall-offset.
- Lap-2/3 alignment-bias target now resolves in priority order:
  1. `recall_grid_column()` -> `sign_grid.target_mm_for_cell()` if lap 1
     got a confident classification for this section/color.
  2. `recall_wall_offset()` (previous behaviour) if grid classification
     was ambiguous during lap 1.
- Everything downstream of `target_mm` (the `±0.2`-capped alignment bias
  math, PID, live-detection-always-wins priority) is unchanged.

## Verified (synthetic, no camera/car)

- `sign_grid.py` in isolation, re-run after the geometry correction:
  exact-seat classification at all 3 columns (left/centre/right) for
  all 3 possible section lengths (1000/1400/1800mm); small jitter within
  tolerance still classifies; exact-midpoint ambiguous case correctly
  returns `None`; `GRID_CARDS` table shape validated (36 cards, all
  columns confirmed in `{0,1,2}`); `target_mm_for_cell()` sanity-checked
  against exact 0%/50%/100%-of-section-length values.
- `pillar_memory.py` grid storage/recall in isolation: confidence
  accumulation unaffected by the new field; lap-1 recall correctly
  suppressed; lap-2 recall returns exactly what was stored; heading-gate
  on `recall()` still works independently; ambiguous-classification case
  (grid column `None`, continuous wall-offset still present) behaves
  correctly.
- Full synthetic wiring test mirroring `main.py`'s actual logic
  end-to-end, re-run after the geometry correction: lap-1 confident
  centre-seat classification -> lap-2 target correctly resolves to the
  exact precomputed 50%-of-section-length mm value (not the noisy live
  reading); alignment_bias output matches hand-calculated expected value.
- All 18 modules in `src/` still compile clean together.

## Still open

- `SNAP_CONFIDENCE_FRACTION` (0.30) -- reasonable starting guess, not
  validated against real camera/lighting noise.
- `SECTION_LENGTH_MM` -- placeholder default (1400mm), must be set per
  round once the actual coin-toss/segment-set result is known. Getting
  this wrong scales every grid-classified target by a fixed factor, so
  it's as consequential as `DIRECTION` and should be checked with the
  same care.
- Row (`near`/`far`) is stored in `GRID_CARDS` but not yet used at
  runtime -- `classify_column()` only classifies laterally (column). A
  single forward-facing camera frame doesn't directly give
  "how far along the section am I," so consuming the row dimension
  would need a proxy (time-since-section-entry, or IMU-based progress)
  that doesn't exist yet. Not required for the alignment-bias use case
  (which only needs lateral position), but would be needed for anything
  wanting to also predict a pillar's longitudinal position ahead of
  detection.
- The card-content digitization (which color/row/column each of the 36
  cards encodes) has now been read twice from two different copies of
  the same figure and cross-checked against a second, independent
  figure (Figure 3's seat layout) -- reasonably high confidence at this
  point, but a third human check against the physical rules document
  before competition is still worthwhile given how consequential a
  transcription error here would be.

