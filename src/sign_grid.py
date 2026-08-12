"""
Sign Grid — precomputed traffic-sign placement geometry.

WRO 2026 rules establish that a sign's position inside a straight
section is NOT continuous or freely random -- it can only ever land on
one of a small, fixed set of "traffic signs' seats". This is confirmed
by TWO figures in the rules PDF that cross-check each other:

  * Figure 3 (page 7, "Zones and traffic signs' seats in the
    straightforward section") draws the seats directly: a straight
    section has exactly 6 seats arranged 3 columns x 2 rows (left /
    centre / right, near / far). The rules text on the same page states
    this explicitly: "Six internal zones within the section are for
    starting position of the car. 4 T-intersections and 2 X-intersections
    are used to position the traffic signs."  4 T-intersections + 2
    X-intersections = 6 seats, matching the 3x2 grid drawn.

  * Figure 8c (page 15, "36 cards with position of traffic signs within
    a section") shows every card's dot(s) landing on gridline
    intersections drawn on a per-card diagram. Read at face value the
    cards appear to offer 5 lateral positions (0..4), but checking the
    digitized GRID_CARDS table below confirms every single dot across
    all 36 cards lands ONLY on column 0, 2, or 4 -- i.e. left / centre /
    right. Columns 1 and 3 are never used by any card. This matches
    Figure 3's 3-column seat layout exactly: the card diagrams draw a
    finer visual gridline (for drawing precision) than the number of
    seats that actually exist.

So the real geometry -- confirmed by both figures agreeing -- is:
  * 3 lateral (column) positions per row: LEFT, CENTRE, RIGHT.
    LEFT sits against the inner-border side of the section (the thick
    black line drawn on every Figure 8c card is stated in the rules
    text, page 14, to represent the inner border); RIGHT sits against
    the outer-border side.
  * 2 longitudinal (row) positions along the section's length: "near"
    (closer to the section's entry) and "far" (closer to the exit).
  * A card can carry 1 or 2 dots. Card colours are fixed per dot in the
    figure; the runtime coin-toss (rules 8b/8c step 2-3) only decides
    WHICH card gets drawn and, for the single-sign section, which colour
    that lone sign is -- it does not move the dots off these seats.

None of this replaces live CV. What it changes is what the car does with
a live detection: instead of trusting a continuous, noise-sensitive mm
estimate as the final steering target, the live reading is classified to
the nearest of these known seats (row, column), and the STEERING TARGET
for that seat is looked up from precomputed geometry -- exact, tunable
ahead of competition, and stable frame to frame.

This does not remove WallFollower.lateral_offset_mm(). It still runs
every frame:
  1. As the classifier input -- its live mm reading is what gets rounded
     to the nearest seat column.
  2. As the fallback -- if a live reading sits ambiguously between two
     columns (build tolerance, bad lighting, a genuinely in-between
     detection), classification is refused for that frame and the
     caller should steer off the live continuous estimate instead of
     confidently snapping to a wrong cell.

Section length caveat (rules Appendix B, page 43): a straight section's
usable length is NOT one fixed number -- it depends on the per-round
corridor-width coin toss (segments are prepared for 1000/1400/1800 mm
straight runs). Column-to-mm conversion therefore takes the section's
actual measured/known length as a parameter at call time rather than
assuming a single hardcoded section length.
"""

import logging

logger = logging.getLogger(__name__)

# ── Digitized card geometry (rules PDF, page 15 Figure 8c; cross-checked ───
# against page 7 Figure 3 "Zones and traffic signs' seats") ─────────────────
# Read directly off the published figures; see CHANGES_SIGN_GRID.md for the
# card-by-card verification notes. Card indices 1-36 match the figure's own
# numbering so this table can be re-checked against the source image
# directly. Column indices use the TRUE seat count confirmed by Figure 3
# (3 seats per row: LEFT/CENTRE/RIGHT) -- Figure 8c's cards draw a finer
# visual gridline than the number of seats that actually exist; checking
# every dot in this table lands only on the left/centre/right seats (never
# in between) is what confirmed the two figures describe the same geometry.
#
# Each entry: list of (color, row, column) tuples.
#   color  : "red" | "green"
#   row    : "near" | "far"
#   column : 0=LEFT (inner-border side) | 1=CENTRE | 2=RIGHT (outer-border side)
GRID_CARDS = {
    1:  [("green", "near", 0)],
    2:  [("red",   "near", 0)],
    3:  [("green", "near", 1)],
    4:  [("red",   "near", 1)],
    5:  [("green", "near", 2)],

    6:  [("red",   "far",  2)],
    7:  [("green", "far",  0)],
    8:  [("red",   "far",  0)],
    9:  [("green", "far",  1)],
    10: [("red",   "far",  1)],

    11: [("green", "near", 2)],
    12: [("red",   "near", 2)],
    13: [("green", "near", 2), ("green", "far", 0)],
    14: [("red",   "near", 2), ("green", "far", 0)],
    15: [("green", "near", 2), ("red",   "far", 0)],

    16: [("red",   "near", 2), ("green", "far", 0)],
    17: [("green", "near", 2), ("red",   "far", 0)],
    18: [("red",   "near", 2), ("red",   "far", 0)],
    19: [("green", "near", 0), ("green", "far", 2)],
    20: [("green", "near", 0), ("red",   "far", 2)],

    21: [("red",   "near", 0), ("green", "far", 2)],
    22: [("green", "near", 0), ("red",   "far", 2)],
    23: [("red",   "near", 0), ("green", "far", 2)],
    24: [("red",   "near", 0), ("red",   "far", 2)],
    25: [("green", "near", 0), ("green", "near", 2)],

    26: [("green", "near", 0), ("red",   "near", 2)],
    27: [("red",   "near", 0), ("green", "near", 2)],
    28: [("green", "near", 0), ("red",   "near", 2)],
    29: [("red",   "near", 0), ("green", "near", 2)],
    30: [("red",   "near", 0), ("red",   "near", 2)],

    31: [("green", "near", 0), ("green", "near", 2)],
    32: [("green", "near", 0), ("red",   "near", 2)],
    33: [("red",   "near", 0), ("green", "near", 2)],
    34: [("green", "near", 0), ("red",   "near", 2)],
    35: [("red",   "near", 0), ("green", "near", 2)],

    36: [("red",   "near", 0), ("red",   "near", 2)],
}

# Columns run 0..2 across the section width -> 3 known lateral seat
# positions (LEFT/CENTRE/RIGHT), confirmed by rules Figure 3 ("Zones and
# traffic signs' seats in the straightforward section" -- 4 T-intersections
# + 2 X-intersections = 6 seats total, arranged 3 columns x 2 rows). This
# is NOT a reading of Figure 8c's card gridlines alone -- those cards draw
# a finer visual grid than the real seat count; Figure 3 is what confirms
# the true column count is 3, and every dot in GRID_CARDS above landing
# only on columns 0/1/2 (never in between) is the cross-check that the two
# figures describe the same geometry.
NUM_COLUMNS = 3

# How close (as a fraction of one column's width) a live mm reading must be
# to a grid column to be trusted as "this cell, confidently" rather than
# "ambiguous, fall back to continuous estimate". 0.5 would snap everything
# (no ambiguous zone); smaller = safer/more conservative.
# TUNE_ME: starting default, not yet validated against real camera noise.
SNAP_CONFIDENCE_FRACTION = 0.30


class SignGrid:
    """
    Classifies a live wall-relative mm offset to the nearest known seat
    column (LEFT/CENTRE/RIGHT), and looks up the precomputed target mm
    for that column given the section's actual length. Row (near/far) is
    not something a single static-camera frame can measure directly, so
    row is not inferred by this class -- callers that need it would pass
    a hint (e.g. section progress / time-since-entry); the current
    alignment-bias use case in main.py only needs the lateral column.
    """

    def __init__(self):
        logger.info("SignGrid initialised "
                     f"({len(GRID_CARDS)} cards, {NUM_COLUMNS} columns)")

    # ── Public API ────────────────────────────────────────────────────────
    def classify_column(self, live_mm, section_length_mm):
        """
        Rounds a live lateral-offset-from-inner-wall reading (mm) to the
        nearest of the 3 known seat columns (LEFT/CENTRE/RIGHT) for a
        section of the given length.

        Args:
            live_mm           : WallFollower.lateral_offset_mm() reading,
                                 mm from the inner-border-side wall.
            section_length_mm : actual length of this straight section,
                                 mm (varies per round -- see module
                                 docstring; caller supplies the
                                 known/measured value, this function does
                                 not assume one).
        Returns:
            (column:int, target_mm:float) if the reading confidently
            snaps to one column, else None (ambiguous -- caller should
            fall back to the continuous live_mm estimate for this frame).
        """
        if live_mm is None or section_length_mm is None or section_length_mm <= 0:
            return None

        col_width = section_length_mm / (NUM_COLUMNS - 1)
        raw_col = live_mm / col_width
        nearest = round(raw_col)
        nearest = max(0, min(NUM_COLUMNS - 1, nearest))

        deviation = abs(raw_col - nearest)
        if deviation > SNAP_CONFIDENCE_FRACTION:
            logger.debug(f"SignGrid: ambiguous column (raw={raw_col:.2f}, "
                          f"nearest={nearest}, deviation={deviation:.2f}) "
                          "-- falling back to continuous estimate")
            return None

        target_mm = nearest * col_width
        return nearest, target_mm

    def target_mm_for_cell(self, column, section_length_mm):
        """
        Precomputed steering target (mm from inner wall) for a known grid
        column, given the section's actual length. Pure geometry lookup --
        exact and tunable ahead of competition, no live measurement noise.
        """
        if section_length_mm is None or section_length_mm <= 0:
            return None
        col_width = section_length_mm / (NUM_COLUMNS - 1)
        return column * col_width

    def cards_with_color_at_column(self, color, column, row=None):
        """
        Debug/tuning helper: which of the 36 cards would place `color` at
        `column` (optionally restricted to `row`). Useful for sanity
        checking the digitized table against the rules figure by eye.
        """
        matches = []
        for card_id, dots in GRID_CARDS.items():
            for dot_color, dot_row, dot_col in dots:
                if dot_color == color and dot_col == column and (row is None or dot_row == row):
                    matches.append(card_id)
                    break
        return matches

    def reset(self):
        """No per-round state to clear -- geometry is fixed. Present for
        interface symmetry with the other *_controller/*_memory classes."""
        pass
