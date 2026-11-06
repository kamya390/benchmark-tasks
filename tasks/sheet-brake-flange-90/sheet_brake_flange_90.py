"""On the station table, in front of the robot, stands a small bench hand brake (a box-and-pan style sheet-metal
folder) running across the table. Its steel bed is a fixed block whose front top edge is the bend line. Hinged
on that edge, the bending leaf (a flat steel bar with two upright handles at its free edge) lies flush with the bed
and reaches toward the robot. Above the bed, the clamping bar (an aluminium bar with a knob at its right end) rides
on two guide posts; it starts raised 13 mm, off the sheet. An aluminium blank lies on the bed against a back
gauge with its scribed bend line on the bed's edge: the base of the blank on the bed and its 5 cm flange on the
bending leaf. A grey tray lies to the right of the brake. The job is to bend the flange up to 90 degrees and put
the finished bracket on the tray.

Mechanisms (mechkit.py, in this folder): the leaf turns on a hinge with Coulomb friction (it stays where it is left);
the clamping bar slides vertically with friction (it stays at any height, and resists the blank's uplift); the blank
is two parts joined at the bend line by a friction hinge (the bend stays where it is formed). Physics enforces the
order: raised against an unclamped blank, the leaf lifts the whole blank (the bend's resistance exceeds the base's
weight), so the base rides up off the bed; with the bar down on the base, the flange bends round the bar's nose,
which stops it at about 92.5 degrees.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/sheet-brake-flange-90/sheet_brake_flange_90.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinges, sliders; installs its yam_arena hooks in memory, incl. mk.MEMORY reset)

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
BEND_X = 0.300  # the bend line: the bed's front top edge
BED = (0.100, 0.260, 0.040)  # x 0.30-0.40, y +-0.13
BED_TOP = TOP + BED[2]
SHEET_T = 0.003  # blank thickness (drawn 3 mm thick; mass as 1.5 mm aluminium)
SHEET_TOP = BED_TOP + SHEET_T
AXIS = (BEND_X, 0.0, SHEET_TOP)  # the leaf's and the bend's hinge line runs along y through here
BASE = (0.080, 0.150)  # blank base: depth (x, from the bend line back to the gauge), width (y)
FLANGE = 0.050  # flange height once bent (its depth on the leaf before)
BASE_KG, FLANGE_KG = 0.049, 0.030
BEND_FRICTION = 0.08  # N*m: the formed bend holds (the base's weight about the bend line is 0.019 N*m)
LEAF = (0.0595, 0.240, 0.010)  # bending leaf plate: depth (x), width, thickness; its top flush with the bed
LEAF_GAP = 0.001  # between the leaf's back edge and the bed's front face
LEAF_KG = 0.25
LEAF_FRICTION = 0.08  # N*m: more than its weight torque (0.074), so it stays where it is left
POST = (0.015, 0.015, 0.080)  # leaf handles: depth, width, height above the leaf top
POST_Y = 0.100
POST_U = -(LEAF_GAP + LEAF[0]) + POST[0] / 2  # handle centre, leaf frame (along -x at 0 deg)
BAR = (0.040, 0.240, 0.012)  # clamping bar: depth (x), width, thickness
BAR_X = BEND_X + 0.0005 + BAR[0] / 2  # nose 0.5 mm behind the bend line: the flange stops against it at ~92.5 deg
BAR_KG = 0.30
BAR_FRICTION = 7.0  # N: more than its weight (2.9 N); it stays at any height; lifting it takes ~10 N
BAR_OPEN = 0.013  # start (and upper stop): the bar's underside 13 mm above the sheet
KNOB = (0.020, 0.020, 0.020)
KNOB_Y = -0.100
GAUGE = (0.005, 0.160, 0.003)
GAUGE_X = BEND_X + BASE[0] + 0.001 + GAUGE[0] / 2
TRAY_XY = (0.310, -0.250)
TRAY = (0.140, 0.200, 0.004)
TRAY_TOP = TOP + TRAY[2]
RIM = (0.004, 0.008)  # tray rim thickness, height above the tray top

STEEL = (0.42, 0.44, 0.47)
DARK = (0.16, 0.17, 0.19)
ALU = (0.76, 0.78, 0.82)
SCRIBE = (0.25, 0.25, 0.28)
RED = (0.72, 0.12, 0.10)
GREY_TRAY = (0.50, 0.51, 0.53)

kit = mk.Kit()
# The leaf: frame on the hinge line; in the reference pose (0 deg) it lies flush with the bed, reaching toward the
# robot (-x). +y axis: a positive angle lifts its free edge (90 deg: standing upright at the bend line).
kit.part("leaf", AXIS, [
    mk.box(LEAF, at=(-(LEAF_GAP + LEAF[0] / 2), 0.0, -SHEET_T - LEAF[2] / 2)),
    *(mk.box(POST, at=(POST_U, sy * POST_Y, -SHEET_T + POST[2] / 2), color=RED) for sy in (-1, 1)),
], mass=LEAF_KG, color=STEEL)
kit.hinge("leaf_hinge", None, "leaf", anchor=AXIS, axis=(0, 1, 0), lower_deg=0.0, upper_deg=135.0, start_deg=0.0,
          friction=LEAF_FRICTION)
# The clamping bar: frame at the middle of its underside, reference pose (0) down on the sheet.
kit.part("bar", (BAR_X, 0.0, SHEET_TOP), [
    mk.box(BAR, at=(0.0, 0.0, BAR[2] / 2)),
    mk.box(KNOB, at=(0.0, KNOB_Y, BAR[2] + KNOB[2] / 2), color=DARK),
], mass=BAR_KG, color=ALU)
kit.slider("clamp", None, "bar", axis=(0, 0, 1), lower=-0.001, upper=BAR_OPEN, start=BAR_OPEN, friction=BAR_FRICTION)
# The blank: the base lies on the bed (bend line on the bed's edge, heel against the gauge); the flange lies on the
# leaf, hinged to the base at the scribed line, so the bend stays where it is formed.
kit.part("blank_base", (BEND_X, 0.0, BED_TOP), [
    mk.box((BASE[0], BASE[1], SHEET_T), at=(BASE[0] / 2, 0.0, SHEET_T / 2)),
], mass=BASE_KG, color=ALU)
kit.part("flange", AXIS, [
    mk.box((FLANGE, BASE[1], SHEET_T), at=(-FLANGE / 2, 0.0, -SHEET_T / 2)),
    mk.box((0.001, BASE[1], 0.0002), at=(-0.0005, 0.0, 0.0001), color=SCRIBE),  # the scribed bend line
], mass=FLANGE_KG, color=ALU)
kit.hinge("bend", "blank_base", "flange", anchor=AXIS, axis=(0, 1, 0), lower_deg=-5.0, upper_deg=135.0,
          start_deg=0.0, friction=BEND_FRICTION)

_POST_H = SHEET_TOP + 0.030 - BED_TOP  # guide posts: up to 5 mm above the raised bar's top
FIXTURES = [
    Box("bed", pos=(BEND_X + BED[0] / 2, 0.0, TOP + BED[2] / 2), size=BED, color=STEEL),
    *(Box(f"guide_post_{n}", pos=(BAR_X, sy * (BAR[1] / 2 + 0.002 + 0.006), BED_TOP + _POST_H / 2),
          size=(0.030, 0.012, _POST_H), color=DARK) for n, sy in (("left", 1), ("right", -1))),
    Box("back_gauge", pos=(GAUGE_X, 0.0, BED_TOP + GAUGE[2] / 2), size=GAUGE, color=DARK),
    Box("tray", pos=(*TRAY_XY, TOP + TRAY[2] / 2), size=TRAY, color=GREY_TRAY),
    *(Box(f"tray_rim_{n}", pos=(TRAY_XY[0] + sx * (TRAY[0] / 2 - RIM[0] / 2), TRAY_XY[1], TRAY_TOP + RIM[1] / 2),
          size=(RIM[0], TRAY[1], RIM[1]), color=GREY_TRAY) for n, sx in (("near", -1), ("far", 1))),
    *(Box(f"tray_rim_{n}", pos=(TRAY_XY[0], TRAY_XY[1] + sy * (TRAY[1] / 2 - RIM[0] / 2), TRAY_TOP + RIM[1] / 2),
          size=(TRAY[0] - 2 * RIM[0], RIM[0], RIM[1]), color=GREY_TRAY) for n, sy in (("left", 1), ("right", -1))),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Everything is read from the parts' poses: the leaf's and the flange's angles, the bar's height, the base's lift.

LEAF_UP_DEG = 10.0  # the leaf counts as raised above 10 deg (and as lowered again below it)
BAR_UP = 0.003  # the bar counts as off the sheet once its underside is 3 mm above the sheet's top
HEEL_LIFT = 0.004  # the blank rides up: its base 4 mm off the bed ...
HEEL_TILT = 4.0  # ... or tilted 4 deg, while the leaf is raised
OVERBEND = 100.0  # bent past 100 deg (reverse-bending a flange back cracks real sheet): capped at 0.25
FORMED = 80.0  # the bend is credited only if the leaf formed it: leaf and flange both >= 80 deg at once
PARTIAL = 45.0
ANGLE = (87.0, 93.0)
TRAY_TOL = 0.0
_TEST_TCPS = None  # self-test stand-in for the grippers' TCPs

K_UNCLAMPED = "sbf_leaf_raised_unclamped"
K_HEEL = "sbf_heel_lift"
K_OVER = "sbf_overbend"
K_FORMED = "sbf_formed_by_leaf"
K_HALF = "sbf_half_formed_by_leaf"


def _tilt(q) -> float:
    x, y, _, _ = (float(v) for v in q)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _released(st, margin=0.025) -> bool:
    """Neither gripper's TCP within 2.5 cm of the part's bounding box (mk.released, with a self-test stand-in)."""
    if _TEST_TCPS is None:
        return mk.released(st, margin)
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _TEST_TCPS)


def leaf_deg(objects) -> float:
    return kit.q_deg(objects, "leaf_hinge")


def flange_deg(objects) -> float:
    """The bend angle: the flange's turn relative to the base (0 flat, 90 square)."""
    return kit.q_deg(objects, "bend")


def bar_lift(objects) -> float:
    """The clamping bar's underside above the sheet's top, m (0 when it clamps the sheet)."""
    return kit.q(objects, "clamp")


def _heel_up(base) -> bool:
    return bool(float(base.low[2]) > BED_TOP + HEEL_LIFT or _tilt(base.quat_xyzw) > HEEL_TILT)


def _update(objects) -> None:
    """Latch this trial's events (mk.MEMORY is cleared only at the embodiment's reset)."""
    leaf, bend = leaf_deg(objects), flange_deg(objects)
    if leaf > LEAF_UP_DEG and bar_lift(objects) > BAR_UP:
        mk.MEMORY[K_UNCLAMPED] = True
    if leaf > LEAF_UP_DEG and _heel_up(objects["blank_base"]):
        mk.MEMORY[K_HEEL] = True
    if bend > OVERBEND:
        mk.MEMORY[K_OVER] = True
    if leaf >= FORMED and bend >= FORMED:
        mk.MEMORY[K_FORMED] = True
    if leaf >= PARTIAL and bend >= PARTIAL:
        mk.MEMORY[K_HALF] = True


def _on_tray(objects) -> bool:
    """The bracket on the tray: the base flat on the tray floor inside the rim, both parts still and let go."""
    base, fl = objects["blank_base"], objects["flange"]
    x0, y0 = TRAY_XY[0] - TRAY[0] / 2 + RIM[0], TRAY_XY[1] - TRAY[1] / 2 + RIM[0]
    x1, y1 = TRAY_XY[0] + TRAY[0] / 2 - RIM[0], TRAY_XY[1] + TRAY[1] / 2 - RIM[0]
    inside = all(x0 <= float(st.low[0]) and float(st.high[0]) <= x1 and y0 <= float(st.low[1])
                 and float(st.high[1]) <= y1 for st in (base, fl))
    return bool(inside and TRAY_TOP - 0.002 < float(base.low[2]) < TRAY_TOP + 0.005
                and _tilt(base.quat_xyzw) < 5.0 and base.resting() and fl.resting()
                and _released(base) and _released(fl))


def score(objects) -> float:
    """0 for the rest of the trial if the leaf was ever raised above 10 deg while the clamping bar was off the sheet
    (3 mm), or the blank's base rode up (4 mm or 4 deg) while the leaf was raised. Otherwise, with the bend formed
    by the leaf (leaf and flange both >= 80 deg at one moment): 1.0 for the flange at 87-93 deg, the leaf back below
    10 deg and the bracket flat on the tray floor, still and let go; 0.5 for the same without the tray. 0.25 for a
    flange bent >= 45 deg by the leaf. Capped at 0.25 if the flange was ever bent past 100 deg."""
    _update(objects)
    m = mk.MEMORY
    if m.get(K_UNCLAMPED) or m.get(K_HEEL):
        return 0.0
    bend, leaf_down = flange_deg(objects), leaf_deg(objects) < LEAF_UP_DEG
    s = 0.0
    if m.get(K_FORMED) and ANGLE[0] <= bend <= ANGLE[1] and leaf_down:
        s = 1.0 if _on_tray(objects) else 0.5
    elif m.get(K_HALF) and bend >= PARTIAL:
        s = 0.25
    return min(s, 0.25) if m.get(K_OVER) else s


def success(objects) -> bool:
    """The bracket bent square by the clamped brake, on the tray, let go; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: the flange's tip arc to 90 deg plus the base's centre to the tray's centre."""
    arc = abs(math.radians(90.0 - flange_deg(objects))) * FLANGE
    b = objects["blank_base"].pos
    return float(arc + math.hypot(float(b[0]) - (TRAY_XY[0] + 0.0), float(b[1]) - TRAY_XY[1]))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the brake's fixed geometry; it reads no joint or simulator state. The closed finger pair (20 mm
# along x at yaw 90 deg, 38 mm along y) pushes the leaf's handle with one lower edge: along the handle's arc going
# up, in a level sweep coming down.

TIP = 0.013  # pad tips below the TCP
HALF = 0.010  # closed pads' face from the TCP along x (yaw 90 deg)
C = FINGER_CLOSED
YX = math.pi / 2  # fingers close along x (right arm; the left arm uses -90 deg, same closing axis)
YL = -math.pi / 2
A = np.array([AXIS[0], AXIS[2]])  # the hinge line in the x-z plane
RAISE_W = 0.065  # the raising push acts 6.5 cm up the handle's front face
POST_FRONT = -(LEAF_GAP + LEAF[0])  # the handle's front face (leaf frame u)
RAISE_ARC = (5, 15, 30, 45, 60, 72, 82, 88, 92, 96)  # commanded past 90: the arm lags ~7 deg, the bar's nose stops
# the flange at ~92.5 deg
LOWER_Z = 0.042  # the lowering push runs level, this high above the hinge line: it meets the handle's end or its
# back face at any leaf angle from 0 to ~95 deg and sweeps the leaf down flush
PUSH = -0.004  # the pushing edge runs 4 mm into the face line, so it keeps contact
BEND_TO = 96.0
SQUEEZE = 0.008
X_TRIM = 0.0015
CARRY_X = 0.150


def _rot(theta, u, w):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([u * c + w * s, -u * s + w * c])


def _raise_tcp(deg, gap, y):
    """TCP putting the pads' lower +x edge ``gap`` in front of the handle's front face, leaf at ``deg``."""
    t = math.radians(deg)
    edge = A + _rot(t, POST_FRONT, RAISE_W) + gap * _rot(t, -1.0, 0.0)
    return (float(edge[0] - HALF), y, float(edge[1] + TIP))


def _plan_s(wps) -> float:
    return sum(w.duration for w in wps)


TRAVEL = TOP + 0.130
PARK = {"left": (0.200, 0.260, TOP + 0.150), "right": (0.200, -0.260, TOP + 0.150)}


def demo():
    """Left arm: press the clamping bar down onto the blank; push the left handle up along its arc until the flange
    stands against the bar's nose (~90 deg); then push the handle back with a level sweep, which swings the leaf
    down flush. Right arm, after that: lift the bar by its knob; grip the standing flange, slide the bracket out
    from under the bar, carry it round in front of the leaf's handles to the tray and set it down. 59.8 s."""
    y = POST_Y
    bar_top = SHEET_TOP + BAR[2]
    press = (BAR_X, 0.060, bar_top + TIP - 0.014)  # pad tips 1 mm above the bed: the arm gives ~5 mm
    left = [
        Waypoint(2.0, (press[0], press[1], TRAVEL), C, "over the clamping bar", 0.0),
        Waypoint(1.2, (press[0], press[1], bar_top + BAR_OPEN + TIP + 0.010), C, "down to the bar", 0.0),
        Waypoint(1.5, press, C, "press the bar down onto the blank", 0.0),
        Waypoint(0.6, press, C, "hold", 0.0),
        Waypoint(1.0, (press[0], press[1], TRAVEL), C, "lift clear", 0.0),
    ]
    start = _raise_tcp(0.0, 0.015, y)
    left += [
        Waypoint(1.8, (start[0], y, TRAVEL), C, "over the leaf handle", YL),
        Waypoint(1.2, start, C, "down in front of the handle", YL),
        Waypoint(0.4, start, C, "settle", YL),
        *(Waypoint(0.7, _raise_tcp(d, PUSH, y), C, f"raise the leaf to {d} deg", YL) for d in RAISE_ARC),
        Waypoint(0.5, _raise_tcp(BEND_TO, PUSH, y), C, "hold (flange against the bar's nose)", YL),
    ]
    up = _raise_tcp(BEND_TO, 0.008, y)

    def lw(u, z):  # TCP putting the pads' lower -x edge at (u, z) from the hinge line
        return (float(A[0] + u + HALF), y, float(A[1] + z + TIP))

    left += [
        Waypoint(0.8, up, C, "lift off the handle", YL),
        Waypoint(1.0, lw(0.090, 0.066), C, "over beyond the handle's end", YL),
        Waypoint(1.0, lw(0.090, LOWER_Z), C, "down beside the handle's end", YL),
        Waypoint(0.4, lw(0.090, LOWER_Z), C, "settle", YL),
        Waypoint(7.0, lw(-0.050, LOWER_Z), C, "push the handle back: the leaf swings down flush", YL),
        Waypoint(0.5, lw(-0.050, LOWER_Z), C, "hold", YL),
        Waypoint(1.0, lw(-0.050, 0.110), C, "lift clear", YL),
        Waypoint(1.5, PARK["left"], C, "park", YL),
    ]
    # right arm waits until the leaf is down
    wait = _plan_s(left)
    o_knob = -(KNOB[0] / 2 + 0.012)
    g_knob = -(KNOB[0] - SQUEEZE) / 2
    kz = bar_top + KNOB[2] - 0.012 + TIP  # the bar is down on the sheet by now
    o_fl = -0.012
    fz = SHEET_TOP + FLANGE - 0.016 + TIP  # grip the flange's top 16 mm (pad tips 8 mm above the raised bar)
    fx = BEND_X - SHEET_T / 2
    out = 0.090  # slide the bracket this far toward the robot, clear of the bar
    # carried with the flange at x 0.15, the bracket (base reaching 8 cm in +x) passes in front of the leaf's
    # upright handles (x 0.24-0.255, 8 cm tall) instead of over them
    qx, qy = TRAY_XY[0] - 0.040 + X_TRIM, TRAY_XY[1]
    qz = TRAY_TOP + SHEET_T + FLANGE - 0.016 + TIP + 0.002  # let go 2 mm above resting
    right = [
        Waypoint(wait, PARK["right"], C, "wait for the leaf", YX),
        Waypoint(1.5, (BAR_X, KNOB_Y, TRAVEL), o_knob, "over the knob", YX),
        Waypoint(1.2, (BAR_X, KNOB_Y, kz + 0.015), o_knob, "descend", YX),
        Waypoint(0.8, (BAR_X, KNOB_Y, kz), o_knob, "down", YX),
        Waypoint(0.4, (BAR_X, KNOB_Y, kz), o_knob, "settle", YX),
        Waypoint(1.0, (BAR_X, KNOB_Y, kz), g_knob, "grip the knob", YX),
        Waypoint(0.4, (BAR_X, KNOB_Y, kz), g_knob, "settle", YX),
        Waypoint(1.5, (BAR_X, KNOB_Y, kz + 0.016), g_knob, "lift the bar off the sheet", YX),
        Waypoint(0.4, (BAR_X, KNOB_Y, kz + 0.016), g_knob, "hold", YX),
        Waypoint(0.8, (BAR_X, KNOB_Y, kz + 0.016), o_knob, "let go", YX),
        Waypoint(1.0, (BAR_X, KNOB_Y, TRAVEL), o_knob, "up", YX),
        Waypoint(1.5, (fx, 0.0, TRAVEL), o_fl, "over the flange", YX),
        Waypoint(1.2, (fx, 0.0, fz + 0.020), o_fl, "descend", YX),
        Waypoint(0.8, (fx, 0.0, fz), o_fl, "down round the flange", YX),
        Waypoint(0.4, (fx, 0.0, fz), o_fl, "settle", YX),
        Waypoint(1.0, (fx, 0.0, fz), C, "grip the flange", YX),
        Waypoint(0.5, (fx, 0.0, fz), C, "settle", YX),
        Waypoint(3.0, (fx - out, 0.0, fz), C, "slide the bracket out from under the bar", YX),
        Waypoint(0.4, (fx - out, 0.0, fz), C, "settle", YX),
        Waypoint(1.5, (fx - out, 0.0, TRAVEL), C, "lift", YX),
        Waypoint(1.2, (CARRY_X, 0.0, TRAVEL), C, "back toward the robot, clear of the handles", YX),
        Waypoint(1.8, (CARRY_X, qy, TRAVEL), C, "across to the tray's side", YX),
        Waypoint(1.2, (qx, qy, TRAVEL), C, "over the tray", YX),
        Waypoint(0.5, (qx, qy, TRAVEL), C, "settle over", YX),
        Waypoint(1.5, (qx, qy, qz + 0.020), C, "lower", YX),
        Waypoint(1.2, (qx, qy, qz), C, "set down slowly", YX),
        Waypoint(0.5, (qx, qy, qz), C, "settle", YX),
        Waypoint(1.0, (qx, qy, qz), o_fl, "release", YX),
        Waypoint(1.2, (qx, qy, TRAVEL), o_fl, "retreat", YX),
    ]
    tail = _plan_s(right) - _plan_s(left)
    left += [Waypoint(tail, PARK["left"], C, "wait", YL)]
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of states (one trial each), built by the kit's forward kinematics."""
    import dataclasses

    global _TEST_TCPS

    def st(leaf=0.0, bend=0.0, bar=BAR_OPEN, base=None):
        """leaf/bend in degrees, bar travel in m, base = (dx, dz, tilt_deg about y at the bend line) or a tray pose."""
        moved = {}
        if base is not None:
            (dx, dy, dz), tilt = base
            t = math.radians(tilt)
            r = mk.axis_angle((0, 1, 0), t)
            moved["blank_base"] = (np.array([BEND_X + dx, dy, BED_TOP + dz]), mk.mat_to_quat(r))
        return kit.fake_states(q={"leaf_hinge": math.radians(leaf), "bend": math.radians(bend), "clamp": bar},
                               moved=moved)

    on_tray_pose = ((TRAY_XY[0] - 0.040 - BEND_X, TRAY_XY[1], TRAY_TOP - BED_TOP), 0.0)
    start = st()
    clamped = st(bar=0.0)
    bent = st(leaf=90.0, bend=90.0, bar=0.0)
    bent_half = st(leaf=50.0, bend=50.0, bar=0.0)
    leaf_down = st(leaf=0.0, bend=90.5, bar=0.0)
    unclamped = st(leaf=0.0, bend=90.5, bar=BAR_OPEN)
    out = st(bend=90.5, bar=BAR_OPEN, base=((-0.09, 0.0, 0.0), 0.0))
    held = st(bend=90.5, bar=BAR_OPEN, base=((-0.09, 0.0, 0.10), 0.0))
    done = st(bend=90.5, bar=BAR_OPEN, base=on_tray_pose)
    done_84 = st(bend=84.0, bar=BAR_OPEN, base=on_tray_pose)
    # the naive pull on the leaf with the bar up: the whole blank rides up with it, pivoting on the bed's edge
    ride = st(leaf=30.0, bend=60.0, bar=BAR_OPEN, base=((0.0, 0.0, 0.0), -30.0))
    ride_clamped_bar = st(leaf=30.0, bend=60.0, bar=0.0, base=((0.0, 0.0, 0.0), -30.0))
    unclamp_leaf_up = st(leaf=90.0, bend=90.0, bar=BAR_OPEN)
    over = st(leaf=110.0, bend=110.0, bar=0.0)
    hand_bent = st(leaf=0.0, bend=90.0, bar=0.0)
    hand_bent_tray = st(bend=90.0, bar=BAR_OPEN, base=on_tray_pose)
    moving = dict(done)
    moving["blank_base"] = dataclasses.replace(done["blank_base"], lin_vel=np.array([0.05, 0.0, 0.0]))
    grip = kit.point(done, "flange", (-FLANGE + 0.01, 0.0, 0.0))

    cases = {  # name: (states in one trial, gripper TCPs, final score, success)
        "start": ([start], [], 0.0, False),
        "bar pressed down": ([start, clamped], [], 0.0, False),
        "clamped, leaf raised to 50 deg": ([start, clamped, bent_half], [], 0.25, False),
        "clamped, bent square, leaf still up": ([start, clamped, bent], [], 0.25, False),
        "bent square, leaf lowered": ([start, clamped, bent, leaf_down], [], 0.5, False),
        "leaf lowered, bar lifted": ([start, clamped, bent, leaf_down, unclamped], [], 0.5, False),
        "bracket slid out and lifted": ([start, clamped, bent, leaf_down, unclamped, out, held], [], 0.5, False),
        "full job: bracket on the tray": ([start, clamped, bent, leaf_down, unclamped, out, held, done], [], 1.0,
                                          True),
        "on the tray but still gripped": ([start, clamped, bent, leaf_down, unclamped, held, done], [grip], 0.5,
                                          False),
        "on the tray, still sliding": ([start, clamped, bent, leaf_down, unclamped, held, moving], [], 0.5, False),
        "on the tray but only 84 deg": ([start, clamped, bent, leaf_down, unclamped, done_84], [], 0.25, False),
        "naive: leaf raised with the bar up (blank rides up)": ([start, ride], [], 0.0, False),
        "naive, then everything reset by hand and done right":
            ([start, ride, start, clamped, bent, leaf_down, unclamped, held, done], [], 0.0, False),
        "blank rode up although the bar reads down": ([start, clamped, ride_clamped_bar], [], 0.0, False),
        "bar lifted before the leaf was lowered": ([start, clamped, bent, unclamp_leaf_up, unclamped, held, done], [],
                                                   0.0, False),
        "over-bent to 110 deg, then finished": ([start, clamped, over, bent, leaf_down, unclamped, held, done], [],
                                                0.25, False),
        "flange bent with the leaf down (not by the brake)": ([start, clamped, hand_bent, unclamped, hand_bent_tray],
                                                              [], 0.0, False),
        "a new trial after the naive fault starts clean":
            ([start, ride, None, start, clamped, bent, leaf_down, unclamped, held, done], [], 1.0, True),
    }
    try:
        for name, (seq, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for s in seq[:-1]:
                if s is None:
                    mk.new_episode()
                else:
                    score(s)
            _TEST_TCPS = tcps
            got, got_success = score(seq[-1]), success(seq[-1])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        # readouts on the kit's poses
        assert abs(flange_deg(done) - 90.5) < 1e-6 and abs(leaf_deg(bent) - 90.0) < 1e-6
        assert abs(bar_lift(start) - BAR_OPEN) < 1e-9 and _heel_up(ride["blank_base"])
        assert not _heel_up(bent["blank_base"])
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")

