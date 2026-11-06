"""On the station table, facing the arms, stands the front of a small open-frame equipment rack: two steel posts
26 cm apart (inner faces), a 1U unit fixed in the rack above a rack-mount LCD console drawer (a KVM keyboard tray with a
fold-down screen) and the equipment below it. The console drawer is pulled out toward the arms on its rails, its
screen standing open on a hinge at the drawer's rear edge, 2 cm in front of the rack face. The job is to stow
the console: put the drawer back into the rack. Pushed in with the screen up, the screen strikes the 1U unit above
(the drawer's top is 2.5 cm below that unit), so the screen must first be folded flat onto the keyboard. Scored
from the drawer's and the screen's poses.

The drawer runs on a slider and the screen on a hinge carried by the drawer (mechkit.py, in this folder, adds both
to the stock runtime in memory).

Run it:  inspect-robots run --embodiment yam_arena \\
           -E task=tasks/console-drawer-fold-before-stow/console_drawer_fold_before_stow.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (installs the slider + hinge hooks)

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
RACK_X = 0.330  # the rack's front face (the posts' and the fixed units' front faces)
POST = (0.020, 0.020, 0.118)  # up to the top of the unit above
POST_Y = 0.140  # post centres; inner faces 13 cm either side of the midline
DEPTH = 0.260  # the fixed units' depth into the rack
UNIT_W = 0.240  # the fixed units' width (they sit between the posts)

# -- the console drawer (free, on a slider along x) and its screen (free, hinged at the drawer's rear edge) --
TRAY = (0.160, 0.240, 0.015)  # depth (x), width (y), thickness: the keyboard tray
BEZEL = (0.012, 0.240, 0.025)  # the front bezel with its pull handle, in front of the tray
DRAWER_TOP = TOP + 0.050  # the keyboard surface
HOME_FRONT = RACK_X - 0.035  # bezel front face with the drawer stowed (the bezel stands proud of the posts, as on real units)
TRAVEL = 0.157  # how far the drawer is pulled out at the start (to its stop)
LID = (0.130, 0.220, 0.006)  # screen panel: height from the hinge, width, thickness
LID_OPEN_DEG = 95.0  # standing open, 5 degrees past upright
DRAWER_KG, LID_KG = 0.400, 0.120
RAIL_FRICTION = 2.0  # N: the slide rails' drag, so the drawer stays where it is pushed
LID_FRICTION = 0.12  # N*m: the screen hinge's friction (its weight gives at most 0.077 N*m), it holds any angle

UNIT_ABOVE_Z = (DRAWER_TOP + 0.025, DRAWER_TOP + 0.069)  # the 1U unit above: underside 2.5 cm over the keyboard
UNIT_BELOW_TOP = DRAWER_TOP - TRAY[2] - 0.003

STEEL = (0.20, 0.21, 0.23)
UNIT_GREY = (0.55, 0.56, 0.58)
TRAY_BLACK = (0.10, 0.10, 0.11)
KEYS = (0.30, 0.30, 0.32)
SCREEN = (0.05, 0.07, 0.12)
BEZEL_GREY = (0.32, 0.33, 0.35)

_TRAY_X0 = HOME_FRONT + BEZEL[0]  # the tray's front edge at home
_HINGE_HOME = (_TRAY_X0 + TRAY[0], 0.0, DRAWER_TOP)  # the tray's rear top edge at home: the screen hinge line

kit = mk.Kit()
# The drawer's frame sits at the tray's front top edge (home position = reference pose).
kit.part("drawer", (_TRAY_X0, 0.0, DRAWER_TOP), [
    mk.box(TRAY, at=(TRAY[0] / 2, 0.0, -TRAY[2] / 2)),
    mk.box((0.100, 0.200, 0.002), at=(0.070, 0.0, 0.001), color=KEYS),  # the keyboard
    mk.box(BEZEL, at=(-BEZEL[0] / 2, 0.0, -TRAY[2] + BEZEL[2] / 2), color=BEZEL_GREY),
], mass=DRAWER_KG, color=TRAY_BLACK)
kit.slider("rails", None, "drawer", axis=(1.0, 0.0, 0.0), lower=-TRAVEL, upper=0.0, start=-TRAVEL,
           friction=RAIL_FRICTION)
# The screen's frame sits on the hinge line; in the reference pose (hinge 0) it lies flat on the keyboard, its top
# edge toward the front; a positive angle raises it.
kit.part("screen", _HINGE_HOME, [mk.box(LID, at=(-LID[0] / 2, 0.0, LID[2] / 2))], mass=LID_KG, color=SCREEN)
kit.hinge("screen_hinge", "drawer", "screen", anchor=_HINGE_HOME, axis=(0.0, 1.0, 0.0), lower_deg=0.0,
          upper_deg=105.0, start_deg=LID_OPEN_DEG, friction=LID_FRICTION)


def _unit(name, z0, z1):
    return Box(name, pos=(RACK_X + DEPTH / 2, 0.0, (z0 + z1) / 2), size=(DEPTH, UNIT_W, z1 - z0), color=UNIT_GREY)


FIXTURES = [
    *(Box(f"post_{s}", pos=(RACK_X + POST[0] / 2, sy * POST_Y, TOP + POST[2] / 2), size=POST, color=STEEL)
      for s, sy in (("left", 1.0), ("right", -1.0))),
    _unit("unit_above", *UNIT_ABOVE_Z),
    _unit("unit_below", TOP, UNIT_BELOW_TOP),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Rack-mount LCD console manuals: fold the screen down flat onto the keyboard before sliding the drawer back into
# the rack; with the screen raised it strikes the equipment above, which breaks the panel or its hinge. Everything
# is measured from the two parts' poses: the drawer's travel along its rails, the screen's angle to the keyboard,
# and whether any point of the screen came within 3 mm of the unit above.

HOME_TOL = 0.005  # stowed: the drawer within 5 mm of its stop
HALF_IN = -TRAVEL / 2
FLAT_DEG = 4.0
STRIKE_GAP = 0.003
CLEAR = 0.030  # both grippers' TCPs at least 3 cm from the drawer and the screen
_STRIKE_KEY = "cdf_screen_struck"
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
# Points on the screen panel (its frame): a 7 x 3 grid over its back face, plus its top edge's front corners.
_SCREEN_PTS = [(-LID[0] * i / 6, sy * LID[1] / 2, LID[2]) for i in range(7) for sy in (-1.0, 0.0, 1.0)] + \
              [(-LID[0], sy * LID[1] / 2, 0.0) for sy in (-1.0, 1.0)]


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else [np.asarray(t, float) for t in _TEST_TCPS]


def _released(st, margin=CLEAR) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())


def _travel(objects) -> float:
    """Drawer travel along its rails, m: 0 stowed, -0.20 pulled out to its stop."""
    return kit.q(objects, "rails")


def _screen_deg(objects) -> float:
    return kit.q_deg(objects, "screen_hinge")


def _hits_unit_above(objects) -> bool:
    """Some point of the screen within 3 mm of the 1U unit above (in front of or inside its box)."""
    for p in _SCREEN_PTS:
        x, y, z = kit.point(objects, "screen", p)
        if (x >= RACK_X - STRIKE_GAP and abs(y) <= UNIT_W / 2 + STRIKE_GAP
                and UNIT_ABOVE_Z[0] - STRIKE_GAP <= z <= UNIT_ABOVE_Z[1] + STRIKE_GAP):
            return True
    return False


def _struck(objects) -> bool:
    """Latched for the trial: the screen ever struck the unit above."""
    if _hits_unit_above(objects):
        mk.MEMORY[_STRIKE_KEY] = True
    return bool(mk.MEMORY.get(_STRIKE_KEY))


def score(objects) -> float:
    """0 for the rest of the trial once the screen has struck the unit above. Otherwise, with the drawer and the
    screen let go of (both TCPs 3 cm clear) and still: 1.0 for the screen flat (within 4 degrees) and the drawer
    stowed (within 5 mm of its stop); 0.5 for the screen flat and the drawer at least half way in; 0.25 for the
    screen flat; else 0."""
    if _struck(objects):
        return 0.0
    dr, sc = objects["drawer"], objects["screen"]
    if not (_released(dr) and _released(sc) and dr.resting() and sc.resting()):
        return 0.0
    if _screen_deg(objects) > FLAT_DEG:
        return 0.0
    d = _travel(objects)
    if d >= -HOME_TOL:
        return 1.0
    return 0.5 if d >= HALF_IN else 0.25


def success(objects) -> bool:
    """Screen folded flat, drawer stowed, nothing struck, both let go and still; holds 1 s."""
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Metres still to go: the screen's top edge's arc to flat plus the drawer's travel to its stop."""
    return float(math.radians(max(0.0, _screen_deg(objects))) * LID[0] + max(0.0, -_travel(objects)))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the fixed scene geometry (where the drawer and the screen hinge sit when pulled out); it reads no
# joint or simulator state. One arm: the closed finger pair comes in from the right, just above the unit above and
# behind the screen, pushes the screen's back face along an arc 8.5 cm out from its hinge until it lies on the
# keyboard, presses it flat and lifts away; then it comes down in front of the drawer's bezel and pushes the drawer
# home (1.2 cm/s on average).

TIP = 0.013  # pad tips below the TCP
HALF_X = 0.010  # closed pads' half thickness along their closing axis (yaw pi/2: along x)
HALF_PUSH = 0.019  # closed pads' half width across it (yaw 0: along x)
REACH = 0.085
GAP0 = 0.025
PUSH = -0.003
PRESS = -0.004
ARC = (85, 72, 58, 44, 30, 18, 8, 2)
C = FINGER_CLOSED
YAW_FOLD = math.pi / 2  # fingers close along x: the pads' 38 mm width lies along the hinge line
YAW_PUSH = 0.0  # fingers close along y: the pads' 38 mm side pushes the bezel
HINGE_OUT = np.array([_HINGE_HOME[0] - TRAVEL, 0.0, DRAWER_TOP])  # the screen hinge with the drawer pulled out
BEZEL_OUT = HOME_FRONT - TRAVEL  # the bezel's front face, pulled out
PUSH_Z = DRAWER_TOP - TRAY[2] + 0.004 + TIP  # pad tips 4 mm above the bezel's bottom edge (bezel is 25 mm tall)
PUSH_END = HOME_FRONT + 0.014  # command the pad face 14 mm past the stowed bezel face: the arm lags ~14 mm against
# the rails' drag (measured in sim), so the drawer reaches its stop
FOLD_Y = -0.080  # the right arm pushes the screen 8 cm right of its centre (its reach is better there)
SIDE_Y = -0.200  # outside the screen and the right post: where the arm comes in at the fold height
PARK = {"left": (0.200, 0.250, 0.950), "right": (0.200, -0.250, 0.950)}


def _edge_tcp(deg, gap, reach=REACH):
    """TCP that puts the closed pads' lower front edge ``gap`` behind the screen's back face, ``reach`` out from
    its hinge, with the screen open ``deg`` degrees."""
    a = math.radians(deg)
    d = np.array([-math.cos(a), 0.0, math.sin(a)])  # along the screen, hinge to top edge
    n = np.array([math.sin(a), 0.0, math.cos(a)])  # the back face's outward normal
    edge = HINGE_OUT + reach * d + (LID[2] + gap) * n
    return tuple(float(v) for v in edge + np.array([HALF_X, FOLD_Y, TIP]))


def _fold():
    start = _edge_tcp(LID_OPEN_DEG, GAP0)
    flat = _edge_tcp(0.0, PRESS)
    side = (start[0], SIDE_Y, start[2] + 0.006)
    return [
        Waypoint(2.5, (start[0] - 0.08, SIDE_Y, start[2] + 0.04), C, "out to the right of the rack", YAW_FOLD),
        Waypoint(1.5, side, C, "level with the screen's top, right of the post", YAW_FOLD),
        Waypoint(2.0, (start[0], start[1], start[2] + 0.006), C, "in behind the screen, over the unit above",
                 YAW_FOLD),
        Waypoint(1.0, start, C, "down behind the screen", YAW_FOLD),
        Waypoint(0.5, start, C, "settle", YAW_FOLD),
        *(Waypoint(1.0, _edge_tcp(deg, PUSH), C, f"fold the screen to {deg} deg", YAW_FOLD) for deg in ARC),
        Waypoint(0.8, flat, C, "press it flat", YAW_FOLD),
        Waypoint(0.6, flat, C, "hold", YAW_FOLD),
        Waypoint(1.2, (flat[0], flat[1], flat[2] + 0.060), C, "lift clear", YAW_FOLD),
    ]


def _stow():
    x0 = BEZEL_OUT - 0.015 - HALF_PUSH
    x1 = PUSH_END - HALF_PUSH
    above = (x0, 0.0, PUSH_Z + 0.060)
    return [
        Waypoint(2.0, above, C, "in front of the drawer", YAW_PUSH),
        Waypoint(1.5, (x0, 0.0, PUSH_Z), C, "down to the bezel", YAW_PUSH),
        Waypoint(0.5, (x0, 0.0, PUSH_Z), C, "settle", YAW_PUSH),
        Waypoint(1.0, (BEZEL_OUT - HALF_PUSH - 0.002, 0.0, PUSH_Z), C, "touch the bezel", YAW_PUSH),
        Waypoint(13.0, (x1, 0.0, PUSH_Z), C, "push the drawer home", YAW_PUSH),
        Waypoint(0.6, (x1, 0.0, PUSH_Z), C, "hold at the stop", YAW_PUSH),
        Waypoint(1.0, (x1 - 0.050, 0.0, PUSH_Z), C, "back off", YAW_PUSH),
        Waypoint(1.5, (x1 - 0.080, 0.0, PUSH_Z + 0.100), C, "up and away", YAW_PUSH),
    ]


def demo():
    """The right arm folds the screen flat onto the keyboard (pushing its back face from behind, just over the
    unit above), then pushes the drawer home by its bezel and backs away; the left arm stays parked."""
    right = _fold() + _stow() + [Waypoint(1.5, PARK["right"], C, "park", YAW_PUSH)]
    left = [Waypoint(sum(w.duration for w in right), PARK["left"], C, "park", 0.0)]
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built poses (single states and short sequences, one trial each)."""
    import dataclasses

    global _TEST_TCPS

    def st(travel=-TRAVEL, deg=LID_OPEN_DEG, vel=0.0):
        s = kit.fake_states(q={"rails": travel, "screen_hinge": math.radians(deg)})
        if vel:
            s["drawer"] = dataclasses.replace(s["drawer"], lin_vel=np.array([vel, 0.0, 0.0]))
        return s

    start = st()
    flat_out = st(deg=0.0)
    stowed = st(0.0, 0.0)
    half = st(-0.07, 0.0)
    quarter = st(-0.12, 0.0)
    up_pushed = st(-TRAVEL + 0.04, LID_OPEN_DEG)  # pushed 4 cm with the screen up: in contact with the unit above
    up_short = st(-TRAVEL + 0.005, LID_OPEN_DEG)  # pushed 5 mm: not yet touching
    folded_under = st(0.0, 9.0)  # the unit above folded the screen down to 9 degrees as the drawer went in
    part_fold_in = st(-0.05, 30.0)
    hand_tcp = kit.point(stowed, "drawer", (-BEZEL[0] - 0.012, 0.0, -0.005))
    cases = {  # name: (sequence of states in one trial, gripper TCPs, final score)
        "start": ([start], [], 0.0),
        "screen folded flat, drawer still out": ([start, flat_out], [], 0.25),
        "screen flat, drawer pushed 3.7 cm in": ([start, flat_out, quarter], [], 0.25),
        "screen flat, drawer half way in": ([start, flat_out, half], [], 0.5),
        "folded, then stowed": ([start, flat_out, half, stowed], [], 1.0),
        "stowed, but the gripper still on the bezel": ([start, flat_out, stowed], [hand_tcp], 0.0),
        "stowed, drawer still sliding": ([start, flat_out, st(0.0, 0.0, vel=0.02)], [], 0.0),
        "stowed, screen 6 degrees up": ([start, flat_out, st(0.0, 6.0)], [], 0.0),
        "naive: pushed in with the screen up (strike), screen folded under the unit": (
            [start, up_pushed, folded_under], [], 0.0),
        "strike, then pulled out, folded and stowed properly": (
            [start, up_pushed, start, flat_out, stowed], [], 0.0),
        "screen half folded (30 deg) and pushed in": ([start, part_fold_in], [], 0.0),
        "pushed 5 mm with the screen up (no contact yet), then folded and stowed": (
            [start, up_short, flat_out, stowed], [], 1.0),
        "stowed end state, no history": ([stowed], [], 1.0),
        "screen open, drawer at the stop (no history)": ([st(0.0, LID_OPEN_DEG)], [], 0.0),
    }
    try:
        for name, (seq, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            for s in seq[:-1]:
                score(s)
            got, ok = score(seq[-1]), success(seq[-1])
            assert abs(got - want) < 1e-9 and ok == (want == 1.0), (name, got, ok, want)
        assert not _hits_unit_above(start) and not _hits_unit_above(up_short) and _hits_unit_above(up_pushed)
        assert not _hits_unit_above(stowed) and _hits_unit_above(part_fold_in)
        mk.new_episode()
        _TEST_TCPS = []
        score(up_pushed)
        mk.new_episode()
        assert score(stowed) == 1.0, "a new trial forgets the strike"
        assert abs(distance(stowed)) < 1e-6 and distance(start) > 0.3
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1


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

