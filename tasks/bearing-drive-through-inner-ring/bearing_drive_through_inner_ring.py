"""On the station table, in front of the left arm, a steel shaft 40 mm across stands upright in a grey cast-iron
base, with a shoulder collar 2.5 cm above the table. A new deep-groove ball bearing (40 mm bore, 80 mm outside, 18 mm
wide) has been started on the shaft by hand: it sits on the shaft 3 cm above the shoulder, and the shaft stands
3.5 cm above it. The bearing's inner ring is an interference fit on the shaft; the balls and their cage, between the
rings, show as a dark band 3 mm below the faces. A fitting sleeve (a nylon tube, 48 mm bore, 54 mm outside, 9.5 cm
long, with a steel impact cap) stands on a white locating peg 15 cm to the left. The job is to fit the bearing down
against the shoulder. Scored from the rings' and the sleeve's poses.

The press fit is a slider along the shaft with Coulomb friction (2.5 N, mechkit.py in this folder): the inner ring
moves only under a push of more than 2.5 N and stays where it is left; 3 cm down it stops on the shoulder. The outer
ring is held to the inner ring only through the balls: a bond that breaks at 1.2 N (brinelled raceways). So any
push on the outer ring big enough to move the bearing breaks the bond first, and a push on the inner ring alone
(the sleeve's end bears only on the inner ring's face) moves it without loading the bond. Once the bond has broken
the trial scores 0 for good.

Run it:  inspect-robots run --embodiment yam_arena \\
           -E task=tasks/bearing-drive-through-inner-ring/bearing_drive_through_inner_ring.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (slider and bond; installs its yam_arena hooks in memory, incl. mk.MEMORY)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
SX, SY = 0.310, 0.160  # shaft axis
SHAFT_R = 0.020
BASE_H = 0.015  # cast-iron base block
SHOULDER = TOP + 0.025  # top of the shoulder collar: the bearing's seat
COLLAR_R = 0.026
TRAVEL = 0.030  # the bearing starts this far above the shoulder
BW = 0.018  # bearing width
SHAFT_TOP = SHOULDER + TRAVEL + BW + 0.035
IR_IN, IR_OUT = 0.0215, 0.028  # inner ring: bore (flats; 1.5 mm clear of the shaft) and outside
CAGE_IN, CAGE_OUT, RECESS = 0.0283, 0.033, 0.003  # balls and cage, recessed 3 mm below both faces
OR_IN, OR_OUT = 0.033, 0.040  # outer ring
IR_KG, OR_KG = 0.045, 0.030
FIT_N = 2.5  # press-fit friction along the shaft
BOND_N = 1.2  # the balls' hold of the outer ring on the inner ring before the raceways brinell
SL_IN, SL_OUT, SL_LEN, CAP = 0.024, 0.027, 0.095, 0.006  # fitting sleeve: bore, outside (flats), length, cap
SL_KG = 0.045
HX, HY = 0.240, 0.290  # sleeve's locating peg
PEG_R, PEG_H = 0.021, 0.020

BEARING_Z0 = SHOULDER + TRAVEL + BW / 2  # the bearing's centre at the start

STEEL = (0.70, 0.71, 0.74)
BEARING = (0.80, 0.81, 0.84)
CAGE = (0.22, 0.20, 0.18)
IRON = (0.42, 0.44, 0.46)
NYLON = (0.15, 0.35, 0.75)
WHITE = (0.92, 0.92, 0.90)


def _ring(r_in, r_out, h, z=0.0, n=8, color=None):
    """An n-sided ring of boxes: flats at r_in (inside) and r_out (outside), height h, centred at z."""
    t = 2.0 * r_out * math.tan(math.pi / n)
    rm = (r_in + r_out) / 2
    return [mk.box((r_out - r_in, t, h), at=(rm * math.cos(a), rm * math.sin(a), z), yaw=a, color=color)
            for a in (2 * math.pi * k / n for k in range(n))]


kit = mk.Kit()
kit.part("inner_ring", (SX, SY, BEARING_Z0), _ring(IR_IN, IR_OUT, BW), mass=IR_KG, color=BEARING)
kit.part("outer_ring", (SX, SY, BEARING_Z0),
         [*_ring(CAGE_IN, CAGE_OUT, BW - 2 * RECESS, color=CAGE), *_ring(OR_IN, OR_OUT, BW)], mass=OR_KG, color=BEARING)
_sl_z0 = TOP + SL_LEN / 2  # standing on the table round its peg
kit.part("sleeve", (HX, HY, _sl_z0),
         [*_ring(SL_IN, SL_OUT, SL_LEN - CAP, z=-CAP / 2),
          *(mk.box((2 * SL_OUT, 2 * SL_OUT * math.tan(math.pi / 8), CAP), at=(0, 0, SL_LEN / 2 - CAP / 2), yaw=a,
                   color=STEEL) for a in (0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4))],
         mass=SL_KG, color=NYLON)
# the press fit: the inner ring slides down the shaft (negative travel along +z) against 2.5 N of friction, to the
# shoulder
kit.slider("press_fit", None, "inner_ring", axis=(0, 0, 1), lower=-TRAVEL, upper=0.0, friction=FIT_N)
# the balls: the outer ring hangs on the inner ring; 1.2 N through them brinells the raceways (the bond breaks)
kit.bond("balls", "inner_ring", "outer_ring", break_force=BOND_N, anchor=(SX, SY, BEARING_Z0))

FIXTURES = [
    Box("base", pos=(SX, SY, TOP + BASE_H / 2), size=(0.110, 0.110, BASE_H), color=IRON),
    Cylinder("collar", pos=(SX, SY, (TOP + BASE_H + SHOULDER) / 2), radius=COLLAR_R, height=SHOULDER - TOP - BASE_H,
             color=STEEL),
    Cylinder("shaft", pos=(SX, SY, (TOP + BASE_H + SHAFT_TOP) / 2), radius=SHAFT_R, height=SHAFT_TOP - TOP - BASE_H,
             color=STEEL),
    Cylinder("sleeve_peg", pos=(HX, HY, TOP + PEG_H / 2), radius=PEG_R, height=PEG_H, color=WHITE),
]
OBJECTS = [*kit.objects(), *FIXTURES]
PARTS = ("inner_ring", "outer_ring", "sleeve")

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Bearing makers' mounting rule: the mounting force goes into the ring with the interference fit (here the inner
# ring, on the shaft), never through the rolling elements. Measured from poses: how far the inner ring has moved
# down the shaft, and whether the outer ring has moved on the inner ring (the balls' bond broken: brinelled).

SEATED_TOL = 0.002  # the inner ring within 2 mm of the shoulder
HALF = TRAVEL / 2
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _released(st, margin=0.025) -> bool:
    """Neither gripper's TCP within 2.5 cm of the part's bounding box."""
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def travel(objects) -> float:
    """How far the inner ring has moved down the shaft from its start, metres."""
    return float(-kit.q(objects, "press_fit"))


def brinelled(objects) -> bool:
    """The outer ring has moved more than 2 mm (or 2 deg) on the inner ring at any moment of the trial."""
    return kit.broken(objects, "balls")


def score(objects) -> float:
    """1.0: the inner ring within 2 mm of the shoulder, the balls' bond intact, the bearing and the sleeve still and
    let go of (both grippers 2.5 cm clear). 0.5: the bond intact and the inner ring at least half way (15 mm) down.
    0 otherwise, and 0 for the rest of the trial once the outer ring has moved on the inner ring."""
    if brinelled(objects):
        return 0.0
    d = travel(objects)
    if (d >= TRAVEL - SEATED_TOL and all(_released(objects[p]) for p in PARTS)
            and all(objects[p].resting() for p in PARTS)):
        return 1.0
    return 0.5 if d >= HALF else 0.0


def success(objects) -> bool:
    """The bearing on its shoulder, undamaged, let go; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres the inner ring still has to go down to the shoulder."""
    return float(max(0.0, TRAVEL - travel(objects)))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm only, open loop from the scene's fixed geometry (it reads no joint or simulator state): pick the sleeve
# off its peg by its upper end, lower it over the shaft until it stands on the bearing (its end lands on the inner
# ring), let go, then press its cap down with the closed fingers until the bearing is on the shoulder, and lift away.

TIP = 0.013
SQUEEZE = 0.008
GRIP = -(2 * SL_OUT - SQUEEZE) / 2
GRIP_DOWN = 0.055  # pad tips 5.5 cm below the sleeve's top
TRAVEL_Z = TOP + 0.185
X_TRIM = 0.001  # the carried sleeve lands about 1 mm short in x (toward the arm base)
OVERSHOOT = 0.035  # the press is commanded 3.5 cm past the shoulder, so the arm keeps pushing on the stop
C, O = FINGER_CLOSED, FINGER_OPEN


def demo():
    """Sleeve over the shaft onto the inner ring, let go, press its cap 3 cm down to the shoulder, lift. About 29 s."""
    top0 = TOP + SL_LEN  # sleeve's top on its peg
    pick_z = top0 - GRIP_DOWN + TIP
    on_ring = SHOULDER + TRAVEL + BW + SL_LEN  # sleeve's top once it stands on the bearing
    set_z = on_ring - GRIP_DOWN + TIP + 0.002  # let go 2 mm above
    cap0, cap1 = on_ring + TIP, on_ring - TRAVEL + TIP - OVERSHOOT  # closed pad tips on the cap: start, end
    sx = SX + X_TRIM
    return {"left": [
        Waypoint(2.0, (HX, HY, TRAVEL_Z), O, "over the sleeve", 0.0),
        Waypoint(1.5, (HX, HY, pick_z + 0.020), O, "descend", 0.0),
        Waypoint(1.0, (HX, HY, pick_z), O, "down beside its upper end", 0.0),
        Waypoint(0.5, (HX, HY, pick_z), O, "settle", 0.0),
        Waypoint(1.2, (HX, HY, pick_z), GRIP, "close", 0.0),
        Waypoint(0.5, (HX, HY, pick_z), GRIP, "settle closed", 0.0),
        Waypoint(2.0, (HX, HY, TRAVEL_Z), GRIP, "lift off the peg", 0.0),
        Waypoint(2.0, (sx, SY, TRAVEL_Z), GRIP, "carry over the shaft", 0.0),
        Waypoint(0.8, (sx, SY, TRAVEL_Z), GRIP, "settle over", 0.0),
        Waypoint(2.5, (sx, SY, set_z + 0.015), GRIP, "lower over the shaft", 0.0),
        Waypoint(1.5, (sx, SY, set_z), GRIP, "onto the bearing", 0.0),
        Waypoint(0.5, (sx, SY, set_z), GRIP, "settle", 0.0),
        Waypoint(1.2, (sx, SY, set_z), O, "let go", 0.0),
        Waypoint(1.5, (SX, SY, cap0 + 0.009), O, "up over the cap", 0.0),
        Waypoint(1.0, (SX, SY, cap0 + 0.009), C, "close the fingers", 0.0),
        Waypoint(1.5, (SX, SY, cap0 + 0.003), C, "down onto the cap", 0.0),
        Waypoint(0.5, (SX, SY, cap0 + 0.003), C, "settle", 0.0),
        Waypoint(6.0, (SX, SY, cap1), C, "press the sleeve down", 0.0),
        Waypoint(1.5, (SX, SY, cap1), C, "hold on the shoulder", 0.0),
        Waypoint(1.5, (SX, SY, TRAVEL_Z), C, "lift away", 0.0),
        Waypoint(1.5, (0.220, 0.300, TRAVEL_Z), C, "park", 0.0),
    ]}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built poses (single states and short sequences, one trial each)."""
    import dataclasses

    global _TEST_TCPS
    q0 = mk.yaw_quat(0.0)

    def sleeve_at(d):  # the sleeve standing on the inner ring, the ring d down the shaft
        return (np.array([SX, SY, SHOULDER + TRAVEL - d + BW + SL_LEN / 2]), q0)

    def st(d=0.0, sleeve=None, outer_dz=None):
        moved = {}
        if sleeve is not None:
            moved["sleeve"] = sleeve
        if outer_dz is not None:
            moved["outer_ring"] = (np.array([SX, SY, BEARING_Z0 - d + outer_dz]), q0)
        return kit.fake_states(q={"press_fit": -d}, moved=moved)

    start = st()
    sleeve_on = st(0.0, sleeve_at(0.0))
    half = st(0.016, sleeve_at(0.016))
    seated = st(TRAVEL, sleeve_at(TRAVEL))
    nearly = st(TRAVEL - 0.003, sleeve_at(TRAVEL - 0.003))
    short = st(0.012, sleeve_at(0.012))
    pushed_outer = st(0.0, None, outer_dz=-0.005)  # the outer ring shoved 5 mm down on the inner ring
    outer_dropped = st(0.0, None, outer_dz=-(BEARING_Z0 - TOP - BASE_H - BW / 2))  # fallen onto the base
    seated_broken = st(TRAVEL, sleeve_at(TRAVEL), outer_dz=0.0)
    seated_tool_back = st(TRAVEL)  # the sleeve put back on its peg
    moving = dict(seated)
    moving["sleeve"] = dataclasses.replace(seated["sleeve"], lin_vel=np.array([0.0, 0.0, -0.05]))
    cap_tcp = np.array([SX, SY, SHOULDER + BW + SL_LEN + TIP])
    rim_tcp = np.array([SX + 0.036, SY, SHOULDER + BW + 0.010])  # pads on the outer ring

    cases = {  # name: (sequence of states in one trial, gripper TCPs, final score, success)
        "start": ([start], [], 0.0, False),
        "sleeve stood on the bearing": ([start, sleeve_on], [], 0.0, False),
        "pressed half way through the sleeve": ([start, sleeve_on, half], [], 0.5, False),
        "pressed only 12 mm": ([start, sleeve_on, short], [], 0.0, False),
        "pressed to the shoulder, let go": ([start, sleeve_on, half, seated], [], 1.0, True),
        "pressed to the shoulder, tool put back": ([start, sleeve_on, seated, seated_tool_back], [], 1.0, True),
        "3 mm short of the shoulder": ([start, sleeve_on, nearly], [], 0.5, False),
        "on the shoulder, gripper still on the cap": ([start, sleeve_on, seated], [cap_tcp], 0.5, False),
        "on the shoulder, sleeve still moving": ([start, sleeve_on, moving], [], 0.5, False),
        "naive: pads pushed the outer ring 5 mm down": ([start, pushed_outer], [rim_tcp], 0.0, False),
        "outer ring knocked off, fallen onto the base": ([start, outer_dropped], [], 0.0, False),
        "outer ring pushed first, then the sleeve to the shoulder": (
            [start, pushed_outer, sleeve_on, seated_broken], [], 0.0, False),
        "end state right, no history": ([seated], [], 1.0, True),
    }
    try:
        for name, (seq, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            for s in seq[:-1]:
                score(s)
            got, got_success = score(seq[-1]), success(seq[-1])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        # a new trial forgets the last one's broken bond
        mk.new_episode()
        _TEST_TCPS = []
        score(pushed_outer)
        assert score(seated) == 0.0
        mk.new_episode()
        assert score(seated) == 1.0, "reset"
        assert abs(travel(half) - 0.016) < 1e-9
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1



if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
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
