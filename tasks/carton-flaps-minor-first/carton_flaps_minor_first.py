"""On the station table, between the two arms, stands an open brown corrugated shipping carton (a regular slotted
carton, 12 cm deep, 22 cm wide, 8 cm tall) packed to the rim with goods. Its four flaps stand open, leaning a
little outward: a long flap hinged on the near side and one on the far side, and a short flap hinged on the left
end and one on the right end. The carton itself does not move. The job is to close the carton so it is ready for
taping. Scored from the flaps' poses at the end.

The flaps are hinged boards (mechkit.py, in this folder, adds the hinges to the stock runtime in memory).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/carton-flaps-minor-first/carton_flaps_minor_first.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (installs the hinge hooks)

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
CX, CY = 0.260, 0.000  # carton centre
OUT = (0.120, 0.220)  # outside depth (x) and width (y)
H = 0.080  # wall height (end walls)
WALL = 0.005  # double-wall corrugated board
FLAP_T = 0.003  # flap board thickness
RIM = TOP + H  # the end walls' top: the short flaps' crease line
RAISE = 0.005  # the long flaps' crease line sits this far above the end walls, so they close over the short ones
HX = OUT[0] / 2 - WALL / 2  # crease lines run along the wall centres
HY = OUT[1] / 2 - WALL / 2
LONG = (0.0555, 0.207)  # long flap: reach from its crease to the centre seam (x), length (y)
SHORT = (0.104, 0.055)  # short flap: width (x), reach from its crease (y)
LONG_KG, SHORT_KG = 0.020, 0.015
OPEN_DEG = 100.0  # every flap starts standing open, 10 degrees past upright, against its stop
DAMPING = 0.02  # N*m*s/rad at each crease: the board resists quick motion; flaps stay open or closed by gravity

KRAFT = (0.62, 0.46, 0.30)
KRAFT_FLAP = (0.68, 0.52, 0.34)
GOODS = (0.86, 0.86, 0.84)

NEAR, FAR, LEFT, RIGHT = "flap_near", "flap_far", "flap_left", "flap_right"
LONG_FLAPS, SHORT_FLAPS = (NEAR, FAR), (LEFT, RIGHT)
FLAPS = (*SHORT_FLAPS, *LONG_FLAPS)

kit = mk.Kit()
kit.part("carton", (CX, CY, TOP), [
    mk.box((OUT[0], OUT[1], 0.004), at=(0, 0, 0.002)),
    *(mk.box((WALL, OUT[1], H + RAISE), at=(sx * HX, 0, (H + RAISE) / 2)) for sx in (-1, 1)),
    *(mk.box((OUT[0] - 2 * WALL, WALL, H), at=(0, sy * HY, H / 2)) for sy in (-1, 1)),
], color=KRAFT)
# Each flap's frame sits on its crease line; in the reference pose (hinge angle 0) it lies flat, closed. A positive
# angle opens it (the axis is chosen so the flap's free edge lifts).
_FLAP_DEFS = {
    NEAR: ((CX - HX, CY, RIM + RAISE), (LONG[0], LONG[1], FLAP_T), (LONG[0] / 2, 0, FLAP_T / 2), (0, -1, 0), LONG_KG),
    FAR: ((CX + HX, CY, RIM + RAISE), (LONG[0], LONG[1], FLAP_T), (-LONG[0] / 2, 0, FLAP_T / 2), (0, 1, 0), LONG_KG),
    LEFT: ((CX, CY + HY, RIM), (SHORT[0], SHORT[1], FLAP_T), (0, -SHORT[1] / 2, FLAP_T / 2), (-1, 0, 0), SHORT_KG),
    RIGHT: ((CX, CY - HY, RIM), (SHORT[0], SHORT[1], FLAP_T), (0, SHORT[1] / 2, FLAP_T / 2), (1, 0, 0), SHORT_KG),
}
for _name, (_crease, _size, _at, _axis, _kg) in _FLAP_DEFS.items():
    kit.part(_name, _crease, [mk.box(_size, at=_at)], mass=_kg, color=KRAFT_FLAP)
    kit.hinge(f"{_name}_crease", "carton", _name, anchor=_crease, axis=_axis, lower_deg=-8.0, upper_deg=OPEN_DEG,
              start_deg=OPEN_DEG, damping=DAMPING)

GOODS_TOP = RIM - 0.001  # packed to 1 mm below the rim
PACKED_GOODS = Box("packed_goods", pos=(CX, CY, (TOP + 0.004 + GOODS_TOP) / 2),
                   size=(OUT[0] - 2 * WALL - 0.002, OUT[1] - 2 * WALL - 0.002, GOODS_TOP - TOP - 0.004), color=GOODS)
OBJECTS = [*kit.objects(), PACKED_GOODS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# The closing rule of a regular slotted carton (FEFCO 0201): the short (minor) end flaps fold in first, then the two
# long (major) flaps close over them and meet at the centre seam, where the tape goes. Everything below is measured
# from the flap boards' poses: their tilt from level, which board lies on top where they overlap, and the seam gap.

FLAT_DEG = 10.0  # a flap counts as closed when its board is within 10 degrees of level
SEAM_MAX = 0.010  # the two long flaps' free edges no more than 10 mm apart
LAYER_TOL = 0.0015
_ORDER_KEY = "cfm_long_flap_closed_first"
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _tilt_deg(st) -> float:
    """Angle of the flap board's face from level, degrees (from its orientation)."""
    x, y, _, _ = (float(v) for v in st.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _released(st, margin=0.025) -> bool:
    """Neither gripper's TCP within 2.5 cm of the flap's bounding box."""
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _closed(objects, name) -> bool:
    return _tilt_deg(objects[name]) <= FLAT_DEG


def _flat(objects, name) -> bool:
    """Closed, still and let go of."""
    st = objects[name]
    return _closed(objects, name) and st.resting() and _released(st)


def _face_z(objects, name, local_z, x, y) -> float:
    """Height of one face of a flap board (local_z 0: underside, FLAP_T: top side) above the world point (x, y)."""
    p0 = kit.point(objects, name, (0.0, 0.0, local_z))
    n = np.cross(kit.point(objects, name, (0.01, 0.0, local_z)) - p0, kit.point(objects, name, (0.0, 0.01, local_z)) - p0)
    return float(p0[2] - (n[0] * (x - p0[0]) + n[1] * (y - p0[1])) / n[2])


# Where each long flap overlaps each short flap when both are closed: halfway out along both boards.
_OVERLAP = {(lf, sf): (CX + sx * (HX - LONG[0] / 2), CY + sy * (HY - SHORT[1] / 2))
            for lf, sx in ((NEAR, -1), (FAR, 1)) for sf, sy in ((LEFT, 1), (RIGHT, -1))}


def _over(objects, long_flap, short_flap) -> bool:
    """The long flap's underside is above the short flap's top side where they overlap: it lies on top."""
    x, y = _OVERLAP[(long_flap, short_flap)]
    return _face_z(objects, long_flap, 0.0, x, y) >= _face_z(objects, short_flap, FLAP_T, x, y) - LAYER_TOL


def _seam(objects) -> float:
    """Gap between the two long flaps' free edges (top corners at mid-length), metres."""
    near = kit.point(objects, NEAR, (LONG[0], 0.0, FLAP_T))
    far = kit.point(objects, FAR, (-LONG[0], 0.0, FLAP_T))
    return float(far[0] - near[0])


def _order_broken(objects) -> bool:
    """Latched for the trial: a long flap was ever closed while a short flap was still open."""
    if any(_closed(objects, f) for f in LONG_FLAPS) and not all(_closed(objects, f) for f in SHORT_FLAPS):
        mk.MEMORY[_ORDER_KEY] = True
    return bool(mk.MEMORY.get(_ORDER_KEY))


def score(objects) -> float:
    """0.25 for any two flaps closed flat (within 10 degrees of level), still and let go of; 0.5 for both short
    flaps flat with a long flap flat on top of both; 1.0 for all four flat with both long flaps on top of both short
    flaps and a seam gap of at most 10 mm. If a long flap was ever closed while a short flap was still open, the
    trial is capped at 0.25."""
    broken = _order_broken(objects)
    flat = {f: _flat(objects, f) for f in FLAPS}
    if broken:
        return 0.25 if sum(flat.values()) >= 2 else 0.0
    shorts = all(flat[f] for f in SHORT_FLAPS)
    on_top = {lf: shorts and flat[lf] and all(_over(objects, lf, sf) for sf in SHORT_FLAPS) for lf in LONG_FLAPS}
    if all(on_top.values()) and _seam(objects) <= SEAM_MAX:
        return 1.0
    if any(on_top.values()):
        return 0.5
    return 0.25 if sum(flat.values()) >= 2 else 0.0


def success(objects) -> bool:
    """The carton closed in the right layering, flaps still and let go of; holds for 1 s."""
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Arc length (m) the four flaps' free edges still have to travel to level."""
    return float(sum(math.radians(_tilt_deg(objects[f])) * (LONG[0] if f in LONG_FLAPS else SHORT[1]) for f in FLAPS))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the carton's fixed geometry; it reads no joint or simulator state. The closed finger pair (20 mm
# thick along the closing axis, 38 mm wide) pushes each flap's outside face with its lower inner edge, which follows
# an arc 40 mm out from the crease while the flap swings closed, then presses the flap flat and lifts away.

TIP = 0.013  # pad tips below the TCP
HALF = 0.010  # closed pads' outer face from the TCP along the closing axis
REACH = 0.040  # the pushing edge's distance out along the flap from its crease
CLEAR = 0.015  # gap to the flap's outside face before the push
PUSH = -0.003  # the pushing edge runs 3 mm into the flap's face line, so it keeps contact
PRESS = -0.004
ARC = (88, 75, 60, 45, 30, 15, 5)
C = FINGER_CLOSED


def _edge_tcp(crease, inward, deg, gap):
    """TCP that puts the pads' lower inner edge ``gap`` off the outside face of a flap open ``deg`` degrees."""
    a = math.radians(deg)
    u = np.array([inward[0], inward[1], 0.0])
    z = np.array([0.0, 0.0, 1.0])
    d, n = math.cos(a) * u + math.sin(a) * z, -math.sin(a) * u + math.cos(a) * z
    edge = np.asarray(crease, float) + REACH * d + (FLAP_T + gap) * n
    return tuple(float(v) for v in edge - HALF * u + TIP * z)


def _fold(crease, inward, yaw, travel_z, label):
    start = _edge_tcp(crease, inward, OPEN_DEG, CLEAR)
    flat = _edge_tcp(crease, inward, 0.0, PRESS)
    return [
        Waypoint(2.0, (start[0], start[1], travel_z), C, f"{label}: over the flap's outside", yaw),
        Waypoint(1.2, start, C, f"{label}: down beside it", yaw),
        Waypoint(0.4, start, C, f"{label}: settle", yaw),
        *(Waypoint(0.6, _edge_tcp(crease, inward, deg, PUSH), C, f"{label}: fold to {deg} deg", yaw) for deg in ARC),
        Waypoint(0.8, flat, C, f"{label}: press flat", yaw),
        Waypoint(0.5, flat, C, f"{label}: hold", yaw),
        Waypoint(1.2, (flat[0], flat[1], travel_z), C, f"{label}: lift clear", yaw),
    ]


def _hold(pos, s, yaw=0.0, label="wait"):
    return [Waypoint(s, pos, C, label, yaw)]


LONG_Y = 0.070  # the long flaps are pushed 7 cm off the carton's centre line, over the short flaps
TRAVEL_SHORT = RIM + 0.078
TRAVEL_LONG = RIM + RAISE + 0.078
PARK = {"left": (0.220, 0.230, 0.950), "right": (0.220, -0.230, 0.950)}


def _plan_s(wps) -> float:
    return sum(w.duration for w in wps)


def demo():
    """Both arms fold the short end flaps at the same time (left arm the left one, right arm the right one), then
    the right arm folds the far long flap and the left arm the near long flap, each over the short flaps. Every
    fold: closed fingers down outside the flap, a slow arc that swings it shut, a press, a lift. 33.9 s."""
    left_short = _fold((CX, CY + HY, RIM), (0, -1), 0.0, TRAVEL_SHORT, "left end flap")
    right_short = _fold((CX, CY - HY, RIM), (0, 1), 0.0, TRAVEL_SHORT, "right end flap")
    far_long = _fold((CX + HX, CY - LONG_Y, RIM + RAISE), (-1, 0), math.pi / 2, TRAVEL_LONG, "far side flap")
    near_long = _fold((CX - HX, CY + LONG_Y, RIM + RAISE), (1, 0), -math.pi / 2, TRAVEL_LONG, "near side flap")
    left = left_short + [Waypoint(1.5, PARK["left"], C, "park", 0.0)]
    right = right_short + far_long + [Waypoint(1.5, PARK["right"], C, "park", math.pi / 2)]
    left += _hold(PARK["left"], _plan_s(right) - _plan_s(left), 0.0, "wait for the far flap") + near_long
    left += [Waypoint(1.5, PARK["left"], C, "park", -math.pi / 2)]
    right += _hold(PARK["right"], _plan_s(left) - _plan_s(right), math.pi / 2, "wait")
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built flap poses (single states and short sequences, one trial each)."""
    import dataclasses

    global _TEST_TCPS
    SHUT_SHORT, SHUT_LONG, LONG_ALONE = -1.0, -2.1, -5.1  # closed angles: on the goods, on the short flaps, on the goods

    def st(**deg):
        return kit.fake_states(q={f"{k}_crease": v for k, v in deg.items()}, deg=True)

    start = st()
    shorts = st(flap_left=SHUT_SHORT, flap_right=SHUT_SHORT)
    one_long = st(flap_left=SHUT_SHORT, flap_right=SHUT_SHORT, flap_far=SHUT_LONG)
    closed = st(flap_left=SHUT_SHORT, flap_right=SHUT_SHORT, flap_far=SHUT_LONG, flap_near=SHUT_LONG)
    longs_first = st(flap_far=LONG_ALONE, flap_near=LONG_ALONE)
    # the short flaps folded last, lying on top of the long ones
    shorts_on_top = st(flap_far=LONG_ALONE, flap_near=LONG_ALONE, flap_left=7.0, flap_right=7.0)
    moving = dict(closed)
    moving[NEAR] = dataclasses.replace(closed[NEAR], ang_vel=np.array([0.0, 0.8, 0.0]))
    long_at_15 = st(flap_left=SHUT_SHORT, flap_right=SHUT_SHORT, flap_far=SHUT_LONG, flap_near=15.0)
    one_short = st(flap_left=SHUT_SHORT)
    near_tcp = kit.point(closed, NEAR, (0.03, 0.0, FLAP_T + 0.01))

    cases = {  # name: (sequence of states in one trial, gripper TCPs, final score, success)
        "start": ([start], [], 0.0, False),
        "one short flap closed": ([start, one_short], [], 0.0, False),
        "both short flaps closed": ([start, shorts], [], 0.25, False),
        "short flaps, then one long flap over them": ([start, shorts, one_long], [], 0.5, False),
        "closed in the right order": ([start, shorts, one_long, closed], [], 1.0, True),
        "closed, but a gripper still on the near flap": ([start, shorts, one_long, closed], [near_tcp], 0.5, False),
        "closed, near flap still swinging": ([start, shorts, one_long, moving], [], 0.5, False),
        "near long flap only 15 degrees from level": ([start, shorts, one_long, long_at_15], [], 0.5, False),
        "naive: long flaps first, short flaps folded on top": ([start, longs_first, shorts_on_top], [], 0.25, False),
        "long flaps first and left there": ([start, longs_first], [], 0.25, False),
        "long flap first, reopened, then the right order": (
            [start, st(flap_near=LONG_ALONE), start, shorts, one_long, closed], [], 0.25, False),
        "end state right but no history (layering check alone)": ([closed], [], 1.0, True),
        "short flaps on top of the long ones, no history": ([shorts_on_top], [], 0.25, False),
    }
    try:
        for name, (seq, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            for s in seq[:-1]:
                score(s)
            got, got_success = score(seq[-1]), success(seq[-1])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        # a new trial forgets the last one's order fault
        mk.new_episode()
        _TEST_TCPS = []
        score(longs_first)
        mk.new_episode()
        assert score(closed) == 1.0, "reset"
        assert abs(_seam(closed) - 0.004) < 0.001, _seam(closed)
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
