"""On the station table, two workcells face each other with no shared buffer between them. In the right arm's cell,
far out on the right (y -52 cm), a 22 cm aluminium square tube (the bar) lies flat along x; in the left arm's cell, far out on the
left (y +52 cm), the next operation's fixture waits: two fixed steel posts with orange cradle walls, 17 cm apart along
x, each leaving a channel 8 mm wider than the bar. Each cell lies outside the other arm's reach (the left arm cannot
reach y below about -0.45 m, the right arm cannot reach y above about +0.46 m at table height), and the bar may not be
set down between the cells. So the right arm lifts the bar and brings it to the middle, the left arm takes the other
end of it in the air, the right arm lets go, and the left arm lays the bar in both cradles. Scored from the bar's pose.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/reach-limited-handoff/reach_limited_handoff.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
BAR = (0.220, 0.020, 0.020)  # 22 cm aluminium square tube, 20 x 20 mm, 2 mm wall, lying along x
BAR_KG = 0.090
BAR_START = (0.310, -0.520)  # in the right arm's cell
FIX_X, FIX_Y = 0.310, 0.520  # the fixture's centre, in the left arm's cell
CRADLE_DX = 0.085  # each cradle post's centre, either side of FIX_X along the bar
POST = (0.025, 0.030, 0.040)  # cradle post: 2.5 cm along the bar, 3 cm across, 4 cm tall
POST_TOP = TOP + POST[2]
CHANNEL = BAR[1] + 0.008  # 28 mm between the cradle walls
WALL = (0.025, 0.005, 0.008)  # cradle wall: along x, 8 mm above the post top
GAP_Y = 0.350  # between the cells (|y| < 35 cm) the bar may not touch the table

STEEL = (0.75, 0.76, 0.78)
DARK = (0.20, 0.20, 0.22)
ORANGE = (0.92, 0.48, 0.10)
ALU = (0.82, 0.84, 0.86)

THE_BAR = Box("bar", pos=(*BAR_START, TOP + BAR[2] / 2), size=BAR, mass=BAR_KG, color=ALU)
FIXTURES = []
for _end, _sx in (("near", -1.0), ("far", 1.0)):
    _x = FIX_X + _sx * CRADLE_DX
    FIXTURES += [
        Box(f"{_end}_post", pos=(_x, FIX_Y, TOP + POST[2] / 2), size=POST, color=DARK),
        Box(f"{_end}_cradle_left", pos=(_x, FIX_Y + CHANNEL / 2 + WALL[1] / 2, POST_TOP + WALL[2] / 2), size=WALL,
            color=ORANGE),
        Box(f"{_end}_cradle_right", pos=(_x, FIX_Y - CHANNEL / 2 - WALL[1] / 2, POST_TOP + WALL[2] / 2), size=WALL,
            color=ORANGE),
    ]
OBJECTS = [THE_BAR, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "best"  # credit for the furthest stage reached, so lift and transfer credit survive a late slip

SEAT_X = 0.010  # seated: centre within 1 cm of the fixture's centre along the bar
SEAT_Z = 0.002  # both ends' bottoms within 2 mm of the post tops
SEAT_DEG = 3.0  # long side along x, and level


def _axis(quat_xyzw):
    """The bar's long axis (its local x) in the world frame."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])


def _ends(state):
    """World positions of the centres of the bar's two end faces."""
    a = _axis(state.quat_xyzw)
    c = np.asarray(state.pos, float)
    return c + a * BAR[0] / 2, c - a * BAR[0] / 2


def _end_in_cradle(p, tol=0.003) -> bool:
    """An end over a cradle post (within 4 cm of its centre along the bar, inside the channel across it), at post
    height: the end face's centre within ``tol`` of half a bar thickness above the post top."""
    return any(
        abs(p[0] - (FIX_X + sx * CRADLE_DX)) <= 0.040
        and abs(p[1] - FIX_Y) <= CHANNEL / 2
        and abs(p[2] - (POST_TOP + BAR[2] / 2)) <= tol
        for sx in (1.0, -1.0)
    )


def _seated(state) -> bool:
    a = _axis(state.quat_xyzw)
    off = math.degrees(math.acos(min(1.0, abs(float(a[0])))))
    return bool(
        abs(float(state.pos[0]) - FIX_X) <= SEAT_X
        and abs(float(state.pos[1]) - FIX_Y) <= CHANNEL / 2
        and all(_end_in_cradle(p, SEAT_Z) for p in _ends(state))
        and off <= SEAT_DEG
        and state.resting()
    )


def _lifted(state) -> bool:
    return float(state.low[2]) >= TOP + 0.030


def _in_fixture(state) -> bool:
    return all(_end_in_cradle(p) for p in _ends(state)) and state.resting()


# The trial's history: whether the bar has touched the table between the cells (there is no shared buffer). The task
# file only sees object states, so the history resets whenever the bar is back on its start mark (each new trial).
_HISTORY = {"buffered": False}


def _at_start(st) -> bool:
    return bool(math.hypot(float(st.pos[0]) - BAR_START[0], float(st.pos[1]) - BAR_START[1]) < 0.005
                and float(st.low[2]) < TOP + 0.003)


def _update(st) -> bool:
    if _at_start(st):
        _HISTORY["buffered"] = False
    elif abs(float(st.pos[1])) < GAP_Y and float(st.low[2]) < TOP + 0.005:
        _HISTORY["buffered"] = True
    return _HISTORY["buffered"]


def success(objects) -> bool:
    """The bar lies in both cradles, centred and square, still, and never touched the table between the cells;
    holds for 1 s."""
    st = objects["bar"]
    return not _update(st) and _seated(st)


def score(objects) -> float:
    """0.25 once the whole bar is 3 cm off the table; 0.5 while it is off the table with its centre past y +15 cm,
    across the midline into the left cell's side; 0.75 resting with both ends in the cradles (bottoms within 3 mm of
    the post tops); 1.0 seated (centre within 1 cm of the fixture centre along the bar, bottoms within 2 mm, square and
    level within 3 degrees, still). If the bar has touched the table between the cells (|y| < 35 cm), the score from
    then on is at most 0.25. The best stage reached counts."""
    st = objects["bar"]
    buffered = _update(st)
    if _seated(st):
        s = 1.0
    elif _in_fixture(st):
        s = 0.75
    elif _lifted(st) and float(st.pos[1]) > 0.150:
        s = 0.5
    elif _lifted(st):
        s = 0.25
    else:
        s = 0.0
    return min(s, 0.25) if buffered else s


def buffered() -> bool:
    """Whether this trial has set the bar down between the cells (for the log)."""
    return _HISTORY["buffered"]


def distance(objects) -> float:
    st = objects["bar"]
    return float(math.dist((float(st.pos[0]), float(st.pos[1]), float(st.low[2])), (FIX_X, FIX_Y, POST_TOP)))


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
SQUEEZE = 0.010
GRIP_D = 0.075  # each arm holds the bar 7.5 cm either side of its centre: the gripper bodies 15 cm apart
TRAVEL = TOP + 0.130  # carry height at the cells
HAND_Z = TOP + 0.120  # handoff height in the middle (TCP)
HAND_X = 0.340
OPEN = -(BAR[1] + 0.024) / 2
GRIP = -(BAR[1] - SQUEEZE) / 2
X_TRIM = 0.005  # a carried part trails the commanded x by about 5 mm on the way down (measured in sim)
YAW_X = math.pi / 2  # the fingers close along x, across a bar lying along y


def demo():
    """Both arms. Right: grip the bar 7.5 cm from its centre on the near side (fingers across its 20 mm width), lift to
    13 cm, carry it to the middle while turning it a quarter turn so it lies along y with its free end towards the left
    arm, and hold it there. Left: wait above, come down over the free end 7.5 cm the other side of the centre with the
    fingers wide open, close, and hold while the right arm opens and backs away. Then the left arm carries the bar out
    to its cell, turning it back along x, and lays it in both cradles. 30.4 s."""
    gz = TOP + BAR[2] - 0.012 + TIP  # pad tips 8 mm above the bar's bottom
    bar_bottom_held = HAND_Z - (BAR[2] - 0.012 + TIP)  # where the bar's bottom hangs at the handoff
    lz = bar_bottom_held - 0.015 + TIP  # the left pad tips 15 mm below it: the free end droops and sags in the grip
    qz = POST_TOP + 0.002 + BAR[2] - 0.012 + TIP  # the bar let go 2 mm above the post tops
    px, py = BAR_START[0] - GRIP_D, BAR_START[1]
    rx, ry = HAND_X, -GRIP_D  # the right TCP at the handoff: the bar's centre on the midline
    lx, ly = HAND_X, GRIP_D
    fx, fy = FIX_X - GRIP_D + X_TRIM, FIX_Y

    right = [
        Waypoint(1.8, (px, py, TRAVEL), OPEN, "over the bar", 0.0),
        Waypoint(1.5, (px, py, gz + 0.020), OPEN, "descend", 0.0),
        Waypoint(1.0, (px, py, gz), OPEN, "down slowly", 0.0),
        Waypoint(0.5, (px, py, gz), OPEN, "settle", 0.0),
        Waypoint(1.5, (px, py, gz), GRIP, "close", 0.0),
        Waypoint(1.5, (px, py, TRAVEL), GRIP, "lift", 0.0),
        Waypoint(3.0, (rx, ry, HAND_Z), GRIP, "carry to the middle, turning", YAW_X),
        Waypoint(1.0, (rx, ry, HAND_Z), GRIP, "settle at the handoff", YAW_X),
    ]
    t_hand = sum(w.duration for w in right)  # 11.8 s
    right += [
        Waypoint(4.5, (rx, ry, HAND_Z), GRIP, "hold while the left arm takes it", YAW_X),
        Waypoint(1.5, (rx, ry, HAND_Z), OPEN, "let go", YAW_X),
        Waypoint(1.0, (rx, ry - 0.030, HAND_Z + 0.050), OPEN, "back off", YAW_X),
        Waypoint(1.5, (0.250, -0.300, TOP + 0.200), OPEN, "clear", 0.0),
    ]

    wait = [Waypoint(3.0, (lx, ly + 0.080, HAND_Z + 0.035), FINGER_OPEN, "to the waiting point", -YAW_X)]
    left = wait + [
        Waypoint(t_hand - 3.0, (lx, ly + 0.080, HAND_Z + 0.035), FINGER_OPEN, "wait", -YAW_X),
        Waypoint(1.2, (lx, ly, HAND_Z + 0.035), FINGER_OPEN, "over the free end", -YAW_X),
        Waypoint(1.5, (lx, ly, lz), FINGER_OPEN, "down round the bar", -YAW_X),
        Waypoint(0.5, (lx, ly, lz), FINGER_OPEN, "settle", -YAW_X),
        Waypoint(1.5, (lx, ly, lz), GRIP, "close", -YAW_X),
        Waypoint(3.0, (lx, ly, lz), GRIP, "hold while the right arm lets go", -YAW_X),
        Waypoint(1.0, (lx, ly, lz + 0.010), GRIP, "take the weight", -YAW_X),
        Waypoint(3.0, (fx, fy, TRAVEL), GRIP, "carry to the fixture, turning", 0.0),
        Waypoint(0.8, (fx, fy, TRAVEL), GRIP, "settle over the cradles", 0.0),
        Waypoint(1.5, (fx, fy, qz + 0.020), GRIP, "lower", 0.0),
        Waypoint(1.5, (fx, fy, qz), GRIP, "into the cradles slowly", 0.0),
        Waypoint(0.6, (fx, fy, qz), GRIP, "settle", 0.0),
        Waypoint(1.5, (fx, fy, qz), OPEN, "let go", 0.0),
        Waypoint(1.0, (fx, fy, TRAVEL), OPEN, "retreat", 0.0),
    ]
    return {"right": right, "left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the no-buffer history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def bar(x, y, bottom, yaw_deg=0.0, pitch_deg=0.0, vel=(0.0, 0.0, 0.0)):
        """Yawed about z, then pitched about y (one end up)."""
        yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
        cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
        q = np.array([-sy * sp, cy * sp, sy * cp, cy * cp])
        half_z = BAR[2] / 2 * math.cos(pitch) + BAR[0] / 2 * abs(math.sin(pitch))
        hx = BAR[0] / 2 * abs(math.cos(math.radians(yaw_deg))) + BAR[1] / 2 * abs(math.sin(math.radians(yaw_deg)))
        hy = BAR[0] / 2 * abs(math.sin(math.radians(yaw_deg))) + BAR[1] / 2 * abs(math.cos(math.radians(yaw_deg)))
        pos = np.array([x, y, bottom + half_z])
        half = np.array([hx, hy, half_z])
        return {"bar": ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)}

    start = bar(*BAR_START, TOP)
    lifted = bar(0.30, -0.40, TOP + 0.10, vel=(0, 0.05, 0))
    handoff = bar(HAND_X, 0.0, TOP + 0.10, yaw_deg=90)
    crossed = bar(0.30, 0.30, TOP + 0.10, vel=(0, 0.05, 0))
    seated = bar(FIX_X, FIX_Y, POST_TOP)
    tilt = math.degrees(math.asin(0.010 / BAR[0]))

    def run(*frames):
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "lifted in the right cell": ((start, lifted), 0.25, False),
        "lifted 2 cm only": ((start, bar(0.30, -0.50, TOP + 0.02, vel=(0, 0, 0.05))), 0.0, False),
        "held at the handoff on the midline": ((start, lifted, handoff), 0.25, False),
        "carried past y +15 cm": ((start, lifted, handoff, crossed), 0.5, False),
        "seated": ((start, lifted, handoff, crossed, seated), 1.0, True),
        "seated, 8 mm along": ((start, bar(FIX_X + 0.008, FIX_Y, POST_TOP)), 1.0, True),
        "both ends in, 1.5 cm along": ((start, bar(FIX_X + 0.015, FIX_Y, POST_TOP)), 0.75, False),
        "both ends in, turned 4 degrees": ((start, bar(FIX_X, FIX_Y + 0.002, POST_TOP, yaw_deg=4)), 0.75, False),
        "one end up on a cradle wall": ((start, bar(FIX_X, FIX_Y, POST_TOP, pitch_deg=tilt)), 0.5, False),
        "set down on the table between the cells": ((start, lifted, bar(0.30, 0.0, TOP)), 0.0, False),
        "buffered on the table, then seated": ((start, lifted, bar(0.30, 0.0, TOP), crossed, seated), 0.25, False),
        "a new trial after a buffered one starts clean": ((start, bar(0.30, 0.0, TOP), start, lifted, crossed, seated),
                                                          1.0, True),
        "dropped back in the right cell": ((start, lifted, bar(0.30, -0.45, TOP)), 0.0, False),
        "seated but still sliding": ((start, bar(FIX_X, FIX_Y, POST_TOP, vel=(0.03, 0, 0))), 0.5, False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)  # leave the history clean
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
