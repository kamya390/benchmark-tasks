"""On the station table, a 58 cm slotted wire duct is fixed across the front of both arms, running left to right: a
grey U-channel of two upright side walls on a base, 6 cm wide and 4 cm tall. Its two ends are already closed by
installed cover sections, 6.1 cm long each, lying on the duct walls. Between them the duct is open for 45.8 cm. A loose
45 cm cover lies flat on the table nearer the arms. It is 7 cm wide, 1 cm wider than the duct, so when it is fitted it
overhangs each wall by 5 mm. The cover must be lifted level with both arms and lowered straight into the open run:
both ends down on the walls between the installed sections, with the overhang equal on both sides. Too long for one
gripper to carry level. Scored from the cover's pose at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/wire-duct-cover/wire_duct_cover.py \\
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
DUCT_X = 0.340
DUCT_W = 0.060  # outside width across the walls
DUCT_H = 0.040
DUCT_LEN = 0.580
DUCT_WALL = 0.003
BASE_T = 0.003
WALL_TOP = TOP + DUCT_H
COVER = (0.070, 0.450, 0.012)  # 5 mm overhang each side when centred
COVER_KG = 0.120
COVER_START = (0.210, 0.0)
END_GAP = 0.004  # cut 4 mm short of the open run at each end
GAP_HALF = COVER[1] / 2 + END_GAP  # the open run reaches +-22.9 cm along y
SECTION = (COVER[0], DUCT_LEN / 2 - GAP_HALF, COVER[2])  # each installed end section, 6.1 cm long

GREY = (0.60, 0.62, 0.63)
COVER_GREY = (0.48, 0.50, 0.52)
DARK = (0.30, 0.31, 0.33)

THE_COVER = Box("cover", pos=(*COVER_START, TOP + COVER[2] / 2), size=COVER, mass=COVER_KG, color=COVER_GREY)
FIXTURES = [
    Box("duct_base", pos=(DUCT_X, 0.0, TOP + BASE_T / 2), size=(DUCT_W, DUCT_LEN, BASE_T), color=GREY),
    *(
        Box(f"duct_wall_{side}", pos=(DUCT_X + s * (DUCT_W - DUCT_WALL) / 2, 0.0, TOP + BASE_T + (DUCT_H - BASE_T) / 2),
            size=(DUCT_WALL, DUCT_LEN, DUCT_H - BASE_T), color=GREY)
        for side, s in (("near", -1.0), ("far", 1.0))
    ),
    *(
        Box(f"installed_cover_{side}", pos=(DUCT_X, s * (GAP_HALF + SECTION[1] / 2), WALL_TOP + COVER[2] / 2),
            size=SECTION, color=DARK)
        for side, s in (("left", 1.0), ("right", -1.0))
    ),
]
OBJECTS = [THE_COVER, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "best"  # credit for the furthest stage reached, so a cover that slides late keeps its lift credit

END_Z = 0.0025  # an end is down when its end face's centre is within 2.5 mm of resting height on the walls
BEAR_X = 0.008  # and within 8 mm of the duct's centreline across it (the cover still spans both walls)
CENTRE_X = 0.004  # fitted: overhang equal on both walls within 4 mm
SQUARE_DEG = 2.0


def _axis(quat_xyzw):
    """The cover's long axis (its local y) in the world frame."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)])


def _ends(state):
    a = _axis(state.quat_xyzw)
    c = np.asarray(state.pos, float)
    return c + a * COVER[1] / 2, c - a * COVER[1] / 2


def _end_down(p) -> bool:
    """An end lying on both duct walls inside the open run, not up on an installed section."""
    return bool(
        abs(p[0] - DUCT_X) <= BEAR_X
        and abs(p[1]) <= GAP_HALF + 0.002
        and abs(p[2] - (WALL_TOP + COVER[2] / 2)) <= END_Z
    )


def _fitted(state) -> bool:
    a = _axis(state.quat_xyzw)
    off = math.degrees(math.acos(min(1.0, abs(float(a[1])))))
    return bool(
        all(_end_down(p) for p in _ends(state))
        and abs(float(state.pos[0]) - DUCT_X) <= CENTRE_X
        and off <= SQUARE_DEG
        and state.resting()
    )


def success(objects) -> bool:
    """The cover lies on both duct walls along its whole length between the installed sections, the overhang equal
    within 4 mm, square, still; holds for 1 s."""
    return _fitted(objects["cover"])


def score(objects) -> float:
    """0.25 once the whole cover is 3 cm off the table; 0.5 with one end down on the duct walls in the open run, at
    rest; 0.75 with both ends; 1.0 fitted (both ends down, centre within 4 mm of the duct centreline so it overhangs
    both walls equally, square within 2 degrees, still). The best stage reached counts."""
    st = objects["cover"]
    if _fitted(st):
        return 1.0
    n = sum(_end_down(p) for p in _ends(st))
    if n == 2 and st.resting():
        return 0.75
    if n >= 1 and st.resting():
        return 0.5
    if float(st.low[2]) >= TOP + 0.030:
        return 0.25
    return 0.0


def distance(objects) -> float:
    st = objects["cover"]
    return float(math.dist((float(st.pos[0]), float(st.pos[1]), float(st.low[2])), (DUCT_X, 0.0, WALL_TOP)))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, across the cover's 70 mm width
TIP = 0.013
TRAVEL = TOP + 0.110
SQUEEZE = 0.008
GRIP_Y = 0.190  # each arm holds the cover 19 cm from its centre, 1.6 cm in from its ends
GRIP_UP = 0.003  # pad tips 3 mm above the cover's bottom face
X_TRIM = 0.0025  # net of the cover trailing ~5 mm in the carry and moving ~2.5 mm outward on release (measured in sim)


def demo():
    """Both arms at once, mirrored and in step so the cover stays level: grip it 19 cm either side of its centre,
    lift together to 11 cm, carry it out over the duct, lower it straight into the open run, let go together. 18.2 s."""
    gz = TOP + GRIP_UP + TIP
    qz = WALL_TOP + 0.0015 + GRIP_UP + TIP  # the cover let go 1.5 mm above the wall tops
    grip = -(COVER[0] - SQUEEZE) / 2
    plans = {}
    for side, sy in (("left", 1.0), ("right", -1.0)):
        px, py = COVER_START[0], sy * GRIP_Y
        qx, qy = DUCT_X + X_TRIM, sy * GRIP_Y
        plans[side] = [
            Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, "over the cover", YAW),
            Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, "descend", YAW),
            Waypoint(1.0, (px, py, gz), FINGER_OPEN, "down slowly", YAW),
            Waypoint(0.5, (px, py, gz), FINGER_OPEN, "settle", YAW),
            Waypoint(1.5, (px, py, gz), grip, "close", YAW),
            Waypoint(2.0, (px, py, TRAVEL), grip, "lift together", YAW),
            Waypoint(2.5, (qx, qy, TRAVEL), grip, "carry out", YAW),
            Waypoint(0.8, (qx, qy, TRAVEL), grip, "settle over the duct", YAW),
            Waypoint(2.0, (qx, qy, qz + 0.020), grip, "lower", YAW),
            Waypoint(1.5, (qx, qy, qz), grip, "onto the walls slowly", YAW),
            Waypoint(0.6, (qx, qy, qz), grip, "settle", YAW),
            Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "let go together", YAW),
            Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "retreat", YAW),
        ]
    return plans


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def cover(x, y, bottom, yaw_deg=0.0, roll_deg=0.0, vel=(0.0, 0.0, 0.0)):
        """Yawed about z, then rolled about x (one end up); ``bottom`` is the lowest point."""
        yaw, roll = math.radians(yaw_deg), math.radians(roll_deg)
        cy, sy, cr, sr = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(roll / 2), math.sin(roll / 2)
        q = np.array([cy * sr, sy * sr, sy * cr, cy * cr])
        half_z = COVER[2] / 2 * math.cos(roll) + COVER[1] / 2 * abs(math.sin(roll))
        pos = np.array([x, y, bottom + half_z])
        half = np.array([COVER[0] / 2, COVER[1] / 2, half_z])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def one_end_up(shift):
        """Shifted along the duct so its left end rides up on the installed section, its right end on the walls."""
        roll = math.degrees(math.asin(COVER[2] / COVER[1]))
        return cover(DUCT_X, shift, WALL_TOP, roll_deg=roll)

    cases = {
        "start": ({"cover": cover(*COVER_START, TOP)}, 0.0, False),
        "lifted 4 cm": ({"cover": cover(0.28, 0.0, TOP + 0.04, vel=(0.05, 0, 0))}, 0.25, False),
        "lifted 2 cm": ({"cover": cover(0.28, 0.0, TOP + 0.02, vel=(0.05, 0, 0))}, 0.0, False),
        "fitted": ({"cover": cover(DUCT_X, 0.0, WALL_TOP)}, 1.0, True),
        "fitted, 3 mm along": ({"cover": cover(DUCT_X, 0.003, WALL_TOP)}, 1.0, True),
        "fitted, 3 mm across": ({"cover": cover(DUCT_X - 0.003, 0.0, WALL_TOP)}, 1.0, True),
        "on the walls, 6 mm across (overhang unequal)": ({"cover": cover(DUCT_X - 0.006, 0.0, WALL_TOP)}, 0.75, False),
        "fitted, turned 1.5 degrees": ({"cover": cover(DUCT_X, 0.0, WALL_TOP, yaw_deg=1.5)}, 1.0, True),
        "across the walls at 3 degrees (ends off the walls)": ({"cover": cover(DUCT_X, 0.0, WALL_TOP, yaw_deg=3)}, 0.25, False),
        "one end up on the installed section": ({"cover": one_end_up(0.010)}, 0.5, False),
        "fitted but still sliding": ({"cover": cover(DUCT_X, 0.0, WALL_TOP, vel=(0.0, 0.03, 0.0))}, 0.25, False),
        "hung on the near wall's edge": ({"cover": cover(DUCT_X - 0.030, 0.0, WALL_TOP)}, 0.25, False),
        "dropped on the table in front of the duct": ({"cover": cover(DUCT_X - 0.08, 0.0, TOP)}, 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
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
