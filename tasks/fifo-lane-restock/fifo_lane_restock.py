"""On the station table, in front of the left arm, a line-side flow lane: two low guide rails along x and a front stop
nearest the arms, room for four cartons. Two orange cartons from the older lot stand in the lane's second and third
places, leaving the front place empty. Two blue cartons from the new lot wait on the table beside the lane. First in,
first out: the older lot must be used first, so the lane must end older, older, newer, newer from the front stop.
Putting a new carton in the empty front place, the quick move, breaks the rule. Scored from the cartons' poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/fifo-lane-restock/fifo_lane_restock.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

# Shared fix: per-episode memory that a new trial clears (instead of guessing a reset from the layout), and the two
# grippers' finger tips, so "released" can be checked. Same mechanism as the accepted reseat-half-inserted-plug task
# (PR #20): wrap YamArenaEmbodiment.reset and ._task_step. Paste into the task file below the imports.
_EP = {"env": None}
_MEM: dict = {}  # per-episode history; the task adds its own keys


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    reset0, step0 = cls.reset, cls._task_step

    def reset(self, *a, **k):
        _MEM.clear()
        return reset0(self, *a, **k)

    def _task_step(self, *a, **k):
        _EP["env"] = self._env
        return step0(self, *a, **k)

    cls.reset, cls._task_step = reset, _task_step


_install_hooks()


def _tcps():
    """World positions of both grippers' TCP (13 mm above the finger pad tips), as in PR #20; [] offline."""
    env = _EP["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(float)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(float) / 2
        out.append(g + (mid - g) / (np.linalg.norm(mid - g) + 1e-12) * 0.134)
    return out


def released(state, margin=0.025) -> bool:
    """Neither gripper's TCP within ``margin`` (2.5 cm) of the part's bounding box: the part is let go."""
    lo, hi = np.asarray(state.low) - margin, np.asarray(state.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
CARTON = (0.060, 0.075, 0.060)  # 6 cm deep along the lane (x), 7.5 cm wide (y), 6 cm tall
CARTON_KG = 0.150
LANE_Y = 0.330
STOP_X = 0.150  # the front stop's rear face
PITCH = 0.075  # place centres 7.5 cm apart: 15 mm between cartons
PLACE_X = [STOP_X + 0.008 + CARTON[0] / 2 + k * PITCH for k in range(4)]  # 1 (front) to 4
RAIL = (0.300, 0.006, 0.010)
RAIL_X = STOP_X + RAIL[0] / 2
RAIL_DY = CARTON[1] / 2 + 0.005 + RAIL[1] / 2
STOP = (0.006, CARTON[1] + 0.022, 0.030)

OLD = "old"
NEW = "new"
COLORS = {OLD: (0.92, 0.48, 0.10), NEW: (0.15, 0.32, 0.80)}
STEEL = (0.75, 0.76, 0.78)

START = {"old_1": (PLACE_X[1], LANE_Y), "old_2": (PLACE_X[2], LANE_Y),
         "new_1": (0.230, 0.200), "new_2": (0.320, 0.200)}
LOT_OF = {name: name.split("_")[0] for name in START}
CARTONS = [Box(n, pos=(x, y, TOP + CARTON[2] / 2), size=CARTON, mass=CARTON_KG, color=COLORS[LOT_OF[n]])
           for n, (x, y) in START.items()]
FIXTURES = [
    Box("rail_left", pos=(RAIL_X, LANE_Y + RAIL_DY, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("rail_right", pos=(RAIL_X, LANE_Y - RAIL_DY, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("front_stop", pos=(STOP_X - STOP[0] / 2, LANE_Y, TOP + STOP[2] / 2), size=STOP, color=STEEL),
]
OBJECTS = [*CARTONS, *FIXTURES]
WANT = (OLD, OLD, NEW, NEW)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
PLACE_TOL = 0.015  # a carton fills a place with its centre within 1.5 cm of the place centre along the lane


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _in_lane(state) -> bool:
    """Standing upright on the table between the rails, still, and let go (no gripper's tool point within 2.5 cm)."""
    _, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(
        abs(float(state.pos[1]) - LANE_Y) <= 0.010
        and STOP_X < float(state.pos[0]) < STOP_X + RAIL[0]
        and abs(float(state.low[2]) - TOP) <= 0.003
        and tilt <= 8.0
        and state.resting()
        and released(state)
    )


def lane(objects):
    """The lot in each of the four places, front to back (None where no carton stands)."""
    out = [None] * 4
    for c in CARTONS:
        st = objects[c.name]
        if not _in_lane(st):
            continue
        for k, x in enumerate(PLACE_X):
            if abs(float(st.pos[0]) - x) <= PLACE_TOL:
                out[k] = LOT_OF[c.name]
    return out


def _right(objects) -> int:
    """Places holding the right lot, counted from the front stop; a new carton in front of an old one, or an empty
    place, ends the count, because the lane then hands out the wrong lot first."""
    n = 0
    for got, want in zip(lane(objects), WANT):
        if got != want:
            break
        n += 1
    return n


def success(objects) -> bool:
    """Older, older, newer, newer from the front stop, all four standing in the lane; holds for 1 s."""
    return _right(objects) == 4


def score(objects) -> float:
    """1/4 for each place holding the right lot, counted from the front stop and stopping at the first wrong or empty
    place: older lot in places 1 and 2, newer lot in 3 and 4. The start (place 1 empty) scores 0, and so does a new
    carton put in the front place."""
    return _right(objects) / 4.0


def distance(objects) -> float:
    return float(4 - _right(objects))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = 0.0  # the fingers close along y, across the carton's 75 mm width, from above the low rails
TIP = 0.013
TRAVEL = TOP + 0.140
SQUEEZE = 0.008
DEPTH = 0.042  # first sim run: gripped 25 mm down, the cartons swung about the finger axis and landed tipped
OPEN = -(CARTON[1] + 0.014) / 2  # 89 mm: the pads clear the carton and stay inside the 94 mm stroke


def _move(pick, place, label):
    (px, py), (qx, qy) = pick, place
    gz = TOP + CARTON[2] - DEPTH + TIP
    qz = gz + 0.002
    grip = -(CARTON[1] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), OPEN, f"{label}: over", YAW),
        Waypoint(1.5, (px, py, gz + 0.020), OPEN, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), OPEN, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), OPEN, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(2.0, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(3.0, (qx, qy, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(2.0, (qx, qy, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set down slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), OPEN, f"{label}: release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), OPEN, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: the front old carton forward from place 2 to place 1, the other from place 3 to place 2, then the
    two new cartons into places 3 and 4. 75.6 s."""
    moves = _move(START["old_1"], (PLACE_X[0], LANE_Y), "old carton to place 1")
    moves += _move(START["old_2"], (PLACE_X[1], LANE_Y), "old carton to place 2")
    moves += _move(START["new_1"], (PLACE_X[2], LANE_Y), "new carton to place 3")
    moves += _move(START["new_2"], (PLACE_X[3], LANE_Y), "new carton to place 4")
    return {"left": moves}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(x, y, bottom=TOP, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        q = np.array([cy * st, sy * st, sy * ct, cy * ct])
        half = np.array(CARTON) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {n: box(*p) for n, p in START.items()}

    def at(**places):
        """Cartons by name -> place number (1 to 4); the rest stay where they start."""
        return start | {n: box(PLACE_X[k - 1], LANE_Y) for n, k in places.items()}

    cases = {
        "start (front place empty)": (start, 0.0, False),
        "done": (at(old_1=1, old_2=2, new_1=3, new_2=4), 1.0, True),
        "done, old cartons the other way round": (at(old_1=2, old_2=1, new_1=4, new_2=3), 1.0, True),
        "quick move: a new carton in the empty front place": (at(new_1=1, new_2=4), 0.0, False),
        "old cartons forward only": (at(old_1=1, old_2=2), 0.5, False),
        "old forward, one new behind": (at(old_1=1, old_2=2, new_1=3), 0.75, False),
        "one old forward, the other left in place 3": (at(old_1=1), 0.25, False),
        "old in 1 and 3, new in 2 and 4": (at(old_1=1, old_2=3, new_1=2, new_2=4), 0.25, False),
        "done but place 2 carton fallen over": (
            at(old_1=1, new_1=3, new_2=4) | {"old_2": box(PLACE_X[1], LANE_Y, tilt_deg=90)}, 0.25, False),
        "done but place 4 carton 2 cm out of place": (
            at(old_1=1, old_2=2, new_1=3) | {"new_2": box(PLACE_X[3] + 0.020, LANE_Y)}, 0.75, False),
        "done but place 3 carton outside the rails": (
            at(old_1=1, old_2=2, new_2=4) | {"new_1": box(PLACE_X[2], LANE_Y + 0.08)}, 0.5, False),
        "done but place 1 carton still moving": (
            at(old_2=2, new_1=3, new_2=4) | {"old_1": box(PLACE_X[0], LANE_Y, vel=(0.0, 0.0, -0.05))}, 0.0, False),
        "done, carton turned 90 degrees in its place": (
            at(old_1=1, old_2=2, new_1=3) | {"new_2": box(PLACE_X[3], LANE_Y, yaw_deg=90)}, 1.0, True),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success, lane(st))
    # A gripper's tool point still on the place-4 carton: that carton is not let go, so place 4 earns nothing.
    global _tcps
    real, done = _tcps, at(old_1=1, old_2=2, new_1=3, new_2=4)
    _tcps = lambda: [np.array([PLACE_X[3], LANE_Y, TOP + CARTON[2] - 0.012])]  # noqa: E731
    try:
        assert score(done) == 0.75 and not success(done), "held carton"
    finally:
        _tcps = real
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
