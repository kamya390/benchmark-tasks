"""On the station table, in front of the left arm, stands the chuck of a small lathe, shown face up as a short fixed
steel cylinder. The chuck key, a steel bar, still stands upright in the key socket on its face (a small black square
collar). Further left and nearer the arms is the key holder, a black open pocket on the table. To the right is the
spindle start lever: a red slider in a steel guide slot, sitting at its OFF end (a red plate beside the slot) with a
green RUN stop at the far end of the slot (a green plate beside it). Machine-shop rule: never start a lathe with the
chuck key in the chuck, because the spinning chuck throws it. The key comes out and goes into its holder first; only
then is the start lever pushed forward to RUN. The order is checked: if the lever leaves OFF while the key is not in
its holder, starting the spindle earns nothing.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/chuck-key-before-start/chuck_key_before_start.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

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
CHUCK_XY = (0.340, 0.370)
CHUCK_R = 0.050
CHUCK_H = 0.045
CHUCK_TOP = TOP + CHUCK_H
SOCKET_INNER = (0.020, 0.020)
SOCKET_WALL = (0.005, 0.012)

KEY = (0.015, 0.015, 0.065)
KEY_KG = 0.090

HOLDER_XY = (0.200, 0.430)
HOLDER_INNER = (0.024, 0.024)
HOLDER_WALL = (0.006, 0.025)

# The start lever: a slider in a guide slot along x, pushed forward from OFF to the RUN stop.
SLOT_Y = 0.220
OFF_X = 0.250
RUN_X = 0.295
LEVER = (0.050, 0.026, 0.020)
LEVER_KG = 0.050
RAIL_GAP = 0.002
RAIL = (0.110, 0.006, 0.006)
RAIL_X = (OFF_X + RUN_X) / 2 - 0.005
RAIL_YS = (SLOT_Y + LEVER[1] / 2 + RAIL_GAP + RAIL[1] / 2, SLOT_Y - LEVER[1] / 2 - RAIL_GAP - RAIL[1] / 2)
STOP = (0.008, LEVER[1] + 2 * RAIL_GAP + 2 * RAIL[1], 0.015)
STOP_X = RUN_X + LEVER[0] / 2 + STOP[0] / 2
MARK = (0.030, 0.020, 0.003)
MARK_Y = SLOT_Y - 0.045

STEEL = (0.72, 0.73, 0.75)
DARK_STEEL = (0.45, 0.46, 0.48)
BLACK = (0.08, 0.08, 0.08)
RED = (0.80, 0.12, 0.10)
GREEN = (0.15, 0.65, 0.25)


def _frame(name, centre, inner, wall, base_z, color):
    cx, cy = centre
    ix, iy = inner
    t, h = wall
    z = base_z + h / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=color),
    ]


CHUCK_KEY = Box("chuck_key", pos=(*CHUCK_XY, CHUCK_TOP + KEY[2] / 2), size=KEY, mass=KEY_KG, color=DARK_STEEL)
START_LEVER = Box("start_lever", pos=(OFF_X, SLOT_Y, TOP + LEVER[2] / 2), size=LEVER, mass=LEVER_KG, color=RED)
FIXTURES = [
    Cylinder("chuck", pos=(*CHUCK_XY, TOP + CHUCK_H / 2), radius=CHUCK_R, height=CHUCK_H, color=STEEL),
    *_frame("key_socket", CHUCK_XY, SOCKET_INNER, SOCKET_WALL, CHUCK_TOP, BLACK),
    *_frame("key_holder", HOLDER_XY, HOLDER_INNER, HOLDER_WALL, TOP, BLACK),
    *(Box(f"slot_rail_{k + 1}", pos=(RAIL_X, y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL) for k, y in enumerate(RAIL_YS)),
    Box("run_stop", pos=(STOP_X, SLOT_Y, TOP + STOP[2] / 2), size=STOP, color=GREEN),
    Box("off_mark", pos=(OFF_X, MARK_Y, TOP + MARK[2] / 2), size=MARK, color=RED),
    Box("run_mark", pos=(RUN_X, MARK_Y, TOP + MARK[2] / 2), size=MARK, color=GREEN),
]
OBJECTS = [CHUCK_KEY, START_LEVER, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _in_holder(st) -> bool:
    """The key standing in its holder: centre inside the pocket, bottom on the table, top still high (upright)."""
    return bool(
        abs(float(st.pos[0]) - HOLDER_XY[0]) < HOLDER_INNER[0] / 2
        and abs(float(st.pos[1]) - HOLDER_XY[1]) < HOLDER_INNER[1] / 2
        and TOP - 0.003 < float(st.low[2]) < TOP + 0.008
        and float(st.high[2]) > TOP + KEY[2] - 0.010
    )


def _stowed(st) -> bool:
    """The key standing at rest in its holder and let go."""
    return _in_holder(st) and st.resting() and released(st)


def _at_run(st) -> bool:
    """The lever at rest against the RUN stop (within 6 mm) and still in its slot."""
    return bool(
        abs(float(st.pos[0]) - RUN_X) < 0.006
        and abs(float(st.pos[1]) - SLOT_Y) < 0.006
        and float(st.low[2]) < TOP + 0.004
        and st.resting()
        and released(st)
    )


def _off(st) -> bool:
    """The lever has not left OFF: centre less than 1 cm forward of the OFF position."""
    return float(st.pos[0]) < OFF_X + 0.010


# The trial's history (in _MEM, cleared only when a new trial resets): whether the lever ever left OFF while the key
# was not standing in its holder. Latched for the trial.


def _update(objects) -> bool:
    if not _off(objects["start_lever"]) and not _in_holder(objects["chuck_key"]):
        _MEM["violated"] = True
    return bool(_MEM.get("violated", False))


def success(objects) -> bool:
    """Key in its holder and the lever at RUN, having left OFF only after the key was stowed; holds for 1 s."""
    violated = _update(objects)
    return not violated and _stowed(objects["chuck_key"]) and _at_run(objects["start_lever"])


def score(objects) -> float:
    """0.5 for the key standing in its holder; 0.5 for the lever at RUN, but only if it never left OFF while the key was
    not in the holder. Starting with the key in the chuck (or lying anywhere else) forfeits the 0.5."""
    violated = _update(objects)
    s = 0.5 * _stowed(objects["chuck_key"])
    if not violated and _at_run(objects["start_lever"]):
        s += 0.5
    return s


def violated() -> bool:
    return bool(_MEM.get("violated", False))


def distance(objects) -> float:
    k, lv = objects["chuck_key"], objects["start_lever"]
    return float(math.hypot(float(k.pos[0]) - HOLDER_XY[0], float(k.pos[1]) - HOLDER_XY[1])
                 + abs(float(lv.pos[0]) - RUN_X))


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
X_TRIM = 0.002  # a light carried part lands ~1.5-3 mm short in x
PUSH_TIP_Z = TOP + 0.010  # pad tips 10 mm above the table: above the 6 mm rails, against the lever's upper half
CLOSED_HALF = 0.010  # closed fingers: each pad 10 mm thick, so the pushing face is nominally 10 mm ahead of the TCP
PUSH_OVER = 0.009  # first sim run: the lever stopped 7 mm short of the stop with no overtravel (arm lag while pushing)


def _key_move():
    (px, py), (qx, qy) = CHUCK_XY, (HOLDER_XY[0] + X_TRIM, HOLDER_XY[1])
    depth = 0.020
    gz = CHUCK_TOP + KEY[2] - depth + TIP
    qz = TOP + KEY[2] - depth + TIP + 0.002
    open_ = -(KEY[1] + 0.024) / 2
    grip = -(KEY[1] - SQUEEZE) / 2
    lbl = "key"
    return [
        Waypoint(1.8, (px, py, TRAVEL), open_, f"{lbl}: over", 0.0),
        Waypoint(1.5, (px, py, gz + 0.020), open_, f"{lbl}: descend", 0.0),
        Waypoint(1.0, (px, py, gz), open_, f"{lbl}: down slowly", 0.0),
        Waypoint(0.5, (px, py, gz), open_, f"{lbl}: settle", 0.0),
        Waypoint(1.5, (px, py, gz), grip, f"{lbl}: close", 0.0),
        Waypoint(2.0, (px, py, TRAVEL), grip, f"{lbl}: lift straight out", 0.0),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{lbl}: carry", 0.0),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{lbl}: settle over", 0.0),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{lbl}: lower", 0.0),
        Waypoint(1.5, (qx, qy, qz), grip, f"{lbl}: set down slowly", 0.0),
        Waypoint(0.6, (qx, qy, qz), grip, f"{lbl}: settle", 0.0),
        Waypoint(1.5, (qx, qy, qz), open_, f"{lbl}: release", 0.0),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{lbl}: retreat", 0.0),
    ]


def _push_lever():
    """Closed fingers turned 90 deg (pads across the slot), down behind the lever, then pushed slowly to the stop."""
    yaw = math.pi / 2
    tz = PUSH_TIP_Z + TIP
    back = OFF_X - LEVER[0] / 2 - CLOSED_HALF - 0.012
    end = RUN_X - LEVER[0] / 2 - CLOSED_HALF + PUSH_OVER
    y = SLOT_Y
    return [
        Waypoint(1.0, (back, y, TRAVEL), 0.0, "lever: close fingers", yaw),
        Waypoint(1.8, (back, y, TRAVEL), 0.0, "lever: over, behind it", yaw),
        Waypoint(1.5, (back, y, tz + 0.020), 0.0, "lever: descend", yaw),
        Waypoint(1.0, (back, y, tz), 0.0, "lever: down slowly", yaw),
        Waypoint(0.5, (back, y, tz), 0.0, "lever: settle", yaw),
        Waypoint(4.0, (end, y, tz), 0.0, "lever: push slowly to RUN", yaw),
        Waypoint(0.8, (end, y, tz), 0.0, "lever: hold", yaw),
        Waypoint(1.0, (end - 0.015, y, tz), 0.0, "lever: back off", yaw),
        Waypoint(1.5, (end - 0.015, y, TRAVEL), 0.0, "lever: retreat", yaw),
    ]


def demo():
    """Left arm only: the key straight up out of its socket and into the holder, then the start lever pushed from OFF
    to the RUN stop with the closed fingers. 30.1 s."""
    return {"left": _key_move() + _push_lever()}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, vel=(0.0, 0.0, 0.0)):
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def lying(x, y):
        size = (KEY[2], KEY[1], KEY[0])
        return box(size, x, y, TOP)

    start = {"chuck_key": box(KEY, *CHUCK_XY, CHUCK_TOP), "start_lever": box(LEVER, OFF_X, SLOT_Y, TOP)}
    stowed = {"chuck_key": box(KEY, *HOLDER_XY, TOP)}
    lifted = {"chuck_key": box(KEY, *CHUCK_XY, CHUCK_TOP + 0.05, vel=(0, 0, 0.05))}
    pushing = {"start_lever": box(LEVER, OFF_X + 0.02, SLOT_Y, TOP, vel=(0.02, 0, 0))}
    run_ = {"start_lever": box(LEVER, RUN_X, SLOT_Y, TOP)}

    def run(*frames):
        _MEM.clear()  # a new trial
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "key out, in the air": ((start, start | lifted), 0.0, False),
        "key stowed": ((start, start | stowed), 0.5, False),
        "key stowed, then start": ((start, start | lifted, start | stowed, stowed | pushing, stowed | run_), 1.0, True),
        "start with the key in the chuck": ((start, start | pushing, start | run_), 0.0, False),
        "start first, then stow the key": ((start, start | run_, stowed | run_), 0.5, False),
        "lever pushed while the key is carried": ((start, lifted | pushing, stowed | run_), 0.5, False),
        "key on the table, not in the holder, then start": ((start, start | {"chuck_key": lying(0.20, 0.36)},
                                                              {"chuck_key": lying(0.20, 0.36)} | run_), 0.0, False),
        "key lying across the holder": ((start, start | {"chuck_key": lying(*HOLDER_XY)}), 0.0, False),
        "lever short of RUN": ((start, stowed | {"start_lever": box(LEVER, RUN_X - 0.012, SLOT_Y, TOP)}), 0.5, False),
        "lever nudged 5 mm, not started": ((start, stowed | {"start_lever": box(LEVER, OFF_X + 0.005, SLOT_Y, TOP)}),
                                          0.5, False),
        "start, then put everything back to the start layout: still latched": ((start, start | run_, start, start | stowed,
                                                                                stowed | run_), 0.5, False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    # The key still gripped in its holder while the lever is pushed: not a violation (it is in the holder), but no
    # credit for the key until it is let go.
    global _tcps
    real = _tcps
    _tcps = lambda: [np.array([HOLDER_XY[0], HOLDER_XY[1], TOP + KEY[2] - 0.010])]  # noqa: E731
    try:
        got, got_success = run(start, start | stowed, stowed | pushing, stowed | run_)
        assert got == 0.5 and not got_success and not violated(), ("key still held", got, got_success)
    finally:
        _tcps = real
    _MEM.clear()
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
