"""On the station table, in front of the left arm, a machine-tending cell. Straight ahead stands the machine's work-
holding nest: a fixed steel base with two fixed parallel jaws running along x, 34 mm apart. Between the jaws lies a
finished part, an aluminium bar 30 x 30 mm in section and 55 mm long (machined down from a 70 mm blank), its long axis
along the jaws. Two identical grey open trays stand nearer the arms: the left one holds two raw blanks (70 mm bars,
their long axis along y, across the jaws' direction), the right one holds one finished 55 mm part from the last
cycle. Tending the machine means unloading the finished part into the finished-parts tray, then loading one raw blank
into the nest, turned so its long axis runs between the jaws. Which tray is which is told by the parts' lengths only.
Scored from the poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/machine-tending-swap/machine_tending_swap.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
SECTION = 0.030  # every bar is 30 x 30 mm in section
BLANK_L = 0.070  # raw blank length
DONE_L = 0.055  # finished length: 15 mm faced off
BLANK_KG, DONE_KG = 0.150, 0.120

NEST = (0.330, 0.300)  # centre of the nest between the jaws
BASE = (0.090, 0.070, 0.012)  # the nest's steel base, x y z
FLOOR = TOP + BASE[2]
JAW = (0.040, 0.012, 0.015)  # each jaw: 40 mm along x, 12 mm thick, 15 mm above the floor
JAW_GAP = 0.034  # between the jaws' inner faces: 2 mm each side of a 30 mm bar
JAW_DY = JAW_GAP / 2 + JAW[1] / 2

TRAY_INNER = (0.120, 0.130)
WALL_T = 0.006
WALL_H = 0.015
RAW_TRAY = (0.180, 0.420)
DONE_TRAY = (0.180, 0.180)
BLANK_DX = 0.028  # the two blanks lie side by side in the raw tray, 56 mm apart along x
DONE_DY = 0.032  # finished parts lie side by side in the finished tray, 64 mm apart along y

STEEL = (0.45, 0.47, 0.50)
ALU = (0.80, 0.81, 0.83)
GREY = (0.55, 0.56, 0.58)


def _walls(name, centre, inner, color):
    cx, cy = centre
    ix, iy = inner
    z = TOP + WALL_H / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - WALL_T / 2, z), size=(ix + 2 * WALL_T, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL_T / 2, cy, z), size=(WALL_T, iy, WALL_H), color=color),
    ]


FINISHED = Box("finished_part", pos=(*NEST, FLOOR + SECTION / 2), size=(DONE_L, SECTION, SECTION), mass=DONE_KG, color=ALU)
PREVIOUS = Box("finished_prev", pos=(DONE_TRAY[0], DONE_TRAY[1] - DONE_DY, TOP + SECTION / 2), size=(DONE_L, SECTION, SECTION),
               mass=DONE_KG, color=ALU)
BLANKS = [
    Box(f"blank_{k + 1}", pos=(RAW_TRAY[0] + dx, RAW_TRAY[1], TOP + SECTION / 2), size=(SECTION, BLANK_L, SECTION),
        mass=BLANK_KG, color=ALU)
    for k, dx in enumerate((-BLANK_DX, BLANK_DX))
]
FIXTURES = [
    Box("nest_base", pos=(*NEST, TOP + BASE[2] / 2), size=BASE, color=STEEL),
    Box("jaw_left", pos=(NEST[0], NEST[1] + JAW_DY, FLOOR + JAW[2] / 2), size=JAW, color=STEEL),
    Box("jaw_right", pos=(NEST[0], NEST[1] - JAW_DY, FLOOR + JAW[2] / 2), size=JAW, color=STEEL),
    *_walls("raw_tray", RAW_TRAY, TRAY_INNER, GREY),
    *_walls("done_tray", DONE_TRAY, TRAY_INNER, GREY),
]
OBJECTS = [FINISHED, PREVIOUS, *BLANKS, *FIXTURES]
FINISHED_NAMES = ("finished_part", "finished_prev")
BLANK_NAMES = tuple(b.name for b in BLANKS)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_Y = 0.003  # seated: centred between the jaws within 3 mm
SEAT_X = 0.010  # and within 1 cm of the nest centre along them
SEAT_Z = 0.002  # bottom within 2 mm of the nest floor
SEAT_DEG = 5.0  # long axis along the jaws
AT_NEST = 0.035  # a part is at the nest with its centre within 3.5 cm of the nest centre


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _off_axis(yaw_deg, axis_deg) -> float:
    """Degrees between a bar's long axis and an axis, a bar turned 180 degrees being the same bar."""
    return abs((yaw_deg - axis_deg + 90.0) % 180.0 - 90.0)


def _at_nest(state) -> bool:
    return math.hypot(float(state.pos[0]) - NEST[0], float(state.pos[1]) - NEST[1]) < AT_NEST


def _in_tray(state, centre) -> bool:
    """At rest, the centre inside the tray's inner walls, the bottom less than 4 cm above the table."""
    return bool(
        abs(float(state.pos[0]) - centre[0]) < TRAY_INNER[0] / 2
        and abs(float(state.pos[1]) - centre[1]) < TRAY_INNER[1] / 2
        and TOP - 0.005 < float(state.low[2]) < TOP + 0.040
        and state.resting()
    )


def _seated(state, long_axis_deg) -> bool:
    """Lying on the nest floor between the jaws, its long axis along them; ``long_axis_deg`` is the bar's long axis
    in its own frame (0 = x, 90 = y)."""
    yaw, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(
        abs(float(state.pos[1]) - NEST[1]) <= SEAT_Y
        and abs(float(state.pos[0]) - NEST[0]) <= SEAT_X
        and abs(float(state.low[2]) - FLOOR) <= SEAT_Z
        and _off_axis(yaw + long_axis_deg, 0.0) <= SEAT_DEG
        and tilt <= SEAT_DEG
        and state.resting()
    )


def _steps(objects):
    fin = objects["finished_part"]
    unloaded = not _at_nest(fin)
    in_done = _in_tray(fin, DONE_TRAY)
    loaded = any(_at_nest(objects[b]) and objects[b].resting() for b in BLANK_NAMES)
    seated = any(_seated(objects[b], 90.0) for b in BLANK_NAMES)
    mixed = any(_in_tray(objects[b], DONE_TRAY) for b in BLANK_NAMES) or any(
        _in_tray(objects[f], RAW_TRAY) for f in FINISHED_NAMES)
    return unloaded, in_done, loaded, seated, mixed


def success(objects) -> bool:
    """Finished part in the finished tray, a blank seated between the jaws, no part in the wrong tray; holds 1 s."""
    _, in_done, _, seated, mixed = _steps(objects)
    return in_done and seated and not mixed


def score(objects) -> float:
    """In order, each step needing the ones before: 0.25 the finished part is out of the nest; 0.5 it rests in the
    finished tray; 0.75 a raw blank also rests at the nest (centre within 3.5 cm, e.g. lying across the jaws); 1.0
    the blank is seated: on the floor between the jaws, centred across them within 3 mm, long axis along them within
    5 degrees. A raw blank in the finished tray or a finished part in the raw tray caps the score at 0.25."""
    unloaded, in_done, loaded, seated, mixed = _steps(objects)
    s = 0.0
    if unloaded:
        s = 0.25
        if in_done:
            s = 0.5
            if loaded:
                s = 1.0 if seated else 0.75
    return min(s, 0.25) if mixed else s


def distance(objects) -> float:
    fin = objects["finished_part"]
    d = math.hypot(float(fin.pos[0]) - DONE_TRAY[0], float(fin.pos[1]) - (DONE_TRAY[1] + DONE_DY))
    return float(d + min(math.hypot(float(objects[b].pos[0]) - NEST[0], float(objects[b].pos[1]) - NEST[1])
                         for b in BLANK_NAMES))


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.130
SQUEEZE = 0.008
DEPTH = 0.012  # pad tips 18 mm above a bar's bottom: clear of the 15 mm jaws and tray walls
X_OPEN = -0.046  # nearly fully open


def _move(pick, place, *, length, pick_bottom, place_bottom, yaw_pick, yaw_place, label):
    """Top-down pick and place across a bar's ``length`` (its ends), turning the wrist from ``yaw_pick`` to
    ``yaw_place`` on the way. 16.5 s."""
    (px, py), (qx, qy) = pick, place
    gz = pick_bottom + SECTION - DEPTH + TIP
    qz = place_bottom + SECTION - DEPTH + TIP + 0.002
    grip = -(length - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), X_OPEN, f"{label}: over", yaw_pick),
        Waypoint(1.5, (px, py, gz + 0.020), X_OPEN, f"{label}: descend", yaw_pick),
        Waypoint(1.0, (px, py, gz), X_OPEN, f"{label}: down slowly", yaw_pick),
        Waypoint(0.5, (px, py, gz), X_OPEN, f"{label}: settle", yaw_pick),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", yaw_pick),
        Waypoint(1.5, (px, py, TRAVEL), grip, f"{label}: lift", yaw_pick),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", yaw_place),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", yaw_place),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower", yaw_place),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set down slowly", yaw_place),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", yaw_place),
        Waypoint(1.5, (qx, qy, qz), X_OPEN, f"{label}: release", yaw_place),
        Waypoint(1.0, (qx, qy, TRAVEL), X_OPEN, f"{label}: retreat", yaw_place),
    ]


def demo():
    """Left arm only: the finished part out of the nest by its ends (fingers along x) into the free half of the
    finished tray, then the near raw blank picked by its ends (fingers along y), turned a quarter turn over the
    nest and lowered between the jaws. 33.0 s."""
    unload = _move(NEST, (DONE_TRAY[0], DONE_TRAY[1] + DONE_DY), length=DONE_L, pick_bottom=FLOOR, place_bottom=TOP,
                   yaw_pick=math.pi / 2, yaw_place=math.pi / 2, label="finished part")
    load = _move((RAW_TRAY[0] - BLANK_DX, RAW_TRAY[1]), NEST, length=BLANK_L, pick_bottom=TOP, place_bottom=FLOOR,
                 yaw_pick=0.0, yaw_place=math.pi / 2, label="blank")
    return {"left": unload + load}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def bar(length, along_y, x, y, bottom, yaw_deg=0.0, vel=(0.0, 0.0, 0.0)):
        size = (SECTION, length, SECTION) if along_y else (length, SECTION, SECTION)
        yaw = math.radians(yaw_deg)
        q = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        half = np.array([c * size[0] / 2 + s * size[1] / 2, s * size[0] / 2 + c * size[1] / 2, SECTION / 2])
        pos = np.array([x, y, bottom + SECTION / 2])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {
        "finished_part": bar(DONE_L, False, *NEST, FLOOR),
        "finished_prev": bar(DONE_L, False, DONE_TRAY[0], DONE_TRAY[1] - DONE_DY, TOP),
        "blank_1": bar(BLANK_L, True, RAW_TRAY[0] - BLANK_DX, RAW_TRAY[1], TOP),
        "blank_2": bar(BLANK_L, True, RAW_TRAY[0] + BLANK_DX, RAW_TRAY[1], TOP),
    }
    fin_done = {"finished_part": bar(DONE_L, False, DONE_TRAY[0], DONE_TRAY[1] + DONE_DY, TOP)}
    blank_in = {"blank_1": bar(BLANK_L, True, NEST[0] - 0.004, NEST[1] + 0.001, FLOOR, yaw_deg=90)}
    done = start | fin_done | blank_in
    cases = {
        "start": (start, 0.0, False),
        "done": (done, 1.0, True),
        "done, blank turned end for end": (start | fin_done | {"blank_1": bar(BLANK_L, True, *NEST, FLOOR, yaw_deg=-90)}, 1.0, True),
        "finished part lifted out onto the table": (start | {"finished_part": bar(DONE_L, False, 0.25, 0.30, TOP)}, 0.25, False),
        "finished part in its tray": (start | fin_done, 0.5, False),
        "blank lying across the jaws": (start | fin_done | {"blank_1": bar(BLANK_L, True, *NEST, FLOOR + JAW[2])}, 0.75, False),
        "blank seated but turned 8 degrees": (start | fin_done | {"blank_1": bar(BLANK_L, True, *NEST, FLOOR, yaw_deg=98)}, 0.75, False),
        "blank still sliding in the nest": (start | fin_done | {"blank_1": bar(BLANK_L, True, *NEST, FLOOR, yaw_deg=90, vel=(0.03, 0, 0))}, 0.5, False),
        "blank seated, finished part left on the table": (start | blank_in | {"finished_part": bar(DONE_L, False, 0.25, 0.20, TOP)}, 0.25, False),
        "finished part into the raw tray, blank seated": (start | blank_in | {"finished_part": bar(DONE_L, False, RAW_TRAY[0], RAW_TRAY[1] - 0.045, TOP)}, 0.25, False),
        "done, plus the other blank in the finished tray": (done | {"blank_2": bar(BLANK_L, False, DONE_TRAY[0], DONE_TRAY[1], TOP)}, 0.25, False),
        "old finished part loaded into the nest": (start | fin_done | {"finished_prev": bar(DONE_L, False, *NEST, FLOOR)}, 0.5, False),
        "blank seated, finished part stacked on the tray's other part": (start | blank_in | {"finished_part": bar(DONE_L, False, DONE_TRAY[0], DONE_TRAY[1] - DONE_DY, TOP + SECTION)}, 1.0, True),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


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
