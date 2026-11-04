"""On the station table, in front of the left arm, a kit for one bracket assembly has come back from inspection with
two errors. The bill of materials is two bolts, two nuts and four washers, each type in its own white pocket. The
bolt pocket is right. The nut pocket holds one nut and a washer that belongs in the washer pocket; the washer pocket
holds only three washers. Three dark-blue bins (bolts, nuts, washers) hold spare stock. Correcting the kit means
moving the stray washer to the washer pocket and adding one nut from its bin, without disturbing the bolt pocket.
Scored from the parts in each pocket at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/kit-audit-and-correct/kit_audit_and_correct.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
PARTS = {  # radius, height, mass, colour; plain cylinders standing on end
    "bolt": (0.006, 0.045, 0.035, (0.62, 0.62, 0.64)),
    "nut": (0.009, 0.009, 0.010, (0.30, 0.30, 0.32)),
    "washer": (0.012, 0.006, 0.006, (0.80, 0.80, 0.78)),
}
BIN_X = {"bolt": 0.150, "nut": 0.275, "washer": 0.400}  # front to back
BIN_Y = 0.410
BIN_INNER = (0.115, 0.090)
STOCK = {"bolt": 2, "nut": 2, "washer": 2}  # spare stock in the bins
POCKETS = {"bolt": (0.180, 0.200), "nut": (0.300, 0.200), "washer": (0.420, 0.200)}
POCKET_INNER = (0.090, 0.070)
WALL_T = 0.006
WALL_H = 0.015
WANT = {"bolt": {"bolt": 2}, "nut": {"nut": 2}, "washer": {"washer": 4}}
# The kit as it came back: (kind, pocket, dx along x from the pocket centre, layer)
KITTED = [("bolt", "bolt", -0.020, 0), ("bolt", "bolt", 0.020, 0), ("nut", "nut", -0.020, 0),
          ("washer", "nut", 0.020, 0), ("washer", "washer", -0.020, 0), ("washer", "washer", 0.020, 0),
          ("washer", "washer", -0.020, 1)]

BLUE = (0.10, 0.20, 0.45)
WHITE = (0.92, 0.92, 0.92)
SPOTS = [(-0.035, 0.0), (0.0, 0.0), (0.035, 0.0)]  # in each bin, along x: clear of the 38 mm pads


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


PART_OBJECTS = []
for _k, (_kind, _pocket, _dx, _layer) in enumerate(KITTED):
    _r, _h, _m, _c = PARTS[_kind]
    _px, _py = POCKETS[_pocket]
    PART_OBJECTS.append(Cylinder(f"{_kind}_kit{_k}", pos=(_px + _dx, _py, TOP + _layer * _h + _h / 2), radius=_r,
                                 height=_h, mass=_m, color=_c))
for _kind, _n in STOCK.items():
    _r, _h, _m, _c = PARTS[_kind]
    for _k in range(_n):
        _dx, _dy = SPOTS[_k]
        PART_OBJECTS.append(Cylinder(f"{_kind}_{_k}", pos=(BIN_X[_kind] + _dx, BIN_Y + _dy, TOP + _h / 2), radius=_r,
                                     height=_h, mass=_m, color=_c))
KIND_OF = {p.name: p.name.split("_")[0] for p in PART_OBJECTS}
FIXTURES = [w for k, x in BIN_X.items() for w in _walls(f"{k}_bin", (x, BIN_Y), BIN_INNER, BLUE)]
FIXTURES += [w for k, c in POCKETS.items() for w in _walls(f"{k}_pocket", c, POCKET_INNER, WHITE)]
OBJECTS = [*PART_OBJECTS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _bottom(state, kind) -> float:
    r, h, _, _ = PARTS[kind]
    x, y, _, _ = (float(v) for v in state.quat_xyzw)
    uz = 1.0 - 2.0 * (x * x + y * y)
    return float(state.pos[2]) - (h / 2 * abs(uz) + r * math.sqrt(max(0.0, 1.0 - uz * uz)))


def _in_pocket(state, kind, centre) -> bool:
    x, y = float(state.pos[0]), float(state.pos[1])
    inside = abs(x - centre[0]) < POCKET_INNER[0] / 2 and abs(y - centre[1]) < POCKET_INNER[1] / 2
    return bool(inside and state.resting() and TOP - 0.005 < _bottom(state, kind) < TOP + 0.050)


def contents(objects):
    out = {p: dict.fromkeys(PARTS, 0) for p in POCKETS}
    for part in PART_OBJECTS:
        kind = KIND_OF[part.name]
        for p, c in POCKETS.items():
            if _in_pocket(objects[part.name], kind, c):
                out[p][kind] += 1
    return out


def _right(counts, want) -> bool:
    return all(counts[k] == want.get(k, 0) for k in PARTS)


def success(objects) -> bool:
    """Two bolts, two nuts and four washers, each type alone in its own pocket; holds for 1 s."""
    c = contents(objects)
    return all(_right(c[p], WANT[p]) for p in POCKETS)


def score(objects) -> float:
    """0.5 for the nut pocket right (exactly two nuts, nothing else) and 0.5 for the washer pocket right (exactly four
    washers, nothing else). The bolt pocket starts right and must stay so: if it no longer holds exactly two bolts and
    nothing else, the score is capped at 0.25. The kit as it came back scores 0."""
    c = contents(objects)
    s = 0.5 * _right(c["nut"], WANT["nut"]) + 0.5 * _right(c["washer"], WANT["washer"])
    if not _right(c["bolt"], WANT["bolt"]):
        return min(s, 0.25)
    return s


def distance(objects) -> float:
    c = contents(objects)
    return float(sum(abs(c[p][k] - WANT[p].get(k, 0)) for p in POCKETS for k in PARTS))


# -- the scripted demo ------------------------------------------------------------------------------------------

TRAVEL = TOP + 0.110
SQUEEZE = 0.008
CLEAR = 0.020
TIP = 0.013


def _grip_z(kind):
    _, h, _, _ = PARTS[kind]
    return TOP + h - 0.022 + TIP if kind == "bolt" else TOP + 0.002 + TIP


def _move(pick, place, kind, layer=0):
    (px, py), (qx, qy) = pick, place
    r = PARTS[kind][0]
    gz, qz = _grip_z(kind), _grip_z(kind) + 0.003 + layer * PARTS[kind][1]
    open_ = -(2 * r + CLEAR) / 2
    grip = -(2 * r - SQUEEZE) / 2
    return [
        Waypoint(1.5, (px, py, TRAVEL), open_, f"{kind}: over the bin"),
        Waypoint(1.5, (px, py, gz), open_, f"{kind}: descend"),
        Waypoint(0.5, (px, py, gz), open_, f"{kind}: settle"),
        Waypoint(1.2, (px, py, gz), grip, f"{kind}: close"),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{kind}: lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{kind}: carry"),
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{kind}: settle over the pocket"),
        Waypoint(2.0, (qx, qy, qz), grip, f"{kind}: set in"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{kind}: settle"),
        Waypoint(1.2, (qx, qy, qz), open_, f"{kind}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{kind}: retreat"),
    ]


def demo():
    """Left arm only: the stray washer from the nut pocket onto the washer pocket's short stack, then a nut from its bin
    into the nut pocket. 26 s."""
    nx, ny = POCKETS["nut"]
    wx, wy = POCKETS["washer"]
    moves = _move((nx + 0.020, ny), (wx + 0.020, wy), "washer", layer=1)
    moves += _move((BIN_X["nut"] + SPOTS[0][0], BIN_Y + SPOTS[0][1]), (nx + 0.020, ny), "nut")
    return {"left": moves}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def part(kind, x, y, bottom=TOP, vel=(0.0, 0.0, 0.0)):
        r, h, _, _ = PARTS[kind]
        pos = np.array([x, y, bottom + h / 2])
        half = np.array([r, r, h / 2])
        return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {p.name: part(KIND_OF[p.name], p.pos[0], p.pos[1], bottom=p.pos[2] - p.height / 2) for p in PART_OBJECTS}
    nx, ny = POCKETS["nut"]
    wx, wy = POCKETS["washer"]
    bx, by = POCKETS["bolt"]
    washer_moved = {"washer_kit3": part("washer", wx + 0.020, wy, bottom=TOP + 0.006)}
    nut_added = {"nut_0": part("nut", nx + 0.020, ny)}
    cases = {
        "as it came back": (start, 0.0, False),
        "corrected": (start | washer_moved | nut_added, 1.0, True),
        "washer moved only": (start | washer_moved, 0.5, False),
        "nut added beside the stray washer": (start | nut_added | {"washer_kit3": part("washer", nx, ny + 0.02)}, 0.0, False),
        "stray washer moved, a fresh washer added too (five)": (
            start | washer_moved | nut_added | {"washer_0": part("washer", wx, wy + 0.02)}, 0.5, False),
        "corrected but a bolt knocked out of its pocket": (
            start | washer_moved | nut_added | {"bolt_kit0": part("bolt", bx, by + 0.06)}, 0.25, False),
        "corrected but a third bolt added": (start | washer_moved | nut_added | {"bolt_0": part("bolt", bx, by + 0.02)}, 0.25, False),
        "stray washer thrown back in its bin, a fresh one added": (
            start | nut_added | {"washer_kit3": part("washer", BIN_X["washer"], BIN_Y + 0.03),
                                 "washer_0": part("washer", wx + 0.020, wy, bottom=TOP + 0.006)}, 1.0, True),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


if __name__ == "__main__":
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
