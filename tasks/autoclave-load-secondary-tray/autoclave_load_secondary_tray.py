"""On the station table, in front of the left arm, a top-loading laboratory autoclave with its lid open: a fixed steel
chamber (four walls) with a flat load rack inside, 4 cm above the table, the walls rising 5 cm above the rack. On the
bench nearer the arms stand two glass media bottles of liquid on a black bench mat, and a white polypropylene
secondary tray lies on the bench beside them. Loading rule for liquids: bottles go into the autoclave only inside a
secondary tray, which catches boil-over or a broken bottle, and the load must not touch the chamber: the tray and
the bottles stay at least 25 mm from every wall so steam can circulate. Scored from the poses at the end.

Run it:  inspect-robots run --embodiment yam_arena \\
           -E task=tasks/autoclave-load-secondary-tray/autoclave_load_secondary_tray.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
TRAY = (0.120, 0.070, 0.020)  # secondary tray: 12 cm along x, 7 cm across (y), 2 cm tall
TRAY_KG = 0.120
BOTTLE_R, BOTTLE_H, BOTTLE_KG = 0.020, 0.060, 0.080
TRAY_START = (0.180, 0.250)
BOTTLE_START = {"bottle_1": (0.160, 0.420), "bottle_2": (0.250, 0.420)}
ON_TRAY = {"bottle_1": -0.032, "bottle_2": 0.032}  # where the demo stands each bottle, along the tray from its centre
BENCH_MAT = (0.220, 0.420)
BENCH_MAT_SIZE = (0.180, 0.080, 0.002)
MAT_TOP = TOP + BENCH_MAT_SIZE[2]

CHAMBER = (0.380, 0.300)  # chamber centre
CH_INNER = (0.200, 0.160)
CH_WALL_T = 0.008
RACK_H = 0.040  # the load rack's top above the table
RACK_TOP = TOP + RACK_H
WALL_TOP = RACK_TOP + 0.050
CLEARANCE = 0.025  # every wall cleared by at least 25 mm

STEEL = (0.72, 0.73, 0.75)
RACK_C = (0.50, 0.51, 0.53)
WHITE = (0.93, 0.93, 0.90)
GLASS = (0.62, 0.78, 0.85)
BLACK = (0.08, 0.08, 0.08)


def _chamber():
    cx, cy = CHAMBER
    ix, iy = CH_INNER
    t, h = CH_WALL_T, WALL_TOP - TOP
    z = TOP + h / 2
    return [
        Box("chamber_wall_left", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=STEEL),
        Box("chamber_wall_right", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=STEEL),
        Box("chamber_wall_front", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=STEEL),
        Box("chamber_wall_back", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=STEEL),
        Box("load_rack", pos=(cx, cy, TOP + RACK_H / 2), size=(ix, iy, RACK_H), color=RACK_C),
    ]


TRAY_OBJ = Box("secondary_tray", pos=(*TRAY_START, TOP + TRAY[2] / 2), size=TRAY, mass=TRAY_KG, color=WHITE)
BOTTLES = [Cylinder(n, pos=(*xy, MAT_TOP + BOTTLE_H / 2), radius=BOTTLE_R, height=BOTTLE_H, mass=BOTTLE_KG, color=GLASS)
           for n, xy in BOTTLE_START.items()]
FIXTURES = [
    *_chamber(),
    Box("bench_mat", pos=(*BENCH_MAT, TOP + BENCH_MAT_SIZE[2] / 2), size=BENCH_MAT_SIZE, color=BLACK),
]
OBJECTS = [TRAY_OBJ, *BOTTLES, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return yaw, tilt


def _on_tray(objects, name) -> bool:
    """Standing upright (within 8 degrees) on the tray: centre over the tray's top face in the tray's own frame,
    bottom within 4 mm of that face, still."""
    t, b = objects["secondary_tray"], objects[name]
    yaw, _ = _yaw_tilt(t.quat_xyzw)
    dx, dy = float(b.pos[0]) - float(t.pos[0]), float(b.pos[1]) - float(t.pos[1])
    u, v = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
    _, tilt = _yaw_tilt(b.quat_xyzw)
    top = float(t.pos[2]) + TRAY[2] / 2
    return bool(abs(u) < TRAY[0] / 2 and abs(v) < TRAY[1] / 2 and abs(float(b.low[2]) - top) <= 0.004
                and tilt <= 8.0 and b.resting())


def _in_chamber(st) -> bool:
    """Centre inside the chamber's inner walls and lowest point below the top of the walls."""
    return bool(
        abs(float(st.pos[0]) - CHAMBER[0]) < CH_INNER[0] / 2
        and abs(float(st.pos[1]) - CHAMBER[1]) < CH_INNER[1] / 2
        and float(st.low[2]) < WALL_TOP
    )


def _tray_on_rack(st) -> bool:
    _, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(_in_chamber(st) and abs(float(st.low[2]) - RACK_TOP) <= 0.004 and tilt <= 5.0 and st.resting())


def _wall_gap(st) -> float:
    """The smallest gap (m) between an object's world-aligned footprint and the four chamber walls."""
    cx, cy = CHAMBER
    ix, iy = CH_INNER
    lo, hi = st.low, st.high
    return float(min(float(lo[0]) - (cx - ix / 2), (cx + ix / 2) - float(hi[0]),
                     float(lo[1]) - (cy - iy / 2), (cy + iy / 2) - float(hi[1])))


def score(objects) -> float:
    """0.25 for each bottle standing on the secondary tray; 0.25 when the tray sits flat on the load rack with both
    bottles on it; 0.25 more when the tray and both bottles clear every chamber wall by 25 mm. A bottle in the
    chamber but not on the tray caps the score at 0.25."""
    names = [b.name for b in BOTTLES]
    on = [_on_tray(objects, n) for n in names]
    tray = objects["secondary_tray"]
    loaded = all(on) and _tray_on_rack(tray)
    clear = loaded and min(_wall_gap(objects[n]) for n in ("secondary_tray", *names)) >= CLEARANCE
    s = 0.25 * sum(on) + 0.25 * loaded + 0.25 * clear
    if any(_in_chamber(objects[n]) and not o for n, o in zip(names, on)):
        s = min(s, 0.25)
    return float(s)


def success(objects) -> bool:
    """Both bottles standing in the secondary tray, the tray flat on the rack, every wall cleared by 25 mm; holds for
    1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: each bottle to its spot on the tray plus the tray to the chamber centre."""
    t = objects["secondary_tray"]
    d = math.hypot(float(t.pos[0]) - CHAMBER[0], float(t.pos[1]) - CHAMBER[1])
    for n, dx in ON_TRAY.items():
        b = objects[n]
        d += math.dist((float(b.pos[0]), float(b.pos[1]), float(b.low[2])),
                       (float(t.pos[0]) + dx, float(t.pos[1]), float(t.pos[2]) + TRAY[2] / 2))
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
SQUEEZE = 0.008
CLEAR = 0.020
TRAVEL = TOP + 0.150
BOTTLE_DEPTH = 0.025  # pad tips 25 mm below a bottle's top
TRAY_DEPTH = 0.016  # pad tips 4 mm above the tray's bottom
HOVER = 0.002  # let go 2 mm above the resting height
Z_SAG = 0.0  # the loaded tray's sag in the grip; measured in sim and set here


def _bottle(name, pick, place):
    """A bottle gripped across its diameter near the top, carried and stood on the tray. 13.2 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    gz = pb + BOTTLE_H - BOTTLE_DEPTH + TIP
    qz = qb + BOTTLE_H - BOTTLE_DEPTH + TIP + HOVER
    open_ = -(2 * BOTTLE_R + CLEAR) / 2
    grip = -(2 * BOTTLE_R - SQUEEZE) / 2
    label = name.replace("_", " ")
    return [
        Waypoint(1.5, (px, py, TRAVEL), open_, f"{label}: over"),
        Waypoint(1.5, (px, py, gz), open_, f"{label}: descend"),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle"),
        Waypoint(1.2, (px, py, gz), grip, f"{label}: close"),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift"),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry"),
        Waypoint(0.5, (qx, qy, TRAVEL), grip, f"{label}: settle over the tray"),
        Waypoint(2.0, (qx, qy, qz), grip, f"{label}: stand on the tray"),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle"),
        Waypoint(1.2, (qx, qy, qz), open_, f"{label}: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat"),
    ]


def _tray(pick, place):
    """The loaded tray gripped across its 7 cm width at its centre, lifted and carried slowly over the chamber wall,
    lowered straight down onto the rack. 19.5 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    gz = pb + TRAY[2] - TRAY_DEPTH + TIP
    qz = qb + TRAY[2] - TRAY_DEPTH + TIP + HOVER + Z_SAG
    grip = -(TRAY[1] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), FINGER_OPEN, "tray: over"),
        Waypoint(1.5, (px, py, gz + 0.020), FINGER_OPEN, "tray: descend"),
        Waypoint(1.0, (px, py, gz), FINGER_OPEN, "tray: down slowly"),
        Waypoint(0.5, (px, py, gz), FINGER_OPEN, "tray: settle"),
        Waypoint(1.5, (px, py, gz), grip, "tray: close"),
        Waypoint(1.5, (px, py, gz + 0.030), grip, "tray: lift slowly"),
        Waypoint(1.2, (px, py, TRAVEL), grip, "tray: lift over the chamber wall"),
        Waypoint(3.0, (qx, qy, TRAVEL), grip, "tray: carry slowly"),
        Waypoint(0.8, (qx, qy, TRAVEL), grip, "tray: settle over the chamber"),
        Waypoint(2.0, (qx, qy, qz + 0.030), grip, "tray: lower into the chamber"),
        Waypoint(1.5, (qx, qy, qz), grip, "tray: set on the rack slowly"),
        Waypoint(0.7, (qx, qy, qz), grip, "tray: settle"),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "tray: release"),
        Waypoint(1.0, (qx, qy, TRAVEL), FINGER_OPEN, "tray: retreat"),
    ]


def demo():
    """Left arm only: both bottles from the bench mat onto the secondary tray, then the loaded tray onto the centre of
    the load rack. 45.9 s."""
    tray_top = TOP + TRAY[2]
    plan = []
    for name, dx in ON_TRAY.items():
        plan += _bottle(name, (*BOTTLE_START[name], MAT_TOP), (TRAY_START[0] + dx, TRAY_START[1], tray_top))
    plan += _tray((*TRAY_START, TOP), (*CHAMBER, RACK_TOP))
    return {"left": plan}


def self_test(ObjectState=None):
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def at(size, x, y, bottom, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    bsize = (2 * BOTTLE_R, 2 * BOTTLE_R, BOTTLE_H)

    def tray_at(x, y, bottom, bottles=("bottle_1", "bottle_2"), **kw):
        st = {"secondary_tray": at(TRAY, x, y, bottom, **kw)}
        for n in bottles:
            st[n] = at(bsize, x + ON_TRAY[n], y, bottom + TRAY[2])
        return st

    start = {"secondary_tray": at(TRAY, *TRAY_START, TOP)}
    start |= {n: at(bsize, *xy, MAT_TOP) for n, xy in BOTTLE_START.items()}
    cx, cy = CHAMBER
    cases = {
        "start": (start, 0.0, False),
        "one bottle on the tray": (start | tray_at(*TRAY_START, TOP, bottles=("bottle_1",)), 0.25, False),
        "both bottles on the tray, still on the bench": (start | tray_at(*TRAY_START, TOP), 0.5, False),
        "done": (start | tray_at(cx - 0.006, cy + 0.003, RACK_TOP), 1.0, True),
        "loaded tray against the back wall": (start | tray_at(cx + 0.030, cy, RACK_TOP), 0.75, False),
        "loaded tray 22 mm from the left wall": (start | tray_at(cx, cy + 0.023, RACK_TOP), 0.75, False),
        "tray on the rack with one bottle, the other on the rack beside it": (
            start | tray_at(cx, cy - 0.020, RACK_TOP, bottles=("bottle_1",)) | {"bottle_2": at(bsize, cx + 0.03, cy + 0.05, RACK_TOP)}, 0.25, False),
        "bottles straight onto the rack, no tray": (
            start | {"bottle_1": at(bsize, cx - 0.03, cy, RACK_TOP), "bottle_2": at(bsize, cx + 0.03, cy, RACK_TOP)}, 0.0, False),
        "tray on the rack but a bottle toppled on it": (
            start | tray_at(cx, cy, RACK_TOP, bottles=("bottle_1",)) | {"bottle_2": at(bsize, cx + 0.03, cy, RACK_TOP + TRAY[2], tilt_deg=90.0)}, 0.25, False),
        "loaded tray resting tilted on a wall": (start | tray_at(cx, cy, RACK_TOP + 0.01, tilt_deg=10.0), 0.5, False),
        "loaded tray still settling": (start | tray_at(cx, cy, RACK_TOP, vel=(0, 0, -0.03)), 0.5, False),
        "empty tray on the rack, bottles on the bench": (start | {"secondary_tray": at(TRAY, cx, cy, RACK_TOP)}, 0.0, False),
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
