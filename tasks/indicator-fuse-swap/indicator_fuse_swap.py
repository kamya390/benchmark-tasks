"""On the station table, in front of the left arm, a DIN rail lies flat, running left to right, carrying four fuse
holders 4 cm apart. Each holder is an open-top cradle (four low walls on the rail) with a cartridge fuse standing in
it. The holders are of two sizes: 14 x 51 mm holders in positions 1 and 3 and 10 x 38 mm holders in positions 2 and 4,
counting from the left. The fuse in position 3 has blown and its indicator has tripped: it stands 4 mm proud of a
good fuse of its size and shows red. Nearer the arms, a white spares tray holds one new fuse of each size; further
out lies a red scrap tray. The job: pull the blown fuse, put it in the scrap tray, and seat the spare of the size its
holder takes (14 x 51) in the vacated holder, without disturbing the three good fuses. Scored from the fuse poses at
the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/indicator-fuse-swap/indicator_fuse_swap.py \\
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
SIZES = {  # fuse class: (body width = depth, body height, mass); the cartridge modelled as a square-section box
    "10x38": (0.010, 0.038, 0.012),
    "14x51": (0.014, 0.051, 0.022),
}
INDICATOR = 0.004  # a tripped fuse's indicator pin stands 4 mm proud of its cap
RAIL_X = 0.300
RAIL_Y = 0.250
RAIL = (0.035, 0.200, 0.0075)  # TS35 x 7.5 rail, 20 cm long, its top-hat profile as a solid bar
FLOOR = TOP + RAIL[2]  # the holders' floor: the rail's top face
PITCH = 0.040
HOLDERS = {1: "14x51", 2: "10x38", 3: "14x51", 4: "10x38"}  # position (from the left) -> size
SLOT_Y = {k: RAIL_Y + (2.5 - k) * PITCH for k in HOLDERS}  # 0.31, 0.27, 0.23, 0.19
BLOWN = 3
CLEAR = 0.003  # cradle inner = fuse + 3 mm each side
CRADLE_T = 0.003
CRADLE_H = {"10x38": 0.010, "14x51": 0.012}

TRAY_INNER = (0.070, 0.110)
SCRAP_INNER = (0.080, 0.080)
WALL_T = 0.006
WALL_H = 0.015
SPARES = (0.160, 0.250)  # tray centres
SCRAP = (0.420, 0.250)
SPARE_Y = {"10x38": 0.280, "14x51": 0.220}

CERAMIC = (0.90, 0.88, 0.80)
RED = (0.80, 0.10, 0.08)
DARK = (0.18, 0.18, 0.20)
STEEL = (0.75, 0.76, 0.78)
WHITE = (0.92, 0.92, 0.92)
SCRAP_RED = (0.70, 0.12, 0.10)


def _frame(name, centre, inner, color, t=WALL_T, h=WALL_H, z0=TOP):
    cx, cy = centre
    ix, iy = inner
    z = z0 + h / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=color),
    ]


def _height(name):
    size = KIND[name]
    return SIZES[size][1] + (INDICATOR if name == f"fuse_{BLOWN}" else 0.0)


def _fuse(name, size, x, y, bottom, color):
    w, h, m = SIZES[size]
    if name == f"fuse_{BLOWN}":
        h += INDICATOR
    return Box(name, pos=(x, y, bottom + h / 2), size=(w, w, h), mass=m, color=color)


KIND = {f"fuse_{k}": s for k, s in HOLDERS.items()} | {"spare_10x38": "10x38", "spare_14x51": "14x51"}
FUSES = [_fuse(f"fuse_{k}", s, RAIL_X, SLOT_Y[k], FLOOR, RED if k == BLOWN else CERAMIC) for k, s in HOLDERS.items()]
SPARE_FUSES = [_fuse(f"spare_{s}", s, SPARES[0], SPARE_Y[s], TOP, CERAMIC) for s in SIZES]
GOOD = [f for f in FUSES if f.name != f"fuse_{BLOWN}"]
FIXTURES = [Box("rail", pos=(RAIL_X, RAIL_Y, TOP + RAIL[2] / 2), size=RAIL, color=STEEL)]
for _k, _s in HOLDERS.items():
    _w = SIZES[_s][0] + 2 * CLEAR
    FIXTURES += _frame(f"holder_{_k}", (RAIL_X, SLOT_Y[_k]), (_w, _w), DARK, CRADLE_T, CRADLE_H[_s], FLOOR)
FIXTURES += _frame("spares", SPARES, TRAY_INNER, WHITE) + _frame("scrap", SCRAP, SCRAP_INNER, SCRAP_RED)
OBJECTS = [*FUSES, *SPARE_FUSES, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

RIGHT_SPARE = f"spare_{HOLDERS[BLOWN]}"
WRONG_SPARE = next(f"spare_{s}" for s in SIZES if s != HOLDERS[BLOWN])
SEAT_XY = CLEAR + 0.001  # seated: centre inside the cradle (within 4 mm of its centre each way)
SEAT_Z = 0.002  # bottom within 2 mm of the holder floor
UPRIGHT_DEG = 5.0
MOVED = 0.003  # a good fuse counts as undisturbed within 3 mm and 5 degrees of its start


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _off(state, slot):
    return float(state.pos[0]) - RAIL_X, float(state.pos[1]) - SLOT_Y[slot]


def _in_holder(state, slot, tol=0.010) -> bool:
    """Standing in the holder's footprint: centre within 1 cm of the cradle centre, bottom within 5 mm of its floor."""
    dx, dy = _off(state, slot)
    return bool(max(abs(dx), abs(dy)) <= tol and abs(float(state.low[2]) - FLOOR) <= 0.005)


def _seated(state, slot) -> bool:
    dx, dy = _off(state, slot)
    return bool(
        max(abs(dx), abs(dy)) <= SEAT_XY
        and abs(float(state.low[2]) - FLOOR) <= SEAT_Z
        and _tilt(state.quat_xyzw) <= UPRIGHT_DEG
        and state.resting()
    )


def _out(state) -> bool:
    """Clear of its holder: centre more than 1.5 cm from the cradle centre, or bottom 2 cm above the floor."""
    dx, dy = _off(state, BLOWN)
    return bool(math.hypot(dx, dy) > 0.015 or float(state.low[2]) > FLOOR + 0.020)


def _in_scrap(state) -> bool:
    """At rest in the scrap tray, standing or lying: centre inside the inner walls, bottom within 2 cm of the table."""
    return bool(
        abs(float(state.pos[0]) - SCRAP[0]) < SCRAP_INNER[0] / 2
        and abs(float(state.pos[1]) - SCRAP[1]) < SCRAP_INNER[1] / 2
        and TOP - 0.005 < float(state.low[2]) < TOP + 0.020
        and state.resting()
    )


def _undisturbed(objects) -> bool:
    for f in GOOD:
        st = objects[f.name]
        if math.dist((float(st.pos[0]), float(st.pos[1]), float(st.pos[2])), f.pos) > MOVED:
            return False
        if _tilt(st.quat_xyzw) > UPRIGHT_DEG:
            return False
    return True


def success(objects) -> bool:
    """The blown fuse in the scrap tray, the 14 x 51 spare seated in its holder, the good fuses untouched; 1 s."""
    return bool(
        _in_scrap(objects[f"fuse_{BLOWN}"]) and _seated(objects[RIGHT_SPARE], BLOWN) and _undisturbed(objects)
    )


def score(objects) -> float:
    """0 while the blown fuse is still in its holder. 0.25 once it is out; 0.5 once it rests in the scrap tray; 0.75
    if the 14 x 51 spare then stands in the vacated holder (centre within 1 cm, bottom within 5 mm of the floor) and
    1.0 when it is seated (centre inside the cradle, bottom within 2 mm of the floor, upright within 5 degrees, still).
    The score is capped at 0.25 if the 10 x 38 spare is in the vacated holder (wrong rating for that circuit) or any
    good fuse has moved more than 3 mm or tilted more than 5 degrees."""
    blown = objects[f"fuse_{BLOWN}"]
    if not _out(blown):
        return 0.0
    if not _undisturbed(objects) or _in_holder(objects[WRONG_SPARE], BLOWN, 0.015):
        return 0.25
    if not _in_scrap(blown):
        return 0.25
    spare = objects[RIGHT_SPARE]
    if _seated(spare, BLOWN):
        return 1.0
    if _in_holder(spare, BLOWN):
        return 0.75
    return 0.5


def distance(objects) -> float:
    """Metres to go: the blown fuse to the scrap tray's centre plus the right spare to the vacated holder."""
    b, s = objects[f"fuse_{BLOWN}"], objects[RIGHT_SPARE]
    return float(
        math.hypot(float(b.pos[0]) - SCRAP[0], float(b.pos[1]) - SCRAP[1])
        + math.dist((float(s.pos[0]), float(s.pos[1]), float(s.low[2])), (RAIL_X, SLOT_Y[BLOWN], FLOOR))
    )


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x, onto the fuse's front and back faces; neighbours sit beside the pads
TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
GRIP_UP = 0.018  # pad tips 18 mm above the fuse's bottom: 6 mm clear of the cradle walls
X_TRIM = 0.0015  # light parts land about 1.5 mm short in x (measured in sim on 10-35 g parts)


def _move(name, pick, place, label):
    """Top-down pick and place of one standing fuse, settling before every close and release, the last 2 cm slow."""
    (px, py, pb), (qx, qy, qb) = pick, place
    w = SIZES[KIND[name]][0]
    gz, qz = pb + GRIP_UP + TIP, qb + GRIP_UP + TIP + 0.002  # set down 2 mm above the floor
    open_ = -(w + 0.020) / 2
    grip = -(w - SQUEEZE) / 2
    qx += X_TRIM
    return [
        Waypoint(1.8, (px, py, TRAVEL), open_, f"{label}: over", YAW),
        Waypoint(1.5, (px, py, gz + 0.020), open_, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), open_, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle", YAW),
        Waypoint(1.2, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, gz + 0.030), grip, f"{label}: straight up", YAW),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set in slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.2, (qx, qy, qz), open_, f"{label}: release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: the blown fuse straight up out of holder 3 and into the scrap tray, then the 14 x 51 spare from
    the spares tray straight down into holder 3. 34.2 s."""
    out = _move(f"fuse_{BLOWN}", (RAIL_X, SLOT_Y[BLOWN], FLOOR), (SCRAP[0], SCRAP[1], TOP), "blown fuse")
    into = _move(RIGHT_SPARE, (SPARES[0], SPARE_Y[HOLDERS[BLOWN]], TOP), (RAIL_X, SLOT_Y[BLOWN], FLOOR), "spare")
    return {"left": out + into}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def fuse(name, x, y, bottom, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        w, h = SIZES[KIND[name]][0], _height(name)
        t = math.radians(tilt_deg)
        q = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])
        hz = h / 2 * math.cos(t) + w / 2 * math.sin(t)
        pos = np.array([x, y, bottom + hz])
        half = np.array([w / 2, w / 2 * math.cos(t) + h / 2 * math.sin(t), hz])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {f.name: fuse(f.name, f.pos[0], f.pos[1], FLOOR) for f in FUSES}
    start |= {f.name: fuse(f.name, f.pos[0], f.pos[1], TOP) for f in SPARE_FUSES}
    blown = f"fuse_{BLOWN}"
    in_scrap = fuse(blown, SCRAP[0], SCRAP[1], TOP)
    seated = fuse(RIGHT_SPARE, RAIL_X, SLOT_Y[BLOWN], FLOOR)

    def scene(**kw):
        return start | kw

    cases = {
        "start": (start, 0.0, False),
        "done": (scene(**{blown: in_scrap, RIGHT_SPARE: seated}), 1.0, True),
        "done, blown fuse lying in the scrap tray": (scene(**{blown: fuse(blown, SCRAP[0], SCRAP[1], TOP, 90), RIGHT_SPARE: seated}), 1.0, True),
        "spare 3 mm off centre": (scene(**{blown: in_scrap, RIGHT_SPARE: fuse(RIGHT_SPARE, RAIL_X - 0.003, SLOT_Y[3], FLOOR)}), 1.0, True),
        "blown fuse lifted 3 cm": (scene(**{blown: fuse(blown, RAIL_X, SLOT_Y[3], FLOOR + 0.03)}), 0.25, False),
        "blown fuse on the table": (scene(**{blown: fuse(blown, 0.25, 0.10, TOP, 90)}), 0.25, False),
        "blown fuse in the scrap tray": (scene(**{blown: in_scrap}), 0.5, False),
        "blown fuse in the spares tray": (scene(**{blown: fuse(blown, SPARES[0], 0.25, TOP)}), 0.25, False),
        "right spare leaning in the holder": (scene(**{blown: in_scrap, RIGHT_SPARE: fuse(RIGHT_SPARE, RAIL_X, SLOT_Y[3], FLOOR, 12)}), 0.75, False),
        "right spare still moving": (scene(**{blown: in_scrap, RIGHT_SPARE: fuse(RIGHT_SPARE, RAIL_X, SLOT_Y[3], FLOOR, vel=(0, 0, -0.05))}), 0.75, False),
        "right spare held over the holder": (scene(**{blown: in_scrap, RIGHT_SPARE: fuse(RIGHT_SPARE, RAIL_X, SLOT_Y[3], FLOOR + 0.03)}), 0.5, False),
        "wrong-size spare in the holder": (scene(**{blown: in_scrap, WRONG_SPARE: fuse(WRONG_SPARE, RAIL_X, SLOT_Y[3], FLOOR)}), 0.25, False),
        "wrong-size spare leaning in the holder": (scene(**{blown: in_scrap, WRONG_SPARE: fuse(WRONG_SPARE, RAIL_X + 0.004, SLOT_Y[3], FLOOR, 15)}), 0.25, False),
        "done but fuse 2 pushed 4 mm": (scene(**{blown: in_scrap, RIGHT_SPARE: seated, "fuse_2": fuse("fuse_2", RAIL_X, SLOT_Y[2] + 0.004, FLOOR)}), 0.25, False),
        "done but fuse 4 tipped": (scene(**{blown: in_scrap, RIGHT_SPARE: seated, "fuse_4": fuse("fuse_4", RAIL_X, SLOT_Y[4], FLOOR, 8)}), 0.25, False),
        "good fuse 1 pulled instead": (scene(fuse_1=fuse("fuse_1", SCRAP[0], SCRAP[1], TOP)), 0.0, False),
        "blown fuse still in, spare beside the rail": (scene(**{RIGHT_SPARE: fuse(RIGHT_SPARE, 0.36, 0.23, TOP)}), 0.0, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(scene(**{blown: in_scrap, RIGHT_SPARE: seated})) < 1e-9
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
