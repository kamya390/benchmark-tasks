"""On the station table, in front of the left arm, a strip of cold-aisle raised floor in front of three racks. The
racks are fixed open-top cabinets further out along x: rack A (left) and rack M (middle) are loaded with servers, rack
B (right) is empty. In front of each rack a floor tile rests on its square pedestal, with finger gaps along its front
and back edges: a solid dark tile in front of rack A, perforated light tiles in front of racks M and B. A perforated
tile lets cold air up in front of a rack's inlets, so it belongs in front of a loaded rack: the tiles at A and B must
be swapped. A white tile cart nearer the arms is the buffer spot. The middle tile is already right and must not be
touched. Scored from tile poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/perforated-tile-to-load/perforated_tile_to_load.py \\
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
TILE = (0.070, 0.070, 0.020)  # a floor tile at model scale: 70 mm square, 20 mm thick
TILE_KG = 0.200
PED = (0.060, 0.060, 0.008)  # each tile's pedestal: smaller than the tile so the tile overhangs it all round
PED_TOP = TOP + PED[2]
FLOOR_X = 0.330
SPOT_Y = {"A": 0.335, "M": 0.250, "B": 0.165}  # tile spots in front of racks A (left), M and B (right)
LOADED = ("A", "M")
RACK_X = 0.445
RACK_INNER = (0.070, 0.064)
RACK_T, RACK_H = 0.005, 0.050
SERVER = (0.066, 0.018, 0.040)  # three server chassis side by side in a loaded rack, seen from above
CART = ((0.170, 0.250), (0.110, 0.110))  # the tile cart (buffer spot): centre, size of its white deck
CART_H = 0.004

DARK = (0.22, 0.23, 0.25)  # solid tile
PERF = (0.80, 0.82, 0.84)  # perforated tile
STEEL = (0.55, 0.56, 0.58)
RACK_C = (0.08, 0.08, 0.09)
SERVER_C = (0.30, 0.32, 0.36)
LED = (0.15, 0.75, 0.30)
WHITE = (0.93, 0.93, 0.93)
SUBFLOOR = (0.62, 0.64, 0.66)


def _rack(name, y, loaded):
    ix, iy = RACK_INNER
    z = TOP + RACK_H / 2
    t = RACK_T
    parts = [
        Box(f"{name}_side_ypos", pos=(RACK_X, y + iy / 2 + t / 2, z), size=(ix + 2 * t, t, RACK_H), color=RACK_C),
        Box(f"{name}_side_yneg", pos=(RACK_X, y - iy / 2 - t / 2, z), size=(ix + 2 * t, t, RACK_H), color=RACK_C),
        Box(f"{name}_front", pos=(RACK_X - ix / 2 - t / 2, y, z), size=(t, iy, RACK_H), color=RACK_C),
        Box(f"{name}_back", pos=(RACK_X + ix / 2 + t / 2, y, z), size=(t, iy, RACK_H), color=RACK_C),
    ]
    if loaded:
        for j, dy in enumerate((-0.021, 0.0, 0.021)):
            parts.append(Box(f"{name}_server_{j}", pos=(RACK_X, y + dy, TOP + SERVER[2] / 2), size=SERVER, color=SERVER_C))
            parts.append(Box(f"{name}_server_{j}_led", pos=(RACK_X - SERVER[0] / 2 + 0.006, y + dy, TOP + SERVER[2] + 0.001),
                             size=(0.006, 0.008, 0.002), color=LED))
    return parts


TILES = [
    Box("solid_tile", pos=(FLOOR_X, SPOT_Y["A"], PED_TOP + TILE[2] / 2), size=TILE, mass=TILE_KG, color=DARK),
    Box("perforated_tile_m", pos=(FLOOR_X, SPOT_Y["M"], PED_TOP + TILE[2] / 2), size=TILE, mass=TILE_KG, color=PERF),
    Box("perforated_tile_b", pos=(FLOOR_X, SPOT_Y["B"], PED_TOP + TILE[2] / 2), size=TILE, mass=TILE_KG, color=PERF),
]
FIXTURES = [
    Box("subfloor", pos=(FLOOR_X, 0.250, TOP + 0.001), size=(0.100, 0.270, 0.002), color=SUBFLOOR),
    *(Box(f"pedestal_{s}", pos=(FLOOR_X, y, TOP + PED[2] / 2), size=PED, color=STEEL) for s, y in SPOT_Y.items()),
    *(p for s, y in SPOT_Y.items() for p in _rack(f"rack_{s}", y, s in LOADED)),
    Box("tile_cart", pos=(*CART[0], TOP + CART_H / 2), size=(*CART[1], CART_H), color=WHITE),
]
OBJECTS = [*TILES, *FIXTURES]
MIDDLE = TILES[1]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_XY = 0.006  # seated: centre within 6 mm of the spot, flat within 3 degrees, square within 5, bottom on the pedestal
SEAT_Z = 0.002
FLAT_DEG, SQUARE_DEG = 3.0, 5.0
KEEP_MOVE, KEEP_DEG = 0.004, 4.0  # the middle tile counts as untouched within 4 mm and 4 degrees


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical, tilt of the up axis from the vertical), degrees."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    """A square tile is square again every 90 degrees."""
    return abs((yaw_deg + 45.0) % 90.0 - 45.0)


def _seated(st, spot) -> bool:
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(
        math.hypot(float(st.pos[0]) - FLOOR_X, float(st.pos[1]) - SPOT_Y[spot]) <= SEAT_XY
        and abs(float(st.low[2]) - PED_TOP) <= SEAT_Z
        and tilt <= FLAT_DEG
        and _square(yaw) <= SQUARE_DEG
        and st.resting()
    )


def _untouched(objects) -> bool:
    st = objects[MIDDLE.name]
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    return math.dist([float(v) for v in st.pos], MIDDLE.pos) <= KEEP_MOVE and _square(yaw) <= KEEP_DEG and tilt <= KEEP_DEG


def score(objects) -> float:
    """0.5 for the perforated tile from rack B's spot seated in front of loaded rack A, 0.5 for the solid tile seated
    in front of empty rack B. The middle tile moved more than 4 mm or 4 degrees caps the score at 0.25: rack M is
    loaded and its perforated tile is already right."""
    s = 0.5 * _seated(objects["perforated_tile_b"], "A") + 0.5 * _seated(objects["solid_tile"], "B")
    if not _untouched(objects):
        s = min(s, 0.25)
    return float(s)


def success(objects) -> bool:
    """Perforated tile in front of rack A, solid tile in front of rack B, the middle tile untouched; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: each swapped tile's centre to its new spot."""
    p, s = objects["perforated_tile_b"], objects["solid_tile"]
    d = math.dist((float(p.pos[0]), float(p.pos[1]), float(p.low[2])), (FLOOR_X, SPOT_Y["A"], PED_TOP))
    d += math.dist((float(s.pos[0]), float(s.pos[1]), float(s.low[2])), (FLOOR_X, SPOT_Y["B"], PED_TOP))
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW = math.pi / 2  # the fingers close along x onto the tile's front and back edges, in its finger gaps
TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
WIDE = -(TILE[0] + 0.016) / 2  # open to 86 mm: the pads clear the tile by 8 mm, the rack fronts by 17 mm
TIP_ABOVE = 0.006  # the pad tips 6 mm above the tile's bottom: 14 mm of pad on each edge
X_TRIM = 0.004  # a carried 200 g tile trails the commanded x; with this trim it lands 1-2 mm long (measured)


def _grip_z(bottom):
    return bottom + TIP_ABOVE + TIP


def _move(pick, place, label, trim=0.0):
    """Top-down across the tile's 70 mm, settling before every close and release and making the last 2 cm slowly. 18.5 s."""
    (px, py, pb), (qx, qy, qb) = pick, place
    qx += trim
    gz, qz = _grip_z(pb), _grip_z(qb) + 0.0015  # set down 1.5 mm above the surface
    grip = -(TILE[0] - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), WIDE, f"{label}: over", YAW),
        Waypoint(1.8, (px, py, gz + 0.020), WIDE, f"{label}: descend", YAW),
        Waypoint(1.0, (px, py, gz), WIDE, f"{label}: down slowly", YAW),
        Waypoint(0.5, (px, py, gz), WIDE, f"{label}: settle", YAW),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", YAW),
        Waypoint(1.5, (px, py, gz + 0.030), grip, f"{label}: lift straight up", YAW),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", YAW),
        Waypoint(2.2, (qx, qy, TRAVEL), grip, f"{label}: carry", YAW),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", YAW),
        Waypoint(1.8, (qx, qy, qz + 0.020), grip, f"{label}: lower", YAW),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set down slowly", YAW),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", YAW),
        Waypoint(1.5, (qx, qy, qz), WIDE, f"{label}: release", YAW),
        Waypoint(1.0, (qx, qy, TRAVEL), WIDE, f"{label}: retreat", YAW),
    ]


def demo():
    """Left arm only: the solid tile from rack A's spot onto the cart, the perforated tile from
    rack B's spot into rack A's, then the solid tile from the cart into rack B's spot. 55.5 s."""
    cart = (*CART[0], TOP + CART_H)
    a, b = (FLOOR_X, SPOT_Y["A"], PED_TOP), (FLOOR_X, SPOT_Y["B"], PED_TOP)
    plan = _move(a, cart, "solid tile to the cart")
    plan += _move(b, a, "perforated tile to loaded rack A", trim=X_TRIM)
    plan += _move(cart, b, "solid tile to empty rack B", trim=X_TRIM)
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def tile(x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        x2, y2, z2, w2 = math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array(TILE) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def spot(s, dx=0.0, dy=0.0, dz=0.0, **kw):
        return tile(FLOOR_X + dx, SPOT_Y[s] + dy, PED_TOP + dz, **kw)

    cart = tile(*CART[0], TOP + CART_H)
    start = {"solid_tile": spot("A"), "perforated_tile_m": spot("M"), "perforated_tile_b": spot("B")}
    s1 = start | {"solid_tile": cart}
    s2 = s1 | {"perforated_tile_b": spot("A", dx=-0.004)}
    s3 = s2 | {"solid_tile": spot("B", dx=-0.003, dy=0.002, yaw_deg=2.0)}
    both_up = start | {"solid_tile": cart, "perforated_tile_b": tile(0.17, 0.10, TOP)}

    def run(*frames):
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "solid tile on the cart": ((start, s1), 0.0, False),
        "perforated tile moved to A": ((start, s1, s2), 0.5, False),
        "done, one opening at a time": ((start, s1, s2, s3), 1.0, True),
        "done, solid tile turned 90 degrees": ((start, s1, s2, s3 | {"solid_tile": spot("B", yaw_deg=90)}), 1.0, True),
        "both tiles on the table": ((start, both_up), 0.0, False),
        "perforated tile held 3 cm over A": ((start, s1, s1 | {"perforated_tile_b": spot("A", dz=0.03)}), 0.0, False),
        "middle perforated tile moved to A instead": ((start, s1, s1 | {"perforated_tile_m": spot("A")}), 0.0, False),
        "done but the middle tile nudged 5 mm": ((start, s1, s2, s3 | {"perforated_tile_m": spot("M", dx=-0.005)}), 0.25, False),
        "perforated tile 9 mm off its spot": ((start, s1, s1 | {"perforated_tile_b": spot("A", dx=-0.009)}), 0.0, False),
        "perforated tile tilted on the pedestal edge": ((start, s1, s1 | {"perforated_tile_b": spot("A", dz=0.003, tilt_deg=6)}), 0.0, False),
        "solid tile at B still moving": ((start, s1, s2, s2 | {"solid_tile": spot("B", vel=(0, 0, -0.05))}), 0.5, False),
        "solid tile stacked on the perforated tile at A": ((start, s1, s2, s2 | {"solid_tile": spot("A", dz=TILE[2])}), 0.5, False),
        "swapped the wrong way: solid at A, perforated at B": ((start, start), 0.0, False),
        "done, middle tile nudged 2 mm": ((start, s3 | {"perforated_tile_m": spot("M", dy=0.002)}), 1.0, True),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(s3) < 0.01 and distance(start) > 0.3
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
