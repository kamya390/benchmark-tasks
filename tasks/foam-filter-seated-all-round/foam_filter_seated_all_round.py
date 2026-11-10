"""On the station table, in front of the right arm, a small engine's air-cleaner base stands on a dark engine block
stand-in: a black moulded housing with an open pocket 80 x 60 mm and 25 mm deep. A clean foam filter element (open-
cell polyurethane foam, orange, 86 x 66 x 22 mm) has just been laid on top of it. As on a real engine, the element
is cut 3 mm oversize on every side so its edges seal against the housing walls, so it does not drop in: its middle
sags into the pocket under its own weight while its edges ride on the rim.

The element is a real soft body (a PhysX FEM volume, about 265 mesh nodes, Young's modulus 30 kPa), not a rigid part. A
fingertip press near an edge (a few newtons) squeezes that edge past the rim and down the wall; pressed in that way
all round, the whole element goes home, its top 3-4 mm below the rim. A press in the middle only dishes it: the
element cocks, one edge stays riding on the rim, and the cover could not seal over it.

The job: seat the element fully in the housing, edges tucked in all round, and let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/foam-filter-seated-all-round/foam_filter_seated_all_round.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (per-episode memory and the grippers' TCPs)
import softkit as sk  # noqa: E402  (the soft foam element: PhysX FEM body, read as mesh nodes)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: log sectors, proud height, node speed and TCPs every 10 steps to ~/dbg/b17-soft.jsonl

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
HX, HY = 0.300, -0.100  # pocket centre
BLOCK = (0.160, 0.200, 0.030)  # engine block stand-in under the air-cleaner base
PLATE = 0.004  # housing floor plate
FLOOR = TOP + BLOCK[2] + PLATE  # pocket floor
IN = (0.080, 0.060)  # pocket interior (x, y)
WALL, DEPTH = 0.006, 0.025
RIM = FLOOR + DEPTH
ELEMENT = (0.086, 0.066, 0.022)  # 3 mm oversize each side
BLACK, ENGINE, FOAM_C = (0.08, 0.08, 0.09), (0.30, 0.31, 0.33), (0.95, 0.55, 0.15)

FOAM = sk.Soft("element", pos=(HX, HY, RIM + ELEMENT[2] / 2 + 0.0005), size=ELEMENT, kind="block", color=FOAM_C,
               youngs=3.0e4, poisson=0.2, density=80.0, friction=1.0, refine=12.0)


def _housing():
    out = [Box("engine_block", pos=(HX, HY, TOP + BLOCK[2] / 2), size=BLOCK, color=ENGINE),
           Box("base_floor", pos=(HX, HY, FLOOR - PLATE / 2), size=(IN[0] + 2 * WALL, IN[1] + 2 * WALL, PLATE),
               color=BLACK)]
    zc = FLOOR + DEPTH / 2
    for s in (-1, 1):
        out.append(Box(f"base_wall_x{'np'[s > 0]}", pos=(HX + s * (IN[0] / 2 + WALL / 2), HY, zc),
                       size=(WALL, IN[1] + 2 * WALL, DEPTH), color=BLACK))
        out.append(Box(f"base_wall_y{'np'[s > 0]}", pos=(HX, HY + s * (IN[1] / 2 + WALL / 2), zc),
                       size=(IN[0], WALL, DEPTH), color=BLACK))
    return out


OBJECTS = [*_housing(), FOAM]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring, from the element's mesh nodes ---------------------------------------------------------------------
BAND_IN = 0.012  # a sector's band reaches this far in from its wall ...
BAND_OUT = WALL + 0.010  # ... and this far out past it (over the rim and beyond)
SEAT_TOL = 0.001  # a seated edge sits at least 1 mm below the rim
WALL_TOL = 0.0005
PROUD_TOL = 0.001  # nothing may stand more than 1 mm above the rim
OFF_Z = FLOOR - 0.005  # any node this low has left the housing (dropped on the block or the table)
CLEAR = 0.03  # let go: both TCPs at least 3 cm from every node
STILL = 0.02  # m/s, every node (FEM nodes jitter up to ~1.2 cm/s at rest)
M = "ffsar"  # this task's key in the per-episode memory
_TEST_TCPS = None  # self-test stand-in for the TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"dropped": False}
    return m


def sectors(objects) -> list[bool]:
    """12 perimeter sectors (3 per wall). A sector is seated when every node in its band (12 mm in from the wall to
    16 mm out past it) is inside the wall and at least 1 mm below the rim, and the band holds at least one node."""
    q = sk.nodes(objects["element"]) - np.array([HX, HY, 0.0])
    out = []
    for ax, other in ((0, 1), (1, 0)):
        h, ho = IN[ax] / 2, IN[other] / 2
        for s in (1, -1):
            d = s * q[:, ax]
            band = (d > h - BAND_IN) & (d < h + BAND_OUT)
            for k in range(3):
                a, b = -ho + 2 * ho * k / 3, -ho + 2 * ho * (k + 1) / 3
                m = band & (q[:, other] >= a) & (q[:, other] <= b)
                out.append(bool(m.any() and np.all((d[m] <= h + WALL_TOL) & (q[m, 2] <= RIM - SEAT_TOL))))
    return out


def proud(objects) -> float:
    """How far the element's highest node stands above the rim (m)."""
    return float(sk.nodes(objects["element"])[:, 2].max() - RIM)


def released(objects) -> bool:
    tc = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    return sk.clear_of_tcps(objects["element"], CLEAR, tc)


def score(objects) -> float:
    """0 once the element has left the housing (latched). Otherwise 0.25 with 6 of 12 edge sectors seated, 0.5 with
    10, 1.0 with all 12 seated and nothing above the rim. Halved unless both grippers are 3 cm clear of the element
    and every node is still (under 2 cm/s, measured from node displacement between control steps)."""
    m = _mem()
    if float(sk.nodes(objects["element"])[:, 2].min()) < OFF_Z:
        m["dropped"] = True
    if m["dropped"]:
        return 0.0
    n = sum(sectors(objects))
    base = 1.0 if (n == 12 and proud(objects) <= PROUD_TOL) else 0.5 if n >= 10 else 0.25 if n >= 6 else 0.0
    ok = released(objects) and sk.motion(objects["element"], "element") < STILL
    if DEBUG and mk._EP.get("env") is not None and mk._EP.get("step", 0) % 10 == 0:
        _debug(objects, n, base, ok)
    return round(base if ok else base / 2, 6)


def _debug(objects, n, base, ok) -> None:
    import json
    import time

    p = sk.nodes(objects["element"])
    rec = {"k": mk._EP.get("step"), "t": round(time.time(), 3), "n": n, "sec": "".join("S" if s else "-" for s in
           sectors(objects)), "proud_mm": round(proud(objects) * 1000, 1), "bottom_mm": round((p[:, 2].min() - FLOOR)
           * 1000, 1), "vmax": round(sk.max_node_speed(objects["element"]), 4), "motion": round(sk.motion(objects["element"], "element"), 4), "base": base, "ok": ok,
           "nodes": len(p), "tcps": [np.round(t, 4).tolist() for t in mk.tcps()]}
    with open(str(Path.home() / "dbg" / "b17-soft.jsonl"), "a") as fh:
        fh.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    """Seated all round, nothing proud, let go and still; held for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Unseated sectors plus how far the element stands proud (m)."""
    return float(12 - sum(sectors(objects)) + max(0.0, proud(objects)))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry; right arm only, the left stays parked. Fingers closed, the pad tips (13 mm
# below the TCP) press the element 8 mm in from each wall, 3 points per side, working round the perimeter from the
# far short side; each press goes down to 16 mm above the pocket floor (the arm's compliance limits it to a few
# newtons once the foam resists), holds 0.5 s and lifts. Closed pads are 12.8 mm thick across the closing axis and
# 26 mm wide: at yaw 0 they close along y, so the thin direction faces the long walls; for the short walls the hand
# turns 90 deg.

TIP = 0.013
IN_FROM_WALL = 0.008
PRESS_Z = FLOOR + 0.016
OVER = RIM + 0.030
PARK = (0.200, -0.250, TOP + 0.160)
YAW_X = math.pi / 2  # at yaw 0 the fingers close along y, so the 26 mm pad width runs along x
NAIVE = False  # True: the naive policy, five presses in the middle (sim check)


def press_points():
    """The 12 press points (x, y, yaw), round the perimeter from the far short side."""
    far = [(HX + IN[0] / 2 - IN_FROM_WALL, HY + y, YAW_X) for y in (0.016, 0.0, -0.016)]
    right = [(HX + x, HY - IN[1] / 2 + IN_FROM_WALL, 0.0) for x in (0.026, 0.0, -0.026)]
    near = [(HX - IN[0] / 2 + IN_FROM_WALL, HY + y, YAW_X) for y in (-0.016, 0.0, 0.016)]
    left = [(HX + x, HY + IN[1] / 2 - IN_FROM_WALL, 0.0) for x in (-0.026, 0.0, 0.026)]
    return far + right + near + left


def demo():
    """Press the element's edges in all round, 12 presses, then back away."""
    c = FINGER_CLOSED
    R = [Waypoint(2.0, (HX, HY, OVER + 0.02 + TIP), c, "over the element, fingers closed", 0.0)]

    def w(dur, pos, label, yaw=0.0):
        R.append(Waypoint(dur, tuple(float(v) for v in pos), c, label, yaw))

    pts = [(HX, HY, 0.0)] * 5 if NAIVE else press_points()
    for i, (x, y, yaw) in enumerate(pts):
        w(0.8, (x, y, OVER + TIP), f"press {i + 1}: over", yaw)
        w(1.5, (x, y, PRESS_Z + TIP), f"press {i + 1}: down", yaw)
        w(0.5, (x, y, PRESS_Z + TIP), f"press {i + 1}: hold", yaw)
        w(0.7, (x, y, OVER + TIP), f"press {i + 1}: up", yaw)
    w(1.0, (HX, HY, OVER + 0.04 + TIP), "up, clear of the element")
    w(2.0, PARK, "park")
    total = sum(x.duration for x in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park")]
    return {"left": left, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built node sets (one trial each, fed frame by frame)."""
    global _TEST_TCPS
    rest = sk.rest_nodes(FOAM)
    c0 = np.array([HX, HY, 0.0])

    def st(p, moving=False):
        v = np.zeros_like(p)
        if moving:
            v[:, 2] = -0.05
        return {"element": sk.fake(FOAM, p, v)}

    def seated_nodes():  # squeezed 3 mm a side into the pocket, bottom on the floor
        p = rest - c0
        p[:, 0] *= (IN[0] - 0.001) / ELEMENT[0]
        p[:, 1] *= (IN[1] - 0.001) / ELEMENT[1]
        p[:, 2] += FLOOR + ELEMENT[2] / 2 - FOAM.pos[2]
        return p + c0

    seated = seated_nodes()

    def edge_up(p, side):  # one side's edge left riding on the rim (the rest seated)
        q = p.copy()
        ax, s = {"x+": (0, 1), "x-": (0, -1), "y+": (1, 1), "y-": (1, -1)}[side]
        m = s * (q[:, ax] - c0[ax]) > IN[ax] / 2 - 0.016
        q[m, ax] = c0[ax] + s * (s * (q[m, ax] - c0[ax]) + 0.0035)
        q[m, 2] = rest[m, 2]
        return q

    def corner_up(p):  # the far right corner left riding on the rim
        q = p.copy()
        m = (q[:, 0] - HX > IN[0] / 2 - 0.012) & (HY - q[:, 1] > IN[1] / 2 - 0.012)
        q[m, 0] += 0.0035
        q[m, 1] -= 0.0035
        q[m, 2] = rest[m, 2]
        return q

    def dished(p):  # centre pressed down to the floor, edges still on the rim all round
        q = rest.copy()
        r = np.maximum(np.abs(q[:, 0] - HX) / (ELEMENT[0] / 2), np.abs(q[:, 1] - HY) / (ELEMENT[1] / 2))
        q[:, 2] -= (1 - r ** 2) * (rest[:, 2].min() - FLOOR)
        return q

    def lin(a, b, n=6):
        return [a + (b - a) * k / n for k in range(n + 1)]

    two_sides = edge_up(edge_up(seated, "x+"), "y-")
    on_table = rest + np.array([0.0, 0.12, TOP + ELEMENT[2] / 2 - FOAM.pos[2]])
    tcp_on = [(HX, HY, RIM + TIP)]
    cases = {  # name: (frames, TCPs at the end, score)
        "start: laid on the rim": ([rest], [], 0.0),
        "edges pressed in all round, let go (demo)": (lin(rest, seated), [], 1.0),
        "seated, finger still on it": (lin(rest, seated), tcp_on, 0.5),
        "seated but still moving": (lin(rest, seated), [], 0.5),
        "centre pressed: dished, edges riding on the rim": (lin(rest, dished(rest)), [], 0.0),
        "one long edge left on the rim (its corners too)": (lin(rest, edge_up(seated, "y-")), [], 0.25),
        "one short edge left on the rim (its corners too)": (lin(rest, edge_up(seated, "x+")), [], 0.25),
        "one corner left on the rim": (lin(rest, corner_up(seated)), [], 0.5),
        "two edges left on the rim": (lin(rest, two_sides), [], 0.0),
        "seated, then pulled out and dropped beside the housing": (lin(rest, seated) + [on_table], [], 0.0),
        "dropped, then put back seated (latched)": ([rest, on_table, seated], [], 0.0),
        "seated but squashed proud in the middle (2 mm over the rim)":
            ([rest, seated + np.where((np.abs(seated[:, :1] - HX) < 0.01) & (seated[:, 2:] > RIM - 0.005),
                                      [0, 0, 0.0055], [0, 0, 0])], [], 0.5),
    }
    try:
        for name, (frames, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for f in frames[:-1]:
                score(st(f))
            _TEST_TCPS = tcps
            moving = name.startswith("seated but still moving")
            got = score(st(frames[-1], moving=moving))
            assert abs(got - want) < 1e-9, (name, got, want, sum(sectors(st(frames[-1]))), proud(st(frames[-1])))
            assert success(st(frames[-1], moving=moving)) == (want >= 1.0), name
        mk.new_episode()
        _TEST_TCPS = []
        score(st(on_table))
        mk.new_episode()
        assert score(st(seated)) == 1.0, "a new trial forgets the last one's drop"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built node sets, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n = self_test()
    print(f"scoring: {n} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
