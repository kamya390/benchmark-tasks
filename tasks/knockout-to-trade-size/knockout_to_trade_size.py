"""On the station table, in front of the left arm, stands a steel junction box (20 x 20 cm, 10 cm tall). Its top
has one concentric knockout, all still closed and flush with the top: a centre slug (5 cm square), an inner ring
(opening it to 11 cm) and an outer ring (opening it to 17 cm), each held in by small break-off tabs. A black
conduit connector stands on a rubber mat nearer the arms: a square body that has to go through the opening, a
flange that has to bear on the box top round it, and a hub on top where the conduit enters. The job is to fit the
connector into the knockout (scaled about 3x for the gripper).

The connector's body (9 cm) is too big for the slug's opening (5 cm) and passes the inner ring's (11 cm); its
flange (14 cm) bears on the outer ring, and passes the outer ring's opening (17 cm). So the right opening is: slug
out, inner ring out, outer ring left in. An opening knocked out too far cannot be closed again (in practice it
needs a reducing washer and spoils bonding through the box), so the trial scores 0 for good once the outer ring is
pushed out. Knockouts are punched out with a tool (here the closed fingers), not with the fitting: a knockout that
comes out while the connector is pressed on it also scores 0 for good.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/knockout-to-trade-size/knockout_to_trade_size.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (breakable bonds, per-episode memory, gripper TCPs)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
BOX_XY = (0.330, 0.200)
BOX = (0.200, 0.200, 0.100)  # junction box outside size
WALL, FLOOR, PLATE_T = 0.008, 0.004, 0.003  # wall, floor and top-plate thickness
PLATE_TOP = TOP + BOX[2]
GAP = 0.002  # clearance round each knockout piece
SLUG = 0.050  # centre slug, square
RING1 = (SLUG + 2 * GAP, 0.110)  # inner ring: inner, outer size
RING2 = (RING1[1] + 2 * GAP, 0.170)  # outer ring
OPENING = RING2[1] + 2 * GAP  # the fixed top plate's own opening
KO_T = 0.003
PIECES = ("slug", "ring1", "ring2")
HOLD_N = {"slug": 10.0, "ring1": 15.0, "ring2": 15.0}  # push needed to break each piece's tabs
KO_KG = {"slug": 0.020, "ring1": 0.040, "ring2": 0.060}

BODY = (0.090, 0.090, 0.030)  # connector: body (goes through the opening) ...
FLANGE = (0.140, 0.140, 0.006)  # ... flange (bears on the box top) ...
HUB = (0.050, 0.050, 0.030)  # ... hub (the conduit end, where it is held)
CONN_KG = 0.080
MAT_XY = (0.170, 0.010)
MAT = (0.160, 0.160, 0.003)
MAT_TOP = TOP + MAT[2]
SEAT_BOTTOM = PLATE_TOP - BODY[2]  # the connector's bottom when its flange bears on the box top

GALV = (0.70, 0.72, 0.74)
KO = (0.62, 0.64, 0.67)
BLACK = (0.07, 0.07, 0.08)
RUBBER = (0.20, 0.30, 0.22)

kit = mk.Kit()
_kz = PLATE_TOP - KO_T / 2


def _frame(inner, outer):
    w, c = (outer - inner) / 2, (outer + inner) / 4
    return [mk.box((outer, w, KO_T), at=(0, s * c, 0)) for s in (-1, 1)] + \
           [mk.box((w, inner, KO_T), at=(s * c, 0, 0)) for s in (-1, 1)]


kit.part("slug", (*BOX_XY, _kz), [mk.box((SLUG, SLUG, KO_T))], mass=KO_KG["slug"], color=KO)
kit.part("ring1", (*BOX_XY, _kz), _frame(*RING1), mass=KO_KG["ring1"], color=KO)
kit.part("ring2", (*BOX_XY, _kz), _frame(*RING2), mass=KO_KG["ring2"], color=KO)
for _p in PIECES:  # the break-off tabs: a push of HOLD_N frees the piece
    kit.bond(f"{_p}_tabs", None, _p, break_force=HOLD_N[_p])
kit.part("connector", (*MAT_XY, MAT_TOP),
         [mk.box(BODY, at=(0, 0, BODY[2] / 2)),
          mk.box(FLANGE, at=(0, 0, BODY[2] + FLANGE[2] / 2)),
          mk.box(HUB, at=(0, 0, BODY[2] + FLANGE[2] + HUB[2] / 2))], mass=CONN_KG, color=BLACK)


def _box_fixtures():
    cx, cy = BOX_XY
    hx, hy, h = BOX[0] / 2, BOX[1] / 2, BOX[2] - PLATE_T
    zt = PLATE_TOP - PLATE_T / 2
    strip = (BOX[0] - OPENING) / 2
    c = (BOX[0] + OPENING) / 4
    return [
        Box("box_floor", pos=(cx, cy, TOP + FLOOR / 2), size=(BOX[0], BOX[1], FLOOR), color=GALV),
        Box("box_wall_near", pos=(cx - hx + WALL / 2, cy, TOP + h / 2), size=(WALL, BOX[1], h), color=GALV),
        Box("box_wall_far", pos=(cx + hx - WALL / 2, cy, TOP + h / 2), size=(WALL, BOX[1], h), color=GALV),
        Box("box_wall_left", pos=(cx, cy + hy - WALL / 2, TOP + h / 2), size=(BOX[0] - 2 * WALL, WALL, h), color=GALV),
        Box("box_wall_right", pos=(cx, cy - hy + WALL / 2, TOP + h / 2), size=(BOX[0] - 2 * WALL, WALL, h), color=GALV),
        Box("top_near", pos=(cx - c, cy, zt), size=(strip, OPENING, PLATE_T), color=GALV),
        Box("top_far", pos=(cx + c, cy, zt), size=(strip, OPENING, PLATE_T), color=GALV),
        Box("top_left", pos=(cx, cy + c, zt), size=(BOX[0], strip, PLATE_T), color=GALV),
        Box("top_right", pos=(cx, cy - c, zt), size=(BOX[0], strip, PLATE_T), color=GALV),
    ]


FIXTURES = [*_box_fixtures(), Box("mat", pos=(*MAT_XY, TOP + MAT[2] / 2), size=MAT, color=RUBBER)]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
OUT_DROP = 0.008  # a piece is out once its top is 8 mm below the box top (fallen into the box)
FORCE_ABOVE = 0.015  # "pressed on": the connector's bottom within 1.5 cm of the box top ...
FORCE_XY = 0.060  # ... and its centre within 6 cm of the knockout's centre
SEAT_XY = 0.012


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _tilt(q) -> float:
    x, y, _, _ = (float(v) for v in q)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _xy_off(st) -> float:
    return math.hypot(float(st.pos[0]) - BOX_XY[0], float(st.pos[1]) - BOX_XY[1])


def _loose(objects, piece) -> bool:
    """The piece's tabs have let go (it moved 2 mm or 2 deg from its place; latched for the trial)."""
    return kit.broken(objects, f"{piece}_tabs")


def _out(objects, piece) -> bool:
    """The piece is out of the opening: loose and fallen, its top 8 mm below the box top."""
    return _loose(objects, piece) and float(objects[piece].high[2]) < PLATE_TOP - OUT_DROP


def _connector_on_opening(objects) -> bool:
    c = objects["connector"]
    return float(c.low[2]) < PLATE_TOP + FORCE_ABOVE and _xy_off(c) < FORCE_XY


def _update(objects) -> bool:
    """Latch the trial's faults; True once the opening was made too big or a knockout was forced by the fitting."""
    mem = mk.MEMORY
    for p in PIECES:
        key = ("ko_loose", p)
        if not mem.get(key) and _loose(objects, p):
            mem[key] = True
            if _connector_on_opening(objects):
                mem["ko_forced"] = True
    if _loose(objects, "ring2"):
        mem["ko_oversize"] = True
    return bool(mem.get("ko_oversize") or mem.get("ko_forced"))


def _seated(objects) -> bool:
    """The connector's body in the opening, its flange flat on the box top (bottom within -2/+3 mm of the seated
    height, under 5 deg), centred within 12 mm, still and let go."""
    c = objects["connector"]
    return bool(SEAT_BOTTOM - 0.002 < float(c.low[2]) < SEAT_BOTTOM + 0.003 and _tilt(c.quat_xyzw) < 5.0
                and _xy_off(c) < SEAT_XY and c.resting() and _released(c))


def score(objects) -> float:
    """1.0: slug and inner ring out, the connector seated, still and let go. 0.5: slug and inner ring out, the
    connector not seated. 0.25: only the slug out. 0 otherwise, and 0 for the rest of the trial once the outer ring
    has come loose (opening too big) or a knockout came loose while the connector was pressed on it."""
    if _update(objects):
        return 0.0
    slug, ring1 = _out(objects, "slug"), _out(objects, "ring1")
    if slug and ring1:
        return 1.0 if _seated(objects) else 0.5
    return 0.25 if slug else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: the connector's bottom centre to its seated position."""
    c = objects["connector"]
    return float(math.hypot(_xy_off(c), float(c.low[2]) - SEAT_BOTTOM))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Left arm only, open loop from the scene's fixed geometry. Punch the slug out with the closed fingers pressed on
# its centre; punch the inner ring out with the closed fingers (turned so their 20 mm side runs across the ring's
# 28 mm band) pressed on the band's inner edge; then lift the connector by its hub and set it into the opening.

TIP = 0.013
SQUEEZE = 0.008
PRESS = 0.025  # command the closed pads this far below the knockout's face (slug)
PRESS_RING = 0.045  # ... and the inner ring (pushed off-centre, it needs a firmer push)
TRAVEL = PLATE_TOP + 0.050  # punching travel height
CARRY = PLATE_TOP + 0.080  # carrying height: the connector's bottom 2.7 cm over the box top
RING1_PUSH_R = 0.038  # pad centre from the knockout centre on the inner ring (band 27..55 mm, pads +-10 mm)
HUB_GRIP_Z = BODY[2] + FLANGE[2] + 0.004 + TIP  # TCP above the connector's bottom: pad tips 4 mm over the flange
GRIP = -(HUB[1] - SQUEEZE) / 2
C, O = FINGER_CLOSED, FINGER_OPEN


def _punch(x, y, yaw, label, press=PRESS):
    face = PLATE_TOP + TIP
    return [
        Waypoint(1.8, (x, y, TRAVEL), C, f"{label}: over it, fingers closed", yaw),
        Waypoint(1.2, (x, y, face + 0.015), C, f"{label}: descend", yaw),
        Waypoint(0.8, (x, y, face + 0.002), C, f"{label}: onto the face", yaw),
        Waypoint(2.0, (x, y, face - press), C, f"{label}: press through", yaw),
        Waypoint(0.5, (x, y, face - press), C, f"{label}: hold", yaw),
        Waypoint(1.2, (x, y, TRAVEL), C, f"{label}: back up", yaw),
    ]


def demo():
    """Left arm: punch out the slug, punch out the inner ring at its inner edge, then carry the connector by its
    hub and lower it into the opening, letting go 3 mm above its seat."""
    bx, by = BOX_XY
    mx, my = MAT_XY
    pick_z = MAT_TOP + HUB_GRIP_Z
    place_z = SEAT_BOTTOM + 0.003 + HUB_GRIP_Z
    wps = [
        *_punch(bx, by, 0.0, "slug"),
        *_punch(bx + RING1_PUSH_R, by, math.pi / 2, "inner ring", PRESS_RING),
        Waypoint(1.8, (mx, my, CARRY), O, "over the connector", 0.0),
        Waypoint(1.5, (mx, my, pick_z + 0.020), O, "descend", 0.0),
        Waypoint(0.8, (mx, my, pick_z), O, "down slowly", 0.0),
        Waypoint(0.5, (mx, my, pick_z), O, "settle", 0.0),
        Waypoint(1.2, (mx, my, pick_z), GRIP, "close on the hub", 0.0),
        Waypoint(0.5, (mx, my, pick_z), GRIP, "settle closed", 0.0),
        Waypoint(1.5, (mx, my, CARRY), GRIP, "lift", 0.0),
        Waypoint(2.0, (bx, by, CARRY), GRIP, "carry over the box", 0.0),
        Waypoint(0.6, (bx, by, CARRY), GRIP, "settle over", 0.0),
        Waypoint(1.5, (bx, by, place_z + 0.020), GRIP, "lower", 0.0),
        Waypoint(1.5, (bx, by, place_z), GRIP, "into the opening slowly", 0.0),
        Waypoint(0.6, (bx, by, place_z), GRIP, "settle", 0.0),
        Waypoint(1.2, (bx, by, place_z), O, "release", 0.0),
        Waypoint(1.2, (bx, by, CARRY), O, "retreat", 0.0),
    ]
    return {"left": wps}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of states (one trial each)."""
    global _TEST_TCPS
    bx, by = BOX_XY
    flat = (0.0, 0.0, 0.0, 1.0)
    fallen = {"slug": ((bx, by, TOP + FLOOR + KO_T / 2), flat),
              "ring1": ((bx + 0.01, by, TOP + FLOOR + KO_T + KO_T / 2), flat),
              "ring2": ((bx, by, TOP + FLOOR + 2 * KO_T + KO_T / 2), flat)}
    seated_conn = ((bx, by, SEAT_BOTTOM), flat)

    def st(out=(), conn=None, **moved):
        m = {p: fallen[p] for p in out}
        if conn is not None:
            m["connector"] = conn
        m.update(moved)
        return kit.fake_states(moved=m)

    start = st()
    held_up = ((bx, by, PLATE_TOP + 0.08), flat)
    slug_pushed = st(slug=((bx, by, PLATE_TOP - KO_T / 2 - 0.004), flat))  # tabs gone, not yet fallen
    slug_out = st(out=("slug",))
    both_out = st(out=("slug", "ring1"))
    over_box = st(out=("slug", "ring1"), conn=held_up)
    done = st(out=("slug", "ring1"), conn=seated_conn)
    on_closed = st(conn=((bx, by, PLATE_TOP), flat))  # set on the closed knockout
    # the connector pressed down onto the closed knockout punches the slug and ring under its body
    forcing = st(conn=((bx, by, PLATE_TOP - 0.004), flat), slug=((bx, by, PLATE_TOP - KO_T / 2 - 0.004), flat),
                 ring1=((bx, by, PLATE_TOP - KO_T / 2 - 0.004), flat))
    forced_in = st(out=("slug", "ring1"), conn=seated_conn)
    all_out = st(out=("slug", "ring1", "ring2"))
    fell_in = st(out=("slug", "ring1", "ring2"), conn=((bx, by, TOP + FLOOR + 3 * KO_T), flat))
    ring1_only = st(out=("ring1",))
    tilted = st(out=("slug", "ring1"), conn=((bx, by, SEAT_BOTTOM + 0.004),
                                             tuple(mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(9.0))))))
    off_centre = st(out=("slug", "ring1"), conn=((bx + 0.009, by + 0.0, SEAT_BOTTOM), flat))
    hub_tcp = np.array([bx, by, SEAT_BOTTOM + HUB_GRIP_Z])

    cases = {  # name: (frames in one trial, TCPs at the end, score, success)
        "start": ([start], [], 0.0, False),
        "connector set on the closed knockout": ([start, st(conn=held_up), on_closed], [], 0.0, False),
        "slug loose but not yet fallen": ([start, slug_pushed], [], 0.0, False),
        "slug out": ([start, slug_pushed, slug_out], [], 0.25, False),
        "slug and inner ring out": ([start, slug_out, both_out], [], 0.5, False),
        "both out, connector held over the box": ([start, slug_out, both_out, over_box], [], 0.5, False),
        "both out, connector seated and let go": ([start, slug_out, both_out, over_box, done], [], 1.0, True),
        "seated but still gripped": ([start, slug_out, both_out, over_box, done], [hub_tcp], 0.5, False),
        "seated off centre by 9 mm (inside the tolerance)": ([start, slug_out, both_out, off_centre], [], 1.0, True),
        "connector tipped 9 deg on the edge": ([start, slug_out, both_out, tilted], [], 0.5, False),
        "outer ring punched too: opening too big": ([start, slug_out, both_out, all_out], [], 0.0, False),
        "too big, connector falls into the box": ([start, slug_out, both_out, all_out, fell_in], [], 0.0, False),
        "connector forced through the closed knockout": ([start, on_closed, forcing, forced_in], [], 0.0, False),
        "inner ring punched, slug still in": ([start, ring1_only], [], 0.0, False),
        "inner ring first, then the slug, then seated": ([start, ring1_only, both_out, done], [], 1.0, True),
        "seated, then picked back out and held": ([start, slug_out, both_out, done, over_box], [], 0.5, False),
    }
    try:
        for name, (frames, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for f in frames[:-1]:
                score(f)
            _TEST_TCPS = tcps
            got, got_success = score(frames[-1]), success(frames[-1])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        # a new trial forgets the last one's oversize fault
        mk.new_episode()
        _TEST_TCPS = []
        score(start)
        score(all_out)
        mk.new_episode()
        assert score(start) == 0.0 and score(both_out) == 0.5, "reset"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through
    # the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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
