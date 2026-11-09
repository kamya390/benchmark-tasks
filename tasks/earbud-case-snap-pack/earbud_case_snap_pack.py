"""On the station table, in front of the right arm, a black charging case for a pair of wireless earbuds stands in
a moulded packing nest with its lid open; a clear moulded retail tray waits nearer the robot. Both white earbuds
are already in their own wells, but each sits proud: set in loosely by the previous station, its head 4.5 mm
higher than home. Each well has a sprung catch (a detent, no magnet): a bud only goes home when its head is pushed
straight down the well with about 2.5 N, and in the last 1.5 mm the catch snaps it home and holds it.

The case lid (hinged along the rear top edge, friction hinge, it stays where it is left) has an inner recess that
clears seated heads by only 0.6 mm, so a bud that is not home holds the lid ajar. Its snap catch at the front
lets the lid go fully shut only under a firm press (about 1.5 N at the front edge), and then holds it.

Physics punishes the wrong order. A closing lid's front lip first meets a proud head on its front edge, at about
35 deg open, and drives it down and forward at that slant (with the lid as a lever, a light press on the lid would
be ample to push the bud in). A bud driven down by the lid binds in its well (the catch is made to engage only
along the well's axis; pushed askew it cocks and jams): it stops for good, still proud, and the lid can no
longer shut. So "set it in loosely and press the lid down", which on a lever would seat the bud through the lid,
leaves a jammed bud and a lid stuck ajar.

The job: snap both buds home, shut the lid until its catch holds, and set the closed case in its tray pocket.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/earbud-case-snap-pack/earbud_case_snap_pack.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (sprung sliders, friction hinge, per-step rules, per-episode memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the mechanism's events and a readout every 10 steps

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the charging case (1:1 with a stem-style earbud case) --------------------------------------------------------
CX, CY = 0.300, -0.100  # case centre, in the packing nest
PLINTH = 0.010  # the nest's moulded base
ZB = TOP + PLINTH  # case bottom
BODY = (0.024, 0.054, 0.034)  # depth (x), width along the hinge (y), height
RIM = ZB + BODY[2]
CUP_D = 0.008  # the head cups in the body's top
CUP_X = (-0.0085, 0.0095)  # cup extent in x (case frame)
BUD_Y = 0.012  # the wells' offset from the centre line, each side
HEAD = (0.016, 0.017, 0.014)  # bud head (x, y, z)
STEM = (0.007, 0.007, 0.028)
HEAD_X = 0.0005  # head centre in x (case frame)
HEAD_UP = 0.0084  # a seated head's top above the rim
PROUD = 0.0045  # start: how far each head sits above home
TRAVEL = 0.006  # slider travel (home at 0)
CLICK = 0.0015  # past this the catch snaps the bud home
RIDGE_N = 2.5  # push needed to drive a bud down its well (N)
LID_D, LID_W = 0.024, 0.054
LID_IN, LID_PLATE, SKIRT = 0.009, 0.0015, 0.0012  # recess height under the plate, plate, skirt walls
LID_TOP = LID_IN + LID_PLATE
HINGE = (CX + BODY[0] / 2, CY, RIM)  # lid hinge axis (along y) on the rear top edge
OPEN_DEG = 115.0
LID_FRICTION = 0.004  # N*m: the lid stays where it is left (its own weight: under 0.001 N*m)
CASE_KG, LID_KG, BUD_KG = 0.045, 0.008, 0.005
SIDES = ("l", "r")
BUD = {"l": "bud_l", "r": "bud_r"}
SEAT = {"l": "seat_l", "r": "seat_r"}
SIGN = {"l": 1.0, "r": -1.0}
PARTS = ("case", "lid", "bud_l", "bud_r")

BLACK, DARK, WHITE = (0.10, 0.10, 0.11), (0.20, 0.20, 0.22), (0.95, 0.95, 0.95)
NEST_C, TRAY_C, GREY = (0.30, 0.42, 0.62), (0.80, 0.86, 0.90), (0.55, 0.57, 0.60)

kit = mk.Kit()
_cup_w = 0.019
_frame_h, _frame_z = CUP_D, BODY[2] - CUP_D / 2
kit.part("case", (CX, CY, ZB), [
    mk.box((BODY[0], BODY[1], BODY[2] - CUP_D), at=(0, 0, (BODY[2] - CUP_D) / 2)),
    mk.box((CUP_X[0] + BODY[0] / 2, BODY[1], _frame_h), at=((CUP_X[0] - BODY[0] / 2) / 2, 0, _frame_z)),
    mk.box((BODY[0] / 2 - CUP_X[1], BODY[1], _frame_h), at=((CUP_X[1] + BODY[0] / 2) / 2, 0, _frame_z)),
    mk.box((CUP_X[1] - CUP_X[0], BODY[1] / 2 - BUD_Y - _cup_w / 2, _frame_h),
           at=(sum(CUP_X) / 2, (BODY[1] / 2 + BUD_Y + _cup_w / 2) / 2, _frame_z)),
    mk.box((CUP_X[1] - CUP_X[0], BODY[1] / 2 - BUD_Y - _cup_w / 2, _frame_h),
           at=(sum(CUP_X) / 2, -(BODY[1] / 2 + BUD_Y + _cup_w / 2) / 2, _frame_z)),
    mk.box((CUP_X[1] - CUP_X[0], 2 * (BUD_Y - _cup_w / 2), _frame_h), at=(sum(CUP_X) / 2, 0, _frame_z)),
], mass=CASE_KG, color=BLACK)
# lid: reference pose = shut; frame origin on the hinge axis, the lid reaching forward (-x) from it
kit.part("lid", HINGE, [
    mk.box((LID_D, LID_W, LID_PLATE), at=(-LID_D / 2, 0, LID_IN + LID_PLATE / 2)),
    mk.box((SKIRT, LID_W, LID_IN), at=(-LID_D + SKIRT / 2, 0, LID_IN / 2)),
    mk.box((SKIRT, LID_W, LID_IN), at=(-SKIRT / 2, 0, LID_IN / 2)),
    mk.box((LID_D - 2 * SKIRT, SKIRT, LID_IN), at=(-LID_D / 2, LID_W / 2 - SKIRT / 2, LID_IN / 2)),
    mk.box((LID_D - 2 * SKIRT, SKIRT, LID_IN), at=(-LID_D / 2, -LID_W / 2 + SKIRT / 2, LID_IN / 2)),
    mk.box((0.0015, 0.012, 0.004), at=(-LID_D - 0.00075, 0, -0.001), color=GREY),  # catch tongue
], mass=LID_KG, color=DARK)
kit.hinge("lid_hinge", "case", "lid", anchor=HINGE, axis=(0, 1, 0), lower_deg=0.0, upper_deg=OPEN_DEG,
          start_deg=OPEN_DEG, friction=LID_FRICTION)
for _s in SIDES:  # bud frame origin = its head's top centre, at home
    kit.part(BUD[_s], (CX + HEAD_X, CY + SIGN[_s] * BUD_Y, RIM + HEAD_UP), [
        mk.box(HEAD, at=(0, 0, -HEAD[2] / 2)),
        mk.box(STEM, at=(0, 0, -HEAD[2] - STEM[2] / 2)),
    ], mass=BUD_KG, color=WHITE)
    kit.slider(SEAT[_s], "case", BUD[_s], axis=(0, 0, 1), lower=0.0, upper=TRAVEL, start=PROUD,
               friction=RIDGE_N)

# packing nest (fixed): a moulded plinth with a fitted pocket 0.75 mm larger than the case each way, 20 mm deep
NEST_IN, NEST_WALL, NEST_H = (BODY[0] + 0.0015, BODY[1] + 0.0015), 0.004, 0.020
# retail tray (fixed): clear PET, one pocket 3 mm larger than the case each way, 10 mm deep
TX, TY = 0.225, -0.190
TRAY_T = 0.004
ZT = TOP + TRAY_T  # pocket floor
TRAY_IN, TRAY_WALL, TRAY_H = (BODY[0] + 0.006, BODY[1] + 0.006), 0.004, 0.010


def _walls(name, cx, cy, inner, wall, h, z0, color):
    ix, iy = inner
    zc, out = z0 + h / 2, []
    for s in (-1, 1):
        out.append(Box(f"{name}_wx{'np'[s > 0]}", pos=(cx + s * (ix / 2 + wall / 2), cy, zc),
                       size=(wall, iy + 2 * wall, h), color=color))
        out.append(Box(f"{name}_wy{'np'[s > 0]}", pos=(cx, cy + s * (iy / 2 + wall / 2), zc), size=(ix, wall, h),
                       color=color))
    return out


FIXTURES = [
    Box("nest_plinth", pos=(CX, CY, TOP + PLINTH / 2), size=(0.040, 0.070, PLINTH), color=NEST_C),
    *_walls("nest", CX, CY, NEST_IN, NEST_WALL, NEST_H, ZB, NEST_C),
    Box("tray_base", pos=(TX, TY, TOP + TRAY_T / 2), size=(0.060, 0.090, TRAY_T), color=TRAY_C),
    *_walls("tray", TX, TY, TRAY_IN, TRAY_WALL, TRAY_H, ZT, TRAY_C),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the mechanism's per-step rules (sim) and the scoring ---------------------------------------------------------
SEATED_TOL = 0.0004  # home: head within 0.4 mm of its seated height
LATCH_DEG = 0.5  # the lid's catch engages once the lid is pressed this far shut
RIDGE_DEG = 4.25  # below this the catch's ridge pushes back until it engages (about 1.5 N at the front edge)
SHUT_DEG = 1.0  # latched and shut
HOLD_DEG = -10.0  # latched: the catch pulls the lid onto its stop (drive target past the stop)
SNAP_M = -0.002  # clicked: the catch spring pulls the bud onto its home stop
CONTACT_TOL = 0.001  # a head corner within 1 mm of the lid's underside counts as touching
JAM_TRAVEL = 0.0003  # a bud driven 0.3 mm by the lid binds
PROUD_SHUT_DEG = 1.5  # lid this close to shut over a bud that is not home: latched fail (physics guard)
POCKET_TOL = 0.0035  # case bottom centre within 3.5 mm of the tray pocket's centre (it has 3 mm play)
FLOOR_TOL = 0.002
TILT_MAX = 4.0
M = "ecsp"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"clicked": {s: False for s in SIDES}, "jam": {s: None for s in SIDES},
                            "touch": {s: None for s in SIDES}, "prev": {s: PROUD for s in SIDES},
                            "latched": False, "proud_shut": False}
    return m


def bud_q(objects, s) -> float:
    """How far bud ``s``'s head stands above home (m), from the bud's and the case's poses."""
    return kit.q(objects, SEAT[s])


def lid_deg(objects) -> float:
    """How far the lid is open (deg; 0 = shut)."""
    return kit.q_deg(objects, "lid_hinge")


_HEAD_LO = np.array([-HEAD[0] / 2, -HEAD[1] / 2, -HEAD[2]])  # head box in the bud's frame
_HEAD_HI = np.array([HEAD[0] / 2, HEAD[1] / 2, 0.0])
_EDGES = [(a, b) for a in range(8) for b in range(a + 1, 8) if bin(a ^ b).count("1") == 1]


def _seg_hits_box(p0, p1, lo, hi) -> bool:
    """Does the segment p0-p1 pass through the axis-aligned box lo..hi? (slab test)"""
    t0, t1, d = 0.0, 1.0, p1 - p0
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if p0[i] < lo[i] or p0[i] > hi[i]:
                return False
            continue
        a, b = (lo[i] - p0[i]) / d[i], (hi[i] - p0[i]) / d[i]
        t0, t1 = max(t0, min(a, b)), min(t1, max(a, b))
        if t0 > t1:
            return False
    return True


def _contact(objects, s, tol) -> bool:
    """Is any part of the lid (plate, skirts, catch tongue) within ``tol`` of bud ``s``'s head? Every edge of every
    lid box is tested against the head box grown by ``tol``, and the head's top corners against the lid boxes."""
    tl, tb = kit._pose_T(objects, "lid"), kit._pose_T(objects, BUD[s])
    lid_to_bud = np.linalg.inv(tb) @ tl
    lo, hi = _HEAD_LO - tol, _HEAD_HI + tol
    bud_to_lid = np.linalg.inv(lid_to_bud)
    tops = [bud_to_lid[:3, :3] @ np.array([dx, dy, 0.0]) + bud_to_lid[:3, 3]
            for dx in (-HEAD[0] / 2, HEAD[0] / 2) for dy in (-HEAD[1] / 2, HEAD[1] / 2)]
    for b in kit.parts["lid"].boxes:
        half = np.asarray(b.size) / 2
        c = np.asarray(b.at)
        corners = [c + half * np.array([1 if i & 1 else -1, 1 if i & 2 else -1, 1 if i & 4 else -1]) for i in range(8)]
        cb = [lid_to_bud[:3, :3] @ v + lid_to_bud[:3, 3] for v in corners]
        if any(_seg_hits_box(cb[i], cb[j], lo, hi) for i, j in _EDGES):
            return True
        if any(np.all(np.abs(t - c) <= half + tol) for t in tops):
            return True
    return False


def touching(objects, s) -> bool:
    """Is the lid on bud ``s``'s head? (any lid box within 1 mm of the head box)"""
    return _contact(objects, s, CONTACT_TOL)


def _drive(name, target) -> None:
    """Sim only: hold joint ``name``'s friction/catch drive at ``target`` (rad or m) this step."""
    path = mk._SIM["joint_paths"].get((kit.id, name))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    j = kit.joints_[name]
    hinge = j.kind == "hinge"
    scale = 180.0 / math.pi if hinge else 1.0
    mk._SIM["stick"][(kit.id, name)] = target
    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "angular" if hinge else "linear")
    d.GetTargetPositionAttr().Set(float(target * scale))


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit ecsp] step {mk._EP.get('step')}: {msg}", flush=True)


def _rules(objects) -> None:
    """One control step of the catches: click, jam, lid catch (sim: also drives the joints)."""
    m, sim = _mem(), mk._EP.get("env") is not None
    for s in SIDES:
        q = bud_q(objects, s)
        if m["jam"][s] is not None:
            _drive(SEAT[s], m["jam"][s])
            continue
        if not m["clicked"][s] and touching(objects, s):
            if m["touch"][s] is None:
                m["touch"][s] = max(m["prev"][s], q)
            if m["touch"][s] - q >= JAM_TRAVEL:
                m["jam"][s] = q
                _drive(SEAT[s], q)
                _say(f"bud {s} JAMMED by the lid at {q * 1000:.2f} mm proud, lid {lid_deg(objects):.1f} deg")
                m["prev"][s] = q
                continue
        else:
            m["touch"][s] = None
        if not m["clicked"][s] and q <= CLICK:
            m["clicked"][s] = True
            _say(f"bud {s} CLICKED at {q * 1000:.2f} mm")
        if m["clicked"][s]:
            _drive(SEAT[s], SNAP_M)
        m["prev"][s] = q
    a = lid_deg(objects)
    if not m["latched"] and a <= LATCH_DEG:
        m["latched"] = True
        _say(f"lid LATCHED at {a:.2f} deg")
    if m["latched"]:
        _drive("lid_hinge", math.radians(HOLD_DEG))
    elif sim and mk._SIM["stick"].get((kit.id, "lid_hinge"), 1.0) < math.radians(RIDGE_DEG):
        _drive("lid_hinge", math.radians(RIDGE_DEG))
    if a <= PROUD_SHUT_DEG and any(bud_q(objects, s) > SEATED_TOL + 0.0003 for s in SIDES):
        if not m["proud_shut"]:
            _say("lid shut over a bud that is not home")
        m["proud_shut"] = True
    if DEBUG and sim and mk._EP.get("step", 0) % 5 == 0:
        cp = kit.pose(objects, "case")[0]
        tc = mk.tcps()
        tz = f"{(tc[1][0] - CX) * 1000:.1f},{(tc[1][2] - RIM) * 1000:.1f}" if tc else "-"
        _say(f"R tcp-x,z(rim) {tz} mm; tilt {_tilt(objects):.1f}; q_l {bud_q(objects, 'l') * 1000:.2f} q_r {bud_q(objects, 'r') * 1000:.2f} mm, lid {a:.2f} deg, "
             f"case ({cp[0]:.4f}, {cp[1]:.4f}, {cp[2] - TOP:.4f}) clicked {m['clicked']} jam {m['jam']} "
             f"latched {m['latched']}")


kit.on_step(_rules)


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def seated(objects, s) -> bool:
    """Bud ``s`` snapped home: its catch clicked and the head within 0.4 mm of home."""
    return bool(_mem()["clicked"][s] and _mem()["jam"][s] is None and bud_q(objects, s) <= SEATED_TOL)


def latched(objects) -> bool:
    """The lid's catch engaged and the lid within 1 deg of shut."""
    return bool(_mem()["latched"] and lid_deg(objects) <= SHUT_DEG)


def _tilt(objects) -> float:
    _, q = kit.pose(objects, "case")
    return math.degrees(math.acos(max(-1.0, min(1.0, float(mk.quat_to_mat(q)[2, 2])))))


def in_tray(objects) -> bool:
    """The case standing in the tray pocket: bottom centre within 3.5 mm of the pocket centre, on the floor, upright."""
    p, _ = kit.pose(objects, "case")
    return bool(abs(p[0] - TX) <= POCKET_TOL and abs(p[1] - TY) <= POCKET_TOL and abs(p[2] - ZT) <= FLOOR_TOL
                and _tilt(objects) <= TILT_MAX)


def score(objects) -> float:
    """0 if the lid ever drove a proud bud down (the bud jams) or shut over a bud not home. Otherwise 0.2 per bud
    snapped home; with both home, +0.2 for the lid latched shut, and with that, +0.4 for the case standing in the
    tray pocket. Halved unless both grippers are 2.5 cm clear of every part and everything is still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _rules(objects)
    m = _mem()
    if any(v is not None for v in m["jam"].values()) or m["proud_shut"]:
        return 0.0
    n = sum(seated(objects, s) for s in SIDES)
    base = 0.2 * n
    if n == 2 and latched(objects):
        base += 0.2
        if in_tray(objects):
            base += 0.4
    clear = all(_released(objects[p]) and objects[p].resting() for p in PARTS)
    return round(base if clear else base / 2, 6)


def success(objects) -> bool:
    """Both buds home, lid latched, case in its tray, let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Stages still to do (buds, lid, tray) plus the case's distance from the tray pocket (m)."""
    left = sum(not seated(objects, s) for s in SIDES) + (not latched(objects)) + (not in_tray(objects))
    p, _ = kit.pose(objects, "case")
    return float(left + math.hypot(p[0] - TX, p[1] - TY) + abs(p[2] - ZT))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry; right arm only, the left stays parked. Fingers closed, pad tips (13 mm below
# the TCP) push each head straight down its well; then, turned 90 deg, the closed fingers come down behind the open
# lid, 17 mm over the rim (under the open lid's tip, over the heads), and sweep it past upright; then they press
# straight down where they overlap the lid at any angle under 60 deg (so the lid, never a head, is what they meet),
# and last press its front edge until the catch holds. Last, fingers open, the
# arm grips the case by its two ends, lifts it out of the nest and stands it in the tray pocket.

TIP = 0.013  # pad tips below the TCP
PUSH_OVER = 0.003  # commanded past home (the arm stops on the bud's home stop)
SWEEP_UP = 0.017  # pad tips this high over the rim while sweeping the lid over (its tip is 22 mm up when open)
MID_X = 0.004  # then press straight down here (case frame): the pads overlap the lid at any angle under 60 deg
PRESS_X = HINGE[0] - 0.018  # final press, 18 mm in front of the hinge (the catch is at 24 mm)
PRESS_OVER = 0.005  # commanded past shut on the lid
SQUEEZE = 0.008
GRIP = -(BODY[1] - SQUEEZE) / 2
GRIP_UP = 0.022  # pad tips this high up the case (2 mm over the nest walls)
LIFT = 0.035
TRAY_TRIM = (0.0035, 0.0015)  # set-down correction: the carried case lands 3 mm short in x, 1.5 mm right (sim run)
PARK = (0.220, -0.230, TOP + 0.160)
YAW90 = math.pi / 2
SMOKE = False  # True: the naive-lid smoke test (one bud pushed, lid pressed hard onto the proud one)


def demo():
    """Push the right bud, then the left, straight down until each clicks home; sweep the lid over and press its
    front edge until the catch holds; grip the case by its ends, lift it out of the nest, stand it in the tray
    pocket, let go and back away."""
    c, o = FINGER_CLOSED, FINGER_OPEN
    R = [Waypoint(2.0, PARK, c, "start", 0.0)]

    def w(dur, pos, f, label, yaw=0.0):
        R.append(Waypoint(dur, tuple(float(v) for v in pos), f, label, yaw))

    bx = CX + HEAD_X
    top_proud, top_home = RIM + HEAD_UP + PROUD, RIM + HEAD_UP
    for s in (("r",) if SMOKE else ("r", "l")):
        by = CY + SIGN[s] * BUD_Y
        w(2.0, (bx, by, top_proud + TIP + 0.020), c, f"over the {s} bud, fingers closed")
        w(1.0, (bx, by, top_proud + TIP + 0.001), c, "fingertips onto its head")
        w(2.5, (bx, by, top_home + TIP - PUSH_OVER), c, "push it straight down until it clicks")
        w(0.5, (bx, by, top_home + TIP - PUSH_OVER), c, "hold")
        w(1.0, (bx, by, top_proud + TIP + 0.020), c, "lift off")
    sz = RIM + SWEEP_UP + TIP
    w(1.5, (CX + 0.045, CY, sz + 0.020), c, "behind the open lid, turned", YAW90)
    w(1.0, (CX + 0.045, CY, sz), c, "down behind it", YAW90)
    w(3.5, (CX - 0.010, CY, sz), c, "sweep the lid over", YAW90)
    w(1.0, (CX - 0.010, CY, RIM + 0.034 + TIP), c, "up, clear of the lid", YAW90)
    w(1.0, (CX + MID_X, CY, RIM + 0.034 + TIP), c, "over the lid's middle", YAW90)
    over = 0.010 if SMOKE else 0.0
    w(2.5, (CX + MID_X, CY, RIM + LID_TOP + TIP - over), c, "press the lid down", YAW90)
    w(1.5 if SMOKE else 0.5, (CX + MID_X, CY, RIM + LID_TOP + TIP - over), c, "hold", YAW90)
    if not SMOKE:
        w(0.8, (CX + MID_X, CY, RIM + 0.016 + TIP), c, "up a little", YAW90)
        w(0.8, (PRESS_X, CY, RIM + 0.016 + TIP), c, "over the lid's front edge", YAW90)
        w(1.5, (PRESS_X, CY, RIM + LID_TOP + TIP - PRESS_OVER), c, "press the front until the catch holds", YAW90)
        w(1.0, (PRESS_X, CY, RIM + LID_TOP + TIP - PRESS_OVER), c, "hold", YAW90)
    w(1.0, (PRESS_X, CY, RIM + 0.050 + TIP), c, "up", YAW90)
    if SMOKE:
        w(2.0, PARK, c, "park")
        w(3.0, PARK, c, "watch")
    else:
        gz = ZB + GRIP_UP + TIP
        tx, ty = TX + TRAY_TRIM[0], TY + TRAY_TRIM[1]
        tz = ZT + GRIP_UP + TIP
        w(1.5, (CX, CY, gz + 0.060), o, "over the case, fingers open")
        w(1.5, (CX, CY, gz), o, "down around its ends")
        w(0.5, (CX, CY, gz), o, "settle")
        w(1.0, (CX, CY, gz), GRIP, "grip the case by its ends")
        w(0.5, (CX, CY, gz), GRIP, "settle closed")
        w(1.5, (CX, CY, gz + LIFT), GRIP, "lift out of the nest")
        w(2.0, (tx, ty, tz + LIFT), GRIP, "carry over the tray")
        w(1.5, (tx, ty, tz + 0.002), GRIP, "lower into the pocket")
        w(0.5, (tx, ty, tz + 0.002), GRIP, "settle")
        w(0.8, (tx, ty, tz + 0.002), o, "let go")
        w(1.5, (tx, ty, tz + 0.080), o, "back up")
        w(1.5, PARK, o, "park")
    total = sum(x.duration for x in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park")]
    return {"left": left, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def contact_deg(q) -> float:
    """Lid angle (deg) at which the closing lid first touches a head standing ``q`` above home (0 if never)."""
    def hit(a):
        return _contact(kit.fake_states(q={"lid_hinge": math.radians(a), "seat_l": q, "seat_r": 0.0}), "l", 0.0)
    a = 60.0
    while a > 0.0 and not hit(a):  # close the lid from 60 deg in 0.25 deg steps until it first touches
        a -= 0.25
    if a <= 0.0:
        return 0.0
    lo, hi = a, a + 0.25
    for _ in range(20):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if hit(mid) else (lo, mid)
    return lo


def self_test():
    """The scoring rules on hand-built sequences of poses (one trial each, fed step by step)."""
    global _TEST_TCPS
    up = (0.0, 0.0, 0.0, 1.0)
    nest = (CX, CY, ZB)
    tray = (TX, TY, ZT)

    def fr(ql=PROUD, qr=PROUD, lid=OPEN_DEG, at=nest, quat=up):
        return kit.fake_states(q={"seat_l": ql, "seat_r": qr, "lid_hinge": math.radians(lid)},
                               moved={"case": (at, quat)})

    def lin(a, b, n=10):
        return [a + (b - a) * k / n for k in range(n + 1)]

    def push(s, q_other_l=None, lid=OPEN_DEG, to=0.0):  # bud s pushed from proud to ``to``, lid open
        out = []
        for q in lin(PROUD, to):
            kw = {"ql": q, "qr": 0.0} if s == "l" else {"ql": PROUD if q_other_l is None else q_other_l, "qr": q}
            out.append(fr(lid=lid, **kw))
        return out

    both = push("r") + push("l")
    close = [fr(0.0, 0.0, a) for a in lin(OPEN_DEG, 0.0, 20)]
    carry = [fr(0.0, 0.0, 0.0, at=tuple(np.asarray(nest) + (np.asarray(tray) - np.asarray(nest)) * k / 10
                                            + np.array([0, 0, 0.035 * math.sin(math.pi * k / 10)])))
             for k in range(11)]
    good = both + close + carry
    ac = contact_deg(PROUD)
    a_click = contact_deg(CLICK + 1e-4)
    a_home = contact_deg(0.0)
    assert 15.0 < ac < 40.0 and a_click > RIDGE_DEG - 0.5 > LATCH_DEG and a_home == 0.0, (ac, a_click, a_home)
    # the lever "loophole" as frames: the lid rides on the proud left head and drives it home, then shuts
    lever = (push("r") + [fr(PROUD, 0.0, a) for a in lin(OPEN_DEG, ac, 10)]
             + [fr(q, 0.0, contact_deg(q)) for q in lin(PROUD, 0.0, 15)] + [fr(0.0, 0.0, 0.0)] * 3)
    gentle = push("r") + [fr(PROUD, 0.0, a) for a in lin(OPEN_DEG, ac, 10)] + [fr(PROUD, 0.0, ac)] * 3
    grip_tcp = (TX, TY, ZT + GRIP_UP + TIP)
    cases = {  # name: (frames, TCPs at the end, score)
        "start": ([fr()], [], 0.0),
        "both buds home, lid shut and latched, case in the tray, let go (demo)": (good, [], 1.0),
        "the same but still holding the case": (good, [grip_tcp], 0.5),
        "naive: lid swung onto both proud buds and pressed (bud jams)":
            ([fr(PROUD, PROUD, a) for a in lin(OPEN_DEG, ac, 10)]
             + [fr(q, q, contact_deg(q)) for q in lin(PROUD, 0.0, 15)] + [fr(0.0, 0.0, 0.0)] * 3 + carry, [], 0.0),
        "lever loophole: lid drives the left bud home, then latched, case in tray": (lever + carry, [], 0.0),
        "lid closed gently onto a proud bud, nothing forced": (gentle, [], 0.2),
        "right bud home only, lid left open": (push("r"), [], 0.2),
        "both buds home, lid left open": (both, [], 0.4),
        "both home, lid open, case carried to the tray anyway":
            (both + [fr(0.0, 0.0, OPEN_DEG, at=tray)], [], 0.4),
        "both home, lid latched, case left in the nest": (both + close, [], 0.6),
        "left bud pushed to 2 mm only (no click), lid open": (push("r") + push("l", to=0.002), [], 0.2),
        "bud let go 1 mm proud after the click (snaps home in sim; here still 1 mm up)":
            (push("r") + push("l", to=0.001), [], 0.2),
        "lid shut through a proud head (penetration guard)": (push("r") + [fr(PROUD, 0.0, 0.0)], [], 0.0),
        "case on the tray wall, tilted 10 deg":
            (both + close + [fr(0.0, 0.0, 0.0, at=(TX, TY + 0.012, ZT + 0.006),
                                quat=tuple(mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(10)))))], [], 0.6),
        "case beside the pocket on the tray base": (both + close + [fr(0.0, 0.0, 0.0, at=(TX + 0.045, TY, ZT))],
                                                    [], 0.6),
    }
    try:
        for name, (frames, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for f in frames[:-1]:
                score(f)
            _TEST_TCPS = tcps
            got = score(frames[-1])
            assert abs(got - want) < 1e-9, (name, got, want, _mem())
            assert success(frames[-1]) == (want >= 1.0), name
        mk.new_episode()
        _TEST_TCPS = []
        for f in lever:
            score(f)
        mk.new_episode()
        assert score(fr()) == 0.0 and _mem()["jam"]["l"] is None, "a new trial forgets the last one"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1, ac, a_click


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n, ac, a_click = self_test()
    print(f"scoring: {n} cases pass (lid meets a proud head at {ac:.1f} deg; one 1.5 mm up holds it at "
          f"{a_click:.1f} deg)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
