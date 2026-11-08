"""On the station table, between the two arms, a square frying pan of burning oil stands on a hob: the oil is alight
(orange flame tongues stand on it). In front of the hob, nearer the robot, a fire blanket hangs by two hang pins
from a pair of hooks on a low stand; its top three bands hang from the hooks and its lower part lies folded out on
the worktop toward the robot. Two short tapes stand up from the blanket's top edge, one near each end. The job is to
put the fire out with the blanket. Scored from the poses of the blanket's bands and the grippers.

The blanket is seven glass-fibre bands joined edge to edge by hinges that turn freely against a little friction
(mechkit.py, in this folder, adds the hinges to the stock runtime in memory), so it hangs, drapes and folds along
its length like a limp sheet.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/pan-fire-blanket-seal/pan_fire_blanket_seal.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (installs the hinge hooks and the per-episode memory)

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).

# -- hob, pan, oil, flames (all fixed) --
HOB = (0.240, 0.300, 0.006)  # glass-ceramic hob plate: x, y, thickness
HOB_X = 0.335
HOB_TOP = TOP + HOB[2]
PX, PY = 0.320, 0.000  # pan centre
PAN_OUT = 0.120  # outside, square
PAN_WALL = 0.010
PAN_H = 0.045  # wall height above the hob
RIM = HOB_TOP + PAN_H  # the rim's top face
OPEN_HALF = PAN_OUT / 2 - PAN_WALL  # the opening: 10 x 10 cm
OIL_TOP = RIM - 0.020
FLAME = (0.020, 0.020, 0.015)
FLAME_TOP = OIL_TOP + FLAME[2]  # 5 mm below the rim
FLAME_XY = ((-0.025, -0.025), (-0.025, 0.025), (0.025, -0.025), (0.025, 0.025), (0.0, 0.0))

# -- the blanket (free): seven bands, hinged edge to edge about the y axis --
N_BANDS = 7
BAND = 0.040  # length of one band along the blanket
WIDTH = 0.240  # blanket width (y)
THICK = 0.004
BAND_KG = 0.010
TOP_KG = 0.014  # the top band carries the tapes and the hang pins
FRICTION = 0.0008  # N*m at each hinge: below one band's own gravity torque (0.002 N*m), so the sheet stays limp
LIMIT_DEG = 170.0
HANG = 3  # bands hanging from the hooks at the start; the rest lie on the worktop
XH = 0.190  # the plane the hanging bands hang in
FOLD_Z = TOP + 0.0005 + THICK / 2  # the hinge between the last hanging band and the first lying one
ZT = FOLD_Z + HANG * BAND  # the blanket's top edge (hinge line of the hang pins)
TAPE = (0.016, 0.025, 0.035)  # tapes: x, y, z (stand up from the top edge)
TAPE_Y = 0.095  # 19 cm apart
PIN = (0.008, 0.020, 0.008)
PIN_Y = 0.130  # the pins stick out 1-3 cm past the blanket's ends, into the hooks

GLASS = (0.86, 0.20, 0.16)  # red glass-fibre fabric
TAPE_C = (0.10, 0.10, 0.10)
STEEL = (0.62, 0.64, 0.67)
HOB_C = (0.10, 0.10, 0.11)
PAN_C = (0.20, 0.20, 0.22)
OIL_C = (0.55, 0.38, 0.08)
FLAME_C = (1.00, 0.55, 0.05)

BANDS = tuple(f"blanket_band{k}" for k in range(N_BANDS))
FOLDS = tuple(f"fold{k}" for k in range(1, N_BANDS))  # fold k joins band k-1 (nearer the top edge) to band k

kit = mk.Kit()
# Reference pose: the whole blanket hanging straight down in the plane x = XH. Each band's frame sits on its upper
# hinge line, its fabric running down (local -z), its width along y, its thickness along x.
for _k, _name in enumerate(BANDS):
    _boxes = [mk.box((THICK, WIDTH, BAND), at=(0.0, 0.0, -BAND / 2))]
    if _k == 0:
        for _s in (-1, 1):
            _boxes.append(mk.box(TAPE, at=(0.0, _s * TAPE_Y, TAPE[2] / 2), color=TAPE_C))
            _boxes.append(mk.box(PIN, at=(0.0, _s * (PIN_Y), -PIN[2] / 2), color=TAPE_C))
    kit.part(_name, (XH, 0.0, ZT - _k * BAND), _boxes, mass=TOP_KG if _k == 0 else BAND_KG, color=GLASS)
for _k, _fold in enumerate(FOLDS, start=1):
    # axis +y: a positive angle swings the bands below toward -x (the robot)
    kit.hinge(_fold, BANDS[_k - 1], BANDS[_k], anchor=(XH, 0.0, ZT - _k * BAND), axis=(0, 1, 0),
              lower_deg=-LIMIT_DEG, upper_deg=LIMIT_DEG, start_deg=90.0 if _k == HANG else 0.0, friction=FRICTION)


def _fixtures():
    out = [Box("hob", pos=(HOB_X, 0.0, TOP + HOB[2] / 2), size=HOB, color=HOB_C)]
    # pan: base and four walls standing on the hob
    out.append(Box("pan_base", pos=(PX, PY, HOB_TOP + 0.002), size=(PAN_OUT, PAN_OUT, 0.004), color=PAN_C))
    for side, sx in (("near", -1), ("far", 1)):
        out.append(Box(f"pan_wall_{side}", pos=(PX + sx * (PAN_OUT - PAN_WALL) / 2, PY, HOB_TOP + PAN_H / 2),
                       size=(PAN_WALL, PAN_OUT, PAN_H), color=PAN_C))
    for side, sy in (("left", 1), ("right", -1)):
        out.append(Box(f"pan_wall_{side}", pos=(PX, PY + sy * (PAN_OUT - PAN_WALL) / 2, HOB_TOP + PAN_H / 2),
                       size=(PAN_OUT - 2 * PAN_WALL, PAN_WALL, PAN_H), color=PAN_C))
    oil_lo = HOB_TOP + 0.004
    out.append(Box("burning_oil", pos=(PX, PY, (oil_lo + OIL_TOP) / 2),
                   size=(2 * OPEN_HALF - 0.001, 2 * OPEN_HALF - 0.001, OIL_TOP - oil_lo), color=OIL_C))
    for i, (dx, dy) in enumerate(FLAME_XY):
        out.append(Box(f"flame{i}", pos=(PX + dx, PY + dy, OIL_TOP + FLAME[2] / 2), size=FLAME, color=FLAME_C))
    # hook stand: a post at each end of the blanket with an upward-open U hook holding a hang pin
    seat = ZT - PIN[2] - 0.0005  # the hook floor's top face
    hook_y = PIN_Y + 0.006
    for side, sy in (("left", 1), ("right", -1)):
        out.append(Box(f"hook_{side}_floor", pos=(XH, sy * hook_y, seat - 0.002), size=(0.020, 0.028, 0.004),
                       color=STEEL))
        for face, sx in (("front", 1), ("back", -1)):
            out.append(Box(f"hook_{side}_{face}", pos=(XH + sx * 0.0075, sy * hook_y, seat + 0.006),
                           size=(0.005, 0.028, 0.016), color=STEEL))
        post_top = seat - 0.004
        out.append(Box(f"hook_{side}_post", pos=(XH, sy * (PIN_Y + 0.015), (TOP + post_top) / 2),
                       size=(0.015, 0.015, post_top - TOP), color=STEEL))
    return out


FIXTURES = _fixtures()
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Everything is measured from the bands' poses (a top-down photo shows what covers the pan) and the grippers' tool
# points. Fire-blanket instructions: lay the blanket over the pan so that it covers it completely (a gap at the rim
# lets air in and the fire flares back), do not throw it, keep your hands shielded and out of the flames, and leave
# it in place until the pan has cooled.

HZ = 20.0
_xs = np.linspace(PX - OPEN_HALF + 0.005, PX + OPEN_HALF - 0.005, 5)
_ys = np.linspace(PY - OPEN_HALF + 0.005, PY + OPEN_HALF - 0.005, 5)
OPEN_PTS = tuple((float(x), float(y)) for x in _xs for y in _ys)  # 25 points over the opening
_RC = PAN_OUT / 2 - PAN_WALL / 2  # the rim's centre line
_rs = np.linspace(-OPEN_HALF + 0.005, OPEN_HALF - 0.005, 4)
RIM_PTS = tuple([(PX - _RC, PY + float(v)) for v in _rs] + [(PX + _RC, PY + float(v)) for v in _rs]
                + [(PX + float(v), PY + _RC) for v in _rs] + [(PX + float(v), PY - _RC) for v in _rs])  # 16
OPEN_BAND = (FLAME_TOP - 0.010, RIM + 0.040)  # a band covers an opening point at a height in this range
RIM_BAND = (RIM - 0.005, RIM + 0.020)  # ... and seals a rim point within 2 cm above it
FIRE_OUT_STEPS = round(1.0 * HZ)  # every point covered for 1 s: the fire is out
LIFT_STEPS = round(0.5 * HZ)  # after that, an opening point uncovered for 0.5 s: the blanket was lifted
FLAME_ZONE = (PX - OPEN_HALF - 0.02, PX + OPEN_HALF + 0.02, PY - OPEN_HALF - 0.02, PY + OPEN_HALF + 0.02,
              RIM + 0.080)  # x lo, x hi, y lo, y hi, z top: the flames above the opening
DROP_HEIGHT = RIM + 0.040  # a band whose lowest point is above this ...
DROP_SPEED = 0.30  # ... falling faster than this (m/s) with no gripper at a tape: thrown or dropped
TAPE_NEAR = 0.040
STILL_M, STILL_DEG = 0.0003, 0.5
OFF_HOOK = 0.012
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _tcps():
    return [np.asarray(t, float) for t in (mk.tcps() if _TEST_TCPS is None else _TEST_TCPS)]


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())


def _height_over(t, x, y):
    """Height of a band's mid-plane above the world point (x, y), or None if (x, y) is outside its top-down footprint
    (or the band stands steeper than 80 degrees). ``t`` is the band's 4x4 pose."""
    r, o = t[:3, :3], t[:3, 3]
    m = np.array([[r[0, 1], r[0, 2]], [r[1, 1], r[1, 2]]])  # world xy of the band's width and length axes
    if abs(np.linalg.det(m)) < math.cos(math.radians(80.0)):
        return None
    a, b = np.linalg.solve(m, [x - o[0], y - o[1]])
    if abs(a) > WIDTH / 2 or b > 0.0 or b < -BAND:
        return None
    return float(o[2] + a * r[2, 1] + b * r[2, 2])


def _covered(poses, pts, band) -> list:
    out = []
    for x, y in pts:
        hit = False
        for t in poses:
            z = _height_over(t, x, y)
            if z is not None and band[0] <= z <= band[1]:
                hit = True
                break
        out.append(hit)
    return out


def _tape_points(objects):
    return [kit.point(objects, BANDS[0], (0.0, s * TAPE_Y, TAPE[2] / 2)) for s in (-1, 1)]


def _observe(objects) -> dict:
    """Update the trial's history once per control step (score() and success() see the same states object) and
    return this step's readings."""
    m = mk.MEMORY
    if m.get("pf_last") is objects:
        return m["pf_now"]
    poses = [kit._pose_T(objects, b) for b in BANDS]
    tcps = _tcps()
    now = {"open": _covered(poses, OPEN_PTS, OPEN_BAND), "rim": _covered(poses, RIM_PTS, RIM_BAND)}
    allc = all(now["open"]) and all(now["rim"])
    # off the hooks: the top band lifted out of the hooks or carried off them
    top = poses[0][:3, 3]
    if top[2] > ZT + OFF_HOOK or abs(top[0] - XH) > 0.03:
        m["pf_off_hooks"] = True
    # (a) a gripper's tool point in the flames above the opening
    x0, x1, y0, y1, z1 = FLAME_ZONE
    if any(x0 <= t[0] <= x1 and y0 <= t[1] <= y1 and t[2] <= z1 for t in tcps):
        m["pf_v_flames"] = True
    # (b) thrown or dropped: a band high above the rim falling fast with no gripper at either tape
    prev = m.get("pf_prev")
    held = any(np.linalg.norm(t - p) <= TAPE_NEAR for t in tcps for p in _tape_points(objects))
    still = False
    if prev is not None:
        dz = [prev[i][2, 3] - poses[i][2, 3] for i in range(N_BANDS)]
        if not held and any(objects[b].low[2] > DROP_HEIGHT and dz[i] > DROP_SPEED / HZ
                            for i, b in enumerate(BANDS)):
            m["pf_v_dropped"] = True
        still = all(np.linalg.norm(prev[i][:3, 3] - poses[i][:3, 3]) < STILL_M
                    and mk._rot_angle(prev[i][:3, :3].T @ poses[i][:3, :3]) < math.radians(STILL_DEG)
                    for i in range(N_BANDS))
    m["pf_prev"] = poses
    # fire out: every opening and rim point covered for 1 s
    m["pf_run"] = m.get("pf_run", 0) + 1 if allc else 0
    if m["pf_run"] >= FIRE_OUT_STEPS:
        m["pf_fire_out"] = True
    # (c) lifted to check: after the fire is out, an opening point uncovered for 0.5 s
    if m.get("pf_fire_out"):
        m["pf_gap_run"] = m.get("pf_gap_run", 0) + 1 if not all(now["open"]) else 0
        if m["pf_gap_run"] >= LIFT_STEPS:
            m["pf_v_lifted"] = True
    now.update(all=allc, still=still)
    m["pf_last"], m["pf_now"] = objects, now
    return now


def _violated() -> bool:
    m = mk.MEMORY
    return bool(m.get("pf_v_flames") or m.get("pf_v_dropped") or m.get("pf_v_lifted"))


def score(objects) -> float:
    """0 for the rest of the trial once a gripper's tool point has been in the flames above the opening, the blanket
    has been thrown or dropped onto the pan, or (after the fire is out) the blanket has been lifted off the opening
    for 0.5 s. Otherwise: 1.0 when the fire is out (all 25 opening points and all 16 rim points covered for 1 s), the
    blanket still covers all 41 points, no gripper is within 2.5 cm of any band and every band is still; 0.75 when the
    fire is out; 0.5 when every opening point is covered; 0.25 when the blanket is off its hooks and covers some
    opening point; 0 otherwise."""
    now = _observe(objects)
    if _violated():
        return 0.0
    m = mk.MEMORY
    if m.get("pf_fire_out"):
        if now["all"] and now["still"] and all(_released(objects[b]) for b in BANDS):
            return 1.0
        return 0.75
    if all(now["open"]):
        return 0.5
    if m.get("pf_off_hooks") and any(now["open"]):
        return 0.25
    return 0.0


def success(objects) -> bool:
    """The fire out, the blanket left on, let go of and still; holds for 1 s."""
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Fraction of the 41 opening and rim points the blanket does not cover yet."""
    now = _observe(objects)
    return float(1.0 - (sum(now["open"]) + sum(now["rim"])) / (len(OPEN_PTS) + len(RIM_PTS)))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's fixed geometry; it reads no joint or simulator state. Each arm takes one tape (fingers
# closing along x, across the tape), lifts the blanket off the hooks, and both carry it forward over the pan while
# lowering, so the blanket's lower part trails over the near rim and the hands come down beyond the far rim; then
# they let go and lift away, staying outside the zone above the opening the whole time.

TIP = 0.013  # pad tips below the TCP
GRIP_Z = ZT + 0.003 + TIP  # pad tips 3 mm above the top edge: 32 mm of each tape between the pads
OPEN = -0.020  # 40 mm between the pads
C = -(TAPE[0] - 0.008) / 2  # close to 8 mm under the 16 mm tape (the usual squeeze)
YAW = {"left": -math.pi / 2, "right": math.pi / 2}
TAPE_AT = {"left": TAPE_Y, "right": -TAPE_Y}
# the laying path (x, TCP height above the table), the same for both hands
LIFT = (XH, 0.170)
PATH = ((0.250, 0.180, 3.0), (0.330, 0.170, 4.0), (0.370, 0.150, 2.5), (0.400, 0.125, 2.0), (0.410, 0.100, 2.0))
PARK = {"left": (0.200, 0.200, TOP + 0.200), "right": (0.200, -0.200, TOP + 0.200)}


def _arm(arm):
    y, yaw = TAPE_AT[arm], YAW[arm]
    w = [
        Waypoint(2.0, (XH, y, GRIP_Z + 0.050), OPEN, "over the tape", yaw),
        Waypoint(1.5, (XH, y, GRIP_Z), OPEN, "down beside the tape", yaw),
        Waypoint(0.5, (XH, y, GRIP_Z), OPEN, "settle", yaw),
        Waypoint(0.8, (XH, y, GRIP_Z), C, "grip the tape", yaw),
        Waypoint(0.3, (XH, y, GRIP_Z), C, "settle", yaw),
        Waypoint(2.0, (LIFT[0], y, TOP + LIFT[1]), C, "lift off the hooks", yaw),
        Waypoint(0.5, (LIFT[0], y, TOP + LIFT[1]), C, "let it hang", yaw),
    ]
    for x, z, s in PATH:
        w.append(Waypoint(s, (x, y, TOP + z), C, "lay it forward over the pan", yaw))
    x_end, z_end = PATH[-1][0], PATH[-1][1]
    w += [
        Waypoint(0.5, (x_end, y, TOP + z_end), C, "settle", yaw),
        Waypoint(0.8, (x_end, y, TOP + z_end), OPEN, "let go", yaw),
        Waypoint(0.5, (x_end, y, TOP + z_end), OPEN, "settle", yaw),
        Waypoint(1.2, (x_end - 0.005, y, TOP + 0.120), OPEN, "lift away beyond the pan", yaw),
        Waypoint(2.0, PARK[arm], OPEN, "park", yaw),
    ]
    return w


def demo():
    """Both arms together: take the tapes, lift the blanket off its hooks, carry it forward over the pan while
    lowering (about 2-3 cm/s), let go just beyond the far rim and lift away. 26 s."""
    return {"left": _arm("left"), "right": _arm("right")}


# -- offline self-test ------------------------------------------------------------------------------------------


def _chain(points, step=BAND):
    """Hinge points every ``step`` along a polyline (x, z), starting at its first point, at y = 0."""
    pts = [np.asarray(p, float) for p in points]
    out = [pts[0]]
    seg, pos = 0, pts[0]
    while len(out) < N_BANDS + 1:
        # walk to the next point at distance ``step`` from ``pos`` along the polyline
        need = step
        cur = pos
        while True:
            if seg >= len(pts) - 1:
                d = pts[-1] - pts[-2]
                cur = cur + d / np.linalg.norm(d) * need
                break
            rest = np.linalg.norm(pts[seg + 1] - cur)
            if rest >= need:
                cur = cur + (pts[seg + 1] - cur) / rest * need
                break
            need -= rest
            cur = pts[seg + 1]
            seg += 1
        out.append(cur)
        pos = cur
    return out


def _lay(points, dy=0.0):
    """The blanket's bands posed along hinge points (x, z): band k from point k to point k+1."""
    moved = {}
    for k, name in enumerate(BANDS):
        a, b = np.asarray(points[k], float), np.asarray(points[k + 1], float)
        down = np.array([b[0] - a[0], 0.0, b[1] - a[1]])
        down /= np.linalg.norm(down)
        ez = -down
        ey = np.array([0.0, 1.0, 0.0])
        ex = np.cross(ey, ez)
        r = np.column_stack([ex, ey, ez])
        moved[name] = (np.array([a[0], dy, a[1]]), tuple(mk.mat_to_quat(r)))
    return kit.fake_states(moved=moved)


def self_test():
    """The scoring rules on hand-built blanket poses (state sequences, one trial each)."""
    global _TEST_TCPS
    t2 = THICK / 2
    x_near, x_far = PX - PAN_OUT / 2, PX + PAN_OUT / 2
    start = kit.fake_states()
    # laid as the demo leaves it: top edge down the far wall, across the flames, over the near rim, onto the hob
    laid_pts = [(x_far + 0.006, HOB_TOP + 0.012), (x_far - 0.001, RIM + t2), (PX, FLAME_TOP + t2),
                (x_near + 0.001, RIM + t2), (x_near - 0.004, HOB_TOP + t2), (0.10, HOB_TOP + t2)]
    laid = _lay(_chain(laid_pts))
    held = _lay(_chain([(x_far + 0.03, RIM + 0.035)] + laid_pts[1:]))  # still in the hands beyond the far rim
    # lowered in place over the middle: a zigzag pile on the flames, the tail trailing over the near rim
    pile_pts = [(PX + 0.02, FLAME_TOP + 0.03), (PX - 0.02, FLAME_TOP + 0.024), (PX + 0.02, FLAME_TOP + 0.018),
                (PX - 0.02, FLAME_TOP + 0.012), (PX + 0.02, FLAME_TOP + 0.006), (PX - 0.03, FLAME_TOP + t2),
                (x_near + 0.001, RIM + t2), (x_near - 0.004, HOB_TOP + t2), (0.10, HOB_TOP + t2)]
    pile = _lay(_chain(pile_pts))
    # laid short: the top edge lands on the flames, the far rim stays open
    short_pts = [(PX + 0.04, FLAME_TOP + t2), (x_near + 0.001, RIM + t2), (x_near - 0.004, HOB_TOP + t2),
                 (0.10, HOB_TOP + t2)]
    short = _lay(_chain(short_pts))
    # laid too far: the free end pulled up past the near rim, which stays open
    far_pts = [(x_far + 0.133, HOB_TOP + t2), (x_far + 0.006, HOB_TOP + t2), (x_far - 0.001, RIM + t2),
               (PX, FLAME_TOP + t2), (x_near - 0.05, RIM + t2)]
    too_far = _lay(_chain(far_pts))
    # blanket shifted 9 cm sideways: one side rim and part of the opening open
    shifted = _lay(_chain(laid_pts), dy=0.090)
    # carried high over the pan (tops 17 cm above the table) and let go: falling 2 cm per step
    high = _lay(_chain([(PX - 0.02, TOP + 0.170), (PX - 0.02, TOP + 0.170 - 7 * BAND)]))
    high2 = _lay(_chain([(PX - 0.02, TOP + 0.150), (PX - 0.02, TOP + 0.150 - 7 * BAND)]))
    # lifted to check: a corner of the blanket raised off the opening after the fire is out
    lifted = _lay(_chain([(PX + 0.03, RIM + 0.10), (PX - 0.01, FLAME_TOP + t2)] + laid_pts[3:]))

    away = [np.array([0.20, 0.20, TOP + 0.20]), np.array([0.20, -0.20, TOP + 0.20])]
    tapes_held = [np.asarray(p) for p in _tape_points(held)]
    tapes_high, tapes_high2 = ([np.asarray(p) for p in _tape_points(s)] for s in (high, high2))
    touch = [np.array([0.20, 0.10, HOB_TOP + 0.015]), away[1]]  # a gripper on the blanket's tail on the hob
    in_flames = [np.array([PX, 0.0, RIM + 0.03]), away[1]]

    def hold(state, n):
        return [dict(state) for _ in range(n)]

    out_s = FIRE_OUT_STEPS + 1
    cases = {  # name: (sequence of (states, tcps) per step, final score, success)
        "start": ([(start, away)], 0.0, False),
        "laid, still held at the tapes": ([(start, away)] + [(s, tapes_held) for s in hold(held, 3)], 0.5, False),
        "laid and let go, still for 1 s": ([(start, away), (held, tapes_held)] + [(s, away) for s in hold(laid, out_s)],
                                           1.0, True),
        "laid and let go, fire not yet out (0.5 s)": ([(start, away)] + [(s, away) for s in hold(laid, 10)], 0.5,
                                                       False),
        "fire out, then a gripper back on the blanket": (
            [(start, away)] + [(s, away) for s in hold(laid, out_s)] + [(dict(laid), touch)], 0.75, False),
        "naive: lowered in place, a zigzag pile": ([(start, away)] + [(s, away) for s in hold(pile, out_s)], 0.25,
                                                   False),
        "laid short, far rim open": ([(start, away)] + [(s, away) for s in hold(short, out_s)], 0.25, False),
        "pulled too far, near rim open": ([(start, away)] + [(s, away) for s in hold(too_far, out_s)], 0.5, False),
        "shifted 9 cm sideways": ([(start, away)] + [(s, away) for s in hold(shifted, out_s)], 0.25, False),
        "dropped from 17 cm above the table, then laid": (
            [(start, away), (high, away), (high2, away)] + [(s, away) for s in hold(laid, out_s)], 0.0, False),
        "held high and lowered slowly by the tapes": (
            [(start, away), (high, tapes_high), (high2, tapes_high2)] + [(s, away) for s in hold(laid, out_s)], 1.0,
            True),
        "a gripper in the flames, then laid": (
            [(start, away), (pile, in_flames)] + [(s, away) for s in hold(laid, out_s)], 0.0, False),
        "fire out, then lifted to check for 0.5 s, laid back": (
            [(start, away)] + [(s, away) for s in hold(laid, out_s)] + [(s, away) for s in hold(lifted, LIFT_STEPS)]
            + [(s, away) for s in hold(laid, out_s)], 0.0, False),
        "fire out, a corner lifted briefly (0.25 s), laid back": (
            [(start, away)] + [(s, away) for s in hold(laid, out_s)] + [(s, away) for s in hold(lifted, 5)]
            + [(s, away) for s in hold(laid, out_s)], 1.0, True),
    }
    try:
        for name, (seq, want, want_success) in cases.items():
            mk.new_episode()
            for states, tcps in seq[:-1]:
                _TEST_TCPS = tcps
                score(states)
            states, tcps = seq[-1]
            _TEST_TCPS = tcps
            got, got_success = score(states), success(states)
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want)
        # a new trial forgets the last one's violation
        mk.new_episode()
        _TEST_TCPS = in_flames
        score(dict(pile))
        assert score(dict(laid)) == 0.0
        mk.new_episode()
        _TEST_TCPS = away
        for s in hold(laid, out_s):
            score(s)
        assert score(dict(laid)) == 1.0, "reset"
        # the start state: nothing covered, still on the hooks
        mk.new_episode()
        assert not any(_observe(dict(start))["open"]) and not mk.MEMORY.get("pf_off_hooks")
        assert distance(dict(laid)) == 0.0 and distance(dict(start)) == 1.0
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
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
