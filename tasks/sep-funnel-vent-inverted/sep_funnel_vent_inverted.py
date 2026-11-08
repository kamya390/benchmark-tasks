"""On the station table, in front of the left arm, a glass separatory funnel stands in a slotted funnel support (two
parallel rubber-coated rails on a stand, 4.4 cm apart): an octagonal bulb 5.6 cm across and 6 cm tall holding two
liquid layers, a ground-glass stopper on its neck, and under the bulb a narrow stem with a PTFE stopcock. The
stopcock's key sits on a short shaft beside the stem: across the stem it is shut, turned a quarter turn in line
with the stem it is open. The far side of the table is the back of the fume hood; the arms work from its open
front. The job is to mix the two layers by shaking the funnel and put it back in its support with the stopper in.

Mixing a solvent with an aqueous layer builds vapour pressure inside the stoppered funnel. The pressure is not
simulated as gas; it is a counter the scene keeps: turning the funnel over (stem above horizontal) adds 2, every
shaking stroke (3 cm or more, turned over, stopcock shut) adds 1, and opening the stopcock (key past 45 deg) with
the stem at least 30 deg above horizontal vents it back to 0. At 6 the stopper blows out (its hold is let go and it
drops out of the neck) and the trial scores 0. Opening the stopcock with the stem less than 30 deg above horizontal
runs liquid out of the stem and also scores 0. Venting with the stem pointed back toward the arms (the operator's
side of the hood) caps the trial at 0.5.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/sep-funnel-vent-inverted/sep_funnel_vent_inverted.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (per-episode memory, gripper TCPs, the stopcock hinge, the stopper's hold)

from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik as _ik  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases (toward the back of the hood), y left, z up, metres.
# -- the funnel (kit part; frame origin = the bulb's centre, +z toward the stopper) --------------------------------
SEAT = np.array([0.300, 0.120, TOP + 0.150])  # the bulb's centre when it rests in the support
FUNNEL_KG, STOPPER_KG, KEY_KG = 0.120, 0.010, 0.005
BULB_A, BULB_H = 0.028, 0.060  # octagon apothem (5.6 cm across the flats), height
NECK = (0.024, 0.024, 0.012)
TAPER = (0.024, 0.024, 0.012)
STEM = (0.010, 0.010, 0.060)  # z -0.102 .. -0.042
PLUG_Z = -0.062  # the stopcock plug's axis crosses the stem here
PLUG_YAW = math.radians(40.0)  # the plug axis, in the bulb's frame (chosen so the key faces the right arm when vented)
BARREL = (0.024, 0.014, 0.014)  # along the plug axis
KEY_SHAFT = (0.021, 0.008, 0.008)  # plug axis 0.012 .. 0.033
KEY_PADDLE = (0.020, 0.026, 0.008)  # plug axis 0.033 .. 0.053: shut across the stem, open in line with it
STOPPER = (0.020, 0.020, 0.014)  # on the neck, z 0.042 .. 0.056
KEY_FRICTION = 0.010  # N*m: a PTFE key stays where it is turned

# -- the support -----------------------------------------------------------------------------------------------
RAIL_GAP = 0.044  # between the rails (the stem and the stopcock pass between them; the bulb rests on them)
RAIL = (0.080, 0.008, 0.008)  # along the plug axis direction
RAIL_TOP = SEAT[2] - BULB_H / 2

# -- the pressure rule -----------------------------------------------------------------------------------------
P_INVERT, P_STROKE, P_POP = 2, 1, 6
STROKE = 0.030  # a stroke: the bulb moves 3 cm or more between two turning points ...
TURN_BACK = 0.010  # ... a turning point is confirmed once it has come back 1 cm
KEY_OPEN, KEY_SHUT, KEY_CLOSED = 45.0, 20.0, 10.0  # deg: venting / shut enough to build pressure / closed at the end
VENT_UP = 0.5  # stem direction's z: at least 30 deg above horizontal to open the stopcock safely
STROKES_NEEDED = 8
SEAT_TOL = 0.010
STILL_MM, STILL_DEG = 1.0, 1.0

GLASS, LIQUID, BLACK, PTFE, GREY = (0.70, 0.82, 0.88), (0.85, 0.65, 0.25), (0.08, 0.08, 0.09), (0.93, 0.93, 0.90), \
    (0.40, 0.41, 0.43)

U = np.array([math.cos(PLUG_YAW), math.sin(PLUG_YAW), 0.0])  # plug axis (world, at spawn)
W = np.array([-math.sin(PLUG_YAW), math.cos(PLUG_YAW), 0.0])  # across the stem, horizontal, at spawn


def _octagon(apothem, height, z):
    s = 2.0 * apothem * math.tan(math.pi / 8)
    return [mk.box((2 * apothem, s, height), at=(0.0, 0.0, z), yaw=k * math.pi / 4) for k in range(4)]


kit = mk.Kit()
kit.part("funnel", tuple(SEAT), [
    *_octagon(BULB_A, BULB_H, 0.0),
    mk.box(NECK, at=(0.0, 0.0, BULB_H / 2 + NECK[2] / 2)),
    mk.box(TAPER, at=(0.0, 0.0, -BULB_H / 2 - TAPER[2] / 2), yaw=PLUG_YAW),  # square to the rails
    mk.box(STEM, at=(0.0, 0.0, -BULB_H / 2 - TAPER[2] - STEM[2] / 2)),
    mk.box(BARREL, at=(0.0, 0.0, PLUG_Z), yaw=PLUG_YAW),
], mass=FUNNEL_KG, color=GLASS)
_plug = SEAT + np.array([0.0, 0.0, PLUG_Z])
kit.part("key", tuple(_plug), [
    mk.box(KEY_SHAFT, at=(0.012 + KEY_SHAFT[0] / 2, 0.0, 0.0)),
    mk.box(KEY_PADDLE, at=(0.033 + KEY_PADDLE[0] / 2, 0.0, 0.0)),
], mass=KEY_KG, color=PTFE, yaw=PLUG_YAW)
# opening = a quarter turn about -U (the way the right arm's wrist rolls); friction holds it where it is left
kit.hinge("stopcock", "funnel", "key", anchor=tuple(_plug), axis=tuple(-U), lower_deg=0.0, upper_deg=90.0,
          start_deg=0.0, friction=KEY_FRICTION)
kit.part("stopper", tuple(SEAT + np.array([0.0, 0.0, BULB_H / 2 + NECK[2] + STOPPER[2] / 2])), [mk.box(STOPPER)],
         mass=STOPPER_KG, color=GLASS)
kit.bond("stopper_seat", "funnel", "stopper")  # let go only by the pressure rule (on_step)


def _support():
    c = SEAT[:2]
    z = RAIL_TOP - RAIL[2] / 2
    yaw = PLUG_YAW
    out = []
    for side, name in ((1, "rail_left"), (-1, "rail_right")):
        p = c + side * (RAIL_GAP / 2 + RAIL[1] / 2) * W[:2]
        out.append(Box(name, pos=(float(p[0]), float(p[1]), z), size=RAIL, yaw=yaw, color=BLACK))
    cb = c - (RAIL[0] / 2 + 0.004) * U[:2]
    out.append(Box("rail_bridge", pos=(float(cb[0]), float(cb[1]), z), size=(0.008, RAIL_GAP + 2 * RAIL[1] + 0.008,
                                                                              RAIL[2]), yaw=yaw, color=BLACK))
    post = c - (RAIL[0] / 2 + 0.014) * U[:2]
    h = RAIL_TOP - TOP
    out.append(Box("support_post", pos=(float(post[0]), float(post[1]), TOP + h / 2), size=(0.012, 0.012, h),
                   yaw=yaw, color=GREY))
    out.append(Box("support_base", pos=(float(post[0]), float(post[1]), TOP + 0.003), size=(0.070, 0.050, 0.006),
                   yaw=yaw, color=GREY))
    return out


OBJECTS = [*kit.objects(), *_support()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
_TEST_TCPS = None  # offline self-test: the TCPs to use instead of the simulator's
M = "sepfunnel"
DEBUG = False
PARTS = ("funnel", "key", "stopper")


def _mem() -> dict:
    return mk.MEMORY.setdefault(M, {"p": 0, "inverted_once": False, "strokes": 0, "anchor": None, "far": None,
                                    "popped": False, "spilled": False, "toward_op": False, "vents": 0,
                                    "last": None, "still": False, "n": 0})


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else [np.asarray(t, float) for t in _TEST_TCPS]


def released(st, margin=0.025) -> bool:
    """Neither gripper's TCP within ``margin`` (2.5 cm) of the object's bounding box."""
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())


def stem_dir(objects) -> np.ndarray:
    """Unit vector from the bulb toward the stem's tip."""
    o = kit.point(objects, "funnel", (0.0, 0.0, 0.0))
    return kit.point(objects, "funnel", (0.0, 0.0, -1.0)) - o


def key_deg(objects) -> float:
    return kit.q_deg(objects, "stopcock")


def seated(objects) -> bool:
    """Upright in the support: the bulb's centre within 1 cm of its seat (horizontally) and within 5 mm of its
    resting height, tilted under 10 deg."""
    o = kit.point(objects, "funnel", (0.0, 0.0, 0.0))
    return bool(math.hypot(o[0] - SEAT[0], o[1] - SEAT[1]) < SEAT_TOL and abs(o[2] - SEAT[2]) < 0.005
                and -stem_dir(objects)[2] > math.cos(math.radians(10.0)))


def _pose(objects, name):
    p, q = kit.pose(objects, name)
    return np.asarray(p, float), np.asarray(q, float)


def _moved(a, b) -> bool:
    (p0, q0), (p1, q1) = a, b
    ang = 2 * math.degrees(math.acos(min(1.0, abs(float(np.dot(q0, q1))))))
    return bool(np.linalg.norm(p1 - p0) * 1000 > STILL_MM or ang > STILL_DEG)


def _update(objects) -> None:
    """One control step of the pressure rule and the trial's history (every step in sim; from score() offline)."""
    mem = _mem()
    if mem.get("frame") is objects:  # offline, success() and score() see the same frame: count it once
        return
    mem["frame"] = objects
    mem["n"] += 1
    pose = {n: _pose(objects, n) for n in PARTS}
    if mem["last"] is not None:
        mem["still"] = not any(_moved(mem["last"][n], pose[n]) for n in PARTS)
    mem["last"] = pose
    if mem["popped"] or mem["spilled"]:
        return
    d = stem_dir(objects)
    k = key_deg(objects)
    bulb = pose["funnel"][0]
    inverted = d[2] > 0.0
    if k > KEY_OPEN:
        if d[2] >= VENT_UP:
            if mem["p"] > 0 or not mem["vents"]:
                mem["vents"] += 1
            mem["p"] = 0
            if d[0] < 0.0:
                mem["toward_op"] = True
        else:
            mem["spilled"] = True
            return
    if inverted and not mem["inverted_once"]:
        mem["inverted_once"] = True
        mem["p"] += P_INVERT
    if inverted and k < KEY_SHUT:
        if mem["anchor"] is None:
            mem["anchor"], mem["far"] = bulb.copy(), bulb.copy()
        else:
            a = mem["anchor"]
            if np.linalg.norm(bulb - a) > np.linalg.norm(mem["far"] - a):
                mem["far"] = bulb.copy()
            if np.linalg.norm(mem["far"] - a) >= STROKE and np.linalg.norm(bulb - mem["far"]) >= TURN_BACK:
                mem["strokes"] += 1
                mem["p"] += P_STROKE
                mem["anchor"], mem["far"] = mem["far"], bulb.copy()
    elif not inverted:
        mem["anchor"] = mem["far"] = None
    if mem["p"] >= P_POP:
        mem["popped"] = True
        if mk._EP.get("env") is not None:
            kit.release("stopper_seat")
    if DEBUG and mem["n"] % 10 == 0:
        print(f"[funnel] step {mem['n']} p {mem['p']} strokes {mem['strokes']} vents {mem['vents']} key {k:.1f} "
              f"stem {np.round(d, 2).tolist()} bulb {np.round(bulb, 3).tolist()} "
              f"tcps {[np.round(t, 3).tolist() for t in _tcps()]}", flush=True)


kit.on_step(_update)


def score(objects) -> float:
    """0 for the rest of the trial once the stopper has blown out (pressure counter reached 6) or the stopcock was
    opened with the stem less than 30 deg above horizontal (liquid run out). Otherwise 0.25 after 4 shaking strokes,
    0.5 after 8; 1.0 after 8 with the funnel back upright in its support, the stopcock shut (under 10 deg), the
    stopper in, funnel, key and stopper let go (grippers 2.5 cm clear) and none of them moved more than 1 mm or 1 deg
    over the last control step. Capped at 0.5 if it was ever vented with the stem pointing back toward the arms."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _update(objects)
    mem = _mem()
    if mem["popped"] or mem["spilled"] or kit.broken(objects, "stopper_seat"):
        return 0.0
    n = mem["strokes"]
    if n >= STROKES_NEEDED:
        done = (seated(objects) and key_deg(objects) < KEY_CLOSED and mem["still"]
                and all(released(objects[p]) for p in PARTS))
        s = 1.0 if done else 0.5
    elif n >= STROKES_NEEDED // 2:
        s = 0.25
    else:
        s = 0.0
    return min(s, 0.5) if mem["toward_op"] else s


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Strokes still to shake (as 1 cm each) plus the bulb's distance from its seat (m)."""
    mem = _mem()
    o = kit.point(objects, "funnel", (0.0, 0.0, 0.0))
    return float(0.01 * max(0, STROKES_NEEDED - mem["strokes"]) + np.linalg.norm(o - SEAT))


# -- tilted grips for the scripted demo ----------------------------------------------------------------------------
# yam_arena's Waypoint carries only a yaw about the straight-down grip. Both arms here need tilted grips, so a
# Waypoint's yaw may be a Tilt: a float (0.0) that also carries the full TCP rotation, which ArmPlan uses instead (an
# in-memory patch of yam_arena.demo.grip_rotation; plain yaws are unchanged).


class Tilt(float):
    def __new__(cls, rot):
        self = super().__new__(cls, 0.0)
        self.rot = np.asarray(rot, dtype=float)
        return self


def _patch_grip_rotation() -> None:
    if getattr(_demo, "_tilt_patch", False):
        return
    base = _demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot.copy() if rot is not None else base(yaw)

    _demo.grip_rotation = grip_rotation
    _demo._tilt_patch = True


_patch_grip_rotation()


def _frame(approach, closing):
    a = np.asarray(approach, float) / np.linalg.norm(approach)
    c = np.asarray(closing, float) - a * float(np.dot(a, closing))
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry. Left arm: grips the bulb across two flats (fingers closing along x), its
# gripper pointing right and 30 deg down; lifts the funnel out of the support and turns it over in place so the stem
# points into the hood, 35 deg above horizontal. Right arm: each vent grips the stopcock key and rolls its wrist a
# quarter turn open and back shut. Between vents the left arm shakes the funnel along x, 4 strokes at a time.

L_GRIP0 = _frame((0.0, -math.cos(math.radians(30)), -math.sin(math.radians(30))), (1.0, 0.0, 0.0))
VENT_ELEV, VENT_TWIST = math.radians(35.0), math.radians(40.0)
_d = np.array([math.cos(VENT_ELEV), 0.0, math.sin(VENT_ELEV)])  # stem direction when vented: into the hood, up
_v = np.array([0.0, 0.0, -1.0])
_ax = np.cross(_v, _d)
Q_VENT = mk.axis_angle(_d, VENT_TWIST) @ mk.axis_angle(_ax / np.linalg.norm(_ax), math.atan2(np.linalg.norm(_ax),
                                                                                                float(_v @ _d)))
L_VENT = Q_VENT @ L_GRIP0
L_HALF = _demo.ik.slerp_matrix(L_GRIP0, L_VENT, 0.5)
VENT = np.array([0.300, 0.020, TOP + 0.180])  # the bulb's centre (the left TCP) while venting and shaking
SHAKE = 0.035  # +- along x
LIFT = 0.090  # straight up out of the support: the stem's tip clears the rails
GRIP_L = -(2 * BULB_A - 0.010) / 2  # squeeze 10 mm: the held funnel must not turn in the grip
HANG = 0.003
SLIP_Y = 0.006  # the held funnel rides ~6 mm toward +y of the left TCP after the shaking (sim runs 2-3): set down short
# right arm on the key: gripper along -(plug axis), fingers closing along the stem
_u_v = Q_VENT @ U  # plug axis when vented
_plug_v = VENT + Q_VENT @ np.array([0.0, 0.0, PLUG_Z])
KEY_TCP = _plug_v + (0.034 + 0.013) * _u_v  # pad tips 1 mm onto the paddle: the pads cover all of it
R_KEY = _frame(-_u_v, _d)
R_KEY_OPEN = mk.axis_angle(-_u_v, math.radians(90.0)) @ R_KEY
R_OPEN_W = -0.020  # fingers 4 cm apart on the way in (clear of the bulb)
R_PARK_P, R_PARK_R = _ik.tcp_pose(np.array(READY), (0.0, -0.305, 0.75))
L_PARK_P, L_PARK_R = _ik.tcp_pose(np.array(READY), (0.0, 0.305, 0.75))
R_STAGE = KEY_TCP + np.array([-0.06, -0.10, 0.05])  # on the way to the key, clear of the funnel


def _vent(L):
    """The right arm's segments for one vent while the left holds the funnel at ``L`` (TCP, finger, rot)."""
    back = KEY_TCP - 0.030 * (-_u_v)
    o, c = R_OPEN_W, FINGER_CLOSED
    return [
        (1.5, "right: stage beside the funnel", L, (R_STAGE, o, R_KEY)),
        (1.2, "right: behind the stopcock key", L, (back, o, R_KEY)),
        (1.0, "right: onto the key", L, (KEY_TCP, o, R_KEY)),
        (0.8, "right: grip the key", L, (KEY_TCP, c, R_KEY)),
        (1.2, "right: turn it a quarter turn open (vent)", L, (KEY_TCP, c, R_KEY_OPEN)),
        (0.5, "vent", L, (KEY_TCP, c, R_KEY_OPEN)),
        (1.2, "right: turn it shut", L, (KEY_TCP, c, R_KEY)),
        (0.8, "right: let go of the key", L, (KEY_TCP, o, R_KEY)),
        (1.0, "right: back off", L, (back, o, R_KEY)),
        (1.5, "right: out of the way", L, (R_STAGE, o, R_KEY)),
    ]


def _shake(n=4, leg=0.5):
    out = []
    for i in range(n):
        s = 1 if i % 2 == 0 else -1
        p = VENT + np.array([s * SHAKE, 0.0, 0.0])
        out.append((leg if i else leg * 0.7, f"left: shake stroke {i + 1}", (p, GRIP_L, L_VENT), None))
    out.append((leg * 0.7, "left: back to the middle", (VENT.copy(), GRIP_L, L_VENT), None))
    out.append((0.5, "left: steady", (VENT.copy(), GRIP_L, L_VENT), None))
    return out


def _phases():
    """(duration, label, left (TCP, finger, rot) or None to hold, right (TCP, finger, rot) or None to hold)."""
    o = FINGER_OPEN
    a0 = L_GRIP0[:, 2]
    seat_tcp = SEAT + np.array([0.0, -SLIP_Y, HANG])
    up = SEAT + np.array([0.0, 0.0, LIFT])
    held = (VENT.copy(), GRIP_L, L_VENT)
    P = [
        (2.0, "left: behind the bulb", (SEAT - 0.060 * a0, o, L_GRIP0), (R_STAGE, R_OPEN_W, R_KEY)),
        (1.5, "left: fingers round the bulb", (SEAT.copy(), o, L_GRIP0), None),
        (0.5, "settle", (SEAT.copy(), o, L_GRIP0), None),
        (1.0, "left: grip the bulb", (SEAT.copy(), GRIP_L, L_GRIP0), None),
        (0.5, "settle", (SEAT.copy(), GRIP_L, L_GRIP0), None),
        (1.5, "left: lift it out of the support", (up, GRIP_L, L_GRIP0), None),
        (1.8, "left: toward the venting spot, turning it on its side", ((up + VENT) / 2, GRIP_L, L_HALF), None),
        (1.8, "left: turned over, stem up into the hood", held, None),
        (0.5, "steady", held, None),
    ]
    P += _vent(held)
    P += _shake()
    P += _vent(held)
    P += _shake()
    P += _vent(held)
    P += [
        (1.8, "left: back toward the support, turning it on its side", ((up + VENT) / 2, GRIP_L, L_HALF),
         (R_PARK_P, R_OPEN_W, R_PARK_R)),
        (1.8, "left: upright over the support", (up, GRIP_L, L_GRIP0), None),
        (1.5, "left: lower it onto the rails", (seat_tcp + np.array([0.0, 0.0, 0.010]), GRIP_L, L_GRIP0), None),
        (1.0, "left: down slowly", (seat_tcp, GRIP_L, L_GRIP0), None),
        (0.5, "settle", (seat_tcp, GRIP_L, L_GRIP0), None),
        (1.0, "left: let go", (seat_tcp, o, L_GRIP0), None),
        (1.5, "left: back out", (SEAT - 0.070 * a0, o, L_GRIP0), None),
    ]
    return P


def demo():
    """Left: lift the funnel out of its support and turn it over, stem up into the hood. Right: vent (key a quarter
    turn open and shut). Left: four strokes. Vent, four strokes, vent. Left: turn it upright, set it in the support,
    let go."""
    L, R = [], []
    lcur = (L_PARK_P, o := FINGER_OPEN, L_PARK_R)
    rcur = (R_PARK_P, o, R_PARK_R)
    for dur, label, lw, rw in _phases():
        lcur = lw if lw is not None else lcur
        rcur = rw if rw is not None else rcur
        for arm, (p, f, r) in (("L", lcur), ("R", rcur)):
            (L if arm == "L" else R).append(Waypoint(dur, tuple(float(v) for v in p), float(f), label, Tilt(r)))
    return {"left": L, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of states (the kit's forward kinematics poses the parts)."""
    global _TEST_TCPS
    R0 = np.eye(3)

    def pose(centre, rot):
        """The funnel's origin (the bulb's centre) at ``centre`` with rotation ``rot`` relative to its spawn."""
        return (np.asarray(centre, float), mk.mat_to_quat(rot))

    def st(centre=SEAT, rot=R0, key=0.0, popped=False):
        return kit.fake_states(q={"stopcock": key}, deg=True, moved={"funnel": pose(centre, rot)},
                               broken=("stopper_seat",) if popped else ())

    away = [(0.15, 0.40, TOP + 0.35), (0.15, -0.40, TOP + 0.35)]
    hold = [SEAT.tolist(), away[1]]
    vh = [VENT.tolist(), away[1]]
    up = SEAT + np.array([0.0, 0.0, LIFT])
    turned = Q_VENT

    def shake(n, start=0):
        out = []
        for i in range(start, start + n):
            s = 1 if i % 2 == 0 else -1
            out.append((st(VENT + np.array([s * SHAKE, 0, 0]), turned), vh))
        return out

    def vent():
        return [(st(VENT, turned, 90.0), vh), (st(VENT, turned, 0.0), vh)]

    start = [(st(), away)]
    out_ = [(st(), hold), (st(up), hold), (st(VENT), vh), (st(VENT, turned), vh)]
    home = [(st(VENT), vh), (st(up), hold), (st(), hold), (st(), away), (st(), away)]
    demo_seq = out_ + vent() + shake(4) + [(st(VENT, turned), vh)] + vent() + shake(4, 0) + \
        [(st(VENT, turned), vh)] + vent() + home
    # turned with the stem pointing back toward the arms (rotated 180 deg about the vertical)
    back = mk.axis_angle((0, 0, 1), math.pi) @ turned
    sideways = mk.axis_angle((0, 1, 0), VENT_ELEV) @ turned  # stem level (horizontal)
    cases = {
        "start": (start, 0.0, False),
        "lifted and turned over, not shaken": (out_, 0.0, False),
        "the demo: vent, 4 strokes, vent, 4 strokes, vent, back in the support, let go": (demo_seq, 1.0, True),
        "the demo but left holding it": (demo_seq[:-2] + [(st(), hold)], 0.5, False),
        "the demo but the stopcock left open in the support": (
            demo_seq[:-5] + [(st(VENT), vh), (st(up), hold), (st(key=50.0), hold)], 0.0, False),
        "the demo but stood back in the support 2 cm off the seat": (
            demo_seq[:-3] + [(st(SEAT + np.array([0.02, 0, 0])), away), (st(SEAT + np.array([0.02, 0, 0])), away)],
            0.5, False),
        "naive: turned over and shaken 8 strokes, never vented: stopper blows at the 4th": (
            out_ + shake(8) + home, 0.0, False),
        "vented once after turning over, then 8 strokes straight: blows at the 6th": (
            out_ + vent() + shake(8) + home, 0.0, False),
        "shaken upright 8 strokes (not counted), then back": (
            [(st(), hold), (st(up), hold)] + [(st(VENT + np.array([(1 if i % 2 == 0 else -1) * SHAKE, 0, 0])), vh)
                                              for i in range(8)] + home, 0.0, False),
        "opened upright in the support (drains the lower layer)": (
            [(st(), hold), (st(key=60.0), hold), (st(), hold)] + out_ + vent() + shake(4) + vent() + shake(4) +
            vent() + home, 0.0, False),
        "opened with the stem level (liquid out of the stem)": (
            out_ + [(st(VENT, sideways, 60.0), vh), (st(VENT, sideways, 0.0), vh)] + shake(4) + vent() + shake(4) +
            vent() + home, 0.0, False),
        "vented with the stem pointing back at the arms: capped at 0.5": (
            out_ + [(st(VENT, back, 90.0), vh), (st(VENT, back, 0.0), vh)] + vent() + shake(4) + vent() + shake(4) +
            vent() + home, 0.5, False),
        "only 4 strokes, then back in the support": (out_ + vent() + shake(4) + vent() + home, 0.25, False),
        "4 strokes with the stopcock open do not count (vented all along)": (
            out_ + vent() + [(st(VENT + np.array([(1 if i % 2 == 0 else -1) * SHAKE, 0, 0]), turned, 90.0), vh)
                             for i in range(8)] + [(st(VENT, turned, 0.0), vh)] + home, 0.0, False),
        "small 2 cm jiggles are not strokes": (
            out_ + vent() + [(st(VENT + np.array([(1 if i % 2 == 0 else -1) * 0.010, 0, 0]), turned), vh)
                             for i in range(12)] + home, 0.0, False),
        "a new trial after a blown stopper starts clean": (out_ + shake(8) + [None] + demo_seq, 1.0, True),
        "stopper out (bond broken) at the end": (demo_seq[:-1] + [(st(popped=True), away)], 0.0, False),
    }
    n = 0
    try:
        for name, (frames, want, want_success) in cases.items():
            mk.new_episode()
            got = got_success = None
            for f in frames:
                if f is None:
                    mk.new_episode()
                    continue
                states, tcps = dict(f[0]), f[1]
                _TEST_TCPS = tcps
                got_success, got = success(states), score(states)
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success,
                                                                           {k: v for k, v in _mem().items()
                                                                            if k not in ("frame", "last")})
            n += 1
    finally:
        _TEST_TCPS = None
        mk.new_episode()
    return n


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
        assert np.all((q[:, 6] >= 0.0) & (q[:, 6] <= 1.0)), arm
        jump = float(np.max(np.abs(np.diff(q[:, :6], axis=0))))
        assert jump < 0.25, (arm, jump)
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg, "
              f"largest joint step {math.degrees(jump):.1f} deg")
