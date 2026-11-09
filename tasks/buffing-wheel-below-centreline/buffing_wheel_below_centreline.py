"""On the station table in front of the arms stands a double-ended bench buffer, running: a grey cast-iron motor
between the arms with a spindle along y, and a 150 mm cotton buffing wheel (dressed with compound) on each end,
their front faces running DOWN toward the robot's side of the bench. Below the front of each wheel is a steel work
rest, its back edge 3 mm off the wheel and its top 3.7 cm above the bench (about 40 deg below the wheel's centre).
In a steel rack in front of the right wheel stands an aluminium flat-bar blank, 100 x 40 x 10 mm, on its long edge,
one face scribed with a small dark mark. The job: polish both faces of the blank on the buffer, then stand it back in
its rack.

Physics, not a rule, decides how: the wheel drags whatever touches it along its surface (Coulomb friction, 1.0). Held
with its lower edge on the work rest and its face on the wheel below the centreline, the drag runs down the blank
into the rest, which takes it. Held anywhere the rest cannot take the drag (above the centreline, on the top of the
wheel, off the rest on the up-running side) the same drag, acting a few cm from the fingers, twists the blank in the
pinch and snatches it out of position (tried in sim: twisted 64 deg within half a second at a 3-5 N press above the
centreline; on the rest below it, under 8 deg at up to 8 N). A blank that turns more than 25 deg in the gripper
while it is at a wheel, or is flung (0.5 m/s), scores 0 for the trial.

Mechanism parts come from mechkit.py (in this folder), which adds hinges to the stock runtime in memory. Each wheel
is a 36-sided rigid body on a PhysX revolute joint with a velocity drive (2 rev/s target, 0.5 N*m*s/rad).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/buffing-wheel-below-centreline/buffing_wheel_below_centreline.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinges and per-episode hooks, installed in memory)

from yam_arena import demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Right arm base (0, -0.305, 0.75), left (0, +0.305, 0.75).
R, WW = 0.075, 0.025  # buffing wheel radius and width (150 x 25 mm)
NSEG = 36  # the wheel's rim: 36 flat segments
XW, ZC = 0.360, TOP + 0.100  # spindle axis (along y)
YW = {"right": -0.200, "left": 0.200}  # wheel centres
SPEED_DEG = 720.0  # drive target: 2 rev/s, front face running down
DRIVE_DAMP = 0.5  # N*m*s/rad velocity drive
WHEEL_KG = 0.30
MOTOR = (0.120, 0.160, 0.130)

PLATE = (0.010, 0.100, 0.040)  # aluminium flat-bar blank: thickness (x), length (y), height (z) in its stand
PLATE_KG = 0.108
STAND_X, STAND_Y = 0.190, -0.200  # the blank's centre in its rack
SLOT_GAP = 0.002  # rack bars 2 mm off each face

GAP = 0.003  # work rest to wheel (OSHA 1910.215 limit: 1/8 in)
REST_D, REST_W, REST_T = 0.060, 0.060, 0.006
CONTACT_DEG = -25.0  # the demo's contact: 25 deg below the centreline
CONTACT_LZ = 0.010  # ... 3 cm up the blank from its lower edge

# scoring
POLISH_S = 3.0  # seconds of contact needed on each face
DT = 1.0 / 20.0  # one control step at 20 Hz
SNATCH_DEG = 25.0  # turn of the blank in the gripper while at a wheel
FLING_MS = 0.5
NEAR_M = 0.020  # "at a wheel": a corner of the blank within 2 cm of a wheel's rim
CONTACT_IN, CONTACT_OUT = 0.004, 0.0025  # face plane this far inside / outside the rim's nominal radius

WHITE = (0.93, 0.90, 0.80)
COMPOUND = (0.78, 0.62, 0.36)
IRON = (0.42, 0.45, 0.48)
STEEL = (0.30, 0.31, 0.33)
ALU = (0.80, 0.82, 0.85)
MARK = (0.15, 0.15, 0.17)
RED = (0.75, 0.15, 0.12)


def _wheel_boxes():
    """A 36-sided cotton buff: rim segments (circumradius R) round a filled centre, flange and arbor stub."""
    a = R * math.cos(math.pi / NSEG)
    seg = 2 * R * math.sin(math.pi / NSEG) * 1.02
    rim = 0.012
    out = []
    for i in range(NSEG):
        t = 2 * math.pi * i / NSEG
        q = tuple(mk.mat_to_quat(mk.axis_angle((0, 1, 0), t)))
        out.append(mk.box((seg, WW, rim), at=((a - rim / 2) * math.sin(t), 0.0, (a - rim / 2) * math.cos(t)),
                          quat=q, color=WHITE if i % 3 else COMPOUND))
    s = (a - rim) * math.sqrt(2) * 0.98
    for k in range(3):
        q = tuple(mk.mat_to_quat(mk.axis_angle((0, 1, 0), k * math.pi / 6)))
        out.append(mk.box((s, WW * 0.9, s), quat=q, color=WHITE))
    return out


kit = mk.Kit()
kit.part("buffer", (XW, 0.0, ZC), [
    mk.box(MOTOR),
    mk.box((0.160, 0.200, ZC - TOP - MOTOR[2] / 2), at=(0, 0, -(ZC - TOP) / 2 - MOTOR[2] / 4)),
    mk.box((0.004, 0.030, 0.020), at=(-MOTOR[0] / 2 - 0.002, 0.0, -0.030), color=RED),  # switch
], color=IRON)
for side, yw in YW.items():
    reach = abs(yw) - MOTOR[1] / 2 - WW / 2
    kit.part(f"wheel_{side}", (XW, yw, ZC), [
        *_wheel_boxes(),
        mk.box((0.016, reach + 0.002, 0.016), at=(0, -math.copysign(reach / 2 + WW / 2, yw), 0), color=STEEL),
    ], mass=WHEEL_KG, color=WHITE)
    # axis -y: a positive turn carries the wheel's front (robot-side) face downward, its top toward the robot
    kit.hinge(f"spindle_{side}", "buffer", f"wheel_{side}", anchor=(XW, yw, ZC), axis=(0, -1, 0),
              lower_deg=-math.inf, upper_deg=math.inf, damping=DRIVE_DAMP)
kit.part("blank", (STAND_X, STAND_Y, TOP + PLATE[2] / 2), [
    mk.box(PLATE),
    mk.box((0.0004, 0.012, 0.008), at=(-PLATE[0] / 2 - 0.0002, -0.035, -0.012), color=MARK),  # scribed mark: face B
], mass=PLATE_KG, color=ALU)


def _contact_rot() -> np.ndarray:
    """The blank leaning back 25 deg (its face toward the wheel tilted 25 deg upward)."""
    return mk.axis_angle((0, 1, 0), math.radians(CONTACT_DEG))


def _rest_geometry():
    """Rest top = the blank's lowest corner when it touches the wheel 25 deg below the centreline; back edge GAP off
    the wheel at that height."""
    q = _contact_rot()
    b = math.radians(CONTACT_DEG)
    t = np.array([XW, 0.0, ZC]) + R * np.array([-math.cos(b), 0.0, math.sin(b)])
    p = t - q @ np.array([PLATE[0] / 2, 0.0, CONTACT_LZ])
    low = min((p + q @ np.array([sx * PLATE[0] / 2, 0.0, sz * PLATE[2] / 2]) for sx in (-1, 1) for sz in (-1, 1)),
              key=lambda c: c[2])
    zr = float(low[2])
    return zr, XW - math.sqrt(R * R - (ZC - zr) ** 2) - GAP


REST_TOP, REST_BACK = _rest_geometry()


def _fixtures():
    out = []
    for side, yw in YW.items():
        out += [
            Box(f"rest_{side}", pos=(REST_BACK - REST_D / 2, yw, REST_TOP - REST_T / 2), size=(REST_D, REST_W, REST_T),
                color=STEEL),
            Box(f"rest_post_{side}", pos=(REST_BACK - REST_D + 0.008, yw, (TOP + REST_TOP - REST_T) / 2),
                size=(0.012, 0.020, REST_TOP - REST_T - TOP), color=STEEL),
        ]
    t = PLATE[0] / 2 + SLOT_GAP + 0.005
    out += [Box(f"rack_bar_{n}", pos=(STAND_X + s * t, STAND_Y, TOP + 0.005), size=(0.010, 0.110, 0.010), color=STEEL)
            for n, s in (("near", -1), ("far", 1))]
    return out


FIXTURES = _fixtures()
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the running wheels ------------------------------------------------------------------------------------------

_DRIVE = {"set": False}


def _run_wheels() -> None:
    """Give both spindle drives their target speed (sim only; the USD attribute persists across resets)."""
    if _DRIVE["set"] or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    for side in YW:
        path = mk._SIM["joint_paths"].get((kit.id, f"spindle_{side}"))
        if path:
            UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "angular").GetTargetVelocityAttr().Set(SPEED_DEG)
    _DRIVE["set"] = True


# -- measurements (poses only; on a real rig: the trial video) ------------------------------------------------

_TEST_GRIPS = None  # self-test stand-in: list of (tcp, 3x3 gripper frame)
_DWELL, _SNATCH, _FLUNG, _REF = "bw_dwell", "bw_snatched", "bw_flung", "bw_ref"


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def _grips() -> list:
    """Both grippers' (TCP, frame): columns = closing axis, its normal, approach axis. [] offline."""
    if _TEST_GRIPS is not None:
        return _TEST_GRIPS
    env = mk._EP["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        raw = art.data.body_pos_w
        pos = (getattr(raw, "torch", raw)[0] - origin).detach().cpu().numpy().astype(float)
        g, tl, tr = (pos[names.index(n)] for n in ("gripper", "tip_left", "tip_right"))
        a = _unit((tl + tr) / 2 - g)
        c = (tr - tl) - float((tr - tl) @ a) * a
        c = _unit(c) if np.linalg.norm(c) > 1e-4 else _unit(np.cross(a, (0.0, 0.0, 1.0)))
        out.append((g + a * mk.TCP_PAST_GRIPPER, np.column_stack([c, np.cross(a, c), a])))
    return out


def _frame(objects):
    p, q = kit.pose(objects, "blank")
    return np.asarray(p, float), mk.quat_to_mat(q)


def _corners(p, r):
    h = np.asarray(PLATE) / 2
    return [p + r @ (h * np.array([sx, sy, sz])) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def _near_wheel(p, r) -> bool:
    """A corner of the blank within NEAR_M of a wheel's rim (inside its width + 5 cm)."""
    for c in _corners(p, r):
        for yw in YW.values():
            if abs(c[1] - yw) < WW / 2 + 0.05 and math.hypot(c[0] - XW, c[2] - ZC) < R + NEAR_M:
                return True
    return False


def _faces_in_contact(p, r) -> set:
    """Faces ('A' = unmarked +x face, 'B' = marked -x face) lying on a wheel's rim: the face plane within the rim's
    radius band, square to the spindle, and the tangent line inside the face across at least two of three points
    over the wheel's width."""
    out = set()
    for name, s in (("A", 1.0), ("B", -1.0)):
        n = s * r[:, 0]
        if abs(n[1]) > 0.25:
            continue
        c = p + n * PLATE[0] / 2
        for yw in YW.values():
            hits = 0
            for dy in (-0.010, 0.0, 0.010):
                w = np.array([XW, yw + dy, ZC])
                d = float(n @ (w - c))
                if not (R - CONTACT_IN <= d <= R + CONTACT_OUT):
                    continue
                loc = r.T @ (w - d * n - p)
                if abs(loc[1]) <= PLATE[1] / 2 - 0.001 and abs(loc[2]) <= PLATE[2] / 2 - 0.001:
                    hits += 1
            if hits >= 2:
                out.add(name)
    return out


def _rel_angle(a, b) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(a @ b.T) - 1.0) / 2.0))))


def _track(objects) -> None:
    """Every control step: polishing time per face; latched snatch (the blank turns > 25 deg in the gripper holding
    it while it is at a wheel) and fling (0.5 m/s). Cleared only by the episode reset."""
    _run_wheels()
    if "blank" not in objects:
        return
    p, r = _frame(objects)
    st = objects["blank"]
    if float(np.linalg.norm(st.lin_vel)) > FLING_MS:
        mk.MEMORY[_FLUNG] = True
    dwell = mk.MEMORY.setdefault(_DWELL, {"A": 0.0, "B": 0.0})
    for f in _faces_in_contact(p, r):
        dwell[f] += DT
    near = _near_wheel(p, r)
    refs = mk.MEMORY.setdefault(_REF, {})
    for i, (tcp, g) in enumerate(_grips()):
        lo, hi = np.asarray(st.low) - 0.025, np.asarray(st.high) + 0.025
        held = bool(np.all(np.asarray(tcp) >= lo) and np.all(np.asarray(tcp) <= hi))
        if not (held and near):
            refs.pop(i, None)
            continue
        rel = g.T @ r
        if i not in refs:
            refs[i] = rel
        elif _rel_angle(rel, refs[i]) > SNATCH_DEG:
            mk.MEMORY[_SNATCH] = True


kit.on_step(_track)


def _in_rack(objects) -> bool:
    """Standing on its long edge between the rack bars (either face forward), upright within 10 deg."""
    p, r = _frame(objects)
    up = abs(float(r[2, 2]))
    along = abs(float(r[1, 1]))
    st = objects["blank"]
    return bool(abs(p[0] - STAND_X) < 0.006 and abs(p[1] - STAND_Y) < 0.020 and up > math.cos(math.radians(10))
                and along > math.cos(math.radians(10)) and float(st.low[2]) < TOP + 0.004)


def _released(st, margin=0.025) -> bool:
    if _TEST_GRIPS is not None:
        lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
        return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t, _ in _TEST_GRIPS)
    return mk.released(st, margin)


def polished(objects) -> dict:
    d = mk.MEMORY.get(_DWELL, {"A": 0.0, "B": 0.0})
    return {f: d[f] >= POLISH_S - 1e-9 for f in ("A", "B")}


def score(objects) -> float:
    """0 for the rest of the trial once the blank was snatched (turned > 25 deg in the gripper while at a wheel) or
    flung (0.5 m/s). Otherwise 0.25 per face polished (3 s of contact on the wheel's rim); 1.0 with both faces
    polished and the blank back in its rack, let go and still."""
    if mk.MEMORY.get(_SNATCH) or mk.MEMORY.get(_FLUNG):
        return 0.0
    pol = polished(objects)
    st = objects["blank"]
    if all(pol.values()) and _in_rack(objects) and st.resting() and _released(st):
        return 1.0
    return 0.25 * sum(pol.values())


def success(objects) -> bool:
    """Both faces polished, the blank back in its rack, let go, no snatch; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Seconds of polishing still owed (both faces), plus metres from the blank to its rack."""
    d = mk.MEMORY.get(_DWELL, {"A": 0.0, "B": 0.0})
    owed = sum(max(0.0, POLISH_S - d[f]) for f in ("A", "B"))
    p, _ = _frame(objects)
    return float(owed + math.hypot(p[0] - STAND_X, p[1] - STAND_Y))


# -- tilted grips for the scripted demo ----------------------------------------------------------------------------
# yam_arena's Waypoint carries only a yaw about the straight-down grip. The demo tilts that grip about y, so a
# Waypoint's yaw may be a Tilt: a float (0.0) that also carries the full TCP rotation, which ArmPlan uses instead (an
# in-memory patch of yam_arena.demo.grip_rotation; plain yaws unchanged).


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

# -- the scripted demo ------------------------------------------------------------------------------------------
# Right arm only, open loop from the scene's fixed geometry (it reads no joint, wheel or latch state). The fingers
# close across the blank's thickness at one end (pads over the top 2.1 cm). Face A: the pinch outboard of the right
# wheel, the blank tilted back 25 deg, set down onto the rest and slid in until its face meets the wheel 25 deg
# below the centreline, pressed (30 mm command, about 5 N through the arm's compliance) for 4 s. Then it is backed
# off, turned 180 deg in the hand about the vertical (well clear of the wheel), and face B is done the same way with
# the pinch inboard. Last it goes back in the rack, turned round (the rack takes it either way).

TCP_BELOW_TOP = 0.008
SQUEEZE = 0.008
LEVER = 0.061  # pinch centre to the wheel's centre plane, along the blank
PRESS = 0.030
PRESS_S = 4.0


def _contact_centre(yw: float, sgn: float) -> np.ndarray:
    """Blank centre with its working face tangent to the wheel at 25 deg below the centreline (no press). ``sgn``
    +1: pinch outboard (toward -y of the right wheel), face A working; -1: turned round, pinch inboard, face B."""
    q = _contact_rot()
    b = math.radians(CONTACT_DEG)
    t = np.array([XW, yw, ZC]) + R * np.array([-math.cos(b), 0.0, math.sin(b)])
    p = t - q @ np.array([PLATE[0] / 2, 0.0, CONTACT_LZ])
    p[1] = yw - sgn * (LEVER - 0.031)
    return p


def _tcp_at(p, sgn: float) -> np.ndarray:
    """TCP for blank centre ``p``: at the pinch end, 8 mm below the top edge, in the blank's mid-plane."""
    q = _contact_rot()
    return p + q @ np.array([0.0, 0.0, PLATE[2] / 2 - TCP_BELOW_TOP]) + np.array([0.0, -sgn * 0.031, 0.0])


def _faces_plan(yaw_a, yaw_b) -> list:
    yw = YW["right"]
    tilt_a = Tilt(_contact_rot() @ _demo.grip_rotation(yaw_a))
    tilt_b = Tilt(_contact_rot() @ _demo.grip_rotation(yaw_b))
    g = -(PLATE[0] - SQUEEZE) / 2
    out = []
    for sgn, tilt, face in ((1.0, tilt_a, "A"), (-1.0, tilt_b, "B")):
        p0 = _contact_centre(yw, sgn)

        def at(dx, dz, p0=p0, sgn=sgn):
            return tuple(float(v) for v in _tcp_at(p0 + np.array([dx, 0.0, dz]), sgn))

        if face == "B":
            mid = Tilt(_demo.grip_rotation(0.0))
            hi = (0.200, yw - 0.070, TOP + 0.130)  # well clear of the wheel, the rest and the rack: room for the half turn
            out += [Waypoint(1.5, hi, g, "clear of the wheel, up", tilt_a),
                    Waypoint(1.5, hi, g, "turn the blank in the hand (half)", mid),
                    Waypoint(1.5, hi, g, "turn the blank in the hand: face B toward the wheel", tilt_b)]
        out += [
            Waypoint(2.5, at(-0.025, 0.020), g, f"face {face}: over the work rest", tilt),
            Waypoint(1.5, at(-0.022, 0.003), g, "lower the edge onto the rest", tilt),
            Waypoint(2.0, at(0.004, 0.001), g, "slide in to the wheel, below the centreline", tilt),
            Waypoint(1.0, at(PRESS, -0.002), g, "press", tilt),
            Waypoint(PRESS_S, at(PRESS, -0.002), g, f"polish face {face}", tilt),
            Waypoint(1.0, at(-0.025, 0.004), g, "back off the wheel", tilt),
        ]
    return out


def demo():
    """Pick the blank from its rack, polish face A then face B on the right wheel's lower front quadrant with the
    blank's edge on the work rest, and stand it back in the rack. About 47 s."""
    o = FINGER_OPEN
    g = -(PLATE[0] - SQUEEZE) / 2
    ya, yb = -math.pi / 2, math.pi / 2  # grip yaw for face A, then (turned in the hand) face B: joint6 in range
    gz = TOP + PLATE[2] - TCP_BELOW_TOP
    py = STAND_Y - 0.031  # the pinch end (local -y)
    qy = STAND_Y + 0.031  # the same end after the half turn
    up = TOP + 0.120
    return {"right": [
        Waypoint(2.0, (STAND_X, py, gz + 0.060), o, "over the blank's end", ya),
        Waypoint(1.5, (STAND_X, py, gz), o, "descend", ya),
        Waypoint(0.5, (STAND_X, py, gz), o, "settle", ya),
        Waypoint(1.0, (STAND_X, py, gz), g, "close across the blank", ya),
        Waypoint(0.5, (STAND_X, py, gz), g, "settle closed", ya),
        Waypoint(1.5, (STAND_X, py, up), g, "lift out of the rack", ya),
        *_faces_plan(ya, yb),
        Waypoint(1.5, (STAND_X + 0.040, qy, up), g, "up, clear of the rest", yb),
        Waypoint(1.5, (STAND_X, qy, up), g, "over the rack", yb),
        Waypoint(1.5, (STAND_X, qy, gz + 0.020), g, "lower into the rack", yb),
        Waypoint(1.0, (STAND_X, qy, gz + 0.002), g, "set down", yb),
        Waypoint(0.5, (STAND_X, qy, gz + 0.002), g, "settle", yb),
        Waypoint(1.0, (STAND_X, qy, gz + 0.002), o, "release", yb),
        Waypoint(1.5, (STAND_X, qy, TOP + 0.110), o, "retreat", yb),
    ]}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of states (the blank posed by the kit; grippers given by hand)."""
    global _TEST_GRIPS
    import dataclasses

    assert REST_BACK < XW - math.sqrt(R * R - (ZC - REST_TOP) ** 2) - 0.002, "rest must clear the wheel"
    rot_c = _contact_rot()
    rot_turn = rot_c @ mk.axis_angle((0, 0, 1), math.pi)  # turned round, leaning back: face B toward the wheel
    p_a = _contact_centre(YW["right"], 1.0)
    p_b = _contact_centre(YW["right"], -1.0)
    up_b = mk.axis_angle((0, 1, 0), math.radians(25.0))  # leaning forward over the top front (above the centreline)
    bb = math.radians(25.0)
    t_hi = np.array([XW, YW["right"], ZC]) + R * np.array([-math.cos(bb), 0.0, math.sin(bb)])
    p_hi = t_hi - up_b @ np.array([PLATE[0] / 2, 0.0, 0.0])
    p_hi[1] = YW["right"] - 0.030
    top_r = mk.axis_angle((0, 1, 0), math.pi / 2)  # lying flat on the top of the wheel, face A down
    p_top = np.array([XW, YW["right"] - 0.030, ZC + R + PLATE[0] / 2])
    g_a = rot_c @ _demo.grip_rotation(math.pi / 2)
    g_b = rot_c @ _demo.grip_rotation(-math.pi / 2)

    def st(pos, rot, vel=(0.0, 0.0, 0.0)):
        q = tuple(mk.mat_to_quat(rot))
        out = kit.fake_states(moved={"blank": (np.asarray(pos, float), q)})
        b = out["blank"]
        out["blank"] = dataclasses.replace(b, lin_vel=np.asarray(vel, float))
        return out

    for p, r in ((p_a, rot_c), (p_b, rot_turn)):
        got_p, got_r = _frame(st(p, r))
        assert np.allclose(got_p, p, atol=1e-6) and _rel_angle(got_r, r) < 1e-3
    start = st((STAND_X, STAND_Y, TOP + PLATE[2] / 2), np.eye(3))
    in_rack_turned = st((STAND_X, STAND_Y, TOP + PLATE[2] / 2), mk.axis_angle((0, 0, 1), math.pi))
    sliding = st((STAND_X, STAND_Y, TOP + PLATE[2] / 2), mk.axis_angle((0, 0, 1), math.pi), vel=(0.03, 0.0, 0.0))
    assert _faces_in_contact(*_frame(st(p_a, rot_c))) == {"A"}, _faces_in_contact(*_frame(st(p_a, rot_c)))
    assert _faces_in_contact(*_frame(st(p_b, rot_turn))) == {"B"}
    assert _faces_in_contact(*_frame(st(p_a + np.array([-0.006, 0, 0]), rot_c))) == set(), "6 mm off is not contact"
    assert _faces_in_contact(*_frame(st(p_hi, up_b))) == {"A"}
    assert _faces_in_contact(*_frame(st(p_top, top_r))) == {"A"}
    assert _faces_in_contact(*_frame(start)) == set() and not _near_wheel(*_frame(start))

    def twisted(p, r, deg):
        """The blank turned ``deg`` in its own plane about the pinch (the wheel's drag), pinch fixed."""
        pin = p + r @ np.array([0.0, -0.031, 0.012])
        tw = mk.axis_angle(r[:, 0], math.radians(-deg))
        return st(pin + tw @ (p - pin), tw @ r)

    tcp_a = _tcp_at(p_a, 1.0)
    tcp_b = _tcp_at(p_b, -1.0)
    held_a, held_b = [(tcp_a, g_a)], [(tcp_b, g_b)]
    clear = [(np.array([0.10, -0.25, TOP + 0.30]), np.eye(3))]
    in_hand = [(np.array([STAND_X, STAND_Y + 0.031, TOP + PLATE[2] - TCP_BELOW_TOP]), _demo.grip_rotation(-math.pi / 2))]

    def polish(p, r, grips, n):
        return [(st(p, r), grips)] * n

    n3 = int(round(POLISH_S / DT))
    face_a = polish(p_a, rot_c, held_a, n3 + 2)
    face_b = polish(p_b, rot_turn, held_b, n3 + 2)
    rack_end = [(in_rack_turned, clear), (in_rack_turned, clear)]
    cases = {  # name: (frames [(states, grips)], score, success)
        "start": ([(start, clear)], 0.0, False),
        "face A polished 3 s (rest, below the centreline)": (face_a, 0.25, False),
        "face A only 2 s": (polish(p_a, rot_c, held_a, n3 - 20), 0.0, False),
        "both faces polished, still held": (face_a + face_b, 0.5, False),
        "both faces, back in the rack, let go": (face_a + face_b + rack_end, 1.0, True),
        "both faces, in the rack but still gripped": (face_a + face_b + [(in_rack_turned, in_hand)], 0.5, False),
        "both faces, in the rack, still moving": (face_a + face_b + [(sliding, clear)], 0.5, False),
        "one face twice as long, other face never, back in rack": (face_a + face_a + rack_end, 0.25, False),
        "polishing on the rest with a 5 deg settle in the grip": (face_a[:5] + [(twisted(p_a, rot_c, 5.0), held_a)] *
                                                                   (n3 + 2) + face_b + rack_end, 1.0, True),
        "naive: face A held above the centreline, snatched 64 deg": (
            polish(p_hi, up_b, [(_tcp_at(p_hi, 1.0), up_b @ _demo.grip_rotation(math.pi / 2))], 10)
            + [(twisted(p_hi, up_b, 64.0), [(_tcp_at(p_hi, 1.0), up_b @ _demo.grip_rotation(math.pi / 2))])], 0.0,
            False),
        "naive top of the wheel: snatched, then both faces done properly": (
            polish(p_top, top_r, [(p_top + np.array([0, -0.031, 0]), top_r @ _demo.grip_rotation(math.pi / 2))],
                   n3 + 2) + [(twisted(p_top, top_r, 40.0), [(p_top + np.array([0, -0.031, 0]),
                                                              top_r @ _demo.grip_rotation(math.pi / 2))])]
            + face_a + face_b + rack_end, 0.0, False),
        "snatch, then twisted back by hand: still 0": (face_a[:3] + [(twisted(p_a, rot_c, 40.0), held_a)] + face_a +
                                                       face_b + rack_end, 0.0, False),
        "flung off the wheel (0.8 m/s)": (face_a + [(st(p_a + np.array([-0.05, 0, -0.05]), rot_c, vel=(-0.8, 0, 0)),
                                                     clear)] + face_b + rack_end, 0.0, False),
        "turned 180 deg in the hand away from the wheel: no snatch": (
            face_a + [(st(p_a + np.array([-0.12, 0, 0.06]), rot_c), [(tcp_a + np.array([-0.12, 0, 0.06]), g_a)]),
                      (st(p_a + np.array([-0.12, 0, 0.06]), rot_turn),
                       [(tcp_a + np.array([-0.12, 0, 0.06]), g_b)])] + face_b + rack_end, 1.0, True),
        "whole blank turning with the gripper at the wheel (wrist, not slip)": (
            face_a[:3] + [(twisted(p_a, rot_c, 30.0), [(tcp_a, mk.axis_angle(rot_c[:, 0], math.radians(-30.0)) @ g_a)])]
            + face_a + face_b + rack_end, 1.0, True),
        "faces done, put down on the table not the rack": (face_a + face_b + [(st((0.15, -0.10, TOP + 0.005),
                                                                                 mk.axis_angle((0, 1, 0), math.pi / 2)),
                                                                              clear)] * 2, 0.5, False),
        "a new trial after a snatch starts clean": (polish(p_hi, up_b, [(_tcp_at(p_hi, 1.0), up_b @ _demo.grip_rotation(
            math.pi / 2))], 3) + [(twisted(p_hi, up_b, 64.0), [(_tcp_at(p_hi, 1.0), up_b @ _demo.grip_rotation(
                math.pi / 2))])] + [None] + face_a + face_b + rack_end, 1.0, True),
    }
    try:
        for name, (frames, want, want_success) in cases.items():
            mk.new_episode()
            for f in frames[:-1]:
                if f is None:
                    mk.new_episode()
                    continue
                _TEST_GRIPS = f[1]
                _track(f[0])
            _TEST_GRIPS = frames[-1][1]
            _track(frames[-1][0])
            got, got_success = score(frames[-1][0]), success(frames[-1][0])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    finally:
        mk.new_episode()
        _TEST_GRIPS = None
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the right arm's plan replayed through the
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
