"""On the station table lies a child-size airway and ventilation training manikin, face up, its head in front of the
right arm (crown toward the robot's left) and its chest beyond the chin. A bag-valve mask rests loosely on its face;
the self-inflating bag stands in its bracket in front of the left arm.

The manikin's lower jaw is hinged at the joint below the ear and hangs slack, as in an unconscious patient: a soft
spring holds it 1.5 deg below neutral, where the tongue closes the airway. The airway is open only while the jaw is
lifted to neutral or above. The mask's cushion is four sprung segments round the rim: two over the bridge of the nose
and the cheeks (on the head), two over the chin (on the jaw). Air goes in only while all four are pressed in at least
2.5 mm (an even seal); a tipped or one-sided mask leaves a segment proud, and the breath leaks out under it.

The training teaches "bring the face to the mask": one hand pinches the mask down and the jaw up at the same time (the
thumb and finger on the mask, the other fingers under the jawbone), so the seal costs no push on the patient. Pushing
the mask down onto the face does the opposite: through the chin segments it drives the jaw down and closes the airway.
A parallel gripper can make that pinch: one finger on the mask's connector, the other under the chin ledge. With the
jaw lifted and the seal held, squeezing the bag with the other hand raises the chest; otherwise the chest stays down.

The job: give the manikin two breaths that make its chest rise, then let go of everything.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/bvm-mask-seal-jaw-lift/bvm_mask_seal_jaw_lift.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinges, sprung sliders, per-step rules, per-episode memory)

from yam_arena import demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the mechanism's readout every 10 steps
NAIVE = False  # demo variant: press the mask straight down with closed fingers instead of the pinch

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the manikin head -------------------------------------------------------------------------------------------
HX = 0.335  # the face's midline (x)
Y_U = -0.055  # the upper cushion segments (bridge of the nose, cheeks), on the head
Y_L = -0.125  # the lower cushion segments (chin), on the jaw
Y_P = -0.090  # the mask's connector, midway between them
Y_H, Z_H = 0.040, TOP + 0.030  # the jaw's hinge axis (along x, below the ear)
Z_F = TOP + 0.085  # the face where the cushion meets it (jaw at neutral)
PAD = (0.012, 0.012, 0.012)  # one cushion segment (x, y, z)
PAD_X = 0.024  # segments this far either side of the midline
SHELL_T = 0.005  # mask shell: a plate 5 mm thick, 12 mm above the face at rest
SHELL = (0.090, Y_U - Y_L + 0.028)
PITCH = math.radians(15.0)  # the pinch comes in 15 deg below horizontal; the connector top and chin ledge face it
A_DIR = np.array([math.cos(PITCH), 0.0, -math.sin(PITCH)])  # the gripper's approach (forward, slightly down)
C_DIR = np.array([math.sin(PITCH), 0.0, math.cos(PITCH)])  # its closing axis (up, slightly forward)
# connector's top face centre: the pinch line runs through the cushion's centre on the face, so the pinch tips the
# mask neither way
P_TOP = np.array([HX + 0.008, Y_P, Z_F + PAD[2] + SHELL_T + 0.010])
STACK = (P_TOP[2] - (TOP + 0.044)) / C_DIR[2]  # connector top to chin ledge underside, along the closing axis (jaw 0)
P_BOT = P_TOP - STACK * C_DIR  # chin ledge's underside centre
JAW_START, JAW_LO, JAW_HI = -1.5, -1.5, 1.5  # deg; + lifts the chin
JAW_K, JAW_REST = 0.05, -8.0  # N*m/rad, deg: the slack jaw sags onto its -1.5 deg stop
PAD_K, PAD_REST, PAD_TRAVEL = 120.0, -0.003, 0.010  # N/m, m: 0.36 N preload, 10 mm travel
BAG_X, BAG_Y = 0.300, 0.200  # bag bracket
BAG_W, BAG_H, BAG_D = 0.016, 0.044, 0.040  # the bag's sprung front wall (y, z, x)
BAG_GAP = 0.026  # bulb space between the front wall and the fixed back
BAG_K, BAG_REST, BAG_TRAVEL = 45.0, -0.005, 0.022
BAG_OUT = 0.038  # outer faces at BAG_Y +- 0.038 (76 mm across)
CHEST_Y = -0.255
CHEST_Z = TOP + 0.050  # chest plate's underside at rest
CHEST_K, CHEST_RISE_MAX = 400.0, 0.025

SKIN, SKIN_D = (0.87, 0.72, 0.62), (0.80, 0.63, 0.53)
MASK_C, CUSH_C, VALVE_C = (0.70, 0.82, 0.92), (0.30, 0.48, 0.80), (0.25, 0.25, 0.28)
BAG_C, BRACKET_C, SHIRT_C, EYE_C = (0.35, 0.62, 0.40), (0.55, 0.57, 0.60), (0.25, 0.33, 0.55), (0.20, 0.15, 0.12)
PADS = ("pad_un", "pad_uf", "pad_ln", "pad_lf")  # upper/lower, near (robot side)/far
SEGS = {p: p.replace("pad", "seg") for p in PADS}
PAD_AT = {"pad_un": (-PAD_X, Y_U), "pad_uf": (PAD_X, Y_U), "pad_ln": (-PAD_X, Y_L), "pad_lf": (PAD_X, Y_L)}


def _rot_y(a):
    return mk.mat_to_quat(mk.axis_angle((0, 1, 0), a))


kit = mk.Kit()
# head (fixed, skin-tone PVC over foam): the face block under the upper segments, the wider cranium behind it
kit.part("head", (HX, 0.0, TOP), [
    mk.box((0.090, 0.040 - (Y_U - 0.008), Z_F - TOP), at=(0, (Y_U - 0.008 + 0.040) / 2, (Z_F - TOP) / 2)),
    mk.box((0.130, 0.070, 0.100), at=(0, 0.075, 0.050)),
    mk.box((0.010, 0.006, 0.001), at=(-0.016, -0.020, Z_F - TOP + 0.0005), color=EYE_C),
    mk.box((0.010, 0.006, 0.001), at=(0.016, -0.020, Z_F - TOP + 0.0005), color=EYE_C),
], color=SKIN)
# jaw (free, 15 g): the chin plate under the lower segments, a ledge under the chin (the jawbone's lower border,
# its underside square to the pinch), and the ramus back to the hinge (inside the head; a hinged pair never collides)
_jz0 = Z_F - 0.030
_rib_c = P_BOT + 0.016 * A_DIR + 0.005 * C_DIR  # 48 mm along the approach from 8 mm behind the contact, 10 mm thick
kit.part("jaw", (HX, Y_H, Z_H), [
    mk.box((0.070, (Y_U - 0.008) - (Y_L - 0.018), 0.030),
           at=(0, (Y_U - 0.008 + Y_L - 0.018) / 2 - Y_H, Z_F - 0.015 - Z_H)),
    mk.box((0.048, 0.008, 0.010), at=tuple(_rib_c - np.array([HX, Y_H, Z_H])), quat=tuple(_rot_y(PITCH)),
           color=SKIN_D),
    mk.box((0.012, Y_H + 0.062, 0.012), at=(0, (Y_H - 0.062) / 2 - Y_H, 0.006)),
    mk.box((0.012, 0.012, _jz0 - Z_H), at=(0, -0.058 - Y_H, (_jz0 - Z_H) / 2)),
], mass=0.015, color=SKIN_D)
kit.hinge("jaw_hinge", "head", "jaw", anchor=(HX, Y_H, Z_H), axis=(-1, 0, 0), lower_deg=JAW_LO, upper_deg=JAW_HI,
          start_deg=JAW_START, stiffness=JAW_K, rest_deg=JAW_REST)


# mask (free): shell + connector, reference pose = resting on the face with the jaw at its start (slack) angle;
# its 4 cushion segments (free, 5 g) slide up into it on springs
def _mask_ref():
    """(origin, rotation) of the mask resting with its segments uncompressed on the head and the slack jaw."""
    r = mk.axis_angle((-1, 0, 0), math.radians(JAW_START))
    hinge = np.array([HX, Y_H, Z_H])
    low = r @ (np.array([HX, Y_L, Z_F]) - hinge) + hinge  # the chin contact point, slack jaw
    up = np.array([HX, Y_U, Z_F])
    tilt = math.atan2(up[2] - low[2], up[1] - low[1])  # the mask's pitch about x (chin end down)
    rot = mk.axis_angle((1, 0, 0), tilt)
    return up, rot


_M0, _MR = _mask_ref()  # mask frame origin: the upper segments' contact line centre on the face
_MQ = tuple(mk.mat_to_quat(_MR))


def _m(local):
    return _M0 + _MR @ np.asarray(local, float)


_dy_l = Y_L - Y_U
_boss_c = np.array([P_TOP[0] - HX, Y_P - Y_U, P_TOP[2] - Z_F]) - 0.006 * C_DIR  # 12 mm thick along the pinch
kit.part("mask", tuple(_M0), [
    mk.box((SHELL[0], SHELL[1], SHELL_T), at=(0, _dy_l / 2, PAD[2] + SHELL_T / 2)),
    mk.box((0.020, 0.008, 0.012), at=tuple(_boss_c), quat=tuple(_rot_y(PITCH)), color=VALVE_C),
], mass=0.028, color=MASK_C, quat=_MQ)
for _p in PADS:
    _px, _py = PAD_AT[_p]
    kit.part(_p, tuple(_m((_px, _py - Y_U, 0.0))), [mk.box(PAD, at=(0, 0, PAD[2] / 2))], mass=0.005, color=CUSH_C,
             quat=_MQ)
    kit.slider(SEGS[_p], "mask", _p, axis=tuple(_MR @ np.array([0, 0, 1.0])), lower=0.0, upper=PAD_TRAVEL,
               stiffness=PAD_K, rest=PAD_REST)
# neck and chest (fixed, a cotton training shirt over the torso), chest plate (free, 30 g) on a vertical slider
kit.part("torso", (HX, CHEST_Y, TOP), [
    mk.box((0.060, 0.025, 0.032), at=(0, Y_L - 0.025 - 0.0125 - CHEST_Y, 0.016), color=SKIN),
    mk.box((0.150, 0.165, 0.050), at=(0, 0.0, 0.025)),
], color=SHIRT_C)
kit.part("chest", (HX, CHEST_Y, CHEST_Z), [mk.box((0.080, 0.080, 0.006), at=(0, 0, 0.003))], mass=0.030,
         color=SHIRT_C)
kit.slider("chest_rise", "torso", "chest", axis=(0, 0, 1), lower=0.0, upper=CHEST_RISE_MAX, stiffness=CHEST_K,
           rest=-0.002, damping=5.0)
# bag (self-inflating bag, green PVC): fixed back half in a steel bracket, sprung front wall squeezed toward it
kit.part("bag", (BAG_X, BAG_Y, TOP), [
    mk.box((BAG_D, BAG_OUT - (BAG_W + BAG_GAP - BAG_OUT), BAG_H),
           at=(0, (BAG_OUT + (BAG_W + BAG_GAP - BAG_OUT)) / 2, BAG_H / 2)),
    mk.box((0.012, 0.010, 0.012), at=(0, BAG_OUT - 0.005, BAG_H + 0.006), color=VALVE_C),
    mk.box((0.050, 0.004, 0.012), at=(0, BAG_OUT + 0.002, 0.006), color=BRACKET_C),
], color=BAG_C)
kit.part("bag_wall", (BAG_X, BAG_Y - BAG_OUT + BAG_W / 2, TOP), [mk.box((BAG_D, BAG_W, BAG_H), at=(0, 0, BAG_H / 2))],
         mass=0.015, color=BAG_C)
kit.slider("bag_squeeze", "bag", "bag_wall", axis=(0, 1, 0), lower=0.0, upper=BAG_TRAVEL, stiffness=BAG_K,
           rest=BAG_REST)
OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- readouts, the breath rule and the scoring -------------------------------------------------------------------
SEAL_MIN = 0.0025  # every cushion segment pressed in at least this much
OPEN_DEG = 0.0  # airway open: jaw at or above neutral
B_ON, B_OFF = 0.006, 0.003  # a squeeze starts / ends (bag wall travel)
RISE_PER_M = 0.75  # chest rise per metre of bag stroke given through an open, sealed airway
RISE_CAP = 0.015
RISE_MIN = 0.010  # a breath counts if the chest plate rose this much
M = "bvm"
_TEST_TCPS = None
_REF_LOCAL = {}  # each segment's / the bag wall's rest offset in its parent's frame


def _local(objects, parent, child) -> np.ndarray:
    """``child``'s origin in ``parent``'s frame (immune to the world-origin lever in Kit.q)."""
    tp, tc = kit._pose_T(objects, parent), kit._pose_T(objects, child)
    return tp[:3, :3].T @ (tc[:3, 3] - tp[:3, 3])


def _ref(parent, child) -> np.ndarray:
    key = (parent, child)
    if key not in _REF_LOCAL:
        _REF_LOCAL[key] = _local(kit.fake_states(), parent, child)
    return _REF_LOCAL[key]


def seg_in(objects, p) -> float:
    """How far cushion segment ``p`` is pressed up into the mask (m), in the mask's own frame."""
    return float(_local(objects, "mask", p)[2] - _ref("mask", p)[2])


def bag_in(objects) -> float:
    """How far the bag's front wall is squeezed in (m)."""
    return float(_local(objects, "bag", "bag_wall")[1] - _ref("bag", "bag_wall")[1])


def jaw_deg(objects) -> float:
    """Jaw angle (deg, + lifted), from the jaw's rotation relative to the head."""
    return kit.q_deg(objects, "jaw_hinge")


def chest_rise(objects) -> float:
    """How far the chest plate stands above its rest (m)."""
    return float(kit.pose(objects, "chest")[0][2] - CHEST_Z)


def sealed(objects) -> bool:
    return all(seg_in(objects, p) >= SEAL_MIN for p in PADS)


def airway_open(objects) -> bool:
    return jaw_deg(objects) >= OPEN_DEG


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"squeeze": False, "b_prev": 0.0, "given": 0.0, "spoiled": False, "peak": 0.0,
                            "target": 0.0, "breaths": 0, "tries": 0, "step": 0}
    return m


def _set_chest(target) -> None:
    """Sim only: move the chest plate's spring set point (the lungs filling)."""
    path = mk._SIM["joint_paths"].get((kit.id, "chest_rise"))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear").GetTargetPositionAttr().Set(float(target))


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit bvm] step {mk._EP.get('step')}: {msg}", flush=True)


def _rules(objects) -> None:
    """One control step of the lungs: a squeeze of the bag fills them (the chest plate rises) only for the stroke given
    while all four segments seal and the jaw is lifted; if the jaw is driven shut during a squeeze, that breath is
    lost; when the bag is let go, the chest falls and the breath counts if the chest rose at least 10 mm."""
    m = _mem()
    m["step"] += 1
    b, rise = bag_in(objects), chest_rise(objects)
    ok_seal, ok_air = sealed(objects), airway_open(objects)
    if not m["squeeze"] and b >= B_ON:
        m.update(squeeze=True, given=0.0, spoiled=False, peak=0.0)
        m["tries"] += 1
        _say(f"squeeze {m['tries']} starts; seal {ok_seal} airway {ok_air}")
    if m["squeeze"]:
        if not ok_air:
            if not m["spoiled"]:
                _say("airway shut during the squeeze: breath lost")
            m["spoiled"] = True
        if ok_seal and ok_air and not m["spoiled"]:
            m["given"] += max(0.0, b - m["b_prev"])
        m["peak"] = max(m["peak"], rise)
        m["target"] = 0.0 if m["spoiled"] else min(RISE_CAP, RISE_PER_M * m["given"])
        if b < B_OFF:
            good = not m["spoiled"] and m["peak"] >= RISE_MIN
            m["breaths"] += int(good)
            _say(f"squeeze {m['tries']} ends: chest peak {m['peak'] * 1000:.1f} mm, given {m['given'] * 1000:.1f} mm "
                 f"-> {'BREATH' if good else 'no breath'} ({m['breaths']})")
            m.update(squeeze=False, target=0.0)
    m["b_prev"] = b
    _set_chest(m["target"])
    if DEBUG and mk._EP.get("env") is not None and m["step"] % 10 == 0:
        tc = mk.tcps()
        tz = " ".join(f"({t[0]:.3f},{t[1]:.3f},{t[2] - TOP:.3f})" for t in tc) if tc else "-"
        mp = kit.pose(objects, "mask")[0]
        _say(f"segs {' '.join(f'{seg_in(objects, p) * 1000:.1f}' for p in PADS)} mm; jaw {jaw_deg(objects):.2f} deg; "
             f"bag {b * 1000:.1f} mm; chest {rise * 1000:.1f} mm (tgt {m['target'] * 1000:.1f}); mask "
             f"({mp[0]:.4f},{mp[1]:.4f},{mp[2] - TOP:.4f}); tcps {tz}")


kit.on_step(_rules)

PARTS = ("mask", *PADS, "jaw", "bag_wall", "chest")


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def score(objects) -> float:
    """0.4 per breath that raised the chest (up to 2); with 2, 1.0 once both grippers are 2.5 cm clear of the mask,
    jaw, bag and chest and everything is still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _rules(objects)
    n = min(_mem()["breaths"], 2)
    if n < 2:
        return round(0.4 * n, 6)
    clear = all(_released(objects[p]) and objects[p].resting() for p in PARTS)
    return 1.0 if clear else 0.8


def success(objects) -> bool:
    """Two breaths that raised the chest, everything let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Breaths still owed, plus how far the seal and the jaw are from good (m and rad)."""
    owed = max(0, 2 - _mem()["breaths"])
    short = sum(max(0.0, SEAL_MIN - seg_in(objects, p)) for p in PADS)
    return float(owed + short + max(0.0, math.radians(OPEN_DEG - jaw_deg(objects))))


# -- tilted grips for the scripted demo ----------------------------------------------------------------------------
# yam_arena's Waypoint carries only a yaw about the straight-down grip. A Waypoint's yaw may be a Tilt: a float (0.0)
# that also carries the full TCP rotation, which ArmPlan uses instead (an in-memory patch of grip_rotation; plain yaws
# unchanged).


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
# Open loop from the scene's geometry (it reads no joint, cushion or chest state). Right hand: pointing forward and
# 15 deg down, fingers closing up/down, it slides in from the manikin's side with the fingers open, one finger above the
# mask's connector, the other below the chin ledge, and closes: the mask comes down and the jaw comes up into it. It
# holds the pinch while the left hand squeezes the bag twice (each squeeze about 1 s, the chest rising), then opens
# and backs out. NAIVE: the right hand instead pushes the mask straight down with closed fingers.

C_TARGET = 0.0065  # aim: every cushion segment 6.5 mm in, the jaw on its +1.5 deg stop
Z_SAG = 0.007  # aim the hand 7 mm high: it sits ~2.5 mm under its command, and the rest becomes a ~0.4 N jaw lift
FINGER_GIVE = 0.0015  # command the fingers 1.5 mm past touching each (they stop ~1 mm short under the pinch)
PAD_MID = 0.005  # the pads' centre is 5 mm past the TCP along the approach
BACK = 0.090  # stand-off before sliding in
GRIP_Z = TOP + 0.030  # left TCP height at the bag (pads over its middle)
BAG_SQ = 0.024  # left squeeze: gap closed 24 mm past the bag's width
R_PINCH = np.column_stack([C_DIR, np.cross(A_DIR, C_DIR), A_DIR])
PARK_R = (0.200, -0.180, TOP + 0.180)


def _pinched_faces():
    """Connector top and chin-ledge underside centres as they sit when pinched: the jaw on its +1.5 deg stop lifts the
    ledge and the chin segments; the mask, pressed in C_TARGET all round, sits midway between nose and chin."""
    up = np.array([0, 0, 1.0])
    chin = (Y_H - Y_L) * math.sin(math.radians(JAW_HI))
    top = P_TOP + (chin / 2 - C_TARGET) * up
    bot = P_BOT + (Y_H - Y_P) * math.sin(math.radians(JAW_HI)) * up
    return top, bot


def pinch_tcp() -> np.ndarray:
    """TCP for the pinch: midway between the connector's top and the chin ledge as they are when pinched, the pads
    centred on the connector."""
    top, bot = _pinched_faces()
    return (top + bot) / 2 - PAD_MID * A_DIR + np.array([0, 0, Z_SAG])


def demo():
    """Right hand pinches the mask down and the jaw up; left hand squeezes the bag twice; both let go and back off."""
    t_r = Tilt(R_PINCH)
    tcp = pinch_tcp()
    top, bot = _pinched_faces()
    pinch = -(float(np.dot(top - bot, C_DIR)) - 2 * FINGER_GIVE) / 2
    o = FINGER_OPEN
    out = tcp - BACK * A_DIR
    R = []

    def w(dur, pos, f, label, yaw=0.0, arm=R):
        arm.append(Waypoint(dur, tuple(float(v) for v in pos), f, label, yaw))

    if NAIVE:
        top = P_TOP + 0.013 * np.array([0, 0, 1.0])
        w(3.0, top + np.array([0, 0, 0.03]), 0.0, "over the mask, fingers closed")
        w(1.5, top + np.array([0, 0, 0.005]), 0.0, "onto the connector")
        w(2.0, top - np.array([0, 0, 0.025]), 0.0, "press the mask down onto the face")
        w(8.0, top - np.array([0, 0, 0.025]), 0.0, "hold it down")
        w(1.5, top + np.array([0, 0, 0.03]), 0.0, "up")
        w(2.0, PARK_R, o, "park")
    else:
        w(3.0, out, o, "beside the head, pointing in, fingers open", t_r)
        w(2.5, tcp, o, "slide in: a finger over the connector, one under the chin ledge", t_r)
        w(0.5, tcp, o, "settle", t_r)
        w(1.5, tcp, pinch, "pinch: mask down, jaw up", t_r)
        w(1.0, tcp, pinch, "hold the seal", t_r)
        w(7.0, tcp, pinch, "hold the seal while the bag is squeezed", t_r)
        w(1.0, tcp, o, "let go", t_r)
        w(2.0, out, o, "back out", t_r)
        w(2.0, PARK_R, o, "park")
    total = sum(x.duration for x in R)

    L = []
    sq = -(2 * BAG_OUT - BAG_SQ) / 2
    rel = -(2 * BAG_OUT + 0.010) / 2
    w(2.0, (BAG_X, BAG_Y, GRIP_Z + 0.070), o, "over the bag", arm=L)
    w(1.5, (BAG_X, BAG_Y, GRIP_Z), o, "fingers either side of it", arm=L)
    w(5.0, (BAG_X, BAG_Y, GRIP_Z), rel, "wait for the seal", arm=L)
    for k in (1, 2):
        w(1.0, (BAG_X, BAG_Y, GRIP_Z), sq, f"squeeze {k} (about 1 s)", arm=L)
        w(0.5, (BAG_X, BAG_Y, GRIP_Z), sq, "hold", arm=L)
        w(1.0, (BAG_X, BAG_Y, GRIP_Z), rel, "let it refill", arm=L)
        w(0.8, (BAG_X, BAG_Y, GRIP_Z), rel, "chest falls", arm=L)
    w(0.5, (BAG_X, BAG_Y, GRIP_Z), o, "open", arm=L)
    w(1.5, (BAG_X, BAG_Y, GRIP_Z + 0.080), o, "up", arm=L)
    lt = sum(x.duration for x in L)
    w(max(1.0, total - lt), (0.150, 0.250, TOP + 0.200), o, "park", arm=L)
    lt = sum(x.duration for x in L)
    if lt > total:
        R.append(Waypoint(lt - total, PARK_R, o, "park"))
    return {"left": L, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences (one trial each, fed step by step; the chest plate follows the
    lungs' set point one step late, as in the sim)."""
    global _TEST_TCPS
    # a frame spec: (segment compressions dict or a single value, jaw deg, bag squeeze m)
    def state(segs, jaw, bag, chest):
        q = {SEGS[p]: (segs[p] if isinstance(segs, dict) else segs) for p in PADS}
        q.update(jaw_hinge=math.radians(jaw), bag_squeeze=bag, chest_rise=chest)
        return kit.fake_states(q=q)

    def run(specs, tcps_end=()):
        global _TEST_TCPS
        mk.new_episode()
        _TEST_TCPS = []
        got = 0.0
        for i, (segs, jaw, bag) in enumerate(specs):
            if i == len(specs) - 1:
                _TEST_TCPS = list(tcps_end)
            got = score(state(segs, jaw, bag, _mem()["target"]))
        return got

    def squeeze(segs, jaw, depth=0.020, n=10, segs_late=None, jaw_late=None):
        out = []
        for k in range(n + 1):
            late = k > n // 2
            out.append((segs_late if late and segs_late is not None else segs,
                        jaw_late if late and jaw_late is not None else jaw, depth * k / n))
        out += [(segs, jaw, depth)] * 4
        out += [(segs, jaw, depth * (1 - k / 6)) for k in range(1, 7)] + [(segs, jaw, 0.0)] * 5
        return out

    rest = [(0.0, JAW_START, 0.0)] * 3
    pinch = 0.006  # every segment 6 mm in, jaw on its +1.5 deg stop
    good = squeeze(pinch, JAW_HI)
    let_go = [(0.001, JAW_START, 0.0)] * 3
    naive = squeeze(0.005, JAW_LO)  # mask pushed down: even seal but the jaw is driven onto its low stop
    chin_end = squeeze({"pad_un": 0.0, "pad_uf": 0.0, "pad_ln": 0.007, "pad_lf": 0.007}, JAW_HI)
    near_side = squeeze({"pad_un": 0.006, "pad_uf": 0.001, "pad_ln": 0.006, "pad_lf": 0.001}, JAW_HI)
    tcp_mask = tuple(kit.pose(kit.fake_states(), "mask")[0])
    cases = {  # name: (frames, TCPs at the end, score)
        "start": (rest, [], 0.0),
        "pinch, two squeezes, let go (demo)": (rest + good + good + let_go, [], 1.0),
        "the same but still pinching the mask at the end": (rest + good + good, [tcp_mask], 0.8),
        "naive: mask pushed down onto the face, two squeezes (airway shut)": (rest + naive + naive + let_go, [], 0.0),
        "pinch at the chin end of the mask (upper segments leak)": (rest + chin_end + chin_end + let_go, [], 0.0),
        "one-sided press on the near rim (far segments leak)": (rest + near_side + near_side + let_go, [], 0.0),
        "jaw lifted, mask on its own weight (no seal)": (rest + squeeze(0.0, JAW_HI) * 2 + let_go, [], 0.0),
        "one good breath": (rest + good + let_go, [], 0.4),
        "two shallow squeezes (10 mm: chest rises 7.5 mm)":
            (rest + squeeze(pinch, JAW_HI, depth=0.010) * 2 + let_go, [], 0.0),
        "jaw driven shut halfway through a squeeze, then one good breath":
            (rest + squeeze(pinch, JAW_HI, jaw_late=-1.0) + good + let_go, [], 0.4),
        "seal made only halfway through each squeeze (half the stroke given)":
            (rest + squeeze(0.0, JAW_HI, segs_late=pinch) * 2 + let_go, [], 0.0),
        "a wasted squeeze before the pinch, then two good": (rest + naive + good + good + let_go, [], 1.0),
        "three good breaths": (rest + good * 3 + let_go, [], 1.0),
        "one long squeeze never let go": (rest + squeeze(pinch, JAW_HI)[:15] + [(pinch, JAW_HI, 0.020)] * 30, [], 0.0),
        "two good breaths, left hand still on the bag":
            (rest + good + good + let_go, [(BAG_X, BAG_Y, GRIP_Z)], 0.8),
        "jaw at neutral (0 deg) counts as open": (rest + squeeze(pinch, 0.0) * 2 + let_go, [], 1.0),
        "segments 2 mm in (under 2.5 mm) leak": (rest + squeeze(0.002, JAW_HI) * 2 + let_go, [], 0.0),
    }
    try:
        for name, (frames, tcps, want) in cases.items():
            got = run(frames, tcps)
            assert abs(got - want) < 1e-9, (name, got, want, _mem())
        run(good + good)
        mk.new_episode()
        assert _mem()["breaths"] == 0, "a new trial forgets the last one"
        # geometry: the reference readouts are zero, and segments/jaw/bag/chest readouts invert the fake poses
        st = state({"pad_un": 0.001, "pad_uf": 0.002, "pad_ln": 0.003, "pad_lf": 0.004}, 2.0, 0.015, 0.012)
        assert all(abs(seg_in(st, p) - v) < 1e-9 for p, v in zip(PADS, (0.001, 0.002, 0.003, 0.004)))
        assert abs(jaw_deg(st) - 2.0) < 1e-6 and abs(bag_in(st) - 0.015) < 1e-9 and abs(chest_rise(st) - 0.012) < 1e-9
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 2


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n = self_test()
    print(f"scoring: {n} cases pass (pinch stack {STACK * 1000:.1f} mm at jaw 0; gripper opens 94 mm)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
