"""On the station table, in front of the arms, the lower front of an open jacket lies flat with its separating
(open-ended) zip undone. The zip runs straight away from the robot along the jacket's centre line. Its left half
(sewn to the left front panel, the panel's hem band nearest the robot) ends at the bottom in the retainer box; the
slider is parked at the bottom of the left track, resting on the box. The right half is a loose tape whose bottom end
is the insertion pin, a stiff bar with a small raised grip tab on its tape. The pin has been put through the slider's
right channel, but it sits 5 mm short of home in the box. Parts are scaled 2.5x so the fingers can work them.

The right tape is flexible: a chain of short hinged sections, each with its own row of teeth, that springs back into
a loose curve away from the left half (an open jacket). The slider's right flange gathers the tape against the left
teeth as it climbs, and behind the slider each section's teeth interlock with the left teeth only if they line up
with them, which happens only when the pin is fully home in the box (the pin sets where every right tooth sits). Pull
the slider with the pin short and every right tooth meets a left tooth head-on: nothing interlocks, the slider climbs
the left half alone and the right tape springs back out behind it (a split zip).

The left front lies loose on the table: the pin needs a firm push to slide home (it is a snug fit in the box), and
that push shoves the whole jacket front across the table unless the hem band is held. Sliding the slider back down to the box undoes what it zipped, so a split can be started again.

The job: zip the jacket front closed: every tooth interlocked up to the slider's top stop, then let go of everything.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/separating-zipper-start/separating_zipper_start.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinged tape sections, sprung/friction sliders, per-step rules, episode memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print latch events and a readout every 10 steps

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# The zip runs along +x on the line y = 0 (the teeth meet there); the left half is at y > 0, the right tape at y < 0.
# -- dimensions (2.5x a #5 moulded separating zip) -----------------------------------------------------------------
X_HEM = 0.130  # hem band front edge
X_BOX = 0.146  # retainer box bottom end (left_front frame origin)
BOX_L = 0.018
X_SL0 = X_BOX + BOX_L  # slider's bottom edge when parked on the box (0.164)
SL_L = 0.038  # slider length
X_PIN = 0.150  # seated pin's bottom end (pin frame origin, seated)
X_SEG0 = 0.226  # top of the pin = start of the first tape section
SEG_L = 0.020  # tape section length (about 3 teeth)
N_SEG = 6
X_TOP = X_SEG0 + N_SEG * SEG_L  # 0.346: top of the right tape
Q_MAX = X_TOP + 0.004 - X_SL0  # slider travel to its top stop (0.186: bottom edge 4 mm past the last section)
ROW_W, ROW_H = 0.007, 0.0045  # tooth row width (y) and height
GAP = 0.0003  # each tooth row stops 0.3 mm short of the centre line
TAPE_W, TAPE_H = 0.030, 0.002  # tape beside each tooth row
PIN_SHORT = 0.005  # start: the pin sits this far short of home
PIN_TRAVEL = 0.012
PIN_FRICTION = 1.2  # N: the pin is a snug fit in the box
ZIP_FRICTION = 0.3  # N: the slider's drag on the teeth
BEND_REST = -5.0  # deg: each tape joint's free bend (the tape springs into a loose curve away from the left half)
BEND_K = 0.15  # N*m/rad
TAB = dict(x=(0.212, 0.226), y=(-0.050, -0.037), z=(0.0, 0.014))  # the pin's raised grip tab (seated pose)
PULL = dict(u=(0.006, 0.032), y=(-0.008, 0.004), z=(0.0105, 0.0205))  # the slider's pull (slider frame)
HEM_Y = (0.0, 0.225)
LEFT_KG, PIN_KG, SEG_KG, SL_KG = 0.035, 0.006, 0.004, 0.012

NAVY, TEETH, METAL, DARK, TABC = (0.16, 0.20, 0.34), (0.12, 0.12, 0.13), (0.62, 0.63, 0.66), (0.08, 0.08, 0.09), \
    (0.85, 0.35, 0.10)
SEGS = [f"seg{k}" for k in range(1, N_SEG + 1)]
PARTS = ("left_front", "pin", "slider", *SEGS)

_STOP = BOX_L + Q_MAX + SL_L + 0.002  # top stop centre (left_front frame)
kit = mk.Kit()
# left front: panel, hem band, retainer box and the left tooth row (frame origin at the box's bottom end, on the table)
kit.part("left_front", (X_BOX, 0.0, TOP), [
    mk.box((X_BOX - X_HEM, HEM_Y[1] - HEM_Y[0], 0.010), at=(-(X_BOX - X_HEM) / 2, sum(HEM_Y) / 2, 0.005)),
    mk.box((_STOP + 0.006, HEM_Y[1] - GAP - ROW_W, 0.002), at=((_STOP + 0.006) / 2, (HEM_Y[1] + GAP + ROW_W) / 2, 0.001)),
    mk.box((BOX_L, 0.0185, 0.008), at=(BOX_L / 2, -0.0025, 0.004), color=DARK),
    mk.box((Q_MAX + SL_L, ROW_W, ROW_H), at=(BOX_L + (Q_MAX + SL_L) / 2, GAP + ROW_W / 2, ROW_H / 2), color=TEETH),
    mk.box((0.004, ROW_W, 0.006), at=(_STOP, GAP + ROW_W / 2, 0.003), color=DARK),  # top stop
], mass=LEFT_KG, color=NAVY)
# insertion pin (reference pose = seated): bar in line with the right teeth, stiffened tape end, grip tab
kit.part("pin", (X_PIN, 0.0, TOP), [
    mk.box((X_SEG0 - X_PIN, ROW_W, ROW_H), at=((X_SEG0 - X_PIN) / 2, -GAP - ROW_W / 2, ROW_H / 2), color=DARK),
    mk.box((0.020, 0.0427, TAPE_H), at=(X_SEG0 - X_PIN - 0.010, -GAP - ROW_W - 0.0427 / 2, TAPE_H / 2)),  # to y -0.050
    mk.box((TAB["x"][1] - TAB["x"][0], TAB["y"][1] - TAB["y"][0], TAB["z"][1] - TAPE_H),
           at=(sum(TAB["x"]) / 2 - X_PIN, sum(TAB["y"]) / 2, (TAB["z"][1] + TAPE_H) / 2), color=TABC),
], mass=PIN_KG, color=NAVY)
kit.slider("pin_seat", "left_front", "pin", axis=(-1, 0, 0), lower=-PIN_TRAVEL, upper=0.0, start=-PIN_SHORT,
           friction=PIN_FRICTION)
# right tape: sections hinged at the centre-line end of each joint (reference pose = zipped, in line)
_prev = "pin"
for _k, _s in enumerate(SEGS):
    _x = X_SEG0 + _k * SEG_L
    kit.part(_s, (_x, 0.0, TOP), [
        mk.box((SEG_L, ROW_W, ROW_H), at=(SEG_L / 2, -GAP - ROW_W / 2, ROW_H / 2), color=TEETH),
        mk.box((SEG_L, TAPE_W, TAPE_H), at=(SEG_L / 2, -GAP - ROW_W - TAPE_W / 2, TAPE_H / 2)),
    ], mass=SEG_KG, color=NAVY)
    kit.hinge(f"bend{_k + 1}", _prev, _s, anchor=(_x, -GAP, TOP + ROW_H / 2), axis=(0, 0, 1), lower_deg=-25.0,
              upper_deg=2.0, start_deg=BEND_REST, stiffness=BEND_K, rest_deg=BEND_REST)
    _prev = _s
# slider (reference pose = parked on the box): top plate, left flange, right flange flared into a funnel, pull
_FL_IN0, _FL_IN1 = -GAP - ROW_W - 0.0005, -0.016  # right flange's inner face at the bottom / top end (y)
_fl_yaw = math.atan2(_FL_IN1 - _FL_IN0, SL_L)
_fl_len = math.hypot(SL_L, _FL_IN1 - _FL_IN0)
_fl_c = (SL_L / 2 + 0.0015 * math.sin(_fl_yaw), (_FL_IN0 + _FL_IN1) / 2 - 0.0015 * math.cos(_fl_yaw))
kit.part("slider", (X_SL0, 0.0, TOP), [
    mk.box((SL_L, 0.0305, 0.004), at=(SL_L / 2, -0.00375, 0.0085)),
    mk.box((SL_L, 0.004, 0.004), at=(SL_L / 2, 0.0095, 0.0045)),
    mk.box((_fl_len, 0.003, 0.004), at=(_fl_c[0], _fl_c[1], 0.0045), yaw=_fl_yaw),
    mk.box((PULL["u"][1] - PULL["u"][0], PULL["y"][1] - PULL["y"][0], PULL["z"][1] - PULL["z"][0]),
           at=(sum(PULL["u"]) / 2, sum(PULL["y"]) / 2, sum(PULL["z"]) / 2)),
], mass=SL_KG, color=METAL)
kit.slider("zip", "left_front", "slider", axis=(1, 0, 0), lower=0.0, upper=Q_MAX, start=0.0, friction=ZIP_FRICTION)

OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the interlock rule and the scoring ---------------------------------------------------------------------------
SEATED_TOL = 0.0008  # pin home: within 0.8 mm of the bottom of the box
REG_X, REG_Y, REG_Z, REG_DEG = 0.0012, 0.0025, 0.002, 6.0  # teeth line up with the left teeth (section vs zipped)
WINDOW = (0.001, 0.012)  # a section interlocks as the slider's bottom edge clears its top by 1..12 mm
M = "zip"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
_REF_REL = {p: np.linalg.inv(kit.parts["left_front"].ref) @ kit.parts[p].ref for p in PARTS}


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"latched": {}, "events": []}
    return m


def _rel(objects, part) -> np.ndarray:
    """``part``'s pose in the left front's frame, relative to where it sits when zipped (4x4, world poses only)."""
    rel = np.linalg.inv(kit._pose_T(objects, "left_front")) @ kit._pose_T(objects, part)
    return np.linalg.inv(_REF_REL[part]) @ rel


def pin_short(objects) -> float:
    """How far (m) the pin's bottom end stands short of home in the box."""
    return float(_rel(objects, "pin")[0, 3])


def slider_travel(objects) -> float:
    """How far (m) the slider has climbed from the box."""
    return float(_rel(objects, "slider")[0, 3])


def section_top(k: int) -> float:
    """Distance (m) from the slider's parked bottom edge to the top of tape section k (1-based), zipped."""
    return X_SEG0 + k * SEG_L - X_SL0


def lined_up(objects, s) -> bool:
    """Tape section ``s`` sits where its teeth interlock with the left teeth (within 1.2 mm along the zip)."""
    d = _rel(objects, s)
    yaw = math.degrees(math.atan2(d[1, 0], d[0, 0]))
    return bool(abs(d[0, 3]) <= REG_X and abs(d[1, 3]) <= REG_Y and abs(d[2, 3]) <= REG_Z and abs(yaw) <= REG_DEG)


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit zip] step {mk._EP.get('step')}: {msg}", flush=True)


_LATCH_PRIMS: list = []  # sim: the interlock joints authored this episode


def _latch(k: int):
    """Sim: weld section k to the left front at its zipped pose (its teeth interlocked). Returns the joint path."""
    if mk._EP.get("env") is None or mk._SIM.get("stage") is None:
        return True
    s = SEGS[k - 1]
    j = mk._Joint(f"teeth{k}", "bond", "left_front", s, kit.parts[s].ref[:3, 3] + np.array([SEG_L / 2, 0, 0]),
                  np.array([1.0, 0, 0]))
    mk._SIM["gen"] += 1
    path = mk._author_joint(mk._SIM["stage"], kit, j, mk._SIM["paths"], mk._SIM["env_T"], suffix=f"_z{mk._SIM['gen']}")
    if path:
        _LATCH_PRIMS.append(path)
    return path or True


def _unlatch(path) -> None:
    if isinstance(path, str) and mk._SIM.get("stage") is not None:
        mk._SIM["stage"].RemovePrim(path)
        if path in _LATCH_PRIMS:
            _LATCH_PRIMS.remove(path)


_friction_reset0 = mk._friction_reset


def _friction_reset() -> None:  # sim reset: drop last episode's interlocks before the parts are put back
    for p in list(_LATCH_PRIMS):
        _unlatch(p)
    _friction_reset0()


mk._friction_reset = _friction_reset


def _rules(objects) -> None:
    """One control step of the zip: a section interlocks as the slider's bottom edge clears it, if its teeth line up
    with the left teeth; the slider pushed back below a section opens it again."""
    m = _mem()
    u = slider_travel(objects)
    for k in range(1, N_SEG + 1):
        top = section_top(k)
        if k in m["latched"]:
            if u < top - WINDOW[0]:
                _unlatch(m["latched"].pop(k))
                m["events"].append(("open", k))
                _say(f"section {k} opened (slider back down)")
        elif top + WINDOW[0] <= u <= top + WINDOW[1]:
            if lined_up(objects, SEGS[k - 1]):
                m["latched"][k] = _latch(k)
                m["events"].append(("mesh", k))
                _say(f"section {k} INTERLOCKED (pin short {pin_short(objects) * 1000:.2f} mm)")
    if DEBUG and mk._EP.get("env") is not None and mk._EP.get("step", 0) % 10 == 0:
        lp = kit.pose(objects, "left_front")[0]
        tc = mk.tcps()
        d = [_rel(objects, s) for s in SEGS]
        _say(f"pin short {pin_short(objects) * 1000:.2f} mm, slider {u * 1000:.1f} mm, latched {sorted(m['latched'])}, "
             f"left_front ({lp[0]:.4f},{lp[1]:.4f}) seg dx/dy mm "
             f"{[(round(x[0, 3] * 1000, 1), round(x[1, 3] * 1000, 1)) for x in d]} "
             f"tcps {[np.round(t, 3).tolist() for t in tc]}")


kit.on_step(_rules)


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def seated(objects) -> bool:
    """The pin is home: its bottom end within 0.8 mm of the bottom of the box."""
    return pin_short(objects) <= SEATED_TOL


def interlocked(objects) -> int:
    """How many tape sections have their teeth interlocked."""
    return len(_mem()["latched"])


def split(objects) -> bool:
    """The slider has climbed past a tape section whose teeth are not interlocked."""
    u, latched = slider_travel(objects), _mem()["latched"]
    return any(u >= section_top(k) + WINDOW[0] and k not in latched for k in range(1, N_SEG + 1))


def score(objects) -> float:
    """0 while the zip is split (the slider above a section that did not interlock). Otherwise 0.25 for the pin home;
    0.5 with at least 3 of the 6 tape sections interlocked; 1.0 with all 6 (zipped to the top stop). Halved unless both
    grippers are 2.5 cm clear of every part and everything is still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _rules(objects)
    if split(objects):
        return 0.0
    n = interlocked(objects)
    base = 0.0
    if n == N_SEG:
        base = 1.0
    elif seated(objects):
        base = 0.5 if n >= (N_SEG + 1) // 2 else 0.25
    clear = all(_released(objects[p]) and objects[p].resting() for p in PARTS)
    return round(base if clear else base / 2, 6)


def success(objects) -> bool:
    """Zipped to the top stop, let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Pin shortfall plus the slider's distance from the top stop, plus one per section not interlocked (m)."""
    return float(max(pin_short(objects), 0.0) + (Q_MAX - slider_travel(objects)) + (N_SEG - interlocked(objects)))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry. Left arm: pinch the hem band (fingers closing along x) 20 cm left of the zip and
# hold the jacket front still. Right arm, fingers closed: come down behind the pin's grip tab and push it 30 mm past
# home (it stops on the bottom of the box); back off, come down behind the slider's pull and run it up at 2 cm/s past
# its top stop (a fingertip push: gripping the thin pull squeezed it out like a seed); back off. Then the left lets go of the hem; both arms back away.

TIP = 0.013  # pad tips below the TCP
PAD_HALF = 0.013  # half a pad's width (26 mm, along x when the fingers close along y)
PUSH_Z = TOP + 0.004 + TIP  # pad tips 4 mm over the table (above the tape, on the tab's face)
PUSH_OVER = 0.040
PULL_Z = TOP + 0.013 + TIP  # pad tips 2.5 mm over the slider's top, on the pull's back face
PULL_OVER = 0.030
HEM_X, HEM_YH = (X_HEM + X_BOX) / 2, 0.200
HEM_Z = TOP + 0.004 + TIP
HEM_OPEN = -0.020
HEM_GRIP = -(X_BOX - X_HEM - 0.008) / 2
YAW90 = -math.pi / 2
PARK_R, PARK_L = (0.220, -0.200, TOP + 0.150), (0.220, 0.200, TOP + 0.150)
SMOKE = False  # True: the naive run (pull the slider without pushing the pin home)


def demo():
    """Pinch the hem band; push the pin home by its tab; run the slider up to its stop by its pull; let go; back away."""
    c, o = FINGER_CLOSED, FINGER_OPEN
    R = [Waypoint(2.0, PARK_R, c, "start", 0.0)]

    def w(dur, pos, f, label, yaw=0.0):
        R.append(Waypoint(dur, tuple(float(v) for v in pos), f, label, yaw))

    ty = sum(TAB["y"]) / 2
    face = TAB["x"][1] + PIN_SHORT  # the tab's back face, pin short
    if not SMOKE:
        w(2.0, (face + PAD_HALF + 0.010, ty, PUSH_Z + 0.040), c, "over the pin's tab, fingers closed")
        w(1.5, (face + PAD_HALF + 0.010, ty, PUSH_Z), c, "down behind the tab")
        w(1.0, (face + PAD_HALF + 0.001, ty, PUSH_Z), c, "fingertips onto the tab")
        w(3.0, (TAB["x"][1] + PAD_HALF - PUSH_OVER, ty, PUSH_Z), c, "push the pin home")
        w(0.5, (TAB["x"][1] + PAD_HALF - PUSH_OVER, ty, PUSH_Z), c, "hold")
        w(1.0, (TAB["x"][1] + PAD_HALF + 0.010, ty, PUSH_Z), c, "back off")
        w(1.0, (TAB["x"][1] + PAD_HALF + 0.010, ty, PUSH_Z + 0.040), c, "up")
    px, py = X_SL0 + PULL["u"][0] - PAD_HALF - 0.001, sum(PULL["y"]) / 2  # fingertips' front just behind the pull
    w(1.5, (px, py, PULL_Z + 0.045), c, "over the slider, behind its pull")
    w(1.5, (px, py, PULL_Z), c, "down behind the pull")
    w(0.5, (px, py, PULL_Z), c, "settle")
    w(10.5, (px + Q_MAX + PULL_OVER, py, PULL_Z), c, "run the slider up to its stop")
    w(0.5, (px + Q_MAX + PULL_OVER, py, PULL_Z), c, "hold")
    w(1.0, (px + Q_MAX - 0.012, py, PULL_Z), c, "back off")
    w(1.5, (px + Q_MAX - 0.012, py, PULL_Z + 0.060), c, "up")
    t_let_go = sum(x.duration for x in R)
    w(2.0, PARK_R, o, "park")
    w(2.5, PARK_R, o, "wait")
    L = [Waypoint(2.0, PARK_L, HEM_OPEN, "start", YAW90),
         Waypoint(1.5, (HEM_X, HEM_YH, HEM_Z + 0.040), HEM_OPEN, "over the hem band", YAW90),
         Waypoint(1.5, (HEM_X, HEM_YH, HEM_Z), HEM_OPEN, "down around it", YAW90),
         Waypoint(0.5, (HEM_X, HEM_YH, HEM_Z), HEM_OPEN, "settle", YAW90),
         Waypoint(1.0, (HEM_X, HEM_YH, HEM_Z), HEM_GRIP, "pinch the hem band", YAW90)]
    held = sum(x.duration for x in L)
    L.append(Waypoint(t_let_go - held + 0.5, (HEM_X, HEM_YH, HEM_Z), HEM_GRIP, "hold the jacket front still", YAW90))
    L.append(Waypoint(0.8, (HEM_X, HEM_YH, HEM_Z), HEM_OPEN, "let go", YAW90))
    L.append(Waypoint(1.5, (HEM_X, HEM_YH, HEM_Z + 0.060), HEM_OPEN, "up", YAW90))
    L.append(Waypoint(2.0, PARK_L, HEM_OPEN, "park", YAW90))
    total_r = sum(x.duration for x in R)
    total_l = sum(x.duration for x in L)
    if total_l > total_r:
        R.append(Waypoint(total_l - total_r, PARK_R, o, "wait"))
    return {"left": L, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rule on hand-built sequences of poses (one trial each, fed step by step)."""
    global _TEST_TCPS
    rest = math.radians(BEND_REST)

    def fr(short=PIN_SHORT, u=0.0, flat=(), bend=None):
        """Pin ``short`` m short, slider ``u`` up; sections in ``flat`` lying in line (bend 0), the rest at rest."""
        q = {"pin_seat": -short, "zip": u}
        for k in range(1, N_SEG + 1):
            q[f"bend{k}"] = 0.0 if k in flat else rest
        q.update(bend or {})
        return kit.fake_states(q=q)

    def pull(short, u0, u1, n=40, herd=True):
        """The slider drawn from u0 to u1; the flange herds each section into line as it passes (``herd``)."""
        out = []
        for i in range(n + 1):
            u = u0 + (u1 - u0) * i / n
            flat = {k for k in range(1, N_SEG + 1) if herd and u >= section_top(k) - SEG_L}
            out.append(fr(short, u, flat))
        return out

    def push(s0, s1, n=8):
        return [fr(s0 + (s1 - s0) * i / n) for i in range(n + 1)]

    good = push(PIN_SHORT, 0.0) + pull(0.0, 0.0, Q_MAX)
    park_tcp = (PARK_R, PARK_L)
    pull_tcp = [(X_SL0 + Q_MAX + 0.019, -0.002, TOP + 0.02)]
    cases = {  # name: (frames, TCPs at the end, score)
        "start": ([fr()], [], 0.0),
        "pin pushed home, zipped to the top stop, let go (demo)": (good, list(park_tcp), 1.0),
        "zipped to the top but still holding the pull": (good, pull_tcp, 0.5),
        "naive: slider pulled to the top with the pin 5 mm short (split)": (pull(PIN_SHORT, 0.0, Q_MAX), [], 0.0),
        "pin pushed only 3 mm (2 mm short), then zipped (split)": (push(PIN_SHORT, 0.002) + pull(0.002, 0, Q_MAX),
                                                                   [], 0.0),
        "pin pushed home, slider not moved": (push(PIN_SHORT, 0.0), [], 0.25),
        "pin home, slider drawn 3 sections up": (push(PIN_SHORT, 0.0) + pull(0.0, 0.0, section_top(3) + 0.004),
                                                 [], 0.5),
        "pin home, slider drawn 2 sections up": (push(PIN_SHORT, 0.0) + pull(0.0, 0.0, section_top(2) + 0.004),
                                                 [], 0.25),
        "split, slider run back down, pin pushed home, zipped again (recovery)":
            (pull(PIN_SHORT, 0.0, section_top(3) + 0.004) + pull(PIN_SHORT, section_top(3) + 0.004, 0.0, herd=False)
             + good, [], 1.0),
        "split half way, then the pin pushed home and the rest zipped (bottom stays split)":
            (pull(PIN_SHORT, 0.0, section_top(3) + 0.004) + push(PIN_SHORT, 0.0)
             + pull(0.0, section_top(3) + 0.004, Q_MAX), [], 0.0),
        "pin home but section 3 knocked out of line as the slider passed (split)":
            (push(PIN_SHORT, 0.0) + [fr(0.0, u, {k for k in range(1, N_SEG + 1) if k != 3 and u >= section_top(k) - SEG_L},
                                         {"bend3": math.radians(-12)})
                                      for u in np.linspace(0.0, Q_MAX, 41)], [], 0.0),
        "zipped to the top, then the slider pushed back below the last two sections":
            (good + pull(0.0, Q_MAX, section_top(4) + 0.004, herd=True), [], 0.5),
        "slider jerked up 3 sections in one step (no section interlocks, split)":
            (push(PIN_SHORT, 0.0) + [fr(0.0, section_top(3) + 0.004, {1, 2, 3})], [], 0.0),
    }
    try:
        for name, (frames, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for f in frames[:-1]:
                score(f)
            _TEST_TCPS = tcps
            got = score(frames[-1])
            assert abs(got - want) < 1e-9, (name, got, want, sorted(_mem()["latched"]))
            assert success(frames[-1]) == (want >= 1.0), name
        mk.new_episode()
        _TEST_TCPS = []
        for f in good:
            score(f)
        assert interlocked(good[-1]) == N_SEG
        mk.new_episode()
        assert score(fr()) == 0.0 and interlocked(fr()) == 0, "a new trial forgets the last one"
        # geometry: the start pose really is 5 mm short and the tape's top section curls well clear of the left half
        st = fr()
        assert abs(pin_short(st) - PIN_SHORT) < 1e-9 and not lined_up(st, "seg1")
        top = kit.point(st, SEGS[-1], (SEG_L, -GAP, 0.0))
        assert top[1] < -0.035, top
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1, float(top[1])


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rule on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n, curl = self_test()
    print(f"scoring: {n} cases pass (the loose tape's top end lies {-curl * 100:.1f} cm off the centre line)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
