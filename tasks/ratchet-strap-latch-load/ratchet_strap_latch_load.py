"""On the station table, in front of the right arm, a section of a delivery vehicle's load deck carries a small wooden
crate standing against a stake board. A ratchet tie-down is already routed round the crate: from an anchor stake on
the left it runs along the crate's side, across its far face and back along the right side into a lever ratchet
buckle bolted to the deck. The strap is slack: its band stands 12 mm off the crate's face. The buckle's handle (a
steel lever on a vertical pin, a grip post at its end) stands part open, 40 deg out from the strap.

The ratchet works as a real one does. Each full stroke of the handle (swung out past 75 deg and back in under 35
deg) turns the mandrel one tooth and winds 5 mm of strap; a pawl holds what has been wound. The strap is stiff
webbing: once its band meets the crate every further tooth adds about 40 N of tension (it stops winding at 6
teeth, as tight as the handle will pull). Closing the handle flat needs a firm push past its detent (a ridge
pushes it back out to 12 deg until it is pressed within 1 deg of shut); there it latches and stays. A handle swung
right out to its release position (150 deg) lifts the pawls and the strap pays out.

Then the trip. Once the robot has let go and stood clear for 1 s, the deck (a 20 kg body on a slide) is pulled
forward gently (0.5 m/s^2) and runs 15 cm into its buffer stop, a hard stop at about 0.38 m/s. A taut strap with its handle
latched holds the crate within a few mm. A slack strap lets the crate pitch forward into the band. A handle left
open is free to swing: it keeps going when the deck stops, and from about 45 deg open it reaches the release
position, the strap pays out and the crate tips.

The job: tension the strap with full strokes until it is taut, then close the handle flat into its latch, let go
and stand clear.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/ratchet-strap-latch-load/ratchet_strap_latch_load.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinge, sliders, per-step rules, per-episode memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the mechanism's events and a readout every 5 steps
VARIANT = "demo"  # sim smoke tests: "demo", "open" (4 strokes, handle left open), "slack" (1 stroke, handle up)

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the deck section: a 20 kg body on a slide along x (the jolt), hovering 2 mm over the table ----------------------
HOVER, DECK_T = 0.002, 0.010
DZ = TOP + HOVER + DECK_T  # deck top
DECK_O = (0.290, -0.110, TOP + HOVER)  # deck part origin (plate bottom centre)
DECK_KG = 20.0
RUN = 0.150  # travel to the buffer stop
K_DECK, FAR = 2.0, 5.0  # drive: target -5 m holds it on its rear stop with 10 N; +5 m pulls it forward with 10 N
# -- the crate (free, on the deck, against the stake board) ---------------------------------------------------------
CRATE = (0.055, 0.100, 0.140)  # x (front to back), y, z
CX, CY = 0.283, -0.075  # bottom centre
CRATE_KG = 1.0
STAKE_X = CX - CRATE[0] / 2 - 0.0005  # the stake board's front face
# -- the strap: a rigid band round the crate's far face on a slide (take-up) to the deck --------------------------
GAP = 0.012  # slack: the band's inner face this far from the crate's far face
PITCH = 0.005  # strap wound per tooth (one full stroke)
K_S = 8000.0  # webbing stiffness, N/m (40 N per tooth once the band is on the crate)
MAX_TEETH = 6
P_MIN = 50.0  # taut: at least 50 N in the strap
STRAP_T, BAND_H, BAND_Z = 0.003, 0.030, 0.090  # thickness, band height, band centre above the deck top
LEG_OFF = CRATE[1] / 2 + 0.004 + STRAP_T / 2  # legs 4 mm clear of the crate's sides
SX = CX + CRATE[0] / 2 + GAP + STRAP_T / 2  # band centre x at spawn
LEG_END_X = 0.266
PAYOUT, TAKE_MAX = -0.060, 0.040
STRAP_KG = 0.10
# -- the ratchet handle: a steel lever on a vertical pin; 0 deg = shut, lying along -x; opens toward -y -----------
HX, HY = 0.262, -0.145
HZ = DZ + BAND_Z
R_POST = 0.064  # grip post's centre from the pin
POST, POST_H = 0.012, 0.036
HANDLE_KG = 0.06
H_FRICTION = 0.002  # N*m: the handle stays where it is left
START_DEG = 40.0
STROKE_OPEN, STROKE_BACK = 75.0, 35.0  # a stroke: out past 75 deg, then back under 35 deg
RIDGE_DEG = 12.0  # the detent's ridge pushes an unlatched handle back out to here
LATCH_DEG = 1.0  # pressed this far shut, the latch engages
SHUT_DEG = 1.5  # latched and shut
HOLD_DEG = -8.0  # latched: the catch holds it on its stop (drive target past the stop)
RELEASE_DEG = 150.0  # handle opened right out: pawls lifted, strap pays out
FREE_S = 0.4  # after the stop, an unlatched handle's friction is off this long (it swings on its own momentum)
PARTS = ("crate", "strap", "handle")

WOOD, STEEL, WEB = (0.72, 0.55, 0.34), (0.30, 0.31, 0.33), (0.95, 0.62, 0.10)
DECK_C, STAKE_C, GRIP_C = (0.32, 0.33, 0.35), (0.45, 0.36, 0.26), (0.80, 0.12, 0.10)

kit = mk.Kit()
_dz = DZ - DECK_O[2]  # deck top in the deck frame


def _d(x, y, z):  # world -> deck frame
    return (x - DECK_O[0], y - DECK_O[1], z - DECK_O[2])


kit.part("deck", DECK_O, [
    mk.box((0.240, 0.240, DECK_T), at=(0, 0, DECK_T / 2)),  # chequer-plate steel deck section
    mk.box((0.012, CRATE[1] + 0.006, 0.120), at=_d(STAKE_X - 0.006, CY, DZ + 0.060), color=STAKE_C),  # stake board
    mk.box((0.012, 0.010, 0.115), at=_d(0.262, CY + LEG_OFF + 0.0095, DZ + 0.0575), color=STEEL),  # anchor stake
    mk.box((0.026, 0.016, BAND_Z - 0.0095), at=_d(HX + 0.004, HY, DZ + (BAND_Z - 0.0095) / 2), color=STEEL),
    mk.box((0.012, 0.010, 0.034), at=_d(HX + 0.006, -0.134, HZ), color=STEEL),  # mandrel cheek
], mass=DECK_KG, color=DECK_C)
kit.slider("deck_run", None, "deck", axis=(1, 0, 0), lower=0.0, upper=RUN, start=0.0, stiffness=K_DECK, rest=-FAR)

kit.part("crate", (CX, CY, DZ), [
    mk.box(CRATE, at=(0, 0, CRATE[2] / 2)),
    mk.box((CRATE[0] - 0.006, 0.012, 0.002), at=(0, -0.030, CRATE[2] + 0.001), color=(0.55, 0.40, 0.24)),
    mk.box((CRATE[0] - 0.006, 0.012, 0.002), at=(0, 0.030, CRATE[2] + 0.001), color=(0.55, 0.40, 0.24)),
], mass=CRATE_KG, color=WOOD)

_leg_len = SX + STRAP_T / 2 - LEG_END_X
kit.part("strap", (SX, CY, HZ), [
    mk.box((STRAP_T, 2 * LEG_OFF + STRAP_T, BAND_H)),
    mk.box((_leg_len, STRAP_T, BAND_H), at=(STRAP_T / 2 - _leg_len / 2, -LEG_OFF, 0)),
    mk.box((_leg_len, STRAP_T, BAND_H), at=(STRAP_T / 2 - _leg_len / 2, LEG_OFF, 0)),
], mass=STRAP_KG, color=WEB)
# take-up: positive pulls the band back toward the stake board (-x)
kit.slider("take_up", "deck", "strap", axis=(-1, 0, 0), lower=PAYOUT, upper=TAKE_MAX, start=0.0, stiffness=K_S,
           rest=0.0)

kit.part("handle", (HX, HY, HZ), [
    mk.box((0.012, 0.012, 0.018)),  # knuckle on the pin
    mk.box((0.070, 0.010, 0.008), at=(-0.037, 0, 0)),
    mk.box((POST, POST, POST_H), at=(-R_POST, 0, 0.004 + POST_H / 2), color=GRIP_C),  # grip post
], mass=HANDLE_KG, color=STEEL)
kit.hinge("handle_hinge", "deck", "handle", anchor=(HX, HY, HZ), axis=(0, 0, 1), lower_deg=0.0, upper_deg=170.0,
          start_deg=START_DEG, friction=H_FRICTION)

OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the mechanism's per-step rules (sim) and the scoring ---------------------------------------------------------
DT = 0.05  # control step
QUIET_STEPS = 20  # 1 s let go and still before the jolt
CLEAR = 0.03  # "stood clear": both TCPs 3 cm from the crate, strap and handle
SETTLE_STEPS = 20  # the jolt is over 1 s after the stop
KNOCK_MM, KNOCK_DEG = 3.0, 2.0  # before the jolt: the crate must not have been moved more than this
SHIFT_MM, TILT_DEG = 2.5, 2.0  # through the jolt: the crate must stay within this of where it stood
WIND_STEP = 0.001  # the mandrel winds a tooth's strap in over 5 control steps
M = "rslt"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"n": 0, "init": False, "teeth": 0, "armed": False, "cmd": 0.0, "latched": False,
                            "released": False, "worked": False, "knocked": False, "quiet": 0, "jolt": None,
                            "t_hit": None, "free": None, "d_prev": 0.0, "base": None, "shift": 0.0, "tilt": 0.0,
                            "shifted": False, "prev": {}, "still": {}}
    return m


def handle_deg(objects) -> float:
    """How far the handle is open (deg; 0 = shut flat)."""
    return kit.q_deg(objects, "handle_hinge")


def take_up(objects) -> float:
    """How far the band has been drawn back from where it started (m): its centre's move toward the stake board in
    the deck's frame (measured at the band itself, so a hair of pitch in the band does not count as travel)."""
    rel = mk._inv(kit._pose_T(objects, "deck")) @ kit._pose_T(objects, "strap")
    return float(_STRAP_REL0[0, 3] - rel[0, 3])


def deck_travel(objects) -> float:
    """How far the deck has run forward from its rear stop (m), at its own origin."""
    return float(kit._pose_T(objects, "deck")[0, 3] - DECK_O[0])


def wound(m=None) -> float:
    """Strap wound onto the mandrel (m): 5 mm per tooth, as wound in so far."""
    return (m or _mem())["cmd"]


def tension(objects) -> float:
    """Strap tension (N): the webbing's stretch (wound minus where the band actually is) times its stiffness."""
    m = _mem()
    if m["released"]:
        return 0.0
    return max(0.0, K_S * (wound(m) - take_up(objects)))


_DECK_REF = kit.parts["deck"].ref
_STRAP_REL0 = mk._inv(_DECK_REF) @ kit.parts["strap"].ref
_CRATE_REF = kit.parts["crate"].ref
_REL0 = mk._inv(_DECK_REF) @ _CRATE_REF  # the crate in the deck's frame at the start
_PTS = [np.array([0, 0, 0.0]), np.array([0, 0, CRATE[2]])]  # bottom and top centre (crate frame)


def _crate_rel(objects) -> np.ndarray:
    return mk._inv(kit._pose_T(objects, "deck")) @ kit._pose_T(objects, "crate")


def _moved(rel, ref) -> tuple:
    """(mm, deg) the crate has moved in the deck's frame, from ``ref``: the larger move of its bottom and top
    centres, and its turn."""
    d = max(float(np.linalg.norm(rel[:3, :3] @ p + rel[:3, 3] - ref[:3, :3] @ p - ref[:3, 3])) for p in _PTS)
    return d * 1000.0, math.degrees(mk._rot_angle(rel[:3, :3] @ ref[:3, :3].T))


def _set_drive(name, target=None, stiffness=None) -> None:
    """Sim only: set joint ``name``'s drive target (rad or m) and/or stiffness (SI) now."""
    path = mk._SIM["joint_paths"].get((kit.id, name))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    j = kit.joints_[name]
    hinge = j.kind == "hinge"
    scale = 180.0 / math.pi if hinge else 1.0
    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "angular" if hinge else "linear")
    if target is not None:
        if j.friction > 0:
            mk._SIM["stick"][(kit.id, name)] = target
        d.GetTargetPositionAttr().Set(float(target * scale))
    if stiffness is not None:
        d.GetStiffnessAttr().Set(float(stiffness / scale))


_K_STICK = mk._k_stick(kit.joints_["handle_hinge"])


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit rslt] step {mk._EP.get('step')}: {msg}", flush=True)


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def _rules(objects) -> None:
    """One control step of the ratchet (strokes, pawl, detent, latch, release) and of the trip (the jolt)."""
    m = _mem()
    sim = mk._EP.get("env") is not None
    m["n"] += 1
    if not m["init"]:  # a new episode: deck on its rear stop, strap slack and held, handle on its friction
        m["init"] = True
        _set_drive("deck_run", target=-FAR)
        _set_drive("take_up", target=0.0, stiffness=K_S)
        _set_drive("handle_hinge", stiffness=_K_STICK)
    for p in (*PARTS, "deck"):  # still: under 0.5 mm and 1.4 deg of motion in a control step (10 mm/s, 0.5 rad/s)
        t = kit._pose_T(objects, p)
        t0 = m["prev"].get(p)
        m["still"][p] = t0 is None or bool(np.linalg.norm(t[:3, 3] - t0[:3, 3]) < 0.0005
                                           and mk._rot_angle(t[:3, :3] @ t0[:3, :3].T) < 0.5 * DT)
        m["prev"][p] = t
    a = handle_deg(objects)
    if abs(a - START_DEG) >= 15.0:
        m["worked"] = True
    # -- the handle: release, strokes, detent and latch
    if not m["released"] and a >= RELEASE_DEG:
        m["released"] = True
        _set_drive("take_up", stiffness=0.0)
        _say(f"handle opened to {a:.1f} deg: pawls lifted, strap paying out")
    if not m["latched"] and not m["released"]:
        if a >= STROKE_OPEN:
            m["armed"] = True
        elif m["armed"] and a <= STROKE_BACK:
            m["armed"] = False
            if m["teeth"] < MAX_TEETH:
                m["teeth"] += 1
                _say(f"stroke: tooth {m['teeth']}")
        if a <= LATCH_DEG:
            m["latched"] = True
            _say(f"handle LATCHED at {a:.2f} deg, {m['teeth']} teeth")
    if m["free"] is None:
        if m["latched"]:
            _set_drive("handle_hinge", target=math.radians(HOLD_DEG))
        elif sim and mk._SIM["stick"].get((kit.id, "handle_hinge"), 1.0) < math.radians(RIDGE_DEG):
            _set_drive("handle_hinge", target=math.radians(RIDGE_DEG))
    # -- the pawl: the mandrel winds the wound strap in, a tooth over 5 steps
    if not m["released"]:
        goal = m["teeth"] * PITCH
        if m["cmd"] < goal:
            m["cmd"] = min(goal, m["cmd"] + WIND_STEP)
            _set_drive("take_up", target=m["cmd"])
    # -- the crate before the trip: nobody may shove it about
    if m["jolt"] is None:
        mm, deg = _moved(_crate_rel(objects), _REL0)
        if mm > KNOCK_MM or deg > KNOCK_DEG:
            if not m["knocked"]:
                _say(f"crate moved {mm:.1f} mm / {deg:.1f} deg before the trip")
            m["knocked"] = True
    # -- the trip: 1 s after the robot has let go and stood clear, the deck runs into its stop
    d = deck_travel(objects)
    if m["jolt"] is None:
        quiet = (m["worked"] and all(_released(objects[p], CLEAR) for p in PARTS)
                 and all(m["still"][p] for p in (*PARTS, "deck")))
        m["quiet"] = m["quiet"] + 1 if quiet else 0
        if m["quiet"] >= QUIET_STEPS:
            m["jolt"], m["base"] = "go", _crate_rel(objects)
            _set_drive("deck_run", target=FAR)
            _say(f"TRIP: deck pulled forward; tension {tension(objects):.0f} N, handle {a:.1f} deg, "
                 f"latched {m['latched']}")
    elif m["jolt"] == "go":
        v = (d - m["d_prev"]) / DT
        if m["free"] is None and not m["latched"] and d + 1.5 * v * DT >= RUN - 0.002:
            m["free"] = m["n"]  # the handle is on its own momentum from here (friction can't follow a fast swing)
            _set_drive("handle_hinge", stiffness=0.0)
            _say(f"handle free at {a:.1f} deg, deck {d * 1000:.1f} mm at {v:.2f} m/s")
        if d >= RUN - 0.002:
            m["jolt"], m["t_hit"] = "hit", m["n"]
            _say(f"deck on its stop at about {v:.2f} m/s")
    if m["jolt"] in ("hit", "done"):
        if m["free"] is not None and m["n"] - m["t_hit"] >= round(FREE_S / DT):
            m["free"] = None
            _set_drive("handle_hinge", target=math.radians(a), stiffness=_K_STICK)
            _say(f"handle caught by its friction at {a:.1f} deg")
        if m["jolt"] == "hit" and m["n"] - m["t_hit"] >= SETTLE_STEPS:
            m["jolt"] = "done"
            _say(f"trip over: crate moved at most {m['shift']:.1f} mm / {m['tilt']:.1f} deg")
    if m["jolt"] is not None:
        mm, deg = _moved(_crate_rel(objects), m["base"])
        m["shift"], m["tilt"] = max(m["shift"], mm), max(m["tilt"], deg)
        if mm > SHIFT_MM or deg > TILT_DEG:
            m["shifted"] = True
    m["d_prev"] = d
    if DEBUG and sim and (m["n"] % 5 == 0 or m["jolt"] in ("go", "hit")):
        tc = mk.tcps()
        tz = f"({tc[1][0]:.3f},{tc[1][1]:.3f},{tc[1][2] - DZ:.3f})" if tc else "-"
        _say(f"R tcp {tz}; handle {a:.1f} deg, teeth {m['teeth']}, wound {m['cmd'] * 1000:.1f} mm, band "
             f"{take_up(objects) * 1000:.1f} mm, T {tension(objects):.0f} N, deck {d * 1000:.1f} mm, crate "
             f"{m['shift']:.1f} mm/{m['tilt']:.1f} deg, jolt {m['jolt']}, latched {m['latched']}, "
             f"released {m['released']}")
        sp = " ".join(f"{p}:{int(m['still'][p])}" for p in (*PARTS, "deck"))
        bf = kit.point(objects, "strap", (-STRAP_T / 2, 0, 0))[0]
        cf = kit.point(objects, "crate", (CRATE[0] / 2, 0, BAND_Z))[0]
        _say(f"   speeds {sp}; band face - crate face {(bf - cf) * 1000:.2f} mm; quiet {m['quiet']}")


kit.on_step(_rules)


def taut(objects) -> bool:
    """At least 50 N in the strap (pawl holding, band on the crate)."""
    return tension(objects) >= P_MIN


def latched(objects) -> bool:
    """The handle pressed past its detent into the latch, and shut flat now."""
    return bool(_mem()["latched"] and handle_deg(objects) <= SHUT_DEG)


def score(objects) -> float:
    """0 if the strap was let out (handle opened to its release position), the crate was shoved before the trip, or
    the crate moved more than 2.5 mm / 2 deg on the deck through the jolt. Otherwise 0.25 for a taut strap (50 N or
    more); with that, +0.25 for the handle latched shut; with both, +0.5 once the jolt is over. Halved unless both
    grippers are 2.5 cm clear of the crate, strap and handle and everything is still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _rules(objects)
    m = _mem()
    if m["released"] or m["knocked"] or m["shifted"]:
        return 0.0
    base = 0.0
    if taut(objects):
        base = 0.25
        if latched(objects):
            base += 0.25
            if m["jolt"] == "done":
                base += 0.5
    clear = all(_released(objects[p]) and m["still"].get(p, True) for p in PARTS)
    return round(base if clear else base / 2, 6)


def success(objects) -> bool:
    """Strap taut, handle latched, the crate held through the jolt; let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Stages still to do (taut, latched, the trip) plus the tension still missing as a fraction of 50 N."""
    m = _mem()
    return float((not taut(objects)) + (not latched(objects)) + (m["jolt"] != "done")
                 + max(0.0, P_MIN - tension(objects)) / P_MIN)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry; right arm only, the left stays parked. The fingers, opened 20 mm, straddle
# the handle's grip post (12 mm) with the grip turned so they close across the handle; the hand then follows the
# post's circle round the pin: out to 100 deg and back to 22 deg four times (the post lags the hand by the 4 mm
# play, about 3.6 deg), then in past shut to press the handle onto its stop through the detent. Then the fingers
# open, lift clear of the post and the arm parks and waits for the trip.

TIP = 0.013  # pad tips below the TCP
PAD_UP = 0.010  # pad tips this far above the handle's centre line (6 mm over the bar)
STRADDLE = 0.020  # finger opening round the post
OUT_DEG, BACK_DEG, SHUT_CMD = 100.0, 22.0, -6.0
STEP_DEG = 13.0
PARK = (0.200, -0.300, TOP + 0.200)


def _post(deg):
    t = math.radians(deg)
    return (HX - R_POST * math.cos(t), HY - R_POST * math.sin(t))


def demo():
    """Straddle the grip post; four full strokes (out to 100 deg, back to 22 deg); press the handle shut through its
    detent; let go, lift off, park and wait while the deck is jolted."""
    sq = -STRADDLE / 2
    wide = -0.015
    tz = HZ + PAD_UP + TIP
    R = [Waypoint(2.0, PARK, FINGER_OPEN * 0.5, "start", 0.0)]

    def w(dur, deg, z, f, label):
        x, y = _post(deg)
        R.append(Waypoint(dur, (x, y, z), f, label, math.radians(deg)))

    def arc(a, b, dur, label):
        n = max(1, math.ceil(abs(b - a) / STEP_DEG))
        for k in range(1, n + 1):
            w(dur / n, a + (b - a) * k / n, tz, sq, label)

    w(2.0, START_DEG, tz + 0.060, sq, "over the grip post, fingers part open")
    w(1.5, START_DEG, tz, sq, "down round the post")
    w(0.4, START_DEG, tz, sq, "settle")
    strokes = {"demo": 4, "open": 4, "slack": 1}[VARIANT]
    a = START_DEG
    for i in range(strokes):
        arc(a, OUT_DEG, 1.6, f"stroke {i + 1}: out")
        w(0.2, OUT_DEG, tz, sq, "turn")
        if VARIANT != "demo" and i == strokes - 1:
            break  # naive: the handle is left up where the last stroke ended
        arc(OUT_DEG, BACK_DEG, 1.6, f"stroke {i + 1}: back")
        w(0.2, BACK_DEG, tz, sq, "turn")
        a = BACK_DEG
    last = OUT_DEG if VARIANT != "demo" else BACK_DEG
    if VARIANT == "demo":
        arc(BACK_DEG, SHUT_CMD, 1.5, "close the handle flat, through the detent")
        w(0.8, SHUT_CMD, tz, sq, "press it home")
        last = SHUT_CMD
    w(0.5, last, tz, wide, "open the fingers")
    w(1.0, last, tz + 0.060, wide, "lift clear of the post")
    R.append(Waypoint(2.0, PARK, wide, "park", 0.0))
    R.append(Waypoint(8.0, PARK, wide, "wait: the trip", 0.0))
    total = sum(x.duration for x in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park")]
    return {"left": left, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of poses (one trial each, fed one control step at a time). The
    band's position is what the physics gives: it follows the wound strap until it meets the crate."""
    global _TEST_TCPS
    up = (0.0, 0.0, 0.0, 1.0)
    near = (*_post(0.0), HZ + PAD_UP + TIP)  # a gripper still round the shut handle's grip post

    def state(h, d=0.0, crate=(0.0, 0.0, 0.0), tilt=0.0, band=None):
        m = _mem()
        if band is None:
            band = PAYOUT if m["released"] else min(wound(m), GAP + 0.0005)
        pos = np.asarray(_CRATE_REF[:3, 3]) + np.array([d, 0, 0]) + np.asarray(crate)
        q = up if not tilt else tuple(mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.radians(tilt))))
        return kit.fake_states(q={"handle_hinge": math.radians(h), "deck_run": d, "take_up": band},
                               moved={"crate": (tuple(pos), q)})

    def lin(a, b, n=8):
        return [a + (b - a) * k / n for k in range(n + 1)]

    def strokes(k, a0=START_DEG, end=BACK_DEG):  # handle angles of k full strokes (out to 96, back to 26)
        out, a = [], a0
        for _ in range(k):
            out += lin(a, 96.0) + lin(96.0, 26.0)
            a = 26.0
        return out + (lin(26.0, end) if end != 26.0 else [])

    def run(plan, tcps_end=()):
        """plan: a list of steps, each a handle angle (hand on the grip post) or a dict for state() with "hand";
        returns the last score and state."""
        global _TEST_TCPS
        got = st = None
        for i, s in enumerate(plan):
            kw = dict(s) if isinstance(s, dict) else {"h": s}
            hand = kw.pop("hand", True)
            if i == len(plan) - 1:
                _TEST_TCPS = list(tcps_end)
            else:
                _TEST_TCPS = [(*_post(kw["h"]), HZ + PAD_UP + TIP)] if hand else []
            st = state(**kw)
            got = score(st)
        return got, st

    def away(h, n=QUIET_STEPS + 2):  # let go and stand clear
        return [{"h": h, "hand": False}] * n

    def trip(h, shift=0.0, tilt=0.0, n_run=16, settle=SETTLE_STEPS + 3, band=None):
        """The deck's run to its stop, the crate shifted by ``shift`` m (and tilted) at its worst, then back."""
        out = []
        for k in range(1, n_run + 1):
            out.append({"h": h, "d": RUN * (k / n_run) ** 2, "hand": False, "band": band})
        for k in range(settle):
            s = shift if k < 4 else shift * 0.25
            out.append({"h": h, "d": RUN, "crate": (s, 0, 0), "tilt": tilt if k < 4 else 0.0, "hand": False,
                        "band": band})
        return out

    good = strokes(4, end=0.0) + away(0.0)
    cases = {  # name: (plan, TCPs at the last step, score)
        "start": ([START_DEG], [], 0.0),
        "four strokes, latched, let go, the trip, the crate held (demo)": (good + trip(0.0, 0.001), [], 1.0),
        "the same, the score read before the trip": (good, [], 0.5),
        "four strokes, latched, hand kept on the handle (no trip)":
            (strokes(4, end=0.0) + [0.0] * 30, [near], 0.25),
        "naive: one stroke, handle left up; the trip swings it out, strap pays out":
            (strokes(1)[:9] + away(96.0) + trip(96.0, 0.0, band=None)[:4]
             + [{"h": 160.0, "d": RUN, "crate": (0.03, 0, 0), "tilt": 15.0, "hand": False}] * 25, [], 0.0),
        "naive: one stroke, handle left up, nothing swings (the slack band lets the crate pitch 7 mm)":
            (strokes(1)[:9] + away(96.0) + trip(96.0, 0.007, tilt=3.0), [], 0.0),
        "naive (as in sim): strokes, handle left up at 97 deg; the stop swings it out to 164 deg, strap let out":
            (strokes(4)[:-9] + away(97.0) + trip(97.0)[:16]
             + [{"h": 164.0, "d": RUN, "crate": (0.0031, 0, 0), "tilt": 0.8, "hand": False}] * 25, [], 0.0),
        "four strokes then latched, but the crate jolted 3 mm on the deck": (good + trip(0.0, 0.003), [], 0.0),
        "four strokes, handle left 26 deg open (not latched), the crate held": (strokes(4) + away(26.0)
                                                                              + trip(26.0, 0.001), [], 0.25),
        "two strokes then latched (slack), the crate pitches 8 mm": (strokes(2, end=0.0) + away(0.0)
                                                                      + trip(0.0, 0.008, tilt=3.0), [], 0.0),
        "three strokes then latched (24 N, not taut), held": (strokes(3, end=0.0) + away(0.0) + trip(0.0, 0.002),
                                                               [], 0.0),
        "over-swung to 155 deg mid-stroke (pawls lifted), then four strokes and latched":
            (lin(START_DEG, 155.0) + strokes(4, a0=155.0, end=0.0) + away(0.0) + trip(0.0, 0.001), [], 0.0),
        "the crate shoved 5 mm by the robot before strapping":
            ([{"h": START_DEG, "crate": (-0.005, 0, 0)}] + strokes(4, end=0.0) + away(0.0) + trip(0.0, 0.001),
             [], 0.0),
        "six strokes (the most it winds), latched, the trip, held": (strokes(8, end=0.0) + away(0.0)
                                                                     + trip(0.0, 0.0005), [], 1.0),
        "latched and taut, but the crate tilts 3 deg in the jolt": (good + trip(0.0, 0.001, tilt=3.0), [], 0.0),
        "short strokes (out to 60 deg only, never counted), latched": (
            lin(START_DEG, 60.0) + (lin(60.0, 26.0) + lin(26.0, 60.0)) * 4 + lin(60.0, 0.0) + away(0.0)
            + trip(0.0, 0.0), [], 0.0),
        "demo, but a gripper still on the post at the end": (good + trip(0.0, 0.001), [(near[0] + RUN, *near[1:])],
                                                              0.5),
    }
    try:
        for name, (plan, tcps, want) in cases.items():
            mk.new_episode()
            got, st = run(plan, tcps)
            assert abs(got - want) < 1e-9, (name, got, want, {k: v for k, v in _mem().items() if k != "base"})
            assert success(st) == (want >= 1.0), name
        mk.new_episode()
        run(good + trip(0.0, 0.001))
        teeth = _mem()["teeth"]
        mk.new_episode()
        assert score(state(START_DEG)) == 0.0 and _mem()["teeth"] == 0, "a new trial forgets the last one"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1, teeth


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n, teeth = self_test()
    print(f"scoring: {n} cases pass (demo winds {teeth} teeth: {K_S * (teeth * PITCH - GAP - 0.0005):.0f} N in the strap)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
