"""On the station table, a non-powered roller conveyor runs straight away from the robot, in front of the right
arm, from just ahead of the arm bases, past the far edge of the table on a leg, to an end stop. The last 22.5 cm
before the end stop is the packing station, marked with yellow tape on both side rails and a yellow line across the
rollers. An open
plastic tote stands on the rollers at the near end; a tall square HDPE bottle (a 500 mL reagent bottle, half full)
stands loose in the middle of it.

The rollers are free-running: a tote given a push rolls on and slows down only by the rollers' rolling resistance
(about 2 % of its weight, so it loses about 0.2 m/s every second). The station is beyond the arms' reach at the
height of the tote, so the tote can only get there under its own way: pushed too softly, it stops short in the
lane; shoved too hard, it is still moving fast when it reaches the end stop, the tote stops dead and the tall
bottle falls over.

The job: send the tote down the conveyor to the packing station with the bottle still standing.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/coast-tote-to-far-stop/coast_tote_to_far_stop.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (parts, per-step rules, per-episode memory)

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the tote's travel and speed and the bottle's tilt every 5 steps
VARIANT = "demo"  # sim smoke tests: "soft" (push at 0.30 m/s), "hard" (a 2.5 m/s^2 shove to 0.75 m/s)

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the roller conveyor ------------------------------------------------------------------------------------------
LY = -0.200  # lane centre line
X_IN, X_END = 0.105, 1.040  # conveyor frame, near and far ends (it runs on past the table's edge at 0.91)
BED_H = 0.030  # roller tops this far above the table
BED_Z = TOP + BED_H
LANE_W = 0.130  # between the side rails
RAIL_W, RAIL_UP = 0.012, 0.020  # side rails: width, height above the rollers
STOP_X = 1.025  # the end stop's face
STOP_UP = 0.045  # end stop height above the rollers
MU_ROLL = 0.02  # rolling resistance of the free roller bed (the bed's friction coefficient)
ROLL_PITCH, ROLL_W = 0.040, 0.016  # rollers (drawn)
# -- the tote and its load ----------------------------------------------------------------------------------------
TOTE = (0.140, 0.120, 0.060)  # outside length (x), width, height
WALL, FLOOR = 0.004, 0.005
TOTE_KG = 0.35
X_T0 = 0.245  # tote centre at the start (rear face at x 0.175)
BOTTLE = (0.060, 0.060, 0.160)
CAP = (0.028, 0.028, 0.014)
BOTTLE_KG = 0.50
# -- the station ----------------------------------------------------------------------------------------------------
ZONE_REAR = 0.800  # the tote is at the station once its rear face is past this line (its front then at 0.94+)
REAR_MAX = STOP_X - TOTE[0]  # rear face with the tote against the end stop (0.885)

GREY, STEEL, DARK = (0.55, 0.57, 0.60), (0.40, 0.42, 0.45), (0.20, 0.21, 0.23)
BLUE, WHITE, YELLOW, RED = (0.15, 0.35, 0.70), (0.92, 0.92, 0.90), (0.95, 0.80, 0.10), (0.80, 0.15, 0.12)

kit = mk.Kit()
_XC = (X_IN + X_END) / 2
_LEN = X_END - X_IN
kit.part("conveyor", (_XC, LY, TOP), [
    mk.box((_LEN, LANE_W, BED_H), at=(0, 0, BED_H / 2), color=GREY),  # the bed (the rollers' common top plane)
    mk.box((_LEN, RAIL_W, BED_H + RAIL_UP), at=(0, (LANE_W + RAIL_W) / 2, (BED_H + RAIL_UP) / 2)),
    mk.box((_LEN, RAIL_W, BED_H + RAIL_UP), at=(0, -(LANE_W + RAIL_W) / 2, (BED_H + RAIL_UP) / 2)),
    mk.box((X_END - STOP_X, LANE_W, BED_H + STOP_UP), at=((STOP_X + X_END) / 2 - _XC, 0, (BED_H + STOP_UP) / 2),
           color=RED),  # end stop
    mk.box((0.040, LANE_W, TOP - 0.02), at=(X_END - 0.03 - _XC, 0, -(TOP - 0.02) / 2)),  # leg past the table's edge
], color=STEEL)
# The rollers and the station's tape are drawn on the bed (no collision: the bed's flat top carries the tote).
_rollers = [mk.box((ROLL_W, LANE_W - 0.002, 0.0008), at=(x - _XC, 0, BED_H + 0.0004), color=DARK)
            for x in np.arange(X_IN + 0.020, STOP_X - 0.005, ROLL_PITCH)]
_tape = [mk.box((STOP_X - ZONE_REAR, 0.008, 0.0008), at=((STOP_X + ZONE_REAR) / 2 - _XC, s * (LANE_W / 2 + RAIL_W / 2),
                                                         BED_H + RAIL_UP + 0.0004), color=YELLOW) for s in (1, -1)]
_tape.append(mk.box((0.010, LANE_W - 0.002, 0.0010), at=(ZONE_REAR - _XC, 0, BED_H + 0.0005), color=YELLOW))
kit.part("rollers", (_XC, LY, TOP), _rollers + _tape, color=DARK)

kit.part("tote", (X_T0, LY, BED_Z + 0.001), [
    mk.box((TOTE[0], TOTE[1], FLOOR), at=(0, 0, FLOOR / 2)),
    mk.box((WALL, TOTE[1], TOTE[2]), at=(TOTE[0] / 2 - WALL / 2, 0, TOTE[2] / 2)),
    mk.box((WALL, TOTE[1], TOTE[2]), at=(-TOTE[0] / 2 + WALL / 2, 0, TOTE[2] / 2)),
    mk.box((TOTE[0] - 2 * WALL, WALL, TOTE[2]), at=(0, TOTE[1] / 2 - WALL / 2, TOTE[2] / 2)),
    mk.box((TOTE[0] - 2 * WALL, WALL, TOTE[2]), at=(0, -TOTE[1] / 2 + WALL / 2, TOTE[2] / 2)),
], mass=TOTE_KG, color=BLUE)
kit.part("bottle", (X_T0, LY, BED_Z + FLOOR + 0.002), [
    mk.box(BOTTLE, at=(0, 0, BOTTLE[2] / 2)),
    mk.box(CAP, at=(0, 0, BOTTLE[2] + CAP[2] / 2), color=RED),
], mass=BOTTLE_KG, color=WHITE)

OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


# -- sim set-up: the roller bed's rolling resistance -----------------------------------------------------------------
def _roller_bed(stage) -> None:
    """Sim only, once the scene is built: give the conveyor's contact surfaces the roller bed's rolling resistance
    (friction MU_ROLL, combined by minimum, so a tote on it rolls on), and make the drawn rollers and tape visual."""
    from pxr import PhysxSchema, Usd, UsdPhysics

    paths = mk._SIM.get("paths") or {}
    n_mat = n_vis = 0
    if "conveyor" in paths:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(paths["conveyor"])):
            if prim.HasAPI(UsdPhysics.MaterialAPI):
                m = UsdPhysics.MaterialAPI(prim)
                m.CreateStaticFrictionAttr().Set(MU_ROLL)
                m.CreateDynamicFrictionAttr().Set(MU_ROLL)
                PhysxSchema.PhysxMaterialAPI.Apply(prim).CreateFrictionCombineModeAttr().Set("min")
                n_mat += 1
    if "rollers" in paths:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(paths["rollers"])):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                n_vis += 1
    print(f"[coast] roller bed: {n_mat} bed materials set to mu {MU_ROLL} (min); {n_vis} drawn parts non-colliding",
          flush=True)


if not getattr(mk, "_coast_bed_hook", False):
    _author0 = mk._author_all

    def _author_with_bed(stage, env_path):
        _author0(stage, env_path)
        _roller_bed(stage)

    mk._author_all = _author_with_bed
    mk._coast_bed_hook = True

# -- the per-step watch and the scoring ---------------------------------------------------------------------------
TIP_DEG = 15.0  # the bottle has fallen once it leans past this (latched)
TOTE_TILT_DEG = 8.0  # the tote was lifted or tipped off the rollers (latched)
LIFT = 0.012  # ... or raised this far off the bed (latched)
STILL_M = 0.00025  # still: moved under 0.25 mm in a control step (5 mm/s)
CLEAR = 0.03  # both grippers this far from the tote and the bottle
BANDS = ((ZONE_REAR - 0.12, 0.25), (ZONE_REAR - 0.05, 0.5))  # rear face past these lines, stopped short of the zone
DT = 0.05
M = "coast"
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"n": 0, "tipped": False, "out": False, "lifted": False, "still": True,
                            "prev": None, "v": 0.0, "vmax": 0.0, "v_stop": None, "rear_max": 0.0}
    return m


def _tilt_deg(objects, part) -> float:
    r = mk.quat_to_mat(kit.pose(objects, part)[1])
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def rear(objects) -> float:
    """x of the tote's rear face (at its floor, on the centre line)."""
    return float(kit.point(objects, "tote", (-TOTE[0] / 2, 0.0, 0.0))[0])


def bottle_in_tote(objects) -> bool:
    """The bottle's base centre lies inside the tote, on or near its floor (tote frame)."""
    t = kit._pose_T(objects, "tote")
    b = mk._inv(t) @ np.r_[kit.point(objects, "bottle", (0.0, 0.0, 0.0)), 1.0]
    return (abs(b[0]) < TOTE[0] / 2 - WALL and abs(b[1]) < TOTE[1] / 2 - WALL
            and -0.005 < b[2] < FLOOR + 0.015)


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _released(st, margin=CLEAR) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[coast] step {mk._EP.get('step')}: {msg}", flush=True)


def _watch(objects) -> None:
    """Every control step: latch a fallen bottle, a bottle taken out of the tote, a tote lifted off the rollers;
    track the tote's speed and whether everything is still."""
    m = _mem()
    m["n"] += 1
    r = rear(objects)
    p = np.r_[kit.point(objects, "tote", (0, 0, 0)), kit.point(objects, "bottle", (0, 0, BOTTLE[2]))]
    prev = m["prev"]
    if prev is not None:
        m["v"] = float(p[0] - prev[0]) / DT
        m["still"] = bool(np.all(np.abs(p - prev) < STILL_M))
    m["prev"] = p
    m["vmax"] = max(m["vmax"], m["v"])
    m["rear_max"] = max(m["rear_max"], r)
    if m["v_stop"] is None and r > REAR_MAX - 0.003:
        m["v_stop"] = m["v"]  # speed as the tote meets the end stop
        _say(f"tote reached the end stop at {m['v'] * 1000:.0f} mm/s")
    tilt = _tilt_deg(objects, "bottle")
    if not m["tipped"] and tilt > TIP_DEG:
        m["tipped"] = True
        _say(f"BOTTLE FELL ({tilt:.1f} deg) with the tote rear at {r:.3f}")
    if not m["out"] and not bottle_in_tote(objects):
        m["out"] = True
        _say("bottle out of the tote")
    lift = kit.point(objects, "tote", (0, 0, 0))[2] - (BED_Z + 0.001)
    if not m["lifted"] and (_tilt_deg(objects, "tote") > TOTE_TILT_DEG or lift > LIFT):
        m["lifted"] = True
        _say(f"tote lifted / tipped off the rollers (rise {lift * 1000:.1f} mm)")
    if DEBUG and m["n"] % 5 == 0:
        tc = mk.tcps()
        tz = f"({tc[1][0]:.3f},{tc[1][1]:.3f},{tc[1][2] - TOP:.3f})" if tc else "-"
        _say(f"R tcp {tz}; tote rear {r:.4f} v {m['v'] * 1000:.0f} mm/s (max {m['vmax'] * 1000:.0f}); bottle tilt "
             f"{tilt:.1f} deg; tote rise {lift * 1000:.1f} mm, tilt {_tilt_deg(objects, 'tote'):.1f} deg")


kit.on_step(_watch)


def score(objects) -> float:
    """0 once the bottle has fallen (leaned past 15 deg), left the tote, or the tote was lifted or tipped off the
    rollers (latched for the trial); 0 while the tote or bottle is moving or a gripper is within 3 cm of either.
    Otherwise, by where the tote stopped: rear face past the station line -> 1.0; within 6 cm short of it -> 0.5;
    within 16 cm short -> 0.25; else 0."""
    if mk._EP.get("env") is None and _mem().get("seen") is not objects:  # offline (once per state); in sim the
        _watch(objects)  # per-step watch has already run
        _mem()["seen"] = objects
    m = _mem()
    if m["tipped"] or m["out"] or m["lifted"]:
        return 0.0
    if not m["still"] or not _released(objects["tote"]) or not _released(objects["bottle"]):
        return 0.0
    r = rear(objects)
    if r >= ZONE_REAR:
        return 1.0
    out = 0.0
    for line, s in BANDS:
        if r >= line:
            out = s
    return out


def success(objects) -> bool:
    """The tote stopped at the packing station with the bottle standing, released; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """How far the tote's rear is short of the station line (m), plus 1 once the bottle has fallen or the tote or
    bottle was lifted out."""
    m = _mem()
    return float(max(0.0, ZONE_REAR - rear(objects)) + (m["tipped"] or m["out"] or m["lifted"]))


# -- the scripted demo ----------------------------------------------------------------------------------------------
# Open loop from the scene's geometry: the right hand, fingers closed and turned so the closed pads' flat face
# points forward, comes down behind the tote's rear wall at its mid-height, eases onto it, then pushes it along the
# lane with a steady 1 m/s^2 up to PUSH_V and stops hard (4 m/s^2). The tote leaves the pads as the hand slows and
# rolls on alone; the roller bed's resistance (about 0.2 m/s^2) brings it to rest at the station. The push is set by
# one command per control step (exact positions at the step times), so the hand's speed profile is the one planned.
# Release-speed window from the scene (model below): about 0.45-0.60 m/s. Sim run 1: the tote left the pads at
# 0.498 m/s, slowed at 0.196 m/s^2 and met the stop at 0.14 m/s; the bottle stood.

PUSH_V = {"soft": 0.30, "hard": 0.75}.get(VARIANT, 0.50)  # m/s at the end of the push
PUSH_A, STOP_A = (2.5 if VARIANT == "hard" else 1.0), 4.0  # m/s^2 speeding up, slowing down
PAD_HALF = 0.0064  # closed pads: 12.8 mm thick along x, centred on the TCP
YAW = math.pi / 2  # fingers close along x: the closed pads present their 26 mm flat face forward
TCP_Z = BED_Z + FLOOR + 0.030 + 0.005  # pads' mid-height at the rear wall's mid-height (TCP 5 mm above it)
GAP = 0.006  # the hand waits this far behind the wall
PARK = (0.200, -0.330, TOP + 0.220)
V_TIP = 0.40  # m/s at the end stop above which the bottle falls (tip-over model; sim: stood at 0.14, fell at 0.42)


def push_profile(v=None, a=PUSH_A, d=STOP_A):
    """The push's TCP x at each control step: ease on, speed up at ``a`` to ``v``, stop at ``d``."""
    v = PUSH_V if v is None else v
    x0 = X_T0 - TOTE[0] / 2 - PAD_HALF  # pads' face on the wall
    xs, x, s = [], x0, 0.0
    while s < v - 1e-9:
        s = min(v, s + a * DT)
        x += s * DT
        xs.append(x)
    while s > 1e-9:
        s = max(0.0, s - d * DT)
        x += s * DT
        xs.append(x)
    return x0, xs


def demo():
    """Come down behind the tote; ease onto its rear wall; push to PUSH_V and stop; lift away; park."""
    x0, xs = push_profile()
    R = [Waypoint(2.0, PARK, FINGER_CLOSED, "start", YAW),
         Waypoint(2.0, (x0 - GAP - 0.02, LY, TCP_Z + 0.08), FINGER_CLOSED, "over the lane, behind the tote", YAW),
         Waypoint(1.5, (x0 - GAP, LY, TCP_Z), FINGER_CLOSED, "down behind the rear wall", YAW),
         Waypoint(0.5, (x0 - GAP, LY, TCP_Z), FINGER_CLOSED, "settle", YAW),
         Waypoint(0.6, (x0 + 0.0005, LY, TCP_Z), FINGER_CLOSED, "ease onto the wall", YAW)]
    R += [Waypoint(DT, (x, LY, TCP_Z), FINGER_CLOSED, "push", YAW) for x in xs]
    xe = xs[-1]
    R += [Waypoint(0.8, (xe, LY, TCP_Z), FINGER_CLOSED, "hold still: the tote rolls on", YAW),
          Waypoint(1.2, (xe - 0.01, LY, TCP_Z + 0.10), FINGER_CLOSED, "lift away", YAW),
          Waypoint(2.0, PARK, FINGER_CLOSED, "park", YAW),
          Waypoint(4.0, PARK, FINGER_CLOSED, "wait: the tote rolls to rest", YAW)]
    total = sum(w.duration for w in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), FINGER_CLOSED, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), FINGER_CLOSED, "park")]
    return {"left": left, "right": R}


def coast(v0, rear0, a=MU_ROLL * 9.81):
    """Model: the tote leaves the hand at ``v0`` with its rear at ``rear0`` and rolls on, slowed at ``a``. Returns
    the rear face's x per control step and the speed at the end stop (None if it stops short)."""
    xs, x, v = [], rear0, v0
    while v > 0:
        v2 = max(0.0, v - a * DT)
        x += 0.5 * (v + v2) * DT
        v = v2
        if x >= REAR_MAX:
            vi = math.sqrt(max(0.0, v0 * v0 - 2 * a * (REAR_MAX - rear0)))
            xs.append(REAR_MAX)
            return xs, vi
        xs.append(x)
    return xs, None


def window(rear0, a=MU_ROLL * 9.81, v_tip=V_TIP):
    """Release speeds (m/s) that bring the tote to rest at the station with the bottle standing (model)."""
    return math.sqrt(2 * a * (ZONE_REAR - rear0)), math.sqrt(v_tip ** 2 + 2 * a * (REAR_MAX - rear0))


# -- offline self-test ------------------------------------------------------------------------------------------------


def self_test():
    """The scoring on hand-built trials, one control step at a time: the hand pushes the tote (rear face at the
    pads), the tote then rolls on by the model above; a tote meeting the end stop faster than V_TIP throws the
    bottle over against the front wall."""
    global _TEST_TCPS
    far = [(0.15, 0.25, 0.95), PARK]
    lean = (math.sin(math.radians(17.5)), 0.0, 0.0, math.cos(math.radians(17.5)))  # 35 deg forward about y

    def state(r, tipped=False, bottle_dz=0.0, tote_dz=0.0, bottle_dx=0.0):
        tc = r + TOTE[0] / 2
        moved = {"tote": ((tc, LY, BED_Z + 0.001 + tote_dz), (0, 0, 0, 1.0))}
        bz = BED_Z + 0.001 + FLOOR + bottle_dz
        moved["bottle"] = ((tc + bottle_dx + (0.03 if tipped else 0.0), LY, bz), lean if tipped else (0, 0, 0, 1.0))
        return kit.fake_states(moved=moved)

    def run(rears, hand=None, tip_at=None, end_hand=None, bottle=None, tote_dz=None, settle=30, down=None):
        """Feed the tote's rear x per step (then ``settle`` still steps). ``hand``: steps with a gripper on the tote;
        ``tip_at``: first step with the bottle fallen (``down``: or step -> fallen); ``bottle``/``tote_dz``: step ->
        (dz, dx) / dz."""
        global _TEST_TCPS
        rears = list(rears) + [rears[-1]] * settle
        got = st = None
        for k, r in enumerate(rears):
            bz, bx = bottle(k) if bottle else (0.0, 0.0)
            fell = down(k) if down else (tip_at is not None and k >= tip_at)
            st = state(r, tipped=fell, bottle_dz=bz, bottle_dx=bx,
                       tote_dz=tote_dz(k) if tote_dz else 0.0)
            on = hand is not None and k in hand
            _TEST_TCPS = [far[0], (r - PAD_HALF, LY, TCP_Z)] if on else list(far)
            if k == len(rears) - 1 and end_hand is not None:
                _TEST_TCPS = list(end_hand)
            got = score(st)
        return got, st

    def trial(v, a_push=PUSH_A, **kw):
        """A push like the demo's to ``v``, then the coast; the bottle falls if the tote meets the stop too fast."""
        x0, xs = push_profile(v, a_push)
        pr = [x + PAD_HALF for x in xs]
        k_sep = int(np.argmax(np.diff([x0] + xs)))  # the hand slows from here on: the tote rolls on alone
        r0 = pr[k_sep]
        cx, vi = coast(v, r0)
        rears = [X_T0 - TOTE[0] / 2] * 10 + pr[:k_sep + 1] + cx
        tip = (10 + k_sep + 1 + len(cx)) if (vi is not None and vi > V_TIP) else None
        hand = set(range(10, 10 + k_sep + 1))
        return run(rears, hand=hand, tip_at=tip, **kw), vi, rears[-1]

    lo, hi = window(X_T0 - TOTE[0] / 2 + 0.11)
    cases = {}

    def case(name, fn, want):
        mk.new_episode()
        out = fn()
        (got, st), extra = (out[0], out[1:]) if isinstance(out[0], tuple) else (out, ())
        cases[name] = (got, rear(st), extra)
        assert abs(got - want) < 1e-9, (name, got, want, round(rear(st), 4), _mem())
        assert success(st) == (want >= 1.0), name

    r_start = X_T0 - TOTE[0] / 2
    case("start", lambda: run([r_start], settle=0), 0.0)
    case(f"demo push to {PUSH_V} m/s: rolls to rest at the station", lambda: trial(PUSH_V), 1.0)
    case("slow end of the window (0.47 m/s)", lambda: trial(0.47), 1.0)
    case("fast end of the window (0.58 m/s): meets the stop gently, bottle stands", lambda: trial(0.58), 1.0)
    case("too soft (0.30 m/s): stops in the lane", lambda: trial(0.30), 0.0)
    case("a bit soft (0.42 m/s): 0.25", lambda: trial(0.42), 0.25)
    case("nearly (0.43 m/s): within 5 cm, 0.5", lambda: trial(0.43), 0.5)
    case("shoved (0.85 m/s): slams the stop, bottle falls", lambda: trial(0.85), 0.0)
    case("shoved hard from a standing start (0.70 m/s)", lambda: trial(0.70, a_push=2.0), 0.0)
    case("pushed with the hand pointing down as far as it reaches (rear to 0.34), let go",
         lambda: run(list(np.linspace(r_start, 0.34, 60)), hand=set(range(60))), 0.0)
    case("walked by hand, pointing forward, to its reach limit (rear 0.76), let go",
         lambda: run(list(np.linspace(r_start, 0.76, 120)), hand=set(range(120))), 0.5)
    case("walked by hand to the reach limit and held there",
         lambda: run(list(np.linspace(r_start, 0.76, 120)), hand=set(range(150)),
                     end_hand=[far[0], (0.76 - PAD_HALF, LY, TCP_Z)]), 0.0)
    case("tote lifted and carried to the station",
         lambda: run(list(np.linspace(r_start, 0.85, 80)), tote_dz=lambda k: 0.05 if 10 < k < 70 else 0.0), 0.0)
    case("bottle lifted out and set back in the tote, then the tote sent to the station",
         lambda: run(list(np.linspace(r_start, 0.85, 80)), bottle=lambda k: (0.08, 0.0) if 5 < k < 20 else (0, 0)),
         0.0)
    case("bottle knocked over at the start, set upright again, then the tote sent to the station",
         lambda: run([r_start] * 40 + list(np.linspace(r_start, 0.85, 40)), down=lambda k: 10 <= k < 20), 0.0)
    case("demo, but a gripper is left near the tote", lambda: trial(PUSH_V, end_hand=[far[0], (0.87, LY, TCP_Z)]), 0.0)
    case("tote still rolling at the last step", lambda: run(list(np.linspace(r_start, 0.85, 40)), settle=0), 0.0)
    mk.new_episode()
    assert score(state(r_start)) == 0.0 and not _mem()["tipped"], "a new trial forgets the last one"
    mk.new_episode()
    _TEST_TCPS = None
    return len(cases) + 1, cases, (lo, hi)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring on hand-built trials, then both arms' plans replayed through the kinematics
    # at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n, res, (lo, hi) = self_test()
    for k, (s, r, extra) in res.items():
        print(f"  {s:5.2f}  tote rear stopped at x {r:.3f}  | {k}")
    print(f"scoring: {n} cases pass; release-speed window {lo:.3f}-{hi:.3f} m/s, demo {PUSH_V} m/s")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
