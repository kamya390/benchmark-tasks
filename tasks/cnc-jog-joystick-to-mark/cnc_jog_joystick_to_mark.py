"""On the station table, a desktop CNC router stands in front of the left arm and its jog pendant in front of the
right arm. The router's gantry runs forward and back (x) on two rails over an MDF spoilboard; a plywood workpiece
is fixed on the bed with a scribed cross mark on it, 12 cm ahead of where the cutter stands. The cutter (a 6 mm
end mill in the spindle's collet) hangs 10 mm above the work. A steel toe clamp holds the work down; its back
face is 10 mm beyond the mark, in the gantry's path, and stands 10 mm higher than the cutter's tip.

The gantry is servo-held: its axis motor holds it where the controller puts it (about 3 N per mm), so pushing the
gantry by hand does not move it. The only way to move it is the pendant's jog lever, a spring-centred
proportional stick on a pivot. Tilting the stick forward jogs the gantry forward, back jogs it back; the speed
follows the tilt (nothing within 2 deg of centre, full jog speed, 40 mm/s, from 14 deg; the stick stops at 20
deg). The controller ramps the axis speed at 80 mm/s^2, as jog controllers do, so the gantry runs on about 10 mm
after the stick is let go at full speed. Run the cutter into the clamp and the job is crashed.

The job: jog the cutter over the cross mark and let the stick go. A careful operator eases the stick back before
the mark; one who holds full speed until the cutter is over the mark runs on into the clamp.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/cnc-jog-joystick-to-mark/cnc_jog_joystick_to_mark.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinge, slider, bond, per-step rules, per-episode memory)

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the controller's readout every 5 steps
VARIANT = "demo"  # sim smoke test: "naive" holds full speed until the cutter is over the mark, then lets go

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the jog pendant: a console with a spring-centred stick on a pivot (axis along y) -------------------------------
JX, JY = 0.300, -0.200  # console centre
CONSOLE = (0.100, 0.100, 0.040)
PZ = TOP + CONSOLE[2] + 0.006  # pivot height
R_KNOB = 0.090  # knob centre from the pivot
KNOB = 0.030  # knob cube
STICK_KG = 0.035
K_STICK = 0.25  # N*m/rad centring spring
F_STICK = 0.003  # N*m pivot friction (the stick re-centres within about 0.7 deg)
D_STICK = 0.004  # N*m*s/rad
LIM_DEG = 20.0  # the stick's stops
# -- the jog controller (a software rule, as on a real machine) ----------------------------------------------------
DEAD_DEG, FULL_DEG = 2.0, 14.0  # no motion within 2 deg of centre; full speed from 14 deg
VMAX = 0.040  # m/s full jog speed
ACC = 0.080  # m/s^2 jog acceleration and deceleration
DT = 0.05  # control step
K_AX, D_AX = 3000.0, 80.0  # axis servo: N/m and N*s/m about the controller's position and speed
# -- the router -------------------------------------------------------------------------------------------------
BX, BY = 0.385, 0.160  # bed centre
BED = (0.270, 0.200, 0.015)  # MDF spoilboard
RAIL = (0.270, 0.020, 0.040)
RAIL_Y = 0.115
WORK_O = (0.400, BY)  # plywood workpiece centre
WORK = (0.140, 0.100, 0.012)
WZ = TOP + BED[2] + WORK[2]  # work top
MARK = (0.420, BY)  # the cross mark
TRAVEL = 0.120  # the mark is this far ahead of the cutter at the start
GX0 = MARK[0] - TRAVEL  # gantry (cutter axis) x at the start
TIP_UP = 0.010  # cutter tip above the work
TOOL = 0.006  # cutter diameter (modelled square)
GANTRY_KG = 0.60
SOFT_LO, SOFT_HI = -0.025, TRAVEL + 0.022  # controller's soft limits (axis travel from the start)
CLAMP = (0.030, 0.024, 0.020)  # steel toe clamp on the work
CL_GAP = 0.010  # clamp's back face this far beyond the mark
CLAMP_KG = 0.06
CLAMP_BREAK = 4.0  # N: the clamp's grip on the work gives way (it is knocked loose)
TIP_Z = WZ + TIP_UP

WOOD, MDF, STEEL, ALU = (0.80, 0.66, 0.45), (0.62, 0.52, 0.40), (0.35, 0.36, 0.38), (0.72, 0.74, 0.76)
DARK, BLACK, ORANGE, INK = (0.18, 0.19, 0.21), (0.08, 0.08, 0.09), (0.92, 0.45, 0.10), (0.05, 0.05, 0.08)

kit = mk.Kit()
kit.part("console", (JX, JY, TOP), [
    mk.box(CONSOLE, at=(0, 0, CONSOLE[2] / 2)),
    mk.box((0.024, 0.024, 0.006), at=(0, 0, CONSOLE[2] + 0.003), color=BLACK),  # boot round the stick
], color=DARK)
kit.part("stick", (JX, JY, PZ), [
    mk.box((0.010, 0.010, R_KNOB - KNOB / 2 + 0.006), at=(0, 0, (R_KNOB - KNOB / 2 - 0.006) / 2), color=BLACK),
    mk.box((KNOB, KNOB, KNOB), at=(0, 0, R_KNOB), color=ORANGE),
], mass=STICK_KG)
kit.hinge("lever", "console", "stick", anchor=(JX, JY, PZ), axis=(0, 1, 0), lower_deg=-LIM_DEG, upper_deg=LIM_DEG,
          start_deg=0.0, stiffness=K_STICK, rest_deg=0.0, damping=D_STICK, friction=F_STICK)


def _b(x, y, z):  # world -> bed frame
    return (x - BX, y - BY, z - TOP)


kit.part("bed", (BX, BY, TOP), [
    mk.box(BED, at=(0, 0, BED[2] / 2), color=MDF),
    mk.box(RAIL, at=(0, RAIL_Y, RAIL[2] / 2)),
    mk.box(RAIL, at=(0, -RAIL_Y, RAIL[2] / 2)),
], color=ALU)
kit.part("work", (*WORK_O, TOP + BED[2]), [
    mk.box(WORK, at=(0, 0, WORK[2] / 2)),
    mk.box((0.016, 0.0012, 0.0004), at=(MARK[0] - WORK_O[0], MARK[1] - WORK_O[1], WORK[2] + 0.0002), color=INK),
    mk.box((0.0012, 0.016, 0.0004), at=(MARK[0] - WORK_O[0], MARK[1] - WORK_O[1], WORK[2] + 0.0002), color=INK),
], color=WOOD)
CLAMP_X = MARK[0] + CL_GAP + CLAMP[0] / 2
kit.part("clamp", (CLAMP_X, MARK[1], WZ), [
    mk.box(CLAMP, at=(0, 0, CLAMP[2] / 2)),
    mk.box((0.010, 0.010, 0.004), at=(0.004, 0, CLAMP[2] + 0.002), color=DARK),  # T-bolt head
], mass=CLAMP_KG, color=STEEL)
kit.bond("clamp_hold", None, "clamp", break_force=CLAMP_BREAK)

# gantry, in its own frame (origin on the cutter axis at table height): uprights over the rails, beam, spindle,
# collet nut and cutter
_up_z0 = RAIL[2] + 0.002
_tip = TIP_Z - TOP
_nut_z0 = _tip + 0.017
_sp_z0 = _nut_z0 + 0.008
_beam_z0 = 0.142
kit.part("gantry", (GX0, BY, TOP), [
    mk.box((0.040, 0.020, _beam_z0 - _up_z0), at=(0, RAIL_Y, (_up_z0 + _beam_z0) / 2)),
    mk.box((0.040, 0.020, _beam_z0 - _up_z0), at=(0, -RAIL_Y, (_up_z0 + _beam_z0) / 2)),
    mk.box((0.040, 2 * RAIL_Y + 0.020, 0.030), at=(0, 0, _beam_z0 + 0.015)),
    mk.box((0.050, 0.050, _beam_z0 - _sp_z0), at=(0, 0, (_sp_z0 + _beam_z0) / 2), color=DARK),  # spindle motor
    mk.box((0.020, 0.020, 0.008), at=(0, 0, _nut_z0 + 0.004), color=STEEL),  # collet nut
    mk.box((TOOL, TOOL, 0.017), at=(0, 0, _tip + 0.0085), color=(0.80, 0.80, 0.82)),  # end mill
], mass=GANTRY_KG, color=ALU)
kit.slider("x_axis", None, "gantry", axis=(1, 0, 0), lower=SOFT_LO - 0.003, upper=SOFT_HI + 0.003, start=0.0,
           stiffness=K_AX, rest=0.0, damping=D_AX)

OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the controller's per-step rule (sim) and the scoring -----------------------------------------------------------
NEAR_MM = (10.0, 4.0, 2.0)  # 0.25 / 0.5 / 0.75 bands for the cutter's distance from the mark
CENTRED_DEG = DEAD_DEG  # the stick let go: back within the dead band
CLEAR = 0.03  # both grippers this far from the stick and the gantry
TOUCH = 0.0006  # the cutter within this of the clamp's face: a crash
STILL_M = 0.00025  # gantry still: the cutter moved under 0.25 mm in a control step (5 mm/s)
M = "cncj"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"n": 0, "init": False, "v": 0.0, "tgt": 0.0, "crash": False, "gap_min": 1.0,
                            "prev_tip": None, "prev_deg": None, "still": True, "stick_still": True}
    return m


def stick_deg(objects) -> float:
    """The jog stick's tilt (deg; + = forward)."""
    return kit.q_deg(objects, "lever")


def jog_speed(deg: float) -> float:
    """The controller's commanded axis speed (m/s) for a stick tilt (deg)."""
    s = min(1.0, max(0.0, (abs(deg) - DEAD_DEG) / (FULL_DEG - DEAD_DEG)))
    return math.copysign(VMAX * s, deg)


def tip(objects) -> np.ndarray:
    """The cutter tip's centre (world)."""
    return kit.point(objects, "gantry", (0.0, 0.0, _tip))


def mark_mm(objects) -> float:
    """The cutter tip's distance from the cross mark in plan (mm)."""
    t = tip(objects)
    return float(math.hypot(t[0] - MARK[0], t[1] - MARK[1])) * 1000.0


def clamp_gap(objects) -> float:
    """Clear gap (m) between the cutter's front face and the clamp's back face, measured in the clamp's frame."""
    t = mk._inv(kit._pose_T(objects, "clamp")) @ np.r_[tip(objects), 1.0]
    over = abs(t[1]) < CLAMP[1] / 2 + TOOL / 2 and t[2] < CLAMP[2] + 0.004
    return float((-CLAMP[0] / 2) - (t[0] + TOOL / 2)) if over else 1.0


def _set_axis(target: float, speed: float) -> None:
    """Sim only: the axis servo's position (m, travel from the start) and speed (m/s) set points."""
    path = mk._SIM["joint_paths"].get((kit.id, "x_axis"))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear")
    d.GetTargetPositionAttr().Set(float(target))
    d.GetTargetVelocityAttr().Set(float(speed))


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit cncj] step {mk._EP.get('step')}: {msg}", flush=True)


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _released(st, margin=CLEAR) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def _rules(objects) -> None:
    """One control step of the jog controller (stick tilt -> axis speed, ramped; position integrated within the
    soft limits) and of the crash watch."""
    m = _mem()
    m["n"] += 1
    if not m["init"]:
        m["init"] = True
        _set_axis(0.0, 0.0)
    a = stick_deg(objects)
    v_cmd = jog_speed(a)
    v0 = m["v"]
    v = v0 + min(ACC * DT, max(-ACC * DT, v_cmd - v0))
    tgt = m["tgt"] + 0.5 * (v0 + v) * DT
    if tgt >= SOFT_HI or tgt <= SOFT_LO:
        tgt, v = min(SOFT_HI, max(SOFT_LO, tgt)), 0.0
    m["v"], m["tgt"] = v, tgt
    _set_axis(tgt, v)
    t = tip(objects)
    p = m["prev_tip"]
    m["still"] = p is None or float(np.linalg.norm(t - p)) < STILL_M
    m["prev_tip"] = t
    m["stick_still"] = m["prev_deg"] is None or abs(a - m["prev_deg"]) < 0.5
    m["prev_deg"] = a
    g = clamp_gap(objects)
    m["gap_min"] = min(m["gap_min"], g)
    if not m["crash"] and (g < TOUCH or kit.broken(objects, "clamp_hold")):
        m["crash"] = True
        _say(f"CRASH: cutter gap to clamp {g * 1000:.2f} mm, clamp loose {kit.broken(objects, 'clamp_hold')}")
    if DEBUG and m["n"] % 5 == 0:
        tc = mk.tcps()
        tz = f"({tc[1][0]:.3f},{tc[1][1]:.3f},{tc[1][2] - TOP:.3f})" if tc else "-"
        _say(f"R tcp {tz}; stick {a:.2f} deg, v_cmd {v_cmd * 1000:.1f}, v {v * 1000:.1f} mm/s, target "
             f"{tgt * 1000:.2f} mm, tip x {(t[0] - GX0) * 1000:.2f} mm (y {(t[1] - BY) * 1000:.2f}, z "
             f"{(t[2] - TIP_Z) * 1000:.2f}), to mark {mark_mm(objects):.2f} mm, clamp gap {g * 1000:.1f} mm")


kit.on_step(_rules)


def score(objects) -> float:
    """0 once the cutter has touched the clamp or the clamp has been knocked loose (latched for the trial), while
    the gantry is still moving, or while a gripper is on the gantry. Otherwise, by the cutter tip's distance from the
    mark in plan: 0.25 within 10 mm, 0.5 within 4 mm, 0.75 within 2 mm; 1.0 within 2 mm with the stick let go
    (back within 2 deg of centre, both grippers 3 cm clear of it) and still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _rules(objects)
    m = _mem()
    if m["crash"] or not m["still"] or not _released(objects["gantry"]):
        return 0.0
    d = mark_mm(objects)
    base = 0.0
    for b, lim in zip((0.25, 0.5, 0.75), NEAR_MM):
        if d <= lim:
            base = b
    if base == 0.75 and abs(stick_deg(objects)) < CENTRED_DEG and _released(objects["stick"]) and m["stick_still"]:
        base = 1.0
    return base


def success(objects) -> bool:
    """The cutter within 2 mm of the mark, stopped, stick let go and centred, no crash; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """The cutter's distance from the mark (m), plus 1 for a crash and 0.5 while the stick is held off centre."""
    m = _mem()
    return float(mark_mm(objects) / 1000.0 + m["crash"] + 0.5 * (abs(stick_deg(objects)) >= CENTRED_DEG))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry and the controller's jog rate: the right hand, fingers closed and turned to
# push with the flat of the pads, comes down behind the stick's knob, tilts the stick forward to its stop for a
# fixed time (full jog speed), then eases it back to centre well before the mark (the gantry ramps down over the
# last ~10 mm), lifts away and parks. The left arm stays parked. The arm is compliant: the knob lags the hand by
# about a quarter of the commanded tilt (the LAG model reproduced sim run 1 within 0.1 mm; see HOLD_S).

PUSH_DEG = 26.0  # commanded tilt (the knob lags the hand; the stick ends on or near its 20 deg stop)
RAMP_S, EASE_S = 0.4, 0.5
HOLD_S = 2.57  # full-speed time (sim run 1: 2.12 s stopped 18.0 mm short, as the lag model below predicts)
BACK = 0.010  # the hand waits this far behind the knob
TCP_UP = 0.005  # TCP above the knob's centre (the pads' mid-height is 5 mm under the TCP)
PARK = (0.200, -0.300, TOP + 0.200)
YAW = math.pi / 2  # fingers close along x: the closed pads present their 26 mm-wide flat face forward
PAD_HALF = 0.0064  # closed pads: 12.8 mm thick along x, centred on the TCP
LAG = 0.76  # model of the arm's give: stick tilt = LAG x the tilt the hand's position asks for (offline estimate)


def hand_x(deg: float) -> float:
    """TCP x that puts the closed pads' front face on the knob's back face with the stick tilted ``deg``."""
    th = math.radians(deg)
    kc = np.array([JX + R_KNOB * math.sin(th), PZ + R_KNOB * math.cos(th)])  # knob centre (x, z)
    ax = np.array([math.cos(th), -math.sin(th)])  # knob's local x in (x, z)
    up = np.array([math.sin(th), math.cos(th)])  # knob's local z
    fc = kc - KNOB / 2 * ax  # back-face centre
    zf = PZ + R_KNOB  # contact height (the pads' mid-height)
    s = (zf - fc[1]) / up[1]
    return float(fc[0] + s * up[0] - PAD_HALF)


def _deg_at(x: float) -> float:
    """Inverse of hand_x: the stick tilt (deg) at which the knob meets pads with their TCP at ``x`` (0 if clear)."""
    if x <= hand_x(0.0):
        return 0.0
    lo, hi = 0.0, 40.0
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if hand_x(mid) < x else (lo, mid)
    return lo


def demo():
    """Come down behind the knob; tilt the stick to its stop for HOLD_S; ease it back to centre; lift off; park."""
    tz = PZ + R_KNOB + TCP_UP
    x0 = hand_x(0.0) - BACK
    R = [Waypoint(2.0, PARK, FINGER_CLOSED, "start", YAW),
         Waypoint(2.0, (x0, JY, tz + 0.060), FINGER_CLOSED, "over the pendant, behind the knob", YAW),
         Waypoint(1.5, (x0, JY, tz), FINGER_CLOSED, "down behind the knob", YAW),
         Waypoint(0.5, (x0, JY, tz), FINGER_CLOSED, "settle", YAW),
         Waypoint(RAMP_S, (hand_x(PUSH_DEG), JY, tz), FINGER_CLOSED, "tilt the stick forward", YAW),
         Waypoint(HOLD_S + (0.25 if VARIANT == "naive" else 0.0), (hand_x(PUSH_DEG), JY, tz), FINGER_CLOSED,
                  "hold: full jog speed", YAW),
         Waypoint(EASE_S, (x0, JY, tz), FINGER_CLOSED, "ease the stick back to centre", YAW),
         Waypoint(1.0, (x0, JY, tz), FINGER_CLOSED, "wait: the gantry ramps down", YAW),
         Waypoint(1.0, (x0 - 0.02, JY, tz + 0.060), FINGER_CLOSED, "lift away", YAW),
         Waypoint(2.0, PARK, FINGER_CLOSED, "park", YAW),
         Waypoint(3.0, PARK, FINGER_CLOSED, "wait", YAW)]
    total = sum(x.duration for x in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), FINGER_CLOSED, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), FINGER_CLOSED, "park")]
    return {"left": left, "right": R}


def demo_tilts(lag: float = LAG) -> list:
    """The stick tilt (deg) at each control step of the demo, from the right hand's planned x and the lag model."""
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    plan = ArmPlan(list(demo()["right"]), station.BASES["right"])
    tz = PZ + R_KNOB + TCP_UP
    out = []
    for k in range(round(plan.duration / DT) + 5):
        p = plan.target((k + 1) * DT)[0]
        at_knob = abs(p[2] - tz) < 0.010 and abs(p[1] - JY) < 0.020  # the pads at the knob's height, in line
        out.append(min(LIM_DEG, lag * _deg_at(float(p[0]))) if at_knob else 0.0)
    return out


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The controller and the scoring on hand-built sequences of stick tilts, one control step at a time. The gantry
    sits where the controller puts it (a stiff servo), unless the case moves it; the clamp stays bolted unless the
    case knocks it."""
    global _TEST_TCPS
    knob = (JX, JY, PZ + R_KNOB + TCP_UP)
    far = [(0.15, 0.25, 0.95), (0.20, -0.30, TOP + 0.20)]

    def run(tilts, hand_end=None, push=None, clamp=None, extra_steps=0):
        """Feed ``tilts`` (deg per step; the hand is on the knob while the tilt is off centre). ``push``: a function
        step -> extra gantry travel (m) forced by hand; ``clamp``: step -> clamp offset (m) or None."""
        global _TEST_TCPS
        tilts = list(tilts) + [0.0] * extra_steps
        got = st = None
        for k, a in enumerate(tilts):
            m = _mem()
            d = m["tgt"] + (push(k) if push else 0.0)
            moved = {}
            if clamp and clamp(k) is not None:
                moved["clamp"] = ((CLAMP_X + clamp(k), MARK[1], WZ), (0, 0, 0, 1.0))
            if d > SOFT_HI + 0.003:
                d = SOFT_HI + 0.003
            gap_now = clamp_gap(kit.fake_states(q={"x_axis": d}, moved=moved))
            if gap_now < 0 and not moved:  # the cutter meets the clamp: physics stops it at the face
                d += gap_now
            st = kit.fake_states(q={"lever": math.radians(a), "x_axis": d}, moved=moved)
            last = k == len(tilts) - 1
            if last and hand_end is not None:
                _TEST_TCPS = list(hand_end)
            elif abs(a) > 0.3:
                _TEST_TCPS = [far[0], knob]
            else:
                _TEST_TCPS = list(far)
            got = score(st)
        return got, st

    def hold(deg, s):
        return [deg] * round(s / DT)

    def ramp(a, b, s):
        n = max(1, round(s / DT))
        return [a + (b - a) * k / n for k in range(1, n + 1)]

    def let_go(a):  # the stick springs back to centre in about 0.1 s
        return [a * 0.4, a * 0.05, 0.0]

    def full_until(mm_short, latency_s=0.0):
        """Full tilt until the controller's target is ``mm_short`` before the mark (plus a reaction latency), then
        let go: the naive policy. Returns the tilt list."""
        mk.new_episode()
        out = []
        for _ in range(400):
            if _mem()["tgt"] >= TRAVEL - mm_short / 1000.0:
                break
            out.append(20.0)
            run([20.0])
        lat = hold(20.0, latency_s)
        mk.new_episode()
        return out + lat + let_go(20.0)

    def operator(pre=(), fast_short_mm=25.0, nudge_deg=5.0, nudge_s=0.3, done_mm=1.0, nudges=30):
        """A careful operator, closed loop on where the cutter is: (``pre`` tilts first), full speed until the
        cutter is ``fast_short_mm`` short of the mark, let go and wait for the gantry to stop, then short gentle
        nudges until it is within ``done_mm``. Returns the tilt list."""
        mk.new_episode()
        out = list(pre)
        run(out) if out else None

        def go(seq):
            out.extend(seq)
            run(seq)

        def left():
            return (TRAVEL - _mem()["tgt"]) * 1000.0

        while left() > fast_short_mm and len(out) < 600:
            go([20.0])
        go(let_go(20.0))
        while abs(_mem()["v"]) > 0 and len(out) < 900:
            go([0.0])
        for _ in range(nudges):
            if left() <= done_mm:
                break
            go(ramp(0, nudge_deg, 0.1) + hold(nudge_deg, nudge_s) + ramp(nudge_deg, 0, 0.1))
            while abs(_mem()["v"]) > 0 and len(out) < 2000:
                go([0.0])
        mk.new_episode()
        return out

    settle = 30  # 1.5 s still at the end
    demo_t = demo_tilts()
    cases = {  # name: (tilts, kwargs, score)
        "start": ([0.0], {}, 0.0),
        "demo (the planned tilts with the arm's give)": (demo_t, {"extra_steps": settle}, 1.0),
        "naive: full speed until the cutter is over the mark, then let go: runs on into the clamp":
            (full_until(0.0), {"extra_steps": settle}, 0.0),
        "naive with a 0.3 s reaction time": (full_until(0.0, 0.3), {"extra_steps": settle}, 0.0),
        "naive, let go 4 mm before the mark: stops 6 mm past it": (full_until(4.0), {"extra_steps": settle}, 0.25),
        "naive, let go 3 mm before the mark: runs into the clamp": (full_until(3.0), {"extra_steps": settle}, 0.0),
        "careful: full speed to 25 mm short, let go, then short gentle nudges": (operator(), {"extra_steps": settle},
                                                                               1.0),
        "careful, but slower nudges at 3.5 deg": (operator(nudge_deg=3.5, nudge_s=0.5), {"extra_steps": settle}, 1.0),
        "demo tilts, but the hand stays on the knob at the end": (demo_t, {"extra_steps": settle,
                                                                           "hand_end": [far[0], knob]}, 0.75),
        "stopped about 3 mm short of the mark": (operator(done_mm=3.5, nudge_deg=3.5, nudge_s=0.2),
                                                 {"extra_steps": settle}, 0.5),
        "stopped about 14 mm short (eased off early, never finished)": (operator(nudges=0),
                                                                        {"extra_steps": settle}, 0.0),
        "stopped about 7 mm short": (operator(done_mm=10.0), {"extra_steps": settle}, 0.25),
        "overshoot into the clamp, then jogged back onto the mark":
            (full_until(0.0) + [0.0] * 10 + ramp(0, -6.0, 0.2) + hold(-6.0, 1.75) + ramp(-6.0, 0, 0.2),
             {"extra_steps": settle}, 0.0),
        "demo, but the clamp was knocked loose by a gripper (7 mm) on the way":
            (demo_t, {"extra_steps": settle, "clamp": lambda k: 0.007 if k > 20 else None}, 0.0),
        "the gantry held on the mark by hand (gripper on it), stick never used":
            ([0.0] * 30, {"push": lambda k: TRAVEL, "hand_end": [far[0], (MARK[0], MARK[1], TOP + 0.16)]}, 0.0),
        "the cutter still moving at 8 mm/s near the mark at the last step":
            (ramp(0, 20, 0.3) + hold(20.0, 2.0) + ramp(20, 4.5, 0.3) + hold(4.5, 3.0), {}, 0.0),
        "jogged back first (the wrong way), then the careful way":
            (operator(pre=ramp(0, -10, 0.2) + hold(-10, 0.5) + ramp(-10, 0, 0.2) + [0.0] * 20),
             {"extra_steps": settle}, 1.0),
        "stick held tilted 1.5 deg (inside the dead band) by the hand on the mark":
            (demo_t + [1.5] * 30, {"hand_end": [far[0], knob]}, 0.75),
    }
    results = {}
    try:
        for name, (tilts, kw, want) in cases.items():
            mk.new_episode()
            got, st = run(tilts, **kw)
            results[name] = (got, mark_mm(st), _mem()["gap_min"] * 1000)
            if want is not None:
                assert abs(got - want) < 1e-9, (name, got, want, round(mark_mm(st), 2), _mem()["crash"])
                assert success(st) == (want >= 1.0), name
        mk.new_episode()
        assert score(kit.fake_states()) == 0.0 and _mem()["tgt"] == 0.0, "a new trial forgets the last one"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1, results


if __name__ == "__main__":
    # Offline checks, no GPU: the controller and the scoring on hand-built tilt sequences, then both arms' plans
    # replayed through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n, res = self_test()
    for k, (s, d, g) in res.items():
        print(f"  {s:5.2f}  cutter {d:6.2f} mm from the mark, closest to the clamp {g:6.2f} mm  | {k}")
    print(f"scoring: {n} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
