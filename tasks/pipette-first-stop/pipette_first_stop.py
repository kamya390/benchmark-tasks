"""On the station table stand a pipette in its stand, a sample reservoir and a rack with one empty tube. The
pipette is an air-displacement pipette built as a rig for two arms: a grey body with a top grip cap and a
disposable tip, and a plunger whose push button sits on a side arm 15 cm to the right of the body (on a hand-held
pipette the button is on top, under the thumb). The plunger has two stops, as on a real pipette: it travels 12 mm
against a soft spring to the first stop, where its arm meets the sprung blowout collar, and 8 mm more against the
stiff collar spring to the second stop. How far the button has gone down is visible as its height against the
body. The reservoir (blue level band on its inner walls, a sight gauge on its right side) holds the blue sample; the
tube has a white EMPTY tag over a blue band on its front.

Liquid is not simulated. The score reads the plunger travel, the tip's position against the reservoir's level band
and the tube, and the pipette's tilt, every control step. The visible effect is emulated: when a volume is drawn
up, the reservoir gauge's blue level marker drops a step; when a volume is dispensed into the tube, the white EMPTY
tag drops away and shows the blue band behind it.

Forward pipetting (pipette makers' manuals, ISO 8655 practice; not stated in the instruction): press the button to
the first stop before the tip enters the liquid, immerse the tip, let the button up slowly, withdraw; in the tube
press to the first stop, then on to the second stop (blowout), and withdraw the tip before letting the button up.
Pressing to the second stop before aspirating draws too much; pressing while the tip is in the liquid blows
bubbles; pressing a loaded pipette outside the tube spills; laying a loaded pipette over runs liquid into the
shaft; letting the button up in the tube draws the sample back.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/pipette-first-stop/pipette_first_stop.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (per-episode memory, gripper TCPs, sliders and bonds)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the pipette (kit parts; frame origin = the tip's end, body vertical) ----------------------------------------
BODY_KG, PLUNGER_KG, COLLAR_KG = 0.150, 0.008, 0.010
TIP = (0.008, 0.008, 0.012)  # polypropylene tip end, from the origin up
SHAFT = (0.020, 0.020, 0.083)  # tip cone and body shaft, z 0.012 .. 0.095
FLANGE = (0.050, 0.050, 0.008)  # shoulder that rests on the stand's rim, z 0.035 .. 0.043
FLANGE_Z = 0.039
CAP = (0.030, 0.030, 0.042)  # grip cap, z 0.095 .. 0.137
CAP_Z = 0.116
ARM_DY = -0.150  # the push button's offset from the body axis (y)
ARM = (0.016, 0.150, 0.006)  # plunger arm, z 0.085 .. 0.091
ARM_Z = 0.088
BUTTON = (0.050, 0.040, 0.010)  # push button, top at z 0.097
BUTTON_Z = 0.092
COLLAR = (0.016, 0.020, 0.010)  # blowout collar under the arm, top at z 0.073 (12 mm below the arm)
COLLAR_DY, COLLAR_Z = -0.035, 0.068
FIRST, SECOND = 0.012, 0.020  # plunger travel at the first stop and at the second stop (m)

STAND = (0.170, 0.100)  # stand sleeve centre
SLEEVE_IN, SLEEVE_WALL, SLEEVE_H = 0.032, 0.008, 0.043
STAND_Z = TOP + SLEEVE_H - (FLANGE_Z - FLANGE[2] / 2)  # pipette origin when standing in its stand
RES = (0.320, 0.120)  # reservoir centre
RES_IN, RES_WALL, RES_H, RES_FLOOR = (0.070, 0.060), 0.006, 0.030, 0.004
LEVEL = TOP + 0.020  # the sample's surface (blue band)
TUBE = (0.245, 0.150)  # tube centre
TUBE_IN, TUBE_WALL, TUBE_H, TUBE_FLOOR = 0.032, 0.004, 0.040, 0.004
TUBE_RIM = TOP + TUBE_H

kit = mk.Kit()
GREY, DARK, CAPC, TIPC = (0.62, 0.63, 0.66), (0.25, 0.26, 0.28), (0.20, 0.35, 0.65), (0.92, 0.88, 0.62)
BLUE, WHITE, STEEL = (0.10, 0.30, 0.85), (0.93, 0.93, 0.93), (0.55, 0.57, 0.60)
kit.part("pipette", (*STAND, STAND_Z), [
    mk.box(TIP, at=(0, 0, TIP[2] / 2), color=TIPC),
    mk.box(SHAFT, at=(0, 0, TIP[2] + SHAFT[2] / 2)),
    mk.box(FLANGE, at=(0, 0, FLANGE_Z), color=DARK),
    mk.box(CAP, at=(0, 0, CAP_Z), color=CAPC),
], mass=BODY_KG, color=GREY)
kit.part("plunger", (*STAND, STAND_Z), [
    mk.box(ARM, at=(0, ARM_DY / 2, ARM_Z)),
    mk.box(BUTTON, at=(0, ARM_DY, BUTTON_Z), color=CAPC),
], mass=PLUNGER_KG, color=DARK)
kit.part("collar", (STAND[0], STAND[1] + COLLAR_DY, STAND_Z + COLLAR_Z), [mk.box(COLLAR)], mass=COLLAR_KG,
         color=(0.85, 0.45, 0.10))
# soft first-stage spring: 0.25 N preload, 0.49 N at the first stop; stiff blowout spring: 1.0 N to 1.8 N
kit.slider("plunger_travel", "pipette", "plunger", axis=(0, 0, -1), lower=0.0, upper=SECOND, stiffness=20.0,
           rest=-0.0125, damping=0.3)
kit.slider("collar_travel", "pipette", "collar", axis=(0, 0, -1), lower=0.0, upper=SECOND - FIRST, stiffness=100.0,
           rest=-0.010, damping=0.3)

# emulated liquid effects: two small markers glued in place, let go by the scoring rule
GAUGE_Y = RES[1] - RES_IN[1] / 2 - RES_WALL - 0.008  # reservoir sight gauge, on its right (-y) face
MARK = (0.030, 0.010, 0.008)
kit.part("level_marker", (RES[0], GAUGE_Y, LEVEL), [mk.box(MARK)], mass=0.004, color=BLUE)
kit.bond("level_hold", None, "level_marker")
TAG_X = TUBE[0] - TUBE_IN / 2 - TUBE_WALL - 0.006  # tube front window
TAG = (0.006, 0.026, 0.022)
TAG_Z = TOP + 0.022
kit.part("empty_tag", (TAG_X, TUBE[1], TAG_Z), [mk.box(TAG)], mass=0.004, color=WHITE)
kit.bond("tag_hold", None, "empty_tag")


def _walls(name, cx, cy, inner, wall, h, z0, color):
    ix, iy = (inner, inner) if np.isscalar(inner) else inner
    zc, out = z0 + h / 2, []
    for s in (-1, 1):
        out.append(Box(f"{name}_wx{'np'[s > 0]}", pos=(cx + s * (ix / 2 + wall / 2), cy, zc),
                       size=(wall, iy + 2 * wall, h), color=color))
        out.append(Box(f"{name}_wy{'np'[s > 0]}", pos=(cx, cy + s * (iy / 2 + wall / 2), zc), size=(ix, wall, h),
                       color=color))
    return out


FIXTURES = [
    Box("stand_base", pos=(*STAND, TOP + 0.003), size=(0.080, 0.080, 0.006), color=DARK),
    *_walls("stand", *STAND, SLEEVE_IN, SLEEVE_WALL, SLEEVE_H - 0.006, TOP + 0.006, DARK),
    Box("res_floor", pos=(*RES, TOP + RES_FLOOR / 2), size=(RES_IN[0] + 2 * RES_WALL, RES_IN[1] + 2 * RES_WALL,
                                                             RES_FLOOR), color=BLUE),
    *_walls("res", *RES, RES_IN, RES_WALL, RES_H - RES_FLOOR, TOP + RES_FLOOR, WHITE),
    Box("res_band_n", pos=(RES[0] - RES_IN[0] / 2 + 0.001, RES[1], LEVEL - 0.003), size=(0.002, RES_IN[1], 0.006),
        color=BLUE),
    Box("res_band_f", pos=(RES[0] + RES_IN[0] / 2 - 0.001, RES[1], LEVEL - 0.003), size=(0.002, RES_IN[1], 0.006),
        color=BLUE),
    Box("gauge_back", pos=(RES[0], GAUGE_Y - 0.007, TOP + 0.017), size=(0.040, 0.004, 0.034), color=WHITE),
    Box("gauge_ledge", pos=(RES[0], GAUGE_Y, LEVEL - 0.014), size=(0.040, 0.016, 0.004), color=STEEL),
    Box("rack_base", pos=(*TUBE, TOP + 0.003), size=(0.060, 0.060, 0.006), color=STEEL),
    *_walls("tube", *TUBE, TUBE_IN, TUBE_WALL, TUBE_H - TUBE_FLOOR, TOP + TUBE_FLOOR, WHITE),
    Box("tube_window", pos=(TUBE[0] - TUBE_IN / 2 - TUBE_WALL - 0.0015, TUBE[1], TAG_Z), size=(0.003, 0.026, 0.022),
        color=BLUE),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
GOOD_MIN, GOOD_MAX = 0.010, 0.0155  # plunger travel when the tip goes in: at the first stop
ASPIRATED_AT = 0.002  # the button let up to within 2 mm of the top
PRESS_TOL = 0.003  # pressing more than 3 mm counts as a press
BLOWOUT_AT = 0.0170  # second stop reached
DISPENSE_AT = 0.010  # first stop reached in the tube
REASP_BELOW = 0.005  # button let up below 5 mm with the tip still in the tube
TILT_MAX = 45.0  # loaded pipette tilted further: liquid runs into the shaft
STAND_TILT = 10.0
M = "pfs"  # key of this task's per-episode memory in mk.MEMORY


def _mem() -> dict:
    return mk.MEMORY.setdefault(M, {"phase": "empty", "stint": None, "flags": set()})


def travel(objects) -> float:
    """How far the push button is pressed down against the body (m)."""
    return kit.q(objects, "plunger_travel")


def tip(objects) -> np.ndarray:
    return kit.point(objects, "pipette", (0.0, 0.0, 0.0))


def tilt_deg(objects) -> float:
    up = kit.point(objects, "pipette", (0.0, 0.0, 1.0)) - tip(objects)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))


def immersed(p) -> bool:
    m = 0.004
    return bool(abs(p[0] - RES[0]) < RES_IN[0] / 2 - m and abs(p[1] - RES[1]) < RES_IN[1] / 2 - m
                and TOP + RES_FLOOR < p[2] < LEVEL)


def in_tube(p) -> bool:
    return bool(abs(p[0] - TUBE[0]) < TUBE_IN / 2 and abs(p[1] - TUBE[1]) < TUBE_IN / 2 and p[2] < TUBE_RIM)


def _record(objects) -> None:
    """Advance the pipetting state machine by one step (every control step in sim)."""
    mem, d, p = _mem(), travel(objects), tip(objects)
    flags = mem["flags"]
    if mem["phase"] in ("empty", "delivered"):
        if immersed(p):
            st = mem["stint"]
            if st is None or st[0] != "res":
                st = mem["stint"] = ["res", d, d, d]  # where, travel at entry, max, running min
            st[2], st[3] = max(st[2], d), min(st[3], d)
            if d > st[3] + PRESS_TOL:
                flags.add("bubbled")
            if d <= ASPIRATED_AT and st[2] >= GOOD_MIN:
                if st[1] > GOOD_MAX:
                    flags.add("over_aspirated")
                if mem["phase"] == "delivered":
                    flags.add("second_volume")
                mem["phase"], mem["stint"], mem["loaded_min"] = "loaded", None, d
                kit.release("level_hold")
        else:
            mem["stint"] = None
    elif mem["phase"] == "loaded":
        if tilt_deg(objects) > TILT_MAX:
            flags.add("shaft_contaminated")
        if in_tube(p):
            st = mem["stint"]
            if st is None or st[0] != "tube":
                st = mem["stint"] = ["tube", d, d, False]  # where, entry, max, let up
            st[2] = max(st[2], d)
            if st[2] >= DISPENSE_AT and d < REASP_BELOW:
                st[3] = True
        else:
            st = mem["stint"]
            if st is not None and st[0] == "tube" and st[2] >= DISPENSE_AT:
                if st[2] < BLOWOUT_AT:
                    flags.add("incomplete")
                if st[3]:
                    flags.add("re_aspirated")
                mem["phase"], mem["stint"] = "delivered", None
                kit.release("tag_hold")
            else:
                mem["stint"] = None
                if d > mem["loaded_min"] + PRESS_TOL:
                    flags.add("spilled")
                mem["loaded_min"] = min(mem["loaded_min"], d)


def _on_step(objects) -> None:
    _record(objects)


kit.on_step(_on_step)


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def stowed(objects) -> bool:
    """Upright in its stand: the tip within 1 cm of the sleeve's axis, the shoulder on the rim, tilt under 10 deg,
    still, and neither gripper within 2.5 cm of the pipette's parts."""
    p = tip(objects)
    flange_bottom = kit.point(objects, "pipette", (0.0, 0.0, FLANGE_Z - FLANGE[2] / 2))[2]
    return bool(math.hypot(p[0] - STAND[0], p[1] - STAND[1]) < 0.010
                and TOP + SLEEVE_H - 0.003 < flange_bottom < TOP + SLEEVE_H + 0.004
                and tilt_deg(objects) < STAND_TILT and objects["pipette"].resting()
                and all(_released(objects[n]) for n in ("pipette", "plunger", "collar")))


def score(objects) -> float:
    """0 unless one volume was drawn up in the reservoir and dispensed in the tube without a spill (pressing the
    loaded pipette outside the tube) or tilting it past 45 deg while loaded. Then 1.0 for the forward technique
    (first stop before the tip went in, no press in the liquid, blowout at the second stop, tip withdrawn before
    the button came up, one volume only) with the pipette back upright in its stand and let go; 0.5 for a
    delivery that skipped the blowout, let the button up in the tube or took a second volume; 0.25 for one drawn
    up at the second stop or with a press in the liquid. A correct transfer not stood back in the stand: 0.75."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _record(objects)
    mem = _mem()
    f = mem["flags"]
    if mem["phase"] != "delivered" or f & {"spilled", "shaft_contaminated"}:
        return 0.0
    if f & {"over_aspirated", "bubbled"}:
        return 0.25
    if f & {"incomplete", "re_aspirated", "second_volume"}:
        return 0.5
    return 1.0 if stowed(objects) else 0.75


def success(objects) -> bool:
    """A clean forward-pipetted transfer, the pipette stood back; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Steps of the transfer still to do, plus the tip's distance from the stand (m)."""
    p, ph = tip(objects), _mem()["phase"]
    left = {"empty": 2, "loaded": 1, "delivered": 0}[ph]
    return float(left + math.hypot(p[0] - STAND[0], p[1] - STAND[1]) + abs(p[2] - STAND_Z))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry. The left arm holds the pipette by its cap (fingers closing along y); the
# right arm works the push button from above with its fingers closed. Both arms' waypoints share one timeline, so
# while the button is held down the right arm follows the pipette exactly.

TIP_PAST_TCP = 0.013
SQUEEZE = 0.008
CAP_TCP = CAP_Z - CAP[2] / 2 + 0.004 + TIP_PAST_TCP  # left TCP above the pipette's origin (pad tips 4 mm up the cap)
BTN_TCP = BUTTON_Z + BUTTON[2] / 2 + TIP_PAST_TCP  # right TCP above the origin with the pads touching the button
HANG = 0.005  # how far the held pipette hangs below where the cap was gripped (sim run 2: 2-5 mm)
PAD_DEEP = 0.0065  # the right arm stops this much higher than commanded on the button (sim run 2)
STAND_TRIM = (0.004, -0.0045)  # the left arm lands this far off at the stand (sim run 2: -3.9 mm x, +4.6 mm y)
PRESS1 = 0.0125  # commanded press for the first stop
PRESS2 = 0.024  # commanded press for the second stop (the plunger's own stop is at 20 mm)
CARRY_Z = TOP + 0.058  # pipette origin while carrying: tip 18 mm above the tube's rim
PRE_Z = TOP + 0.042  # tip 12 mm above the reservoir rim, 22 mm above the sample
DIP_Z = TOP + 0.009  # tip 11 mm under the sample's surface
TUBE_Z = TOP + 0.014  # tip 26 mm down in the tube
OUT_Z = TOP + 0.047  # tip 7 mm above the tube's rim
GRIP = -(CAP[1] - SQUEEZE) / 2
PARK = (0.230, -0.200, TOP + 0.160)


def demo():
    """Left: lift the pipette from its stand and hold it over the reservoir. Right: press the button to the first
    stop. Both lower together so the tip goes 15 mm into the sample; the right lets the button up slowly; the
    left lifts the tip out and carries it into the tube. Right: press to the first stop, pause, on to the second
    stop. Both lift together so the tip leaves the tube with the button held down; then the right lets it up.
    Left: stand the pipette back in its stand and let go."""
    o, g, c = FINGER_OPEN, GRIP, FINGER_CLOSED
    L, R = [], []

    def ph(dur, label, lo, lf, ro, rf=c):
        """One shared segment: left TCP at pipette origin ``lo`` (or an absolute point), right TCP at ``ro``."""
        L.append(Waypoint(dur, tuple(float(v) for v in lo), lf, label, 0.0))
        R.append(Waypoint(dur, tuple(float(v) for v in ro), rf, label, 0.0))

    def cap(x, y, z):  # left TCP holding the pipette with its origin at (x, y, z)
        return (x, y, z + CAP_TCP + HANG)

    def btn(x, y, z, press):  # right TCP pressing the button ``press`` down, pipette origin at (x, y, z)
        return (x, y + ARM_DY, z + BTN_TCP - press - (PAD_DEEP if press > 0 else 0.0))

    sx, sy = STAND
    rx, ry = RES
    tx, ty = TUBE
    ph(2.0, "over the pipette's cap", cap(sx, sy, STAND_Z + 0.045), o, PARK)
    ph(1.5, "down around the cap", (sx, sy, STAND_Z + CAP_TCP), o, PARK)
    ph(0.5, "settle", (sx, sy, STAND_Z + CAP_TCP), o, PARK)
    ph(1.0, "grip the cap", (sx, sy, STAND_Z + CAP_TCP), g, PARK)
    ph(0.5, "settle closed", (sx, sy, STAND_Z + CAP_TCP), g, PARK)
    ph(2.0, "lift out of the stand", cap(sx, sy, CARRY_Z), g, PARK)
    ph(2.0, "carry over the reservoir", cap(rx, ry, CARRY_Z), g, PARK)
    ph(1.5, "lower: tip just above the sample", cap(rx, ry, PRE_Z), g, btn(rx, ry, PRE_Z, -0.015))
    ph(0.5, "settle", cap(rx, ry, PRE_Z), g, btn(rx, ry, PRE_Z, -0.015))
    ph(1.0, "right pads onto the button", cap(rx, ry, PRE_Z), g, btn(rx, ry, PRE_Z, -0.002))
    ph(1.5, "press to the first stop", cap(rx, ry, PRE_Z), g, btn(rx, ry, PRE_Z, PRESS1))
    ph(1.0, "hold at the first stop", cap(rx, ry, PRE_Z), g, btn(rx, ry, PRE_Z, PRESS1))
    ph(2.5, "together: tip 15 mm into the sample", cap(rx, ry, DIP_Z), g, btn(rx, ry, DIP_Z, PRESS1))
    ph(0.5, "settle", cap(rx, ry, DIP_Z), g, btn(rx, ry, DIP_Z, PRESS1))
    ph(2.5, "let the button up slowly", cap(rx, ry, DIP_Z), g, btn(rx, ry, DIP_Z, -0.006))
    ph(0.8, "wait with the tip in", cap(rx, ry, DIP_Z), g, btn(rx, ry, DIP_Z, -0.006))
    ph(1.0, "right arm up and away", cap(rx, ry, DIP_Z), g, (rx, ry + ARM_DY - 0.05, TOP + 0.165))
    ph(1.5, "lift the tip out", cap(rx, ry, CARRY_Z), g, PARK)
    ph(2.0, "carry over the tube", cap(tx, ty, CARRY_Z), g, PARK)
    ph(1.5, "tip down into the tube", cap(tx, ty, TUBE_Z), g, btn(tx, ty, TUBE_Z, -0.025))
    ph(0.5, "settle", cap(tx, ty, TUBE_Z), g, btn(tx, ty, TUBE_Z, -0.025))
    ph(1.0, "right pads onto the button", cap(tx, ty, TUBE_Z), g, btn(tx, ty, TUBE_Z, -0.002))
    ph(1.5, "press to the first stop (dispense)", cap(tx, ty, TUBE_Z), g, btn(tx, ty, TUBE_Z, PRESS1))
    ph(1.0, "pause at the first stop", cap(tx, ty, TUBE_Z), g, btn(tx, ty, TUBE_Z, PRESS1))
    ph(1.2, "on to the second stop (blowout)", cap(tx, ty, TUBE_Z), g, btn(tx, ty, TUBE_Z, PRESS2))
    ph(1.2, "hold at the second stop", cap(tx, ty, TUBE_Z), g, btn(tx, ty, TUBE_Z, PRESS2))
    ph(2.0, "together: tip out of the tube, button held", cap(tx, ty, OUT_Z), g, btn(tx, ty, OUT_Z, PRESS2))
    ph(1.5, "let the button up", cap(tx, ty, OUT_Z), g, btn(tx, ty, OUT_Z, -0.006))
    ph(1.0, "right arm away", cap(tx, ty, OUT_Z), g, (tx, ty + ARM_DY - 0.05, TOP + 0.165))
    px, py = sx + STAND_TRIM[0], sy + STAND_TRIM[1]
    ph(2.0, "carry over the stand", cap(px, py, CARRY_Z), g, PARK)
    ph(0.5, "settle over the stand", cap(px, py, CARRY_Z), g, PARK)
    ph(1.5, "lower into the sleeve", cap(px, py, STAND_Z + 0.015), g, PARK)
    ph(1.2, "set the shoulder on the rim", cap(px, py, STAND_Z + 0.002), g, PARK)
    ph(0.5, "settle", cap(px, py, STAND_Z + 0.002), g, PARK)
    ph(1.0, "let go", cap(px, py, STAND_Z + 0.002), o, PARK)
    ph(1.5, "retreat", cap(px, py, STAND_Z + 0.050), o, PARK)
    return {"left": L, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of pipette poses (one trial each, fed step by step)."""
    global _TEST_TCPS

    def fr(x, y, z, d=0.0, tilt=0.0):
        """The pipette with its tip end at (x, y, z), tilted ``tilt`` deg about x, button pressed ``d`` (m)."""
        quat = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(tilt)))
        return kit.fake_states(q={"plunger_travel": d, "collar_travel": max(0.0, d - FIRST)},
                               moved={"pipette": ((x, y, z), quat)})

    def move(a, b, n=10, d0=0.0, d1=None):
        d1 = d0 if d1 is None else d1
        return [fr(*(np.asarray(a) + (np.asarray(b) - np.asarray(a)) * k / n), d=d0 + (d1 - d0) * k / n)
                for k in range(n + 1)]

    st0 = (*STAND, STAND_Z)
    carry_s, carry_r, carry_t = (*STAND, CARRY_Z), (*RES, CARRY_Z), (*TUBE, CARRY_Z)
    pre, dip, tub = (*RES, PRE_Z), (*RES, DIP_Z), (*TUBE, TUBE_Z)

    def aspirate(depth=0.0125, press_in=False):
        seq = move(st0, carry_s) + move(carry_s, carry_r) + move(carry_r, pre)
        if press_in:  # dip first, press in the liquid, then let up
            seq += move(pre, dip) + move(dip, dip, d0=0.0, d1=depth) + move(dip, dip, d0=depth, d1=0.0)
        else:
            seq += move(pre, pre, d0=0.0, d1=depth) + move(pre, dip, d0=depth) + move(dip, dip, d0=depth, d1=0.0)
        return seq + move(dip, carry_r) + move(carry_r, carry_t)

    def dispense(blow=True, hold_out=True, depth=None):
        dd = depth if depth is not None else (0.0195 if blow else 0.0125)
        seq = move(carry_t, tub) + move(tub, tub, d0=0.0, d1=dd)
        if hold_out:
            seq += move(tub, carry_t, d0=dd) + move(carry_t, carry_t, d0=dd, d1=0.0)
        else:
            seq += move(tub, tub, d0=dd, d1=0.0) + move(tub, carry_t)
        return seq

    stow = move(carry_t, carry_s) + move(carry_s, st0)
    good = aspirate() + dispense() + stow
    grip_tcp = kit.point(fr(*st0), "pipette", (0.0, 0.0, CAP_TCP))
    cases = {  # name: (frames, TCPs at the end, score)
        "start": ([fr(*st0)], [], 0.0),
        "lifted and put back": (move(st0, carry_s) + move(carry_s, st0), [], 0.0),
        "forward technique, stood back, let go": (good, [], 1.0),
        "forward technique, still held in the stand": (good, [grip_tcp], 0.75),
        "forward technique, left over the tube": (aspirate() + dispense(), [], 0.75),
        "pressed to the bottom before aspirating (naive)": (aspirate(depth=0.0195) + dispense() + stow, [], 0.25),
        "dipped first, pressed in the liquid": (aspirate(press_in=True) + dispense() + stow, [], 0.25),
        "no blowout (first stop only)": (aspirate() + dispense(blow=False) + stow, [], 0.5),
        "button let up in the tube": (aspirate() + dispense(hold_out=False) + stow, [], 0.5),
        "loaded pipette pressed over the bench": (aspirate() + move(carry_t, carry_t, d0=0.0, d1=0.012)
                                                  + move(carry_t, carry_t, d0=0.012, d1=0.0) + dispense() + stow,
                                                  [], 0.0),
        "loaded pipette laid over": (aspirate() + [fr(*carry_t, tilt=60.0)] + dispense() + stow, [], 0.0),
        "loaded pipette tilted 30 deg": (aspirate() + [fr(*carry_t, tilt=30.0)] + dispense() + stow, [], 1.0),
        "aspirated, never dispensed": (aspirate() + stow, [], 0.0),
        "pressed to 5 mm only (nothing drawn)": (aspirate(depth=0.005) + dispense() + stow, [], 0.0),
        "two volumes into the tube": (aspirate() + dispense() + move(carry_t, carry_r) + move(carry_r, pre)
                                      + move(pre, pre, d0=0.0, d1=0.0125) + move(pre, dip, d0=0.0125)
                                      + move(dip, dip, d0=0.0125, d1=0.0) + move(dip, carry_r)
                                      + move(carry_r, carry_t) + dispense() + stow, [], 0.5),
        "dispensed outside the tube (beside it)": (aspirate() + move(carry_t, (TUBE[0] + 0.05, TUBE[1], TUBE_Z))
                                                   + move((TUBE[0] + 0.05, TUBE[1], TUBE_Z),
                                                          (TUBE[0] + 0.05, TUBE[1], TUBE_Z), d0=0.0, d1=0.0195)
                                                   + stow, [], 0.0),
        "stood back on the sleeve's edge": (good[:-1] + [fr(STAND[0] + 0.02, STAND[1], STAND_Z + 0.01)], [], 0.75),
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
        for f in good:
            score(f)
        mk.new_episode()
        assert score(fr(*st0)) == 0.0, "a new trial forgets the last one"
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

