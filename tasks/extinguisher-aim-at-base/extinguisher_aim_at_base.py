"""On the station table a small stored-pressure fire extinguisher stands on a grey mat in front of the arms, and
farther out stands a steel waste bin on fire: three orange flame blocks rise from the burning paper (the fuel bed)
in its left, middle and right thirds. The extinguisher is a rig for two arms: a red cylinder with a valve head, a
grip block on top, a short fixed nozzle that points forward and 30 deg down, a fixed carry handle bar running out
to its left, and above that bar the sprung squeeze lever (hinged at the head, held 15 deg open by its spring). A
safety pin with a pull ring passes between lever and handle bar near the pivot and blocks the lever; it slides
out toward the arms against light friction (the tamper seal itself is not modelled). On a hand-held unit one hand squeezes lever and handle
together; here the lever reaches 15 cm out so the second arm can press it.

The extinguishing agent is not simulated. While the pin is out and the lever is squeezed to within 8 deg of closed,
the unit discharges (10 s of charge); every control step the score follows the nozzle's axis as a ray and finds
where it meets the fuel bed. The visible effect is emulated: once a third of the bin has had 1.2 s of discharge
on its fuel bed, that third's flame block is let go and topples down into the bin.

PASS, as taught under OSHA 29 CFR 1910.157(g) training and printed on extinguisher labels (not stated in the
instruction): pull the pin, aim at the base of the fire (not at the flames), squeeze, sweep side to side, and
keep your distance. A ray that passes over the flames or into the far wall wastes the charge; a nozzle closer than
6 cm to the bin's rim (scaled from the label's range) is too close.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/extinguisher-aim-at-base/extinguisher_aim_at_base.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (per-episode memory, gripper TCPs, hinge, slider, bonds)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the extinguisher (kit parts; frame origin = the bottom centre of the cylinder) ------------------------------
MAT = (0.260, -0.060)  # mat centre = the extinguisher's start
MAT_SIZE = (0.120, 0.120, 0.003)
EXT_KG, LEVER_KG, PIN_KG = 0.300, 0.020, 0.010
BODY = (0.060, 0.060, 0.080)  # red steel cylinder (square section)
HEAD = (0.040, 0.040, 0.015)  # valve head, z 0.080 .. 0.095
GRIPB = (0.050, 0.024, 0.045)  # grip block on top, z 0.095 .. 0.140
BAR = (0.020, 0.110, 0.006)  # fixed carry handle bar, y 0.020 .. 0.130, z 0.077 .. 0.083
HINGE_Y, HINGE_Z = 0.030, 0.090  # lever pivot (axis along x)
LEVER = (0.030, 0.120, 0.006)  # squeeze lever, y 0.030 .. 0.150, z 0.087 .. 0.093 when closed
LEVER_END = (0.040, 0.060, 0.006)  # press pad at the lever's end, y 0.120 .. 0.180
LEVER_OPEN = 15.0  # deg, held there by its spring
SQUEEZED = 8.0  # discharging below this lever angle (squeezed over half its travel)
PIN_Y = 0.110  # the pin crosses the bar 8 cm from the pivot
PIN_ROD = (0.052, 0.006, 0.015)  # x -0.035 .. 0.017, z 0.0865 .. 0.1015: blocks the lever at ~11 deg
PIN_ROD_X, PIN_ROD_Z = -0.009, 0.0025
RING = (0.020, 0.024, 0.024)  # pull ring, centred x -0.045
RING_X, PIN_Z = -0.045, 0.0915
PIN_OUT = 0.035  # pulled at least this far: the rod is clear of the lever (x -0.015 .. 0.015)
NOZ_DEG = 30.0
NOZ_LEN = 0.040
NOZ_BASE = (0.020, 0.088)  # where the nozzle leaves the head (x, z)
NOZ_TIP = (NOZ_BASE[0] + NOZ_LEN * math.cos(math.radians(NOZ_DEG)), 0.0,
           NOZ_BASE[1] - NOZ_LEN * math.sin(math.radians(NOZ_DEG)))
NOZ_DIR = (math.cos(math.radians(NOZ_DEG)), 0.0, -math.sin(math.radians(NOZ_DEG)))
EXT_Z = TOP + MAT_SIZE[2]

# -- the bin and the fire -----------------------------------------------------------------------------------------
BIN_Y = -0.080
BIN_IN = (0.120, 0.100)  # inner x, y
BIN_WALL, BIN_H = 0.006, 0.045
NEAR_X = 0.400  # inner face of the near wall
FUEL_Z = TOP + 0.025  # top of the burning paper: the base of the fire
RIM_Z = TOP + BIN_H
FLAME_X = NEAR_X + 0.075
FLAME_DY = (-0.0333, 0.0, 0.0333)  # flame centres from the bin's centre line: right, middle, left thirds
THIRD = BIN_IN[1] / 3
BASE_FAR = FLAME_X + 0.020  # the fire's base: the fuel bed up to the flames' far side (3 mm past it)
CHARGE_S = 10.0
OUT_S = 1.2  # discharge on a third's fuel bed that puts it out
TOO_CLOSE = 0.060  # nozzle closer than this to the near rim (horizontal) while discharging
DT = 0.05

RED, BLACK, GREY, STEEL = (0.78, 0.08, 0.06), (0.10, 0.10, 0.11), (0.45, 0.46, 0.48), (0.55, 0.57, 0.60)
ORANGE, PAPER, YELLOW = (1.00, 0.45, 0.05), (0.32, 0.24, 0.16), (0.95, 0.80, 0.10)

kit = mk.Kit()
_noz_c = (NOZ_BASE[0] + NOZ_LEN / 2 * math.cos(math.radians(NOZ_DEG)), 0.0,
          NOZ_BASE[1] - NOZ_LEN / 2 * math.sin(math.radians(NOZ_DEG)))
kit.part("extinguisher", (*MAT, EXT_Z), [
    mk.box(BODY, at=(0, 0, BODY[2] / 2)),
    mk.box(HEAD, at=(0, 0, BODY[2] + HEAD[2] / 2), color=BLACK),
    mk.box(GRIPB, at=(0, 0, BODY[2] + HEAD[2] + GRIPB[2] / 2), color=BLACK),
    mk.box(BAR, at=(0, 0.020 + BAR[1] / 2, 0.077 + BAR[2] / 2), color=BLACK),
    mk.box((NOZ_LEN, 0.010, 0.010), at=_noz_c, color=BLACK,
           quat=tuple(mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.radians(NOZ_DEG))))),
], mass=EXT_KG, color=RED)
kit.part("lever", (MAT[0], MAT[1] + HINGE_Y, EXT_Z + HINGE_Z), [
    mk.box(LEVER, at=(0, LEVER[1] / 2, 0.0)),
    mk.box(LEVER_END, at=(0, 0.120, 0.0)),
], mass=LEVER_KG, color=STEEL)
# axis +x: a positive angle lifts the lever's far end; the spring holds it at its 15 deg stop (0.06 N*m closed)
kit.hinge("lever_hinge", "extinguisher", "lever", anchor=(MAT[0], MAT[1] + HINGE_Y, EXT_Z + HINGE_Z),
          axis=(1, 0, 0), lower_deg=-2.0, upper_deg=LEVER_OPEN, start_deg=LEVER_OPEN, stiffness=0.12, rest_deg=30.0,
          damping=0.002)
kit.part("pin", (MAT[0], MAT[1] + PIN_Y, EXT_Z + PIN_Z), [
    mk.box(PIN_ROD, at=(PIN_ROD_X, 0, PIN_ROD_Z)),
    mk.box(RING, at=(RING_X, 0, 0), color=YELLOW),
], mass=PIN_KG, color=STEEL)
kit.slider("pin_travel", "extinguisher", "pin", axis=(-1, 0, 0), lower=0.0, upper=0.045, friction=0.2)

FLAMES = ("flame_r", "flame_m", "flame_l")
for name, dy in zip(FLAMES, FLAME_DY):
    # an off-balance tongue of flame: once let go it topples forward and lies flat on the paper, below the rim
    kit.part(name, (FLAME_X, BIN_Y + dy, FUEL_Z), [
        mk.box((0.012, 0.026, 0.008), at=(0, 0, 0.004)),
        mk.box((0.014, 0.026, 0.037), at=(-0.010, 0, 0.008 + 0.0185)),
    ], mass=0.010, color=ORANGE)
    kit.bond(f"{name}_hold", None, name)

_bx = NEAR_X + BIN_IN[0] / 2
FIXTURES = [
    Box("mat", pos=(*MAT, TOP + MAT_SIZE[2] / 2), size=MAT_SIZE, color=GREY),
    Box("bin_floor", pos=(_bx, BIN_Y, TOP + 0.002), size=(BIN_IN[0] + 2 * BIN_WALL, BIN_IN[1] + 2 * BIN_WALL, 0.004),
        color=STEEL),
    Box("fuel_bed", pos=(_bx, BIN_Y, (TOP + 0.004 + FUEL_Z) / 2), size=(BIN_IN[0], BIN_IN[1], FUEL_Z - TOP - 0.004),
        color=PAPER),
    Box("bin_near", pos=(NEAR_X - BIN_WALL / 2, BIN_Y, TOP + BIN_H / 2), size=(BIN_WALL, BIN_IN[1] + 2 * BIN_WALL, BIN_H),
        color=STEEL),
    Box("bin_far", pos=(NEAR_X + BIN_IN[0] + BIN_WALL / 2, BIN_Y, TOP + BIN_H / 2),
        size=(BIN_WALL, BIN_IN[1] + 2 * BIN_WALL, BIN_H), color=STEEL),
    Box("bin_right", pos=(_bx, BIN_Y - BIN_IN[1] / 2 - BIN_WALL / 2, TOP + BIN_H / 2), size=(BIN_IN[0], BIN_WALL, BIN_H),
        color=STEEL),
    Box("bin_left", pos=(_bx, BIN_Y + BIN_IN[1] / 2 + BIN_WALL / 2, TOP + BIN_H / 2), size=(BIN_IN[0], BIN_WALL, BIN_H),
        color=STEEL),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
_TEST_TCPS = None
M = "ext"


def _mem() -> dict:
    return mk.MEMORY.setdefault(M, {"used": 0.0, "hit": [0.0, 0.0, 0.0], "out": [False] * 3, "close": False})


def pin_out(objects) -> bool:
    return kit.q(objects, "pin_travel") >= PIN_OUT


def discharging(objects) -> bool:
    """Pin pulled clear and lever squeezed to within 8 deg of closed."""
    return pin_out(objects) and kit.q_deg(objects, "lever_hinge") < SQUEEZED


def aim(objects):
    """(nozzle tip, the point where the nozzle's axis meets the fuel bed's plane or None, base third index or None,
    clears the near rim)."""
    n = kit.point(objects, "extinguisher", NOZ_TIP)
    u = kit.point(objects, "extinguisher", tuple(np.add(NOZ_TIP, NOZ_DIR))) - n
    if u[2] > -1e-6 or n[2] <= FUEL_Z:
        return n, None, None, False
    f = n + (FUEL_Z - n[2]) / u[2] * u
    rim_x = NEAR_X - BIN_WALL
    if n[0] < rim_x:
        if u[0] <= 0:
            return n, f, None, False
        zr = n[2] + (rim_x - n[0]) / u[0] * u[2]
        clear = bool(zr > RIM_Z)
    else:
        clear = True
    third = None
    dy = f[1] - BIN_Y
    if clear and NEAR_X <= f[0] <= BASE_FAR and abs(dy) < BIN_IN[1] / 2:
        third = min(2, int((dy + BIN_IN[1] / 2) // THIRD))
    return n, f, third, clear


def _record(objects) -> None:
    """One control step of discharge bookkeeping (every step in sim)."""
    mem = _mem()
    if not discharging(objects) or mem["used"] >= CHARGE_S - 1e-9:
        return
    mem["used"] += DT
    n, f, third, _ = aim(objects)
    if NEAR_X - BIN_WALL - n[0] < TOO_CLOSE and abs(n[1] - BIN_Y) < BIN_IN[1] / 2 + 0.10:
        mem["close"] = True
    if third is not None and not mem["out"][third]:
        mem["hit"][third] += DT
        if mem["hit"][third] >= OUT_S - 1e-9:
            mem["out"][third] = True
            kit.release(f"{FLAMES[third]}_hold")


kit.on_step(_record)


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def stowed(objects) -> bool:
    """Standing upright on the mat (base centre on it, bottom on it, tilt under 10 deg), lever back open past 10
    deg, still, and neither gripper within 2.5 cm of any of its parts."""
    o = kit.point(objects, "extinguisher", (0, 0, 0))
    up = kit.point(objects, "extinguisher", (0, 0, 1)) - o
    return bool(abs(o[0] - MAT[0]) < MAT_SIZE[0] / 2 - 0.02 and abs(o[1] - MAT[1]) < MAT_SIZE[1] / 2 - 0.02
                and abs(o[2] - EXT_Z) < 0.004 and up[2] > math.cos(math.radians(10.0))
                and kit.q_deg(objects, "lever_hinge") > 10.0 and objects["extinguisher"].resting()
                and all(_released(objects[n]) for n in ("extinguisher", "lever", "pin")))


def score(objects) -> float:
    """0.25 for each third of the bin put out (1.2 s of discharge aimed on its fuel bed, over the near rim and
    short of the flames' far side), capped at 0.5 if the nozzle was ever under 6 cm from the bin's near rim while
    discharging. 1.0 needs all three out, never too close, and the extinguisher back upright on the mat with the
    lever released, let go and still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _record(objects)
    mem = _mem()
    n = sum(mem["out"])
    if mem["close"]:
        return min(0.5, 0.25 * n)
    if n == 3 and stowed(objects):
        return 1.0
    return 0.25 * n


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Seconds of base discharge still needed, plus the extinguisher's distance from the mat (m)."""
    mem = _mem()
    o = kit.point(objects, "extinguisher", (0, 0, 0))
    left = sum(0.0 if out else OUT_S - h for out, h in zip(mem["out"], mem["hit"]))
    return float(left + math.hypot(o[0] - MAT[0], o[1] - MAT[1]) + abs(o[2] - EXT_Z))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry. Right arm: holds the grip block (fingers closing along y), lifts the unit
# to its aiming height and sweeps it sideways. Left arm: pulls the pin, then presses the lever's end down with
# closed fingers and follows the sweep. Both arms share one timeline.

TIP_PAST_TCP = 0.013
SQUEEZE = 0.008
GRIP_TCP = BODY[2] + HEAD[2] + 0.004 + TIP_PAST_TCP  # right TCP above the origin (pad tips 4 mm down the block)
RING_TCP = PIN_Z - RING[2] / 2 + 0.002 + TIP_PAST_TCP
LEVER_TOP = HINGE_Z + 0.003  # lever's top face at the pivot when closed
LEVER_R = 0.120  # pivot to the press pad
HANG = 0.012  # how far the held unit hangs below where its grip block was taken (sim runs 1-2: 7-13 mm)
LIFT = 0.045  # aiming height: unit bottom 4.5 cm above the mat
SWEEP = 0.036  # sweep half-width (unit y about the bin's centre line)
PRESS_PAST = 0.014  # commanded past the closed lever (the arms gave way 5-10 mm in sim runs 3-7)
AIM_DX = -0.012  # the unit is aimed 1.2 cm behind the mat's centre (sim run 3 landed ~2 cm past the aim point)
AIM_DY = 0.016  # the sweep is centred 1.6 cm left of the bin's centre line: pressing the lever rolls the held
# unit, which puts the nozzle's aim 1.6 cm to the right of the grip (sim run 5)
R_GRIP = -(GRIPB[1] - SQUEEZE) / 2
L_GRIP = -(RING[1] - SQUEEZE) / 2
L_PARK = (0.220, 0.200, TOP + 0.160)
PULL = 0.046  # commanded pull on the ring: just past the pin's 45 mm stop (pulling on past it yanked the joint)
R_YAW = 0.0  # fingers close along y: the lever's push (at +y) loads the grip in its stiff direction


def _demo_phases():
    """(duration, label, left TCP, left finger, right TCP, right finger) for each shared segment."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    ux, uy = MAT
    z0 = EXT_Z
    za = EXT_Z + LIFT
    rx = ux + RING_X
    ry = uy + PIN_Y

    def grip(y, z):  # right TCP holding the unit with its origin at (x, y, z); x = the aiming x when lifted
        return (ux + (AIM_DX if z >= za - 1e-9 else 0.0), y, z + GRIP_TCP + HANG)

    def lever(y, z, deg):  # left TCP with the pads on the lever's end at angle deg, unit origin at (ux, y, z)
        return (ux + AIM_DX, y + HINGE_Y + LEVER_R, z + LEVER_TOP + LEVER_R * math.sin(math.radians(deg)) + TIP_PAST_TCP)

    P = [
        (2.0, "over the grip block and the pull ring", (rx, ry, z0 + RING_TCP + 0.04), o,
         (ux, uy, z0 + GRIP_TCP + 0.04), o),
        (1.5, "down around them", (rx, ry, z0 + RING_TCP), o, (ux, uy, z0 + GRIP_TCP), o),
        (0.5, "settle", (rx, ry, z0 + RING_TCP), o, (ux, uy, z0 + GRIP_TCP), o),
        (1.0, "grip", (rx, ry, z0 + RING_TCP), L_GRIP, (ux, uy, z0 + GRIP_TCP), R_GRIP),
        (0.5, "settle closed", (rx, ry, z0 + RING_TCP), L_GRIP, (ux, uy, z0 + GRIP_TCP), R_GRIP),
        (4.0, "pull the pin", (rx - PULL, ry, z0 + RING_TCP), L_GRIP, (ux, uy, z0 + GRIP_TCP), R_GRIP),
        (0.5, "settle", (rx - PULL, ry, z0 + RING_TCP), L_GRIP, (ux, uy, z0 + GRIP_TCP), R_GRIP),
        (1.0, "let the ring go", (rx - PULL, ry, z0 + RING_TCP), o, (ux, uy, z0 + GRIP_TCP), R_GRIP),
        (1.2, "left up; lift the unit to aiming height", (rx - PULL - 0.010, ry, z0 + RING_TCP + 0.05), o,
         grip(uy, za), R_GRIP),
        (1.5, "to the right end of the sweep", lever(BIN_Y + AIM_DY - SWEEP, za, LEVER_OPEN + 8.0), c,
         grip(BIN_Y + AIM_DY - SWEEP, za), R_GRIP),
        (0.8, "pads onto the lever", lever(BIN_Y + AIM_DY - SWEEP, za, LEVER_OPEN - 1.0), c, grip(BIN_Y + AIM_DY - SWEEP, za), R_GRIP),
        (1.2, "squeeze", (lambda p: (p[0], p[1], p[2] - PRESS_PAST))(lever(BIN_Y + AIM_DY - SWEEP, za, 0.0)), c,
         grip(BIN_Y + AIM_DY - SWEEP, za), R_GRIP),
    ]
    squeezed = lambda y: (lambda p: (p[0], p[1], p[2] - PRESS_PAST))(lever(y, za, 0.0))  # noqa: E731
    P += [
        (3.0, "sweep across to the left", squeezed(BIN_Y + AIM_DY + SWEEP), c, grip(BIN_Y + AIM_DY + SWEEP, za), R_GRIP),
        (3.0, "sweep back to the right", squeezed(BIN_Y + AIM_DY - SWEEP), c, grip(BIN_Y + AIM_DY - SWEEP, za), R_GRIP),
        (1.5, "back to the middle", squeezed(BIN_Y + AIM_DY), c, grip(BIN_Y + AIM_DY, za), R_GRIP),
        (1.0, "let the lever up", lever(BIN_Y + AIM_DY, za, LEVER_OPEN + 10.0), c, grip(BIN_Y + AIM_DY, za), R_GRIP),
        (1.5, "left away; unit back over the mat", L_PARK, c, grip(uy, z0 + 0.010), R_GRIP),
        (1.0, "set it down", L_PARK, c, grip(uy, z0 + 0.002), R_GRIP),
        (0.5, "settle", L_PARK, c, grip(uy, z0 + 0.002), R_GRIP),
        (1.0, "let go", L_PARK, c, grip(uy, z0 + 0.002), o),
        (1.5, "retreat", L_PARK, c, (ux, uy - 0.070, z0 + GRIP_TCP + 0.050), o),
    ]
    return P


def demo():
    """Left: pull the pin while the right holds the unit. Right: lift it to aiming height in front of the bin.
    Left: squeeze the lever. Both: sweep the unit across the bin and back to the middle, aimed at the fuel bed
    in front of the flames. Left lets the lever up; right stands the unit back on the mat and lets go."""
    L, R = [], []
    for dur, label, lp, lf, rp, rf in _demo_phases():
        L.append(Waypoint(dur, tuple(float(v) for v in lp), lf, label, 0.0))
        R.append(Waypoint(dur, tuple(float(v) for v in rp), rf, label, R_YAW))
    return {"left": L, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def _fr(pos=None, lever=LEVER_OPEN, pin=0.0, yaw_deg=0.0, pitch_deg=0.0, tip=None):
    """The extinguisher with its origin at ``pos`` (or its nozzle tip at ``tip``), turned ``yaw_deg`` about z and
    pitched ``pitch_deg`` nose-down, lever at ``lever`` deg, pin pulled ``pin`` m."""
    r = mk.axis_angle((0, 0, 1), math.radians(yaw_deg)) @ mk.axis_angle((0, 1, 0), math.radians(pitch_deg))
    if tip is not None:
        pos = np.asarray(tip, float) - r @ np.asarray(NOZ_TIP)
    pos = (*MAT, EXT_Z) if pos is None else tuple(float(v) for v in pos)
    quat = mk.mat_to_quat(r)
    st = kit.fake_states(q={"lever_hinge": lever, "pin_travel": pin}, deg=True,
                         moved={"extinguisher": (tuple(pos), quat)})
    return st


def _demo_frames():
    """Frames from the demo's own targets: unit at the right TCP minus the grip offset, lever angle from the left
    TCP's height when it is over the lever's end, pin from the left TCP's x while it holds the ring."""
    from yam_arena import station

    plans = {arm: ArmPlan(list(w), station.BASES[arm]) for arm, w in demo().items()}
    ph = _demo_phases()
    ends = np.cumsum([p[0] for p in ph])
    frames, pin = [], 0.0
    t = 0.0
    while t < ends[-1]:
        t += DT
        lp = np.asarray(plans["left"].target(t)[0])
        rp = np.asarray(plans["right"].target(t)[0])
        lf = plans["left"].target(t)[2]
        rf = plans["right"].target(t)[2]
        held = rf > R_GRIP - 0.002
        uo = rp - np.array([0, 0, GRIP_TCP + HANG]) if held else None
        if uo is None or uo[2] < EXT_Z:
            uo = np.array([MAT[0], frames[-1][1][1] if frames else MAT[1], EXT_Z]) if uo is None else \
                np.array([uo[0], uo[1], EXT_Z])
        if lf > L_GRIP - 0.002 and abs(lp[1] - (uo[1] + PIN_Y)) < 0.01:
            pin = max(pin, min(0.045, (uo[0] + RING_X) - lp[0]))
        lev = LEVER_OPEN
        if abs(lp[1] - (uo[1] + HINGE_Y + LEVER_R)) < 0.01 and abs(lp[0] - uo[0]) < 0.02:
            h = lp[2] - TIP_PAST_TCP - uo[2] - LEVER_TOP + PRESS_PAST * 0.5  # the arm stops a few mm short
            lev = float(np.clip(math.degrees(math.asin(np.clip(h / LEVER_R, -1, 1))), 0.0, LEVER_OPEN))
            if pin < PIN_OUT:
                lev = max(lev, 11.0)
        if lev < SQUEEZED:  # the press rolls the held unit (measured in sim): its aim moves right
            uo = uo - np.array([0.0, AIM_DY, 0.0])
        frames.append((_fr(tuple(uo), lever=lev, pin=pin), uo))
    return [f for f, _ in frames]


def self_test():
    """The scoring rules on hand-built sequences of extinguisher poses (one trial each, fed step by step)."""
    global _TEST_TCPS
    za = EXT_Z + LIFT

    def hold(n, **kw):
        return [_fr(**kw) for _ in range(n)]

    def sweep(z=za, x=MAT[0], lever=0.0, pin=0.045, half=SWEEP, passes=2, n=60, tip=None, pitch=0.0):
        out = []
        ys = [BIN_Y - half, BIN_Y + half] * passes + [BIN_Y - half, BIN_Y]
        for a, b in zip(ys[:-1], ys[1:]):
            for k in range(n):
                s = k / n
                s = s * s * (3 - 2 * s)
                y = a + (b - a) * s
                out.append(_fr(pos=(x, y, z), lever=lever, pin=pin) if tip is None else
                           _fr(tip=(tip[0], y, tip[1]), pitch_deg=pitch, lever=lever, pin=pin))
        return out

    start, home = _fr(), _fr(pin=0.045)
    flames_high = za + 0.040  # nozzle 4 cm higher: the ray goes over the flames
    grip_tcp = kit.point(home, "extinguisher", (0, 0, GRIP_TCP))
    cases = {  # name: (frames, TCPs at the end, score)
        "start": ([start], [], 0.0),
        "lifted and put back": ([start, _fr(pos=(*MAT, za)), start], [], 0.0),
        "pin pulled, swept at the base, set back": (hold(3, pin=0.045) + sweep() + [home], [], 1.0),
        "the demo's own path": (_demo_frames() + [home], [], 1.0),
        "same, still held": (hold(3, pin=0.045) + sweep() + [home], [grip_tcp], 0.75),
        "same, left lying on its side": (hold(3, pin=0.045) + sweep() + [_fr(pos=(MAT[0], MAT[1], EXT_Z + 0.03),
                                                                            yaw_deg=0.0, lever=12.0)]
                                         , [], 0.75),
        "squeezed without pulling the pin": (sweep(pin=0.0) + [start], [], 0.0),
        "aimed at the flames": (hold(3, pin=0.045) + sweep(z=flames_high) + [home], [], 0.0),
        "aimed at the middle only": (hold(3, pin=0.045) + sweep(half=0.0, passes=3) + [home], [], 0.25),
        "left and middle thirds only": (hold(3, pin=0.045) + [_fr(pos=(MAT[0], BIN_Y + 0.035, za), lever=0.0, pin=0.045) for _ in range(40)]
         + [_fr(pos=(MAT[0], BIN_Y, za), lever=0.0, pin=0.045) for _ in range(40)] + [home], [], 0.5),
        "too close (nozzle over the rim)": (hold(3, pin=0.045) + sweep(tip=(NEAR_X - 0.045, TOP + 0.155), pitch=25.0) + [home],
                                            [], 0.5),
        "charge wasted on the flames first": (hold(3, pin=0.045) + sweep(z=flames_high, passes=3) + sweep()
                                              + [home], [], 0.0),
        "lever not squeezed far enough (10 deg)": (hold(3, pin=0.045) + sweep(lever=10.0) + [home], [], 0.0),
        "lever left squeezed at the end": (hold(3, pin=0.045) + sweep() + [_fr(lever=0.0, pin=0.045)], [], 0.75),
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
        for f in hold(3, pin=0.045) + sweep():
            score(f)
        mk.new_episode()
        assert score(start) == 0.0, "a new trial forgets the last one"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
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

