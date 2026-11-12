"""On the station table, in front of the right arm, a 110 wiring block lies on the bench with its 4-pair connecting
block facing up (as on the back of a patch panel being terminated on a bench). One blue 24 AWG solid conductor
(0.51 mm copper, 0.9 mm insulation) has been laid across position 5: its run comes from the fanning strip behind the
block and its loose end sticks 20 mm out over the front. The conductor rests in the entry vee of the position's
insulation-displacement contact (IDC): two blade halves sprung together to a 0.45 mm slot, standing 2 mm above the
plastic between ribs. Pushed down 1.8 mm, the conductor spreads the halves and drops into the groove under them,
where the contact grips it.

A non-impact 110 punch-down tool (154 mm long, a 20 x 24 mm handle, 90 g) stands in its holster to the right, keyed
by its blade. The blade is a fork that straddles the contact and pushes the conductor down on both sides of it. On
ONE side it carries a cutting lip that reaches 1.3 mm lower, just outside the block's edge (the orange stripe on the
handle marks that side, as the "CUT" mark on a real blade). In the holster that side faces the run. As the fork
seats the conductor, the lip shears the conductor's outside length down across the block's edge, and the conductor
parts there.

Physics punishes the wrong way round. Used as it comes out of the holster, the lip is over the run: the press seats
the conductor and shears the RUN off at the back edge, leaving the loose end attached. Turned half a turn so the lip
is over the loose end, the same press seats the conductor and trims the waste, and the run stays whole. A fingertip
push can't do it at all: the pads are far wider than the 2 mm channel and land on the ribs.

The job: terminate the conductor (seated in its contact, loose end trimmed, run intact) and lift the tool off.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/punch-down-blade-to-waste-side/punch_down_blade_to_waste_side.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (jointed parts, sprung sliders, friction hinges, breakable bonds, episode memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the mechanism's state every 10 steps
SMOKE = False  # True: a second, mirrored station on the left that the left arm punches WITHOUT turning the tool

# Frame: x forward from the arm bases, y left, z up, metres. Right arm base (0, -0.305, 0.75).
# -- the 110 block ---------------------------------------------------------------------------------------------------
BASE = (0.040, 0.060, 0.012)  # 110 wiring base (x, y, height), white polycarbonate
CB = (0.0064, 0.030)  # connecting block top: depth along the conductor (x) and length along the row (y)
ZT = TOP + 0.022  # top of the connecting block's plastic (the blade bottoms here)
PITCH = 0.00375  # contact pitch (8 positions on 30 mm)
NPOS, ACTIVE = 8, 4  # the conductor is on position 5 (index 4)
GROOVE_W, GROOVE_D = 0.0010, 0.0009  # the conductor groove under each contact
ZF = ZT - GROOVE_D  # groove floor
GAP = 0.00045  # the contact's slot, sprung shut (narrower than the 0.51 mm copper)
JAW_T, JAW_W = 0.0005, 0.0012  # one blade half: thickness (x), width (y)
JAW_UP = 0.0020  # the contact's top above the plastic
RAMP_Z, RAMP_DEG = 0.0008, 25.0  # the entry vee: from 0.8 mm up to the top, 25 deg off vertical
SLIT = 0.00035  # half-width of the slit the active contact works in
JAW_K, JAW_PRE = 1500.0, 2.0  # contact spring (N/m) and its preload (N) per half
JAW_KG = 0.003
RIB_W, RIB_X, RIB_UP = 0.00175, 0.0016, 0.0024  # plastic ribs between positions: width, half-length, wall height
RIB_TIP = RIB_W / 2  # ...each topped by a 90-deg ridge (tip RIB_UP + RIB_TIP above the plastic)
# -- the conductor (24 AWG solid: 0.51 mm copper, 0.9 mm PVC insulation) -----------------------------------------
WIRE_R = 0.00045
WASTE_L = 0.020  # loose end beyond the block's front edge (straight solid copper: one stiff piece)
RUN_NEAR, RUN_FAR = 0.0025, 0.0125  # run: a short piece at the block's back edge, then 12.5 mm to the fanning strip
SLOT_KG, WASTE_KG, RUN_KG = 0.003, 0.003, 0.003  # heavier than real (about 0.1 g): stiff joints in the solver
CUT_N = 15.0  # the joint at each block edge also breaks above this force
SHEAR_MM = 0.45  # ... and parts once the blade has driven the outside length this far down across the edge
BEND_NM = 0.002  # N*m at each run joint: the 0.51 mm copper's plastic bending moment (it stays where it is bent)
SMALL_I = 2e-7  # kg*m^2: rotational inertia given to each wire piece and contact half (PhysX joint stability)
ENTRY_Z = ZT + RAMP_Z + (WIRE_R / math.cos(math.radians(RAMP_DEG)) - GAP / 2) / math.tan(math.radians(RAMP_DEG))
SEATED_Z = ZF + WIRE_R  # conductor centre when seated on the groove floor
# -- the punch-down tool ----------------------------------------------------------------------------------------------
LIP_DROP = 0.0013  # the cutting lip reaches this far below the fork's tips
TOOL_KG = 0.090
HANDLE = (0.020, 0.024, 0.100)  # flat-sided handle (x across the flats, y, length)
SHANK_R, SHANK_L = 0.0035, 0.030
BLADE_TOP = 0.014  # blade length above its tips
BLADE_W = 0.0018  # blade thickness (y): fits the 2.0 mm channel between ribs
NOTCH_IN, NOTCH_OUT, NOTCH_CH = 0.0006, 0.0015, 0.0010  # fork notch half-width above / at the tips; chamfer height
FORK_N, FORK_P, LIP_X = 0.0030, 0.0048, 0.0038  # fork reach on the plain / cutting side; lip's inner face
HANDLE_Z0 = BLADE_TOP + SHANK_L  # handle bottom above the tips
# -- holster (fixed, black ABS): keyed pocket for the blade, square bore for the shank ------------------------------
HOLSTER_H, FLOOR_T = 0.045, 0.003

STEEL, BRASS, WHITE, BLUE = (0.62, 0.64, 0.68), (0.80, 0.66, 0.32), (0.93, 0.93, 0.90), (0.15, 0.35, 0.85)
YELLOW, ORANGE, BLACK, DARK = (0.95, 0.80, 0.10), (0.95, 0.45, 0.08), (0.10, 0.10, 0.11), (0.25, 0.25, 0.27)

kit = mk.Kit()
FIXTURES: list = []
STATIONS: dict = {}  # suffix -> geometry of that station


def _q(axis, rad):
    return tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle(axis, rad)))


def _station(sfx, bx, by, hx, hy):
    """One 110 block with its conductor, and one tool in its holster. Returns the station's geometry."""
    ys = by + (ACTIVE - (NPOS - 1) / 2) * PITCH  # the active groove's centre line
    g = dict(bx=bx, by=by, ys=ys, hx=hx, hy=hy)
    # wiring base + connecting block (fixed): body up to the groove floor, lands between the grooves, ribs between
    # the positions, the other seven contacts as fixed blade pairs
    boxes = [mk.box(BASE, at=(0, 0, BASE[2] / 2 - (ZT - TOP)), color=WHITE),
             mk.box((CB[0], CB[1], ZF - TOP - BASE[2]), at=(0, 0, (ZF + TOP + BASE[2]) / 2 - ZT), color=WHITE)]
    yg = [(i - (NPOS - 1) / 2) * PITCH for i in range(NPOS)]
    edges = [-CB[1] / 2] + [y for c in yg for y in (c - GROOVE_W / 2, c + GROOVE_W / 2)] + [CB[1] / 2]
    zl = -GROOVE_D / 2
    for k in range(NPOS + 1):
        y0, y1 = edges[2 * k], edges[2 * k + 1]
        if k in (ACTIVE, ACTIVE + 1):  # the lands either side of the active groove: split round the contact's slit
            for s in (-1, 1):
                boxes.append(mk.box((CB[0] / 2 - SLIT, y1 - y0, GROOVE_D),
                                    at=(s * (CB[0] / 4 + SLIT / 2), (y0 + y1) / 2, zl), color=WHITE))
        else:
            boxes.append(mk.box((CB[0], y1 - y0, GROOVE_D), at=(0, (y0 + y1) / 2, zl), color=WHITE))
    ribs = [yg[0] - PITCH / 2] + [(yg[i] + yg[i + 1]) / 2 for i in range(NPOS - 1)] + [yg[-1] + PITCH / 2]
    d = RIB_W / math.sqrt(2)  # a square rod turned 45 deg about x: a ridge RIB_W wide
    for y in ribs:
        boxes.append(mk.box((2 * RIB_X, RIB_W, RIB_UP), at=(0, y, RIB_UP / 2), color=WHITE))
        boxes.append(mk.box((2 * RIB_X, d, d), at=(0, y, RIB_UP), quat=_q((1, 0, 0), math.pi / 4), color=WHITE))
    for i, c in enumerate(yg):
        if i == ACTIVE:
            continue
        for s in (-1, 1):
            boxes.append(mk.box((JAW_T, JAW_W, JAW_UP + GROOVE_D),
                                at=(0, c + s * (GAP / 2 + JAW_W / 2), (JAW_UP - GROOVE_D) / 2), color=BRASS))
    kit.part(f"block{sfx}", (bx, by, ZT), boxes, color=WHITE)
    # the active contact: two sprung blade halves with an entry vee
    a = math.radians(RAMP_DEG)
    rl = (JAW_UP - RAMP_Z) / math.cos(a)  # ramp face length
    rt = 0.0006
    for s, nm in ((1, "p"), (-1, "m")):
        cy = s * (math.tan(a) * (JAW_UP - RAMP_Z) / 2 + math.cos(a) * rt / 2)
        cz = RAMP_Z + (JAW_UP - RAMP_Z) / 2 - math.sin(a) * rt / 2
        kit.part(f"jaw_{nm}{sfx}", (bx, ys + s * GAP / 2, ZT), [
            mk.box((JAW_T, JAW_W, RAMP_Z + GROOVE_D - 0.00005), at=(0, s * JAW_W / 2, (RAMP_Z - GROOVE_D + 0.00005) / 2)),
            mk.box((JAW_T, rt, rl), at=(0, cy, cz), quat=_q((1, 0, 0), -s * a)),
        ], mass=JAW_KG, color=BRASS)
        mk.INERTIA[f"jaw_{nm}{sfx}"] = (SMALL_I,) * 3
        kit.slider(f"idc_{nm}{sfx}", f"block{sfx}", f"jaw_{nm}{sfx}", axis=(0, s, 0), lower=0.0, upper=0.0012,
                   stiffness=JAW_K, rest=-JAW_PRE / JAW_K)
    # the conductor: run_far (pivots on the fanning strip) - run_near - slot length - loose end
    z = ENTRY_Z
    xb, xf = bx + CB[0] / 2, bx - CB[0] / 2  # back and front edges of the block
    cap = 2 * WIRE_R * 0.4  # pieces overlap a little at their ends (joined pieces do not collide)
    kit.part(f"run_far{sfx}", (xb + RUN_NEAR + RUN_FAR / 2, ys, z),
             [mk.capsule(WIRE_R, RUN_FAR + cap, axis=(1, 0, 0))], mass=RUN_KG, color=BLUE)
    kit.part(f"run_near{sfx}", (xb + RUN_NEAR / 2, ys, z), [mk.capsule(WIRE_R, RUN_NEAR + cap, axis=(1, 0, 0))],
             mass=RUN_KG, color=BLUE)
    kit.part(f"slot{sfx}", (bx, ys, z), [mk.capsule(WIRE_R, CB[0], axis=(1, 0, 0))], mass=SLOT_KG, color=BLUE)
    kit.part(f"waste{sfx}", (xf - WASTE_L / 2, ys, z), [mk.capsule(WIRE_R, WASTE_L + cap, axis=(1, 0, 0))],
             mass=WASTE_KG, color=BLUE)
    for n in ("run_far", "run_near", "slot", "waste"):
        mk.INERTIA[f"{n}{sfx}"] = (SMALL_I,) * 3
    xs = xb + RUN_NEAR + RUN_FAR
    kit.hinge(f"strip{sfx}", None, f"run_far{sfx}", anchor=(xs, ys, z), axis=(0, 1, 0), lower_deg=-40, upper_deg=40,
              friction=BEND_NM)
    kit.hinge(f"bend{sfx}", f"run_far{sfx}", f"run_near{sfx}", anchor=(xb + RUN_NEAR, ys, z), axis=(0, 1, 0),
              lower_deg=-40, upper_deg=40, friction=BEND_NM)
    kit.bond(f"cut_run{sfx}", f"run_near{sfx}", f"slot{sfx}", break_force=CUT_N, anchor=(xb, ys, z),
             slip_mm=5.0, slip_deg=60.0)
    kit.bond(f"cut_waste{sfx}", f"slot{sfx}", f"waste{sfx}", break_force=CUT_N, anchor=(xf, ys, z),
             slip_mm=5.0, slip_deg=60.0)
    # fanning strip (fixed): a post with a notch the run passes through, 12.5 mm behind the block
    hz = z - WIRE_R - TOP - BASE[2]
    FIXTURES.append(Box(f"fanning{sfx}", pos=(xs + 0.001, ys, TOP + BASE[2] + hz / 2), size=(0.003, 0.010, hz),
                        color=WHITE))
    for s in (-1, 1):
        FIXTURES.append(Box(f"fan_tooth{'np'[s > 0]}{sfx}", pos=(xs + 0.001, ys + s * (WIRE_R + 0.0003 + 0.0015), z),
                            size=(0.003, 0.003, 2 * WIRE_R + 0.002), color=WHITE))
    # the tool, standing in its holster: frame origin = the fork's tip centre, +x = its cutting side
    tz = TOP + FLOOR_T + LIP_DROP
    g["tool_z0"] = tz
    bw = BLADE_W
    ch = math.hypot(NOTCH_OUT - NOTCH_IN, NOTCH_CH)
    cha = math.atan2(NOTCH_OUT - NOTCH_IN, NOTCH_CH)  # chamfer face angle from vertical
    tool = []
    for s, reach in ((-1, FORK_N), (1, FORK_P)):
        tool.append(mk.box((reach - NOTCH_OUT, bw, NOTCH_CH), at=(s * (reach + NOTCH_OUT) / 2, 0, NOTCH_CH / 2)))
        tool.append(mk.box((reach - NOTCH_IN, bw, 0.0024 - NOTCH_CH),
                           at=(s * (reach + NOTCH_IN) / 2, 0, (0.0024 + NOTCH_CH) / 2)))
        # chamfer: a slab whose inner face runs from (NOTCH_OUT, 0) up to (NOTCH_IN, NOTCH_CH)
        t = 0.0006
        mx, mz = s * (NOTCH_OUT + NOTCH_IN) / 2, NOTCH_CH / 2
        nx, nz = s * math.cos(cha), math.sin(cha)  # into the prong
        tool.append(mk.box((t, bw, ch), at=(mx + nx * t / 2, 0, mz + nz * t / 2), quat=_q((0, 1, 0), -s * cha)))
    tool += [
        mk.box((FORK_P - LIP_X, bw, LIP_DROP), at=((FORK_P + LIP_X) / 2, 0, -LIP_DROP / 2)),  # cutting lip
        mk.box((FORK_N + FORK_P, bw, BLADE_TOP - 0.0024), at=((FORK_P - FORK_N) / 2, 0, (BLADE_TOP + 0.0024) / 2)),
        mk.cyl(SHANK_R, SHANK_L, at=(0, 0, BLADE_TOP + SHANK_L / 2)),
        mk.box(HANDLE, at=(0, 0, HANDLE_Z0 + HANDLE[2] / 2), color=YELLOW),
        mk.box((0.0004, 0.006, 0.045), at=(HANDLE[0] / 2 + 0.0001, 0, HANDLE_Z0 + 0.027), color=ORANGE),  # CUT mark
    ]
    kit.part(f"tool{sfx}", (hx, hy, tz), tool, mass=TOOL_KG, color=STEEL)
    # holster (fixed): floor, a keyed blade pocket 0.8 mm larger than the blade, a 9.4 mm square bore above it
    pk = (-FORK_N - 0.0008, FORK_P + 0.0008, -bw / 2 - 0.0008, bw / 2 + 0.0008)  # blade pocket (tool frame)
    bore = 0.0047
    hw = 0.020
    hb = [Box(f"hol_floor{sfx}", pos=(hx, hy, TOP + FLOOR_T / 2), size=(2 * hw, 2 * hw, FLOOR_T), color=BLACK)]
    zb0, zb1 = TOP + FLOOR_T, TOP + FLOOR_T + 0.016  # blade pocket height
    hb += [Box(f"hol_bxn{sfx}", pos=(hx + (pk[0] - hw) / 2, hy, (zb0 + zb1) / 2), size=(hw + pk[0], 2 * hw, zb1 - zb0),
               color=BLACK),
           Box(f"hol_bxp{sfx}", pos=(hx + (pk[1] + hw) / 2, hy, (zb0 + zb1) / 2), size=(hw - pk[1], 2 * hw, zb1 - zb0),
               color=BLACK),
           Box(f"hol_byn{sfx}", pos=(hx + (pk[0] + pk[1]) / 2, hy + (pk[2] - hw) / 2, (zb0 + zb1) / 2),
               size=(pk[1] - pk[0], hw + pk[2], zb1 - zb0), color=BLACK),
           Box(f"hol_byp{sfx}", pos=(hx + (pk[0] + pk[1]) / 2, hy + (pk[3] + hw) / 2, (zb0 + zb1) / 2),
               size=(pk[1] - pk[0], hw - pk[3], zb1 - zb0), color=BLACK)]
    zt1 = TOP + HOLSTER_H
    for s in (-1, 1):
        hb.append(Box(f"hol_tx{'np'[s > 0]}{sfx}", pos=(hx + s * (bore + hw) / 2, hy, (zb1 + zt1) / 2),
                      size=(hw - bore, 2 * hw, zt1 - zb1), color=BLACK))
        hb.append(Box(f"hol_ty{'np'[s > 0]}{sfx}", pos=(hx, hy + s * (bore + hw) / 2, (zb1 + zt1) / 2),
                      size=(2 * bore, hw - bore, zt1 - zb1), color=BLACK))
    FIXTURES.extend(hb)
    STATIONS[sfx] = g
    return g


A = _station("", 0.270, -0.240, 0.205, -0.335)
if SMOKE:
    _station("_b", 0.270, 0.240, 0.205, 0.335)
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.00025  # seated: conductor centre within 0.25 mm of resting on the groove floor
LINE_TOL = 0.0004  # ... within 0.4 mm of the groove's centre line
X_TOL = 0.0015
LEVEL_DEG = 6.0
GRIP_MIN = 0.00005  # each contact half pushed open at least 0.05 mm: it is gripping the conductor
CLEAR_Z = 0.005  # the tool's fork tips at least 5 mm above the block, or 15 mm away from the contact
CLEAR_XY = 0.015
M = "pdbw"
_TEST_TCPS = None


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {}
    return m


def _wire(sfx=""):
    return [f"run_far{sfx}", f"run_near{sfx}", f"slot{sfx}", f"waste{sfx}"]


def jaw_open(objects, nm, sfx="") -> float:
    """How far contact half ``nm`` ("p" or "m") has been pushed open (m), from its pose: the block is fixed, so
    this is the half's sideways travel from where the spring holds it shut."""
    s = 1.0 if nm == "p" else -1.0
    p, _ = kit.pose(objects, f"jaw_{nm}{sfx}")
    return float(s * (p[1] - (STATIONS[sfx]["ys"] + s * GAP / 2)))


def slot_pose(objects, sfx=""):
    """(centre, axis direction) of the conductor's slot length."""
    p, q = kit.pose(objects, f"slot{sfx}")
    return p, mk.quat_to_mat(q)[:, 0]


def seated(objects, sfx="") -> bool:
    """The conductor's slot length lies level on the groove floor, on the groove's line, between the contact's
    halves, and both halves are pushed open on it (the contact grips it)."""
    g = STATIONS[sfx]
    p, ax = slot_pose(objects, sfx)
    level = math.degrees(math.asin(min(1.0, abs(float(ax[2]))))) <= LEVEL_DEG
    on = (abs(p[2] - SEATED_Z) <= SEAT_TOL and abs(p[1] - g["ys"]) <= LINE_TOL and abs(p[0] - g["bx"]) <= X_TOL)
    grip = all(jaw_open(objects, n, sfx) >= GRIP_MIN for n in ("p", "m"))
    return bool(level and on and grip)


_OUTER = {"cut_run": "run_near", "cut_waste": "waste"}


def shear_down(objects, bond, sfx="") -> float:
    """How far (m) the blade has driven the outside piece DOWN across the block edge at joint ``bond``: the joint
    point as carried by the outside piece, minus the same point carried by the in-block length, along the in-block
    length's own down direction. (Measured on a real rig as the step between the two ends at the block's edge.)"""
    j = kit.joints_[bond + sfx]
    a = np.r_[j.anchor, 1.0]
    ts, to = kit._pose_T(objects, f"slot{sfx}"), kit._pose_T(objects, _OUTER[bond] + sfx)
    ps = ts @ mk._inv(kit.parts[f"slot{sfx}"].ref) @ a
    po = to @ mk._inv(kit.parts[_OUTER[bond] + sfx].ref) @ a
    return float(-(ts[:3, :3].T @ (po[:3] - ps[:3]))[2])


def parted(objects, bond, sfx="") -> bool:
    """The conductor has parted at this block edge: sheared SHEAR_MM down across it, now or earlier in the trial."""
    key = ("parted", bond + sfx)
    m = _mem()
    if not m.get(key) and shear_down(objects, bond, sfx) >= SHEAR_MM * 1e-3:
        m[key] = True
    return bool(m.get(key))


def trimmed(objects, sfx="") -> bool:
    """The loose end is sheared off at the block's front edge (latched)."""
    return parted(objects, "cut_waste", sfx)


def run_cut(objects, sfx="") -> bool:
    """The run has been sheared off at the block's back edge (latched for the trial)."""
    return parted(objects, "cut_run", sfx)


def tool_clear(objects, sfx="") -> bool:
    g = STATIONS[sfx]
    tip = kit.point(objects, f"tool{sfx}", (0, 0, 0))
    return bool(tip[2] - ZT >= CLEAR_Z or math.hypot(tip[0] - g["bx"], tip[1] - g["ys"]) >= CLEAR_XY)


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


STILL_M, STILL_DEG = 0.0005, 1.5  # still: no conductor piece moved more than this since the previous control step


def _kept(objects, sfx=""):
    """The pieces of the terminated conductor: the run and the in-block length, plus the loose end until it is
    trimmed off (the cut-off scrap may fall and roll anywhere; it is waste)."""
    return [n for n in _wire(sfx) if not (n == f"waste{sfx}" and trimmed(objects, sfx))]


def wire_still(objects) -> bool:
    """No piece of the terminated conductor has moved more than 0.5 mm or 1.5 deg across the poses stored at the last six score
    calls (at least three stored). Pose-based, so it does not depend on how often score() is called or on the
    reported velocities of these small pieces, which carry solver noise while they sit clamped in the contact."""
    m = _mem()
    cur = {n: kit._pose_T(objects, n) for n in _wire()}
    kept = _kept(objects)
    hist = m.setdefault("hist", [])
    hist.append(cur)
    del hist[:-6]
    if len(hist) < 3:
        return False
    return all(float(np.linalg.norm(cur[n][:3, 3] - h[n][:3, 3])) < STILL_M
               and math.degrees(mk._rot_angle(cur[n][:3, :3] @ h[n][:3, :3].T)) < STILL_DEG
               for h in hist for n in kept)


def _log_state(objects, parts) -> None:
    """DEBUG (sim): one JSON line per score() call with every input of the last 0.2 of the score."""
    if not DEBUG or mk._EP.get("env") is None:
        return
    import json

    rec = {"step": mk._EP.get("step"), **parts,
           "tcps": [[round(float(v), 4) for v in t] for t in mk.tcps()],
           "tip": [round(float(v), 4) for v in kit.point(objects, "tool", (0, 0, 0))]}
    for n in _wire():
        st = objects[n]
        rec[n] = {"p": [round(float(v), 5) for v in st.pos], "v": round(float(np.linalg.norm(st.lin_vel)), 4),
                  "w": round(float(np.linalg.norm(st.ang_vel)), 3), "lo": [round(float(v), 4) for v in st.low],
                  "hi": [round(float(v), 4) for v in st.high]}
    with open(str(Path.home() / "dbg" / "pdbw_state.jsonl"), "a") as f:
        f.write(json.dumps(rec) + "\n")


def score(objects) -> float:
    """0 once the run has been sheared off (latched for the trial). Otherwise 0.5 for the conductor seated in its
    contact; with that, +0.3 for the loose end trimmed off; with either, +0.2 for the tool lifted off the block,
    the grippers 2.5 cm clear of the conductor and the conductor still."""
    still = wire_still(objects)  # every call: it keeps the pose history
    cut, seat = run_cut(objects), seated(objects)
    trim, clear = trimmed(objects), tool_clear(objects)
    rel = all(_released(objects[n]) for n in _kept(objects))
    _log_state(objects, dict(still=still, run_cut=cut, seated=seat, trimmed=trim, tool_clear=clear, released=rel))
    if cut or not seat:
        return 0.0
    base = 0.5 + (0.3 if trim else 0.0)
    if clear and rel and still:
        base += 0.2
    return round(base, 6)


def success(objects) -> bool:
    """Seated, trimmed, run intact, tool lifted off, still; held for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Stages still to do (seat, trim, lift off) plus the tool tips' distance from the contact (m); 9 if the run is cut."""
    if run_cut(objects):
        return 9.0
    g = STATIONS[""]
    tip = kit.point(objects, "tool", (0, 0, 0))
    left = (not seated(objects)) + (not trimmed(objects)) + (not tool_clear(objects))
    return float(left + math.dist(tuple(tip), (g["bx"], g["ys"], ZT)))


# -- debug readout (sim, DEBUG only) ---------------------------------------------------------------------------------


def _debug(objects) -> None:
    if not DEBUG or mk._EP.get("env") is None or (mk._EP.get("step", 0) % 10 and mk._EP.get("step", 0) > 3):
        return
    out = [f"step {mk._EP['step']}"]
    for sfx in STATIONS:
        p, ax = slot_pose(objects, sfx)
        tp, tq = kit.pose(objects, f"tool{sfx}")
        tr = mk.quat_to_mat(tq)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(tr[2, 2])))))
        yaw = math.degrees(math.atan2(tr[1, 0], tr[0, 0]))
        w1 = kit.point(objects, f"waste{sfx}", (-WASTE_L / 2, 0, 0))  # its inner end, at the block's edge
        r1 = kit.pose(objects, f"run_near{sfx}")[0]
        g = STATIONS[sfx]
        out.append(f"[{sfx or 'A'}] slot dz {(p[2] - SEATED_Z) * 1e3:.2f} dy {(p[1] - g['ys']) * 1e3:.2f} "
                   f"dx {(p[0] - g['bx']) * 1e3:.2f} tilt {math.degrees(math.asin(min(1, abs(ax[2])))):.1f}; "
                   f"jaws {jaw_open(objects, 'p', sfx) * 1e3:.2f}/{jaw_open(objects, 'm', sfx) * 1e3:.2f}; "
                   f"w1 dz {(w1[2] - SEATED_Z) * 1e3:.2f} r1 dz {(r1[2] - SEATED_Z) * 1e3:.2f}; "
                   f"shW {shear_down(objects, 'cut_waste', sfx) * 1e3:.2f} shR {shear_down(objects, 'cut_run', sfx) * 1e3:.2f} "
                   f"cutW {trimmed(objects, sfx)} cutR {run_cut(objects, sfx)}; "
                   f"tool tip ({(tp[0] - g['bx']) * 1e3:.1f}, {(tp[1] - g['ys']) * 1e3:.1f}, {(tp[2] - ZT) * 1e3:.2f}) "
                   f"tilt {tilt:.1f} yaw {yaw:.1f}")
    tc = mk.tcps()
    if tc:
        out.append("tcp R " + ",".join(f"{v:.4f}" for v in tc[1]) + " L " + ",".join(f"{v:.4f}" for v in tc[0]))
    try:
        base = mk._EP["env"].unwrapped
        for key in ("right_robot", "left_robot"):
            d = base.scene[key].data
            jp = getattr(d.joint_pos, "torch", d.joint_pos)[0][:6].detach().cpu().numpy()
            jt = getattr(d.joint_pos_target, "torch", d.joint_pos_target)[0][:6].detach().cpu().numpy()
            out.append(f"{key[0]} dq " + ",".join(f"{v:.4f}" for v in (jt - jp)) + " q " + ",".join(f"{v:.3f}" for v in jp))
    except Exception as e:  # noqa: BLE001
        out.append(f"(joints: {e!r})")
    print("[pdbw] " + " | ".join(out), flush=True)


def _fracture(objects) -> None:
    """Sim: a block-edge joint that the blade has driven SHEAR_MM down across the edge parts for good (the scorer
    reads the same latch); PhysX's force break (CUT_N) is kept as well."""
    for sfx in STATIONS:
        for b in ("cut_run", "cut_waste"):
            name = b + sfx
            if name not in kit._released and parted(objects, b, sfx):
                kit.release(name)
                if DEBUG:
                    print(f"[pdbw] step {mk._EP.get('step')}: {name} PARTED", flush=True)


kit.on_step(_fracture)
kit.on_step(_debug)

# -- the scripted demo -----------------------------------------------------------------------------------------------
# Open loop from the scene's geometry; right arm only. Fingers wide open over the tool, down round the handle 15 mm
# below its top, fully closed (the handle must not slip under the press), straight up out of the holster, carried
# over the block while the wrist turns the tool half a turn (yaw -90 -> 0 -> +90 deg) so the cutting lip faces the
# loose end, lowered until the fork is just over the conductor, then pressed: the target goes 60 mm below where the
# fork bottoms, so the arm's joint springs push about 40 N; then up, tool held clear.

GRASP_DZ = 0.130  # TCP height above the table when gripping the handle in the holster
TOFF = GRASP_DZ - (STATIONS[""]["tool_z0"] - TOP)  # fork tips below the TCP
LIFT_DZ = 0.185  # TCP height while carrying (tips 60 mm up)
PRESS_OVER = 0.060  # commanded this far below the fork's bottomed height
TRIM = {"": (0.0010, -0.0009), "_b": (0.0028, -0.0017)}  # set-down correction of the fork (x, y), from sim runs
YAW_TRIM = {"": 0.0, "_b": 0.0}
PRESS_DX = 0.004  # the press target leads 4 mm in x: under load the TCP sags back toward the arm
YAW_PICK = -math.pi / 2
PARK_R = (0.200, -0.250, TOP + 0.200)
PARK_L = (0.200, 0.250, TOP + 0.200)


def _punch_plan(sfx, park, turn: bool):
    g = STATIONS[sfx]
    c, o = FINGER_CLOSED, FINGER_OPEN
    W = [Waypoint(2.0, park, o, "start", YAW_PICK)]

    def w(dur, pos, f, label, yaw):
        W.append(Waypoint(dur, tuple(float(v) for v in pos), f, label, yaw))

    hx, hy = g["hx"], g["hy"]
    yp = YAW_PICK
    w(2.0, (hx, hy, TOP + GRASP_DZ + 0.040), o, "over the tool, fingers open", yp)
    w(1.5, (hx, hy, TOP + GRASP_DZ), o, "down round the handle", yp)
    w(0.8, (hx, hy, TOP + GRASP_DZ), c, "grip the handle", yp)
    w(0.5, (hx, hy, TOP + GRASP_DZ), c, "settle", yp)
    w(2.0, (hx, hy, TOP + LIFT_DZ), c, "lift it out of the holster", yp)
    px, py = g["bx"] + TRIM[sfx][0], g["ys"] + TRIM[sfx][1]
    y_mid = 0.0 if turn else yp
    y_end = (-yp if turn else yp) + YAW_TRIM[sfx]
    w(2.0, ((hx + px) / 2, (hy + py) / 2, TOP + LIFT_DZ), c, "carry, turning", y_mid)
    w(2.0, (px, py, TOP + LIFT_DZ), c, "over the block, cutting side to the loose end" if turn else "over the block",
      y_end)
    zb = ZT + TOFF  # TCP height with the fork bottomed on the block
    w(2.0, (px, py, zb + 0.008), c, "lower over the contact", y_end)
    w(1.5, (px, py, zb + 0.005), c, "fork just over the ribs", y_end)
    w(4.0, (px + PRESS_DX, py, zb - PRESS_OVER), c, "press", y_end)
    w(1.0, (px + PRESS_DX, py, zb - PRESS_OVER), c, "hold", y_end)
    w(1.5, (px, py, zb + 0.002), c, "ease off", y_end)
    w(1.5, (px, py, zb + 0.040), c, "lift the tool off", y_end)
    w(3.0, (px, py, zb + 0.040), c, "hold it clear", y_end)
    return W


def demo():
    """Pick the tool out of its holster, turn it half a turn so the cutting lip faces the loose end, punch the
    conductor down over its contact, lift the tool off and hold it clear."""
    right = _punch_plan("", PARK_R, turn=True)
    total = sum(x.duration for x in right)
    if SMOKE:  # the naive way on the mirrored station: same press, tool used as it comes out of the holster
        left = _punch_plan("_b", PARK_L, turn=False)
        lt = sum(x.duration for x in left)
        if lt < total:
            left.append(Waypoint(total - lt, left[-1].pos, left[-1].finger, "wait", left[-1].yaw))
    else:
        left = [Waypoint(2.0, PARK_L, FINGER_OPEN, "park"), Waypoint(total - 2.0, PARK_L, FINGER_OPEN, "park")]
    return {"left": left, "right": right}


# -- offline self-test -----------------------------------------------------------------------------------------------


def _rot_y(deg):
    return tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.radians(deg))))


def self_test():
    """The scoring rules on hand-built sequences of poses (one trial each, fed step by step)."""
    global _TEST_TCPS
    g = STATIONS[""]
    up = (0.0, 0.0, 0.0, 1.0)
    turned = tuple(float(v) for v in mk.mat_to_quat(mk.axis_angle((0, 0, 1), math.pi)))
    drop = ENTRY_Z - SEATED_Z
    tool0 = ((g["hx"], g["hy"], g["tool_z0"]), up)

    def st(slot_dz=0.0, jaws=0.0, cut=(), tool=None, slot_dy=0.0, slot_tilt=0.0):
        """The slot length moved down slot_dz (its run and waste follow rigidly unless a bond is cut, in which case
        the cut side stays where it was); jaws pushed open by ``jaws``."""
        ts = mk._T(mk.quat_to_mat(_rot_y(slot_tilt)), (g["bx"], g["ys"] + slot_dy, ENTRY_Z - slot_dz))
        moved = {"slot": (ts[:3, 3], _rot_y(slot_tilt))}
        r1 = ts @ mk._inv(kit.parts["slot"].ref) @ kit.parts["run_near"].ref  # the run's first piece, bonded rigidly
        moved["run_near"] = (r1[:3, 3], tuple(mk.mat_to_quat(r1[:3, :3])))
        if "cut_waste" in cut:  # loose end sheared: pushed 1.3 mm further down than the slot length
            moved["waste"] = ((g["bx"] - CB[0] / 2 - WASTE_L / 2, g["ys"], ENTRY_Z - slot_dz - LIP_DROP - 0.0006), up)
        if "run_up" in cut:
            moved["run_near"] = ((moved["run_near"][0][0], moved["run_near"][0][1], moved["run_near"][0][2] + 0.0008), up)
        if "waste_04" in cut:
            moved["waste"] = ((g["bx"] - CB[0] / 2 - WASTE_L / 2, g["ys"], ENTRY_Z - slot_dz - 0.0004), up)
        if "cut_run" in cut:
            moved["run_near"] = ((g["bx"] + CB[0] / 2 + RUN_NEAR / 2, g["ys"], ENTRY_Z - slot_dz - LIP_DROP - 0.0006), up)
        moved["tool"] = tool or tool0
        for nm, sg in (("p", 1.0), ("m", -1.0)):
            moved[f"jaw_{nm}"] = ((g["bx"], g["ys"] + sg * (GAP / 2 + jaws), ZT), up)
        return kit.fake_states(moved=moved)

    def scrap(k):  # the demo's end state with the trimmed loose end rolling across the table
        f = st(drop, spread, ("cut_waste",), lifted)
        o = dict(f)
        o.update(kit.fake_states(moved={"waste": ((g["bx"] - 0.020, g["ys"] + 0.010 * (k + 1), TOP + WIRE_R), up)}))
        o.update({n: f[n] for n in f if n != "waste"})
        return o

    def lin(a, b, n=8):
        return [a + (b - a) * k / n for k in range(n + 1)]

    on_block = ((g["bx"], g["ys"], ZT), turned)
    lifted = ((g["bx"], g["ys"], ZT + 0.040), turned)
    spread = (WIRE_R * 2 - GAP) / 2
    press = [st(d, jaws=spread * min(1.0, d / drop), tool=((g["bx"], g["ys"], ENTRY_Z + WIRE_R - d), turned))
             for d in lin(0.0, drop)]
    good = press + [st(drop, spread, ("cut_waste",), on_block), st(drop, spread, ("cut_waste",), lifted)]
    naive = press + [st(drop, spread, ("cut_run",), on_block), st(drop, spread, ("cut_run",), lifted)]
    no_trim = press + [st(drop, spread, (), lifted)]
    half = [st(d, spread * 0.6) for d in lin(0.0, drop * 0.5)] + [st(drop * 0.5, spread * 0.6, ("cut_waste",), lifted)]
    near_tcp = (g["bx"], g["ys"], ZT + 0.010)
    cases = {  # name: (frames, TCPs at the end, score)
        "start": ([st()], [], 0.0),
        "seated, loose end trimmed, run whole, tool lifted off (demo)": (good, [], 1.0),
        "the same with the tool still bottomed on the block": (good[:-1], [], 0.8),
        "the same but a gripper 1 cm over the conductor": (good, [near_tcp], 0.8),
        "naive: tool used as it came out, the RUN sheared, loose end on": (naive, [], 0.0),
        "run sheared early, then everything else right": (press[:3] + [st(drop * 0.3, 0, ("cut_run",))] + good,
                                                         [], 0.0),
        "seated but nothing trimmed (light press), tool lifted": (no_trim, [], 0.7),
        "pushed half way, loose end sheared, not seated": (half, [], 0.0),
        "seated, trimmed, then pulled 1 mm back up out of the groove":
            (good + [st(drop - 0.001, spread, ("cut_waste",), lifted)], [], 0.0),
        "seated pose but the contact not gripping it (halves shut)": (good[:-2] + [st(drop, 0.0, ("cut_waste",), lifted)],
                                                                      [], 0.0),
        "lying in the groove 0.6 mm off the contact's line": (good + [st(drop, spread, ("cut_waste",), lifted, 0.0006)],
                                                            [], 0.0),
        "seated but tilted 10 deg": (good + [st(drop, spread, ("cut_waste",), lifted, 0.0, 10.0)], [], 0.0),
        "nothing done, tool left in the holster": ([st()] * 3, [], 0.0),
        "the cut-off scrap rolling away on the table at the end (it is waste)": (good + [scrap(k) for k in range(6)],
                                                                                [], 1.0),
        "run piece jerked 0.8 mm UP at the back edge (not a blade cut), then the demo end":
            (press + [st(drop, spread, ("run_up",), on_block)] + good[-2:], [], 1.0),
        "loose end bent down only 0.4 mm at the edge, nothing parted, tool lifted": (press + [st(drop, spread, ("waste_04",), lifted)], [], 0.7),
    }
    try:
        for name, (frames, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for f in frames[:-1]:
                score(f)
            _TEST_TCPS = tcps
            for _ in range(6):  # the end state held (stillness reads the last six scored poses)
                got = score(frames[-1])
            assert abs(got - want) < 1e-9, (name, got, want, _mem())
            assert success(frames[-1]) == (want >= 1.0), name
        mk.new_episode()
        _TEST_TCPS = []
        for f in naive:
            score(f)
        mk.new_episode()
        for _ in range(5):  # (stillness needs the end state held)
            score(good[-1])
        assert score(good[-1]) == 1.0 and not run_cut(good[-1]), "a new trial forgets the last one"
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

    n = self_test()
    print(f"scoring: {n} cases pass (conductor drops {(ENTRY_Z - SEATED_Z) * 1e3:.2f} mm from the entry vee to seated; "
          f"fork tips {TOFF * 1e3:.1f} mm below the TCP)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
