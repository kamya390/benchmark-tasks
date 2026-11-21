"""Lay three limp bacon rashers flat on a quarter sheet pan with spring tongs.

A deli tray butts against a quarter sheet pan.  Three limp rashers (250 x 35 x 3 mm silicone stand-ins, each a
chain of six hinged links with the sheet's bending stiffness) lie shingled in the tray with their near ends resting
over the two rims and sticking 2 cm out over the pan.  A pair of 24 cm spring tongs (two rigid arms on a sprung
hinge) lies on a stand behind the pan.  The robot must take the tongs, pick each rasher by its end with them, and
lay it flat on the pan floor, side by side, none folded or overlapping, then put the tongs back and let go.
Nothing is scripted about the rashers: they drape, drag, fold and overlap through contact physics only.
All scoring logic lives in ``scoring.py`` (no simulator needed); this file builds the scene and feeds poses to it.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, allow_matrix_grips, as_numpy, body_state, hinge, on_reset,  # noqa: E402
                     on_step, rotation)
import scoring  # noqa: E402
from yam_arena.constants import FINGER_OPEN, READY, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Lay the bacon out on the sheet pan for the oven, using the tongs."
STAGE = os.environ.get("BACON_STAGE", "")     # "", "droop" (stand-in calibration), "tongs" (take and return only),
#                                               "naive" (rasher 3 only, lowered straight down: it folds)
DEBUG = bool(os.environ.get("BACON_DEBUG"))

# -- layout (world, m) --------------------------------------------------------------------------------------------
PAN_C = np.array([0.315, 0.050])              # centre of the pan
FLOOR_T = 0.003                               # pan floor thickness: its top face is the scoring frame's z = 0
PAN_ORIGIN = np.array([PAN_C[0], PAN_C[1], TOP + FLOOR_T])
RIM_H = 0.025                                 # pan rim and tray rim: 25 mm above the bench
RIM_W = 0.015                                 # pan rim (sloped wall and rolled edge) modelled as a 15 mm wide wall
F = PAN_C[1] - scoring.PAN_HALF[1]            # y of the pan floor's edge nearest the tray
TRAY_WALL = 0.005
TRAY_IN = (0.150, 0.220)                      # tray inside, x by y
TRAY_NEAR_OUT = F - RIM_W - 0.002             # tray's near wall outer face: 2 mm from the pan
TRAY_C = np.array([PAN_C[0], TRAY_NEAR_OUT - TRAY_WALL - TRAY_IN[1] / 2])
TRAY_FLOOR_T = 0.005

# -- rashers: 250 x 35 x 3 mm, 30 g; six links; hinge stiffness matched to a 3.2 mm 40A silicone sheet ------------
N_LINKS = 6
R_LEN, R_W, R_T, R_MASS = scoring.RASHER_LEN, 0.035, 0.003, 0.030
LINK = R_LEN / N_LINKS
R_SPRING = 1.4e-4       # N m per degree (8e-3 N m/rad): see README "Stand-in calibration"
R_DAMP = 1.0e-5         # N m s per degree
R_MU = float(os.environ.get("BACON_MU", "1.0"))   # rasher friction (static; dynamic 0.1 less): sensitivity runs only
OVERHANG = 0.020        # the rasher ends start 20 mm out over the pan floor
SHINGLE = 0.022         # each rasher 22 mm to the side of the one below it
R_X = [PAN_C[0] - SHINGLE, PAN_C[0], PAN_C[0] + SHINGLE]
LANES = [PAN_C[0] - 0.060, PAN_C[0], PAN_C[0] + 0.060]      # where each one is laid (demo)
BACON = [(0.78, 0.30, 0.27), (0.80, 0.34, 0.30), (0.74, 0.27, 0.25)]
FAT = (0.95, 0.86, 0.80)

# -- tongs: two 240 x 16 mm arms (3 mm collision thickness) on a sprung hinge that opens them 15 degrees ----------
T_LEN, T_W, T_T = 0.240, 0.016, 0.003
T_HALF_GAP = 0.004      # each arm's mid-plane is 4 mm off the hinge line
PAD = (0.016, 0.012, 0.0025)
PAD_Y = -(T_LEN - 0.006)
T_OPEN = 15.0
T_SPRING = 0.020        # N m per degree: about 3.5 N at the grip point to close them
T_GRIP = 0.100          # the gripper holds the arms 100 mm from the hinge
T_TIP = -PAD_Y - T_GRIP  # grip point to jaw pads
STAND_H = 0.075
HINGE0 = np.array([0.475, 0.250, TOP + STAND_H + T_HALF_GAP + T_T / 2])
STEEL = (0.78, 0.79, 0.80)
ALU = (0.72, 0.73, 0.74)
BLACK = (0.10, 0.10, 0.11)


def _pan() -> Body:
    hx, hy = scoring.PAN_HALF
    shapes = [BoxShape((0, 0, FLOOR_T / 2), (2 * hx + 2 * RIM_W, 2 * hy + 2 * RIM_W, FLOOR_T), ALU)]
    for sx in (-1, 1):
        shapes.append(BoxShape((sx * (hx + RIM_W / 2), 0, RIM_H / 2), (RIM_W, 2 * hy + 2 * RIM_W, RIM_H), ALU))
    for sy in (-1, 1):
        shapes.append(BoxShape((0, sy * (hy + RIM_W / 2), RIM_H / 2), (2 * hx, RIM_W, RIM_H), ALU))
    return Body.build("sheet_pan", (PAN_C[0], PAN_C[1], TOP), 1.0, shapes, color=ALU, kinematic=True,
                      friction=(0.8, 0.7))


def _tray() -> Body:
    ix, iy = TRAY_IN
    ox, oy = ix + 2 * TRAY_WALL, iy + 2 * TRAY_WALL
    shapes = [BoxShape((0, 0, TRAY_FLOOR_T / 2), (ox, oy, TRAY_FLOOR_T), BLACK)]
    for sx in (-1, 1):
        shapes.append(BoxShape((sx * (ix + TRAY_WALL) / 2, 0, RIM_H / 2), (TRAY_WALL, oy, RIM_H), BLACK))
    for sy in (-1, 1):
        shapes.append(BoxShape((0, sy * (iy + TRAY_WALL) / 2, RIM_H / 2), (ix, TRAY_WALL, RIM_H), BLACK))
    return Body.build("deli_tray", (TRAY_C[0], TRAY_C[1], TOP), 1.0, shapes, color=BLACK, kinematic=True,
                      friction=(0.8, 0.7))


def _stand() -> Body:
    """Two 75 mm posts with side lips: the lower tong arm lies across them, clear of the bench for a side grip."""
    shapes = [BoxShape((0, -0.105, 0.003), (0.060, 0.250, 0.006), BLACK)]
    for y in (-0.020, -0.205):
        shapes.append(BoxShape((0, y, STAND_H / 2), (0.022, 0.012, STAND_H), BLACK))
        for sx in (-1, 1):
            shapes.append(BoxShape((sx * 0.0125, y, STAND_H + 0.004), (0.003, 0.012, 0.008), BLACK))
    return Body.build("tongs_stand", (HINGE0[0], HINGE0[1], TOP), 1.0, shapes, color=BLACK, kinematic=True,
                      friction=(0.8, 0.7))


def _arm(name: str, sign: float, **kw) -> Body:
    """One tong arm, its origin on the hinge line; ``sign`` -1: the lower arm, +1: the upper."""
    z = sign * T_HALF_GAP
    shapes = [BoxShape((0, -T_LEN / 2, z), (T_W, T_LEN, T_T), STEEL),
              BoxShape((0, PAD_Y, z - sign * (T_T + PAD[2]) / 2), PAD, STEEL),
              BoxShape((0, -0.004, z / 2), (T_W, 0.008, T_HALF_GAP), STEEL)]
    return Body.build(name, HINGE0, 0.045, shapes, color=STEEL, friction=(1.0, 0.9), damping=(0.3, 2.0),
                      solver_iterations=48, **kw)


def _quat_x(deg: float):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _rasher(i: int, tail_y: float, z: float, x: float) -> list[Body]:
    links = []
    for k in range(N_LINKS):
        shapes = [BoxShape((0, 0, 0), (R_W, LINK, R_T), BACON[i]),
                  BoxShape((0.008, 0, R_T / 2), (0.006, LINK, 0.0004), FAT, collide=False),
                  BoxShape((-0.011, 0, R_T / 2), (0.004, LINK, 0.0004), FAT, collide=False)]
        links.append(Body.build(f"rasher{i + 1}_link{k + 1}", (x, tail_y + (k + 0.5) * LINK, z), R_MASS / N_LINKS,
                                shapes, color=BACON[i], friction=(R_MU, R_MU - 0.1), damping=(0.3, 2.0),
                                solver_iterations=48))
        if k:
            hinge(f"rasher{i + 1}_hinge{k}", links[k - 1].name, links[k].name, (x, tail_y + k * LINK, z), axis="X",
                  limits_deg=(-150.0, 150.0), spring=R_SPRING, damping=R_DAMP)
    return links


PAN, TRAY, STAND = _pan(), _tray(), _stand()
ARM_LO = _arm("tongs_lower_arm", -1.0)
ARM_UP = _arm("tongs_upper_arm", +1.0, start_xyzw=_quat_x(-T_OPEN))
hinge("tongs_hinge", ARM_LO.name, ARM_UP.name, tuple(HINGE0), axis="X", limits_deg=(-T_OPEN - 0.5, 0.0),
      spring=T_SPRING, damping=2.0e-4, target_deg=-T_OPEN - 3.0)

if STAGE == "droop":
    # Calibration: one rasher on a 60 mm block with exactly two links (83 mm) overhanging its edge.
    BLOCK = Body.build("cal_block", (0.30, -0.10, TOP), 1.0, [BoxShape((0, 0, 0.03), (0.10, 0.20, 0.06), ALU)],
                       color=ALU, kinematic=True, friction=(0.8, 0.7))
    RASHERS = [_rasher(0, -0.10 + 0.10 + 2 * LINK - R_LEN, TOP + 0.06 + R_T / 2 + 0.001, 0.30)]
    OBJECTS = [BLOCK, *RASHERS[0], STAND, ARM_LO, ARM_UP]
else:
    RASHERS = [_rasher(i, F + OVERHANG - R_LEN, TOP + RIM_H + R_T / 2 + 0.002 + 0.006 * i, R_X[i])
               for i in range(scoring.N_RASHERS)]
    OBJECTS = [PAN, TRAY, STAND, *[b for r in RASHERS for b in r], ARM_LO, ARM_UP]

HALF = np.array([R_W / 2, LINK / 2, R_T / 2])
JAW_LO = np.array([0.0, PAD_Y, 0.0])      # pad faces meet on the hinge plane when the tongs are shut
ARM_PTS = [np.array([0.0, -y, 0.0]) for y in (0.0, 0.06, 0.12, 0.18, 0.24)]

EP = {"tracker": scoring.Tracker(), "score": 0.0, "step": 0}


@on_reset
def _reset_episode():
    EP.update(tracker=scoring.Tracker(), score=0.0, step=0)


def _gripper_points(env):
    """Per arm: the centre point (TCP) and the two finger pads, each also 13 mm further out at the pad tips."""
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    tcps, pts = [], []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        pose = as_numpy(robot.data.body_link_pose_w)[0][i]
        rot = rotation(pose[3:7])
        tcp = pose[:3] - origin + rot @ np.asarray(TCP_OFFSET)
        s = abs(float(as_numpy(robot.data.joint_pos)[0][6]))
        tcps.append(tcp)
        for dx in (0.0, s, -s):
            for dz in (0.0, 0.013):
                pts.append(tcp + rot @ np.array([dx, 0.0, dz]))
    return np.array(tcps), np.array(pts)


@on_step
def _follow(env):
    EP["step"] += 1
    states = [[body_state(env, b) for b in r] for r in RASHERS]
    lo, up = body_state(env, ARM_LO), body_state(env, ARM_UP)
    tcps, pts = _gripper_points(env)
    rl, ru = lo.rotation(), up.rotation()
    jaws = np.array([lo.pos + rl @ JAW_LO, up.pos + ru @ JAW_LO])
    tong_pts = np.array([lo.pos + rl @ p for p in ARM_PTS] + [up.pos + ru @ p for p in ARM_PTS])
    tip0 = HINGE0 + [0.0, PAD_Y, -T_HALF_GAP]
    rest_err = max(float(np.linalg.norm(lo.pos - HINGE0)),
                   float(np.linalg.norm(lo.pos + rl @ np.array([0.0, PAD_Y, -T_HALF_GAP]) - tip0)))
    still = all(s.resting() for r in states for s in r) and lo.resting() and up.resting()
    obs = scoring.Obs(
        rashers=[[(s.pos - PAN_ORIGIN, s.rotation(), HALF) for s in r] for r in states],
        gripper_points=pts - PAN_ORIGIN, jaws=jaws - PAN_ORIGIN, tcps=tcps - PAN_ORIGIN,
        tongs_points=tong_pts - PAN_ORIGIN, tongs_rest_err=rest_err, still=still, dt=1.0 / 20.0)
    EP["score"] = EP["tracker"].update(obs)
    if DEBUG and EP["step"] % 20 == 0:
        mm = lambda v: np.round((np.asarray(v) - [0, 0, TOP]) * 1000, 1).tolist()  # noqa: E731
        rr = env.unwrapped.scene["right_robot"]
        print("[bacon]", {
            "step": EP["step"], "score": EP["score"], **EP["tracker"].detail,
            "tcp_r": mm(tcps[1]), "finger": round(float(as_numpy(rr.data.joint_pos)[0][6]), 4),
            "hinge": mm(lo.pos), "jaws": [mm(j) for j in jaws],
            "open_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, float(rl[:, 1] @ ru[:, 1]))))), 1),
            "ends": [[mm(r[0].pos), mm(r[-1].pos)] for r in states],
            "top_mm": [round(max(float(scoring.corners((s.pos - PAN_ORIGIN, s.rotation(), HALF))[:, 2].max())
                                 for s in r) * 1000, 1) for r in states],
            "end_edge_mm": [mm(r[-1].pos + r[-1].rotation() @ np.array([0.0, LINK / 2, 0.0])) for r in states],
            "len_mm": [round(scoring.end_to_end([(s.pos, s.rotation(), HALF) for s in r]) * 1000, 1) for r in states],
            "moving": [n for n, s in [(b.name, s) for r, ss in zip(RASHERS, states) for b, s in zip(r, ss)]
                       + [("lo", lo), ("up", up)] if not s.resting()][:4],
        }, flush=True)


def score(objects) -> float:
    return EP["score"]


def success(objects) -> bool:
    return EP["score"] == 1.0


# -- scripted demo ------------------------------------------------------------------------------------------------

def _frame(theta_deg: float) -> np.ndarray:
    """TCP rotation for the tongs pointing along -y, pitched ``theta`` down: the gripper reaches forward (+x) with
    its fingers one above the other, so its wrist roll alone sets the tongs' pitch."""
    th = math.radians(theta_deg)
    t = np.array([0.0, -math.cos(th), -math.sin(th)])
    a = np.array([1.0, 0.0, 0.0])
    return np.column_stack([np.cross(t, a), t, a])


def _tcp(b, theta_deg: float) -> np.ndarray:
    """TCP that puts the point midway between the jaw pads at ``b``."""
    return np.asarray(b, float) - T_TIP * _frame(theta_deg)[:, 1]


def _ik(pos, rot, seed) -> np.ndarray:
    from yam_arena import ik, station

    for s in (seed, READY, [1.0, 2.4, 1.6, 0.8, 1.0, -1.5]):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(s, float),
                             base_pos=station.BASES["right"], iters=300)
        if pe < 0.001 and re < math.radians(0.5):
            return q
    raise ValueError(f"no IK solution at {np.round(pos, 3)}")


def _joint_path(q0, q1, duration: float, finger: float, label: str) -> list[Waypoint]:
    from yam_arena import ik, station

    n = max(15, int(round(duration * 20)))
    out = []
    for k in range(1, n + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / n, station.BASES["right"])
        out.append(Waypoint(duration / n, tuple(float(v) for v in p), finger, label, r))
    return out


# Finger joint targets (m): pads 2 x |q| apart.
F_WIDE = -0.030     # to pass over and under the open tongs on their stand
F_HOLD = -0.0150    # tongs held about 11 degrees open
F_PINCH = -0.0040   # tongs shut on a rasher (the pads stop at about 6 mm)
PICK_PITCH = 30.0
LAY_PITCH = 68.0
# Measured in the debug run: where each rasher's end settles (x of the strip, z of its underside at the end).
PICK_X = [R_X[0], R_X[1] + 0.0065, R_X[2] + 0.0065]
LAY_X = [LANES[0], LANES[1] + 0.0065, LANES[2] + 0.0065]


def demo():
    """Right arm only: take the tongs, lay rashers 3, 2, 1 (top of the shingle first), put the tongs back."""
    th0 = -T_OPEN / 2
    grip0 = HINGE0 + T_GRIP * _frame(th0)[:, 1]
    r0 = _frame(th0)
    pre = grip0 + [-0.095, 0.0, 0.0]
    q_pre = _ik(pre, r0, [0.99, 2.76, 1.9, 0.85, 0.99, -1.70])
    right = _joint_path(np.asarray(READY, float), q_pre, 4.0, F_WIDE, "reach beside the tongs")

    def go(t, p, finger, label, theta):
        right.append(Waypoint(t, tuple(float(v) for v in p), finger, label, _frame(theta)))

    go(1.0, pre, F_WIDE, "settle beside the tongs", th0)
    go(1.5, grip0, F_WIDE, "slide the fingers over and under the tongs", th0)
    go(1.0, grip0, F_HOLD, "take the tongs", th0)
    go(1.0, grip0 + [0, 0, 0.045], F_HOLD, "lift the tongs off their stand", th0)

    order = [] if STAGE in ("tongs", "droop") else [2] if STAGE == "naive" else [2, 1, 0]
    for i in order:
        n = i + 1
        # Lower jaw pad 4 mm under the last centimetre of the end that sticks out over the pan.
        pad = np.array([PICK_X[i], F + OVERHANG - 0.008, TOP + RIM_H - 0.004])
        xg = _frame(PICK_PITCH)[:, 0]
        half_open = (T_LEN - 0.006) * math.sin(math.radians(5.5))
        b_pick = pad + half_open * xg
        t30 = _frame(PICK_PITCH)[:, 1]
        go(3.0, _tcp(b_pick - 0.05 * t30 + [0, 0, 0.03], PICK_PITCH), F_HOLD, f"carry the tongs to rasher {n}",
           PICK_PITCH)
        go(1.0, _tcp(b_pick - 0.05 * t30, PICK_PITCH), F_HOLD, f"line the jaws up with rasher {n}'s end", PICK_PITCH)
        go(1.5, _tcp(b_pick, PICK_PITCH), F_HOLD, f"slide the lower jaw under rasher {n}'s end", PICK_PITCH)
        go(1.2, _tcp(b_pick, PICK_PITCH), F_PINCH, f"pinch rasher {n}'s end", PICK_PITCH)
        b = b_pick
        if STAGE == "naive":   # carry it over the lane, then just lower the held end straight down and let go
            for dt, dy, z in ((1.5, 0.030, 0.090), (2.5, 0.100, 0.130), (2.5, 0.150, 0.200), (3.0, 0.150, 0.030)):
                b = np.array([LAY_X[i], F + dy, TOP + z])
                go(dt, _tcp(b, 35.0), F_PINCH, "naive: lower the held end straight down", 35.0)
            go(0.8, _tcp(b, 35.0), F_HOLD, "naive: open the tongs", 35.0)
            go(1.5, _tcp(b + [0, 0.03, 0.06], 35.0), F_HOLD, "naive: lift the tongs clear", 35.0)
            continue
        for dt, dy, z, th, x, label in (
                (1.5, 0.030, 0.090, 30.0, PICK_X[i], f"peel rasher {n} up off the stack"),
                (2.5, 0.100, 0.130, 35.0, LAY_X[i], f"draw rasher {n} out over the pan"),
                (2.5, 0.200, 0.120, 45.0, LAY_X[i], f"trail rasher {n} along its lane"),
                (3.0, 0.250, 0.018, LAY_PITCH, LAY_X[i], f"lay rasher {n} down, trailing it straight")):
            b = np.array([x, F + dy, TOP + z])
            go(dt, _tcp(b, th), F_PINCH, label, th)
        # Open, then keep going the same way, level, so the lower jaw slides out from under the end instead of
        # lifting it.
        go(0.8, _tcp(b, LAY_PITCH), F_HOLD, f"open the tongs on rasher {n}", LAY_PITCH)
        b = b + [0, 0.035, 0]
        go(1.5, _tcp(b, LAY_PITCH), F_HOLD, f"slide the lower jaw out from under rasher {n}", LAY_PITCH)
        go(1.5, _tcp(b + [0, -0.03, 0.09], 40.0), F_HOLD, "lift the tongs clear", 40.0)

    go(3.0, grip0 + [0, 0, 0.045], F_HOLD, "carry the tongs back over their stand", th0)
    go(1.5, grip0 + [0, 0, 0.002], F_HOLD, "lower the tongs onto the stand", th0)
    go(1.0, grip0 + [0, 0, 0.002], F_WIDE, "let go of the tongs", th0)
    go(1.5, pre, F_WIDE, "slide the fingers back out", th0)
    go(1.5, pre + [-0.05, -0.05, 0.10], F_WIDE, "move clear", th0)
    go(3.0, pre + [-0.05, -0.05, 0.10], F_WIDE, "hold still", th0)
    # The left arm is omitted, so the scripted policy holds it at its ready posture.
    return {"right": right}
