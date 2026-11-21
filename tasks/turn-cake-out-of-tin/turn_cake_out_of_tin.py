"""Turn a cake out of its tin onto a plate: plate on top, turn both over together, lift the tin off.

A 20 cm round cake tin (5 cm deep, tapered wall, rolled rim) holds a snug 0.5 kg cake that sticks lightly to it.  The
tin stands on a fixed 11 cm round, 14 cm tall stand; a flat 25 cm plate lies face-up on a second, identical stand
nearer the robot.  The plate has to be laid face-down on the tin's rim, plate and tin clamped together by both
grippers and turned over as one, the pair set back down plate-side, and the tin lifted straight off and put on the
free stand, leaving the cake centred on the plate.  The cake is never held: while the pair turns, only the clamp
keeps it between tin and plate.  If the cake reaches the bench the episode scores 0.
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
from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, body_state, glue,  # noqa: E402
                     on_reset, on_step, plate as wall_panel, rotation, set_glue_on)
import scoring  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Turn the cake out of its tin onto the plate."
STAGE = os.environ.get("CAKE_STAGE", "")      # "", "clamp" (plate already on the tin) or "lift" (pair already turned)

H, TIN_H, PLATE_T, CAKE_H = scoring.STAND_H, scoring.TIN_H, scoring.PLATE_T, scoring.CAKE_H
MAIN = np.array([0.450, 0.000])               # stand under the tin
SIDE = np.array([0.200, 0.000])               # stand under the plate; the empty tin ends up here
STAND_R = 0.055

ALU = (0.78, 0.79, 0.81)
SPONGE = (0.86, 0.63, 0.30)
PARCHMENT = (0.95, 0.93, 0.86)
WHITE = (0.96, 0.96, 0.94)
WELL = (0.74, 0.84, 0.93)
WOOD = (0.62, 0.45, 0.28)
CORK = (0.70, 0.55, 0.38)
FLIP_XYZW = (1.0, 0.0, 0.0, 0.0)              # half a turn about x


def _stand(name: str, xy) -> Body:
    return Body.build(name, (xy[0], xy[1], TOP), 1.0, [CylShape((0, 0, H / 2), STAND_R, H, WOOD)], color=WOOD,
                      kinematic=True, friction=(0.9, 0.8))


def _tin(pos, flipped: bool) -> Body:
    """Floor disc, 24 tapered wall panels (2.5 mm of draft) and a rolled rim drawn as 24 short bars (visual only)."""
    n, wall = 24, scoring.TIN_WALL
    rb, rt = scoring.TIN_R_FLOOR + wall / 2, scoring.TIN_R_RIM + wall / 2
    width = 2 * (rt + wall / 2) * math.tan(math.pi / n) + 0.001
    shapes = [CylShape((0, 0, scoring.TIN_FLOOR / 2), scoring.TIN_R_FLOOR + wall, scoring.TIN_FLOOR, ALU)]
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        shapes.append(wall_panel((rb * c, rb * s, 0.001), (rt * c, rt * s, TIN_H), width, wall, (c, s, 0.0), ALU))
        shapes.append(BoxShape((0.1025 * c, 0.1025 * s, TIN_H - 0.0015), (0.003, width + 0.001, 0.003), ALU, yaw=a,
                               collide=False))    # the rolled rim's bead: visual only
    return Body.build("tin", pos, 0.25, shapes, color=ALU, friction=(1.0, 0.9), damping=(0.2, 1.0),
                      solver_iterations=48, com=(0.0, 0.0, 0.018), start_xyzw=FLIP_XYZW if flipped else None)


def _cake(pos, flipped: bool) -> Body:
    """Three stacked discs following the tin's taper (1.5 mm clear all round) and a visible parchment base."""
    shapes = [CylShape((0, 0, 0.0075), 0.0960, 0.015, SPONGE),
              CylShape((0, 0, 0.0225), 0.0968, 0.015, SPONGE),
              CylShape((0, 0, 0.0370), scoring.CAKE_R, 0.014, SPONGE),
              CylShape((0, 0, -0.0002), 0.0940, 0.0006, PARCHMENT, collide=False)]
    return Body.build("cake", pos, scoring.CAKE_MASS, shapes, color=SPONGE, friction=(0.7, 0.6), damping=(0.2, 1.0),
                      solver_iterations=48, com=(0.0, 0.0, CAKE_H / 2), start_xyzw=FLIP_XYZW if flipped else None)


def _plate(pos, flipped: bool) -> Body:
    shapes = [CylShape((0, 0, PLATE_T / 2), scoring.PLATE_R, PLATE_T, WHITE),
              CylShape((0, 0, PLATE_T + 0.0002), 0.095, 0.0006, WELL, collide=False),      # eating face
              CylShape((0, 0, -0.0002), 0.070, 0.0006, (0.80, 0.80, 0.78), collide=False)]  # foot mark underneath
    return Body.build("plate", pos, 0.28, shapes, color=WHITE, friction=(0.9, 0.8), damping=(0.2, 1.0),
                      solver_iterations=48, start_xyzw=FLIP_XYZW if flipped else None)


def _at(xy, z):
    return np.array([xy[0], xy[1], TOP + z])


if STAGE == "lift":      # the pair already turned over and set down on the main stand
    TIN = _tin(_at(MAIN, H + PLATE_T + TIN_H + 0.0005), True)
    CAKE = _cake(_at(MAIN, H + PLATE_T + CAKE_H + 0.0003), True)
    PLATE = _plate(_at(MAIN, H + 0.0002), False)
else:
    TIN = _tin(_at(MAIN, H + 0.0005), False)
    CAKE = _cake(_at(MAIN, H + 0.0005 + scoring.TIN_FLOOR + 0.0005), False)
    PLATE = (_plate(_at(MAIN, H + TIN_H + PLATE_T + 0.001), True) if STAGE == "clamp"
             else _plate(_at(SIDE, H + 0.0005), False))
    # The cake's light sticking to its tin: a fixed joint the scorer lets go once the cake's weight pulls on it
    # with more than scoring.STICK_N (the tin tipped past about 128 degrees).
    glue("cake_stick", "tin", "cake", tuple(CAKE.pos))

OBJECTS = [_stand("tin_stand", MAIN), _stand("plate_stand", SIDE), TIN, CAKE, PLATE]

TRACKER = scoring.Tracker(SIDE, TOP, stand_xy=MAIN)
_DBG = [0]


@on_reset
def _reset_episode():
    TRACKER.reset(stuck=STAGE != "lift", plate_on_tin=STAGE == "lift", pair_inverted=STAGE == "lift")
    _DBG[0] = 0


def _arm_points(env):
    """TCP and finger-pad positions of both arms (task frame), and each arm's finger joint."""
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    tcps, pads, fingers = [], [], []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        tcps.append(poses[i, :3] - origin + rotation(poses[i, 3:7]) @ np.asarray(TCP_OFFSET))
        pads += [poses[k, :3] - origin for k, n in enumerate(robot.body_names)
                 if "tip_left" in n or "tip_right" in n]
        fingers.append(float(as_numpy(robot.data.joint_pos)[0][6]))
    return tcps, pads, fingers


@on_step
def _follow(env):
    tin, cake, plate = body_state(env, TIN), body_state(env, CAKE), body_state(env, PLATE)
    tcps, pads, fingers = _arm_points(env)
    still = bool(tin.resting() and cake.resting() and plate.resting())
    cmd = TRACKER.update(scoring.Frame(np.asarray(tin.pos), tin.rotation(), np.asarray(cake.pos), cake.rotation(),
                                       np.asarray(plate.pos), plate.rotation(), tcps + pads, still))
    if cmd.release_stick and STAGE != "lift":
        set_glue_on("cake_stick", False)
    _DBG[0] += 1
    if os.environ.get("CAKE_DEBUG") and _DBG[0] % 20 == 0:
        e = TRACKER.evidence
        print("[cake]", {"step": _DBG[0],
                         "tin_mm": np.round(np.asarray(tin.pos) * 1000, 1).tolist(),
                         "tin_axis": np.round(tin.rotation()[:, 2], 3).tolist(),
                         "cake_mm": np.round(np.asarray(cake.pos) * 1000, 1).tolist(),
                         "cake_axis": np.round(cake.rotation()[:, 2], 3).tolist(),
                         "plate_mm": np.round(np.asarray(plate.pos) * 1000, 1).tolist(),
                         "plate_axis": np.round(plate.rotation()[:, 2], 3).tolist(),
                         "tcps_mm": [np.round(t * 1000, 1).tolist() for t in tcps],
                         "fingers": [round(f, 4) for f in fingers],
                         "flags": [k for k, v in e.__dict__.items() if v], "detail": TRACKER.detail,
                         "score": TRACKER.score()}, flush=True)


def score(objects=None) -> float:
    return TRACKER.score()


def task_score(*args, **kwargs) -> float:
    return TRACKER.score()


def success(objects=None) -> bool:
    return score() == 1.0


# -- scripted demo ------------------------------------------------------------------------------------------------

def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _frame(approach, closing):
    z, x = np.asarray(approach, float), np.asarray(closing, float)
    return np.column_stack([x, np.cross(z, x), z])


_IK_CACHE: dict = {}


def _good_ik(arm: str, pos, rot, hint=None) -> np.ndarray:
    """A roomy IK solution (far from joint limits), tried from a few fixed seeds; ``hint`` is tried first."""
    from yam_arena import ik, station

    key = (arm, tuple(np.round(pos, 5)), tuple(np.round(rot, 5).ravel()))
    if key in _IK_CACHE:
        return _IK_CACHE[key]
    rng = np.random.default_rng(1)
    seeds = [np.asarray(READY, float)] + [ik.LOWER + (ik.UPPER - ik.LOWER) * rng.random(6) for _ in range(40)]
    if hint is not None:
        seeds.insert(0, np.asarray(hint, float))
    best = None
    for seed in seeds:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), seed,
                             base_pos=station.BASES[arm], iters=80)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
            if room > 0.3 or (hint is not None and seed is seeds[0]):
                break
    if best is None:
        raise ValueError(f"no IK solution for {arm} at {np.round(pos, 3)}")
    _IK_CACHE[key] = best[0]
    return best[0]


class _Arm:
    """One arm's waypoint list, with Cartesian moves, joint-space transits that start from the joints the scripted
    controller will really be in (its IK is warm-started from its own last command), and holds."""

    def __init__(self, name: str):
        from yam_arena import ik, station
        from yam_arena.demo import ArmPlan

        self.name, self.way = name, []
        self.base = station.BASES[name]
        self.plan, self.k = ArmPlan(self.way, self.base), 0
        self.pos, self.rot = ik.tcp_pose(np.asarray(READY, float), self.base)
        self.finger = FINGER_OPEN

    @property
    def t(self) -> float:
        return sum(w.duration for w in self.way)

    def q(self) -> np.ndarray:
        while self.k / 20.0 < self.plan.duration:
            self.plan.action7(self.k / 20.0)
            self.k += 1
        return self.plan.q.copy()

    def go(self, t, pos, finger, label, rot=None):
        self.pos = np.asarray(pos, float)
        self.rot = self.rot if rot is None else np.asarray(rot, float)
        self.finger = finger
        self.way.append(Waypoint(t, tuple(float(v) for v in self.pos), finger, label, self.rot))

    def hold(self, t, label, finger=None):
        if t > 1e-6:
            self.go(t, self.pos, self.finger if finger is None else finger, label)

    def joints(self, q1, t, finger, label):
        from yam_arena import ik

        q0, q1 = self.q(), np.asarray(q1, float)
        n = max(15, int(round(t * 20)))
        for k in range(1, n + 1):
            s = k / n
            s = s * s * (3 - 2 * s)
            p, r = ik.tcp_pose(q0 + (q1 - q0) * s, self.base)
            f = self.finger + (finger - self.finger) * k / n
            self.way.append(Waypoint(t / n, tuple(float(v) for v in p), f, label, r))
        self.pos, self.rot, self.finger = np.asarray(p, float), r, finger

    def to(self, pos, rot, t, finger, label, hint=None):
        self.joints(_good_ik(self.name, pos, rot, hint), t, finger, label)
        self.go(0.4, pos, finger, label, rot)


def _sync(a: _Arm, b: _Arm, label="wait for the other arm"):
    d = a.t - b.t
    (b if d > 0 else a).hold(abs(d), label)


# Demo constants.  Both grippers stay horizontal, pointing at each other along y, for the whole demo; at this
# station that pose only reaches |y| <= 0.118, so the fingers enter and leave sideways (along x), never outward.
OPEN = FINGER_OPEN
PLATE_GAP = -0.022                    # finger joint for a 44 mm opening around the 8 mm plate
EDGE_Y = 0.118                        # TCP |y| holding the plate by the 2 cm of rim that overhangs the tin
CLAMP_Y = 0.085                       # TCP |y| clamping plate and tin together
CLAMP_Z = TOP + H + (TIN_H + PLATE_T) / 2
PLATE_Z = TOP + H + PLATE_T / 2
ROLL_Z = TOP + H + float(os.environ.get("CAKE_ROLL_LIFT", "0.150"))     # the turning plate clears its stand
ON_TIN_Z = TOP + H + TIN_H + PLATE_T / 2 + float(os.environ.get("CAKE_PLATE_DROP", "0.003"))
FLIP_X = float(os.environ.get("CAKE_FLIP_X", "0.325"))                  # the pair is turned over between the stands
FLIP_Z = CLAMP_Z + float(os.environ.get("CAKE_FLIP_LIFT", "0.095"))
SQ_Z = TOP + H + PLATE_T + TIN_H - 0.012                                 # closed finger tips 12 mm below the tin's top
SQ_TOUCH = 0.111                                                          # TCP |y| when the tips meet the wall (measured)
SQ_N = float(os.environ.get("CAKE_SQ_N", "16"))                           # squeeze force per side, N (approximate)
SQ_ARC = float(os.environ.get("CAKE_SQ_ARC", "0.0"))                     # x drift per m of overlap (measured)
SQ_RISE = float(os.environ.get("CAKE_SQ_RISE", "5.0"))                    # x drift per m of overlap per m of lift (measured)
SQ_F = float(os.environ.get("CAKE_SQ_F", "-0.020"))                    # fingers 40 mm apart while squeezing
SQ_LIFT = float(os.environ.get("CAKE_SQ_LIFT", "0.062"))
SQ_DX = float(os.environ.get("CAKE_SQ_DX", "-0.004"))                        # where the turned pair really lands (measured)
REST_DOWN = float(os.environ.get("CAKE_REST_DOWN", "0.045"))
STOP = os.environ.get("CAKE_STOP", "")


def demo():
    """Both arms mirror each other throughout, grippers horizontal and facing each other along y.

    1. Pinch the plate's edge from both sides, lift it, roll both wrists half a turn so it faces down, carry it
       over the tin and lower it onto the rim.
    2. Open, drop to the tin's mid-height, slide one finger over the plate and one under the tin, clamp, lift,
       carry the pair between the stands, roll both wrists back half a turn together, carry it back, set it down.
    3. Open, slide out sideways, close the fingers, press their tips on opposite sides of the tin, lift it straight
       off the cake, carry it to the free stand, let go.
    Open-loop waypoints from the known start layout; it reads no simulator state."""
    left, right = _Arm("left"), _Arm("right")
    lrot, rrot = _frame((0, -1, 0), (0, 0, 1)), _frame((0, 1, 0), (0, 0, -1))

    def both(t, x, y, z, finger, label, phi=0.0):
        left.go(t, (x, y, z), finger, label, _ry(phi) @ lrot)
        right.go(t, (x, -y, z), finger, label, _ry(phi) @ rrot)

    def lead(x, y, z, finger, label):
        left.to((x, y, z), lrot, 4.0, finger, label)
        right.to((x, -y, z), rrot, 4.0, finger, label)
        _sync(left, right)

    def roll(x, y, z, a0, a1, label, steps=12, dt=0.55):
        for k in range(1, steps + 1):
            both(dt, x, y, z, FINGER_CLOSED, label, a0 + (a1 - a0) * k / steps)

    ax, bx = MAIN[0], SIDE[0]
    phi = 0.0
    if STAGE == "":
        enter = bx + 0.100
        lead(enter, EDGE_Y, PLATE_Z + 0.060, PLATE_GAP, "bring both grippers level, facing each other")
        both(1.5, enter, EDGE_Y, PLATE_Z, PLATE_GAP, "lower to the plate's height")
        both(2.0, bx, EDGE_Y, PLATE_Z, PLATE_GAP, "slide a finger over and under the plate's edge on each side")
        both(0.8, bx, EDGE_Y, PLATE_Z, FINGER_CLOSED, "pinch the plate's edge on both sides")
        both(2.0, bx, EDGE_Y, ROLL_Z, FINGER_CLOSED, "lift the plate")
        roll(bx, EDGE_Y, ROLL_Z, 0.0, -math.pi, "turn the plate face-down")
        phi = -math.pi
        both(3.0, ax, EDGE_Y, ON_TIN_Z + 0.030, FINGER_CLOSED, "carry the plate over the tin", phi)
        both(1.5, ax, EDGE_Y, ON_TIN_Z, FINGER_CLOSED, "lower the plate onto the tin's rim", phi)
        both(0.8, ax, EDGE_Y, ON_TIN_Z, OPEN, "let go of the plate", phi)
        if STOP == "plate":
            both(3.0, ax, EDGE_Y, ON_TIN_Z, OPEN, "hold", phi)
            return {"left": left.way, "right": right.way}
    if STAGE == "clamp":
        phi = -math.pi
        left.to((ax, EDGE_Y, ON_TIN_Z), _ry(phi) @ lrot, 4.0, OPEN, "bring both grippers level with the plate")
        right.to((ax, -EDGE_Y, ON_TIN_Z), _ry(phi) @ rrot, 4.0, OPEN, "bring both grippers level with the plate")
        _sync(left, right)
    if STAGE != "lift":
        both(1.2, ax, EDGE_Y, CLAMP_Z, OPEN, "drop to the middle of plate and tin", phi)
        both(1.5, ax, CLAMP_Y, CLAMP_Z, OPEN, "slide one finger over the plate and one under the tin", phi)
        both(1.0, ax, CLAMP_Y, CLAMP_Z, FINGER_CLOSED, "clamp plate and tin together", phi)
        both(0.5, ax, CLAMP_Y, CLAMP_Z, FINGER_CLOSED, "hold the clamp", phi)
        both(1.5, ax, CLAMP_Y, CLAMP_Z + 0.020, FINGER_CLOSED, "lift the clamped pair off the stand", phi)
        both(3.0, FLIP_X, CLAMP_Y, FLIP_Z, FINGER_CLOSED, "carry the pair between the stands", phi)
        roll(FLIP_X, CLAMP_Y, FLIP_Z, -math.pi, 0.0, "turn the pair over together")
        phi = 0.0
        both(1.0, FLIP_X, CLAMP_Y, FLIP_Z, FINGER_CLOSED, "steady the turned pair")
        both(3.0, ax, CLAMP_Y, CLAMP_Z + 0.020, FINGER_CLOSED, "carry the pair back over its stand")
        both(1.5, ax, CLAMP_Y, CLAMP_Z + 0.002, FINGER_CLOSED, "set the pair down on the stand")
        both(0.8, ax, CLAMP_Y, CLAMP_Z + 0.002, OPEN, "open both clamps")
        if STOP == "flip":
            both(3.0, ax, CLAMP_Y, CLAMP_Z + 0.002, OPEN, "hold")
            return {"left": left.way, "right": right.way}
        out = ax - 0.135
        both(2.5, out, CLAMP_Y, CLAMP_Z + 0.002, OPEN, "slide the fingers out sideways")
    else:
        out = ax - 0.135
        lead(out, CLAMP_Y, SQ_Z, OPEN, "bring both grippers level, facing each other")
    # Fingers side by side (wrists rolled a quarter turn) and 40 mm apart: two fingertips on each side of the tin, so
    # it cannot swing about the line between the grippers.
    # The squeeze is the arms' own springiness: the TCPs are commanded past the wall and the joint drives push back.
    # The arm is stiffer close in (shorter lever about the shoulder), so the commanded overlap shrinks with x to keep
    # the force near SQ_N; the small x offsets cancel the arc the deflected arm swings through.
    q, f = -math.pi / 2, SQ_F

    def press(t, x, z, label):
        d = SQ_N * x * x / 80.0
        both(t, x - SQ_ARC * d - SQ_RISE * d * (z - SQ_Z), SQ_TOUCH - d, z, f, label, q)

    both(2.0, out, EDGE_Y, SQ_Z, f, "turn the fingers side by side at the height of the tin's wall", q)
    sx = ax + SQ_DX
    both(2.5, sx, EDGE_Y, SQ_Z, f, "bring two fingertips to each side of the tin", q)
    press(1.5, sx, SQ_Z, "press in on both sides of the tin")
    press(0.5, sx, SQ_Z, "hold the squeeze")
    press(3.0, sx, SQ_Z + SQ_LIFT, "lift the tin straight off the cake")
    for k in range(1, 6):
        press(0.9, sx + (bx - sx) * k / 5, SQ_Z + SQ_LIFT, "carry the tin to the free stand")
    # The upturned tin is wider than the stand, so it comes to rest over it like a cap; lower it as far as the arms
    # reach at this stand before letting go.
    rest_z = SQ_Z - PLATE_T - REST_DOWN
    press(3.0, bx, rest_z, "lower the tin over the free stand")
    both(1.0, bx, EDGE_Y, rest_z, f, "let go of the tin", q)
    both(2.0, bx, EDGE_Y, rest_z + 0.090, f, "rise clear", q)
    both(2.0, bx, EDGE_Y, rest_z + 0.090, f, "hold clear", q)
    return {"left": left.way, "right": right.way}
