"""On the station table, in front of the right arm, is the infeed of a bottle capper, stopped and locked out: a
stainless machine deck with a blue four-pocket starwheel turning on a vertical shaft, a curved stainless guide
rail round the far side of the wheel, and white chevron plates on the deck beside the guide showing the direction
the bottles travel (the wheel turns clockwise seen from above). The guide is wide where bottles come in and closes
in on the wheel along the flow. One white square bottle is jammed in a pocket, pinched between the pocket's
trailing lobe and the guide. A red reject bin stands on the table to the right of the deck, and a grey isolator
with a red padlock and a yellow tag sits on the deck's far corner. The job is to clear the jam: take the bottle
out of the starwheel and put it in the reject bin.

The pinch is a bond between the bottle and the deck that holds against 6 N and lets go once the wheel has been
turned back (anticlockwise, against the flow) by 6 degrees, which opens the pocket. The wheel turns on a friction
bearing (0.06 N*m), so it stays where it is left. A bottle pulled out of the pinch before the wheel is backed off
(it moves 3 mm or tips 5 deg), or a wheel turned 3 deg forward with the flow, scores 0 for the rest of the trial.

Mechanism parts come from mechkit.py (in this folder), which adds hinges and bonds to the stock runtime in memory.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/starwheel-jam-back-off/starwheel_jam_back_off.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinge, bond and per-episode hooks, installed in memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The right arm's base is at (0, -0.305, 0.75).
DECK = (0.290, 0.290, 0.004)  # stainless machine deck
DECK_XY = (0.315, -0.115)
DECK_TOP = TOP + DECK[2]

# The starwheel: four lobes (UHMW-PE) on a hub, turning about a vertical shaft. Angles are measured anticlockwise
# from +x seen from above; the flow turns the wheel clockwise.
WX, WY = 0.300, -0.130  # shaft axis
WHEEL_Z0 = DECK_TOP + 0.030  # underside of the wheel (it rides on its shaft, 3 cm above the deck)
WHEEL_T = 0.015
LOBE_R = 0.080  # lobe tip radius
LOBE_W = 0.016
LOBES_DEG = (-90.0, 0.0, 90.0, 180.0)
HUB = 0.040
WHEEL_KG = 0.25
WHEEL_FRICTION = 0.06  # N*m bearing friction: the wheel stays where it is left
WHEEL_LIMIT = 40.0

# The bottle: a 40 mm French square (HDPE), filled, standing in the pocket between the 0 and 90 deg lobes, its outer
# face against the guide and its side 1.5 mm from the trailing (90 deg) lobe.
BOTTLE = (0.040, 0.040, 0.110)
NECK = (0.020, 0.020, 0.016)
CAP = (0.026, 0.026, 0.014)
BOTTLE_H = BOTTLE[2] + NECK[2] + CAP[2]
BOTTLE_KG = 0.150
POCKET_DEG = 60.0  # the jammed bottle's angle about the shaft (and its yaw: square to the pocket)
POCKET_R = 0.074  # its centre's distance from the shaft
_pa = math.radians(POCKET_DEG)
JAM_XY = (WX + POCKET_R * math.cos(_pa), WY + POCKET_R * math.sin(_pa))
PINCH_N = 6.0  # the pinch holds the bottle against this pull

# Guide rail (stainless, 8 mm thick, 50 mm tall): straight segments round the far side, square to the bottle at the
# jam (1.5 mm off its outer face), opening out upstream and closing in on the lobe tips downstream.
GUIDE_T, GUIDE_H, GUIDE_SEG = 0.008, 0.050, 0.038
GUIDE = {18.0: 0.0860, 39.0: 0.0975, 60.0: 0.0955, 81.0: 0.1010, 102.0: 0.1065, 123.0: 0.1120}  # deg: inner r

BIN = (0.120, 0.120, 0.060)  # red polypropylene reject bin, 4 mm walls, 3 mm floor
BIN_XY = (0.190, -0.330)
BIN_WALL, BIN_FLOOR = 0.004, 0.003
BIN_FLOOR_TOP = TOP + BIN_FLOOR

BACK_DEG = 6.0  # the pocket opens (the pinch lets go) once the wheel is backed off this far
FWD_DEG = 3.0  # a wheel turned this far forward (with the flow) is a violation
MOVE_MM = 3.0  # the bottle counts as pulled once it moves this far from its jam pose ...
TILT_DEG = 5.0  # ... or tips this far, before the wheel is backed off
OUT_M = 0.050  # out of the starwheel: its centre at least 5 cm from the jam pose
UPRIGHT_DEG = 15.0

BLUE = (0.15, 0.35, 0.75)
STEEL = (0.70, 0.71, 0.73)
DARK = (0.30, 0.31, 0.33)
WHITE = (0.93, 0.93, 0.92)
CAP_C = (0.20, 0.45, 0.80)
RED = (0.80, 0.12, 0.10)
YELLOW = (0.95, 0.80, 0.10)

kit = mk.Kit()
kit.part("shaft", (WX, WY, DECK_TOP), [mk.box((0.020, 0.020, 0.030), at=(0, 0, 0.015))], color=DARK)
kit.part("starwheel", (WX, WY, WHEEL_Z0), [
    mk.box((HUB, HUB, WHEEL_T), at=(0, 0, WHEEL_T / 2)),
    *(mk.box((LOBE_R, LOBE_W, WHEEL_T), at=(LOBE_R / 2 * math.cos(math.radians(a)),
                                              LOBE_R / 2 * math.sin(math.radians(a)), WHEEL_T / 2), yaw=math.radians(a))
      for a in LOBES_DEG),
    # white index mark on the -90 deg lobe (read against the deck's scale on a real rig)
    mk.box((0.004, 0.030, 0.001), at=(0.0, -0.055, WHEEL_T + 0.0005), color=WHITE),
], mass=WHEEL_KG, color=BLUE)
# axis -z: a positive angle is clockwise seen from above, the way the flow drives the wheel
kit.hinge("wheel", "shaft", "starwheel", anchor=(WX, WY, WHEEL_Z0), axis=(0, 0, -1), lower_deg=-WHEEL_LIMIT,
          upper_deg=WHEEL_LIMIT, friction=WHEEL_FRICTION)
kit.part("bottle", (*JAM_XY, DECK_TOP), [
    mk.box(BOTTLE, at=(0, 0, BOTTLE[2] / 2)),
    mk.box(NECK, at=(0, 0, BOTTLE[2] + NECK[2] / 2)),
    mk.box(CAP, at=(0, 0, BOTTLE[2] + NECK[2] + CAP[2] / 2), color=CAP_C),
], mass=BOTTLE_KG, color=WHITE, yaw=_pa)
kit.bond("pinch", None, "bottle", break_force=PINCH_N, anchor=(*JAM_XY, DECK_TOP))


def _open_pocket(objects) -> None:
    """The pinch lets go once the wheel is backed off 6 deg (sim only; runs every control step)."""
    if "pinch" not in kit._released and kit.q_deg(objects, "wheel") <= -BACK_DEG:
        kit.release("pinch")


kit.on_step(_open_pocket)


def _guide():
    out = []
    for i, (deg, r) in enumerate(GUIDE.items()):
        a = math.radians(deg)
        rc = r + GUIDE_T / 2
        out.append(Box(f"guide_{i}", pos=(WX + rc * math.cos(a), WY + rc * math.sin(a), DECK_TOP + GUIDE_H / 2),
                       size=(GUIDE_T, GUIDE_SEG, GUIDE_H), color=STEEL, yaw=a))
    return out


def _chevrons():
    """Arrowheads on the deck just outside the guide, pointing along the flow (clockwise round the wheel)."""
    out = []
    for i, deg in enumerate((112.0, 82.0, 52.0)):
        a = math.radians(deg)
        c = np.array([WX + 0.140 * math.cos(a), WY + 0.140 * math.sin(a)])
        d = np.array([math.sin(a), -math.cos(a)])  # clockwise tangent: the flow
        for s in (1, -1):
            arm = math.atan2(d[1], d[0]) + s * math.radians(135.0)  # the two strokes trail back from the tip
            p = c + 0.009 * np.array([math.cos(arm), math.sin(arm)])
            out.append(Box(f"chevron_{i}{'ab'[s < 0]}", pos=(float(p[0]), float(p[1]), DECK_TOP + 0.0005),
                           size=(0.022, 0.006, 0.001), color=WHITE, yaw=arm))
    return out


def _bin():
    (bx, by), (sx, sy, h), t = BIN_XY, BIN, BIN_WALL
    return [
        Box("bin_floor", pos=(bx, by, TOP + BIN_FLOOR / 2), size=(sx, sy, BIN_FLOOR), color=RED),
        Box("bin_wall_near", pos=(bx - sx / 2 + t / 2, by, TOP + h / 2), size=(t, sy, h), color=RED),
        Box("bin_wall_far", pos=(bx + sx / 2 - t / 2, by, TOP + h / 2), size=(t, sy, h), color=RED),
        Box("bin_wall_left", pos=(bx, by + sy / 2 - t / 2, TOP + h / 2), size=(sx - 2 * t, t, h), color=RED),
        Box("bin_wall_right", pos=(bx, by - sy / 2 + t / 2, TOP + h / 2), size=(sx - 2 * t, t, h), color=RED),
    ]


FIXTURES = [
    Box("deck", pos=(*DECK_XY, TOP + DECK[2] / 2), size=DECK, color=STEEL),
    *_guide(),
    *_chevrons(),
    *_bin(),
    Box("isolator", pos=(0.420, -0.225, DECK_TOP + 0.035), size=(0.040, 0.050, 0.070), color=DARK),
    Box("padlock", pos=(0.397, -0.225, DECK_TOP + 0.040), size=(0.008, 0.020, 0.025), color=RED),
    Box("lockout_tag", pos=(0.394, -0.225, DECK_TOP + 0.012), size=(0.002, 0.030, 0.022), color=YELLOW),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Measured from poses (on a real rig: the wheel's index mark against a degree scale, and video of the bottle).

_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
_JAM = kit.fake_states()["bottle"]  # the bottle's pose in the jam
_PULLED, _FORWARD, _BACKED = "sw_pulled", "sw_forward", "sw_backed"


def _released(st, margin=0.025) -> bool:
    """Neither gripper's TCP within 2.5 cm of the object's bounding box."""
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _tilt(st) -> float:
    """Tilt of the bottle's axis from the vertical, degrees."""
    x, y, _, _ = (float(v) for v in st.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _from_jam(st) -> float:
    return float(np.linalg.norm(np.asarray(st.pos, float) - np.asarray(_JAM.pos, float)))


def _in_bin(st) -> bool:
    """The bottle's whole footprint inside the bin's walls, standing on its floor."""
    (bx, by), (sx, sy, _), t = BIN_XY, BIN, BIN_WALL
    lo, hi = np.asarray(st.low, float), np.asarray(st.high, float)
    return bool(lo[0] >= bx - sx / 2 + t and hi[0] <= bx + sx / 2 - t and lo[1] >= by - sy / 2 + t
                and hi[1] <= by + sy / 2 - t and lo[2] < BIN_FLOOR_TOP + 0.010)


def _history(objects) -> bool:
    """Update this trial's latches (cleared only by the reset); True once a violation has happened."""
    q = kit.q_deg(objects, "wheel")
    if q <= -BACK_DEG:
        mk.MEMORY[_BACKED] = True
    if q >= FWD_DEG:
        mk.MEMORY[_FORWARD] = True
    b = objects["bottle"]
    if not mk.MEMORY.get(_BACKED) and (_from_jam(b) > MOVE_MM / 1000 or _tilt(b) > TILT_DEG):
        mk.MEMORY[_PULLED] = True
    return bool(mk.MEMORY.get(_PULLED) or mk.MEMORY.get(_FORWARD))


def score(objects) -> float:
    """0 for the rest of the trial once the bottle was pulled from the pinch (moved 3 mm or tipped 5 deg) before the
    wheel had been backed off 6 deg, or once the wheel was turned 3 deg forward. Otherwise: 1.0 with the bottle
    upright (under 15 deg) in the bin, let go and still; 0.5 with it upright, let go and still at least 5 cm out of
    the pocket but not in the bin; 0.25 while the wheel is backed off 6 deg or more; else 0."""
    if _history(objects):
        return 0.0
    b = objects["bottle"]
    placed = _tilt(b) < UPRIGHT_DEG and b.resting() and _released(b)
    if placed and _in_bin(b):
        return 1.0
    if placed and _from_jam(b) >= OUT_M:
        return 0.5
    return 0.25 if kit.q_deg(objects, "wheel") <= -BACK_DEG else 0.0


def success(objects) -> bool:
    """The bottle upright in the bin, let go, no violation; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres from the bottle's centre to the bin's centre (in plan)."""
    p = objects["bottle"].pos
    return float(math.hypot(float(p[0]) - BIN_XY[0], float(p[1]) - BIN_XY[1]))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Right arm only, open loop from the scene's fixed geometry (it reads no joint or bond state). The closed fingers
# (20 mm thick along the closing axis) come down on the near (-x) side of the -90 deg lobe and push it away from
# the robot, which turns the wheel back (anticlockwise) about 9 deg; then the arm lifts the bottle straight up out
# of the opened pocket and lowers it into the bin.

TIP = 0.013  # pad tips below the TCP
HALF = 0.010  # closed pads' outer face from the TCP along the closing axis
PUSH_YAW = math.pi / 2  # fingers close along x: the pads' flat outer face pushes in +x
PUSH_Y = WY - 0.068  # pad centre (the pads span 38 mm along y: 4.9 to 8.7 cm out along the lobe)
PAD_W = 0.038
BACK_TARGET = 14.0  # deg the push turns the wheel back
PUSH_GAP = 0.012  # start this far off the lobe face
PUSH_LAG = 0.010  # the pushing face ends ~10 mm short of the nominal one against the wheel (arm lag, and the
# fingers' outer face nearer the TCP than 10 mm; measured in sim: the wheel then turns back about 9 deg)
GRIP_YAW = POCKET_DEG - 90.0  # fingers close radially across the bottle (square to its faces)
GRIP_UP = 0.080  # TCP above the bottle's bottom: pad tips 6.7 cm up, clear of the guide (5 cm) and the wheel
SQUEEZE = 0.008
X_TRIM = 0.004  # a carried part lands a few mm short in x
TRAVEL = TOP + 0.160  # the bottle's bottom 2 cm over the bin walls; the arm's reach tops out ~TOP + 0.165 here


def _push_face_x(back_deg: float) -> float:
    """x of the -90 deg lobe's near face where the pads' inner edge meets it, with the wheel backed ``back_deg``."""
    t = math.radians(back_deg)
    inner_y = PUSH_Y + PAD_W / 2  # the pad edge nearest the shaft
    s = (WY - inner_y - LOBE_W / 2 * math.sin(t)) / math.cos(t)  # distance out along the lobe at that edge
    return WX + s * math.sin(t) - LOBE_W / 2 * math.cos(t)


def demo():
    """Push the wheel back about 9 deg with the closed fingers, then pick the bottle out of the opened pocket and
    put it in the bin. About 30 s."""
    c, o = FINGER_CLOSED, FINGER_OPEN
    pz = WHEEL_Z0 + 0.002 + TIP  # pad tips 2 mm above the wheel's underside: the pads cover the lobe's side
    x0 = _push_face_x(0.0) - PUSH_GAP - HALF
    x1 = _push_face_x(BACK_TARGET) - HALF + PUSH_LAG
    py, gy = PUSH_YAW, math.radians(GRIP_YAW)
    jx, jy = JAM_XY
    gz = DECK_TOP + GRIP_UP
    grip = -(BOTTLE[0] - SQUEEZE) / 2
    bx, by = BIN_XY[0] + X_TRIM, BIN_XY[1]
    qz = BIN_FLOOR_TOP + 0.003 + GRIP_UP  # let go 3 mm above the bin floor
    return {"right": [
        Waypoint(2.0, (x0, PUSH_Y, TRAVEL), c, "over the near side of the -90 deg lobe", py),
        Waypoint(1.5, (x0, PUSH_Y, pz + 0.030), c, "descend", py),
        Waypoint(1.0, (x0, PUSH_Y, pz), c, "down beside the lobe", py),
        Waypoint(0.5, (x0, PUSH_Y, pz), c, "settle", py),
        Waypoint(2.5, (x1, PUSH_Y, pz), c, "push the lobe away: the wheel turns back against the flow", py),
        Waypoint(0.5, (x1, PUSH_Y, pz), c, "hold", py),
        Waypoint(1.0, (x1 - 0.020, PUSH_Y, pz), c, "back off the lobe", py),
        Waypoint(1.0, (x1 - 0.020, PUSH_Y, TRAVEL), c, "up", py),
        Waypoint(2.0, (jx, jy, TRAVEL), o, "over the bottle", gy),
        Waypoint(1.5, (jx, jy, gz + 0.030), o, "descend", gy),
        Waypoint(1.0, (jx, jy, gz), o, "down slowly", gy),
        Waypoint(0.5, (jx, jy, gz), o, "settle", gy),
        Waypoint(1.5, (jx, jy, gz), grip, "close", gy),
        Waypoint(0.5, (jx, jy, gz), grip, "settle closed", gy),
        Waypoint(2.0, (jx, jy, gz + 0.060), grip, "lift straight up out of the pocket", gy),
        Waypoint(1.0, (jx, jy, TRAVEL), grip, "lift", gy),
        Waypoint(2.5, (bx, by, TRAVEL), grip, "carry to the reject bin", gy),
        Waypoint(0.6, (bx, by, TRAVEL), grip, "settle over", gy),
        Waypoint(1.5, (bx, by, qz + 0.030), grip, "lower", gy),
        Waypoint(1.5, (bx, by, qz), grip, "set down slowly", gy),
        Waypoint(0.6, (bx, by, qz), grip, "settle", gy),
        Waypoint(1.5, (bx, by, qz), o, "release", gy),
        Waypoint(1.5, (bx, by, TOP + 0.200), o, "retreat above the bottle's cap (2.5 cm clear of it)", gy),
    ]}


# -- offline self-test ------------------------------------------------------------------------------------------


def _geometry_check() -> None:
    """The pocket clears the bottle from 0 to 20 deg of back-off, the trailing lobe sits 1-2 mm off it, the lobes
    clear the guide over the whole travel, and the bottle clears the guide (2-D separating-axis test in plan)."""

    def rect(c, size, yaw):
        r = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        return np.array([np.asarray(c) + r @ np.array(p) for p in
                         [(-size[0] / 2, -size[1] / 2), (size[0] / 2, -size[1] / 2), (size[0] / 2, size[1] / 2),
                          (-size[0] / 2, size[1] / 2)]])

    def gap(a, b):
        best = -1.0
        for p in (a, b):
            for i in range(4):
                e = p[(i + 1) % 4] - p[i]
                n = np.array([-e[1], e[0]]) / np.linalg.norm(e)
                best = max(best, (b @ n).min() - (a @ n).max(), (a @ n).min() - (b @ n).max())
        return best

    def lobe(deg, back):
        a = math.radians(deg + back)
        return rect((WX + LOBE_R / 2 * math.cos(a), WY + LOBE_R / 2 * math.sin(a)), (LOBE_R, LOBE_W), a)

    bottle = rect(JAM_XY, BOTTLE[:2], _pa)
    guides = [rect(g.pos[:2], g.size[:2], g.yaw) for g in FIXTURES if g.name.startswith("guide_")]
    trail = gap(lobe(90.0, 0.0), bottle)
    assert 0.001 < trail < 0.002, trail
    for back in range(0, 21):
        for deg in LOBES_DEG:
            assert gap(lobe(deg, back), bottle) > 0.001 or (deg == 90.0 and back == 0), (deg, back)
            assert min(gap(lobe(deg, back), g) for g in guides) > 0.003, (deg, back)
    assert all(gap(bottle, g) > 0.001 for g in guides), [round(gap(bottle, g), 4) for g in guides]
    assert gap(lobe(90.0, -FWD_DEG), bottle) < 0, "a forward turn of 3 deg must reach the bottle"
    # the push really turns the wheel: the pad face starts clear of the lobe and ends 14 deg on
    assert _push_face_x(BACK_TARGET) - _push_face_x(0.0) > 0.008


def self_test():
    """The scoring rules on hand-built sequences of states (the wheel and bottle posed by the kit's kinematics)."""
    global _TEST_TCPS
    _geometry_check()
    jx, jy = JAM_XY
    yaw_q = mk.mat_to_quat(mk.axis_angle((0, 0, 1), _pa))
    tip_q = mk.mat_to_quat(mk.axis_angle((0, 0, 1), _pa) @ mk.axis_angle((1, 0, 0), math.radians(80)))

    def st(wheel=0.0, bottle=None, quat=None, vel=None):
        moved = {} if bottle is None else {"bottle": (np.asarray(bottle, float), yaw_q if quat is None else quat)}
        out = kit.fake_states(q={"wheel": wheel}, moved=moved, deg=True)
        if vel is not None:
            import dataclasses

            out["bottle"] = dataclasses.replace(out["bottle"], lin_vel=np.asarray(vel, float))
        return out

    bin_pose = (BIN_XY[0], BIN_XY[1], BIN_FLOOR_TOP)
    start = st()
    backed = st(-12.0)
    lifted = st(-12.0, (jx, jy, DECK_TOP + 0.080))
    over_bin = st(-12.0, (BIN_XY[0], BIN_XY[1], TOP + 0.090))
    in_bin = st(-12.0, bin_pose)
    on_deck = st(-12.0, (0.250, -0.020, DECK_TOP))
    held_tcp = (BIN_XY[0], BIN_XY[1], BIN_FLOOR_TOP + GRIP_UP)
    proper = (start, backed, lifted, over_bin, in_bin)
    cases = {  # name: (frames, gripper TCPs at the last frame, score, success)
        "start": ((start,), [], 0.0, False),
        "wheel backed off 12 deg, bottle still in the pocket": ((start, backed), [], 0.25, False),
        "wheel backed off only 4 deg": ((start, st(-4.0)), [], 0.0, False),
        "backed off, bottle lifted out and held": ((start, backed, lifted), [(jx, jy, DECK_TOP + 0.080 + GRIP_UP)], 0.25,
                                                   False),
        "backed off, bottle in the bin, let go": (proper, [], 1.0, True),
        "in the bin but still gripped": (proper, [held_tcp], 0.25, False),
        "in the bin, still sliding": ((*proper[:-1], st(-12.0, bin_pose, vel=(0.03, 0, 0))), [], 0.25, False),
        "in the bin, knocked over": ((*proper[:-1], st(-12.0, (BIN_XY[0], BIN_XY[1], BIN_FLOOR_TOP + 0.02),
                                                         quat=tip_q)), [], 0.25, False),
        "out of the pocket, stood on the deck, not in the bin": ((start, backed, lifted, on_deck), [], 0.5, False),
        "naive: grab and lift straight out of the pinch": ((start, st(0.0, (jx, jy, DECK_TOP + 0.030))), [], 0.0,
                                                            False),
        "naive lift, then back off and bin it": ((start, st(0.0, (jx, jy, DECK_TOP + 0.030)), backed, lifted,
                                                  in_bin), [], 0.0, False),
        "bottle dragged 4 mm along the guide, wheel at -3 deg": ((start, st(-3.0, (jx, jy - 0.004, DECK_TOP))), [],
                                                                0.0, False),
        "bottle moved 2 mm (within the pinch) then the right way": ((start, st(0.0, (jx, jy - 0.002, DECK_TOP)),
                                                                    *proper[1:]), [], 1.0, True),
        "wheel turned forward 4 deg (with the flow)": ((start, st(4.0)), [], 0.0, False),
        "forward 4 deg, back to -12 and binned": ((start, st(4.0), *proper[1:]), [], 0.0, False),
        "binned, then the wheel indexed forward 5 deg": ((*proper, st(5.0, bin_pose)), [], 0.0, False),
        "backed off, then turned back to 0 with the bottle in the pocket": ((start, backed, start), [], 0.0, False),
        "a new trial after a naive pull starts clean": ((start, st(0.0, (jx, jy, DECK_TOP + 0.030)), None, *proper), [],
                                                        1.0, True),
    }
    try:
        for name, (frames, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for f in frames[:-1]:
                if f is None:
                    mk.new_episode()
                    continue
                score(f)
            _TEST_TCPS = tcps
            got, got_success = score(frames[-1]), success(frames[-1])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: geometry and the scoring rules on hand-built states, then the right arm's plan replayed
    # through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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
