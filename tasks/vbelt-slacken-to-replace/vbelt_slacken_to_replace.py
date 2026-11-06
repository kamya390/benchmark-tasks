"""On the station table, in front of the right arm, stands a small two-pulley belt drive with its shafts upright, so
the belt runs flat. A fixed steel channel along x carries two deck rails 4 cm above the table. At its far end a fixed
driven pulley (an octagonal 40 mm hub with a 48 mm retaining flange on top) rises between the rails; 16 cm nearer the
arms the motor pulley (the same hub and flange) rises between the rails from a steel slide base under the deck. The
slide base runs along x on a friction slideway (it stays where it is pushed, about 1.5 N to move it) and carries a grey
motor on its near end, outside the channel; it can move 30 mm toward the driven pulley (slack) or 5 mm away.

A worn red belt (a rigid loop, 20.1 x 6.4 cm inside, 8 mm wall, 14 mm tall) lies on the deck round both hubs, under
both flanges, snug: at the start there is 1 mm of play in its length and each flange overlaps its end of the loop by
3.5 mm, so it cannot be lifted off. A new black belt of the same size lies on two rests to the right of the drive,
and a red return bin stands to its left. With the motor slid in at least 7 mm the loop clears both flanges, and with
about 2 cm of slack there is room to centre it and lift it off, or lower a belt on. The job: change the belt, bin the
worn one, and leave the drive tensioned (the motor slid back out until the new belt is snug on both hubs).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/vbelt-slacken-to-replace/vbelt_slacken_to_replace.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (multi-box parts and the motor slideway; installs its yam_arena hooks)

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The right arm's base is at (0, -0.305, 0.75).
Y_D = -0.240  # the drive's centre line
X_MOTOR0 = 0.240  # the motor pulley's axis with the slide base at d = 0
X_DRIVEN = 0.400  # the driven pulley's axis
DECK = TOP + 0.040  # the deck rails' top: the belts rest on it
HUB = 0.040  # octagonal hubs, 40 mm across flats ...
FLANGE = 0.048  # ... with a 48 mm retaining flange on top
GROOVE = 0.022  # deck to the underside of the flange
FLANGE_T = 0.005
FLANGE_TOP = DECK + GROOVE + FLANGE_T
D_MIN, D_MAX = -0.005, 0.030  # slide-base travel (+ = toward the driven pulley = slack)
SLIDE_N = 1.5  # Coulomb friction of the slideway (N): it stays where it is left
MOTOR_KG = 0.300

BELT_IN = (0.201, 0.064)  # inside length, inside width
BELT_WALL, BELT_H, BELT_KG = 0.008, 0.014, 0.040
BELT_OUT = (BELT_IN[0] + 2 * BELT_WALL, BELT_IN[1] + 2 * BELT_WALL)
CLEAR_D = (X_DRIVEN - X_MOTOR0) + FLANGE - BELT_IN[0]  # 7 mm: the slack at which the loop clears both flanges

RAIL_X = (0.200, 0.450)
RAIL_Y = (0.023, 0.050)  # inner and outer edge, either side of the centre line
RED_START = ((X_MOTOR0 + X_DRIVEN) / 2, Y_D)  # the worn belt, centred on the two hubs
SPARE = (0.320, -0.385)  # the new belt's centre, on its rests
BIN = (0.310, -0.085)  # the return bin's centre
BIN_IN = (0.250, 0.120)
BIN_FLOOR = TOP + 0.003
BIN_WALL = (0.006, 0.030)

STEEL = (0.55, 0.57, 0.60)
DARK = (0.22, 0.23, 0.25)
GREYM = (0.50, 0.52, 0.55)
RED = (0.75, 0.12, 0.10)
BLACK = (0.07, 0.07, 0.08)
ZINC = (0.80, 0.81, 0.83)
BIN_RED = (0.80, 0.20, 0.15)


def _octagon(across, h, z, color=None):
    """A regular octagon (``across`` flat to flat), ``h`` tall, centred at height ``z`` in the part frame: four
    boxes turned 0/45/90/135 deg."""
    w = across * math.tan(math.pi / 8)
    return [mk.box((across, w, h), at=(0.0, 0.0, z), color=color, yaw=k * math.pi / 4) for k in range(4)]


def _belt_boxes():
    """A rigid loop, origin at its centre on its bottom face: two side walls along x, two end walls."""
    lx, ly = BELT_OUT
    z = BELT_H / 2
    return [
        mk.box((lx, BELT_WALL, BELT_H), at=(0.0, (ly - BELT_WALL) / 2, z)),
        mk.box((lx, BELT_WALL, BELT_H), at=(0.0, -(ly - BELT_WALL) / 2, z)),
        mk.box((BELT_WALL, BELT_IN[1], BELT_H), at=((lx - BELT_WALL) / 2, 0.0, z)),
        mk.box((BELT_WALL, BELT_IN[1], BELT_H), at=(-(lx - BELT_WALL) / 2, 0.0, z)),
    ]


# The slide base's frame: on the motor pulley's axis at deck height. Under the deck a 15.5 x 8.8 cm base (2.6 cm
# thick, 4 mm clear of the table and of the rails); the hub rises between the rails; the motor sits on the near end.
MOTOR_GRIP_X = -0.1025  # the motor body's centre, from the pulley axis
MOTOR_BODY = (0.045, 0.060, 0.050)
kit = mk.Kit()
kit.part("motor", (X_MOTOR0, Y_D, DECK), [
    mk.box((0.155, 0.088, 0.026), at=(-0.0475, 0.0, -0.023), color=STEEL),
    *_octagon(HUB, GROOVE + 0.010, GROOVE / 2 - 0.005, ZINC),
    *_octagon(FLANGE, FLANGE_T, GROOVE + FLANGE_T / 2, ZINC),
    mk.box(MOTOR_BODY, at=(MOTOR_GRIP_X, 0.0, -0.010 + MOTOR_BODY[2] / 2), color=GREYM),
], mass=MOTOR_KG, color=STEEL)
kit.slider("motor_slide", None, "motor", axis=(1.0, 0.0, 0.0), lower=D_MIN, upper=D_MAX, friction=SLIDE_N, damping=5.0)
kit.part("driven", (X_DRIVEN, Y_D, DECK), [
    *_octagon(HUB, GROOVE + 0.040, (GROOVE - 0.040) / 2),
    *_octagon(FLANGE, FLANGE_T, GROOVE + FLANGE_T / 2),
], color=ZINC)
kit.part("belt_red", (*RED_START, DECK), _belt_boxes(), mass=BELT_KG, color=RED)
kit.part("belt_black", (*SPARE, DECK), _belt_boxes(), mass=BELT_KG, color=BLACK)


def _fixtures():
    (x0, x1), (yi, yo) = RAIL_X, RAIL_Y
    xc, lx = (x0 + x1) / 2, x1 - x0
    out = []
    for s, side in ((1, "left"), (-1, "right")):
        out.append(Box(f"deck_rail_{side}", pos=(xc, Y_D + s * (yi + yo) / 2, DECK - 0.003), size=(lx, yo - yi, 0.006),
                       color=STEEL))
        out.append(Box(f"channel_wall_{side}", pos=(xc, Y_D + s * (yo + 0.003), TOP + 0.020), size=(lx, 0.006, 0.040),
                       color=DARK))
    for s, side in ((1, "far"), (-1, "near")):
        out.append(Box(f"spare_rest_{side}", pos=(SPARE[0] + s * (BELT_OUT[0] - BELT_WALL) / 2, SPARE[1], TOP + 0.020),
                       size=(0.030, 0.090, 0.040), color=DARK))
    bx, by = BIN
    (ix, iy), (t, h) = BIN_IN, BIN_WALL
    out += [
        Box("bin_floor", pos=(bx, by, TOP + 0.0015), size=(ix + 2 * t, iy + 2 * t, 0.003), color=BIN_RED),
        Box("bin_wall_far", pos=(bx + ix / 2 + t / 2, by, TOP + h / 2), size=(t, iy + 2 * t, h), color=BIN_RED),
        Box("bin_wall_near", pos=(bx - ix / 2 - t / 2, by, TOP + h / 2), size=(t, iy + 2 * t, h), color=BIN_RED),
        Box("bin_wall_left", pos=(bx, by + iy / 2 + t / 2, TOP + h / 2), size=(ix, t, h), color=BIN_RED),
        Box("bin_wall_right", pos=(bx, by - iy / 2 - t / 2, TOP + h / 2), size=(ix, t, h), color=BIN_RED),
    ]
    return out


OBJECTS = [*kit.objects(), *_fixtures()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SNUG_D = 0.0015  # tensioned: the slide base back out to within 1.5 mm of d = 0 (the belt's 1 mm of play + 0.5)
SEATED_Z = 0.004  # a belt on the deck: its bottom within 4 mm of the deck
LEVEL_DEG = 5.0


def _tilt(q) -> float:
    x, y, _, _ = (float(v) for v in q)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def slack(objects) -> float:
    """d: how far the motor pulley has been slid in toward the driven pulley (m), from the slide base's pose."""
    return kit.q(objects, "motor_slide")


def _hubs(objects):
    """World xy of the two pulley axes."""
    m = kit.point(objects, "motor", (0.0, 0.0, 0.0))
    return (float(m[0]), float(m[1])), (X_DRIVEN, Y_D)


def _round_hubs(objects, belt) -> bool:
    """The loop encircles both hubs and is not above the flanges: both pulley axes inside its inner rectangle (in
    the belt's own frame) and its lowest point under the flanges' top."""
    pos, quat = kit.pose(objects, belt)
    r = mk.quat_to_mat(quat)
    for hx, hy in _hubs(objects):
        local = r.T @ (np.array([hx, hy, float(pos[2])]) - pos)
        if abs(float(local[0])) > BELT_IN[0] / 2 or abs(float(local[1])) > BELT_IN[1] / 2:
            return False
    return float(objects[belt].low[2]) < FLANGE_TOP


def _seated(objects, belt) -> bool:
    """Round both hubs, down on the deck (bottom within 4 mm), level within 5 deg, still and let go."""
    st = objects[belt]
    return bool(_round_hubs(objects, belt) and float(st.low[2]) < DECK + SEATED_Z and _tilt(st.quat_xyzw) < LEVEL_DEG
                and st.resting() and mk.released(st))


def _in_bin(objects, belt) -> bool:
    """Wholly inside the bin's walls (its footprint), down on the floor (bottom within 6 mm), still and let go."""
    st = objects[belt]
    (bx, by), (ix, iy) = BIN, BIN_IN
    inside = (float(st.low[0]) > bx - ix / 2 - 0.001 and float(st.high[0]) < bx + ix / 2 + 0.001
              and float(st.low[1]) > by - iy / 2 - 0.001 and float(st.high[1]) < by + iy / 2 + 0.001)
    return bool(inside and float(st.low[2]) < BIN_FLOOR + 0.006 and st.resting() and mk.released(st))


# The trial's history (mk.MEMORY, cleared by the embodiment's reset at the start of each trial): whether a belt was
# ever taken off or put on the hubs while the drive was not slack enough for the loop to clear the flanges (d under
# 7 mm), i.e. forced over a flange. A rigid loop cannot do that in the simulator; on a real rig with a rubber belt
# it is the violation the procedure exists to prevent, and it caps the trial at 0.25.
_KEY = "vbelt_history"


def _update(objects) -> dict:
    h = mk.MEMORY.setdefault(_KEY, {"forced": False, "on": {}})
    d = slack(objects)
    for belt in ("belt_red", "belt_black"):
        on = _round_hubs(objects, belt)
        was = h["on"].get(belt, on)
        if on != was and d < CLEAR_D:
            h["forced"] = True
        h["on"][belt] = on
    return h


def score(objects) -> float:
    """1.0: the worn red belt in the bin and the black belt seated round both hubs with the slide base back out to
    tension (d at most 1.5 mm), every belt and the motor let go and still. 0.5: red in the bin and black seated, the
    drive still slack. 0.25: red in the bin. 0 otherwise. Capped at 0.25 for the rest of the trial once a belt has
    gone on or off the hubs with the drive under 7 mm of slack (forced over a flange)."""
    h = _update(objects)
    if not _in_bin(objects, "belt_red"):
        return 0.0
    s = 0.25
    if _seated(objects, "belt_black"):
        s = 0.5
        m = objects["motor"]
        if slack(objects) <= SNUG_D and mk.released(m) and m.resting():
            s = 1.0
    return min(s, 0.25) if h["forced"] else s


def success(objects) -> bool:
    """The belt changed and the drive tensioned, everything let go; holds 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: red belt to the bin centre, black belt to the hubs' midpoint, and the slack left in the drive."""
    r, b = objects["belt_red"].pos, objects["belt_black"].pos
    (mx, my), (dx, dy) = _hubs(objects)
    return float(math.dist((float(r[0]), float(r[1])), BIN) + math.dist((float(b[0]), float(b[1])),
                 ((mx + dx) / 2, (my + dy) / 2)) + max(0.0, slack(objects)))


# -- the scripted demo (open loop, from the fixed scene geometry only) ------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.115
SQUEEZE = 0.008
SLACK_TO = D_MAX  # slide the motor in to its 30 mm stop (2.3 cm of play in the loop) ...
SLACK_CMD = 0.042  # ... by commanding 12 mm past it: the gripped base trails the fingers by ~1 cm (measured in sim)
SHIFT = 0.014  # then slide the worn belt 14 mm toward the driven pulley: it clears both flanges by 1 cm or more
TENSION_TO = -0.020  # pull the motor back 20 mm past d = 0: the new belt stops it at about -1 mm (its play); the
# fingers slip on the motor once it is snug, and the gripped base trails them by up to 13 mm (measured in sim)
WALL_Y = BELT_IN[1] / 2 + BELT_WALL / 2  # the belt's left side wall, from its centre line: gripped at mid-span
BELT_OPEN = -0.017  # fingers 34 mm apart over the 8 mm wall
MOTOR_OPEN = -0.045
MOTOR_GRIP = -(MOTOR_BODY[1] - 0.012) / 2  # 12 mm squeeze: the pads drag the base against its friction


def _grip_motor(d0, d1, label):
    x0, x1 = X_MOTOR0 + MOTOR_GRIP_X + d0, X_MOTOR0 + MOTOR_GRIP_X + d1
    gz = DECK - 0.010 + MOTOR_BODY[2] - 0.030 + TIP  # pad tips 30 mm down the motor's sides
    o, g = MOTOR_OPEN, MOTOR_GRIP
    return [
        Waypoint(2.0, (x0, Y_D, TRAVEL), o, f"over the motor ({label})"),
        Waypoint(1.2, (x0, Y_D, gz + 0.020), o, "descend"),
        Waypoint(0.8, (x0, Y_D, gz), o, "down"),
        Waypoint(1.2, (x0, Y_D, gz), g, "grip the motor"),
        Waypoint(0.4, (x0, Y_D, gz), g, "settle"),
        Waypoint(3.5, (x1, Y_D, gz), g, f"slide the motor {label}"),
        Waypoint(0.5, (x1, Y_D, gz), g, "settle"),
        Waypoint(1.0, (x1, Y_D, gz), o, "let go"),
        Waypoint(1.0, (x1, Y_D, TRAVEL), o, "up"),
    ]


def demo():
    """Right arm only, open loop from the scene's fixed geometry: slide the motor in to its 30 mm stop; grip the worn belt's left
    side at mid-span, slide it 14 mm toward the driven pulley so it is centred between the flanges, lift it off and
    lay it in the bin; pick the new belt off its rests, lower it over both flanges onto the deck; slide the motor back
    out until the belt is snug, let go and back off."""
    gz_deck = DECK + 0.002 + TIP  # pad tips 2 mm above the belt's bottom (12 of its 14 mm in the grip)
    lift = gz_deck + FLANGE_TOP - DECK + 0.015  # belt bottom 15 mm over the flanges
    gz_bin = BIN_FLOOR + 0.003 + 0.002 + TIP  # let go 3 mm above the floor
    xr, yr = RED_START[0], RED_START[1] + WALL_Y
    xs, ys = SPARE[0], SPARE[1] + WALL_Y
    xb, yb = BIN[0], BIN[1] - 0.010 + WALL_Y  # the belt 1 cm right of the bin's centre line
    xn = (X_MOTOR0 + SLACK_TO + X_DRIVEN) / 2  # the slackened hubs' midpoint
    o, g = BELT_OPEN, FINGER_CLOSED
    plan = _grip_motor(0.0, SLACK_CMD, "in to its stop (slacken the drive)")
    plan += [
        Waypoint(1.5, (xr, yr, TRAVEL), o, "over the worn belt"),
        Waypoint(1.5, (xr, yr, gz_deck + 0.020), o, "descend"),
        Waypoint(1.0, (xr, yr, gz_deck), o, "down"),
        Waypoint(1.0, (xr, yr, gz_deck), g, "grip its side wall"),
        Waypoint(0.5, (xr, yr, gz_deck), g, "settle"),
        Waypoint(1.5, (xr + SHIFT, yr, gz_deck), g, "centre it between the flanges"),
        Waypoint(1.5, (xr + SHIFT, yr, lift), g, "lift it off over the flanges"),
        Waypoint(1.0, (xr + SHIFT, yr, TRAVEL), g, "up"),
        Waypoint(2.0, (xb, yb, TRAVEL), g, "carry to the bin"),
        Waypoint(1.5, (xb, yb, gz_bin + 0.020), g, "lower into the bin"),
        Waypoint(1.0, (xb, yb, gz_bin), g, "down"),
        Waypoint(1.0, (xb, yb, gz_bin), o, "let go"),
        Waypoint(1.0, (xb, yb, TRAVEL), o, "up"),
        Waypoint(2.0, (xs, ys, TRAVEL), o, "over the new belt"),
        Waypoint(1.5, (xs, ys, gz_deck + 0.020), o, "descend"),
        Waypoint(1.0, (xs, ys, gz_deck), o, "down"),
        Waypoint(1.0, (xs, ys, gz_deck), g, "grip its side wall"),
        Waypoint(0.5, (xs, ys, gz_deck), g, "settle"),
        Waypoint(1.5, (xs, ys, TRAVEL), g, "lift off the rests"),
        Waypoint(2.0, (xn, yr, TRAVEL), g, "carry over the drive"),
        Waypoint(0.6, (xn, yr, TRAVEL), g, "settle"),
        Waypoint(1.5, (xn, yr, lift), g, "lower to just over the flanges"),
        Waypoint(2.0, (xn, yr, gz_deck + 0.002), g, "lower it round both hubs onto the deck"),
        Waypoint(0.5, (xn, yr, gz_deck + 0.002), g, "settle"),
        Waypoint(1.0, (xn, yr, gz_deck + 0.002), o, "let go"),
        Waypoint(1.0, (xn, yr, TRAVEL), o, "up"),
    ]
    plan += _grip_motor(SLACK_TO, TENSION_TO, "back out (tension the belt)")
    plan += [Waypoint(1.5, (0.250, -0.140, TRAVEL + 0.03), MOTOR_OPEN, "back off")]
    return {"right": plan}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states (posed by the kit's own forward kinematics)."""
    import dataclasses

    def belt_at(x, y, bottom=DECK, yaw_deg=0.0):
        return (np.array([x, y, bottom]), np.asarray(mk.yaw_quat(math.radians(yaw_deg))))

    def st(d=0.0, red=None, black=None, vel=None):
        moved = {"belt_red": red or belt_at(*RED_START), "belt_black": black or belt_at(*SPARE)}
        out = kit.fake_states(q={"motor_slide": d}, moved=moved)
        if vel:
            out = {n: (dataclasses.replace(s, lin_vel=np.asarray(vel[n], float)) if n in vel else s)
                   for n, s in out.items()}
        return out

    snug_x = X_DRIVEN + HUB / 2 - BELT_IN[0] / 2  # the black belt pulled snug against the driven hub
    red_bin = belt_at(*BIN, BIN_FLOOR)
    red_held = belt_at(RED_START[0] + SHIFT, Y_D, FLANGE_TOP + 0.015)
    on_slack = belt_at((X_MOTOR0 + SLACK_TO + X_DRIVEN) / 2, Y_D)
    on_snug = belt_at(snug_x, Y_D)
    start = st()
    slackened = st(SLACK_TO)
    red_out = st(SLACK_TO, red=red_held)
    binned = st(SLACK_TO, red=red_bin)
    new_on = st(SLACK_TO, red=red_bin, black=on_slack)
    done = st(-0.001, red=red_bin, black=on_snug)

    def run(*frames):
        """Feed the states in order; the last score and success. None = the embodiment's reset (a new trial); a
        (frame, tcp) pair puts one gripper's TCP at ``tcp``."""
        tcps0 = mk.tcps
        mk.new_episode()
        try:
            for f in frames:
                if f is None:
                    mk.new_episode()
                    continue
                tcp = []
                if isinstance(f, tuple):
                    f, tcp = f[0], [np.asarray(f[1], float)]
                mk.tcps = lambda tcp=tcp: tcp  # noqa: E731
                got_success, got = success(f), score(f)
        finally:
            mk.tcps = tcps0
        return got, got_success

    motor_grip = (X_MOTOR0 + MOTOR_GRIP_X - 0.001, Y_D, DECK + 0.030)
    cases = {
        "start": ((start,), 0.0, False),
        "drive slackened, nothing else": ((start, slackened), 0.0, False),
        "worn belt lifted off, held": ((start, slackened, (red_out, (RED_START[0] + SHIFT, Y_D + WALL_Y,
                                                                    FLANGE_TOP + 0.03))), 0.0, False),
        "worn belt in the bin": ((start, slackened, red_out, binned), 0.25, False),
        "worn belt in the bin, still gripped": ((start, slackened, red_out, (binned, (BIN[0], BIN[1] + WALL_Y,
                                                                                    BIN_FLOOR + 0.015))), 0.0, False),
        "worn belt dropped on the table beside the bin": ((start, slackened, red_out,
                                                           st(SLACK_TO, red=belt_at(0.31, 0.06, TOP))), 0.0, False),
        "new belt on, drive left slack": ((start, slackened, red_out, binned, new_on), 0.5, False),
        "belt changed and drive tensioned": ((start, slackened, red_out, binned, new_on, done), 1.0, True),
        "tensioned, motor still gripped": ((start, slackened, red_out, binned, new_on, (done, motor_grip)), 0.5,
                                           False),
        "tension only part way (d 5 mm)": ((start, slackened, red_out, binned, new_on,
                                            st(0.005, red=red_bin, black=belt_at(snug_x - 0.003, Y_D))), 0.5, False),
        "worn belt left on the table, new belt on and tensioned": (
            (start, slackened, red_out, st(-0.001, red=belt_at(0.31, 0.06, TOP), black=on_snug)), 0.0, False),
        "new belt resting on top of the flanges": ((start, slackened, red_out, binned,
                                                    st(SLACK_TO, red=red_bin, black=belt_at(on_slack[0][0], Y_D,
                                                                                            FLANGE_TOP))), 0.25, False),
        "new belt turned across the drive": ((start, slackened, red_out, binned,
                                              st(SLACK_TO, red=red_bin, black=belt_at(0.334, Y_D, DECK, 90.0))), 0.25,
                                             False),
        "new belt seated but still sliding": ((start, slackened, red_out, binned,
                                               st(SLACK_TO, red=red_bin, black=on_slack,
                                                  vel={"belt_black": (0.03, 0, 0)})), 0.25, False),
        "worn belt forced off a tight drive (d 0)": ((start, st(0.0, red=red_held), st(0.0, red=red_bin),
                                                      st(SLACK_TO, red=red_bin, black=on_slack), done), 0.25, False),
        "new belt forced on a tight drive (d 3 mm)": ((start, slackened, red_out, binned, st(0.003, red=red_bin),
                                                      st(0.003, red=red_bin, black=belt_at(snug_x - 0.002, Y_D)),
                                                      done), 0.25, False),
        "a new trial after a forced belt starts clean": ((start, st(0.0, red=red_held), None, start, slackened,
                                                          red_out, binned, new_on, done), 1.0, True),
        "motor still moving at the end": ((start, slackened, red_out, binned, new_on,
                                           st(-0.001, red=red_bin, black=on_snug, vel={"motor": (-0.02, 0, 0)})), 0.5,
                                          False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    # geometry facts the task rests on
    assert abs(CLEAR_D - 0.007) < 1e-9, CLEAR_D
    assert _round_hubs(start, "belt_red") and not _round_hubs(start, "belt_black")
    assert abs(slack(done) + 0.001) < 1e-9
    mk.new_episode()
    return len(cases)



if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the right arm's plan replayed through the
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
