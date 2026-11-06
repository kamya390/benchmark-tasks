"""On the station table, in front of the left arm, a steel plate (12 x 10 cm, 4 mm thick) lies flat on the bench. A
small switchable lifting magnet sits on its centre, switched ON: a 7 x 5 x 4 cm block with a red switch lever on a
pivot at its far end, lying flat along the magnet's top and pointing away from the arms (ON). Swinging the lever 45
degrees or more round toward the left switches the magnet OFF. Nearer the middle of the table stands a machine table
(a 4 cm tall fixed block) with a printed outline for the plate, and to its left, nearer the arms, the magnet's
stand. The plate is too thin to pinch and wider than the gripper opens, so it can only be moved by the magnet. The
job is to set the plate in the outline and put the magnet on its stand.

While ON, the magnet holds the plate with up to 40 N. Once the lever passes 45 degrees the hold is gone for the
rest of the trial: a plate that is in the air at that moment falls, and a magnet switched off before the move leaves
the plate where it is. The trial scores 0 if the magnet lets go of the plate while the plate's bottom is more than
3 mm above whatever is under it, or if the plate ever moves down faster than 0.25 m/s (a dropped load).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/lifting-magnet-release-on-rest/lifting_magnet_release_on_rest.py \\
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
import mechkit as mk  # noqa: E402  (multi-box parts, the lever pivot and the magnet's hold; installs its hooks)

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
PLATE = (0.120, 0.100, 0.004)
PLATE_KG = 0.38  # 4 mm mild steel
PLATE_START = (0.300, 0.300)
MAG = (0.070, 0.050, 0.040)  # the magnet's body
MAG_KG = 0.25
BOSS = (0.012, 0.016, 0.006)  # the lever's pivot boss on the far end of the body, flush with its top
LEVER = (0.085, 0.010, 0.008)  # the lever bar: 5 mm behind the pivot to 80 mm in front of it
LEVER_KG = 0.015
PIVOT_X = MAG[0] / 2 + BOSS[0] / 2  # the pivot axis, from the body's centre
LEVER_FRICTION = 0.02  # N*m: the lever stays where it is left
HOLD_N = 40.0  # the magnet's hold on the plate (pull)
OFF_DEG = 45.0  # the lever this far round = OFF

MT = (0.170, 0.150, 0.040)  # the machine table
MT_XY = (0.300, 0.120)
MT_TOP = TOP + MT[2]
OUTLINE = (0.136, 0.116)  # the printed outline (inside edges): 8 mm round the plate
LINE = (0.003, 0.0005)  # printed line width, thickness
STAND = (0.080, 0.080, 0.060)
STAND_XY = (0.160, 0.200)
STAND_TOP = TOP + STAND[2]

DROP_GAP = 0.003  # released with the plate's bottom more than 3 mm above the surface under it = dropped
DROP_SPEED = 0.25  # ... or the plate ever moving down faster than this (a 3 mm free fall)
LIFTED = 0.030
FLAT_DEG = 3.0

STEEL = (0.50, 0.52, 0.55)
MAG_GREY = (0.30, 0.31, 0.33)
POLE = (0.62, 0.63, 0.65)
RED = (0.80, 0.10, 0.08)
TABLE_COL = (0.35, 0.40, 0.45)
INK = (0.95, 0.85, 0.10)
STAND_COL = (0.20, 0.35, 0.60)

kit = mk.Kit()
_PZ = TOP + PLATE[2]  # the plate's top at the start: the magnet's face
kit.part("plate", (*PLATE_START, TOP), [mk.box(PLATE, at=(0.0, 0.0, PLATE[2] / 2))], mass=PLATE_KG, color=STEEL)
kit.part("magnet_body", (*PLATE_START, _PZ), [
    mk.box((MAG[0], MAG[1], 0.006), at=(0.0, 0.0, 0.003), color=POLE),
    mk.box((MAG[0], MAG[1], MAG[2] - 0.006), at=(0.0, 0.0, 0.006 + (MAG[2] - 0.006) / 2)),
    mk.box(BOSS, at=(PIVOT_X, 0.0, MAG[2] - BOSS[2] / 2)),
], mass=MAG_KG, color=MAG_GREY)
_PIVOT = (PLATE_START[0] + PIVOT_X, PLATE_START[1], _PZ + MAG[2])
kit.part("lever", _PIVOT, [mk.box(LEVER, at=(LEVER[0] / 2 - 0.005, 0.0, LEVER[2] / 2))], mass=LEVER_KG, color=RED)
# q = the lever's swing about the vertical, + toward the left (+y): 0 = ON (pointing away), 45+ = OFF
kit.hinge("lever_pivot", "magnet_body", "lever", anchor=_PIVOT, axis=(0.0, 0.0, 1.0), lower_deg=-5.0, upper_deg=90.0,
          friction=LEVER_FRICTION)
# The hold acts through a small pole-face part fixed inside the body (a joint's two bodies never collide, even once
# the joint is switched off, so bonding the body itself to the plate let it sink through the plate when switched OFF).
kit.part("pole_face", (*PLATE_START, _PZ), [mk.box((0.020, 0.020, 0.004), at=(0.0, 0.0, 0.003))], mass=0.005,
         color=POLE)
kit.bond("pole_mount", "magnet_body", "pole_face")
kit.bond("magnet", "pole_face", "plate", break_force=HOLD_N, anchor=(*PLATE_START, _PZ))


def _switch(objects) -> None:
    """The magnet's switch (sim, every control step): the lever 45 deg or more round lets go of the plate for the
    rest of the trial."""
    if "magnet" in kit._released or kit.q_deg(objects, "lever_pivot") < OFF_DEG:
        return
    kit.release("magnet")


kit.on_step(_switch)


def _fixtures():
    (cx, cy), (ox, oy), (w, t) = MT_XY, OUTLINE, LINE
    z = MT_TOP + t / 2
    return [
        Box("machine_table", pos=(cx, cy, TOP + MT[2] / 2), size=MT, color=TABLE_COL),
        Box("outline_far", pos=(cx + ox / 2 + w / 2, cy, z), size=(w, oy + 2 * w, t), color=INK),
        Box("outline_near", pos=(cx - ox / 2 - w / 2, cy, z), size=(w, oy + 2 * w, t), color=INK),
        Box("outline_left", pos=(cx, cy + oy / 2 + w / 2, z), size=(ox, w, t), color=INK),
        Box("outline_right", pos=(cx, cy - oy / 2 - w / 2, z), size=(ox, w, t), color=INK),
        Box("magnet_stand", pos=(*STAND_XY, TOP + STAND[2] / 2), size=STAND, color=STAND_COL),
    ]


OBJECTS = [*kit.objects(), *_fixtures()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _tilt(q) -> float:
    x, y, _, _ = (float(v) for v in q)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def lever_deg(objects) -> float:
    """The lever's swing from ON, degrees (from the lever's and the body's poses)."""
    return kit.q_deg(objects, "lever_pivot")


def _surface_under(xy) -> float:
    """Height of the top surface under a point: the machine table, the stand or the bench."""
    for (cx, cy), (sx, sy, _), top in ((MT_XY, MT, MT_TOP), (STAND_XY, STAND, STAND_TOP)):
        if abs(xy[0] - cx) <= sx / 2 and abs(xy[1] - cy) <= sy / 2:
            return top
    return TOP


def _plate_corners(objects):
    return [kit.point(objects, "plate", (sx * PLATE[0] / 2, sy * PLATE[1] / 2, 0.0)) for sx in (-1, 1) for sy in (-1, 1)]


def _in_outline(objects) -> bool:
    """The plate down flat on the machine table (bottom within 2 mm of its top, under 3 deg) with all four corners
    inside the printed outline, still and let go."""
    st = objects["plate"]
    (cx, cy), (ox, oy) = MT_XY, OUTLINE
    inside = all(abs(float(c[0]) - cx) <= ox / 2 and abs(float(c[1]) - cy) <= oy / 2 for c in _plate_corners(objects))
    return bool(inside and abs(float(st.low[2]) - MT_TOP) < 0.002 and _tilt(st.quat_xyzw) < FLAT_DEG and st.resting()
                and mk.released(st))


def _on_stand(objects) -> bool:
    """The magnet's body centred on its stand (within 2.5 cm), down on it (bottom within 5 mm), upright, still,
    let go."""
    st = objects["magnet_body"]
    c = kit.point(objects, "magnet_body", (0.0, 0.0, 0.0))
    return bool(math.hypot(float(c[0]) - STAND_XY[0], float(c[1]) - STAND_XY[1]) < 0.025
                and abs(float(c[2]) - STAND_TOP) < 0.005 and _tilt(st.quat_xyzw) < 10.0 and st.resting()
                and mk.released(st))


# The trial's history (mk.MEMORY, cleared by the embodiment's reset at the start of each trial): whether the plate was
# ever lifted 3 cm on the magnet, whether and where the magnet let go of it, and whether it was dropped.
_KEY = "magnet_history"


def _update(objects) -> dict:
    h = mk.MEMORY.setdefault(_KEY, {"lifted": False, "let_go": False, "dropped": False})
    st = objects["plate"]
    if float(st.lin_vel[2]) < -DROP_SPEED:
        h["dropped"] = True
    let_go = lever_deg(objects) >= OFF_DEG or kit.broken(objects, "magnet")
    if let_go and not h["let_go"]:
        h["let_go"] = True
        centre = kit.point(objects, "plate", (0.0, 0.0, 0.0))
        if float(st.low[2]) - _surface_under((float(centre[0]), float(centre[1]))) > DROP_GAP:
            h["dropped"] = True
    if not h["let_go"] and float(st.low[2]) > TOP + LIFTED:
        h["lifted"] = True
    return h


def score(objects) -> float:
    """1.0: the plate flat in the outline and let go, the magnet switched OFF (lever 45+ deg) while the plate was
    resting, and the magnet then put on its stand and let go. 0.75: as 1.0 but the magnet not (yet) on its stand.
    0.5: the plate flat in the outline with the magnet still ON on it, both let go. 0.25: the plate lifted 3 cm on the
    magnet at some point. 0 otherwise, and 0 for the rest of the trial once the plate has been dropped."""
    h = _update(objects)
    if h["dropped"]:
        return 0.0
    if _in_outline(objects):
        if h["let_go"] and lever_deg(objects) >= OFF_DEG:
            return 1.0 if _on_stand(objects) else 0.75
        if not h["let_go"] and mk.released(objects["magnet_body"]):
            return 0.5
    return 0.25 if h["lifted"] else 0.0


def success(objects) -> bool:
    """The plate in the outline, the magnet switched off on rest and on its stand; holds 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: the plate's centre to the outline's centre, plus the magnet's centre to its stand."""
    p = kit.point(objects, "plate", (0.0, 0.0, 0.0))
    m = kit.point(objects, "magnet_body", (0.0, 0.0, 0.0))
    return float(math.dist((float(p[0]), float(p[1])), MT_XY) + math.dist((float(m[0]), float(m[1])), STAND_XY))


# -- the scripted demo (open loop, from the fixed scene geometry only) ------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.130  # carried, the plate's bottom ~9 cm above the bench (5 cm over the machine table)
SQUEEZE = 0.008
OPEN = -0.040  # 80 mm over the 50 mm body
GRIP = -(MAG[1] - SQUEEZE) / 2
DOWN = 0.022  # pad tips 22 mm down the body's sides
X_TRIM = 0.005  # the carried 0.65 kg lands ~5 mm short in x (toward the arm)
PRESS = 0.003  # set the plate down 3 mm into the table (the held load hangs low; the arm gives)
PUSH_R = 0.055  # push the lever 5.5 cm out from its pivot ...
PUSH_OFF = 0.017  # ... with the closed fingers' side (10 mm) against the lever's side (5 mm), 2 mm clear
PUSH_TO = 85.0  # swing it to 85 deg (the lever trails the fingers: ~70 deg in sim)


def _push_point(deg):
    """TCP for the closed fingers pressing on the lever's trailing (-y) side at ``deg`` of swing."""
    a = math.radians(deg)
    px, py = MT_XY[0] + X_TRIM + PIVOT_X, MT_XY[1]
    return (px + PUSH_R * math.cos(a) + PUSH_OFF * math.sin(a), py + PUSH_R * math.sin(a) - PUSH_OFF * math.cos(a)), a


def demo():
    """Left arm only, open loop from the scene's fixed geometry: grip the magnet's body across its width, lift the
    plate on it, carry it over the outline, set it down and let go; with the plate resting, swing the lever round to
    about 70 deg with the closed fingers (OFF); grip the magnet again, lift it off the plate and set it on its stand."""
    (sx, sy), (mx, my), (tx, ty) = PLATE_START, MT_XY, STAND_XY
    mx += X_TRIM
    gz0 = TOP + PLATE[2] + MAG[2] - DOWN + TIP
    gz1 = MT_TOP + PLATE[2] + MAG[2] - DOWN + TIP
    gz2 = STAND_TOP + MAG[2] - DOWN + TIP
    top = MT_TOP + PLATE[2] + MAG[2]  # the lever's underside, the plate on the machine table
    pz = top - 0.006 + TIP  # pad tips 6 mm under the lever: the pads cover its 8 mm side
    o, g, c = OPEN, GRIP, FINGER_CLOSED
    p0, a0 = _push_point(-25.0)
    plan = [
        Waypoint(2.0, (sx, sy, TRAVEL), o, "over the magnet"),
        Waypoint(1.5, (sx, sy, gz0 + 0.020), o, "descend"),
        Waypoint(1.0, (sx, sy, gz0), o, "down"),
        Waypoint(0.5, (sx, sy, gz0), o, "settle"),
        Waypoint(1.5, (sx, sy, gz0), g, "grip the magnet's body"),
        Waypoint(0.5, (sx, sy, gz0), g, "settle"),
        Waypoint(2.0, (sx, sy, gz0 + LIFTED + 0.010), g, "lift the plate on the magnet, slowly"),
        Waypoint(1.5, (sx, sy, TRAVEL), g, "lift"),
        Waypoint(2.5, (mx, my, TRAVEL), g, "carry over the machine table"),
        Waypoint(0.8, (mx, my, TRAVEL), g, "settle"),
        Waypoint(2.0, (mx, my, gz1 + 0.015), g, "lower"),
        Waypoint(2.0, (mx, my, gz1 - PRESS), g, "set the plate down slowly"),
        Waypoint(0.8, (mx, my, gz1 - PRESS), g, "settle"),
        Waypoint(1.5, (mx, my, gz1 - PRESS), o, "let go: the plate rests, magnet still ON"),
        Waypoint(1.0, (mx, my, TRAVEL), o, "up"),
        Waypoint(1.5, (*p0, TRAVEL), c, "over the lever, behind it, fingers closed", a0),
        Waypoint(1.5, (*p0, pz), c, "down beside the lever", a0),
    ]
    for deg in (0.0, 20.0, 40.0, 60.0, 75.0, PUSH_TO):
        p, a = _push_point(deg)
        plan.append(Waypoint(1.0, (*p, pz), c, f"swing the lever to {deg:.0f} deg", a))
    p, a = _push_point(PUSH_TO)
    plan += [
        Waypoint(1.0, (*p, TRAVEL), c, "up off the lever (magnet OFF)", a),
        Waypoint(1.5, (mx, my, TRAVEL), o, "over the magnet"),
        Waypoint(1.5, (mx, my, gz1 + 0.020), o, "descend"),
        Waypoint(1.0, (mx, my, gz1), o, "down"),
        Waypoint(0.5, (mx, my, gz1), o, "settle"),
        Waypoint(1.5, (mx, my, gz1), g, "grip the magnet's body"),
        Waypoint(0.5, (mx, my, gz1), g, "settle"),
        Waypoint(1.5, (mx, my, gz1 + LIFTED), g, "lift it off the plate"),
        Waypoint(1.0, (mx, my, TRAVEL), g, "up"),
        Waypoint(2.0, (tx, ty, TRAVEL), g, "carry to the stand"),
        Waypoint(1.5, (tx, ty, gz2 + 0.015), g, "lower"),
        Waypoint(1.5, (tx, ty, gz2 + 0.002), g, "set it on the stand"),
        Waypoint(0.5, (tx, ty, gz2 + 0.002), g, "settle"),
        Waypoint(1.5, (tx, ty, gz2 + 0.002), o, "let go"),
        Waypoint(1.0, (tx, ty, TRAVEL), o, "up"),
    ]
    return {"left": plan}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states (posed by the kit's own forward kinematics)."""
    import dataclasses

    def st(plate_xy=PLATE_START, plate_z=TOP, lever=0.0, mag=None, plate_vel=None, plate_yaw=0.0, mag_vel=None):
        """The plate's bottom centre at (plate_xy, plate_z); the magnet on it (default) or with its bottom centre at
        ``mag``; the lever at ``lever`` deg."""
        q = mk.yaw_quat(math.radians(plate_yaw))
        moved = {"plate": (np.array([*plate_xy, plate_z]), np.asarray(q))}
        if mag is not None:
            moved["magnet_body"] = (np.asarray(mag, float), np.array([0.0, 0.0, 0.0, 1.0]))
        else:
            r = mk.quat_to_mat(q)
            moved["magnet_body"] = (np.array([*plate_xy, plate_z]) + r @ np.array([0, 0, PLATE[2]]), np.asarray(q))
        out = kit.fake_states(q={"lever_pivot": lever}, moved=moved, deg=True)
        for name, v in (("plate", plate_vel), ("magnet_body", mag_vel)):
            if v is not None:
                out[name] = dataclasses.replace(out[name], lin_vel=np.asarray(v, float))
        return out

    start = st()
    lifted = st(plate_z=TOP + 0.05)
    over = st(MT_XY, MT_TOP + 0.05)
    down = st(MT_XY, MT_TOP)
    off = st(MT_XY, MT_TOP, lever=75.0)
    mag_up = st(MT_XY, MT_TOP, lever=75.0, mag=(*MT_XY, MT_TOP + PLATE[2] + 0.04))
    stand = (*STAND_XY, STAND_TOP)
    done = st(MT_XY, MT_TOP, lever=75.0, mag=stand)

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

    held_on_table = (*MT_XY, MT_TOP + PLATE[2] + 0.030)
    cases = {
        "start": ((start,), 0.0, False),
        "plate lifted 5 cm on the magnet": ((start, (lifted, (*PLATE_START, TOP + 0.08))), 0.25, False),
        "carried over the machine table": ((start, lifted, over), 0.25, False),
        "set in the outline, magnet ON, let go (naive stop)": ((start, lifted, over, down), 0.5, False),
        "set in the outline, magnet ON, still gripped": ((start, lifted, over, (down, held_on_table)), 0.25, False),
        "switched OFF on rest": ((start, lifted, over, down, off), 0.75, False),
        "switched OFF on rest, magnet lifted off and held": ((start, lifted, over, down, off, mag_up), 0.75, False),
        "full procedure, magnet on its stand": ((start, lifted, over, down, off, mag_up, done), 1.0, True),
        "magnet on its stand, still gripped": ((start, lifted, over, down, off, mag_up,
                                                (done, (*STAND_XY, STAND_TOP + 0.03))), 0.75, False),
        "switched OFF in the air over the table (plate drops)": (
            (start, lifted, st(MT_XY, MT_TOP + 0.05, lever=75.0), st(MT_XY, MT_TOP, lever=75.0, mag=stand)), 0.0,
            False),
        "switched OFF 5 mm above the table": ((start, lifted, over, st(MT_XY, MT_TOP + 0.005, lever=60.0), done), 0.0,
                                              False),
        "switched OFF 2 mm above the table counts as resting": (
            (start, lifted, over, st(MT_XY, MT_TOP + 0.002, lever=60.0), off, mag_up, done), 1.0, True),
        "switched OFF first on the bench: plate stranded": ((start, st(lever=75.0), st(lever=75.0, mag=stand)), 0.0,
                                                           False),
        "set down hard (plate moving down at 0.4 m/s)": (
            (start, lifted, st(MT_XY, MT_TOP + 0.01, plate_vel=(0, 0, -0.4)), down, off, mag_up, done), 0.0, False),
        "a new trial after a drop starts clean": ((start, lifted, st(MT_XY, MT_TOP + 0.05, lever=75.0), None, start,
                                                   lifted, over, down, off, mag_up, done), 1.0, True),
        "plate 2 cm off the outline": ((start, lifted, st((MT_XY[0] + 0.02, MT_XY[1]), MT_TOP + 0.03),
                                        st((MT_XY[0] + 0.02, MT_XY[1]), MT_TOP),
                                        st((MT_XY[0] + 0.02, MT_XY[1]), MT_TOP, lever=75.0, mag=stand)), 0.25, False),
        "plate turned 15 deg in the outline": ((start, lifted, st(MT_XY, MT_TOP + 0.03, plate_yaw=15.0),
                                                st(MT_XY, MT_TOP, plate_yaw=15.0),
                                                st(MT_XY, MT_TOP, plate_yaw=15.0, lever=75.0, mag=stand)), 0.25,
                                               False),
        "lever pushed back to ON after release": ((start, lifted, over, down, off, st(MT_XY, MT_TOP, lever=10.0,
                                                                                      mag=stand)), 0.25, False),
        "magnet left on the bench, not the stand": ((start, lifted, over, down, off,
                                                     st(MT_XY, MT_TOP, lever=75.0, mag=(0.30, -0.05, TOP))), 0.75,
                                                    False),
        "magnet on the stand but still swinging": ((start, lifted, over, down, off,
                                                    st(MT_XY, MT_TOP, lever=75.0, mag=stand, mag_vel=(0.05, 0, 0))),
                                                   0.75, False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert abs(lever_deg(st(lever=30.0)) - 30.0) < 1e-6 and not kit.broken(start, "magnet")
    mk.new_episode()
    return len(cases)



if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
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
