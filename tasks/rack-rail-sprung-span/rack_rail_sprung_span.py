"""On the station table stands a short open rack frame, turned side-on to the arms: two fixed steel posts 26 cm apart
(inner faces), the front post on the right, the rear post on the left. Each post carries a column of six square
holes, 14 mm across at 20 mm pitch; an orange tab beside the front post's third hole from the bottom marks the rack
position to use. In front of the rack, lying flat on the table across both arms, is a loose tool-less slide rail: an
outer steel member with a square pin on its left (rear) end and an inner member that slides out of it to the right,
with a square pin on its right (front) end. A spring inside pushes the inner member out to its stop, so the free rail
is 30 cm long, 4 cm longer than the gap between the posts. To mount it, both ends must be pulled in together until
the rail fits between the posts, the pins lined up with a hole on each post at the same height, and the rail let
go so the spring drives both pins home. Scored from the two members' poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/rack-rail-sprung-span/rack_rail_sprung_span.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (our joint kit: installs the slider + spring at scene build)


# Shared fix: per-episode memory that a new trial clears (instead of guessing a reset from the layout), and the two
# grippers' finger tips, so "released" can be checked. Same mechanism as the accepted reseat-half-inserted-plug task
# (PR #20): wrap YamArenaEmbodiment.reset and ._task_step.

_EP = {"env": None}
_MEM: dict = {}  # per-episode history; the task adds its own keys


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    reset0, step0 = cls.reset, cls._task_step

    def reset(self, *a, **k):
        _MEM.clear()
        return reset0(self, *a, **k)

    def _task_step(self, *a, **k):
        _EP["env"] = self._env
        return step0(self, *a, **k)

    cls.reset, cls._task_step = reset, _task_step


_install_hooks()


def _tcps():
    """World positions of both grippers' TCP (13 mm above the finger pad tips), as in PR #20; [] offline."""
    env = _EP["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(float)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(float) / 2
        out.append(g + (mid - g) / (np.linalg.norm(mid - g) + 1e-12) * 0.134)
    return out


_tcps_live = _tcps  # the self-test swaps _tcps for fixed points and restores this


def released(state, margin=0.025) -> bool:
    """Neither gripper's TCP within ``margin`` (2.5 cm) of the part's bounding box: the part is let go."""
    lo, hi = np.asarray(state.low) - margin, np.asarray(state.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())


SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
# -- the rack frame (fixed) --
RACK_X = 0.340  # the posts' hole column, along x
FACE_Y = 0.130  # each post's inner face, either side of the midline (26 cm between them)
POST_T, POST_W, POST_H = 0.004, 0.030, 0.160  # 4 mm steel, 3 cm wide, 16 cm tall
HOLE = 0.014  # square holes, 14 mm
PITCH = 0.020
N_HOLES = 6
HOLE_Z = {k: TOP + 0.040 + PITCH * (k - 1) for k in range(1, N_HOLES + 1)}  # hole centres, 1 at the bottom
TARGET = 3  # the hole the orange tab marks on the front post
FRONT, REAR = -1.0, 1.0  # the front post is on the right (-y), the rear post on the left (+y)

# -- the rail (free): outer member + inner member on a sprung slider --
OUTER = (0.020, 0.160, 0.014)  # x (width), y (length), z (height)
INNER = (0.012, 0.160, 0.010)
PEG = (0.006, 0.010, 0.006)  # square pin, 10 mm long along the rail
OUTER_KG, INNER_KG = 0.100, 0.050
REST_L = 0.300  # end to end (without the pins), the spring holding the inner member at its outer stop
MIN_L = 0.210  # fully pushed in
SEATED_L = 2 * FACE_Y  # 26 cm: both member ends against the post faces
SPRING_K = 10.0  # N/m
SPRING_PRELOAD = 0.030  # the spring's free length is 3 cm past the stop (0.3 N holding it there)
SPRING_DAMP = 3.0  # N*s/m, so the rail does not slam into the posts when let go
START_X = 0.220
# The rail's own frame: origin midway between the member ends, on the pins' axis, 5 mm above the members' bottoms.
AXIS_UP = 0.005
OUTER_AT = (0.0, REST_L / 2 - OUTER[1] / 2, OUTER[2] / 2 - AXIS_UP)  # member centres in the rail frame at rest
INNER_AT = (0.0, -REST_L / 2 + INNER[1] / 2, INNER[2] / 2 - AXIS_UP)
REAR_PEG = (0.0, OUTER[1] / 2 + PEG[1] / 2, AXIS_UP - OUTER[2] / 2)  # pin centres in each member's own frame
FRONT_PEG = (0.0, -INNER[1] / 2 - PEG[1] / 2, AXIS_UP - INNER[2] / 2)

STEEL = (0.62, 0.64, 0.67)
ZINC = (0.80, 0.81, 0.83)
DARK = (0.18, 0.18, 0.20)
ORANGE = (0.95, 0.45, 0.08)

kit = mk.Kit()
_RAIL_Z = TOP + AXIS_UP
kit.part("rail_outer", (START_X, OUTER_AT[1], _RAIL_Z + OUTER_AT[2]),
         [mk.box(OUTER), mk.box(PEG, at=REAR_PEG)], mass=OUTER_KG, color=STEEL)
kit.part("rail_inner", (START_X, INNER_AT[1], _RAIL_Z + INNER_AT[2]),
         [mk.box(INNER), mk.box(PEG, at=FRONT_PEG)], mass=INNER_KG, color=ZINC)
# d = how far the inner member has slid out (-y) from its rest stop: 0 at rest, -0.09 fully pushed in.
kit.slider("rail_slide", "rail_outer", "rail_inner", axis=(0.0, -1.0, 0.0), lower=MIN_L - REST_L, upper=0.0,
           stiffness=SPRING_K, rest=SPRING_PRELOAD, damping=SPRING_DAMP)


def _post(side, sy):
    """A post: two full-height side strips and, between them, bars that leave the six square holes."""
    y = sy * (FACE_Y + POST_T / 2)
    strip = (POST_W - HOLE) / 2
    parts = [Box(f"{side}_post_{n}", pos=(RACK_X + s * (HOLE / 2 + strip / 2), y, TOP + POST_H / 2),
                 size=(strip, POST_T, POST_H), color=DARK) for n, s in (("near", -1.0), ("far", 1.0))]
    edges = [TOP] + [z for k in range(1, N_HOLES + 1) for z in (HOLE_Z[k] - HOLE / 2, HOLE_Z[k] + HOLE / 2)]
    edges.append(TOP + POST_H)
    for i in range(0, len(edges), 2):
        lo, hi = edges[i], edges[i + 1]
        parts.append(Box(f"{side}_post_bar{i // 2}", pos=(RACK_X, y, (lo + hi) / 2), size=(HOLE, POST_T, hi - lo),
                         color=DARK))
    return parts


TAB = Box("unit_tab", pos=(RACK_X - POST_W / 2 - 0.005, FRONT * (FACE_Y + POST_T / 2), HOLE_Z[TARGET]),
          size=(0.010, POST_T, 0.008), color=ORANGE)
FIXTURES = [*_post("front", FRONT), *_post("rear", REAR), TAB]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

MIN_DEPTH = 0.005  # a pin counts as in its hole with its tip at least 5 mm past the post's inner face
IN_HOLE = (HOLE - PEG[0]) / 2 + 0.001  # 5 mm: the pin's centre inside the hole across x and z (4 mm of play + 1)
LEVEL_DEG = 1.5


def _pins(objects):
    """Each pin's (centre, tip) in the world."""
    out = []
    for part, at in (("rail_outer", REAR_PEG), ("rail_inner", FRONT_PEG)):
        tip = (at[0], at[1] + math.copysign(PEG[1] / 2, at[1]), at[2])
        out.append((kit.point(objects, part, at), kit.point(objects, part, tip)))
    return out


def _engaged(centre, tip):
    """(post, hole) a pin is home in, or None: its tip at least 5 mm through a post's inner face, its centre inside
    one of that post's holes. post is FRONT or REAR."""
    side = FRONT if float(tip[1]) < 0 else REAR
    if side * float(tip[1]) - FACE_Y < MIN_DEPTH or abs(float(centre[0]) - RACK_X) > IN_HOLE:
        return None
    for k, z in HOLE_Z.items():
        if abs(float(centre[2]) - z) <= IN_HOLE:
            return side, k
    return None


def _tilt_deg(objects) -> float:
    """The rail's slope along its length: the line between the two pin centres against the horizontal."""
    (a, _), (b, _) = _pins(objects)
    d = np.asarray(a, float) - np.asarray(b, float)
    return math.degrees(math.atan2(abs(float(d[2])), math.hypot(float(d[0]), float(d[1]))))


def _let_go_still(objects) -> bool:
    return all(released(objects[p]) and objects[p].resting() for p in ("rail_outer", "rail_inner"))


def _holes(objects):
    """{post: hole} for the pins that are home."""
    got = {}
    for c, t in _pins(objects):
        e = _engaged(c, t)
        if e is not None:
            got[e[0]] = e[1]
    return got


def score(objects) -> float:
    """At the end, with both members let go and still: 1.0 with both pins home in the marked hole (front post) and
    the same hole of the rear post, the rail level within 1.5 degrees; 0.5 with both pins home in matching holes at
    another height, level; 0.25 with only one pin home, or both home at different heights (a skewed rail); else 0."""
    if not _let_go_still(objects):
        return 0.0
    holes = _holes(objects)
    if len(holes) == 2 and holes[FRONT] == holes[REAR] and _tilt_deg(objects) <= LEVEL_DEG:
        return 1.0 if holes[FRONT] == TARGET else 0.5
    return 0.25 if holes else 0.0


def success(objects) -> bool:
    """Both pins home in the marked hole and the matching rear hole, level, let go, still; holds 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres from each pin's centre to a seated pin in the marked hole (front) and the matching rear hole."""
    (rear, _), (front, _) = _pins(objects)
    goal_z = HOLE_Z[TARGET] - (HOLE - PEG[2]) / 2
    gr = (RACK_X, FACE_Y + PEG[1] / 2, goal_z)
    gf = (RACK_X, -FACE_Y - PEG[1] / 2, goal_z)
    return float(math.dist([float(v) for v in rear], gr) + math.dist([float(v) for v in front], gf))


# -- the scripted demo (open loop, from the fixed scene geometry only) ------------------------------------------

YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along x, across the rail; each wrist's own sense
TIP = 0.013
TRAVEL = TOP + 0.100
SQUEEZE = 0.008
GRIP_FROM_END = 0.030  # each arm holds its member 3 cm in from that member's outer end
LIFT_Z = HOLE_Z[TARGET] - AXIS_UP + 0.002 + TIP  # pins centred on the hole: members' bottoms 5 mm below the pin axis
Z_TRIM = 0.003  # the held rail hangs 3 mm below the command (measured in sim)
X_TRIM = {"left": 0.003, "right": 0.0007}  # the held rail trails the commanded x by 3 mm at the outer member and
# leads it by 2.3 mm at the inner member's end, so the rail turns about 1 degree (measured in sim)
YAW_FIX = math.radians(-2.0)  # the held rail turns ~2 degrees (rear end back) at the rack: carry it turned back
SHORT = 0.002  # stop the ends 2 mm short of the post faces; the spring drives the last 2 mm
PULL_L = 0.204  # commanded length when pulled in; the arms give ~8 mm each against the spring (measured in sim)


def demo():
    """Both arms at once, mirrored: grip the outer member (left arm) and the inner member (right arm) 3 cm in from
    the rail's ends, lift it to the marked hole's height, pull both ends in together until the rail is about 22 cm long,
    carry it forward between the posts, ease both ends out together until the pins are through both posts, let go and
    back off. 22.1 s."""
    gz = TOP + 0.002 + TIP  # pad tips 2 mm above the table
    zl = LIFT_Z + Z_TRIM
    up = zl + 0.035  # pad tips 2 cm above the rail's top: clear to back out
    plans = {}
    for side, sy, width in (("left", 1.0, OUTER[0]), ("right", -1.0, INNER[0])):
        open_, grip, yaw = -(width + 0.030) / 2, -(width - SQUEEZE) / 2, YAW[side]
        y0 = sy * (REST_L / 2 - GRIP_FROM_END)  # 12 cm out
        yc = sy * (PULL_L / 2 - GRIP_FROM_END)  # 7.2 cm: the spring holds the arms ~8 mm wide, so ~22 cm
        ys = sy * (SEATED_L / 2 - SHORT - GRIP_FROM_END)  # 9.8 cm: the ends 2 mm short of the posts
        x0 = START_X

        def rack(y):  # a point at the rack, turned YAW_FIX about the rack's centre
            return RACK_X + X_TRIM[side] - math.sin(YAW_FIX) * y, y * math.cos(YAW_FIX)

        ry = yaw + YAW_FIX
        plans[side] = [
            Waypoint(1.8, (x0, y0, TRAVEL), open_, "over the rail", yaw),
            Waypoint(1.5, (x0, y0, gz + 0.020), open_, "descend", yaw),
            Waypoint(1.0, (x0, y0, gz), open_, "down slowly", yaw),
            Waypoint(0.5, (x0, y0, gz), open_, "settle", yaw),
            Waypoint(1.5, (x0, y0, gz), grip, "close", yaw),
            Waypoint(2.0, (x0, y0, zl), grip, "lift to the marked height", yaw),
            Waypoint(3.0, (x0, yc, zl), grip, "pull both ends in together", yaw),
            Waypoint(0.5, (x0, yc, zl), grip, "settle", yaw),
            Waypoint(2.5, (*rack(yc), zl), grip, "carry in between the posts", ry),
            Waypoint(0.8, (*rack(yc), zl), grip, "settle", ry),
            Waypoint(2.5, (*rack(ys), zl), grip, "ease both ends out into the holes", ry),
            Waypoint(0.5, (*rack(ys), zl), grip, "settle", ry),
            Waypoint(1.5, (*rack(ys), zl), open_, "let go together", ry),
            Waypoint(1.2, (*rack(ys), up), open_, "up off the rail", ry),
            Waypoint(1.3, (x0, ys, up), open_, "back out of the rack", yaw),
        ]
    return plans


# -- offline self-test ------------------------------------------------------------------------------------------

def _rail(cx, cy, axis_z, length=REST_L, tilt_deg=0.0, yaw_deg=0.0, vel=(0.0, 0.0, 0.0)):
    """Hand-built states of both members: the rail's frame (midway between the member ends, on the pins' axis) at
    (cx, cy, axis_z), ``length`` end to end, turned ``yaw_deg`` about the vertical, then its +y (rear) end raised by
    ``tilt_deg``. Posed by the kit's own forward kinematics through the slider."""
    import dataclasses

    yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, math.cos(tilt), -math.sin(tilt)], [0, math.sin(tilt), math.cos(tilt)]])
    r = rz @ rx
    outer = np.array([cx, cy, axis_z]) + r @ np.array([0.0, length / 2 - OUTER[1] / 2, OUTER_AT[2]])
    states = kit.fake_states(q={"rail_slide": length - REST_L}, moved={"rail_outer": (outer, mk.mat_to_quat(r))})
    return {n: dataclasses.replace(st, lin_vel=np.asarray(vel, float)) for n, st in states.items()}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states (both members posed through the slider by the kit)."""
    seat_z = HOLE_Z[TARGET] - (HOLE - PEG[2]) / 2  # a seated pin rests on the bottom of its hole
    seated = _rail(RACK_X, 0.0, seat_z, SEATED_L)
    grip_r = (RACK_X, -(SEATED_L / 2 - GRIP_FROM_END), seat_z + 0.004)
    front3_rear4 = _rail(RACK_X, 0.0, (seat_z + HOLE_Z[TARGET + 1] - 0.003) / 2, SEATED_L + 0.0008,
                         tilt_deg=math.degrees(math.atan2(PITCH, SEATED_L + 2 * PEG[1] / 2)))
    cases = {
        "start, on the table": (_rail(START_X, 0.0, TOP + AXIS_UP), (), 0.0),
        "lifted and pulled in, held between the posts": (_rail(RACK_X, 0.0, HOLE_Z[TARGET], 0.22), (grip_r,), 0.0),
        "seated at the marked hole, let go": (seated, (), 1.0),
        "seated, a gripper still on the inner member": (seated, (grip_r,), 0.0),
        "seated, the grippers backed off above": (seated, ((RACK_X, 0.098, seat_z + 0.05),
                                                           (RACK_X, -0.098, seat_z + 0.05)), 1.0),
        "seated but still sliding": (_rail(RACK_X, 0.0, seat_z, SEATED_L, vel=(0.0, 0.02, 0.0)), (), 0.0),
        "both pins in hole 5, level (wrong rack position)": (_rail(RACK_X, 0.0, HOLE_Z[5] - 0.003, SEATED_L), (), 0.5),
        "both pins in hole 1, level": (_rail(RACK_X, 0.0, HOLE_Z[1] - 0.003, SEATED_L), (), 0.5),
        "front pin in hole 3, rear pin in hole 4 (skewed)": (front3_rear4, (), 0.25),
        "pins at mismatched holes 3 and 2": (_rail(RACK_X, 0.0, seat_z - PITCH / 2, SEATED_L + 0.0008,
                                                   tilt_deg=-math.degrees(math.atan2(PITCH, SEATED_L + 0.01))),
                                             (), 0.25),
        "one-arm attempt: front pin in hole 3, rail full length, rear end swung out in front of the rear post": (
            _rail(RACK_X - 0.155 * math.sin(math.radians(32)), -FACE_Y - 0.004 + 0.155 * math.cos(math.radians(32)),
                  seat_z, REST_L, yaw_deg=32.0), (), 0.25),
        "one-arm attempt: full-length rail laid across the front of the posts": (
            _rail(RACK_X - 0.03, 0.0, HOLE_Z[TARGET], REST_L), (), 0.0),
        "pins only 3 mm through the posts": (_rail(RACK_X, 0.0, seat_z, SEATED_L - 0.014), (), 0.0),
        "pins 6 mm off the hole column": (_rail(RACK_X - 0.006, 0.0, seat_z, SEATED_L), (), 0.0),
        "seated end for end (rear pin in the front post)": (_rail(RACK_X, 0.0, seat_z, SEATED_L, yaw_deg=180.0),
                                                            (), 1.0),
        "seated, front pin 3 mm higher in its hole (level within 1.5 degrees)": (
            _rail(RACK_X, 0.0, seat_z + 0.0015, SEATED_L, tilt_deg=-math.degrees(math.atan2(0.003, SEATED_L))), (), 1.0),
        "dropped on the table under the rack": (_rail(RACK_X, 0.0, TOP + AXIS_UP, 0.24), (), 0.0),
    }

    def run(*frames, tcps=()):
        _MEM.clear()  # a new episode, as the embodiment's reset() does
        mk.new_episode()
        globals()["_tcps"] = lambda: [np.asarray(t, float) for t in tcps]
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    seq = {
        "held at the marked height, then let go and seated": (
            (_rail(START_X, 0.0, TOP + AXIS_UP), _rail(RACK_X, 0.0, HOLE_Z[TARGET], 0.22), seated), 1.0),
        "seated at the end of a run after a skewed try": ((front3_rear4, seated), 1.0),
        "seated, then knocked out onto the table before the end": ((seated, _rail(RACK_X, 0.0, TOP + AXIS_UP,
                                                                                  0.24)), 0.0),
    }
    try:
        for name, (st, tcps, want) in cases.items():
            got, ok = run(st, tcps=tcps)
            assert abs(got - want) < 1e-9 and ok == (want == 1.0), (name, got, ok, want)
        for name, (frames, want) in seq.items():
            got, ok = run(*frames)
            assert abs(got - want) < 1e-9 and ok == (want == 1.0), (name, got, ok, want)
    finally:
        globals()["_tcps"] = _tcps_live
        _MEM.clear()
        mk.new_episode()
    assert distance(seated) < 0.002 and distance(_rail(START_X, 0.0, TOP + AXIS_UP)) > 0.2
    return len(cases) + len(seq)


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
