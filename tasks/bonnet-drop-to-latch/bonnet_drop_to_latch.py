"""On the station table, in front of the right arm, stands the front end of a car body (scaled down): a grey front
panel, a fender on each side, and the bonnet, a painted steel panel hinged along its rear edge (the far side). The
bonnet is down but not latched: it rests ajar on the latch's catch, about 4 degrees open, with its front edge 2 cm
above the fender line. A short release tab sticks out of the middle of its front edge. Under the front edge is the
bonnet latch: a striker (a post with a foot) hanging from the bonnet, a spring catch that snaps over the foot when the
foot is driven past it, and a sprung rubber buffer the closed bonnet sits on. The job is to close the bonnet. Scored
from the poses of the bonnet and the latch parts at the end.

The catch needs a real blow to snap over the striker: the bonnet's weight alone leaves it resting ajar on the catch,
so it must be lifted and let fall. Dropped from too high it slams: the striker drives the buffer down past a second,
deeper catch (the latch's over-travel catch), which holds the bonnet pulled down below the fender line. Pushing the
bonnet shut with a gripper is a press (it dents the panel) and scores 0 for the trial.

Mechanisms (mechkit.py, in this folder, adds the joints to the stock runtime in memory): the bonnet hinge, the two
catches (sprung slides) and the buffer (a sprung slide). Every readout is geometry from the parts' poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/bonnet-drop-to-latch/bonnet_drop_to_latch.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (installs the joint hooks and the per-episode memory)

from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Right arm base at (0, -0.305, 0.75).
YB = -0.100  # bonnet centre line
HX, HZ = 0.480, TOP + 0.065  # hinge line (along y) at the bonnet's rear edge, at the panel's mid-thickness
L, W, T = 0.280, 0.220, 0.004  # bonnet panel: length (x), width (y), thickness
BONNET_KG = 0.30
TAB = (0.065, 0.026, 0.010)  # release tab out of the front edge: length (x), width (y), thickness (z)
GRIP_IN = 0.045  # the demo grips the tab this far in front of the front edge

# Striker (on the bonnet, local frame: origin on the hinge line, x along the panel toward the hinge... the panel
# runs from local x -L to 0). A post hangs from the underside 3 cm behind the front edge, with a foot at its bottom.
SX = -0.250  # post centre, local x
POST = (0.008, 0.016, 0.040)
FOOT = (0.020, 0.016, 0.004)
FOOT_TOP = -T / 2 - POST[2] + FOOT[2]  # local z of the foot's top face
FOOT_BOT = FOOT_TOP - FOOT[2]
FOOT_FRONT, FOOT_BACK = SX - FOOT[0] / 2, SX + FOOT[0] / 2  # local x of the foot's ends

# Main catch (in front of the foot, slides along x, spring pushes it +x over the foot): one plate leaning back
# 30 deg, so its striker face is a 60 deg ramp and its underside the hook. Its frame's origin is the plate's
# bottom front edge.
P_LEAN, P_PLATE = 30.0, (0.010, 0.016, 0.022)
HOOK = 0.004  # the catch's edge reaches 4 mm over the foot's front end
P_F0, P_K, P_TRAVEL, P_KG = 1.2, 150.0, 0.009, 0.020  # preload N, spring N/m, travel m, mass kg
# Over-travel catch (behind the foot, slides along -x): a 60 deg ramp, its edge 2.5 mm over the foot's back end and
# OT below the foot's top at the flush position; reached only when the buffer is driven down by a hard blow.
S_LEAN, S_PLATE = 30.0, (0.010, 0.016, 0.022)
HOOK_S = 0.0025
OT = 0.004
S_F0, S_K, S_TRAVEL, S_KG = 1.5, 150.0, 0.009, 0.020
# Buffer under the foot (vertical slide, spring pushes it up against its stop at the flush position).
BUF = (0.008, 0.012, 0.006)
B_F0, B_K, B_TRAVEL, B_KG = 3.0, 400.0, 0.015, 0.020

FLUSH_CLEAR = 0.001  # foot top this far below the main catch's underside (over the foot's end) when flush


def _rot_y(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def _qy(deg):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def _world(local_x, local_z, deg):
    """World (x, z) of a bonnet-frame point with the bonnet open ``deg`` degrees."""
    r = _rot_y(deg)
    p = r @ np.array([local_x, 0.0, local_z])
    return HX + p[0], HZ + p[2]


# Flush geometry (bonnet angle 0)
_FOOT_TOP_W = HZ + FOOT_TOP
P_EDGE = (HX + FOOT_FRONT + HOOK, _FOOT_TOP_W + HOOK * math.tan(math.radians(P_LEAN)) + FLUSH_CLEAR)
S_EDGE = (HX + FOOT_BACK - HOOK_S, _FOOT_TOP_W - OT)
BUF_TOP = HZ + FOOT_BOT


def _ramp_z_main(x):
    """Height of the main catch's ramp face above world x (catch at rest)."""
    return P_EDGE[1] + (P_EDGE[0] - x) * math.tan(math.radians(90.0 - P_LEAN))


def ajar_deg() -> float:
    """The angle at which the foot's front bottom corner rests on the main catch's ramp (bisection)."""
    lo, hi = 0.0, 15.0
    for _ in range(60):
        mid = (lo + hi) / 2
        x, z = _world(FOOT_FRONT, FOOT_BOT, mid)
        if z > _ramp_z_main(x):
            hi = mid
        else:
            lo = mid
    return hi


START_DEG = ajar_deg() + 0.15
OPEN_MAX = 60.0

STEEL_RED = (0.62, 0.10, 0.10)
GREY = (0.42, 0.43, 0.45)
DARK = (0.12, 0.12, 0.13)
ZINC = (0.70, 0.71, 0.72)
RUBBER = (0.06, 0.06, 0.06)


def build(kit, name="", yc=YB, width=W, tab=True, start_deg=None, hinge_friction=0.0):
    """Adds one bonnet with its latch to ``kit``; returns the part/joint names. (A builder, so a calibration rig can
    lay out several.)"""
    n = {k: f"{k}{name}" for k in ("bonnet", "hinge", "catch", "catch_slide", "otcatch", "otcatch_slide", "buffer",
                                   "buffer_slide")}
    boxes = [mk.box((L, width, T), at=(-L / 2, 0, 0)),
             mk.box((POST[0], POST[1], POST[2]), at=(SX, 0, -T / 2 - POST[2] / 2), color=ZINC),
             mk.box(FOOT, at=(SX, 0, FOOT_TOP - FOOT[2] / 2), color=ZINC)]
    if tab:
        boxes.append(mk.box(TAB, at=(-L - TAB[0] / 2 + 0.002, 0, 0), color=DARK))
    kit.part(n["bonnet"], (HX, yc, HZ), boxes, mass=BONNET_KG, color=STEEL_RED)
    kit.hinge(n["hinge"], None, n["bonnet"], anchor=(HX, yc, HZ), axis=(0, 1, 0), lower_deg=-3.0, upper_deg=OPEN_MAX,
              start_deg=START_DEG if start_deg is None else start_deg, friction=hinge_friction)
    # main catch: plate leaning back; frame origin at its bottom front edge
    rp = _rot_y(-P_LEAN)
    c = -rp @ np.array([P_PLATE[0] / 2, 0, -P_PLATE[2] / 2])
    kit.part(n["catch"], (P_EDGE[0], yc, P_EDGE[1]), [mk.box(P_PLATE, at=tuple(c), quat=_qy(-P_LEAN))], mass=P_KG,
             color=ZINC)
    kit.slider(n["catch_slide"], None, n["catch"], axis=(1, 0, 0), lower=-P_TRAVEL, upper=0.0,
               stiffness=P_K, rest=P_F0 / P_K)
    rs = _rot_y(S_LEAN)
    c = -rs @ np.array([-S_PLATE[0] / 2, 0, -S_PLATE[2] / 2])
    kit.part(n["otcatch"], (S_EDGE[0], yc, S_EDGE[1]), [mk.box(S_PLATE, at=tuple(c), quat=_qy(S_LEAN))], mass=S_KG,
             color=DARK)
    kit.slider(n["otcatch_slide"], None, n["otcatch"], axis=(-1, 0, 0), lower=-S_TRAVEL, upper=0.0,
               stiffness=S_K, rest=S_F0 / S_K)
    kit.part(n["buffer"], (HX + SX, yc, BUF_TOP), [mk.box(BUF, at=(0, 0, -BUF[2] / 2))], mass=B_KG, color=RUBBER)
    kit.slider(n["buffer_slide"], None, n["buffer"], axis=(0, 0, 1), lower=-B_TRAVEL, upper=0.0,
               stiffness=B_K, rest=B_F0 / B_K)
    return n


def fixtures(yc=YB, width=W, name=""):
    """The fixed body round one bonnet: front panel, fenders flush with the closed bonnet, hinge brackets, latch
    housing. Nothing touches the bonnet's swept space."""
    fx0, fx1 = HX - L, HX
    return [
        Box(f"fender_l{name}", pos=((fx0 + fx1) / 2, yc + width / 2 + 0.004 + 0.025, (TOP + HZ + T / 2) / 2),
            size=(L, 0.050, HZ + T / 2 - TOP), color=STEEL_RED),
        Box(f"fender_r{name}", pos=((fx0 + fx1) / 2, yc - width / 2 - 0.004 - 0.025, (TOP + HZ + T / 2) / 2),
            size=(L, 0.050, HZ + T / 2 - TOP), color=STEEL_RED),
        Box(f"scuttle{name}", pos=(HX + 0.030, yc, (TOP + HZ - 0.010) / 2), size=(0.040, width, HZ - 0.010 - TOP),
            color=GREY),
        Box(f"latch_plate{name}", pos=(HX + SX, yc, TOP + 0.002), size=(0.080, 0.050, 0.004), color=DARK),
    ]


kit = mk.Kit()
N = build(kit)
OBJECTS = [*kit.objects(), *fixtures()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Owner's-manual practice: lower the bonnet to a short height above the latch and let it drop so it latches; do not
# press it down (it dents) and do not slam it. Everything below is geometry from the parts' poses: the catches over
# the striker foot (latched / over-travel), the bonnet's angle, and where the grippers are.

OVERLAP_MIN = 0.0015  # a catch counts as over the foot once its edge reaches 1.5 mm past the foot's end
PRESS_DEG = 8.0  # near closed: a gripper on the panel top below this angle is pressing it
PRESS_GAP = 0.020
_TEST_TCPS = None


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else [np.asarray(t, float) for t in _TEST_TCPS]


def _near(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())


def angle_deg(objects, n=N) -> float:
    return kit.q_deg(objects, n["hinge"])


def latched(objects, n=N) -> bool:
    """The main catch's edge is over the foot (past its front end) and the foot is below that edge."""
    edge = kit.point(objects, n["catch"], (0, 0, 0))
    foot = kit.point(objects, n["bonnet"], (FOOT_FRONT, 0, FOOT_TOP))
    return bool(edge[0] >= foot[0] + OVERLAP_MIN and foot[2] < edge[2])


def overtravel(objects, n=N) -> bool:
    """The over-travel catch's edge is over the foot's back end and the foot below it: the bonnet was slammed."""
    edge = kit.point(objects, n["otcatch"], (0, 0, 0))
    foot = kit.point(objects, n["bonnet"], (FOOT_BACK, 0, FOOT_TOP))
    return bool(edge[0] <= foot[0] - OVERLAP_MIN and foot[2] < edge[2])


def _pressing(objects, n=N) -> bool:
    """A gripper's tool point on (within 2 cm above) the panel's top face while the bonnet is within 8 deg of
    closed."""
    if angle_deg(objects, n) > PRESS_DEG:
        return False
    o = kit.point(objects, n["bonnet"], (-L, 0, T / 2))
    ex = kit.point(objects, n["bonnet"], (0, 0, T / 2)) - o
    ez = kit.point(objects, n["bonnet"], (-L, 0, T / 2 + 1)) - o
    for t in _tcps():
        d = np.asarray(t) - o
        u, h = float(d @ ex) / float(ex @ ex), float(d @ ez)
        if 0.0 <= u <= 1.0 and abs(float(t[1]) - float(o[1])) <= W / 2 and -0.005 <= h <= PRESS_GAP:
            return True
    return False


def _update(objects, n=N):
    """Per-trial latches (mk.MEMORY is cleared at every reset): pressed, slammed, and how the latch first engaged."""
    m = mk.MEMORY
    if _pressing(objects, n):
        m["pressed"] = True
    if latched(objects, n) and not m.get("engaged"):
        m["engaged"] = True
        if _near(objects[n["bonnet"]]):  # it latched with a gripper on it: pushed shut
            m["pressed"] = True
    if overtravel(objects, n):
        m["slammed"] = True
    return m


def score(objects) -> float:
    """1.0: latched, never pressed and never slammed, the bonnet let go (both grippers 2.5 cm clear) and still.
    0.5: latched but slammed (the over-travel catch engaged at any time), or latched but still held / moving.
    0: not latched (ajar or open), or pressed shut by a gripper at any time in the trial."""
    m = _update(objects)
    if m.get("pressed") or not latched(objects):
        return 0.0
    b = objects[N["bonnet"]]
    if m.get("slammed") or _near(b) or not b.resting():
        return 0.5
    return 1.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    """How far (m) the striker foot still is above the main catch's edge (0 once below it)."""
    edge = kit.point(objects, N["catch"], (0, 0, 0))
    foot = kit.point(objects, N["bonnet"], (FOOT_FRONT, 0, FOOT_TOP))
    return float(max(0.0, foot[2] - edge[2]))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's fixed geometry (the bonnet's starting angle is its resting angle on the catch, computed
# from the drawings above); it reads no joint or simulator state. The right arm grips the release tab across its
# width, lifts the front along the hinge arc to the drop angle, opens its fingers quickly and backs away.

DROP_DEG = 25.0
SQUEEZE = 0.006
TCP_DOWN = 0.0  # TCP at the tab's mid-thickness


def _tab_tcp(deg):
    x, z = _world(-L - GRIP_IN, 0.0, deg)
    return (float(x), YB, float(z + TCP_DOWN))


def demo():
    """Right arm only: over the tab, down, close on it, lift the front along the hinge arc to 25 deg, settle, let go
    (fingers open in 0.2 s), back up and away. The left arm stays parked."""
    o = FINGER_OPEN
    g = -(TAB[1] - SQUEEZE) / 2
    a0 = START_DEG
    p0 = _tab_tcp(a0)
    arc = [a0 + (DROP_DEG - a0) * k / 6 for k in range(1, 7)]
    right = [
        Waypoint(2.0, (p0[0] - 0.01, p0[1], p0[2] + 0.06), o, "over the tab"),
        Waypoint(1.5, (p0[0], p0[1], p0[2] + 0.03), o, "descend"),
        Waypoint(1.0, p0, o, "down beside the tab"),
        Waypoint(0.5, p0, o, "settle"),
        Waypoint(1.0, p0, g, "close on the tab"),
        Waypoint(0.5, p0, g, "settle closed"),
        *(Waypoint(0.5, _tab_tcp(a), g, f"lift to {a:.0f} deg") for a in arc),
        Waypoint(1.0, _tab_tcp(DROP_DEG), g, "hold at the drop angle"),
        Waypoint(0.2, _tab_tcp(DROP_DEG), o, "let go"),
        Waypoint(0.6, _tab_tcp(DROP_DEG), o, "fingers open, wait"),
        Waypoint(1.5, (0.220, -0.220, TOP + 0.18), o, "back away"),
        Waypoint(3.0, (0.220, -0.220, TOP + 0.18), o, "park"),
    ]
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), o, "park"), Waypoint(sum(w.duration for w in right) - 2.0,
                                                                      (0.150, 0.250, 0.950), o, "park")]
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built latch poses (forward kinematics of the bonnet, catches and buffer)."""
    import dataclasses

    global _TEST_TCPS
    catch_open = -(HOOK + 0.0015)  # the main catch pushed back clear of the foot

    def st(deg, catch=0.0, ot=0.0, buf=0.0, moving=False):
        s = kit.fake_states(q={N["hinge"]: math.radians(deg), N["catch_slide"]: catch, N["otcatch_slide"]: ot,
                               N["buffer_slide"]: buf})
        if moving:
            s[N["bonnet"]] = dataclasses.replace(s[N["bonnet"]], ang_vel=np.array([0.0, 2.0, 0.0]))
        return s

    slam_deg = -math.degrees((OT + 0.002) / abs(SX))
    start = st(START_DEG)
    lifted = st(DROP_DEG)
    falling = st(6.0, moving=True)
    passing = st(1.0, catch=catch_open)  # the foot pushing the catch back on its way down
    shut = st(0.0)
    deep = st(slam_deg, catch=0.0, ot=0.0, buf=-(OT + 0.002))
    deep_passing = st(slam_deg * 0.6, ot=-(HOOK_S + 0.0015), buf=-OT * 0.6)
    far = (0.22, -0.22, TOP + 0.18)
    tab_at = _tab_tcp
    panel_top = tuple(kit.point(shut, N["bonnet"], (-L / 2, 0, T / 2 + 0.010)))

    cases = {  # name: ([(state, tcps), ...] in one trial, final score, success)
        "start: resting ajar on the catch": ([(start, [])], 0.0, False),
        "start, gripped at the tab": ([(start, [tab_at(START_DEG)])], 0.0, False),
        "lifted to 25 deg": ([(start, [tab_at(START_DEG)]), (lifted, [tab_at(DROP_DEG)])], 0.0, False),
        "dropped from 25 deg, latched, gripper clear": (
            [(start, [tab_at(START_DEG)]), (lifted, [tab_at(DROP_DEG)]), (falling, [tab_at(DROP_DEG)]),
             (passing, [tab_at(DROP_DEG)]), (shut, [tab_at(DROP_DEG)]), (shut, [far])], 1.0, True),
        "latched, still swinging": ([(start, []), (passing, [far]), (st(0.0, moving=True), [far])], 0.5, False),
        "latched with the gripper still on the tab (lowered by hand onto it)": (
            [(start, [tab_at(START_DEG)]), (passing, [tab_at(1.0)]), (shut, [tab_at(0.0)])], 0.0, False),
        "pushed shut on the panel top": ([(start, []), (st(3.0), [panel_top]), (shut, [panel_top]), (shut, [far])],
                                         0.0, False),
        "dropped clean, then pressed on the panel": ([(start, []), (passing, [far]), (shut, [far]),
                                                      (shut, [panel_top]), (shut, [far])], 0.0, False),
        "slammed: the over-travel catch held it down": ([(start, []), (lifted, [far]), (deep_passing, [far]),
                                                         (deep, [far])], 0.5, False),
        "slammed, then pulled back up to flush": ([(start, []), (deep_passing, [far]), (deep, [far]),
                                                   (shut, [far])], 0.5, False),
        "lowered gently by hand: still ajar on the catch": ([(start, [tab_at(START_DEG)]), (start, [far])], 0.0,
                                                            False),
        "foot below the catch edge but the catch pushed back (not latched)": ([(passing, [far])], 0.0, False),
        "open, let go": ([(st(30.0), [far])], 0.0, False),
    }
    try:
        for name, (seq, want, want_success) in cases.items():
            mk.new_episode()
            for s, t in seq[:-1]:
                _TEST_TCPS = t
                score(s)
            s, t = seq[-1]
            _TEST_TCPS = t
            got, got_success = score(s), success(s)
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        # geometry sanity
        assert not latched(start) and latched(shut) and not overtravel(shut) and overtravel(deep), "latch geometry"
        assert 2.0 < START_DEG < 8.0, START_DEG
        # a new trial forgets the last one's press
        mk.new_episode()
        _TEST_TCPS = [panel_top]
        score(shut)
        mk.new_episode()
        _TEST_TCPS = [far]
        assert score(passing) == 0.0 and score(shut) == 1.0, "reset"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 2


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass (bonnet rests ajar at {START_DEG:.2f} deg)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
