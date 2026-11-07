"""On the station table, a single-phase branch circuit runs from a grey breaker panel on the left to a black rack
PDU on the right: two insulated single-core conductors, stretched level side by side 9 cm above the table, the
black line conductor nearer the arms and the white neutral 4.4 cm behind it. In front of them, on the right, a
split-core current sensor (a current transformer) hangs on a parking peg with its jaw swung open: a grey
rectangular frame whose top carries the sensor housing with a white arrow, and whose near side and bottom form
one hinged jaw. The job is to fit the sensor so it measures the current the PDU draws, then let go. Scored from
the sensor's and the jaw's poses.

The jaw is a hinged part (mechkit.py, in this folder, adds the hinge to the stock runtime in memory).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/ct-around-one-conductor/ct_around_one_conductor.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (installs the hinge hooks)

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
# -- the circuit (fixed) --
COND = 0.016  # conductor section (square stand-in for a round insulated single core)
COND_Z = TOP + 0.090  # conductor centres
COND_TOP = COND_Z + COND / 2
L_X, N_X = 0.280, 0.324  # line (black, near) and neutral (white, far) conductor centres
RUN_Y = (-0.190, 0.190)  # the conductors run from the PDU (right, -y) to the panel (left, +y)
BLOCK = (0.090, 0.060, 0.150)  # panel and PDU: depth, width, height
BLOCK_X = 0.302

# -- the sensor, in its own frame: origin on the top bar's underside, centred where a conductor it hangs on sits;
#    X across the window toward the fixed side bar, Y through the window (the arrow points along -Y), Z up --
BAR = 0.012  # frame bar section in the window plane
DEPTH = 0.016  # frame depth along Y
WIN_W, WIN_H = 0.084, 0.028  # window: wide enough for both conductors (6.0 cm over both), 2.8 cm tall
X_FAR = COND / 2 + 0.008  # the fixed side bar's inner face: 8 mm clear of a conductor hanging in the notch
X_NEAR = X_FAR - WIN_W  # the jaw's side bar inner face
HINGE_X = X_NEAR - BAR / 2  # the jaw's side bar runs up past the top bar's end and hinges at its top outer corner
HINGE_Z = BAR
LIP = (0.003, DEPTH, 0.003)  # the notch lips under the top bar, 3 mm clear of the conductor on each side
LIP_X = COND / 2 + 0.003 + LIP[0] / 2
HOUSING = (0.030, 0.024, 0.022)
HOUSING_X = 0.019  # the housing sits toward the fixed side, so the closed sensor hangs level on one core
CT_KG, JAW_KG = 0.300, 0.015
JAW_FRICTION = 0.0065  # N*m at the jaw hinge: more than the jaw's weight torque when shut (0.0044), so it stays shut
JAW_OPEN_DEG = 38.0  # the jaw's own gravity rest: it hangs there, swung open, when nothing holds it

GREY = (0.46, 0.48, 0.50)
HOUSING_C = (0.30, 0.34, 0.40)
WHITE = (0.95, 0.95, 0.95)
BLACK = (0.06, 0.06, 0.06)
PANEL_GREY = (0.62, 0.63, 0.64)
STEEL = (0.55, 0.57, 0.60)

_body_boxes = [
    mk.box((X_FAR + BAR - X_NEAR, DEPTH, BAR), at=((X_FAR + BAR + X_NEAR) / 2, 0.0, BAR / 2)),
    mk.box((BAR, DEPTH, WIN_H + BAR), at=(X_FAR + BAR / 2, 0.0, -(WIN_H + BAR) / 2)),
    *(mk.box(LIP, at=(s * LIP_X, 0.0, -LIP[2] / 2)) for s in (-1.0, 1.0)),
    mk.box(HOUSING, at=(HOUSING_X, 0.0, BAR + HOUSING[2] / 2), color=HOUSING_C),
    # the white polarity arrow on the housing's top, pointing along -Y (toward the load when fitted)
    mk.box((0.004, 0.014, 0.001), at=(HOUSING_X, 0.002, BAR + HOUSING[2] + 0.0005), color=WHITE),
    *(mk.box((0.003, 0.009, 0.001), at=(HOUSING_X + s * 0.0028, -0.0065, BAR + HOUSING[2] + 0.0005),
             quat=mk.yaw_quat(s * math.radians(-35)), color=WHITE) for s in (-1.0, 1.0)),
]
_jaw_boxes = [
    mk.box((BAR, DEPTH, WIN_H + 2 * BAR), at=(HINGE_X, 0.0, -WIN_H / 2)),
    mk.box((WIN_W, DEPTH, BAR), at=((X_NEAR + X_FAR) / 2, 0.0, -WIN_H - BAR / 2)),
]

# Where the sensor starts: hanging on the parking peg, turned so its arrow points away from the arms (+x).
PARK_Y = -0.135
PARK_X = 0.190
PARK_TOP = COND_TOP + 0.015
START_YAW = math.pi / 2


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


_START_ORIGIN = np.array([PARK_X, PARK_Y, PARK_TOP])
kit = mk.Kit()
kit.part("sensor", _START_ORIGIN, _body_boxes, mass=CT_KG, color=GREY, yaw=START_YAW)
kit.part("jaw", _START_ORIGIN, _jaw_boxes, mass=JAW_KG, color=GREY, yaw=START_YAW)
kit.hinge("jaw_hinge", "sensor", "jaw", anchor=_START_ORIGIN + _rz(START_YAW) @ np.array([HINGE_X, 0.0, HINGE_Z]),
          axis=_rz(START_YAW) @ np.array([0.0, 1.0, 0.0]), lower_deg=0.0, upper_deg=115.0,
          start_deg=JAW_OPEN_DEG, friction=JAW_FRICTION)

_run_len = RUN_Y[1] - RUN_Y[0]
FIXTURES = [
    Box("line_conductor", pos=(L_X, 0.0, COND_Z), size=(COND, _run_len, COND), color=BLACK),
    Box("neutral_conductor", pos=(N_X, 0.0, COND_Z), size=(COND, _run_len, COND), color=WHITE),
    Box("breaker_panel", pos=(BLOCK_X, RUN_Y[1] + BLOCK[1] / 2, TOP + BLOCK[2] / 2), size=BLOCK, color=PANEL_GREY),
    Box("rack_pdu", pos=(BLOCK_X, RUN_Y[0] - BLOCK[1] / 2, TOP + BLOCK[2] / 2), size=BLOCK, color=BLACK),
    # parking peg: a steel post whose top sits in the sensor's notch
    Box("park_peg", pos=(PARK_X, PARK_Y, (TOP + PARK_TOP) / 2), size=(0.012, 0.020, PARK_TOP - TOP), color=STEEL),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Current-sensor installation guides: a split-core CT goes around ONE current-carrying conductor (the line), never
# around a whole cable or line and neutral together, because their currents cancel and it reads about zero; its
# arrow (P1 -> P2) points toward the load; the jaw is closed fully. All of it is measured from the poses: where
# each conductor's centreline crosses the sensor's plane, in the sensor's own frame; the jaw angle; the arrow.

JAW_SHUT_DEG = 5.0
ARROW_TOL_DEG = 25.0
LOAD_DIR = np.array([0.0, -1.0, 0.0])  # toward the PDU
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else [np.asarray(t, float) for t in _TEST_TCPS]


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())


def _frame(objects):
    pos, quat = kit.pose(objects, "sensor")
    return np.asarray(pos, float), mk.quat_to_mat(quat)


def _encircles(objects, cond_x) -> bool:
    """The conductor's centreline passes through the sensor's window rectangle (in the sensor's frame)."""
    p, r = _frame(objects)
    ny = r[:, 1]  # the window's normal (sensor Y) in the world
    if abs(ny[1]) < 0.2:  # sensor plane nearly parallel to the conductor: it cannot be around it
        return False
    a = np.array([cond_x, 0.0, COND_Z])
    s = float(np.dot(p - a, ny) / ny[1])  # where along the run the conductor meets the sensor plane
    if not RUN_Y[0] < s < RUN_Y[1]:
        return False
    local = r.T @ (a + np.array([0.0, s, 0.0]) - p)
    return bool(X_NEAR < local[0] < X_FAR and -WIN_H < local[2] < 0.0)


def _arrow_deg(objects) -> float:
    """Angle between the sensor's arrow (its -Y) and the direction toward the PDU, degrees."""
    _, r = _frame(objects)
    c = float(np.clip(np.dot(-r[:, 1], LOAD_DIR), -1.0, 1.0))
    return math.degrees(math.acos(c))


def _jaw_deg(objects) -> float:
    return kit.q_deg(objects, "jaw_hinge")


def score(objects) -> float:
    """With the sensor and its jaw let go of (both TCPs 2.5 cm clear) and still: 1.0 for the jaw shut (within 5 deg)
    around the line conductor only, arrow within 25 deg of the PDU direction; 0.5 for the same with the arrow off
    (reversed); 0.25 for the jaw shut around the neutral only, or the sensor hanging on the line only with its jaw
    open; 0 when the jaw is shut around both conductors (net current zero), or anything else."""
    ct, jaw = objects["sensor"], objects["jaw"]
    if not (_released(ct) and _released(jaw) and ct.resting() and jaw.resting()):
        return 0.0
    line, neutral = _encircles(objects, L_X), _encircles(objects, N_X)
    shut = _jaw_deg(objects) <= JAW_SHUT_DEG
    if shut and line and not neutral:
        return 1.0 if _arrow_deg(objects) <= ARROW_TOL_DEG else 0.5
    if shut and neutral and not line:
        return 0.25
    if not shut and line and not neutral:
        return 0.25
    return 0.0


def success(objects) -> bool:
    """Shut around the line conductor alone, arrow toward the PDU, let go and still; holds 1 s."""
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Metres from the sensor's notch to its fitted place on the line conductor, plus the jaw edge's arc to shut."""
    p, _ = _frame(objects)
    goal = np.array([L_X, FIT_Y, COND_TOP])
    return float(np.linalg.norm(p - goal) + math.radians(max(0.0, _jaw_deg(objects))) * (WIN_H + BAR))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's fixed geometry (where the peg, the conductors and the jaw hinge are when the
# sensor sits where the demo puts it); it reads no joint or simulator state. The right arm lifts the sensor off its
# peg by the housing, turns it a quarter turn clockwise (arrow toward the PDU), lowers it over the line
# conductor so the fixed side bar drops between the two cores, lets go, and swings the jaw shut with its closed
# fingers (an arc 3.6 cm out from the hinge).

TIP = 0.013
SQUEEZE = 0.008
HALF = 0.008  # closed pads' half thickness along their closing axis (yaw pi/2: along x), near the tips
YAW_JAW = math.pi / 2  # fingers close along x for the jaw push: the pads' 38 mm width spans the jaw's depth
GAP0 = 0.015
# How far the pads' edge runs into the jaw's face line at each step of the arc (m), so the closed fingers keep
# about 3 mm of contact along the swing; worked out offline from the finger pads' collision hulls and the jaw's
# geometry at each angle (the pad tips are rounded, so the offset changes along the arc).
JAW_GAP = {2: -0.009, 0: -0.010}
PRESS = -0.010
REACH = 0.036
ARC = (33, 27, 21, 15, 10, 5, 2, 0)
C = FINGER_CLOSED
FIT_Y = -0.080  # where along the run the demo fits the sensor
X_TRIM = 0.0  # the held sensor's lag in x (measured in sim)
Z_HOLD = 0.0015  # hold the notch this far above the conductor while the jaw is shut
GRIP_DZ = BAR + 0.005  # pad tips 5 mm up the housing
PARK = {"left": (0.200, 0.250, 0.950), "right": (0.200, -0.250, 0.950)}


def _housing_world(origin, yaw):
    return np.asarray(origin, float) + _rz(yaw) @ np.array([HOUSING_X, 0.0, 0.0])


def _hold_tcp(origin, yaw, dz=0.0):
    h = _housing_world(origin, yaw)
    return (float(h[0]), float(h[1]), float(origin[2] + GRIP_DZ + TIP + dz))


FIT_ORIGIN = np.array([L_X, FIT_Y, COND_TOP])
Y_LAG = -0.004  # the hung sensor sits 4 mm toward -y of where the arm set it (measured in sim)
HINGE_FIT = FIT_ORIGIN + np.array([HINGE_X, Y_LAG, HINGE_Z])  # the jaw hinge with the sensor hung (yaw 0)


def _jaw_tcp(deg, gap):
    """TCP putting the closed pads' edge ``gap`` off the jaw side bar's outer face, ``REACH`` from the hinge,
    with the jaw open ``deg`` degrees."""
    a = math.radians(deg)
    d = np.array([-math.sin(a), 0.0, -math.cos(a)])  # along the jaw's side bar, hinge to its lower end
    n = np.array([-math.cos(a), 0.0, math.sin(a)])  # its outer face's normal
    edge = HINGE_FIT + REACH * d + (BAR / 2 + gap) * n
    return tuple(float(v) for v in edge + np.array([-HALF, 0.0, TIP]))


def demo():
    """One arm (the right): pick the sensor off the peg by its housing, turn it a quarter turn, hang it on the line
    conductor and let go; then come down beside it and swing the jaw shut with the closed fingers. The left arm
    stays parked."""
    open_ = -(HOUSING[1] + 0.016) / 2
    grip = -(HOUSING[1] - SQUEEZE) / 2
    pick = _hold_tcp(_START_ORIGIN, START_YAW)
    lift = _hold_tcp(_START_ORIGIN, START_YAW, 0.070)
    fit_o = FIT_ORIGIN + np.array([X_TRIM, 0.0, Z_HOLD])
    over = _hold_tcp(fit_o, 0.0, WIN_H + BAR + 0.010)
    hold = _hold_tcp(fit_o, 0.0)
    ry0, ry1 = START_YAW, 0.0
    start = _jaw_tcp(JAW_OPEN_DEG, GAP0)
    shut = _jaw_tcp(0.0, PRESS)
    right = [
        Waypoint(2.0, (pick[0], pick[1], pick[2] + 0.050), open_, "over the sensor's housing", ry0),
        Waypoint(1.5, pick, open_, "down around the housing", ry0),
        Waypoint(0.5, pick, open_, "settle", ry0),
        Waypoint(1.0, pick, grip, "grip the housing", ry0),
        Waypoint(2.0, lift, grip, "lift it off the peg", ry0),
        Waypoint(2.0, lift, grip, "turn it a quarter turn in place, clear of the conductors", ry1),
        Waypoint(2.5, over, grip, "carry it over the line conductor, the jaw trailing on the near side", ry1),
        Waypoint(0.5, over, grip, "settle", ry1),
        Waypoint(2.5, hold, grip, "lower it onto the line conductor", ry1),
        Waypoint(0.8, hold, grip, "settle", ry1),
        Waypoint(1.0, hold, open_, "let go: it hangs on the line conductor", ry1),
        Waypoint(1.5, (hold[0], hold[1], hold[2] + 0.040), open_, "up off the housing", ry1),
        Waypoint(2.0, (start[0], start[1], start[2] + 0.025), C, "over the open jaw", YAW_JAW),
        Waypoint(1.0, start, C, "down onto the jaw", YAW_JAW),
        Waypoint(0.5, start, C, "settle", YAW_JAW),
        *(Waypoint(0.8, _jaw_tcp(deg, JAW_GAP.get(deg, -0.007)), C, f"swing the jaw to {deg} deg", YAW_JAW) for deg in ARC),
        Waypoint(0.8, shut, C, "press it shut", YAW_JAW),
        Waypoint(0.5, shut, C, "hold", YAW_JAW),
        Waypoint(1.2, (shut[0] - 0.040, shut[1], shut[2]), C, "back off", YAW_JAW),
        Waypoint(1.5, (shut[0] - 0.060, shut[1], shut[2] + 0.080), C, "up", YAW_JAW),
        Waypoint(1.5, PARK["right"], C, "park", 0.0),
    ]
    left = [Waypoint(sum(w.duration for w in right), PARK["left"], C, "park", 0.0)]
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built poses of the sensor (single states and short sequences, one trial each)."""
    import dataclasses

    global _TEST_TCPS

    def st(x=L_X, y=FIT_Y, yaw=0.0, jaw=0.0, z=COND_TOP, tilt_deg=0.0, vel=0.0):
        r = _rz(yaw) @ mk.axis_angle((0, 1, 0), math.radians(tilt_deg))
        s = kit.fake_states(q={"jaw_hinge": math.radians(jaw)},
                            moved={"sensor": (np.array([x, y, z]), mk.mat_to_quat(r))})
        if vel:
            s["sensor"] = dataclasses.replace(s["sensor"], lin_vel=np.array([0.0, 0.0, vel]))
        return s

    start = kit.fake_states()
    fitted = st()
    # the notch on the neutral, window on the far side (sensor turned the other way round: arrow to the panel)
    reversed_on_line = st(x=L_X, yaw=math.pi, z=COND_TOP)  # turned the wrong way: window spans line AND neutral
    neutral_only = st(x=N_X, yaw=math.pi)
    both = st(x=N_X)  # notch on the neutral, window reaching back over the line: around both
    hold_tcp = _hold_tcp(FIT_ORIGIN, 0.0)
    cases = {  # name: (sequence, TCPs, final score)
        "start, on the peg": ([start], [], 0.0),
        "fitted on the line, jaw shut, arrow to the PDU": ([start, fitted], [], 1.0),
        "fitted, still held by the housing": ([start, fitted], [hold_tcp], 0.0),
        "fitted, still swinging": ([start, st(vel=0.05)], [], 0.0),
        "hung on the line, jaw left open": ([start, st(jaw=JAW_OPEN_DEG)], [], 0.25),
        "hung on the line, jaw 8 deg open": ([start, st(jaw=8.0)], [], 0.25),
        "naive: shut around both conductors": ([start, both], [], 0.0),
        "turned the wrong way, notch on the line: shut around both": ([start, reversed_on_line], [], 0.0),
        "turned the wrong way, shut around the neutral only": ([start, neutral_only], [], 0.25),
        "line only but arrow toward the panel (sensor on its side, tilted)": (
            [start, st(yaw=math.pi, x=L_X + X_FAR + X_NEAR, tilt_deg=0.0)], [], 0.5),
        "fitted, hanging tilted 20 deg about the conductor": ([start, st(tilt_deg=20.0)], [], 1.0),
        "fitted, arrow 20 deg off the run": ([start, st(yaw=math.radians(20))], [], 1.0),
        "fitted, arrow 30 deg off the run": ([start, st(yaw=math.radians(30))], [], 0.5),
        "fitted but skewed 40 deg across the run": ([start, st(yaw=math.radians(40))], [], 0.5),
        "dropped on the table": ([start, st(x=0.20, y=0.0, z=TOP + WIN_H + 2 * BAR)], [], 0.0),
        "fitted end state, no history": ([fitted], [], 1.0),
    }
    try:
        for name, (seq, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            for s in seq[:-1]:
                score(s)
            got, ok = score(seq[-1]), success(seq[-1])
            assert abs(got - want) < 1e-9 and ok == (want == 1.0), (name, got, ok, want)
        assert _encircles(fitted, L_X) and not _encircles(fitted, N_X)
        assert _encircles(both, L_X) and _encircles(both, N_X)
        assert not _encircles(start, L_X) and not _encircles(start, N_X)
        assert distance(fitted) < 1e-6 and distance(start) > 0.1
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases)


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

