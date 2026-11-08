"""On the station table, in front of the left arm, a short length of grey square-section PVC drain pipe lies on a
plywood floor board, running from the left (its open mouth) to the right (its outlet over a steel catch trough).
The pipe is blocked 2 cm inside the mouth by a plug of congealed fat and wipes. A flexible sectional drain rod is
already started in the mouth: nine black segments joined by sprung couplings, with an orange T-handle at its tail.
The part of the rod outside the pipe lies on the board in a loose curve (the rod's coil set). The job is to push
the clog out of the far end of the pipe with the rod, then let go of it. Scored from the poses at the end.

The rod's couplings are spring hinges (mechkit.py, in this folder): the rod bends only sideways, on the board.
A slender rod pushed from far back buckles: the free length between the hand and the mouth bows out and kinks long
before it pushes hard enough to move the clog. Fed in short strokes with the hand just behind the mouth, the free
length stays short, the pipe guides the rest, and the rod drives the clog through. A coupling bent past 40 degrees
is a kinked rod and scores 0 for the trial.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/drain-rod-short-strokes/drain_rod_short_strokes.py \\
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
import mechkit as mk  # noqa: E402  (spring hinges and the clog's friction slide; installs its hooks in memory)

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
PX = 0.300  # the pipe's centre line, along y
MOUTH_Y = 0.070  # the open mouth (the rod side); the pipe runs toward -y from here
PIPE_L = 0.220
OUTLET_Y = MOUTH_Y - PIPE_L
BOARD_T = 0.003
FL = TOP + BOARD_T  # the floor board's top = the pipe's bore floor
BORE_W, BORE_H, WALL = 0.011, 0.016, 0.003  # square-section bore 11 mm wide, 16 mm tall; 3 mm PVC walls

# -- the rod (free): nine segments + T-handle, joined by spring hinges about the vertical --
SEG = (0.008, 0.032, 0.012)  # x (width), y (length), z (height)
SEG_KG = 0.0025
N_SEG = 9
HANDLE_SHAFT = (0.008, 0.026, 0.012)
HANDLE_BAR = (0.050, 0.012, 0.012)
HANDLE_KG = 0.010
TIP_IN = 0.034  # the tip segment starts wholly inside the mouth
K_HINGE = 0.006  # N*m/rad: the couplings' straightening spring
SET_DEG = 6.0  # coil set: the outer couplings rest bent 6 degrees, so the loose rod lies in a curve toward +x
SET_FROM = 4  # couplings 4..9 (counted from the tip) carry the set
LIMIT_DEG = 70.0
GAP = 0.0003  # spawn the rod 0.3 mm above the board

# -- the clog (free) on a friction slide along -y --
CLOG = (0.009, 0.012, 0.020)  # x, z, y
CLOG_KG = 0.015
CLOG_REAR = MOUTH_Y - TIP_IN - 0.002  # its rear face, 2 mm ahead of the rod tip
CLOG_DRAG = 0.4  # N: Coulomb friction of the packed plug
CLOG_TRAVEL_MAX = 0.230
GOAL = (CLOG_REAR - CLOG[2]) - OUTLET_Y  # 0.164 m: the clog's front face out at the outlet

PVC = (0.55, 0.57, 0.58)
PLY = (0.78, 0.66, 0.47)
STEEL = (0.62, 0.64, 0.67)
ROD = (0.08, 0.08, 0.09)
ORANGE = (0.95, 0.45, 0.08)
FOG = (0.42, 0.30, 0.16)

SEGS = tuple(f"rod_seg{i}" for i in range(N_SEG))
HANDLE = "rod_handle"
ROD_PARTS = (*SEGS, HANDLE)
HINGES = tuple(f"coupling{i}" for i in range(1, N_SEG + 1))  # coupling i joins part i-1 to part i (tip = 0)

kit = mk.Kit()
_ZC = FL + GAP + SEG[2] / 2
for _i, _name in enumerate(SEGS):
    _y0 = MOUTH_Y - TIP_IN + SEG[1] * _i  # the segment's tip-side end
    kit.part(_name, (PX, _y0 + SEG[1] / 2, _ZC), [mk.box(SEG)], mass=SEG_KG, color=ROD)
_HY0 = MOUTH_Y - TIP_IN + SEG[1] * N_SEG
kit.part(HANDLE, (PX, _HY0 + HANDLE_SHAFT[1] / 2, _ZC),
         [mk.box(HANDLE_SHAFT), mk.box(HANDLE_BAR, at=(0.0, HANDLE_SHAFT[1] / 2 + HANDLE_BAR[1] / 2, 0.0))],
         mass=HANDLE_KG, color=ORANGE)
for _i, _hinge in enumerate(HINGES, start=1):
    _set = SET_DEG if _i >= SET_FROM else 0.0
    # axis -z: a positive angle turns the rod's outer part from +y toward +x (away from the arms)
    kit.hinge(_hinge, ROD_PARTS[_i - 1], ROD_PARTS[_i], anchor=(PX, MOUTH_Y - TIP_IN + SEG[1] * _i, _ZC),
              axis=(0, 0, -1), lower_deg=-LIMIT_DEG, upper_deg=LIMIT_DEG, start_deg=_set, stiffness=K_HINGE,
              rest_deg=_set)

kit.part("clog", (PX, CLOG_REAR - CLOG[2] / 2, FL + 0.001 + CLOG[1] / 2), [mk.box((CLOG[0], CLOG[2], CLOG[1]))],
         mass=CLOG_KG, color=FOG)
kit.slider("clog_slide", None, "clog", axis=(0, -1, 0), lower=0.0, upper=CLOG_TRAVEL_MAX, friction=CLOG_DRAG)


def _fixtures():
    out = [Box("floor_board", pos=(PX + 0.050, (OUTLET_Y - 0.090 + MOUTH_Y + 0.340) / 2, TOP + BOARD_T / 2),
               size=(0.220, MOUTH_Y + 0.340 - (OUTLET_Y - 0.090), BOARD_T), color=PLY)]
    yc = (MOUTH_Y + OUTLET_Y) / 2
    for side, sx in (("near", -1.0), ("far", 1.0)):
        out.append(Box(f"pipe_{side}_wall", pos=(PX + sx * (BORE_W + WALL) / 2, yc, FL + BORE_H / 2),
                       size=(WALL, PIPE_L, BORE_H), color=PVC))
        out.append(Box(f"trough_{side}_lip", pos=(PX + sx * (BORE_W + WALL) / 2, OUTLET_Y - 0.040, FL + 0.004),
                       size=(WALL, 0.080, 0.008), color=STEEL))
    out.append(Box("pipe_crown", pos=(PX, yc, FL + BORE_H + WALL / 2), size=(BORE_W + 2 * WALL, PIPE_L, WALL),
                   color=PVC))
    out.append(Box("trough_end", pos=(PX, OUTLET_Y - 0.082, FL + 0.004), size=(BORE_W + 2 * WALL, 0.004, 0.008),
                   color=STEEL))
    return out


FIXTURES = _fixtures()
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Measured from the parts' poses: the angle between neighbouring rod segments (a photo of the rod shows a kink), and
# how far the clog has been driven down the pipe (it shows at the outlet when it is out).

KINK_DEG = 40.0
_KINK_KEY = "drain_rod_kinked"


def _bends(objects) -> dict:
    return {h: kit.q_deg(objects, h) for h in HINGES}


def _kinked(objects) -> bool:
    """Latched for the trial: some coupling was ever bent more than 40 degrees."""
    if any(abs(v) > KINK_DEG for v in _bends(objects).values()):
        _MEM[_KINK_KEY] = True
    return bool(_MEM.get(_KINK_KEY))


kit.on_step(_kinked)  # checked every control step in sim, not only when the scorer looks


def _progress(objects) -> float:
    """Fraction of the way from the clog's start to the outlet."""
    return kit.q(objects, "clog_slide") / GOAL


def score(objects) -> float:
    """0 for the whole trial once any rod coupling has bent past 40 degrees (a kinked rod). Otherwise, by how far the
    clog has been driven toward the outlet (p = travel / 16.4 cm): 0.25 for p >= 0.25, 0.5 for p >= 0.6, and 1.0
    for p >= 0.97 with neither gripper within 2.5 cm of any rod part and the rod and clog still."""
    if _kinked(objects):
        return 0.0
    p = _progress(objects)
    if p >= 0.97:
        let_go = all(released(objects[n]) for n in ROD_PARTS)
        still = all(objects[n].resting() for n in (*ROD_PARTS, "clog"))
        if let_go and still:
            return 1.0
    if p >= 0.6:
        return 0.5
    return 0.25 if p >= 0.25 else 0.0


def success(objects) -> bool:
    """The clog out at the outlet, the rod never kinked, let go and still; holds for 1 s."""
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Metres the clog still has to travel to the outlet."""
    return float(max(0.0, GOAL - kit.q(objects, "clog_slide")))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the pipe's fixed geometry; it reads no joint or simulator state. The left arm grips the rod just
# behind the mouth (fingers closing across it), pushes it 22 mm in, opens, slides back 22 mm and grips again: the
# rod's free length between the pads' leading edge and the mouth is never more than 25 mm (under one segment).

YAW = -math.pi / 2  # the left arm's fingers close along x, across the rod
TIP = 0.013  # pad tips below the TCP
PAD_HALF = 0.019  # half the pads' 38 mm width, along the rod
GZ = FL + 0.002 + TIP  # pad tips 2 mm above the board
NEAR_Y = MOUTH_Y + PAD_HALF + 0.003  # end of a stroke: the pads' leading edge 3 mm behind the mouth
STROKE = 0.022
FAR_Y = NEAR_Y + STROKE  # start of a stroke: 25 mm of rod free in front of the pads
N_STROKES = 13
OPEN = -0.016  # 32 mm between the pads: 12 mm clear each side of the 8 mm rod
GRIP = FINGER_CLOSED  # 8 mm squeeze on the 8 mm rod
UP = FL + 0.090


def demo():
    """The left arm feeds the rod in thirteen 22 mm strokes, each gripped 2.2-4.4 cm behind the mouth: close, push at
    about 1.4 cm/s, open, slide back, close again; then it lets go and lifts away. The right arm stays still. 67.4 s."""
    w = [
        Waypoint(2.5, (PX, FAR_Y, UP), OPEN, "over the rod behind the mouth", YAW),
        Waypoint(1.5, (PX, FAR_Y, GZ + 0.015), OPEN, "descend", YAW),
        Waypoint(0.8, (PX, FAR_Y, GZ), OPEN, "down beside the rod", YAW),
    ]
    for k in range(1, N_STROKES + 1):
        w += [
            Waypoint(0.7, (PX, FAR_Y, GZ), GRIP, f"stroke {k}: grip", YAW),
            Waypoint(0.3, (PX, FAR_Y, GZ), GRIP, f"stroke {k}: settle", YAW),
            Waypoint(1.6, (PX, NEAR_Y, GZ), GRIP, f"stroke {k}: push 22 mm", YAW),
            Waypoint(0.3, (PX, NEAR_Y, GZ), GRIP, f"stroke {k}: settle", YAW),
            Waypoint(0.6, (PX, NEAR_Y, GZ), OPEN, f"stroke {k}: open", YAW),
        ]
        if k < N_STROKES:
            w += [
                Waypoint(0.3, (PX, NEAR_Y, GZ + 0.006), OPEN, f"stroke {k}: lift off", YAW),
                Waypoint(0.6, (PX, FAR_Y, GZ + 0.006), OPEN, f"stroke {k}: slide back", YAW),
                Waypoint(0.3, (PX, FAR_Y, GZ), OPEN, f"stroke {k}: down", YAW),
            ]
    w += [
        Waypoint(1.2, (PX, NEAR_Y, UP), OPEN, "lift clear", YAW),
        Waypoint(1.5, (0.22, 0.20, UP + 0.05), OPEN, "park", YAW),
    ]
    return {"left": w}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built rod and clog poses (single states and short sequences, one trial each)."""
    import dataclasses

    def st(travel=0.0, feed=0.0, bends=None, moving=()):
        """The clog driven ``travel`` m; the rod fed ``feed`` m into the pipe; extra coupling angles (deg)."""
        q = {"clog_slide": travel}
        q.update({h: (SET_DEG if i >= SET_FROM else 0.0) for i, h in enumerate(HINGES, start=1)})
        q.update(bends or {})
        tip0 = kit.parts[SEGS[0]].ref[:3, 3]
        states = kit.fake_states(q=q, deg=True,
                                 moved={SEGS[0]: (tip0 + np.array([0.0, -feed, 0.0]), (0.0, 0.0, 0.0, 1.0))})
        for n in moving:
            states[n] = dataclasses.replace(states[n], lin_vel=np.array([0.0, -0.03, 0.0]))
        return states

    start = st()
    done = st(GOAL + 0.004, GOAL + 0.006)
    grip_tcp = np.array([PX, NEAR_Y, GZ])  # the left TCP still on the rod at the mouth
    kinked = st(0.002, 0.0, {"coupling6": 52.0, "coupling7": 38.0})
    cases = {  # name: (sequence of states in one trial, gripper TCPs, final score)
        "start": ([start], [], 0.0),
        "clog 5 cm in (p 0.30)": ([start, st(0.05, 0.05)], [], 0.25),
        "clog 11 cm in (p 0.67)": ([start, st(0.11, 0.11)], [], 0.5),
        "clog 15 cm in (p 0.91), let go": ([start, st(0.15, 0.15)], [], 0.5),
        "clog out at the outlet, let go, still": ([start, st(0.08, 0.08), done], [], 1.0),
        "clog out, a gripper still on the rod": ([start, done], [grip_tcp], 0.5),
        "clog out, gripper lifted 9 cm above the rod": ([start, done], [np.array([PX, NEAR_Y, UP])], 1.0),
        "clog out, rod still sliding": ([start, st(GOAL + 0.004, GOAL + 0.006, moving=(SEGS[3],))], [], 0.5),
        "naive: pushed from the handle, rod kinked": ([start, kinked], [], 0.0),
        "kinked, then straightened and the clog pushed out": ([start, kinked, st(0.05, 0.05), done], [], 0.0),
        "kinked the other way (-45 deg) late in the trial": (
            [start, st(0.12, 0.12), st(0.12, 0.12, {"coupling2": -45.0}), done], [], 0.0),
        "bent 38 deg at a coupling but never kinked": (
            [start, st(0.06, 0.06, {"coupling8": 38.0}), done], [], 1.0),
        "clog 1 cm short of the outlet (p 0.94)": ([start, st(GOAL - 0.010, GOAL - 0.008)], [], 0.5),
    }
    try:
        for name, (seq, tcps, want) in cases.items():
            _MEM.clear()  # a new episode, as the embodiment's reset() does
            mk.new_episode()
            globals()["_tcps"] = lambda tcps=tcps: [np.asarray(t, float) for t in tcps]
            for s in seq[:-1]:
                score(s)
            got = score(seq[-1])
            assert abs(got - want) < 1e-9 and success(seq[-1]) == (want == 1.0), (name, got, want)
        # a new trial forgets the last one's kink
        globals()["_tcps"] = lambda: []
        _MEM.clear()
        score(kinked)
        assert score(done) == 0.0
        _MEM.clear()
        assert score(done) == 1.0, "reset"
        assert abs(_bends(start)["coupling5"] - SET_DEG) < 1e-6 and abs(_bends(start)["coupling2"]) < 1e-6
        assert distance(start) > 0.16 and distance(done) == 0.0
    finally:
        globals()["_tcps"] = _tcps_live
        _MEM.clear()
        mk.new_episode()
    return len(cases) + 1


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the arm's plan replayed through the
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
