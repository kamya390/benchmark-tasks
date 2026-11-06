"""On the station table, in front of the left arm, a green circuit board lies flat with a black processor socket on
it: four low walls round a 41 mm opening. A processor (a flat 40 mm square with a nickel lid) sits in the socket,
and an aluminium heatsink (a base plate with seven fins on top) sits on the processor, stuck to its lid by dried
thermal compound. A grey ESD mat lies on the table 15 cm to the right of the socket. The job is to take the heatsink
off and put it on the mat while the processor stays in its socket.

The compound is a bond that is strong in tension (25 N) and weak in twist (it lets go past a 4 deg turn, about
0.14 N*m); the processor's hold in its socket is weaker than that pull (6 N). So a straight pull lifts the processor
out with the heatsink, and a twist about the vertical breaks the compound first, after which the heatsink lifts off
alone. If the processor ever rises
3 mm out of its seat or moves 5 mm off the socket's centre, the trial scores 0 from then on.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/heatsink-twist-free/heatsink_twist_free.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (bonds between bodies; installs its yam_arena hooks in memory)

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


def released(state, margin=0.025) -> bool:
    """Neither gripper's TCP within ``margin`` (2.5 cm) of the part's bounding box: the part is let go."""
    lo, hi = np.asarray(state.low) - margin, np.asarray(state.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())


SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
SOCKET_XY = (0.320, 0.220)
BOARD = (0.200, 0.160, 0.002)  # FR-4 board, flat on the table, centred on the socket
SEAT_Z = TOP + BOARD[2]  # the processor's seat: the board's top
SOCKET_INNER = 0.041  # the socket frame's opening
SOCKET_WALL = (0.004, 0.003)  # wall thickness, height (lower than the processor's lid)
CPU = (0.040, 0.040, 0.004)
CPU_KG = 0.030
CPU_TOP = SEAT_Z + CPU[2]
HS = (0.060, 0.060, 0.035)  # heatsink footprint and height: a 6 mm base plate and seven 2 mm fins along x
HS_BASE = 0.006
HS_FINS = 7
HS_KG = 0.250
MAT = (0.120, 0.120, 0.003)
MAT_XY = (0.320, 0.070)
MAT_TOP = TOP + MAT[2]

CPU_HOLD_N = 6.0  # the socket's hold on the processor (pull)
PASTE_PULL_N = 25.0  # the dried compound in tension
PASTE_TWIST_K = 2.0  # the dried compound in twist, N*m/rad, until it lets go ...
PASTE_RELEASE_DEG = 4.0  # ... past a 4 deg turn (about 0.14 N*m)

CPU_RISE = 0.003  # the processor counts as pulled once its bottom is 3 mm above its seat
CPU_SHIFT = 0.005  # ... or its centre 5 mm off the socket's centre (out of the frame)
CPU_TILT = 10.0  # ... or tipped 10 deg
TURN_DEG = 5.0  # the heatsink counts as turned on the processor at 5 deg about the vertical
CLEAR = 0.010  # the heatsink counts as off the processor once its bottom is 1 cm above the lid
AWAY = 0.050  # ... or its centre 5 cm from the socket's centre
ON_MAT = 0.035  # the heatsink's centre within 3.5 cm of the mat's centre (fully on the 12 cm mat)

GREEN = (0.10, 0.45, 0.20)
BLACK = (0.08, 0.08, 0.09)
NICKEL = (0.78, 0.78, 0.76)
ALU = (0.72, 0.74, 0.78)
GREY = (0.45, 0.46, 0.48)

kit = mk.Kit()
kit.part("cpu", (*SOCKET_XY, SEAT_Z), [mk.box(CPU, at=(0.0, 0.0, CPU[2] / 2))], mass=CPU_KG, color=NICKEL)
_pitch = (HS[1] - 0.002) / (HS_FINS - 1)
kit.part("heatsink", (*SOCKET_XY, CPU_TOP),
         [mk.box((HS[0], HS[1], HS_BASE), at=(0.0, 0.0, HS_BASE / 2)),
          *(mk.box((HS[0], 0.002, HS[2] - HS_BASE), at=(0.0, -HS[1] / 2 + 0.001 + i * _pitch, HS_BASE + (HS[2] - HS_BASE) / 2))
            for i in range(HS_FINS))],
         mass=HS_KG, color=ALU)
# the socket's hold: a pull of 6 N frees the processor; no torque limit (the frame walls stop it turning)
kit.bond("socket_hold", None, "cpu", break_force=CPU_HOLD_N, anchor=(*SOCKET_XY, SEAT_Z))
# the compound: a pull of 25 N tears it; in twist about the vertical it yields like sheared paste and lets go for
# good past 4 deg (about 0.14 N*m), the turn taken about the centre of the lid/heatsink contact
kit.bond("paste", "cpu", "heatsink", break_force=PASTE_PULL_N, anchor=(*SOCKET_XY, CPU_TOP), twist_axis=(0, 0, 1),
         twist_release_deg=PASTE_RELEASE_DEG, twist_stiffness=PASTE_TWIST_K)


def _socket_walls():
    cx, cy = SOCKET_XY
    i, (t, h) = SOCKET_INNER, SOCKET_WALL
    z = SEAT_Z + h / 2
    return [
        Box("socket_wall_ypos", pos=(cx, cy + i / 2 + t / 2, z), size=(i + 2 * t, t, h), color=BLACK),
        Box("socket_wall_yneg", pos=(cx, cy - i / 2 - t / 2, z), size=(i + 2 * t, t, h), color=BLACK),
        Box("socket_wall_near", pos=(cx - i / 2 - t / 2, cy, z), size=(t, i, h), color=BLACK),
        Box("socket_wall_far", pos=(cx + i / 2 + t / 2, cy, z), size=(t, i, h), color=BLACK),
    ]


FIXTURES = [
    Box("board", pos=(*SOCKET_XY, TOP + BOARD[2] / 2), size=BOARD, color=GREEN),
    *_socket_walls(),
    Box("esd_mat", pos=(*MAT_XY, TOP + MAT[2] / 2), size=MAT, color=GREY),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _tilt(q) -> float:
    """Tilt of the body's up axis from the vertical, degrees."""
    x, y, _, _ = (float(v) for v in q)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def _yaw(q) -> float:
    x, y, z, w = (float(v) for v in q)
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _turn(hs, cpu) -> float:
    """The heatsink's turn about the vertical relative to the processor, degrees, folded by the square's 90 deg
    symmetry into [0, 45]."""
    return abs((_yaw(hs.quat_xyzw) - _yaw(cpu.quat_xyzw) + 45.0) % 90.0 - 45.0)


def _xy_from(st, xy) -> float:
    return math.hypot(float(st.pos[0]) - xy[0], float(st.pos[1]) - xy[1])


def _cpu_seated(cpu) -> bool:
    """The processor down on its seat (bottom under 3 mm above the board), centred in the socket within 5 mm, flat."""
    return bool(float(cpu.low[2]) < SEAT_Z + CPU_RISE and _xy_from(cpu, SOCKET_XY) < CPU_SHIFT
                and _tilt(cpu.quat_xyzw) < CPU_TILT)


def _off_cpu(hs) -> bool:
    """The heatsink clear of the processor: its bottom 1 cm above the lid, or its centre 5 cm from the socket."""
    return bool(float(hs.low[2]) > CPU_TOP + CLEAR or _xy_from(hs, SOCKET_XY) > AWAY)


def _on_mat(hs) -> bool:
    """The heatsink at rest, flat (under 10 deg) on the mat, centre within 3.5 cm of the mat's centre, let go."""
    return bool(
        _xy_from(hs, MAT_XY) < ON_MAT
        and MAT_TOP - 0.002 < float(hs.low[2]) < MAT_TOP + 0.005
        and _tilt(hs.quat_xyzw) < 10.0
        and hs.resting()
        and released(hs)
    )


# The trial's history (in _MEM, cleared only by the embodiment's reset at the start of each trial): whether the
# processor has ever left its seat. Pressing it back into the socket afterwards does not undo that.
def _update(objects) -> bool:
    """Record the processor leaving its seat; returns whether it has in this trial."""
    if not _cpu_seated(objects["cpu"]):
        _MEM["cpu_pulled"] = True
    return bool(_MEM.get("cpu_pulled"))


def score(objects) -> float:
    """1.0: the heatsink at rest on the mat, flat and let go, with the processor seated. 0.5: the heatsink off the
    processor (held up or set down anywhere but on the mat as above). 0.25: the heatsink still on the processor but
    turned at least 5 deg about the vertical (the compound broken by twisting). 0 otherwise, and 0 for the rest of
    the trial once the processor has left its seat."""
    if _update(objects):
        return 0.0
    hs, cpu = objects["heatsink"], objects["cpu"]
    if _on_mat(hs):
        return 1.0
    if _off_cpu(hs):
        return 0.5
    if _turn(hs, cpu) >= TURN_DEG:
        return 0.25
    return 0.0


def success(objects) -> bool:
    """The heatsink on the mat, the processor never pulled; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def cpu_pulled() -> bool:
    """Whether this trial has pulled the processor out of its seat (for the log)."""
    return bool(_MEM.get("cpu_pulled"))


def distance(objects) -> float:
    """Metres to go: the heatsink's centre to the mat's centre."""
    return float(_xy_from(objects["heatsink"], MAT_XY))


# -- the scripted demo ------------------------------------------------------------------------------------------

YAW0 = 0.0  # the fingers close along y, on the two outer fins
TWIST = math.radians(20.0)  # the twist that breaks the compound
TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008
DEPTH = 0.015  # grip 15 mm below the heatsink's top


def demo():
    """Left arm only, open loop from the scene's fixed geometry: grip the heatsink across its fins (fingers closed
    8 mm under its 60 mm width), twist it 20 deg about the vertical while it is still down on the processor, which
    breaks the compound, then lift it straight up and set it on the mat."""
    (sx, sy), (mx, my) = SOCKET_XY, MAT_XY
    gz = CPU_TOP + HS[2] - DEPTH + TIP
    qz = MAT_TOP + HS[2] - DEPTH + TIP + 0.002  # let go 2 mm above the resting height
    grip = -(HS[1] - SQUEEZE) / 2
    o, y0, y1 = FINGER_OPEN, YAW0, YAW0 + TWIST
    return {"left": [
        Waypoint(2.0, (sx, sy, TRAVEL), o, "over the heatsink", y0),
        Waypoint(1.5, (sx, sy, gz + 0.020), o, "descend", y0),
        Waypoint(1.0, (sx, sy, gz), o, "down slowly", y0),
        Waypoint(0.5, (sx, sy, gz), o, "settle", y0),
        Waypoint(1.5, (sx, sy, gz), grip, "close", y0),
        Waypoint(0.5, (sx, sy, gz), grip, "settle closed", y0),
        Waypoint(2.5, (sx, sy, gz), grip, "twist 20 deg to break the compound", y1),
        Waypoint(0.5, (sx, sy, gz), grip, "hold", y1),
        Waypoint(1.5, (sx, sy, gz + 0.030), grip, "lift straight up", y1),
        Waypoint(1.2, (sx, sy, TRAVEL), grip, "lift", y1),
        Waypoint(2.0, (mx, my, TRAVEL), grip, "carry", y1),
        Waypoint(0.6, (mx, my, TRAVEL), grip, "settle over", y1),
        Waypoint(1.5, (mx, my, qz + 0.020), grip, "lower", y1),
        Waypoint(1.5, (mx, my, qz), grip, "set down slowly", y1),
        Waypoint(0.6, (mx, my, qz), grip, "settle", y1),
        Waypoint(1.5, (mx, my, qz), o, "release", y1),
        Waypoint(1.0, (mx, my, TRAVEL), o, "retreat", y1),
    ]}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def body(half, x, y, bottom, vel=(0.0, 0.0, 0.0), yaw_deg=0.0, tilt_deg=0.0):
        t, w = math.radians(tilt_deg), math.radians(yaw_deg)
        qt = np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)])  # tilt about x
        qy = np.array([0.0, 0.0, math.sin(w / 2), math.cos(w / 2)])  # then yaw about z
        x1, y1, z1, w1 = qy
        x2, y2, z2, w2 = qt
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        c, s = abs(math.cos(w)), abs(math.sin(w))
        half = np.array([half[0] * c + half[1] * s, half[0] * s + half[1] * c, half[2]])
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def cpu(x=SOCKET_XY[0], y=SOCKET_XY[1], bottom=SEAT_Z, **kw):
        return {"cpu": body(np.array(CPU) / 2, x, y, bottom, **kw)}

    def hs(x=SOCKET_XY[0], y=SOCKET_XY[1], bottom=CPU_TOP, **kw):
        return {"heatsink": body(np.array(HS) / 2, x, y, bottom, **kw)}

    T = 20.0  # the demo's twist, degrees
    start = cpu() | hs()
    twisted = cpu() | hs(yaw_deg=T)
    lifted = cpu() | hs(bottom=CPU_TOP + 0.030, yaw_deg=T, vel=(0, 0, 0.05))
    held_over_mat = cpu() | hs(*MAT_XY, MAT_TOP + 0.030, yaw_deg=T)
    done = cpu() | hs(*MAT_XY, MAT_TOP, yaw_deg=T)
    # a straight pull: the processor comes up stuck under the heatsink
    yank = cpu(bottom=SEAT_Z + 0.030) | hs(bottom=CPU_TOP + 0.030)
    yank_small = cpu(bottom=SEAT_Z + 0.002) | hs(bottom=CPU_TOP + 0.002)
    on_mat_with_cpu = cpu(*MAT_XY, MAT_TOP + HS[2] + 0.0) | hs(*MAT_XY, MAT_TOP)

    def run(*frames):
        """Feed the states in order (as the simulator would, step by step); the last score and success. None is the
        embodiment's reset (a new trial); a (frame, tcp) pair puts one gripper's TCP at ``tcp``."""
        global _tcps
        tcps0 = _tcps
        _MEM.clear()
        try:
            for f in frames:
                if f is None:
                    _MEM.clear()
                    continue
                tcp = []
                if isinstance(f, tuple):
                    f, tcp = f[0], [np.asarray(f[1], float)]
                _tcps = lambda tcp=tcp: tcp  # noqa: E731
                got_success, got = success(f), score(f)
        finally:
            _tcps = tcps0
        return got, got_success

    grip_on_mat = (*MAT_XY, MAT_TOP + HS[2] - DEPTH)
    cases = {
        "start": ((start,), 0.0, False),
        "twisted 20 deg on the processor": ((start, twisted), 0.25, False),
        "turned only 3 deg": ((start, cpu() | hs(yaw_deg=3.0)), 0.0, False),
        "turned 84 deg reads as 6 deg off square": ((start, cpu() | hs(yaw_deg=84.0)), 0.25, False),
        "twisted, then lifted clear": ((start, twisted, lifted), 0.5, False),
        "twisted, lifted, held above the mat": ((start, twisted, lifted, held_over_mat), 0.5, False),
        "twist, lift, set on the mat and let go": ((start, twisted, lifted, held_over_mat, done), 1.0, True),
        "naive straight pull yanks the processor out": ((start, yank), 0.0, False),
        "naive pull, then everything pressed back and the heatsink put on the mat":
            ((start, yank, start, twisted, lifted, done), 0.0, False),
        "naive pull carries the processor to the mat under the heatsink": ((start, yank, on_mat_with_cpu), 0.0, False),
        "processor lifted 2 mm and back is still seated": ((start, yank_small, start, twisted, lifted, done), 1.0, True),
        "processor dragged 6 mm off the socket": ((start, cpu(SOCKET_XY[0] + 0.006) | hs(SOCKET_XY[0] + 0.006)), 0.0,
                                                  False),
        "processor tipped 15 deg in the socket": ((start, cpu(tilt_deg=15.0) | hs()), 0.0, False),
        "a new trial after a yank starts clean": ((start, yank, None, start, twisted, lifted, done), 1.0, True),
        "on the mat but still gripped": ((start, twisted, lifted, (done, grip_on_mat)), 0.5, False),
        "on the mat, still sliding": ((start, twisted, lifted, cpu() | hs(*MAT_XY, MAT_TOP, yaw_deg=T, vel=(0.03, 0, 0))),
                                      0.5, False),
        "on the mat, leaning 30 deg": ((start, twisted, lifted, cpu() | hs(*MAT_XY, MAT_TOP, tilt_deg=30.0)), 0.5,
                                       False),
        "set down on the table beside the mat": ((start, twisted, lifted, cpu() | hs(0.200, 0.070, TOP)), 0.5, False),
        "set down on the board beside the socket": ((start, twisted, lifted, cpu() | hs(0.320, 0.150, SEAT_Z)), 0.5,
                                                    False),
        "on the mat, then picked back up and held": ((start, twisted, lifted, done, lifted), 0.5, False),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    # the same rules on the kit's own forward-kinematics states (the poses the simulator reports for its parts)
    kit_cases = {
        "kit: start pose": ((kit.fake_states(),), 0.0, False),
        "kit: heatsink twisted 6 deg on the bond": ((kit.fake_states(), kit.fake_states(q={"paste": 6.0}, deg=True)),
                                                    0.25, False),
    }
    for name, (frames, want, want_success) in kit_cases.items():
        mk.new_episode()
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    mk.new_episode()
    _MEM.clear()  # leave the history clean
    assert not cpu_pulled()
    return len(cases) + len(kit_cases)


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
