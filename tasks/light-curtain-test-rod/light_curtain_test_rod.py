"""On the station table, in front of the right arm, a small press stands guarded by a safety light curtain: an
emitter column (near the arms) and a receiver column (farther out) stand 22 cm apart across the press's front, each
with a dark lens strip facing the other. Between them runs an invisible vertical sensing plane, the protected zone,
from 4.5 cm to 15.5 cm above the table. Behind the plane (to the left) lie the press's bolster plate and lower die. In
front of the plane, on the right, a black 14 mm test rod lies along y in a two-saddle holder. The job is the
start-of-shift check of the curtain with the test rod, then the rod goes back in its holder.

The check the curtain makers prescribe (not stated in the instruction): pass the test rod slowly through the whole
height of the protected zone, held across the plane, along three paths: close to the emitter, midway, and close to
the receiver. The score reads where the rod's axis pierces the sensing plane at every step (interpolated between
steps): each of the three paths counts once those piercing heights, taken with the rod within 30 deg of the plane's
normal and moving up or down slower than 0.25 m/s, cover 85 % of the zone's height.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/light-curtain-test-rod/light_curtain_test_rod.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import dataclasses
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (per-episode memory, gripper TCPs; the rod is a kit part so it is not jittered)

from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The right arm's base is at (0, -0.305, 0.75).
PLANE_Y = -0.040  # the sensing plane: y = PLANE_Y, between the two columns
EMITTER_X, RECEIVER_X = 0.140, 0.360  # column centres
COL = (0.025, 0.025, 0.170)  # column section (x, y) and height
COL_IN = (EMITTER_X + COL[0] / 2, RECEIVER_X - COL[0] / 2)  # the lens faces (x)
ZONE = (TOP + 0.045, TOP + 0.155)  # the protected zone's height range
BAND = 0.040  # "close to" a column: within 4 cm of its lens face; "midway": within 4 cm of the centre line
MID_X = (COL_IN[0] + COL_IN[1]) / 2
STATIONS = {
    "emitter": (COL_IN[0], COL_IN[0] + BAND),
    "midway": (MID_X - BAND, MID_X + BAND),
    "receiver": (COL_IN[1] - BAND, COL_IN[1]),
}
COVER = 0.85  # a path counts once the piercing heights span 85 % of the zone
SLOW = 0.25  # m/s, the rod's vertical speed limit for a pass to count
NORMAL_DEG = 30.0  # the rod within 30 deg of the plane's normal
BIN = 0.002  # coverage is counted on 2 mm height bins (bin centres inside a swept interval)
MAX_GAP = 0.030  # consecutive valid samples further apart than this are not joined

ROD = (0.014, 0.200, 0.014)  # black acetal test rod, lying along y
ROD_KG = 0.055
HOLDER_XY = (0.250, -0.190)  # the rod's centre when home
SADDLE_H = 0.015  # saddle floor height above the table: the rod's resting bottom
SLOT = 0.030  # saddle slot width (x)
SADDLE_Y = (-0.020, 0.070)  # saddle centres along the rod, from the rod's centre
SADDLE = (0.040, 0.020)  # saddle base footprint (x, y)
POST = (0.008, 0.020, 0.010)  # the two posts on each saddle that form the slot
REST_Z = TOP + SADDLE_H + ROD[2] / 2  # rod centre when home

BED = (0.220, 0.200, 0.020)  # press bolster plate, behind the plane
BED_XY = (0.250, 0.075)
DIE = (0.120, 0.080, 0.025)
DIE_XY = (0.250, 0.090)

BLACK = (0.06, 0.06, 0.07)
YELLOW = (0.92, 0.74, 0.10)
LENS = (0.12, 0.02, 0.02)
STEEL = (0.50, 0.52, 0.55)
DARK_STEEL = (0.32, 0.33, 0.35)
GREY = (0.45, 0.46, 0.48)

kit = mk.Kit()
kit.part("test_rod", (*HOLDER_XY, REST_Z), [mk.box(ROD)], mass=ROD_KG, color=BLACK)


def _holder():
    out = []
    hx, hy = HOLDER_XY
    for i, dy in enumerate(SADDLE_Y):
        y = hy + dy
        out.append(Box(f"holder_saddle{i}", pos=(hx, y, TOP + SADDLE_H / 2), size=(SADDLE[0], SADDLE[1], SADDLE_H),
                       color=GREY))
        for s in (-1, 1):
            out.append(Box(f"holder_post{i}{'np'[s > 0]}", pos=(hx + s * (SLOT / 2 + POST[0] / 2), y,
                                                               TOP + SADDLE_H + POST[2] / 2), size=POST, color=GREY))
    return out


FIXTURES = [
    Box("emitter", pos=(EMITTER_X, PLANE_Y, TOP + COL[2] / 2), size=COL, color=YELLOW),
    Box("receiver", pos=(RECEIVER_X, PLANE_Y, TOP + COL[2] / 2), size=COL, color=YELLOW),
    Box("emitter_lens", pos=(COL_IN[0] + 0.001, PLANE_Y, (ZONE[0] + ZONE[1]) / 2), size=(0.002, 0.010, ZONE[1] - ZONE[0]),
        color=LENS),
    Box("receiver_lens", pos=(COL_IN[1] - 0.001, PLANE_Y, (ZONE[0] + ZONE[1]) / 2),
        size=(0.002, 0.010, ZONE[1] - ZONE[0]), color=LENS),
    Box("bolster", pos=(*BED_XY, TOP + BED[2] / 2), size=BED, color=DARK_STEEL),
    Box("lower_die", pos=(*DIE_XY, TOP + BED[2] + DIE[2] / 2), size=DIE, color=STEEL),
    *_holder(),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------

_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
_KEY = "lc_bins"  # station -> set of covered height bins (this trial)
_LAST = "lc_last"  # the previous valid piercing sample: (station, z)
_NBINS = int(round((ZONE[1] - ZONE[0]) / BIN))


def _released(st, margin=0.025) -> bool:
    """Neither gripper's TCP within 2.5 cm of the rod's bounding box."""
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _axis(objects) -> np.ndarray:
    """The rod's long axis in world (unit)."""
    c = kit.point(objects, "test_rod", (0.0, 0.0, 0.0))
    return kit.point(objects, "test_rod", (0.0, 1.0, 0.0)) - c


def _pierce(objects):
    """(x, z) where the rod's axis pierces the sensing plane, if the rod is held across it (within 30 deg of the
    plane's normal) and actually passes through it; else None."""
    u = _axis(objects)
    if abs(u[1]) < math.cos(math.radians(NORMAL_DEG)):
        return None
    c = kit.point(objects, "test_rod", (0.0, 0.0, 0.0))
    t = (PLANE_Y - c[1]) / u[1]
    if abs(t) > ROD[1] / 2:
        return None
    p = c + t * u
    return float(p[0]), float(p[2])


def _station(x):
    for name, (lo, hi) in STATIONS.items():
        if lo <= x <= hi:
            return name
    return None


def _bins(z0, z1) -> set:
    """Height bins whose centres lie in [z0, z1]."""
    lo, hi = min(z0, z1), max(z0, z1)
    i0 = max(0, math.ceil((lo - ZONE[0]) / BIN - 0.5))
    i1 = min(_NBINS - 1, math.floor((hi - ZONE[0]) / BIN - 0.5))
    return set(range(i0, i1 + 1))


def _record(objects) -> None:
    """Log the rod's piercing point for this step (called every control step and from score())."""
    rod = objects["test_rod"]
    hit = _pierce(objects)
    station = _station(hit[0]) if hit else None
    if station is None or abs(float(rod.lin_vel[2])) >= SLOW:
        mk.MEMORY[_LAST] = None
        return
    z = hit[1]
    covered = mk.MEMORY.setdefault(_KEY, {})
    last = mk.MEMORY.get(_LAST)
    z0 = last[1] if last is not None and last[0] == station and abs(last[1] - z) <= MAX_GAP else z
    covered.setdefault(station, set()).update(_bins(z0, z))
    mk.MEMORY[_LAST] = (station, z)


kit.on_step(_record)


def coverage() -> dict:
    """Fraction of the zone's height swept at each station so far this trial."""
    got = mk.MEMORY.get(_KEY, {})
    return {s: len(got.get(s, ())) / _NBINS for s in STATIONS}


def _home(objects) -> bool:
    """The rod lying in its holder: centre within 1.5 cm (along the rod) and 8 mm (across) of home, its bottom on
    the saddles, level and along y within 10 deg, still and let go."""
    rod = objects["test_rod"]
    c = kit.point(objects, "test_rod", (0.0, 0.0, 0.0))
    u = _axis(objects)
    return bool(abs(c[1] - HOLDER_XY[1]) < 0.015 and abs(c[0] - HOLDER_XY[0]) < 0.008
                and TOP + SADDLE_H - 0.002 < float(rod.low[2]) < TOP + SADDLE_H + 0.004
                and abs(u[1]) > math.cos(math.radians(10.0)) and abs(u[2]) < math.sin(math.radians(5.0))
                and rod.resting() and _released(rod))


def score(objects) -> float:
    """0.25 for each of the three paths (close to the emitter, midway, close to the receiver) swept over 85 % of
    the zone's height, slowly and with the rod across the plane, at any time in the trial; plus 0.25 if the rod is
    back in its holder, let go and still, once at least one path has been swept."""
    _record(objects)
    n = sum(v >= COVER for v in coverage().values())
    return 0.25 * n + (0.25 if n >= 1 and _home(objects) else 0.0)


def success(objects) -> bool:
    """All three paths swept and the rod home; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Zone height still to sweep, summed over the three paths (m), plus the rod's distance from home."""
    c = kit.point(objects, "test_rod", (0.0, 0.0, 0.0))
    left = sum(max(0.0, COVER - v) for v in coverage().values()) * (ZONE[1] - ZONE[0])
    return float(left + math.hypot(c[0] - HOLDER_XY[0], c[1] - HOLDER_XY[1]))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Right arm only, open loop from the scene's fixed geometry. The fingers close along x (yaw 90 deg) on the rod's
# near end, 8 cm behind its centre, so the rod sticks out along +y across the plane while the hand stays in front
# of the curtain. Each path: rod in at the top, a slow 4 s pass down through the zone, back up.

YAW = math.pi / 2
TIP = 0.013
SQUEEZE = 0.008
GRIP_DY = -0.080  # grip point from the rod's centre, along the rod
GRIP = -(ROD[0] - SQUEEZE) / 2
ROD_BELOW_TCP = TIP + 0.002 - ROD[2] / 2  # pad tips 2 mm above the rod's bottom: rod centre this far below the TCP
SWEEP_Y = PLANE_Y - 0.040 + GRIP_DY  # TCP y during the passes: the rod's axis pierces the plane 4 cm past its centre
TOP_Z = ZONE[1] + 0.007 + ROD_BELOW_TCP  # rod centre 7 mm above the zone's top
LOW_Z = ZONE[0] + 0.008 + ROD_BELOW_TCP  # ... and 8 mm above its bottom
PATH_X = (STATIONS["emitter"][0] + 0.025, MID_X, STATIONS["receiver"][1] - 0.025)


def demo():
    """Right arm: pick the rod from its holder at its near end, lift it, slide it into the plane at the top of
    the zone near the emitter, pass it slowly down and back up there, then midway, then near the receiver, draw it
    back out and lay it back in its holder."""
    hx, hy = HOLDER_XY
    gy = hy + GRIP_DY
    pick_z = REST_Z + ROD_BELOW_TCP
    place_z = pick_z + 0.002
    o, g = FINGER_OPEN, GRIP
    wps = [
        Waypoint(2.0, (hx, gy, TOP_Z), o, "over the rod's grip end", YAW),
        Waypoint(1.5, (hx, gy, pick_z + 0.020), o, "descend", YAW),
        Waypoint(1.0, (hx, gy, pick_z), o, "down slowly", YAW),
        Waypoint(0.5, (hx, gy, pick_z), o, "settle", YAW),
        Waypoint(1.2, (hx, gy, pick_z), g, "close", YAW),
        Waypoint(0.5, (hx, gy, pick_z), g, "settle closed", YAW),
        Waypoint(1.5, (hx, gy, TOP_Z), g, "lift", YAW),
        Waypoint(1.5, (PATH_X[0], gy, TOP_Z), g, "to the emitter side, in front of the plane", YAW),
    ]
    for i, x in enumerate(PATH_X):
        name = ("near the emitter", "midway", "near the receiver")[i]
        if i:
            wps.append(Waypoint(1.5, (x, SWEEP_Y, TOP_Z), g, f"along the top to {name}", YAW))
        else:
            wps.append(Waypoint(1.5, (x, SWEEP_Y, TOP_Z), g, "slide the rod into the plane at the top", YAW))
        wps += [
            Waypoint(0.5, (x, SWEEP_Y, TOP_Z), g, f"{name}: settle", YAW),
            Waypoint(4.0, (x, SWEEP_Y, LOW_Z), g, f"{name}: slowly down through the zone", YAW),
            Waypoint(0.4, (x, SWEEP_Y, LOW_Z), g, f"{name}: bottom", YAW),
            Waypoint(2.5, (x, SWEEP_Y, TOP_Z), g, f"{name}: back up", YAW),
        ]
    wps += [
        Waypoint(1.5, (PATH_X[-1], gy, TOP_Z), g, "draw the rod back out of the plane", YAW),
        Waypoint(1.5, (hx, gy, TOP_Z), g, "over the holder", YAW),
        Waypoint(0.5, (hx, gy, TOP_Z), g, "settle over", YAW),
        Waypoint(1.5, (hx, gy, place_z + 0.020), g, "lower", YAW),
        Waypoint(1.2, (hx, gy, place_z), g, "set down slowly", YAW),
        Waypoint(0.5, (hx, gy, place_z), g, "settle", YAW),
        Waypoint(1.2, (hx, gy, place_z), o, "release", YAW),
        Waypoint(1.5, (hx, gy, TOP_Z), o, "retreat", YAW),
    ]
    return {"right": wps}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of rod poses (one trial each, fed step by step at 20 Hz)."""
    global _TEST_TCPS

    def rod(x, y, z, yaw_deg=0.0, pitch_deg=0.0, vz=0.0):
        """The rod's centre at (x, y, z), its axis along y turned yaw about z and pitched about x."""
        r = mk.axis_angle((0, 0, 1), math.radians(yaw_deg)) @ mk.axis_angle((1, 0, 0), math.radians(pitch_deg))
        st = kit.fake_states(moved={"test_rod": ((x, y, z), mk.mat_to_quat(r))})
        st["test_rod"] = dataclasses.replace(st["test_rod"], lin_vel=np.array([0.0, 0.0, vz]))
        return st

    home = rod(*HOLDER_XY, REST_Z)
    held = rod(*HOLDER_XY, REST_Z + 0.10)
    cy = PLANE_Y - 0.040  # rod centre y during a pass (the axis pierces the plane 4 cm past the centre)
    hi, lo = ZONE[1] + 0.007, ZONE[0] + 0.008

    def sweep(x, z0=hi, z1=lo, speed=0.06, **kw):
        n = max(2, int(abs(z1 - z0) / (speed / 20.0)))
        v = math.copysign(speed, z1 - z0)
        return [rod(x, cy, z0 + (z1 - z0) * k / n, vz=v, **kw) for k in range(n + 1)]

    def three(**kw):
        return [f for x in PATH_X for f in sweep(x, **kw)]

    def diagonal():
        n = 120
        return [rod(PATH_X[0] + (PATH_X[2] - PATH_X[0]) * k / n, cy, hi + (lo - hi) * k / n, vz=-0.05)
                for k in range(n + 1)]

    on_bed = rod(0.25, 0.08, TOP + BED[2] + DIE[2] + 0.007)
    grip_tcp = kit.point(home, "test_rod", (0.0, GRIP_DY, 0.010))

    cases = {  # name: (frames in one trial, TCPs at the end, score, success)
        "start": ([home], [], 0.0, False),
        "rod lifted out and held": ([home, held], [], 0.0, False),
        "picked up and put straight back": ([home, held, home], [], 0.0, False),
        "one pass midway, rod put back": ([home, held, *sweep(PATH_X[1]), held, home], [], 0.5, False),
        "one pass midway, rod still held there": ([home, held, *sweep(PATH_X[1])], [], 0.25, False),
        "three passes, rod put back": ([home, held, *three(), held, home], [], 1.0, True),
        "three passes, passing upward instead": ([home, held, *three(z0=lo, z1=hi), held, home], [], 1.0, True),
        "three passes, rod left on the die": ([home, held, *three(), on_bed], [], 0.75, False),
        "three passes, rod home but still gripped": ([home, held, *three(), held, home], [grip_tcp], 0.75, False),
        "three passes too fast (0.5 m/s)": ([home, held, *three(speed=0.5), held, home], [], 0.0, False),
        "three passes with the rod held upright": ([home, held, *three(pitch_deg=90.0), held, home], [], 0.0, False),
        "three passes with the rod turned 45 deg": ([home, held, *three(yaw_deg=45.0), held, home], [], 0.0, False),
        "three passes, rod tilted 8 deg (still across)": ([home, held, *three(pitch_deg=8.0), held, home], [],
                                                           1.0, True),
        "three passes over the top half only": ([home, held, *three(z1=(hi + lo) / 2), held, home], [], 0.0, False),
        "one diagonal sweep across all three paths": ([home, held, *diagonal(), held, home], [], 0.0, False),
        "passes between the paths only": ([home, held, *sweep(STATIONS["emitter"][1] + 0.008),
                                           *sweep(STATIONS["receiver"][0] - 0.008), held, home], [], 0.0, False),
        "rod poked in short of the plane": ([home, held, *[rod(PATH_X[1], PLANE_Y - 0.120, z) for z in
                                                         np.linspace(hi, lo, 60)], held, home], [], 0.0, False),
        "three passes, put back, picked up again": ([home, held, *three(), held, home, held], [], 0.75, False),
        "rod back but lying on the saddle posts": ([home, held, *three(), rod(HOLDER_XY[0] + 0.019, HOLDER_XY[1],
                                                                                REST_Z + 0.010)], [], 0.75, False),
    }
    try:
        for name, (frames, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for f in frames[:-1]:
                score(f)
            _TEST_TCPS = tcps
            got, got_success = score(frames[-1]), success(frames[-1])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success,
                                                                            coverage())
        # a new trial forgets the last one's passes
        mk.new_episode()
        _TEST_TCPS = []
        for f in [home, held, *three()]:
            score(f)
        mk.new_episode()
        assert score(held) == 0.0 and score(home) == 0.0, "reset"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the right arm's plan replayed through
    # the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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
