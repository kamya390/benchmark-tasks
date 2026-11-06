"""On the station table, in front of the right arm, stands a bench drill press: a fixed column and head, and in
front of the head the quill, which slides vertically on a return spring. A 6 mm drill hangs from the quill's chuck
over a 40 mm steel block held in a vise; a red feed knob sticks out from the quill toward the robot. Pushing the
knob down feeds the drill into the block; letting it go, the spring brings the quill back up. The quill's lower
stop is the depth stop, 30 mm below the block's top. The job is to drill the hole down to the depth stop and bring
the drill back up.

Mechanisms (mechkit.py, in this folder): the quill is a sprung slider with Coulomb friction; the drill is held in
the chuck by a bond. The chip-clearing rule (peck drilling) is emulated: a feed that runs more than 9 mm (1.5
drill diameters) deeper than the depth the drill had reached at its last full retraction (tip at least 2 mm out of
the hole) packs the flutes, and the drill snaps: its bond is released and the bit stays in the hole.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/peck-drill-deep-hole/peck_drill_deep_hole.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (sliders, bonds; installs its yam_arena hooks in memory, incl. mk.MEMORY reset)

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
DX, DY = 0.340, -0.200  # the drill's axis
BLOCK = (0.050, 0.040, 0.040)
BT = TOP + BLOCK[2]  # the block's top
BORE = 0.010  # the hole the drill runs in (square, 2 mm clear of the drill each side)
BORE_DEPTH = 0.031
DRILL = (0.006, 0.006, 0.045)
CLEAR0 = 0.008  # the drill's tip starts 8 mm above the block
DEPTH_STOP = 0.030
TRAVEL = CLEAR0 + DEPTH_STOP  # the quill's full travel (its lower stop is the depth stop)
CHUCK = (0.022, 0.022, 0.020)
QUILL = (0.030, 0.030, 0.070)
ARM = (0.060, 0.016, 0.014)  # the feed knob's stem, out toward the robot
KNOB = (0.030, 0.030, 0.015)
KNOB_X = DX - QUILL[0] / 2 - ARM[0] + KNOB[0] / 2 - 0.005
QUILL_KG, DRILL_KG = 0.150, 0.010
SPRING_K = 30.0  # N/m return spring ...
SPRING_UP = 3.5  # ... pushing the quill up with 3.5 N at its top stop (weight 1.6 N, friction 1 N)
FRICTION = 1.0  # N

TIP0 = BT + CLEAR0
CHUCK_Z = TIP0 + DRILL[2]  # the chuck's underside
KNOB_TOP = CHUCK_Z + 0.022  # the knob's top at the start (quill up)

STEEL = (0.45, 0.47, 0.50)
CAST = (0.22, 0.34, 0.30)  # machine-green cast iron
DARK = (0.12, 0.12, 0.13)
RED = (0.75, 0.10, 0.08)
HSS = (0.30, 0.30, 0.33)
CHROME = (0.78, 0.79, 0.80)

kit = mk.Kit()
# The quill: frame on the drill's axis at the chuck's underside (reference pose: quill up, at its top stop).
kit.part("quill", (DX, DY, CHUCK_Z), [
    mk.box(CHUCK, at=(0.0, 0.0, CHUCK[2] / 2), color=CHROME),
    mk.box(QUILL, at=(0.0, 0.0, CHUCK[2] + QUILL[2] / 2)),
    mk.box(ARM, at=(-QUILL[0] / 2 - ARM[0] / 2, 0.0, 0.016), color=DARK),
    mk.box(KNOB, at=(KNOB_X - DX, 0.0, 0.022 - KNOB[2] / 2), color=RED),
], mass=QUILL_KG, color=CHROME)
kit.slider("feed", None, "quill", axis=(0, 0, -1), lower=0.0, upper=TRAVEL, start=0.0, stiffness=SPRING_K,
           rest=-SPRING_UP / SPRING_K, friction=FRICTION)
# The drill: frame at its tip, held in the chuck by a bond the chip rule can release (the drill snaps).
kit.part("drill", (DX, DY, TIP0), [mk.box(DRILL, at=(0.0, 0.0, DRILL[2] / 2))], mass=DRILL_KG, color=HSS)
kit.bond("chuck_grip", "quill", "drill", anchor=(DX, DY, CHUCK_Z))


def _block_boxes():
    """The block as boxes round its bore: a floor under the bore, two side walls along y, two end walls along x."""
    bx, by, _ = BLOCK
    z0, zb = TOP, BT - BORE_DEPTH
    w = (bx - BORE) / 2
    e = (by - BORE) / 2
    hz = BORE_DEPTH
    return [
        Box("block_floor", pos=(DX, DY, (z0 + zb) / 2), size=(bx, by, zb - z0), color=STEEL),
        Box("block_near", pos=(DX - BORE / 2 - w / 2, DY, zb + hz / 2), size=(w, by, hz), color=STEEL),
        Box("block_far", pos=(DX + BORE / 2 + w / 2, DY, zb + hz / 2), size=(w, by, hz), color=STEEL),
        Box("block_left", pos=(DX, DY + BORE / 2 + e / 2, zb + hz / 2), size=(BORE, e, hz), color=STEEL),
        Box("block_right", pos=(DX, DY - BORE / 2 - e / 2, zb + hz / 2), size=(BORE, e, hz), color=STEEL),
    ]


_HEAD_Z0 = CHUCK_Z + 0.010
_HEAD_TOP = CHUCK_Z + 0.120
FIXTURES = [
    *_block_boxes(),
    *(Box(f"vise_jaw_{n}", pos=(DX, DY + sy * (BLOCK[1] / 2 + 0.008), TOP + 0.015), size=(0.060, 0.016, 0.030),
          color=CAST) for n, sy in (("left", 1), ("right", -1))),
    Box("vise_screw_block", pos=(DX, DY - BLOCK[1] / 2 - 0.030, TOP + 0.012), size=(0.030, 0.012, 0.024), color=CAST),
    Box("head", pos=(DX + QUILL[0] / 2 + 0.002 + 0.035, DY, (_HEAD_Z0 + _HEAD_TOP) / 2),
        size=(0.070, 0.080, _HEAD_TOP - _HEAD_Z0), color=CAST),
    Box("column", pos=(DX + 0.120, DY, (TOP + _HEAD_TOP) / 2), size=(0.040, 0.040, _HEAD_TOP - TOP), color=CAST),
    Box("depth_scale", pos=(DX + QUILL[0] / 2 + 0.0015, DY + 0.025, CHUCK_Z + 0.040), size=(0.002, 0.012, 0.050),
        color=CHROME),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Read from the quill's and the drill's poses: the drill's depth (its tip below the block's top) is what the quill's
# depth scale shows. Per trial (mk.MEMORY, cleared at the embodiment's reset): the deepest depth so far, and the
# deepest depth at the drill's last full retraction.

OUT = 0.002  # a full retraction: the tip at least 2 mm above the block's top
PECK = 0.009  # the chip rule: no feed more than 9 mm (1.5 D) past the depth at the last full retraction
UP_END = 0.005  # at the end the drill must stand at least 5 mm above the block
FULL = DEPTH_STOP - 0.001
HALF_D = 0.015
QUARTER_D = 0.008
_TEST_TCPS = None
K_DEEP, K_CLEAR, K_SNAP = "pdh_deepest", "pdh_depth_at_clear", "pdh_snapped"


def _released(st, margin=0.025) -> bool:
    if _TEST_TCPS is None:
        return mk.released(st, margin)
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _TEST_TCPS)


def depth(objects) -> float:
    """The drill's depth in the block, m: its tip below the block's top, read through the quill (the depth scale).
    Negative while the tip is above the block."""
    return float(kit.q(objects, "feed")) - CLEAR0


def tip_height(objects) -> float:
    """The drill's own tip above the block's top, m (from the drill's pose; it stays in the hole if it snapped)."""
    return float(kit.point(objects, "drill", (0.0, 0.0, 0.0))[2]) - BT


def _update(objects) -> bool:
    """Track this trial's feed history; returns True once the chip rule has been broken (the drill snapped)."""
    m = mk.MEMORY
    d = depth(objects)
    deep = max(m.get(K_DEEP, 0.0), d)
    m[K_DEEP] = deep
    if d <= -OUT:
        m[K_CLEAR] = deep
    if d > m.get(K_CLEAR, 0.0) + PECK or kit.broken(objects, "chuck_grip"):
        m[K_SNAP] = True
    return bool(m.get(K_SNAP))


def _chip_rule(objects) -> None:
    """Sim only (every control step): the drill snaps once the chip rule is broken."""
    if _update(objects) and "chuck_grip" not in kit._released:
        kit.release("chuck_grip")


kit.on_step(_chip_rule)


def score(objects) -> float:
    """0 for the rest of the trial once a feed ran more than 9 mm deeper than the depth reached at the drill's last
    full retraction (tip 2 mm out of the hole), which snaps the drill. Otherwise, with the drill back up (its tip
    at least 5 mm above the block), the quill still and let go: 1.0 if the hole reached the depth stop (29 mm or
    more), 0.5 for 15 mm or more, 0.25 for 8 mm or more."""
    if _update(objects):
        return 0.0
    quill = objects["quill"]
    if tip_height(objects) < UP_END or not quill.resting() or not _released(quill):
        return 0.0
    deep = mk.MEMORY.get(K_DEEP, 0.0)
    if deep >= FULL:
        return 1.0
    if deep >= HALF_D:
        return 0.5
    return 0.25 if deep >= QUARTER_D else 0.0


def success(objects) -> bool:
    """Drilled to the stop in pecks, the drill back up and let go; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres of depth still to drill."""
    return float(max(0.0, DEPTH_STOP - max(mk.MEMORY.get(K_DEEP, 0.0), depth(objects))))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the drill press's fixed geometry; it reads no joint or simulator state. The closed fingers press the
# feed knob's top down to each peck depth, then lift, and the return spring brings the drill out of the hole.

TIP = 0.013
C = FINGER_CLOSED
PECKS = (0.007, 0.014, 0.021, 0.028, 0.035)  # commanded depths; the last runs onto the depth stop (30 mm)
LAG = 0.005  # commanded extra push: the arm gives ~5 mm under the spring load (measured in sim)
ABOVE = 0.012  # the fingers lift this far above the knob's top at the start, so the quill returns to its stop


def _press_z(d):
    return KNOB_TOP - (CLEAR0 + d) - LAG + TIP


def demo():
    """Right arm only: closed fingers onto the feed knob, then five pecks (7, 14, 21, 28 mm and onto the 30 mm
    stop), each followed by a full retraction (the fingers lift 12 mm above the knob's start height and wait while
    the spring returns the quill), then lift away."""
    x, y = KNOB_X, DY
    top = KNOB_TOP + ABOVE + TIP
    wps = [
        Waypoint(2.0, (x, y, top + 0.030), C, "over the feed knob", 0.0),
        Waypoint(1.0, (x, y, top), C, "down above the knob", 0.0),
        Waypoint(0.4, (x, y, top), C, "settle", 0.0),
    ]
    for i, d in enumerate(PECKS, 1):
        wps += [
            Waypoint(0.5, (x, y, KNOB_TOP + TIP + 0.001), C, f"peck {i}: onto the knob", 0.0),
            Waypoint(0.8 + 25.0 * d, (x, y, _press_z(d)), C, f"peck {i}: feed to {d * 1000:.0f} mm", 0.0),
            Waypoint(0.3, (x, y, _press_z(d)), C, f"peck {i}: dwell", 0.0),
            Waypoint(0.8, (x, y, top), C, f"peck {i}: retract", 0.0),
            Waypoint(0.6, (x, y, top), C, f"peck {i}: clear the chips", 0.0),
        ]
    wps += [Waypoint(1.5, (x - 0.060, y, top + 0.040), C, "lift away, clear of the quill", 0.0)]
    return {"right": wps}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of quill positions (one trial each)."""
    global _TEST_TCPS
    import dataclasses

    def at(d, snapped=False):
        """The quill fed so the drill's tip is ``d`` m deep (negative: above the block)."""
        s = kit.fake_states(q={"feed": d + CLEAR0})
        if snapped:  # the bit left in the hole at the bore's bottom, the quill where it is
            tip = BT - BORE_DEPTH
            s = kit.fake_states(q={"feed": d + CLEAR0}, moved={"drill": (np.array([DX, DY, tip]), np.array([0, 0, 0, 1.0]))})
        return s

    up = at(-CLEAR0)
    out = at(-0.003)

    def pecks(*depths):
        seq = [up]
        for d in depths:
            seq += [at(d), up]
        return seq

    good = pecks(0.007, 0.014, 0.021, 0.028, 0.030)
    moving = dict(up)
    moving["quill"] = dataclasses.replace(up["quill"], lin_vel=np.array([0.0, 0.0, 0.05]))
    knob_tcp = (KNOB_X, DY, KNOB_TOP + 0.005)
    cases = {  # name: (states, TCPs at the last state, score, success)
        "start": ([up], [], 0.0, False),
        "one peck to 8 mm, retracted": (pecks(0.008), [], 0.25, False),
        "two pecks to 15 mm": (pecks(0.008, 0.015), [], 0.5, False),
        "five pecks onto the stop": (good, [], 1.0, True),
        "four pecks of just under 9 mm onto the stop": (pecks(0.0089, 0.0178, 0.0267, 0.030), [], 1.0, True),
        "at the stop, drill still in the hole": (good[:-1], [], 0.0, False),
        "done but the fingers still on the knob": (good, [knob_tcp], 0.0, False),
        "done, quill still moving": (good[:-1] + [moving], [], 0.0, False),
        "naive: one plunge to the stop": ([up, at(0.030), up], [], 0.0, False),
        "plunge to 10 mm (1 mm over the first peck)": ([up, at(0.010), up], [], 0.0, False),
        "peck retracted only to 1 mm above the block": ([up, at(0.008), at(-0.001), at(0.018), up], [], 0.0, False),
        "the same with a 2 mm retraction is fine": ([up, at(0.008), at(-0.0021), at(0.016), up], [], 0.5, False),
        "retracted 3 mm out each time (enough)": ([up, at(0.008), out, at(0.016), out, at(0.024), out, at(0.030), up],
                                                 [], 1.0, True),
        "snapped drill left in the hole": ([up, at(0.008, snapped=True), at(-CLEAR0, snapped=True)], [], 0.0, False),
        "a new trial after a snap starts clean": ([up, at(0.030), up, None] + good, [], 1.0, True),
    }
    try:
        for name, (seq, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for s in seq[:-1]:
                if s is None:
                    mk.new_episode()
                else:
                    score(s)
            _TEST_TCPS = tcps
            got, got_success = score(seq[-1]), success(seq[-1])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        assert abs(depth(at(0.012)) - 0.012) < 1e-9 and abs(tip_height(up) - CLEAR0) < 1e-9
    finally:
        mk.new_episode()
        _TEST_TCPS = None
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
