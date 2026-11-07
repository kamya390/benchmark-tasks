"""On the station table, in front of the right arm, is a small hinged lift-arm assembly brought in for weld repair:
a grey painted steel base frame, and a yellow painted steel lift arm joined to it by a pivot pin running through
bronze bushings in two clevis ears. The arm reaches forward from the pivot and rests on its lift cylinder's rod end;
an orange paint mark across its far end shows a crack that is going to be welded. Two bare steel fins (8 mm plates,
4 cm tall) stand up as clamping lugs: one on the base frame, nearest the robot, and one on the lift arm, about 8 cm
from the crack. The welder's work clamp, a spring clamp, hangs on its bracket to the right of the assembly. The job
is to put the work clamp on for the repair and let go. Scored from the poses of the clamp's two halves at the end.

The clamp is two steel halves on a pivot with a torsion spring (about 3 N at the jaw teeth when they grip an 8 mm
fin). Squeezing its two handles together opens the jaws; let go on a fin, the spring closes the teeth on the fin's
faces and the clamp sits on the fin's top edge. Welding current flows from the arc to wherever the work clamp is
attached: on the frame fin it would cross the pivot pin and bushings (arcing pits them), on the arm fin it stays in
the arm being repaired.

Mechanisms (mechkit.py, in this folder, adds the joints to the stock runtime in memory): the clamp pivot (a sprung
hinge between the two free halves) and the lift-arm pivot (a hinge with friction, +-3 deg).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/weld-work-clamp-same-member/weld_work_clamp_same_member.py \\
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
YW = -0.180  # the assembly's centre line (runs along x)
FIN_T, FIN_H = 0.008, 0.040  # every clamping fin: 8 mm bare steel plate, 40 mm tall
FRAME_FIN = (0.200, 0.080, TOP + 0.020)  # centre x, length (x), base z: on the frame's base block
PIVOT = (0.280, YW, TOP + 0.0325)  # lift-arm pivot pin (axis along y)
ARM_FIN = (0.360, 0.080, TOP + 0.045)  # on the lift arm's top
CRACK_X = 0.440
HANGER = (0.280, -0.330, 0.060, TOP + 0.020)  # the clamp's bracket fin: centre x, y, length, base z
ARM_KG = 0.40

# The clamp, in its own frame: origin on the pivot, x along the pivot axis (world x), the halves open in y-z.
CD = 0.030  # clamp depth along the pivot axis
TOOTH = (0.005, 0.012)  # each tooth: reach in from its jaw plate (y), height (z)
JAW_Z0, JAW_Z1 = -0.047, -0.010  # jaw plates span this z (teeth at the bottom)
BOSS_Z = -0.012  # the pivot boss's underside: rests on the fin's top edge
HANDLE_C, HANDLE_L, HANDLE_T, HANDLE_TILT = (0.026, 0.040), 0.055, 0.008, 14.0  # centre (y, z), length, thick, deg out
BODY_KG, JAW_KG = 0.12, 0.08
SPRING_K, SPRING_REST = 0.36, -8.0  # N*m/rad, deg (preloaded closed against the 0 deg stop)
OPEN_MAX = 40.0
TOOTH_R = -(JAW_Z0 + TOOTH[1] / 2)  # teeth centre below the pivot


def _on_fin_deg(thick=FIN_T) -> float:
    return math.degrees(2 * math.asin(thick / 2 / TOOTH_R))


Q0 = _on_fin_deg() + 0.8  # spawn with the teeth 0.3 mm off the bracket fin's faces

STEEL = (0.66, 0.67, 0.69)
PAINT_GREY = (0.40, 0.42, 0.45)
YELLOW = (0.90, 0.70, 0.08)
ORANGE = (1.00, 0.45, 0.05)
BRONZE = (0.70, 0.50, 0.25)
DARK = (0.10, 0.10, 0.11)
CLAMP_RED = (0.75, 0.12, 0.10)
COPPER = (0.80, 0.45, 0.25)


def _qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _half(side, boss=False):
    """Boxes of one clamp half; side +1 = the +y half, -1 = the -y half."""
    s = side
    tooth_y = (TOOTH[0] / 2) * s
    plate_y = (TOOTH[0] + 0.003) * s
    bx = [
        mk.box((CD, TOOTH[0], TOOTH[1]), at=(0, tooth_y, JAW_Z0 + TOOTH[1] / 2), color=COPPER),
        mk.box((CD, 0.006, JAW_Z1 - JAW_Z0), at=(0, plate_y, (JAW_Z0 + JAW_Z1) / 2)),
        mk.box((CD, 0.020, 0.020), at=(0, 0.017 * s, 0.0)),  # bridge from jaw to handle
        mk.box((CD, HANDLE_T, HANDLE_L), at=(0, HANDLE_C[0] * s, HANDLE_C[1]), quat=_qx(-HANDLE_TILT * s),
               color=DARK),
    ]
    if boss:
        bx.append(mk.box((CD, 0.012, 0.022), at=(0, 0, BOSS_Z + 0.011), color=STEEL))
    return bx


def _fin_top(fin):
    return fin[2] + FIN_H


kit = mk.Kit()
# Base frame (fixed): base block and clevis ears with the pivot pin and bushings (the pin is where the frame and the
# arm meet; the frame fin stands on the block, separately, as a plain fixed plate).
kit.part("frame", (0.200, YW, TOP), [
    mk.box((0.140, 0.090, 0.020), at=(0.0, 0, 0.010)),
    *(mk.box((0.030, 0.008, 0.050), at=(0.080, s * 0.028, 0.025)) for s in (1, -1)),
    mk.box((0.012, 0.076, 0.012), at=(0.080, 0, PIVOT[2] - TOP), color=STEEL),
    *(mk.box((0.020, 0.006, 0.020), at=(0.080, s * 0.035, PIVOT[2] - TOP), color=BRONZE) for s in (1, -1)),
], color=PAINT_GREY)
_ax, _al, _az = ARM_FIN
kit.part("lift_arm", PIVOT, [
    mk.box((0.260, 0.044, 0.025), at=(0.120, 0, 0.0)),
    mk.box((_al, FIN_T, FIN_H), at=(_ax - PIVOT[0], 0, _az + FIN_H / 2 - PIVOT[2]), color=STEEL),
    mk.box((0.006, 0.045, 0.0012), at=(CRACK_X - PIVOT[0], 0, 0.0128), color=ORANGE),
    *(mk.box((0.006, 0.0012, 0.020), at=(CRACK_X - PIVOT[0], s * 0.0226, 0.0), color=ORANGE) for s in (1, -1)),
], mass=ARM_KG, color=YELLOW)
kit.hinge("arm_pivot", "frame", "lift_arm", anchor=PIVOT, axis=(0, 1, 0), lower_deg=-3.0, upper_deg=3.0,
          friction=0.5)

_hx, _hy, _hl, _hz = HANGER
CLAMP0 = (_hx, _hy, _hz + FIN_H - BOSS_Z + 0.0005)  # pivot of the clamp hanging on its bracket
kit.part("clamp_body", CLAMP0, _half(-1, boss=True), mass=BODY_KG, color=CLAMP_RED, quat=_qx(-Q0 / 2))
kit.part("clamp_jaw", CLAMP0, _half(+1), mass=JAW_KG, color=CLAMP_RED, quat=_qx(-Q0 / 2))
kit.hinge("clamp_pivot", "clamp_body", "clamp_jaw", anchor=CLAMP0, axis=(1, 0, 0), lower_deg=0.0,
          upper_deg=OPEN_MAX, start_deg=Q0, stiffness=SPRING_K, rest_deg=SPRING_REST)

FINS = {
    "frame": (FRAME_FIN[0], YW, FRAME_FIN[1], FRAME_FIN[2]),
    "arm": (ARM_FIN[0], YW, ARM_FIN[1], ARM_FIN[2]),
    "bracket": HANGER,
}
FIXTURES = [
    Box("frame_fin", pos=(FRAME_FIN[0], YW, FRAME_FIN[2] + FIN_H / 2), size=(FRAME_FIN[1], FIN_T, FIN_H), color=STEEL),
    Box("lift_cylinder", pos=(0.380, YW, TOP + 0.007), size=(0.160, 0.030, 0.014), color=DARK),
    Box("cylinder_rod_end", pos=(0.500, YW, TOP + 0.010), size=(0.040, 0.030, 0.020), color=STEEL),
    Box("bracket_post", pos=(_hx, _hy, TOP + 0.010), size=(_hl, 0.040, 0.020), color=PAINT_GREY),
    Box("bracket_fin", pos=(_hx, _hy, _hz + FIN_H / 2), size=(_hl, FIN_T, FIN_H), color=PAINT_GREY),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
# Welding practice (welder makers' manuals, ANSI Z49.1, equipment makers' weld-repair instructions): the work clamp
# goes on the part being welded, close to the weld, so the current never crosses pins, bushings, bearings or
# cylinders. Everything below is geometry from the two clamp halves' poses: where the pivot sits, how far the jaws
# are open, and where the grippers are.

JAW_MIN, JAW_MAX = 5.0, 20.0  # closed on an 8 mm fin is about 11 deg; fully shut is 0, held open is 25+
SEAT_DZ = (0.004, 0.022)  # pivot above the fin's top edge (it rests there 12 mm up)
SIDE_TOL = 0.006
END_IN = 0.008  # the clamp's centre at least 8 mm in from the fin's ends
_TEST_TCPS = None


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else [np.asarray(t, float) for t in _TEST_TCPS]


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())


def jaw_deg(objects) -> float:
    return kit.q_deg(objects, "clamp_pivot")


def clamped_on(objects, fin) -> bool:
    """The clamp sits on that fin with its teeth closed on it: pivot over the fin's top edge (within 6 mm of its
    mid-plane, 4-22 mm above the edge, 8 mm in from its ends) and the jaws open 5-20 deg (gripping the plate)."""
    cx, cy, ln, z0 = FINS[fin]
    p = kit.point(objects, "clamp_body", (0, 0, 0))
    dz = float(p[2]) - (z0 + FIN_H)
    return bool(abs(float(p[0]) - cx) <= ln / 2 - END_IN and abs(float(p[1]) - cy) <= SIDE_TOL
                and SEAT_DZ[0] <= dz <= SEAT_DZ[1] and JAW_MIN <= jaw_deg(objects) <= JAW_MAX)


def _settled(objects) -> bool:
    return all(objects[n].resting() and _released(objects[n]) for n in ("clamp_body", "clamp_jaw"))


def score(objects) -> float:
    """1.0: the clamp closed on the lift arm's fin, let go (both grippers 2.5 cm clear of both halves) and still.
    0.5: closed on the lift arm's fin but still held or moving. 0 otherwise: on the frame's fin (the current would
    cross the pivot), still on its bracket, or anywhere else."""
    if not clamped_on(objects, "arm"):
        return 0.0
    return 1.0 if _settled(objects) else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Metres from the clamp's pivot to its seat on the lift arm's fin."""
    p = kit.point(objects, "clamp_body", (0, 0, 0))
    goal = np.array([ARM_FIN[0], YW, _fin_top(ARM_FIN) - BOSS_Z])
    return float(np.linalg.norm(p - goal))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's fixed geometry; it reads no joint or simulator state. The right arm's pads squeeze the
# clamp's two handles (closing along y) so the jaws open, lift it off its bracket, lower it over the lift arm's fin
# and open slowly so the spring closes the teeth on the fin.

TCP_ABOVE_PIVOT = 0.040  # pad tips 27 mm above the pivot, on the handles
SQUEEZE_DEG = 28.0  # jaw opening commanded by the squeeze (the spring's push-back leaves it about 25 deg)
TRAVEL_Z = TOP + 0.180
DROP = 0.002  # let go with the boss 2 mm above the fin's top edge
X_TRIM = 0.0


def _handle_outer_y(q_deg) -> float:
    """Outermost y of the +y handle within the pads' height band, with the clamp opened q_deg symmetrically."""
    r = np.array([[1, 0, 0], [0, math.cos(math.radians(q_deg / 2)), -math.sin(math.radians(q_deg / 2))],
                  [0, math.sin(math.radians(q_deg / 2)), math.cos(math.radians(q_deg / 2))]])
    t = math.radians(-HANDLE_TILT)
    rt = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
    lo, hi = TCP_ABOVE_PIVOT - 0.013, TCP_ABOVE_PIVOT - 0.013 + 0.045
    best = 0.0
    for s in np.linspace(-0.5, 0.5, 101):
        p = r @ (np.array([0, HANDLE_C[0], HANDLE_C[1]]) + rt @ np.array([0, HANDLE_T / 2, s * HANDLE_L]))
        if lo <= p[2] <= hi:
            best = max(best, float(p[1]))
    return best


def demo():
    """Right arm only: open over the clamp on its bracket, down onto the handles, squeeze them (jaws open), lift,
    carry over the lift arm's fin, lower, open slowly (the teeth close on the fin), lift away and park."""
    o = FINGER_OPEN
    sq = -_handle_outer_y(SQUEEZE_DEG)
    px, py, pz = CLAMP0
    pick = (px, py, pz + TCP_ABOVE_PIVOT)
    gx, gy = ARM_FIN[0] + X_TRIM, YW
    place = (gx, gy, _fin_top(ARM_FIN) - BOSS_Z + DROP + TCP_ABOVE_PIVOT)
    right = [
        Waypoint(2.0, (px, py, TRAVEL_Z), o, "over the clamp"),
        Waypoint(1.5, pick, o, "down to the handles"),
        Waypoint(0.5, pick, o, "settle"),
        Waypoint(1.5, pick, sq, "squeeze the handles: jaws open"),
        Waypoint(0.5, pick, sq, "settle squeezed"),
        Waypoint(1.5, (px, py, TRAVEL_Z), sq, "lift off the bracket"),
        Waypoint(2.0, (gx, gy, TRAVEL_Z), sq, "carry over the lift arm's fin"),
        Waypoint(0.6, (gx, gy, TRAVEL_Z), sq, "settle over"),
        Waypoint(1.5, place, sq, "lower over the fin"),
        Waypoint(0.6, place, sq, "settle"),
        Waypoint(2.0, place, o, "open slowly: the teeth close on the fin"),
        Waypoint(0.5, place, o, "wait"),
        Waypoint(1.5, (gx, gy, TRAVEL_Z), o, "lift away"),
        Waypoint(2.0, (0.200, -0.300, TRAVEL_Z), o, "park"),
    ]
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), o, "park"),
            Waypoint(sum(w.duration for w in right) - 2.0, (0.150, 0.250, 0.950), o, "park")]
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built clamp poses (forward kinematics of the two halves)."""
    import dataclasses

    global _TEST_TCPS
    on = _on_fin_deg()

    def at(fin, q=on, dx=0.0, dy=0.0, dz=0.0, roll=None, moving=False):
        cx, cy, _, z0 = FINS[fin]
        pos = np.array([cx + dx, cy + dy, z0 + FIN_H - BOSS_Z + dz])
        s = kit.fake_states(q={"clamp_pivot": math.radians(q)},
                            moved={"clamp_body": (pos, _qx(-q / 2 if roll is None else roll))})
        if moving:
            s["clamp_body"] = dataclasses.replace(s["clamp_body"], lin_vel=np.array([0.0, 0.0, -0.05]))
        return s

    def tcp_on(fin, dz=TCP_ABOVE_PIVOT):
        cx, cy, _, z0 = FINS[fin]
        return [(cx, cy, z0 + FIN_H - BOSS_Z + dz)]

    far = [(0.20, -0.30, TRAVEL_Z), (0.15, 0.25, 0.95)]
    cases = {  # name: (state, tcps, score, success)
        "start: the clamp on its bracket": (kit.fake_states(), [], 0.0, False),
        "start, kit spawn pose reads as clamped on the bracket": (kit.fake_states(), far, 0.0, False),
        "on the lift arm's fin, let go, still": (at("arm"), far, 1.0, True),
        "on the lift arm's fin near one end (12 mm in)": (at("arm", dx=0.028), far, 1.0, True),
        "on the lift arm's fin, still squeezed in the gripper": (at("arm", q=on), tcp_on("arm"), 0.5, False),
        "on the lift arm's fin, still settling": (at("arm", moving=True), far, 0.5, False),
        "naive: on the frame's fin (current through the pivot)": (at("frame"), far, 0.0, False),
        "left on its bracket": (at("bracket"), far, 0.0, False),
        "held open over the arm fin (jaws 26 deg)": (at("arm", q=26.0, dz=0.004), tcp_on("arm"), 0.0, False),
        "jaws shut, balanced on the fin's top edge": (at("arm", q=0.0), far, 0.0, False),
        "beside the fin, 1 cm off its plane": (at("arm", dy=0.010), far, 0.0, False),
        "hanging off the fin's end": (at("arm", dx=0.036), far, 0.0, False),
        "lying on the arm's beam beside the fin (2.5 cm low)": (at("arm", dz=-0.025, dy=0.02), far, 0.0, False),
        "lifted 4 cm above the arm fin": (at("arm", dz=0.040), far, 0.0, False),
    }
    try:
        for name, (s, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            got, got_success = score(s), success(s)
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        st0 = kit.fake_states()
        assert clamped_on(st0, "bracket") and abs(jaw_deg(st0) - Q0) < 1e-6, "spawn"
        assert 0.025 < _handle_outer_y(0.0) < 0.046 and _handle_outer_y(SQUEEZE_DEG) < _handle_outer_y(on), "handles"
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

    print(f"scoring: {self_test()} cases pass (jaws {_on_fin_deg():.1f} deg on a fin; squeeze to "
          f"{2 * _handle_outer_y(SQUEEZE_DEG) * 1000:.1f} mm across the handles)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
