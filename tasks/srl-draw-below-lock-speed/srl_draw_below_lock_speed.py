"""On the station table, a fall-protection training rig. A self-retracting lifeline (SRL) hangs off an anchor beam
on the left: a yellow housing whose webbing line runs out of its outlet toward the right arm. The line ends in an
orange moulded grip with a steel snap hook on a pivot; the hook lies level, its bill hanging down. On the right
stands a training torso wearing a harness, its back to the line, with the steel dorsal D-ring standing 5 cm off the back,
19 cm beyond the hook's bill. The line is fully wound in: its spring holds the grip against the outlet.

The SRL works as real ones do. A spring in the housing winds the line in (0.8 N with it wound in, about 1.7 N at
the D-ring). A speed brake (a centrifugal pawl) locks the drum whenever the line pays out faster than its lock speed:
the line then cannot be drawn out any further, however hard it is pulled, until it is let back 10 mm, which drops
the pawl. Let go of a drawn-out line and the spring winds it in freely, slamming the hook into the housing.

The job: draw the line out to the harness, hook the D-ring and let go. Drawn steadily below the lock speed, the
line comes out to the ring; the hook's bill rides over the D-ring and drops behind it, and as the line is eased
back the bill catches the ring and the line's own pull keeps it there. Yanked, the line locks after a few
centimetres; pulling harder gets nowhere; it has to be let back first.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/srl-draw-below-lock-speed/srl_draw_below_lock_speed.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import dataclasses
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (sprung slider, hinge, per-step rules, per-episode memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the brake's events and a readout every few steps
VARIANT = "demo"  # sim smoke test: "smoke" yanks first (the brake locks), lets back, then draws slowly

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the SRL: housing on an anchor beam (fixed), the line (webbing + end grip) on a slider out along -y ----------------
LX = 0.300  # the line's axis, x
ZL = TOP + 0.100  # the line's axis, z
Y_OUT = 0.015  # the housing's outlet face
HOUSING = (0.075, 0.120, 0.130)  # x, y, z
BEAM = (0.040, 0.280, 0.040)
GRIP = (0.030, 0.060, 0.034)  # the line's moulded end grip (x, y along the line, z)
STRAP = (0.016, 0.300, 0.003)  # webbing, x wide, y long, z thick (rigid here)
LINE_KG = 0.10
K_RET = 4.0  # N/m retract spring
PRELOAD = 0.8  # N with the line wound in
REST = -PRELOAD / K_RET  # spring rest point behind the wound-in stop (preload)
MAX_OUT = 0.26  # the slider's far stop (never reached: the torso is in the way)
# -- the snap hook: on a pivot at the grip's end (axis along x), lying level on its stop, bill hanging down ----------
HINGE_Y = Y_OUT - GRIP[1]  # pivot at the grip's outer end
HOOK_KG = 0.03
ARM = (0.020, 0.072, 0.007)  # the hook's body, riding just above the pivot's height
ARM_Y = -0.003 - ARM[1] / 2
ARM_Z = 0.001 + ARM[2] / 2
TOOTH = (0.020, 0.008, 0.012)  # the bill's tooth: its foot leans back toward the pivot, so a pull makes it bite
TOOTH_AT = (0.0, -0.046, -0.002)
TOOTH_TILT = 15.0  # deg
RAMP_A = (-0.049, -0.0092)  # the bill's underside: a ramp from the tooth's foot (y, z in the hook frame) ...
RAMP_B = (-0.074, -0.001)  # ... up to the hook's nose, about 16 deg: the D-ring bar lifts the bill as it passes
RAMP_T = 0.004
HOOK_UP_DEG = 60.0
# -- the training torso and its harness's dorsal D-ring (fixed) --------------------------------------------------------
TF = -0.325  # the torso's back face (y)
TORSO = (0.160, 0.090, 0.160)
GAP = 0.050  # clear gap between the back and the D-ring bar
RING_BAR = (0.050, 0.008, 0.008)
Y_BAR = TF + GAP + RING_BAR[1] / 2  # D-ring bar centre
Z_BAR = ZL - 0.006  # its top 3 mm under the hook's body, 6 mm above the tooth's foot
LUG = (0.006, GAP + RING_BAR[1], 0.014)

YELLOW, BLACK, ORANGE, STEEL = (0.95, 0.78, 0.10), (0.10, 0.10, 0.11), (0.92, 0.45, 0.10), (0.70, 0.72, 0.75)
WEB, NAVY, GREY, HARNESS = (0.20, 0.20, 0.22), (0.16, 0.20, 0.32), (0.45, 0.46, 0.48), (0.85, 0.20, 0.15)

kit = mk.Kit()
_hz = HOUSING[2] / 2
kit.part("anchor", (LX, Y_OUT, ZL), [
    mk.box(HOUSING, at=(0, HOUSING[1] / 2, 0), color=YELLOW),  # SRL housing
    mk.box((0.030, 0.004, 0.016), at=(0, 0.002, 0), color=BLACK),  # outlet guide
    mk.box(BEAM, at=(0, HOUSING[1] + BEAM[1] / 2 - 0.010, 0), color=GREY),  # anchor beam
    mk.box((0.060, 0.080, ZL - _hz - TOP), at=(0, HOUSING[1] / 2, -_hz - (ZL - _hz - TOP) / 2), color=GREY),  # stand
    mk.box((0.040, 0.040, ZL - BEAM[2] / 2 - TOP), at=(0, HOUSING[1] + BEAM[1] - 0.040,
                                                         -BEAM[2] / 2 - (ZL - BEAM[2] / 2 - TOP) / 2), color=GREY),
], color=GREY)
kit.part("line", (LX, Y_OUT - GRIP[1] / 2, ZL), [
    mk.box(GRIP, color=ORANGE),
    mk.box(STRAP, at=(0, GRIP[1] / 2 + STRAP[1] / 2, -0.004), color=WEB),
], mass=LINE_KG, color=ORANGE)
kit.slider("payout", "anchor", "line", axis=(0, -1, 0), lower=0.0, upper=MAX_OUT, start=0.0, stiffness=K_RET,
           rest=REST)
_tooth_q = tuple(mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(TOOTH_TILT))))
_ramp_len = math.hypot(RAMP_A[0] - RAMP_B[0], RAMP_A[1] - RAMP_B[1])
_ramp_ang = math.atan2(RAMP_B[1] - RAMP_A[1], RAMP_A[0] - RAMP_B[0])  # rises toward the nose (-y)
kit.part("hook", (LX, HINGE_Y, ZL), [
    mk.box((0.022, 0.010, 0.012), at=(0, -0.005, 0.002)),  # eye on the pivot
    mk.box(ARM, at=(0, ARM_Y, ARM_Z)),
    mk.box(TOOTH, at=TOOTH_AT, quat=_tooth_q),
    mk.box((0.020, _ramp_len, RAMP_T),
           at=(0, (RAMP_A[0] + RAMP_B[0]) / 2 + RAMP_T / 2 * math.sin(_ramp_ang),
               (RAMP_A[1] + RAMP_B[1]) / 2 + RAMP_T / 2 * math.cos(_ramp_ang)),
           quat=tuple(mk.mat_to_quat(mk.axis_angle((1, 0, 0), -_ramp_ang)))),
], mass=HOOK_KG, color=STEEL)
# axis -x: a positive angle lifts the bill; 0 = lying level on its stop
kit.hinge("hook_pivot", "line", "hook", anchor=(LX, HINGE_Y, ZL), axis=(-1, 0, 0), lower_deg=0.0,
          upper_deg=HOOK_UP_DEG, start_deg=0.0)

_tc = TF - TORSO[1] / 2
FIXTURES = [
    Box("torso", pos=(LX, _tc, TOP + TORSO[2] / 2), size=TORSO, color=NAVY),
    Box("strap_l", pos=(LX + 0.048, TF + 0.0015, TOP + TORSO[2] / 2 + 0.004), size=(0.022, 0.003, TORSO[2] - 0.008),
        color=HARNESS),
    Box("strap_r", pos=(LX - 0.048, TF + 0.0015, TOP + TORSO[2] / 2 + 0.004), size=(0.022, 0.003, TORSO[2] - 0.008),
        color=HARNESS),
    Box("ring_pad", pos=(LX, TF + 0.0015, Z_BAR), size=(0.074, 0.003, 0.030), color=HARNESS),
    Box("ring_bar", pos=(LX, Y_BAR, Z_BAR), size=RING_BAR, color=STEEL),
    Box("ring_lug_l", pos=(LX + RING_BAR[0] / 2 + LUG[0] / 2, TF + LUG[1] / 2, Z_BAR), size=LUG, color=STEEL),
    Box("ring_lug_r", pos=(LX - RING_BAR[0] / 2 - LUG[0] / 2, TF + LUG[1] / 2, Z_BAR), size=LUG, color=STEEL),
]
OBJECTS = [*kit.objects(), *FIXTURES]
PARTS = ("line", "hook")

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the brake (a per-step rule, as the pawl), and the scoring ----------------------------------------------------------
DT = 0.05  # control step
V_LOCK = 0.15  # m/s payout speed that locks the brake (scaled from about 1.4 m/s on real SRLs)
UNLOCK = 0.010  # let the line back this far from where it locked and the pawl drops
K_LOCK, D_LOCK = 5000.0, 25.0  # the locked drum: N/m and N*s/m, one way (it follows the line in, never out)
V_FLY = 0.30  # m/s winding in faster than this (averaged over a control step) = let go: flew back
FLY_MIN = 0.020  # ... while more than this much line was out
DRAWN = 0.10  # 0.25 rung: line drawn out at least this far
SEAT_TOL = 0.003  # hooked: the bill within 3 mm of the D-ring bar
BITE = 0.003  # hooked: the tooth's foot at least 3 mm below the bar's top
HOOK_MAX_DEG = 20.0
CLEAR = 0.025
M = "srl"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs

_LINE_Y0 = Y_OUT - GRIP[1] / 2  # the line's origin (grip centre) wound in


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"init": False, "n": 0, "prev": None, "locked": False, "q_lock": None, "brake": None,
                            "locks": 0, "flew": False, "vmax": 0.0, "vmin": 0.0}
    return m


def payout(objects) -> float:
    """How far the line is drawn out (m): the grip's travel along -y from wound in, from its world pose."""
    return float(_LINE_Y0 - kit._pose_T(objects, "line")[1, 3])


def hook_deg(objects) -> float:
    """The hook's lift (deg; 0 = lying level on its stop)."""
    return kit.q_deg(objects, "hook_pivot")


_BILL_BOX = kit.parts["hook"].boxes[2]  # the tooth
_BILL_T = mk._T(mk.quat_to_mat(_BILL_BOX.quat), _BILL_BOX.at)  # bill box in the hook frame
_BILL_HALF = np.asarray(_BILL_BOX.size) / 2
_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * _BILL_HALF


def _bill_T(objects) -> np.ndarray:
    return kit._pose_T(objects, "hook") @ _BILL_T


def bill_gap(objects) -> float:
    """Clear distance (m) from the D-ring bar's surface to the bill (negative: overlapping)."""
    t = _bill_T(objects)
    p = mk._inv(t) @ np.array([LX, Y_BAR, Z_BAR, 1.0])
    d = np.maximum(np.abs(p[:3]) - _BILL_HALF, 0.0)
    return float(np.linalg.norm(d[1:]) - RING_BAR[1] / 2)  # in the bill's y-z plane (the bar runs along x)


def bill_low(objects) -> float:
    """The bill's lowest point (world z)."""
    t = _bill_T(objects)
    return float(np.min(_CORNERS @ t[:3, :3].T[:, 2] + t[2, 3]))


def bill_centre(objects) -> np.ndarray:
    return _bill_T(objects)[:3, 3].copy()


def past_ring(objects) -> bool:
    """The bill has been drawn past the D-ring: its centre beyond the bar's far face."""
    return bool(bill_centre(objects)[1] < Y_BAR - RING_BAR[1] / 2)


def hooked(objects) -> bool:
    """The hook is on the D-ring: the bill behind the bar, its foot 4 mm below the bar's top, touching the bar
    (within 3 mm), and the hook lying within 20 deg of level."""
    return bool(past_ring(objects) and bill_low(objects) <= Z_BAR + RING_BAR[2] / 2 - BITE
                and bill_gap(objects) <= SEAT_TOL and hook_deg(objects) <= HOOK_MAX_DEG)


def _set_drive(target=None, stiffness=None, damping=None) -> None:
    """Sim only: the line's drive (the retract spring, or the locked drum): target (m), stiffness, damping."""
    path = mk._SIM["joint_paths"].get((kit.id, "payout"))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear")
    if target is not None:
        d.GetTargetPositionAttr().Set(float(target))
    if stiffness is not None:
        d.GetStiffnessAttr().Set(float(stiffness))
    if damping is not None:
        d.GetDampingAttr().Set(float(damping))


def _brake_at(s) -> None:
    """Locked: the drum holds the line at ``s`` (stiff), the retract spring still pulling it in."""
    _set_drive(target=(K_LOCK * s + K_RET * REST) / (K_LOCK + K_RET))


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit srl] step {mk._EP.get('step')}: {msg}", flush=True)


def _rules(objects) -> None:
    """One control step of the SRL: the speed brake (lock, follow the line in, unlock) and the fly-back watch."""
    m = _mem()
    m["n"] += 1
    sim = mk._EP.get("env") is not None
    if not m["init"]:  # a new episode: the line free on its spring
        m["init"] = True
        _set_drive(target=REST, stiffness=K_RET, damping=0.0)
    q = payout(objects)
    v = 0.0 if m["prev"] is None else (q - m["prev"]) / DT
    m["vmax"], m["vmin"] = max(m["vmax"], v), min(m["vmin"], v)
    if m["prev"] is not None and v < -V_FLY and m["prev"] > FLY_MIN:
        if not m["flew"]:
            _say(f"line FLEW BACK: winding in at {-v:.2f} m/s from {m['prev'] * 1000:.0f} mm out")
        m["flew"] = True
    if not m["locked"] and v > V_LOCK:
        m["locked"], m["q_lock"], m["brake"] = True, q, q
        m["locks"] += 1
        _set_drive(stiffness=K_LOCK + K_RET, damping=D_LOCK)
        _brake_at(q)
        _say(f"brake LOCKED at {q * 1000:.1f} mm out, paying out at {v:.2f} m/s (lock {m['locks']})")
    elif m["locked"]:
        if q <= m["q_lock"] - UNLOCK:
            m["locked"] = False
            _set_drive(target=REST, stiffness=K_RET, damping=0.0)
            _say(f"brake released at {q * 1000:.1f} mm (locked at {m['q_lock'] * 1000:.1f})")
        elif q < m["brake"]:  # the drum follows the line in (the pawl only stops payout)
            m["brake"] = q
            _brake_at(q)
    m["prev"] = q
    if DEBUG and sim and (m["n"] % 4 == 0 or abs(v) > 0.08):
        tc = mk.tcps()
        tz = f"({tc[1][0]:.3f},{tc[1][1]:.3f},{tc[1][2] - TOP:.3f})" if tc else "-"
        b = bill_centre(objects)
        _say(f"R tcp {tz}; out {q * 1000:.1f} mm v {v:+.3f} locked {m['locked']} hook {hook_deg(objects):.1f} deg "
             f"bill ({b[1]:.4f},{b[2] - TOP:.4f}) low {bill_low(objects) - TOP:.4f} gap {bill_gap(objects) * 1000:.1f}"
             f" mm past {past_ring(objects)} hooked {hooked(objects)} flew {m['flew']}")


kit.on_step(_rules)


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _released(st, margin=CLEAR) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def score(objects) -> float:
    """0 if the line ever flew back (wound in faster than 0.30 m/s with more than 2 cm out: it was let go). Otherwise,
    at the end: 0.25 with the line drawn out 10 cm; 0.5 with the hook's bill past the D-ring; 0.75 with the hook on
    the D-ring; 1.0 with that, both grippers 2.5 cm clear of the line and hook, and both still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _rules(objects)
    m = _mem()
    if m["flew"]:
        return 0.0
    s = 0.0
    if payout(objects) >= DRAWN:
        s = 0.25
    if past_ring(objects):
        s = 0.5
    if hooked(objects):
        s = 0.75
        if all(_released(objects[p]) and objects[p].resting() for p in PARTS):
            s = 1.0
    return s


def success(objects) -> bool:
    """The hook on the D-ring, the line never let fly, let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Stages still to do plus the bill's distance from its seat behind the D-ring (m)."""
    b = bill_centre(objects)
    seat = np.array([LX, Y_BAR - RING_BAR[1] / 2 - 0.009, Z_BAR])
    return float((not past_ring(objects)) + (not hooked(objects)) + np.linalg.norm(b - seat))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry; right arm only, the left stays parked. Turned 90 deg (fingers closing along
# x), the arm grips the line's end grip by its sides, draws it straight out along -y in one slow 6 s move (peak
# about 0.06 m/s, well under the lock speed) to a little past where the bill meets the torso's back (the arm lags
# its target by the spring's pull over its ~90 N/m compliance), waits for the bill to drop behind the D-ring, eases
# 6 cm back so the bill catches the ring and the arm leans on it, lets go and backs away. It never reads the brake.

TIP = 0.013  # pad tips below the TCP
SQUEEZE = 0.008
GRIP_F = -(GRIP[0] - SQUEEZE) / 2
GRIP_DY = -0.006  # grip a little toward the grip's outer end (clear of the housing)
GRIP_Z = ZL + GRIP[2] / 2 - 0.005 + TIP - 0.016  # pad tips 21 mm below the grip's top
OVER = 0.034  # commanded draw past where the nose meets the torso: the arm's lag (~18 mm) and ~1 N of lean
EASE = 0.060  # then eased back this far (the bill catches the ring ~10 mm in)
PARK = (0.180, -0.150, TOP + 0.200)
YAW90 = math.pi / 2


def demo():
    """Grip the line's end grip, draw it out slowly past the D-ring, ease it back onto the ring, let go."""
    c, o = FINGER_CLOSED, FINGER_OPEN
    R = [Waypoint(2.0, PARK, o, "start", YAW90)]

    def w(dur, pos, f, label):
        R.append(Waypoint(dur, tuple(float(v) for v in pos), f, label, YAW90))

    y0 = Y_OUT - GRIP[1] / 2 + GRIP_DY
    DRAW = q_front() + OVER  # from the scene's geometry
    w(2.0, (LX, y0, GRIP_Z + 0.045), o, "over the line's end grip, fingers open")
    w(1.5, (LX, y0, GRIP_Z), o, "down around its sides")
    w(0.5, (LX, y0, GRIP_Z), o, "settle")
    w(1.0, (LX, y0, GRIP_Z), GRIP_F, "grip it")
    w(0.5, (LX, y0, GRIP_Z), GRIP_F, "settle closed")
    if VARIANT == "smoke":  # yank: the brake locks; pull on; let it back; then draw slowly
        w(0.3, (LX, y0 - 0.060, GRIP_Z), GRIP_F, "yank")
        w(1.0, (LX, y0 - 0.080, GRIP_Z), GRIP_F, "pull on against the brake")
        w(2.0, (LX, y0 + 0.005, GRIP_Z), GRIP_F, "let it back")
        w(0.5, (LX, y0 + 0.005, GRIP_Z), GRIP_F, "hold")
        w(5.0, (LX, y0 - DRAW, GRIP_Z), GRIP_F, "draw it out slowly to the harness")
    else:
        w(6.0, (LX, y0 - DRAW, GRIP_Z), GRIP_F, "draw it out slowly to the harness")
    w(1.0, (LX, y0 - DRAW, GRIP_Z), GRIP_F, "hold: the bill drops behind the D-ring")
    w(2.5, (LX, y0 - DRAW + EASE, GRIP_Z), GRIP_F, "ease it back onto the ring")
    w(0.5, (LX, y0 - DRAW + EASE, GRIP_Z), GRIP_F, "hold")
    w(1.0, (LX, y0 - DRAW + EASE, GRIP_Z), o, "let go")
    w(1.5, (LX, y0 - DRAW + EASE, GRIP_Z + 0.080), o, "back up")
    w(1.5, PARK, o, "park")
    w(2.0, PARK, o, "watch")
    total = sum(x.duration for x in R)
    left = [Waypoint(2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park"),
            Waypoint(total - 2.0, (0.150, 0.250, 0.950), FINGER_OPEN, "park")]
    return {"left": left, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def q_seat() -> float:
    """Payout at which the level hook's bill first touches the D-ring bar from behind (the catch)."""
    lo, hi = 0.15, q_front()
    for _ in range(40):
        mid = (lo + hi) / 2
        st = kit.fake_states(q={"payout": mid, "hook_pivot": 0.0})
        lo, hi = (mid, hi) if bill_gap(st) <= 0.0 or not past_ring(st) else (lo, mid)
    return hi


def q_front() -> float:
    """Payout at which the level hook's nose reaches the harness on the torso's back."""
    st = kit.fake_states(q={"payout": 0.0, "hook_pivot": 0.0})
    th = kit._pose_T(st, "hook")
    front = 1.0
    for b in kit.parts["hook"].boxes:
        t = th @ mk._T(mk.quat_to_mat(b.quat), b.at)
        c = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.asarray(b.size) / 2
        front = min(front, float(np.min(c @ t[:3, :3].T[:, 1] + t[1, 3])))
    return front - (TF + 0.003)  # the harness webbing on the back is 3 mm thick


def lift_to_clear(q) -> float:
    """Hook lift (deg) at which the bill's foot clears the bar's top, at payout ``q``."""
    a = 0.0
    while a < HOOK_UP_DEG:
        st = kit.fake_states(q={"payout": q, "hook_pivot": math.radians(a)})
        if bill_low(st) > Z_BAR + RING_BAR[2] / 2:
            return a
        a += 0.25
    return a


def self_test():
    """The scoring rules on hand-built sequences of poses (one trial each, fed step by step at 20 Hz)."""
    global _TEST_TCPS
    qs, qf = q_seat(), q_front()
    assert qs + 0.008 < qf, (qs, qf)
    a_clear = lift_to_clear(qs - 0.02)
    assert 5.0 < a_clear < 30.0, a_clear

    def fr(q, a=0.0, vel=0.0):
        st = kit.fake_states(q={"payout": q, "hook_pivot": math.radians(a)})
        if vel:
            st["line"] = dataclasses.replace(st["line"], lin_vel=np.array([0.0, -vel, 0.0]))
        return st

    def run(q0, q1, speed, a=0.0):  # frames from q0 to q1 at ``speed`` m/s (20 Hz)
        n = max(1, int(math.ceil(abs(q1 - q0) / (speed * DT))))
        return [fr(q0 + (q1 - q0) * k / n, a) for k in range(1, n + 1)]

    ride = [fr(q, lift_to_clear(q) + 1.0) for q in np.linspace(qs - 0.03, qs + 0.002, 6)]
    slow = [fr(0.0)] + run(0.0, qs - 0.03, 0.06) + ride + run(qs + 0.002, qf, 0.04)
    seat = run(qf, qs, 0.02)
    good = slow + seat + [fr(qs)] * 3
    yank = [fr(0.0)] + run(0.0, 0.04, 0.4) + [fr(0.04)] * 10  # locks at 2 cm; pulled on, it creeps to 4 cm (sim)
    let_back = run(0.04, 0.008, 0.02)
    hand = (LX, 0.0, ZL)  # a TCP on the grip, somewhere along the line
    cases = {  # name: (frames, TCPs at the end, score)
        "start": ([fr(0.0)], [], 0.0),
        "drawn slowly past the ring, eased back onto it, let go (demo)": (good, [], 1.0),
        "the same, still holding the grip": (good, [(LX, Y_OUT - qs - 0.03, ZL)], 0.75),
        "the same but the line still swinging": (good[:-1] + [fr(qs, vel=0.05)], [], 0.75),
        "yanked: the brake locks at 2 cm, held there": (yank, [hand], 0.0),
        "yanked, then let go: the line flies back": (yank + run(0.04, 0.0, 0.6), [], 0.0),
        "yanked, let back, drawn slowly, hooked, let go": (yank + let_back + run(0.008, qs - 0.03, 0.06) + ride
                                                           + run(qs + 0.002, qf, 0.04) + seat + [fr(qs)] * 3, [], 1.0),
        "drawn 15 cm slowly and held": ([fr(0.0)] + run(0.0, 0.15, 0.06), [hand], 0.25),
        "drawn 15 cm and let go: the line flies back": ([fr(0.0)] + run(0.0, 0.15, 0.06) + run(0.15, 0.0, 0.8),
                                                       [], 0.0),
        "fly back first, then a perfect hook-up (latched)": ([fr(0.0)] + run(0.0, 0.15, 0.06)
                                                            + run(0.15, 0.0, 0.8) + good, [], 0.0),
        "drawn past the ring, bill against the torso, still held": (slow, [hand], 0.5),
        "drawn past and let go: the bill snaps 1 cm onto the ring (not a fly-back)": (slow + [fr(qs)] * 4, [], 1.0),
        "eased back slowly by hand: no fly-back": ([fr(0.0)] + run(0.0, 0.15, 0.06) + run(0.15, 0.02, 0.2),
                                                   [hand], 0.0),
        "past the ring but the hook held up 30 deg (not on it)": (slow + [fr(qs, 30.0)], [hand], 0.5),
        "bill riding on top of the bar, short of past it": (slow[:-12] + [fr(qs - 0.02, lift_to_clear(qs - 0.02) + 1)],
                                                            [hand], 0.25),
        "drawn to 9 cm only and held": ([fr(0.0)] + run(0.0, 0.09, 0.05), [hand], 0.0),
    }
    try:
        for name, (frames, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for f in frames[:-1]:
                score(f)
            _TEST_TCPS = tcps
            got = score(frames[-1])
            assert abs(got - want) < 1e-9, (name, got, want, {k: v for k, v in _mem().items() if k != "prev"})
            assert success(frames[-1]) == (want >= 1.0), name
        # the brake: a yank locks it; 5 mm let back keeps it locked; 10 mm releases it; slow payout never locks
        mk.new_episode()
        for f in yank:
            score(f)
        m = _mem()
        assert m["locked"] and m["locks"] == 1 and abs(m["q_lock"] - 0.02) < 1e-9, m
        for f in run(0.04, 0.015, 0.02):
            score(f)
        assert _mem()["locked"], "still locked 5 mm back"
        for f in run(0.015, m["q_lock"] - UNLOCK - 0.001, 0.02):
            score(f)
        assert not _mem()["locked"], "released 10 mm back"
        mk.new_episode()
        for f in good:
            score(f)
        assert _mem()["locks"] == 0 and not _mem()["flew"], "the demo's pace never locks"
        mk.new_episode()
        assert score(fr(0.0)) == 0.0 and not _mem()["flew"], "a new trial forgets the last one"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 4, qs, qf, a_clear


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n, qs, qf, a_clear = self_test()
    print(f"scoring: {n} cases pass (the bill catches the D-ring at {qs * 1000:.1f} mm out, meets the torso at "
          f"{qf * 1000:.1f} mm; it clears the bar lifted {a_clear:.1f} deg)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
