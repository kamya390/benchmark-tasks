"""On the station table, between the two arms, a vaccine vial lies on its side in a bench vial holder, rubber stopper
facing right. An auto-disable (AD) syringe is already on it: the syringe lies level along the table's y axis with
its needle pushed through the vial's stopper up to the hub, held only by the needle's grip in the rubber. The syringe is an
enlarged training model (so the robot's finger pads fit its barrel and plunger): a square 36 mm barrel 160 mm long
with a viewing slot along its top and graduations every 0.1 ml beside the slot (the 0.5 ml line is red), the black
rubber plunger seal visible through the slot, and the plunger's 24 mm square grip block and 50 mm thumb plate to the
right.

The job: draw one 0.5 ml dose into the syringe and leave the syringe in the vial.

What makes it hard is the auto-disable clip, the feature that keeps these syringes from being used twice. The plunger
draws back freely (1.0 N of seal friction), but the moment it is pushed forward again -- even 1.2 mm, i.e. 0.02 ml --
the clip engages and the plunger is locked for good: the dose cannot be topped up and an overdraw cannot be pushed
back into the vial. The clip also engages at the end of travel (0.6 ml). So the dose has to be drawn in one smooth,
monotonic pull that stops inside the 0.5 ml band (+-0.05 ml), with no correction. And the barrel must be held: the
needle grips the stopper with only 0.4 N, less than the seal's 1.0 N, so pulling the plunger alone pulls the whole
syringe out of the vial (past 20 mm the needle is out and the syringe comes away).

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory. The barrel
rides on the vial on a prismatic joint (the needle in the stopper) with Coulomb friction; the plunger rides in the
barrel on a second prismatic joint with Coulomb friction, light damping and a hard stop at 36 mm. The clip is a
per-step rule (documented in task.json fidelity): once the plunger is 1.2 mm short of its furthest draw, or at the
stop, its joint is locked (a 40 kN/m hold at the current position) for the episode. A needle pulled out of the
stopper frees the barrel (its joint is switched off).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/auto-disable-syringe-one-pull/auto_disable_syringe_one_pull.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (prismatic joints, friction, per-step hook, per-episode memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print draw, seat, clip and TCPs every 10 steps
MODE = "demo"  # demo | nohold (pull the plunger without holding the barrel) | jerk (pull, push back, pull) | over

# Frame: x forward from the arm bases, y left, z up, metres. Right arm base (0, -0.305, 0.75), left (0, +0.305).
XS, ZS = 0.300, TOP + 0.055  # the syringe's axis (along y)
# -- the syringe (3x training model) ------------------------------------------------------------------------------
WB, WALL, SLOT = 0.036, 0.004, 0.010  # barrel outer width (square), wall, viewing slot in its top
YBF, LB = 0.063, 0.160  # barrel front (hub end) and length
YBR = YBF - LB  # barrel rear (finger flange)
NEEDLE_L, NEEDLE_W = 0.020, 0.0025
YV = YBF + 0.010  # the vial stopper's outer face: the needle is in to its hub (20 mm in the stopper)
NEEDLE_IN = YBF + 0.010 + NEEDLE_L - YV  # 0.020
BARREL_KG, PLUNGER_KG = 0.040, 0.030
Y_SEAL0 = YBF - WALL  # the seal's front face with the plunger fully in (0 ml)
ML = 0.060  # m of draw per ml (0.5 ml = 30 mm)
DOSE_ML, TOL_ML = 0.5, 0.05
D_DOSE, D_TOL = DOSE_ML * ML, TOL_ML * ML  # 30 mm +- 3 mm
D_STOP = 0.6 * ML  # 36 mm: end of travel (the clip engages there too)
HEAD_W, HEAD_L = 0.024, 0.046  # the plunger's grip block (square 24 mm, 46 mm long) behind the barrel
Y_HEAD0 = YBR - 0.010  # its front face at 0 ml (10 mm of rod shows)
Y_TF = Y_HEAD0 - HEAD_L  # the thumb plate's front face at 0 ml
ROD_W = 0.016
F_SEAT, C_SEAT = 0.4, 15.0  # N, N*s/m: the needle's grip in the stopper
F_DRAW, C_DRAW = 1.0, 15.0  # N, N*s/m: the plunger seal's friction in the barrel
BACK_LOCK = 0.0012  # m: a forward (push) movement this far from the furthest draw engages the clip
K_LOCK = 40000.0  # N/m: the engaged clip's hold
SEATED_TOL = 0.010  # m: the needle still at least 10 of its 20 mm in the stopper

BARREL_C, ROD_C, SEAL_C = (0.86, 0.89, 0.91), (0.95, 0.95, 0.93), (0.08, 0.08, 0.09)
INK, RED, STEEL = (0.10, 0.12, 0.30), (0.80, 0.10, 0.10), (0.70, 0.72, 0.75)
GLASS, ALU, RUBBER, HOLDER, LABEL = (0.70, 0.82, 0.88), (0.78, 0.80, 0.83), (0.35, 0.36, 0.38), (0.25, 0.42, 0.62), (
    0.96, 0.96, 0.94)


def _qy(a: float) -> tuple:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _ngon_y(n, r_out, length, at, color=None):
    """A regular n-gon prism along y (circumradius r_out): n/2 bars across the centre, turned about y."""
    a = r_out * math.cos(math.pi / n)
    side = 2 * a * math.tan(math.pi / n)
    return [mk.box((2 * a, length, side), at=at, quat=_qy(math.pi * 2 * k / n), color=color) for k in range(n // 2)]


kit = mk.Kit()
# Vial holder and vial (one fixed part): the vial lies along y, stopper facing -y at YV.
VIAL_R, VIAL_L = 0.026, 0.062
YVB = YV + 0.018  # vial body starts behind cap and neck
kit.part("vial", (XS, YV, ZS), [
    mk.box((0.120, 0.110, 0.006), at=(0, 0.050, TOP + 0.003 - ZS), color=HOLDER),  # holder base plate
    mk.box((0.060, 0.050, ZS - VIAL_R - TOP - 0.006), at=(0, 0.048, (TOP + 0.006 + ZS - VIAL_R) / 2 - ZS),
           color=HOLDER),  # cradle block under the vial
    *[mk.box((0.008, 0.050, ZS - 0.004 - TOP - 0.006), at=(s * (VIAL_R + 0.004), 0.048, (TOP + 0.006 - 0.004) / 2
                                                             + ZS / 2 - ZS), color=HOLDER)
      for s in (-1, 1)],  # cradle cheeks
    *_ngon_y(8, VIAL_R, VIAL_L, (0, YVB - YV + VIAL_L / 2, 0), color=GLASS),  # glass vial body
    *_ngon_y(8, VIAL_R + 0.0008, 0.030, (0, YVB - YV + 0.034, 0), color=LABEL),  # paper label band
    mk.box((0.026, 0.008, 0.026), at=(0, 0.014, 0), color=GLASS),  # neck
    mk.box((0.034, 0.010, 0.034), at=(0, 0.005, 0), color=ALU),  # aluminium crimp cap
    mk.box((0.018, 0.001, 0.018), at=(0, -0.0005, 0), color=RUBBER),  # stopper face in the cap's opening
], color=GLASS)

# The barrel (free): walls with a slot in the top, front cap, hub, needle, rear finger flange, graduations.
_yc, _zt = (YBF + YBR) / 2, WB / 2
_lip = (WB - SLOT) / 2
_marks = []
for k in range(7):  # 0 .. 0.6 ml every 0.1 ml, on both lips beside the slot
    y = Y_SEAL0 - k * 0.1 * ML
    big = k == 5
    for s in (-1, 1):
        _marks.append(mk.box((0.010 if big else 0.006, 0.0015 if big else 0.0010, 0.0006),
                             at=(s * (SLOT / 2 + (0.005 if big else 0.003)), y - YBF, _zt + 0.0003),
                             color=RED if big else INK))
kit.part("barrel", (XS, YBF, ZS), [
    mk.box((WB, LB, WALL), at=(0, _yc - YBF, -_zt + WALL / 2)),  # bottom
    *[mk.box((WALL, LB, WB - 2 * WALL), at=(s * (WB - WALL) / 2, _yc - YBF, 0)) for s in (-1, 1)],  # sides
    *[mk.box((_lip, LB, WALL), at=(s * (SLOT + _lip) / 2, _yc - YBF, _zt - WALL / 2)) for s in (-1, 1)],  # top lips
    mk.box((WB, WALL, WB), at=(0, -WALL / 2, 0)),  # front end
    mk.box((0.012, 0.010, 0.012), at=(0, 0.005, 0)),  # needle hub
    mk.box((NEEDLE_W, NEEDLE_L, NEEDLE_W), at=(0, 0.010 + NEEDLE_L / 2, 0), color=STEEL),  # needle
    mk.box((0.076, 0.005, 0.024), at=(0, -LB + 0.0025, 0)),  # finger flange
    *_marks,
], mass=BARREL_KG, color=BARREL_C)
kit.slider("seat", "vial", "barrel", axis=(0, 1, 0), lower=-0.030, upper=0.0, friction=F_SEAT, damping=C_SEAT)

# The plunger (free): rubber seal (seen through the slot), rod, grip block, thumb plate.
_rod_y0, _rod_y1 = Y_SEAL0 - 0.010, Y_HEAD0
kit.part("plunger", (XS, Y_SEAL0, ZS), [
    mk.box((WB - 2 * WALL - 0.002, 0.010, WB - 2 * WALL - 0.002), at=(0, -0.005, 0), color=SEAL_C),  # seal
    mk.box((ROD_W, _rod_y0 - _rod_y1, ROD_W), at=(0, (_rod_y0 + _rod_y1) / 2 - Y_SEAL0, 0)),  # rod
    mk.box((HEAD_W, HEAD_L, HEAD_W), at=(0, Y_HEAD0 - HEAD_L / 2 - Y_SEAL0, 0)),  # grip block
    mk.box((0.050, 0.006, 0.050), at=(0, Y_TF - 0.003 - Y_SEAL0, 0)),  # thumb plate
], mass=PLUNGER_KG, color=ROD_C)
kit.slider("draw", "barrel", "plunger", axis=(0, -1, 0), lower=0.0, upper=D_STOP, friction=F_DRAW, damping=C_DRAW)

_q_kit = kit.q


def _q(objects, name):
    """Joint value from the two bodies' poses. Sliders are read at the child's own origin in the parent's frame (the
    kit's own reading takes the translation at the world origin, which a tiny tilt of the barrel would pollute)."""
    j = kit.joints_[name]
    if j.kind != "slider":
        return _q_kit(objects, name)
    tc = kit._pose_T(objects, j.child)
    if j.parent is not None and kit.parts[j.parent].free:
        tc = kit.parts[j.parent].ref @ mk._inv(kit._pose_T(objects, j.parent)) @ tc
    return float(np.dot(tc[:3, 3] - kit.parts[j.child].ref[:3, 3], j.axis))


kit.q = _q  # this kit only (used by the readouts below and by the kit's friction model)

OBJECTS = [*kit.objects()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

M = "adsy"  # this task's key in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"dmax": 0.0, "locked": False, "lock_at": None, "why": "", "out": False, "n": 0}
        _restore()
    return m


def _tcps() -> list:
    return mk.tcps() if _TEST_TCPS is None else [np.asarray(t, float) for t in _TEST_TCPS]


def draw(objects) -> float:
    """How far the plunger has been drawn back in the barrel (m; 0 = fully in)."""
    return kit.q(objects, "draw")


def seat(objects) -> float:
    """How far the needle is pushed into the stopper relative to the start (m; negative = pulled out)."""
    return kit.q(objects, "seat")


def dose_ml(objects) -> float:
    return draw(objects) / ML


# -- the clip and the needle (sim only: they set PhysX joint attributes) ------------------------------------------


def _drive(name):
    path = mk._SIM["joint_paths"].get((kit.id, name))
    if not path or mk._SIM.get("stage") is None:
        return None
    from pxr import UsdPhysics

    return UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "linear")


def _restore() -> None:
    """New episode: the clip disengaged (the seal's friction spring back) and the needle joint on."""
    d = _drive("draw")
    if d is None:
        return
    d.GetStiffnessAttr().Set(float(F_DRAW / mk.STICK_M))
    mk._set_joint_enabled(kit, "seat", True)


def _engage(at: float) -> None:
    """The clip: hold the plunger where it is, for the rest of the episode."""
    d = _drive("draw")
    if d is None:
        return
    mk._SIM["stick"][(kit.id, "draw")] = at  # the kit's friction model then leaves the set point alone
    d.GetStiffnessAttr().Set(K_LOCK)
    d.GetTargetPositionAttr().Set(float(at))


def _track(objects) -> None:
    """Every control step: furthest draw so far, the clip (a push back of BACK_LOCK, or the end stop), and the
    needle leaving the stopper (latched; in sim the barrel's joint is then switched off)."""
    m = _mem()
    d, s = draw(objects), seat(objects)
    m["n"] += 1
    if not m["locked"]:
        m["dmax"] = max(m["dmax"], d)
        why = ("pushed back" if m["dmax"] - d > BACK_LOCK else "end stop" if d >= D_STOP - 0.0007 else "")
        if why:
            m.update(locked=True, lock_at=d, why=why)
            _engage(d)
            _say(f"CLIP ENGAGED ({why}) at {d * 1000:.1f} mm (furthest {m['dmax'] * 1000:.1f} mm)")
    if not m["out"] and s < -NEEDLE_IN:
        m["out"] = True
        if mk._SIM.get("stage") is not None:
            mk._set_joint_enabled(kit, "seat", False)
        _say(f"NEEDLE OUT of the stopper (seat {s * 1000:.1f} mm)")
    if DEBUG and mk._EP.get("env") is not None and mk._EP.get("step", 0) % 10 == 0:
        tc = " ".join(f"({t[0]:.3f},{t[1]:.3f},{t[2] - TOP:.3f})" for t in _tcps())
        b, p = objects["barrel"], objects["plunger"]
        _say(f"draw {d * 1000:.2f} mm (max {m['dmax'] * 1000:.2f}) seat {s * 1000:.2f} mm locked {m['locked']} "
             f"out {m['out']} vb {np.linalg.norm(b.lin_vel):.4f} vp {np.linalg.norm(p.lin_vel):.4f} tcps {tc}")


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit adsy] step {mk._EP.get('step')}: {msg}", flush=True)


kit.on_step(_track)


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def in_band(objects) -> bool:
    return abs(draw(objects) - D_DOSE) <= D_TOL + 1e-9


def score(objects) -> float:
    """0 once the clip has engaged (pushed back 0.02 ml, or drawn to the 0.6 ml stop) or the needle has come out of
    the stopper. Otherwise 0.25 once the plunger is drawn at least 0.05 ml; 0.5 with it inside the 0.5 ml band
    (+-0.05 ml); 1.0 with that, the needle still fully in the stopper, both grippers clear of barrel and plunger, and
    the syringe at rest."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step hook has already run
        _track(objects)
    m = _mem()
    if m["locked"] or m["out"]:
        return 0.0
    if draw(objects) < 0.05 * ML:
        return 0.0
    if not in_band(objects):
        return 0.25
    b, p = objects["barrel"], objects["plunger"]
    done = (seat(objects) >= -SEATED_TOL and _released(b) and _released(p) and b.resting() and p.resting())
    return 1.0 if done else 0.5


def success(objects) -> bool:
    """0.5 ml drawn in one pull (clip never engaged), needle still in the vial, syringe let go and still; 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Clip or needle failure (2) plus how far the plunger is from the 0.5 ml line (m)."""
    m = _mem()
    return float(2.0 * (m["locked"] or m["out"]) + abs(draw(objects) - D_DOSE))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry. Left arm: grip the barrel from above (fingers closing across it) and hold it.
# Right arm: grip the plunger's grip block (in front of the thumb plate) and draw it back along -y in one slow, steady pull
# to the red line, hold, let go, back away; then the left lets go of the barrel. Neither reads the plunger's position.

TIP = 0.013  # pad tips below the TCP
ZG = ZS + 0.004  # TCP height for both grips: the pads straddle the syringe's axis
YG_L = 0.045  # left grip on the barrel, near its front (17.5 cm from the right grip: the wrists clear)
YG_R = Y_HEAD0 - HEAD_L / 2  # right grip: mid grip block (the 40 mm wide fingertips clear barrel and thumb plate)
SQUEEZE = 0.008
GRIP_B = -(WB - SQUEEZE) / 2
GRIP_R = -(HEAD_W - SQUEEZE) / 2
LET_GO = -0.030
PULL_TRIM = 0.006  # extra commanded pull: the soft arm lags ~4-6 mm under the seal's friction (sim runs 3-4)
LEFT_PRELOAD = 0.005  # the left presses the barrel this far toward the vial during the pull (its arm gives ~5 mm)
PULL_S = 4.0
PARK_L, PARK_R = (0.150, 0.250, TOP + 0.200), (0.200, -0.300, TOP + 0.170)
YAW_L, YAW_R = -math.pi / 2, math.pi / 2  # fingers close along x, across the syringe


def demo():
    """Hold the barrel with the left hand; draw the plunger to the 0.5 ml line with the right in one steady pull
    (the left presses the barrel toward the vial as the pull builds); let go with both hands and back away."""
    c, o = FINGER_CLOSED, FINGER_OPEN
    L = [Waypoint(2.0, PARK_L, o, "start", 0.0)]
    R = [Waypoint(2.0, PARK_R, c, "start", 0.0)]

    def w(arm, dur, pos, f, label, yaw=None):
        yaw = (YAW_L if arm is L else YAW_R) if yaw is None else yaw
        arm.append(Waypoint(dur, tuple(float(v) for v in pos), f, label, yaw))

    hold_l = MODE != "nohold"
    if hold_l:
        w(L, 2.0, (XS, YG_L, ZG + 0.060), o, "over the barrel, fingers open")
        w(L, 1.5, (XS, YG_L, ZG), o, "down around it")
        w(L, 0.5, (XS, YG_L, ZG), o, "settle")
        w(L, 1.0, (XS, YG_L, ZG), GRIP_B, "grip the barrel")
    else:
        w(L, 5.0, PARK_L, o, "stay parked", 0.0)
    w(R, 2.0, (XS, YG_R, ZG + 0.060), o, "over the grip block, fingers open")
    w(R, 1.5, (XS, YG_R, ZG), o, "down around it")
    w(R, 1.5, (XS, YG_R, ZG), o, "settle (left has the barrel)")
    w(R, 1.0, (XS, YG_R, ZG), GRIP_R, "grip the plunger")
    w(R, 0.5, (XS, YG_R, ZG), GRIP_R, "settle closed")
    t_pull = sum(x.duration for x in R)
    y_end = YG_R - D_DOSE - PULL_TRIM
    if MODE == "jerk":  # pull partway, push back 15 mm (the soft arm absorbs ~8 mm), pull on to the line
        w(R, 2.0, (XS, YG_R - 0.018, ZG), GRIP_R, "pull partway")
        w(R, 1.0, (XS, YG_R - 0.003, ZG), GRIP_R, "push back")
        w(R, 2.5, (XS, y_end, ZG), GRIP_R, "pull on to the line")
    elif MODE == "over":
        w(R, PULL_S, (XS, YG_R - D_STOP - 0.008, ZG), GRIP_R, "pull past the line")
    else:
        w(R, PULL_S, (XS, y_end, ZG), GRIP_R, "draw the plunger to the 0.5 ml line in one pull")
    t_pulled = sum(x.duration for x in R)
    w(R, 1.0, R[-1].pos, GRIP_R, "hold")
    w(R, 0.8, R[-1].pos, LET_GO, "let go")
    w(R, 1.2, (XS, R[-1].pos[1], ZG + 0.070), LET_GO, "back up")
    t_clear = sum(x.duration for x in R)
    w(R, 1.5, PARK_R, o, "park", 0.0)
    if hold_l:
        y_hold = YG_L + LEFT_PRELOAD
        w(L, t_pull - sum(x.duration for x in L), (XS, YG_L, ZG), GRIP_B, "hold the barrel")
        w(L, t_pulled - t_pull, (XS, y_hold, ZG), GRIP_B, "keep it pressed into the vial as the pull builds")
        w(L, t_clear - t_pulled, (XS, y_hold, ZG), GRIP_B, "hold it there while the right lets go")
        w(L, 0.8, (XS, y_hold, ZG), LET_GO, "let go")
        w(L, 1.2, (XS, y_hold, ZG + 0.070), LET_GO, "back up")
        w(L, 1.5, PARK_L, o, "park", 0.0)
    else:
        w(L, sum(x.duration for x in R) - sum(x.duration for x in L), PARK_L, o, "stay parked", 0.0)
    w(R, sum(x.duration for x in L) - sum(x.duration for x in R) + 0.5, PARK_R, o, "wait", 0.0)
    w(L, 0.5, PARK_L, o, "wait", 0.0)
    return {"left": L, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built trials (fed step by step, with both TCPs every step)."""
    global _TEST_TCPS
    park = [PARK_L, PARK_R]
    hands_on = [(XS, YG_L, ZG), (XS, YG_R - D_DOSE, ZG)]

    def frame(d, s=0.0, tcps=park, moving=False):
        st = kit.fake_states(q={"draw": d, "seat": s})
        if moving:
            p = st["plunger"]
            st["plunger"] = ObjectState(pos=p.pos, quat_xyzw=p.quat_xyzw, lin_vel=np.array([0.0, -0.02, 0.0]),
                                        ang_vel=p.ang_vel, low=p.low, high=p.high)
        return st, tcps

    def ramp(a, b, n=10, **kw):
        return [frame(a + (b - a) * k / n, **kw) for k in range(1, n + 1)]

    pull = ramp(0.0, D_DOSE, 15, tcps=hands_on)
    done = [frame(D_DOSE, tcps=park)] * 3
    cases = {  # name: (frames, score)
        "start": ([frame(0.0)], 0.0),
        "one steady pull to the 0.5 ml line, both hands off (demo)": (pull + done, 1.0),
        "the same, 0.47 ml (inside the band)": (ramp(0.0, D_DOSE - 0.0018, 15, tcps=hands_on) + done[:0]
                                                 + [frame(D_DOSE - 0.0018)] * 3, 1.0),
        "the same but still gripping": (pull, 0.5),
        "in the band but the plunger still moving": (pull + [frame(D_DOSE, moving=True)], 0.5),
        "short: stopped at 0.3 ml": (ramp(0.0, 0.3 * ML, 10, tcps=hands_on) + [frame(0.3 * ML)] * 3, 0.25),
        "over: 0.57 ml, not at the stop (can't be pushed back)": (ramp(0.0, 0.57 * ML) + [frame(0.57 * ML)] * 3,
                                                                  0.25),
        "overdrawn to the 0.6 ml stop: clip engaged": (ramp(0.0, D_STOP) + [frame(D_STOP)] * 3, 0.0),
        "overdrawn then pushed back into the band: clip engaged": (ramp(0.0, 0.58 * ML) + ramp(0.58 * ML, D_DOSE, 3),
                                                                  0.0),
        "jerky: 0.25 ml, eased back 2 mm, pulled on to the line": (ramp(0.0, 0.015) + ramp(0.015, 0.013, 2)
                                                                   + ramp(0.013, D_DOSE) + done, 0.0),
        "0.5 mm spring-back after letting go (inside the clip's play)": (pull + [frame(D_DOSE - 0.0005)] * 3, 1.0),
        "drawn to the line, then a 1.5 mm push while letting go": (pull + [frame(D_DOSE - 0.0015)] * 3, 0.0),
        "barrel not held: the syringe pulled out of the stopper": (
            [frame(0.0, s=-0.001 * k) for k in range(1, 26)] + [frame(0.0, s=-0.025)] * 3, 0.0),
        "needle pulled out, pushed back in, then drawn to the line": (
            [frame(0.0, s=-0.001 * k) for k in range(1, 23)] + [frame(0.0, s=0.0)] + pull + done, 0.0),
        "syringe pulled 12 mm out of the stopper but drawn to the line": (
            [frame(0.0, s=-0.012)] + ramp(0.0, D_DOSE, 15, s=-0.012) + [frame(D_DOSE, s=-0.012)] * 3, 0.5),
        "syringe pulled 6 mm out (needle still 14 mm in), drawn to the line": (
            [frame(0.0, s=-0.006)] + ramp(0.0, D_DOSE, 15, s=-0.006) + [frame(D_DOSE, s=-0.006)] * 3, 1.0),
        "syringe pulled 3 mm out during the pull, pressed back to the hub": (
            ramp(0.0, D_DOSE, 15, s=-0.003) + [frame(D_DOSE, s=0.0)] * 3, 1.0),
    }
    try:
        for name, (frames, want) in cases.items():
            mk.new_episode()
            got = None
            for s, tc in frames:
                _TEST_TCPS = tc
                got = score(s)
            assert abs(got - want) < 1e-9, (name, got, want, _mem())
            assert success(frames[-1][0]) == (want >= 1.0), name
        mk.new_episode()
        _TEST_TCPS = park
        for s, _ in ramp(0.0, D_STOP):
            score(s)
        assert _mem()["locked"] and _mem()["why"] == "end stop"
        mk.new_episode()
        assert score(frame(D_DOSE)[0]) == 1.0 and not _mem()["locked"], "a new trial forgets the clip"
        assert abs(draw(kit.fake_states(q={"draw": 0.012, "seat": -0.01})) - 0.012) < 1e-9
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 3


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built trials, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n = self_test()
    print(f"scoring: {n} cases pass (dose band {(D_DOSE - D_TOL) * 1000:.0f}-{(D_DOSE + D_TOL) * 1000:.0f} mm of draw; "
          f"clip at {BACK_LOCK * 1000:.1f} mm back or the {D_STOP * 1000:.0f} mm stop)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
