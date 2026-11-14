"""On the station table lies a freshly assembled cabinet face frame, face up, out of square. It is the face frame of
an 18 in wide x 12 in tall stock wall cabinet: 457 mm wide (along y) by 305 mm tall (along x), 19 x 38 mm poplar.
The two stiles run the full 305 mm; the two rails (381 mm between the stiles) join them with one 5 x 30 mm loose
tenon per joint. Each tenon is fixed in its rail end and sits in a stile mortise cut 6 mm longer than the tenon is
wide, so each joint can turn a few degrees in the plane of the frame, with drag. The frame is racked 2.7 deg: its far
rail sits 13 mm to the left of its near rail and its long diagonal (near-right corner to far-left corner) is 20 mm
longer than its short one. Tape marks on the table show where its four outer corners started.

The job: make the frame square (its two corner-to-corner diagonals equal), without sliding it off its marks or lifting
it, then let go. The frame has no stop at square: it racks straight through square the other way, and it stays where
it is left. Pushing one corner (or dragging one rail) alone slides the whole frame instead of racking it.

Mechanism parts come from mechkit.py (in this folder); the fourth joint, which closes the frame's loop, is authored
here as a plain revolute joint (mechkit builds trees only).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/square-racked-face-frame/square_racked_face_frame.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinges, friction and per-episode hooks, installed in memory)

import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
M = "sf"  # log prefix / MEMORY key
DEBUG = bool(os.environ.get("SF_DEBUG"))  # per-step trace in the log

# -- the frame (full size) -------------------------------------------------------------------------------------------
# x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
W, H = 0.457, 0.305  # outer width (rails' direction, y) and height (stiles' direction, x): an 18 x 12 in wall cabinet
S, TH = 0.038, 0.019  # 1 1/2 in x 3/4 in face-frame stock
RHO = 470.0  # kg/m3: yellow-poplar, specific gravity 0.42 at 12 % moisture (Wood Handbook) x 1.12
XC, YC = 0.27, 0.0  # frame centre
RACK_DEG = 2.71  # start rack: long diagonal 20.0 mm longer than the short one
LIMIT_DEG = 8.0  # sim joint stop, far beyond any rack the task needs
# Joint drag. The four joints all turn by the same angle when the frame racks; the loop-closing joint is frictionless,
# so the three kit hinges carry the drag of four. With the frame on the table (mu 0.35) this puts the push-gauge
# reading of the setup's reset check (near rail held, push the far-left stile 30 mm from its end along -y) at ~2 N.
TAU = 0.139  # N*m per kit hinge
mk.STICK_RAD = math.radians(0.05)  # mechkit friction 'stick' travel: 0.05 deg (0.4 mm of diagonal) of elastic give
MU_FRAME = 0.2  # frame material; PhysX averages it with the table's default 0.5 -> 0.35 wood on bench, ~0.7 vs pads
POPLAR = (0.86, 0.78, 0.58)
END_GRAIN = (0.74, 0.64, 0.45)
M_RAIL = (W - 2 * S) * S * TH * RHO  # 0.129 kg
M_STILE = H * S * TH * RHO  # 0.103 kg
Z = TOP + TH / 2
XN, XF, YS = XC - H / 2 + S / 2, XC + H / 2 - S / 2, W / 2 - S  # rail centre lines; stile inner edges (shoulders)
PIVOT = {"nl": (XN, YS), "nr": (XN, -YS), "fl": (XF, YS), "fr": (XF, -YS)}  # each joint turns about its shoulder

kit = mk.Kit()
kit.part("rail_near", (XN, YC, Z), [mk.box((S, W - 2 * S, TH))], mass=M_RAIL, color=POPLAR)
kit.part("rail_far", (XF, YC, Z), [mk.box((S, W - 2 * S, TH))], mass=M_RAIL, color=POPLAR)
for _n, _sg in (("stile_l", 1), ("stile_r", -1)):
    kit.part(_n, (XC, _sg * (W / 2 - S / 2), Z),
             [mk.box((H, S, TH)), mk.box((0.002, S, TH + 0.0004), at=(H / 2 - 0.001, 0, 0), color=END_GRAIN),
              mk.box((0.002, S, TH + 0.0004), at=(-H / 2 + 0.001, 0, 0), color=END_GRAIN)],
             mass=M_STILE, color=POPLAR)
kit.hinge("j_nl", "rail_near", "stile_l", (*PIVOT["nl"], Z), (0, 0, 1), -LIMIT_DEG, LIMIT_DEG, start_deg=RACK_DEG,
          friction=TAU)
kit.hinge("j_nr", "rail_near", "stile_r", (*PIVOT["nr"], Z), (0, 0, 1), -LIMIT_DEG, LIMIT_DEG, start_deg=RACK_DEG,
          friction=TAU)
kit.hinge("j_fl", "stile_l", "rail_far", (*PIVOT["fl"], Z), (0, 0, 1), -LIMIT_DEG, LIMIT_DEG, start_deg=-RACK_DEG,
          friction=TAU)
MEMBERS = ("rail_near", "rail_far", "stile_l", "stile_r")

# The outer corners (the stiles' ends, outside face): where the diagonals are measured with a tape
CORNER_LOCAL = {"nl": ("stile_l", (-H / 2, S / 2, 0.0)), "fl": ("stile_l", (H / 2, S / 2, 0.0)),
                "nr": ("stile_r", (-H / 2, -S / 2, 0.0)), "fr": ("stile_r", (H / 2, -S / 2, 0.0))}


def corners(objects) -> dict:
    return {k: kit.point(objects, p, loc) for k, (p, loc) in CORNER_LOCAL.items()}


def diagonals(objects) -> tuple:
    """(near-left to far-right, near-right to far-left) corner-to-corner lengths in the table plane, m."""
    c = corners(objects)
    return (float(np.linalg.norm((c["nl"] - c["fr"])[:2])), float(np.linalg.norm((c["nr"] - c["fl"])[:2])))


def diag_diff_mm(objects) -> float:
    """Signed: + while racked as at the start (near-right to far-left the longer)."""
    a, b = diagonals(objects)
    return (b - a) * 1000.0


_START = kit.fake_states()
_C0 = np.mean([np.asarray(_START[n].pos, float) for n in ("rail_near", "rail_far", "stile_l", "stile_r")], axis=0)
START_CORNERS = corners(_START)
START_DIFF = diag_diff_mm(_START)


def _marks():
    """Fixed L-shaped blue tape marks on the table just outside each start corner (1 mm thick, 3 mm clear of the corner)."""
    out = []
    for k, c in START_CORNERS.items():
        sx = -1.0 if k[0] == "n" else 1.0
        sy = 1.0 if k[1] == "l" else -1.0
        out.append(Box(f"mark_{k}_x", pos=(c[0] + sx * 0.015, c[1] + sy * 0.007, TOP + 0.0005), size=(0.024, 0.008, 0.001),
                       color=(0.15, 0.35, 0.80)))
        out.append(Box(f"mark_{k}_y", pos=(c[0] + sx * 0.007, c[1] + sy * 0.015, TOP + 0.0005), size=(0.008, 0.024, 0.001),
                       color=(0.15, 0.35, 0.80)))
    return out


OBJECTS = [*kit.objects(), *_marks()]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scene setup: the loop-closing joint, wood friction ---------------------------------------------------------------
_RT: dict = {}


def _descendants(prim):
    out = []
    for c in prim.GetAllChildren():
        out.append(c)
        out.extend(_descendants(c))
    return out


def _scene_setup(stage) -> None:
    """Once mechkit has authored its three hinges: the fourth (far-right) joint as a revolute between stile_r and
    rail_far at their spawn poses (so it reads 0 at spawn), no collision between the pair; the frame's materials."""
    from pxr import Gf, Sdf, UsdPhysics

    paths, env_T = mk._SIM["paths"], mk._SIM["env_T"]
    poses = kit._spawn_poses()
    ref = mk._T(mk._frame_with_x((0, 0, 1)), (*PIVOT["fr"], Z))
    frame = env_T @ poses["rail_far"] @ mk._inv(kit.parts["rail_far"].ref) @ ref
    f0 = mk._inv(env_T @ poses["stile_r"]) @ frame
    f1 = mk._inv(env_T @ poses["rail_far"]) @ frame
    path = f"{paths['rail_far'].rsplit('/', 1)[0]}/mechkit_joints/j_fr_loop"
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([Sdf.Path(paths["stile_r"])])
    j.CreateBody1Rel().SetTargets([Sdf.Path(paths["rail_far"])])

    def gq(r):
        x, y, z, w = mk.mat_to_quat(r)
        return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

    j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in f0[:3, 3]]))
    j.CreateLocalRot0Attr().Set(gq(f0[:3, :3]))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in f1[:3, 3]]))
    j.CreateLocalRot1Attr().Set(gq(f1[:3, :3]))
    j.CreateCollisionEnabledAttr().Set(False)
    j.CreateExcludeFromArticulationAttr().Set(True)
    j.CreateAxisAttr().Set("X")
    j.CreateLowerLimitAttr().Set(-20.0)
    j.CreateUpperLimitAttr().Set(20.0)
    n = 0
    for part in MEMBERS:
        root = stage.GetPrimAtPath(paths[part])
        for p in [root, *_descendants(root)]:
            if p.HasAPI(UsdPhysics.MaterialAPI):
                mat = UsdPhysics.MaterialAPI(p)
                mat.GetStaticFrictionAttr().Set(MU_FRAME)
                mat.GetDynamicFrictionAttr().Set(MU_FRAME)
                n += 1
    _RT["stage"] = stage
    print(f"[{M}] scene: loop joint {path}; {n} frame materials at {MU_FRAME}; joint drag {TAU} N*m x3; start diagonal "
          f"difference {START_DIFF:+.2f} mm", flush=True)


def _patch_scene(mod) -> None:
    from isaaclab.scene import interactive_scene as isc

    cls = isc.InteractiveScene
    if getattr(cls, "_sf_setup", False):
        return
    init0 = cls.__init__

    def __init__(self, *a, **k):
        init0(self, *a, **k)
        try:
            _scene_setup(self.stage)
        except Exception as e:  # noqa: BLE001
            import traceback

            traceback.print_exc()
            print(f"[{M}] scene setup failed: {e!r}", flush=True)

    cls.__init__ = __init__
    cls._sf_setup = True


def _install_scene_patch() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
        return
    for f in sys.meta_path:  # after mechkit's own patch of yam_arena.scene (it authors the three hinges first)
        if isinstance(f, mk._AfterImport) and f.name == "yam_arena.scene":
            p0 = f.patch

            def both(mod, p0=p0):
                p0(mod)
                _patch_scene(mod)

            f.patch = both
            return


_install_scene_patch()

# -- robot points (TCP, both pad tips, the finger middle) -----------------------------------------------------------
_TEST_PTS = None


def _robot_points() -> list:
    if _TEST_PTS is not None:
        return [np.asarray(p, float) for p in _TEST_PTS]
    env = mk._EP.get("env")
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        raw = art.data.body_pos_w
        pos = (getattr(raw, "torch", raw)[0] - origin).detach().cpu().numpy().astype(float)
        g, a, b = pos[names.index("gripper")], pos[names.index("tip_left")], pos[names.index("tip_right")]
        mid = (a + b) / 2
        tcp = g + (mid - g) / (np.linalg.norm(mid - g) + 1e-12) * mk.TCP_PAST_GRIPPER
        out += [tcp, a, b, mid]
    return out


RELEASE_M = 0.025


def released(objects) -> bool:
    """No robot point within 2.5 cm of any member's bounding box."""
    pts = _robot_points()
    for n in MEMBERS:
        lo, hi = np.asarray(objects[n].low) - RELEASE_M, np.asarray(objects[n].high) + RELEASE_M
        if any(bool(np.all(p >= lo) and np.all(p <= hi)) for p in pts):
            return False
    return True


# -- scoring ---------------------------------------------------------------------------------------------------------
SQUARE_MM, NEAR_MM, BETTER_MM = 2.0, 6.0, 12.0  # |diagonal difference| bands (start: 20 mm)
OUTLINE_M = 0.025  # any outer corner more than 25 mm from its start mark: off its outline
LIFT_M = 0.010  # any member's underside more than 10 mm off the table: lifted


def _mem() -> dict:
    return mk.MEMORY.setdefault(M, {"off": None, "lift": None, "k": 0, "label": "", "best": None})


def _latches(objects) -> dict:
    m = _mem()
    if m["off"] is None:
        c = corners(objects)
        worst = max(float(np.linalg.norm((c[k] - START_CORNERS[k])[:2])) for k in c)
        if worst > OUTLINE_M:
            m["off"] = m["k"]
            print(f"[{M}] LATCH off-outline: a corner is {worst * 1000:.1f} mm from its start mark (limit "
                  f"{OUTLINE_M * 1000:.0f} mm) at control step {m['k']}, waypoints '{m.get('label_l', '')}' | "
                  f"'{m['label']}'", flush=True)
    if m["lift"] is None:
        hi = max(float(objects[n].low[2]) - TOP for n in MEMBERS)
        if hi > LIFT_M:
            m["lift"] = m["k"]
            print(f"[{M}] LATCH lifted: a member's underside is {hi * 1000:.1f} mm off the table (limit "
                  f"{LIFT_M * 1000:.0f} mm) at control step {m['k']}, waypoints '{m.get('label_l', '')}' | "
                  f"'{m['label']}'", flush=True)
    return m


def _still(objects) -> bool:
    return all(objects[n].resting() for n in MEMBERS)


# The joint fit (the real rig's check that the frame is a rectangle, not a trapezoid whose diagonals happen to match:
# the mortise play would let a rail slide along a stile). At each of the four joints, the rail end's two shoulder
# corners are read in the stile's frame: their position along the stile against where they started (the rail's edge on
# its pencil mark) and the gap between them across the stile (the open shoulder's gap while the other corner bears).
EDGE_MM, GAP_MM = 1.0, 0.5
SHOULDERS = {"nl": ("rail_near", "stile_l", 1.0), "nr": ("rail_near", "stile_r", -1.0),
             "fl": ("rail_far", "stile_l", 1.0), "fr": ("rail_far", "stile_r", -1.0)}


def _shoulders(objects) -> dict:
    """Per joint: the rail end's two shoulder corners in the stile's frame, [[along, across], [along, across]] (m)."""
    out = {}
    for k, (rail, stile, sg) in SHOULDERS.items():
        t = kit._pose_T(objects, stile)
        pts = [t[:3, :3].T @ (kit.point(objects, rail, (sx * S / 2, sg * (W / 2 - S), 0.0)) - t[:3, 3])
               for sx in (-1.0, 1.0)]
        out[k] = np.array([[p[0], p[1]] for p in pts])
    return out


_SH0 = _shoulders(_START)  # the pencil marks: each rail's edges where they meet the stiles at the start


def joint_fit(objects) -> tuple:
    """(worst rail-edge offset from its pencil mark along a stile, worst shoulder gap), mm, over the four joints."""
    sh = _shoulders(objects)
    edge = max(float(np.max(np.abs(sh[k][:, 0] - _SH0[k][:, 0]))) for k in sh)
    gap = max(float(abs(sh[k][0, 1] - sh[k][1, 1])) for k in sh)
    return edge * 1000.0, gap * 1000.0


def _joints_closed(objects) -> bool:
    """Each rail's edge within 1 mm of its pencil mark on both stiles and the shoulder gaps closed to <= 0.5 mm.
    Trivially true in the sim whenever d <= 2 mm: each joint is a pin at its shoulder, so a rail cannot slide along
    its stile (the edges stay on their marks to within the joints' closure error, < 0.005 mm), and the shoulder gap is
    38 sin(rack), at most 0.18 mm at d = 2 mm. It is kept so the rule matches the real rig, where the mortise play
    lets a rail slide."""
    edge, gap = joint_fit(objects)
    return edge <= EDGE_MM and gap <= GAP_MM


def score(objects) -> float:
    """0 for the trial once any outer corner has been more than 25 mm from its start mark or any member more than
    10 mm off the table. Otherwise by d = |long diagonal - short diagonal| (start 20 mm): 1.0 d <= 2 mm, each rail's
    edge within 1 mm of its pencil mark on both stiles (shoulder gaps closed <= 0.5 mm), with the frame let go (no
    robot point within 2.5 cm) and still; 0.75 d <= 2 mm and the same joint fit, still held or moving; 0.5 d <= 6 mm;
    0.25 d <= 12 mm; else 0 (overshooting past square counts the same as falling short)."""
    m = _latches(objects)
    if m["off"] is not None or m["lift"] is not None:
        return 0.0
    d = abs(diag_diff_mm(objects))
    if d <= SQUARE_MM and _joints_closed(objects):  # joint fit: trivially true in the sim (pins), see _joints_closed
        return 1.0 if (released(objects) and _still(objects)) else 0.75
    if d <= NEAR_MM:
        return 0.5
    if d <= BETTER_MM:
        return 0.25
    return 0.0


def success(objects) -> bool:
    """Square within 2 mm, on its marks, let go and still; held for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres of diagonal difference left."""
    return abs(diag_diff_mm(objects)) / 1000.0


def _on_step(objects) -> None:
    m = _mem()
    m["k"] += 1
    _latches(objects)
    _demo_feedback(objects)
    if DEBUG and m["k"] % 10 == 0:
        c = corners(objects)
        worst = max(float(np.linalg.norm((c[k] - START_CORNERS[k])[:2])) for k in c)
        cen = np.mean([np.asarray(objects[n].pos, float) for n in MEMBERS], axis=0) - _C0
        _, qn = kit.pose(objects, "rail_near")
        yaw = math.degrees(math.atan2(mk.quat_to_mat(qn)[1, 0], mk.quat_to_mat(qn)[0, 0]))
        cl = _cl()
        print(f"[{M}] k {m['k']} '{m.get('label_l', '')}' | '{m['label']}' diff {diag_diff_mm(objects):+.2f} mm corner "
              f"drift {worst * 1000:.1f} mm centre {cen[0] * 1000:+.1f}/{cen[1] * 1000:+.1f} mm yaw {yaw:+.2f} released "
              f"{released(objects)} still {_still(objects)} squeeze {cl['s'] * 1000:+.1f} balance {cl['b'] * 1000:+.1f} "
              f"mm", flush=True)


kit.on_step(_on_step)

# -- the scripted demo -----------------------------------------------------------------------------------------------
# The shop fix: squeeze the long diagonal. Each hand, fingers closed, pushes one end of the long diagonal (the
# near-right and the far-left outer corner) toward the other, along the diagonal, so the two pushes cancel and the
# frame racks without sliding. The demo reads the sim, as the Guide allows: the squeeze advances by a live correction
# from the measured diagonal difference, and a balance term from the frame centre's drift along the diagonal moves
# push from one hand to the other; at square (|d| < 1 mm, or d changes sign) both hands' targets drop at once to where
# the hands actually are, 2 mm back (no push left), then they back away and lift clear. controls() (scorer latch checks): one hand alone grips the far rail
# and drags it, then lifts it.
CL_KEY = "sf_cl"
GAIN = 0.12  # m of correction per m of error, per control step
STEP_MAX = 0.0006  # m per control step (12 mm/s)
RELAX_S = 1.0
STOP_MM = 1.0  # stop pushing once |d| is under 1 mm (the frame coasts a little after the push stops)
UNLOAD_M = 0.002
U = START_CORNERS["fl"] - START_CORNERS["nr"]
U[2] = 0.0
U = U / np.linalg.norm(U)  # the long diagonal, near-right -> far-left (start)


def _cl() -> dict:
    return mk.MEMORY.setdefault(CL_KEY, {"s": 0.0, "b": 0.0, "done": False, "last": None, "relax": None,
                                         "ax_l": 0.0, "ax_r": 0.0})


def _centre_xy(objects) -> np.ndarray:
    return np.mean([np.asarray(objects[n].pos, float)[:2] for n in MEMBERS], axis=0)


def _demo_feedback(objects) -> None:
    cl, m = _cl(), _mem()
    lab_l, lab_r = m.get("label_l", ""), m.get("label", "")
    pts = _robot_points()
    if pts:  # controls(): before closing on a rail, the open hand centres itself on it in x
        for side, lab, i, rail in (("l", lab_l, 0, "rail_far"), ("r", lab_r, 4, "rail_near")):
            if lab.startswith("[align]"):
                ex = float(np.asarray(objects[rail].pos, float)[0]) - float(pts[i][0])
                cl[f"ax_{side}"] += float(np.clip(0.3 * ex, -STEP_MAX, STEP_MAX))
    if lab_l.startswith("[push]") and not cl["done"]:
        dd = diag_diff_mm(objects)
        crossed = cl["last"] is not None and dd * cl["last"] < 0
        cl["last"] = dd
        if (abs(dd) < STOP_MM or crossed) and pts:
            # there: both hands stop pushing at once. The arms lag their targets under load, so freezing the target
            # would let them keep pushing; instead each target is set to where its hand actually is, 2 mm back.
            cl["done"] = True
            a_l = float(np.dot(pts[0][:2] - _P_L0[:2], -U[:2]))
            a_r = float(np.dot(pts[4][:2] - _P_R0[:2], U[:2]))
            cl["relax"] = (a_l - UNLOAD_M, a_r - UNLOAD_M)
            print(f"[{M}] demo: square to {dd:+.2f} mm at step {m['k']} ({'crossed' if crossed else 'within 1 mm'}): "
                  f"stop; hands got {a_l * 1000:+.1f} / {a_r * 1000:+.1f} mm of {(cl['s'] + cl['b']) * 1000:+.1f} / "
                  f"{(cl['s'] - cl['b']) * 1000:+.1f} mm commanded", flush=True)
        else:
            lim = STEP_MAX if abs(dd) > 6.0 else STEP_MAX / 3  # slow down for the last 6 mm
            cl["s"] += float(np.clip(GAIN * dd / 4000.0, -lim, lim))  # each corner moves ~d/4 to square
            drift = float(np.dot(_centre_xy(objects) - _C0[:2], U[:2]))  # the frame sliding along the diagonal
            cl["b"] += float(np.clip(GAIN * drift, -STEP_MAX, STEP_MAX))
    if lab_l.startswith("[relax]") and cl["relax"] is None and pts:
        # never got there in time: ease off to where the hands actually are (2 mm back)
        a_l = float(np.dot(pts[0][:2] - _P_L0[:2], -U[:2]))
        a_r = float(np.dot(pts[4][:2] - _P_R0[:2], U[:2]))
        cl["relax"] = (a_l - UNLOAD_M, a_r - UNLOAD_M)
        print(f"[{M}] demo: time up at {diag_diff_mm(objects):+.2f} mm; ease off", flush=True)


def _patch_arm_plan() -> None:
    cls = _demo.ArmPlan
    if getattr(cls, "_sf_cl", False):
        return
    target0 = cls.target

    def target(self, t):
        pos, rot, finger, label = target0(self, t)
        if mk._EP.get("env") is None:
            return pos, rot, finger, label
        cl = _cl()
        left = self.base_pos[1] > 0
        _mem()["label_l" if left else "label"] = label
        if label.startswith(("[align]", "[grip]", "[ctl]")):  # controls(): grip on a rail
            pos = pos + np.array([cl["ax_l"] if left else cl["ax_r"], 0.0, 0.0])
        if label.startswith(("[push]", "[relax]", "[back]")):
            d = -U if left else U  # this hand's push direction
            s = cl["s"] + cl["b"] if left else cl["s"] - cl["b"]
            if cl["relax"] is not None:  # stopped: hold at (or back off to) where the hand actually was
                s = cl["relax"][0] if left else cl["relax"][1]
            pos = pos + d * s
        return pos, rot, finger, label

    cls.target = target
    cls._sf_cl = True


_patch_arm_plan()

TIP = PAD_TIP_PAST_TCP
PUSH_Z = TOP + 0.005 + TIP  # closed finger tips 5 mm above the table, against the frame's 19 mm corner
OVER_Z = TOP + 0.060
CLEAR_Z = TOP + 0.110
START_GAP = 0.030  # each hand starts 30 mm outside its corner, along the diagonal
_P_R0 = START_CORNERS["nr"] - U * START_GAP
_P_L0 = START_CORNERS["fl"] + U * START_GAP
YAW_PUSH_R = math.atan2(U[1], U[0])  # closing axis across the push: the closed fingers' edges meet the corner
YAW_PUSH_L = YAW_PUSH_R  # the same axis (the plan's IK is cleaner this way round)
# controls(): a grip across a rail
GRIP_Z = TOP + 0.004 + TIP
OPEN = -0.024  # fingers 48 mm apart over the 38 mm rail
YAW_L = -math.pi / 2  # fingers close along x, across the rail
_NOMINAL_Y = 0.10 + float(kit.point(_START, "rail_far", (0, 0, 0))[1])


def _pusher(p0, d, yaw, park, corner):
    c = FINGER_CLOSED
    (x, y, _), (bx, by, _) = p0, p0 - d * 0.020
    return [
        Waypoint(2.0, (x, y, OVER_Z), c, f"over the {corner} corner", yaw),
        Waypoint(1.5, (x, y, PUSH_Z), c, f"down outside the {corner} corner", yaw),
        Waypoint(0.5, (x, y, PUSH_Z), c, "settle", yaw),
        Waypoint(11.0, (x, y, PUSH_Z), c, "[push] squeeze the long diagonal until the diagonals match", yaw),
        Waypoint(RELAX_S, (x, y, PUSH_Z), c, "[relax] ease off", yaw),
        Waypoint(1.0, (bx, by, PUSH_Z), c, "[back] back away 20 mm", yaw),
        Waypoint(1.5, (bx, by, CLEAR_Z), c, "[back] up and clear", yaw),
        Waypoint(1.5, park, c, "back", yaw),
    ]


def demo():
    """Both hands squeeze the long diagonal (near-right and far-left corners) under live correction until the
    diagonals match, ease off, back away and lift clear. About 20 s."""
    return {"left": _pusher(_P_L0, -U, YAW_PUSH_L, (0.16, 0.30, TOP + 0.16), "far-left"),
            "right": _pusher(_P_R0, U, YAW_PUSH_R, (0.16, -0.30, TOP + 0.16), "near-right")}


def controls():
    """Sim controls for the scorer latches: one hand alone grips the far rail and drags it 40 mm (the frame slides
    off its marks: off-outline), then lifts it 25 mm (lifted); the right arm stays parked."""
    x, y = XF, _NOMINAL_Y
    left = [
        Waypoint(2.0, (x, y, OVER_Z), OPEN, "over the far rail", YAW_L),
        Waypoint(1.5, (x, y, GRIP_Z), OPEN, "[align] down beside the rail", YAW_L),
        Waypoint(1.0, (x, y, GRIP_Z), OPEN, "[align] centre on the rail", YAW_L),
        Waypoint(1.5, (x, y, GRIP_Z), FINGER_CLOSED, "[grip] grip the far rail", YAW_L),
        Waypoint(0.5, (x, y, GRIP_Z), FINGER_CLOSED, "[grip] settle", YAW_L),
        Waypoint(4.0, (x, y - 0.040, GRIP_Z), FINGER_CLOSED, "[ctl] drag the far rail alone", YAW_L),
        Waypoint(0.5, (x, y - 0.040, GRIP_Z), FINGER_CLOSED, "[ctl] settle", YAW_L),
        Waypoint(2.0, (x, y - 0.040, GRIP_Z + 0.025), FINGER_CLOSED, "[ctl] lift the far rail 25 mm", YAW_L),
        Waypoint(1.0, (x, y - 0.040, GRIP_Z + 0.025), FINGER_CLOSED, "[ctl] hold", YAW_L),
        Waypoint(1.5, (x, y - 0.040, GRIP_Z + 0.002), FINGER_CLOSED, "[ctl] lower", YAW_L),
        Waypoint(1.0, (x, y - 0.040, GRIP_Z + 0.002), OPEN, "[ctl] let go", YAW_L),
        Waypoint(1.5, (x, y - 0.040, CLEAR_Z), OPEN, "[ctl] up", YAW_L),
    ]
    right = [Waypoint(2.0, (0.16, -0.30, TOP + 0.16), OPEN, "parked"),
             Waypoint(20.0, (0.16, -0.30, TOP + 0.16), OPEN, "parked")]
    return {"left": left, "right": right}


# -- offline self-test -----------------------------------------------------------------------------------------------
def _racked(deg, shift=(0.0, 0.0), yaw_deg=0.0, lift=0.0):
    """Kit states with the frame racked ``deg`` (start: RACK_DEG) about its centre (the members' mean position kept where
    it started, as the demo does), then slid by ``shift`` (m), turned ``yaw_deg`` about that centre and the near rail
    lifted ``lift`` m (the rest follows through the joints)."""
    q = {"j_nl": deg, "j_nr": deg, "j_fl": -deg}
    t = kit.parts["rail_near"].ref.copy()
    c0 = np.mean([np.asarray(_START[n].pos, float) for n in MEMBERS], axis=0)
    st = kit.fake_states(q=q, deg=True)
    dc = np.mean([np.asarray(st[n].pos, float) for n in MEMBERS], axis=0) - c0
    r = mk.axis_angle((0, 0, 1), math.radians(yaw_deg))
    c = np.array([c0[0], c0[1], 0.0])
    pos = r @ (t[:3, 3] - np.array([dc[0], dc[1], 0.0]) - c) + c + np.array([shift[0], shift[1], lift])
    return kit.fake_states(q=q, deg=True, moved={"rail_near": (pos, mk.mat_to_quat(r))})


def _deg_for(diff_mm):
    """The rack angle giving a signed diagonal difference ``diff_mm`` (bisection on the kinematics)."""
    lo, hi = -LIMIT_DEG, LIMIT_DEG
    for _ in range(60):
        mid = (lo + hi) / 2
        if diag_diff_mm(_racked(mid)) < diff_mm:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def self_test():
    """The scoring rules on hand-built frame poses (posed through the joints by the kit)."""
    global _TEST_PTS
    import dataclasses

    assert abs(START_DIFF - 20.0) < 0.1, START_DIFF
    far = [np.array([0.16, 0.30, TOP + 0.16]), np.array([0.16, -0.30, TOP + 0.16])]
    on_rail = [np.array([XF, _NOMINAL_Y, GRIP_Z])]
    moving = dict(_racked(_deg_for(0.5)))
    moving["rail_far"] = dataclasses.replace(moving["rail_far"], lin_vel=np.array([0.0, -0.05, 0.0]))
    slid = dict(_racked(_deg_for(0.0)))  # the far rail slid 2 mm along both stiles (real-rig mortise play): diagonals
    t_far = kit._pose_T(slid, "rail_far")  # still equal (they run between the stiles' corners), rails off their marks
    t_far[0, 3] += 0.002
    slid["rail_far"] = kit._state("rail_far", t_far)
    cases = {  # name: (state, robot points, score)
        "start: racked 20 mm": (_START, far, 0.0),
        "partly squared: 10 mm": (_racked(_deg_for(10.0)), far, 0.25),
        "near square: 4 mm": (_racked(_deg_for(4.0)), far, 0.5),
        "overshot past square: -4 mm": (_racked(_deg_for(-4.0)), far, 0.5),
        "overshot far: -14 mm": (_racked(_deg_for(-14.0)), far, 0.0),
        "square 1.5 mm, let go": (_racked(_deg_for(1.5)), far, 1.0),
        "square -1.5 mm, let go": (_racked(_deg_for(-1.5)), far, 1.0),
        "square 1.5 mm, still gripped": (_racked(_deg_for(1.5)), far + on_rail, 0.75),
        "square but still moving": (moving, far, 0.75),
        "diagonals equal, far rail slid 2 mm off its pencil marks": (slid, far, 0.5),
        "square, slid 30 mm off its marks": (_racked(_deg_for(0.0), shift=(0.0, -0.030)), far, 0.0),
        "square, slid 12 mm (still on its marks)": (_racked(_deg_for(0.0), shift=(0.0, -0.012)), far, 1.0),
        "square, turned 8 deg on the table (corners ~37 mm off)": (_racked(_deg_for(0.0), yaw_deg=8.0), far, 0.0),
        "square, near rail lifted 15 mm": (_racked(_deg_for(0.0), lift=0.015), far, 0.0),
    }
    try:
        for name, (st, pts, want) in cases.items():
            mk.new_episode()
            _TEST_PTS = pts
            got = score(st)
            assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0), (name, got, want, diag_diff_mm(st))
        edge, gap = joint_fit(_racked(_deg_for(2.0)))  # the sim's pins: rails on their marks, shoulders closed at 2 mm
        assert edge < 0.05 and gap < 0.2, (edge, gap)
        edge, gap = joint_fit(slid)
        assert abs(edge - 2.0) < 0.05 and abs(diag_diff_mm(slid)) < 0.05, (edge, diag_diff_mm(slid))
        # latches hold for the trial: slid off then put back still scores 0
        mk.new_episode()
        _TEST_PTS = far
        score(_racked(_deg_for(0.0), shift=(0.0, -0.030)))
        assert score(_racked(_deg_for(0.0))) == 0.0
    finally:
        mk.new_episode()
        _TEST_PTS = None
    return len(cases) + 1


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans (demo and controls) replayed
    # through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass; start diagonal difference {START_DIFF:+.2f} mm; members "
          f"{M_RAIL:.3f}/{M_STILE:.3f} kg; frame {2 * (M_RAIL + M_STILE):.3f} kg")
    task = load(__file__)
    hz = 20.0
    for which, plans in (("demo", task.demo()), ("controls", controls())):
        for arm, waypoints in plans.items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (which, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (which, arm)
            print(f"{which} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
