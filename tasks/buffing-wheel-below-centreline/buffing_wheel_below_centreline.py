"""On the station table in front of the arms stands a double-ended bench buffer, running: a grey cast-iron motor
between the arms with a spindle along y, and a 150 mm cotton buffing wheel (dressed with compound) on each end,
their front faces running DOWN toward the robot's side of the bench. Below the front of each wheel is a steel work
rest, its back edge 3 mm off the wheel and its top 3.7 cm above the bench (about 40 deg below the wheel's centre).
In a steel rack in front of the right wheel stands an aluminium flat-bar blank, 100 x 40 x 10 mm, on its long edge,
one face scribed with a small dark mark. The job: polish both faces of the blank on the buffer, then stand it back in
its rack.

Physics, not a rule, decides how: the wheel drags whatever touches it along its surface (Coulomb friction, 1.0). Held
with its lower edge on the work rest and its face on the wheel below the centreline, the drag runs down the blank
into the rest, which takes it. Held anywhere the rest cannot take the drag (above the centreline, on the top of the
wheel, off the rest on the up-running side) the same drag, acting a few cm from the fingers, twists the blank in the
pinch and snatches it out of position (tried in sim: twisted 64 deg within half a second at a 3-5 N press above the
centreline; on the rest below it, under 8 deg at up to 8 N). A blank that turns more than 25 deg in the gripper
while it is at a wheel, or is flung (0.5 m/s), scores 0 for the trial.

Polishing is measured, not inferred from a pose: PhysX contact reports between the blank and the wheels are read
after every physics step. A physics step polishes a face only when the wheel really touches it (contact points at
0.5 mm separation or less, on the face, on the wheel's down-running front below the spindle) and presses it (at
least 1 N of contact force along the face's normal); a face held off the rim, however close, earns nothing. Each
face is scored in ten 10 mm strips along its length: the strips under the wheel's 25 mm width get the step's time,
a strip is polished after 0.5 s, and a face is polished with 8 of its 10 strips. A face held still at one place
covers at most 3 strips: the work has to be moved across the wheel, and the end under the fingers can only be
reached by shifting the grip.

Mechanism parts come from mechkit.py (in this folder), which adds hinges to the stock runtime in memory. Each wheel
is a 36-sided rigid body on a PhysX revolute joint with a velocity drive (2 rev/s target, 0.5 N*m*s/rad).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/buffing-wheel-below-centreline/buffing_wheel_below_centreline.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import importlib.abc
import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinges and per-episode hooks, installed in memory)

from yam_arena import demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Right arm base (0, -0.305, 0.75), left (0, +0.305, 0.75).
R, WW = 0.075, 0.025  # buffing wheel radius and width (150 x 25 mm)
NSEG = 36  # the wheel's rim: 36 flat segments
XW, ZC = 0.360, TOP + 0.100  # spindle axis (along y)
YW = {"right": -0.200, "left": 0.200}  # wheel centres
SPEED_DEG = 720.0  # drive target: 2 rev/s, front face running down
DRIVE_DAMP = 0.5  # N*m*s/rad velocity drive
WHEEL_KG = 0.30
MOTOR = (0.120, 0.160, 0.130)

PLATE = (0.010, 0.100, 0.040)  # aluminium flat-bar blank: thickness (x), length (y), height (z) in its stand
PLATE_KG = 0.108
STAND_X, STAND_Y = 0.190, -0.200  # the blank's centre in its rack
SLOT_GAP = 0.002  # rack bars 2 mm off each face

GAP = 0.003  # work rest to wheel (OSHA 1910.215 limit: 1/8 in)
REST_D, REST_W, REST_T = 0.060, 0.060, 0.006
CONTACT_DEG = -25.0  # the demo's contact: 25 deg below the centreline
CONTACT_LZ = 0.010  # ... 3 cm up the blank from its lower edge

# scoring
N_STRIPS = 10  # each face is scored in 10 strips of 10 mm along its length
MIN_STRIPS = 8  # a face is polished when at least 8 of its 10 strips are
STRIP_S = 0.5  # seconds of pressed wheel contact a strip needs
PRESS_N = 1.0  # least normal force (N) of a wheel on a face for a physics step to count as polishing it
TOUCH_SEP = 0.0005  # m: a reported contact point counts only at this separation or less (not a hover)
DT = 1.0 / 20.0  # one control step at 20 Hz
SNATCH_DEG = 25.0  # turn of the blank in the gripper while at a wheel
FLING_MS = 0.5
NEAR_M = 0.020  # "at a wheel": a corner of the blank within 2 cm of a wheel's rim
STRIP_W = PLATE[1] / N_STRIPS
STRIP_C = -PLATE[1] / 2 + STRIP_W * (np.arange(N_STRIPS) + 0.5)  # strip centres along the blank (local y)

WHITE = (0.93, 0.90, 0.80)
COMPOUND = (0.78, 0.62, 0.36)
IRON = (0.42, 0.45, 0.48)
STEEL = (0.30, 0.31, 0.33)
ALU = (0.80, 0.82, 0.85)
MARK = (0.15, 0.15, 0.17)
RED = (0.75, 0.15, 0.12)


def _wheel_boxes():
    """A 36-sided cotton buff: rim segments (circumradius R) round a filled centre, flange and arbor stub."""
    a = R * math.cos(math.pi / NSEG)
    seg = 2 * R * math.sin(math.pi / NSEG) * 1.02
    rim = 0.012
    out = []
    for i in range(NSEG):
        t = 2 * math.pi * i / NSEG
        q = tuple(mk.mat_to_quat(mk.axis_angle((0, 1, 0), t)))
        out.append(mk.box((seg, WW, rim), at=((a - rim / 2) * math.sin(t), 0.0, (a - rim / 2) * math.cos(t)),
                          quat=q, color=WHITE if i % 3 else COMPOUND))
    s = (a - rim) * math.sqrt(2) * 0.98
    for k in range(3):
        q = tuple(mk.mat_to_quat(mk.axis_angle((0, 1, 0), k * math.pi / 6)))
        out.append(mk.box((s, WW * 0.9, s), quat=q, color=WHITE))
    return out


kit = mk.Kit()
kit.part("buffer", (XW, 0.0, ZC), [
    mk.box(MOTOR),
    mk.box((0.160, 0.200, ZC - TOP - MOTOR[2] / 2), at=(0, 0, -(ZC - TOP) / 2 - MOTOR[2] / 4)),
    mk.box((0.004, 0.030, 0.020), at=(-MOTOR[0] / 2 - 0.002, 0.0, -0.030), color=RED),  # switch
], color=IRON)
for side, yw in YW.items():
    reach = abs(yw) - MOTOR[1] / 2 - WW / 2
    kit.part(f"wheel_{side}", (XW, yw, ZC), [
        *_wheel_boxes(),
        mk.box((0.016, reach + 0.002, 0.016), at=(0, -math.copysign(reach / 2 + WW / 2, yw), 0), color=STEEL),
    ], mass=WHEEL_KG, color=WHITE)
    # axis -y: a positive turn carries the wheel's front (robot-side) face downward, its top toward the robot
    kit.hinge(f"spindle_{side}", "buffer", f"wheel_{side}", anchor=(XW, yw, ZC), axis=(0, -1, 0),
              lower_deg=-math.inf, upper_deg=math.inf, damping=DRIVE_DAMP)
kit.part("blank", (STAND_X, STAND_Y, TOP + PLATE[2] / 2), [
    mk.box(PLATE),
    mk.box((0.0004, 0.012, 0.008), at=(-PLATE[0] / 2 - 0.0002, -0.035, -0.012), color=MARK),  # scribed mark: face B
], mass=PLATE_KG, color=ALU)


def _contact_rot() -> np.ndarray:
    """The blank leaning back 25 deg (its face toward the wheel tilted 25 deg upward)."""
    return mk.axis_angle((0, 1, 0), math.radians(CONTACT_DEG))


def _rest_geometry():
    """Rest top = the blank's lowest corner when it touches the wheel 25 deg below the centreline; back edge GAP off
    the wheel at that height."""
    q = _contact_rot()
    b = math.radians(CONTACT_DEG)
    t = np.array([XW, 0.0, ZC]) + R * np.array([-math.cos(b), 0.0, math.sin(b)])
    p = t - q @ np.array([PLATE[0] / 2, 0.0, CONTACT_LZ])
    low = min((p + q @ np.array([sx * PLATE[0] / 2, 0.0, sz * PLATE[2] / 2]) for sx in (-1, 1) for sz in (-1, 1)),
              key=lambda c: c[2])
    zr = float(low[2])
    return zr, XW - math.sqrt(R * R - (ZC - zr) ** 2) - GAP


REST_TOP, REST_BACK = _rest_geometry()


def _fixtures():
    out = []
    for side, yw in YW.items():
        out += [
            Box(f"rest_{side}", pos=(REST_BACK - REST_D / 2, yw, REST_TOP - REST_T / 2), size=(REST_D, REST_W, REST_T),
                color=STEEL),
            Box(f"rest_post_{side}", pos=(REST_BACK - REST_D + 0.008, yw, (TOP + REST_TOP - REST_T) / 2),
                size=(0.012, 0.020, REST_TOP - REST_T - TOP), color=STEEL),
        ]
    t = PLATE[0] / 2 + SLOT_GAP + 0.005
    out += [Box(f"rack_bar_{n}", pos=(STAND_X + s * t, STAND_Y, TOP + 0.005), size=(0.010, 0.110, 0.010), color=STEEL)
            for n, s in (("near", -1), ("far", 1))]
    return out


FIXTURES = _fixtures()
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the running wheels ------------------------------------------------------------------------------------------

_DRIVE = {"set": False}


def _run_wheels() -> None:
    """Give both spindle drives their target speed (sim only; the USD attribute persists across resets)."""
    if _DRIVE["set"] or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    for side in YW:
        path = mk._SIM["joint_paths"].get((kit.id, f"spindle_{side}"))
        if path:
            UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "angular").GetTargetVelocityAttr().Set(SPEED_DEG)
    _DRIVE["set"] = True


# -- blank/wheel contact monitor (sim only) -----------------------------------------------------------------------
# The blank carries PhysX's contact-report API (applied when the scene is built, as Isaac Lab's ContactSensor does),
# and a PhysX rigid-contact view (the mechanism behind Isaac Lab's ContactSensor) pairs it with the two wheels only.
# It is read after EVERY physics step: per wheel, the contact points with their separations, and the filtered contact
# force between the blank and that wheel (the view's force matrix, Isaac Lab's force_matrix_w). Only points at a
# separation of 0.5 mm or less are kept: PhysX also lists points inside its contact offset (a hover a millimetre or
# so off the rim), and a hover also carries no force. The points and the force vector (world frame) are queued in
# mk.MEMORY and turned into strip coverage at the next control step, where the force is resolved along the face's
# normal (the wheel's drag lies in the face's plane).

_CBUF, _COVER = "bw_contacts", "bw_cover"
_MON: dict = {"view": None, "err": None, "polls": 0, "touch": 0, "pressed": 0, "fsum": 0.0, "fmax": 0.0,
              "sep_min": float("inf"), "shown": 0}
_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _enable_contact_processing() -> None:
    """Isaac Lab turns PhysX contact processing off unless a ContactSensor exists (this embodiment builds none);
    switch it back on, exactly as Isaac Lab's own ContactSensor does."""
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover  (outside Isaac Sim)
        pass


def _apply_contact_report(stage) -> None:
    """Contact reporting on the blank's rigid body (threshold 0), before the simulation parses the stage."""
    from pxr import PhysxSchema

    path = mk._SIM["paths"].get("blank")
    if path:
        api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path))
        api.CreateThresholdAttr().Set(0.0)
        print(f"[buff] contact reporting on {path}", flush=True)


def _hook_authoring() -> None:
    base = mk._author_all
    if getattr(base, "_buff_hooked", False):
        return

    def _author_all(stage, env_path):
        base(stage, env_path)
        _apply_contact_report(stage)

    _author_all._buff_hooked = True
    mk._author_all = _author_all


def _make_view(mgr) -> None:
    sv = mgr.get_physics_sim_view()
    paths = mk._SIM.get("paths") or {}
    wheels = [paths[f"wheel_{s}"] for s in YW if f"wheel_{s}" in paths]
    if sv is None or "blank" not in paths or not wheels:
        return
    _MON["view"] = sv.create_rigid_contact_view([paths["blank"]], filter_patterns=[wheels],
                                                max_contact_data_count=256)
    _MON["wheel_y"] = [YW[s] for s in YW if f"wheel_{s}" in paths]  # filter order
    print(f"[buff] contact monitor: {paths['blank']} x {wheels}", flush=True)


def _poll_contacts(mgr) -> None:
    """After every physics step: queue, per wheel, the blank's touching contact points and the contact force."""
    try:
        if _MON["view"] is None:
            _make_view(mgr)
        view = _MON["view"]
        if view is None:
            return
        dt = float(mgr.get_physics_dt())
        _, pts, _, sep, counts, starts = view.get_contact_data(dt=dt)
        counts = _np(counts).reshape(-1)
        if not counts.any():
            _MON["polls"] += 1
            return
        fmat = _np(view.get_contact_force_matrix(dt=dt)).reshape(-1, 3).astype(float)  # one row per wheel
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[buff] contact monitor error: {e!r}", flush=True)
        return
    _MON["polls"] += 1
    _MON["touch"] += 1
    starts = _np(starts).reshape(-1)
    pts = _np(pts).reshape(-1, 3).astype(float)
    sep = _np(sep).reshape(-1).astype(float)
    env = mk._SIM["env_T"][:3, 3]
    q = mk.MEMORY.setdefault(_CBUF, [])
    for j in np.flatnonzero(counts):
        idx = np.arange(starts[j], starts[j] + counts[j]).astype(int)
        _MON["sep_min"] = min(_MON["sep_min"], float(sep[idx].min()))
        if _MON["shown"] < 3:
            _MON["shown"] += 1
            print(f"[buff] contact sample: wheel {j}, {len(idx)} points, separations {np.round(sep[idx] * 1000, 2)} mm,"
                  f" force {np.round(fmat[j], 2)} N", flush=True)
        keep = idx[sep[idx] <= TOUCH_SEP]
        if len(keep):
            q.append((pts[keep] - env, fmat[j].copy(), dt, _MON["wheel_y"][j]))


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_buff_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll_contacts(cls)

    step._buff_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    """Patches Isaac Lab's PhysxManager when it is first imported (it needs Kit, so not at task import)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


def _install_hooks() -> None:
    _hook_authoring()
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(type(f).__name__ == "_MgrFinder" for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_buff_hooked", False):
        return
    base_reset = cls.reset

    def reset(self, *a, **k):
        _MON["view"] = None  # rebuilt on the next physics step (the env may have been rebuilt)
        return base_reset(self, *a, **k)  # mechkit's reset clears mk.MEMORY (queued contacts, coverage, latches)

    reset._buff_hooked = True
    cls.reset = reset


_install_hooks()

# -- measurements (poses and contact reports; on a real rig: the trial video and a force-sensing work rest) ------

_TEST_GRIPS = None  # self-test stand-in: list of (tcp, 3x3 gripper frame)
_SNATCH, _FLUNG, _REF = "bw_snatched", "bw_flung", "bw_ref"


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def _grips() -> list:
    """Both grippers' (TCP, frame): columns = closing axis, its normal, approach axis. [] offline."""
    if _TEST_GRIPS is not None:
        return _TEST_GRIPS
    env = mk._EP["env"]
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
        g, tl, tr = (pos[names.index(n)] for n in ("gripper", "tip_left", "tip_right"))
        a = _unit((tl + tr) / 2 - g)
        c = (tr - tl) - float((tr - tl) @ a) * a
        c = _unit(c) if np.linalg.norm(c) > 1e-4 else _unit(np.cross(a, (0.0, 0.0, 1.0)))
        out.append((g + a * mk.TCP_PAST_GRIPPER, np.column_stack([c, np.cross(a, c), a])))
    return out


def _frame(objects):
    p, q = kit.pose(objects, "blank")
    return np.asarray(p, float), mk.quat_to_mat(q)


def _corners(p, r):
    h = np.asarray(PLATE) / 2
    return [p + r @ (h * np.array([sx, sy, sz])) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def _near_wheel(p, r) -> bool:
    """A corner of the blank within NEAR_M of a wheel's rim (inside its width + 5 cm)."""
    for c in _corners(p, r):
        for yw in YW.values():
            if abs(c[1] - yw) < WW / 2 + 0.05 and math.hypot(c[0] - XW, c[2] - ZC) < R + NEAR_M:
                return True
    return False


def _cover() -> dict:
    return mk.MEMORY.setdefault(_COVER, {"A": np.zeros(N_STRIPS), "B": np.zeros(N_STRIPS)})


def _credit(p, r, batches) -> None:
    """Turn queued contact batches [(touching points (k,3) in the env frame, the wheel's contact force on the blank
    (3,), seconds, the wheel's centre y)], one per wheel and physics step, into strip time. A batch polishes a face
    when its points on that face (A = unmarked +x face, B = marked -x face; not the edges) lie on the wheel's
    down-running front below the spindle and the force along the face's normal is at least PRESS_N. The strips whose
    centres lie under the wheel's 25 mm width (its rim edges at the contact, mapped onto the face) get the batch's
    seconds: the rigid 36-sided buff often touches a slightly skewed face at one rim edge only, where a cloth buff
    would bear on its whole width."""
    cov = _cover()
    for pts, fvec, dt, wy in batches:
        pts = np.asarray(pts, float).reshape(-1, 3)
        fn = abs(float(np.asarray(fvec, float) @ r[:, 0]))  # normal force on the faces (the drag is in their plane)
        if fn < PRESS_N:
            continue
        loc = (pts - p) @ r  # blank frame
        zone = (pts[:, 0] < XW) & (pts[:, 2] < ZC)
        for face, s in (("A", 1.0), ("B", -1.0)):
            on = zone & (s * loc[:, 0] > PLATE[0] / 2 - 0.0015) & (np.abs(loc[:, 2]) < PLATE[2] / 2 - 0.0005)
            if not on.any():
                continue
            _MON["pressed"] += 1
            _MON["fsum"] += fn
            _MON["fmax"] = max(_MON["fmax"], fn)
            c = pts[on].mean(axis=0)
            ends = np.array([[c[0], wy - WW / 2, c[2]], [c[0], wy + WW / 2, c[2]]])
            ly = (ends - p) @ r[:, 1]
            lo, hi = max(float(ly.min()), -PLATE[1] / 2), min(float(ly.max()), PLATE[1] / 2)
            cov[face][(STRIP_C >= lo) & (STRIP_C <= hi)] += dt


def _rel_angle(a, b) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(a @ b.T) - 1.0) / 2.0))))


def _track(objects) -> None:
    """Every control step: strip coverage per face from the queued contacts; latched snatch (the blank turns > 25 deg
    in the gripper holding it while it is at a wheel) and fling (0.5 m/s). Cleared only by the episode reset."""
    _run_wheels()
    batches = mk.MEMORY.pop(_CBUF, [])
    if "blank" not in objects:
        return
    p, r = _frame(objects)
    if batches:
        _credit(p, r, batches)
    st = objects["blank"]
    if float(np.linalg.norm(st.lin_vel)) > FLING_MS:
        mk.MEMORY[_FLUNG] = True
    near = _near_wheel(p, r)
    refs = mk.MEMORY.setdefault(_REF, {})
    for i, (tcp, g) in enumerate(_grips()):
        lo, hi = np.asarray(st.low) - 0.025, np.asarray(st.high) + 0.025
        held = bool(np.all(np.asarray(tcp) >= lo) and np.all(np.asarray(tcp) <= hi))
        if not (held and near):
            refs.pop(i, None)
            continue
        rel = g.T @ r
        if i not in refs:
            refs[i] = rel
        elif _rel_angle(rel, refs[i]) > SNATCH_DEG:
            mk.MEMORY[_SNATCH] = True
    if mk._EP.get("env") is not None and mk._EP.get("step", 0) % 40 == 0:
        cov = _cover()
        n = max(1, _MON["pressed"])
        print(f"[buff] step {mk._EP['step']}: strips A {coverage()['A']}/10 {np.round(cov['A'], 1).tolist()} "
              f"B {coverage()['B']}/10 {np.round(cov['B'], 1).tolist()}; physics steps {_MON['polls']}, touching "
              f"{_MON['touch']}, pressed {_MON['pressed']} (mean {_MON['fsum'] / n:.1f} N, max {_MON['fmax']:.1f} N), "
              f"least separation {_MON['sep_min'] * 1000:.2f} mm; "
              f"snatched {bool(mk.MEMORY.get(_SNATCH))} flung {bool(mk.MEMORY.get(_FLUNG))}", flush=True)


kit.on_step(_track)


def _in_rack(objects) -> bool:
    """Standing on its long edge between the rack bars (either face forward), upright within 10 deg."""
    p, r = _frame(objects)
    up = abs(float(r[2, 2]))
    along = abs(float(r[1, 1]))
    st = objects["blank"]
    return bool(abs(p[0] - STAND_X) < 0.006 and abs(p[1] - STAND_Y) < 0.020 and up > math.cos(math.radians(10))
                and along > math.cos(math.radians(10)) and float(st.low[2]) < TOP + 0.004)


def _released(st, margin=0.025) -> bool:
    if _TEST_GRIPS is not None:
        lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
        return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t, _ in _TEST_GRIPS)
    return mk.released(st, margin)


def coverage(objects=None) -> dict:
    """Polished strips per face (0..10): strips with STRIP_S of pressed contact."""
    cov = mk.MEMORY.get(_COVER)
    if cov is None:
        return {"A": 0, "B": 0}
    return {f: int((cov[f] >= STRIP_S - 1e-9).sum()) for f in ("A", "B")}


def polished(objects=None) -> dict:
    return {f: n >= MIN_STRIPS for f, n in coverage(objects).items()}


def score(objects) -> float:
    """0 for the rest of the trial once the blank was snatched (turned > 25 deg in the gripper while at a wheel) or
    flung (0.5 m/s). Otherwise 0.25 per polished face (8 of its 10 strips with 0.5 s each of pressed contact on the
    wheel's down-running front, from PhysX contact reports); 1.0 with both faces polished and the blank back in its
    rack, let go and still."""
    if mk.MEMORY.get(_SNATCH) or mk.MEMORY.get(_FLUNG):
        return 0.0
    pol = polished(objects)
    st = objects["blank"]
    if all(pol.values()) and _in_rack(objects) and st.resting() and _released(st):
        return 1.0
    return 0.25 * sum(pol.values())


def success(objects) -> bool:
    """Both faces polished, the blank back in its rack, let go, no snatch; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Strips still owed (both faces, 1 cm each), plus metres from the blank to its rack."""
    owed = sum(max(0, MIN_STRIPS - n) for n in coverage(objects).values()) * STRIP_W
    p, _ = _frame(objects)
    return float(owed + math.hypot(p[0] - STAND_X, p[1] - STAND_Y))


# -- tilted grips for the scripted demo ----------------------------------------------------------------------------
# yam_arena's Waypoint carries only a yaw about the straight-down grip. The demo tilts that grip about y, so a
# Waypoint's yaw may be a Tilt: a float (0.0) that also carries the full TCP rotation, which ArmPlan uses instead (an
# in-memory patch of yam_arena.demo.grip_rotation; plain yaws unchanged).


class Tilt(float):
    def __new__(cls, rot):
        self = super().__new__(cls, 0.0)
        self.rot = np.asarray(rot, dtype=float)
        return self


def _patch_grip_rotation() -> None:
    if getattr(_demo, "_tilt_patch", False):
        return
    base = _demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return rot.copy() if rot is not None else base(yaw)

    _demo.grip_rotation = grip_rotation
    _demo._tilt_patch = True


_patch_grip_rotation()

# -- the scripted demo ------------------------------------------------------------------------------------------
# Right arm only, open loop from the scene's fixed geometry (it reads no joint, wheel, contact or latch state). The
# fingers close across the blank's thickness at one end (pads over the top 2.1 cm). Face A: the pinch outboard of
# the right wheel, the blank tilted back 25 deg, set down onto the rest and slid in until its face meets the wheel 25
# deg below the centreline, pressed (30 mm command, about 17 N mean contact force in sim through the arm's compliance),
# then worked across the wheel in overlapping presses, from the blank's free end to 22 mm (pinch outboard) or 14 mm
# (pinch inboard) from its middle,
# easing off between presses to slide along the rest (pressed, the rest's friction holds it: tried in sim). Then it is
# backed off, turned 180 deg in
# the hand about the vertical (well clear of the wheel), and face B is done the same way with the pinch inboard. The
# fingers cover the held end, so the blank is stood in the rack, re-gripped by its other end and both faces are done
# again (the same motions, now polishing the other half of each face). Last it goes back in the rack.

TCP_BELOW_TOP = 0.008
SQUEEZE = 0.008
PINCH = 0.031  # pinch centre to the blank's centre, along the blank
PRESS = 0.030
# The wheel's centre plane on the blank (from its centre toward the free end) at each press. With the pinch outboard
# of the wheel the hand reaches the wheel past u = 0.020 (tried in sim: the blank twisted 30 deg in the grip at 0.012),
# so that pass stops at 0.022; with the pinch inboard it goes on to 0.014.
U_OUT = (0.038, 0.030, 0.022)
U_IN = (*U_OUT, 0.014)
U_FREE = U_OUT[0]


def _contact_centre(yw: float, sgn: float, u: float) -> np.ndarray:
    """Blank centre with its working face tangent to the wheel at 25 deg below the centreline (no press) and the
    wheel's centre plane ``u`` from the blank's centre toward its free end. ``sgn`` +1: pinch outboard (toward -y of
    the right wheel); -1: pinch inboard."""
    q = _contact_rot()
    b = math.radians(CONTACT_DEG)
    t = np.array([XW, yw, ZC]) + R * np.array([-math.cos(b), 0.0, math.sin(b)])
    p = t - q @ np.array([PLATE[0] / 2, 0.0, CONTACT_LZ])
    p[1] = yw - sgn * u
    return p


def _tcp_at(p, sgn: float) -> np.ndarray:
    """TCP for blank centre ``p``: at the pinch end, 8 mm below the top edge, in the blank's mid-plane."""
    q = _contact_rot()
    return p + q @ np.array([0.0, 0.0, PLATE[2] / 2 - TCP_BELOW_TOP]) + np.array([0.0, -sgn * PINCH, 0.0])


def _faces_plan(yaw_a, yaw_b) -> list:
    """Both faces, one hold: the face toward the buffer (pinch outboard), then turned in the hand, the other."""
    yw = YW["right"]
    tilt_a = Tilt(_contact_rot() @ _demo.grip_rotation(yaw_a))
    tilt_b = Tilt(_contact_rot() @ _demo.grip_rotation(yaw_b))
    g = -(PLATE[0] - SQUEEZE) / 2
    out = []
    for sgn, tilt, k, steps in ((1.0, tilt_a, 1, U_OUT), (-1.0, tilt_b, 2, U_IN)):

        def at(dx, dz, u=U_FREE, sgn=sgn):
            return tuple(float(v) for v in _tcp_at(_contact_centre(yw, sgn, u) + np.array([dx, 0.0, dz]), sgn))

        u_prev = steps[0]
        if k == 2:
            mid = Tilt(_demo.grip_rotation(0.0))
            hi = (0.200, yw - 0.070, TOP + 0.130)  # well clear of the wheel, the rest and the rack: room for the half turn
            out += [Waypoint(1.5, hi, g, "clear of the wheel, up", tilt_a),
                    Waypoint(1.5, hi, g, "turn the blank in the hand (half)", mid),
                    Waypoint(1.5, hi, g, "turn the blank in the hand: the other face toward the wheel", tilt_b)]
        out += [
            Waypoint(2.5, at(-0.025, 0.020), g, f"face {k} of this hold: over the work rest", tilt),
            Waypoint(1.5, at(-0.022, 0.003), g, "lower the edge onto the rest", tilt),
            Waypoint(2.0, at(0.004, 0.001), g, "slide in to the wheel, below the centreline", tilt),
            Waypoint(1.0, at(PRESS, -0.002), g, "press, the wheel at the blank's free end", tilt),
            Waypoint(1.0, at(PRESS, -0.002), g, "polish the free end", tilt),
        ]
        for u in steps[1:]:  # work across the wheel in overlapping presses, easing off to move along the rest
            out += [
                Waypoint(0.5, at(-0.004, 0.001, u_prev), g, "ease off the wheel", tilt),
                Waypoint(0.6, at(-0.004, 0.001, u), g, "slide along the rest toward the fingers", tilt),
                Waypoint(0.7, at(PRESS, -0.002, u), g, "press", tilt),
                Waypoint(0.8, at(PRESS, -0.002, u), g, "polish", tilt),
            ]
            u_prev = u
        out += [Waypoint(1.0, at(-0.025, 0.004, steps[-1]), g, "back off the wheel", tilt)]
    return out


def demo():
    """Pick the blank from its rack; polish the free half of each face on the right wheel's lower front quadrant with
    the blank's edge on the work rest; stand it in the rack and re-grip it by its other end; polish the other half of
    each face; stand it back in the rack. About 104 s."""
    o = FINGER_OPEN
    g = -(PLATE[0] - SQUEEZE) / 2
    ya, yb = -math.pi / 2, math.pi / 2  # grip yaw toward the buffer, then (turned in the hand) away: joint6 in range
    gz = TOP + PLATE[2] - TCP_BELOW_TOP
    py = STAND_Y - PINCH  # the pinch end (local -y)
    qy = STAND_Y + PINCH  # the same end after the half turn
    up = TOP + 0.120

    def pick():
        return [
            Waypoint(2.0, (STAND_X, py, gz + 0.060), o, "over the blank's end", ya),
            Waypoint(1.5, (STAND_X, py, gz), o, "descend", ya),
            Waypoint(0.5, (STAND_X, py, gz), o, "settle", ya),
            Waypoint(1.0, (STAND_X, py, gz), g, "close across the blank", ya),
            Waypoint(0.5, (STAND_X, py, gz), g, "settle closed", ya),
            Waypoint(1.5, (STAND_X, py, up), g, "lift out of the rack", ya),
        ]

    def place():
        return [
            Waypoint(1.5, (STAND_X + 0.040, qy, up), g, "up, clear of the rest", yb),
            Waypoint(1.5, (STAND_X, qy, up), g, "over the rack", yb),
            Waypoint(1.5, (STAND_X, qy, gz + 0.020), g, "lower into the rack", yb),
            Waypoint(1.0, (STAND_X, qy, gz + 0.002), g, "set down", yb),
            Waypoint(0.5, (STAND_X, qy, gz + 0.002), g, "settle", yb),
            Waypoint(1.0, (STAND_X, qy, gz + 0.002), o, "release", yb),
            Waypoint(1.5, (STAND_X, qy, TOP + 0.110), o, "retreat", yb),
        ]

    return {"right": [
        *pick(),
        *_faces_plan(ya, yb),
        *place(),
        Waypoint(1.5, (STAND_X, STAND_Y, TOP + 0.110), o, "re-grip: over the rack, wrist square", 0.0),
        *pick(),  # the blank stands turned round: this end was the free end of the first hold
        *_faces_plan(ya, yb),
        *place(),
    ]}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of states (the blank posed by the kit; grippers given by hand; the
    contact reports built from the wheel's tangent line on the blank)."""
    global _TEST_GRIPS
    import dataclasses

    assert REST_BACK < XW - math.sqrt(R * R - (ZC - REST_TOP) ** 2) - 0.002, "rest must clear the wheel"
    yw = YW["right"]
    turn = mk.axis_angle((0, 0, 1), math.pi)

    def st(pos, rot, vel=(0.0, 0.0, 0.0)):
        out = kit.fake_states(moved={"blank": (np.asarray(pos, float), tuple(mk.mat_to_quat(rot)))})
        out["blank"] = dataclasses.replace(out["blank"], lin_vel=np.asarray(vel, float))
        return out

    def pose(face, hold, u, deg=CONTACT_DEG):
        """Blank (centre, rot) with ``face`` on the right wheel at ``deg`` (negative = below the centreline), the
        wheel's centre plane ``u`` from the blank's centre toward its free end, in ``hold`` 1 (fingers on the local
        -y end) or 2 (re-gripped: fingers on the +y end)."""
        qa = mk.axis_angle((0, 1, 0), math.radians(deg))
        b = math.radians(deg)
        t = np.array([XW, yw, ZC]) + R * np.array([-math.cos(b), 0.0, math.sin(b)])
        p = t - qa @ np.array([PLATE[0] / 2, 0.0, CONTACT_LZ])
        outboard = (face == "A") == (hold == 1)
        p[1] = yw - u if outboard else yw + u
        return p, (qa if face == "A" else qa @ turn), t

    def line(p, r, t, force, xmirror=False, edge=False):
        """The wheel's contact line on the blank: its two ends (the wheel's width, cut at the blank's ends).
        ``xmirror``: the same line on the wheel's back; ``edge``: moved onto the blank's top edge."""
        lo, hi = max(yw - WW / 2, p[1] - PLATE[1] / 2), min(yw + WW / 2, p[1] + PLATE[1] / 2)
        x = 2 * XW - t[0] if xmirror else t[0]
        pts = np.array([[x, lo, t[2]], [x, hi, t[2]]])
        if edge:
            loc = (pts - p) @ r
            loc[:, 2] = PLATE[2] / 2
            pts = p + loc @ r.T
        return [(pts, -force * r[:, 0], DT, yw)]

    def grip(p, r, hold):
        return [(p + r @ np.array([0.0, -PINCH if hold == 1 else PINCH, PLATE[2] / 2 - TCP_BELOW_TOP]),
                 r @ _demo.grip_rotation(math.pi / 2))]

    def pss(face, hold, us, force=5.0, deg=CONTACT_DEG, **kw):
        out = []
        for u in us:
            p, r, t = pose(face, hold, u, deg)
            out.append((st(p, r), grip(p, r, hold), line(p, r, t, force, **kw)))
        return out

    us_out = [U_FREE] * 40 + [u for u in U_OUT[1:] for _ in range(30)]  # the demo's presses at 20 Hz: pinch outboard
    us_in = us_out + [U_IN[-1]] * 30  # pinch inboard

    def away(face, hold):  # backed off and turned in the hand, well clear of the wheel
        p, r, _ = pose(face, hold, U_OUT[-1])
        off = np.array([-0.12, 0.0, 0.06])
        return [(st(p + off, r), grip(p + off, r, hold), []), (st(p + off, r @ turn), grip(p + off, r @ turn, hold), [])]

    start = st((STAND_X, STAND_Y, TOP + PLATE[2] / 2), np.eye(3))
    in_rack_turned = st((STAND_X, STAND_Y, TOP + PLATE[2] / 2), turn)
    clear = [(np.array([0.10, -0.25, TOP + 0.30]), np.eye(3))]
    mid_rack = [(in_rack_turned, clear, [])]
    rack_end = [(start, clear, [])] * 2

    def both(a1=None, b1=None, b2=None, a2=None, **kw):
        """The demo's routine: hold 1 face A (pinch outboard) then B (inboard); re-grip; hold 2 face B then A."""
        return (pss("A", 1, a1 or us_out, **kw) + away("A", 1) + pss("B", 1, b1 or us_in, **kw) + mid_rack
                + pss("B", 2, b2 or us_out, **kw) + away("B", 2) + pss("A", 2, a2 or us_in, **kw))

    def twisted(frame, deg):
        """The blank turned ``deg`` in its own plane about the pinch (the wheel's drag), the gripper fixed."""
        states, grips, _ = frame
        p, r = _frame(states)
        pin = grips[0][0]
        tw = mk.axis_angle(r[:, 0], math.radians(-deg))
        return (st(pin + tw @ (p - pin), tw @ r), grips, [])

    # geometry: the demo's pass covers 5 strips per hold; a still contact line at most 3; a hover nothing
    def cover_of(frames):
        mk.new_episode()
        for s, gr, c in frames:
            mk.MEMORY.setdefault(_CBUF, []).extend(c)
            _track(s)
        out = coverage()
        mk.new_episode()
        return out

    try:
        _TEST_GRIPS = []
        assert cover_of(pss("A", 1, us_out)) == {"A": 4, "B": 0}, cover_of(pss("A", 1, us_out))
        assert cover_of(pss("B", 1, us_in)) == {"A": 0, "B": 5}
        assert cover_of(pss("B", 2, us_out)) == {"A": 0, "B": 4}
        assert cover_of(both()) == {"A": 9, "B": 9}, cover_of(both())
        assert max(cover_of(pss("A", 1, [u] * 400))["A"] for u in np.linspace(-0.05, 0.05, 41)) <= 3
        assert cover_of(pss("A", 1, us_out, force=0.0)) == {"A": 0, "B": 0}, "a hover must earn nothing"
    finally:
        _TEST_GRIPS = None

    full = both()
    cases = {  # name: (frames [(states, grips, contact batches)], score, success)
        "start": ([(start, clear, [])], 0.0, False),
        "one contact line on each face for 10 s each (the old 3 s rule), back in the rack": (
            pss("A", 1, [0.030] * 200) + away("A", 1) + pss("B", 1, [0.030] * 200) + rack_end, 0.0, False),
        "one hold only: the free half of each face, back in the rack": (
            pss("A", 1, us_out) + away("A", 1) + pss("B", 1, us_in) + rack_end, 0.0, False),
        "face A both holds, face B never, still held": (
            pss("A", 1, us_out) + mid_rack + pss("A", 2, us_in), 0.25, False),
        "both faces, both holds, still held": (full, 0.5, False),
        "both faces, both holds, back in the rack, let go": (full + rack_end, 1.0, True),
        "hovering 1 mm off the rim (contact-offset points, no force), the whole routine": (
            both(force=0.0) + rack_end, 0.0, False),
        "grazing the wheel (0.6 N), the whole routine": (both(force=0.6) + rack_end, 0.0, False),
        "every pass rushed (0.2 s per press)": (both(*[[u for u in U_IN for _ in range(4)]] * 4) + rack_end, 0.0, False),
        "8 of 10 strips on each face (no press past 22 mm)": (both(b1=us_out, a2=us_out) + rack_end, 1.0, True),
        "7 of 10 strips on each face": (both(b1=us_out, a2=us_out[:70], b2=us_out[:70]) + rack_end, 0.0, False),
        "light touch above the centreline (no snatch), the whole routine": (both(deg=25.0) + rack_end, 0.0, False),
        "contact on the up-running back, behind the spindle": (both(xmirror=True) + rack_end, 0.0, False),
        "the blank's top edge on the wheel, not a face": (both(edge=True) + rack_end, 0.0, False),
        "snatched above the centreline (64 deg), then everything right": (
            pss("A", 1, [0.030] * 10, deg=25.0) + [twisted(pss("A", 1, [0.030], deg=25.0)[0], 64.0)] + full + rack_end,
            0.0, False),
        "snatch, then twisted back by hand: still 0": (
            full[:3] + [twisted(full[3], 40.0)] + full + rack_end, 0.0, False),
        "flung off the wheel (0.8 m/s)": (
            full[:200] + [(st(_frame(full[200][0])[0] + np.array([-0.05, 0, -0.05]), _frame(full[200][0])[1],
                              vel=(-0.8, 0, 0)), clear, [])] + full[200:] + rack_end, 0.0, False),
        "polishing with a 5 deg settle in the grip": (
            full[:5] + [twisted(f, 5.0)[:2] + (f[2],) for f in full[5:100]] + full[100:] + rack_end, 1.0, True),
        "whole blank turning with the gripper at the wheel (wrist, not slip)": (
            full[:3] + [(twisted(full[3], 30.0)[0], [(full[3][1][0][0], mk.axis_angle(
                _frame(full[3][0])[1][:, 0], math.radians(-30.0)) @ full[3][1][0][1])], [])] + full + rack_end, 1.0,
            True),
        "both faces, in the rack but still gripped": (
            full + [(in_rack_turned, [(np.array([STAND_X, STAND_Y + PINCH, TOP + PLATE[2] - TCP_BELOW_TOP]),
                                       _demo.grip_rotation(-math.pi / 2))], [])], 0.5, False),
        "both faces, in the rack, still moving": (
            full + [(st((STAND_X, STAND_Y, TOP + PLATE[2] / 2), turn, vel=(0.03, 0.0, 0.0)), clear, [])], 0.5, False),
        "faces done, put down on the table not the rack": (
            full + [(st((0.15, -0.10, TOP + 0.005), mk.axis_angle((0, 1, 0), math.pi / 2)), clear, [])] * 2, 0.5,
            False),
        "a new trial after a snatch starts clean": (
            pss("A", 1, [0.030] * 3, deg=25.0) + [twisted(pss("A", 1, [0.030], deg=25.0)[0], 64.0)] + [None] + full
            + rack_end, 1.0, True),
    }
    try:
        for name, (frames, want, want_success) in cases.items():
            mk.new_episode()
            for f in frames[:-1]:
                if f is None:
                    mk.new_episode()
                    continue
                _TEST_GRIPS = f[1]
                mk.MEMORY.setdefault(_CBUF, []).extend(f[2])
                _track(f[0])
            _TEST_GRIPS = frames[-1][1]
            mk.MEMORY.setdefault(_CBUF, []).extend(frames[-1][2])
            _track(frames[-1][0])
            got, got_success = score(frames[-1][0]), success(frames[-1][0])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want,
                                                                            want_success, coverage())
    finally:
        mk.new_episode()
        _TEST_GRIPS = None
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
