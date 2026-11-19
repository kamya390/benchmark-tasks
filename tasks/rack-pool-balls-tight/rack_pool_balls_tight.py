"""On the station table, at full size: a patch of worsted pool cloth stands for the foot end of a pool table's bed,
with the foot spot and a chalked gather ring marked on it. The fifteen 2 1/4 in balls start scattered across the
cloth at fifteen listed positions, all outside the ring, as after play; the wooden 8-ball triangle rack waits on the
table beyond the foot spot, apex away from the robot. The job: collect the balls to the foot spot, rack them tight
with the head ball on the foot spot in a legal order facing the head of the table, then take the rack away without
disturbing them.

Why it is hard: every ball has to be picked off the cloth and set down in a growing close-packed cluster without
touching its neighbours; then the rack's cavity is larger than the pack (269.9 mm inside from the head ball to the
rear centre, against 255.1 mm for fifteen touching balls), so lowering it around the cluster and lifting it straight
off a loose pack leaves the gaps in place. The pack has to be squeezed into the rack's apex, then the frame lifted
off a group of rolling balls without dragging any of them (a few mm of sideways motion while the walls still touch
the balls rolls them apart) and set down on the table away from the pack.

Physics: the balls are rigid spheres (0.165 kg) whose rolling resistance on the cloth is modelled by angular damping
(PhysX has no rolling friction); contact offsets of 0.1 mm keep the 0.5 mm gap measure meaningful. The rack is a free
wooden body (0.227 kg) made of three bars.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/rack-pool-balls-tight/rack_pool_balls_tight.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (multi-box free part for the rack, per-episode memory, installed in memory)

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = bool(os.environ.get("RK_DEBUG"))
M = "rk"

# -- dimensions (m) ----------------------------------------------------------------------------------------------------
IN = 0.0254
D = 2.25 * IN  # WPA ball, 57.15 mm
R = D / 2
ROW = D * math.sqrt(3) / 2  # row pitch of a tight pack, 49.49 mm
BALL_KG = 0.165  # WPA 5.5-6 oz (156-170 g)
HEAD_TO_REAR = 10.625 * IN  # rack inside, 'from head ball to rear center' (wood 8-ball rack listing)
GAP_B = HEAD_TO_REAR - (4 * ROW + D)  # 14.8 mm: the back bar this far behind a tight pack's back row
WALL_T, WALL_H = 0.015, 1.25 * IN  # bar thickness (UNVERIFIED), height 1.25 in
RACK_KG = 0.227  # 0.5 lb
CLOTH_T = 0.003
CLOTH_TOP = TOP + CLOTH_T
# The foot spot sits at x 0.345 so that every site of the close-packed cluster, including the head ball's, is inside
# the arms' straight-down reach for the collection phase (the head site is the farthest point the grippers must
# place a ball). The earlier revision had the spot at x 0.46 with the balls already in the rack; with collection
# added, that head site is 6 cm beyond reach, so the whole scene moved toward the robot. Everything about the rack,
# pack and scoring is unchanged relative to the spot.
X_SPOT, Y_SPOT = 0.345, 0.0  # the foot spot
TABLE_X0 = 0.36 - 1.1 / 2  # the station table's edge behind the arm bases
START_PAST = float(os.environ.get("RK_START_PAST", "0.015"))  # the loose head ball this far past the spot (+x)
G_LOOSE = 0.002  # the balls stand this far apart in the built cluster
VX0 = X_SPOT + START_PAST + 2 * R  # the rack's inner apex vertex when lowered to nominal (head ball touches both sides)
L_IN = 2 * R + 4 * ROW + R + GAP_B  # inner vertex to the back bar's inner face
# gather ring: a chalked circle the balls must be collected into, centred 90 mm behind the spot; it covers the loose
# and the tight pack (rear corner centres 157/159 mm from its centre) and every scattered ball starts >= 20 mm
# outside it
X_RING, R_RING = X_SPOT - 0.090, 0.165
# the rack starts, and is finally set down, beyond the foot spot (apex away from the robot), its back-bar centre
# 95 mm past the spot: its bar face is 58.9 mm horizontally from the head ball even with the head dead on the spot
# (the 50 mm clearance rule holds with margin for a perfect rack), and the pinch poses there are within reach low
# over the table (the hands keep below about 55 mm when this far out, and do their vertical travel at x 0.40)
X_BAR_START = X_SPOT + 0.095

# legal 8-ball order by rows from the apex (1 at the head, 8 in the middle, a solid and a stripe in the back corners)
ORDER = [[1], [9, 2], [10, 8, 3], [11, 7, 14, 4], [5, 13, 15, 6, 12]]
HUES = {1: (0.95, 0.80, 0.10), 2: (0.10, 0.25, 0.80), 3: (0.85, 0.10, 0.10), 4: (0.45, 0.10, 0.55),
        5: (0.95, 0.45, 0.05), 6: (0.05, 0.50, 0.20), 7: (0.50, 0.12, 0.08), 8: (0.04, 0.04, 0.04)}


def ball_color(n):
    if n <= 8:
        return HUES[n]
    return tuple(0.55 * c + 0.45 for c in HUES[n - 8])  # stripes: rendered as a lighter solid of the same hue


def lattice(x_head, y0, spacing):
    """Ball centres (x, y) by rows from the head ball, rows toward -x, at the given centre spacing."""
    return [[(x_head - i * spacing * math.sqrt(3) / 2, y0 + (j - i / 2) * spacing) for j in range(i + 1)] for i in range(5)]


NAMES = [f"ball_{n}" for row in ORDER for n in row]  # row by row from the head
HEAD = "ball_1"
_ix = {}
for _i in range(5):
    for _j in range(_i + 1):
        _ix[(_i, _j)] = len(_ix)


# -- the rack (a free body of three bars) --------------------------------------------------------------------------------
def rack_boxes():
    """Rack frame: origin at the inner apex vertex, x toward the apex, z at mid-height of the bars. The side bars' inner
    faces are the two 60-deg V lines; the back bar's inner face is L_IN behind the vertex."""
    c30, s30 = math.cos(math.radians(30)), math.sin(math.radians(30))
    side = L_IN / c30 + 2 * WALL_T
    out = []
    for s in (1, -1):
        u = np.array([-c30, s * s30])
        n_in = np.array([-s30, -s * c30])
        c = u * (side / 2 - WALL_T) - n_in * WALL_T / 2
        out.append(mk.box((side, WALL_T, WALL_H), at=(c[0], c[1], 0.0), yaw=math.atan2(u[1], u[0])))
    half = L_IN * math.tan(math.radians(30)) + WALL_T / c30
    out.append(mk.box((WALL_T, 2 * half, WALL_H), at=(-L_IN - WALL_T / 2, 0.0, 0.0)))
    return out


RACK_BOXES = rack_boxes()
WOOD = (0.55, 0.33, 0.16)
kit = mk.Kit()
# the rack starts at its waiting spot beyond the foot spot (back-bar centre at X_BAR_START); its back half lies on
# the cloth and its apex on the bare table, so it settles with a 0.5-degree tilt over the 3 mm cloth step
VX_START = X_BAR_START + WALL_T / 2 + L_IN  # inner apex vertex at the start
kit.part("rack", (VX_START, Y_SPOT, CLOTH_TOP + 0.0005 + WALL_H / 2), RACK_BOXES, mass=RACK_KG, color=WOOD)

SITES = [p for row in lattice(X_SPOT + START_PAST, Y_SPOT, D + G_LOOSE) for p in row]  # the loose cluster, by NAMES
# The fifteen scattered start positions (m, world; listed in task.json in cm): a fixed seeded layout, every ball on
# the cloth, >= 26 mm outside the gather ring, >= 87 mm from every other ball, clear of the rack's waiting spot, and
# within the matching arm's reach (y > 0 collected by the left arm, y < 0 by the right, by row from the apex,
# centre-out). Each scattered position is paired with the ball number whose cluster site that arm builds.
SCATTER_L = [(0.210, 0.275), (0.105, 0.270), (0.375, 0.265), (0.070, 0.190), (0.170, 0.175), (0.360, 0.170),
             (0.075, 0.100)]
SCATTER_R = [(0.075, -0.095), (0.360, -0.160), (0.165, -0.180), (0.070, -0.185), (0.270, -0.205), (0.375, -0.260),
             (0.105, -0.270), (0.210, -0.270)]
# site index (row i, pos j) -> NAMES index is row-major: [(0,0),(1,0),(1,1),(2,0),(2,1),(2,2),(3,0)..(4,4)]
SITE_IX = {(i, j): i * (i + 1) // 2 + j for i in range(5) for j in range(i + 1)}
# global fill order (rows from the apex, centre-out) and the arm that owns each site: sites with y > 0 go to the
# left arm, y < 0 to the right; the three centre-line sites split 1 left / 2 right so each arm's load matches its
# side of the scatter (7 left, 8 right)
FILL = [((0, 0), "L"), ((1, 1), "L"), ((1, 0), "R"), ((2, 1), "R"), ((2, 2), "L"), ((2, 0), "R"), ((3, 2), "L"),
        ((3, 1), "R"), ((3, 3), "L"), ((3, 0), "R"), ((4, 2), "R"), ((4, 3), "L"), ((4, 1), "R"), ((4, 4), "L"),
        ((4, 0), "R")]
# release grip yaw (Waypoint convention: 0 closes the fingers along world y; the pad axis repeats every 180 deg) and
# release height (ball bottom above the cloth, m) per site, from the offline pad-clearance scan: sites with three
# placed neighbours take a 20 mm drop (+2.4 mm pad clearance), the rest are set down 2 mm up (>= +8.9 mm). Each
# site's yaw is the scan's optimum carried into the Waypoint frame (+90 deg), on the 180-mirror branch its arm's
# wrist can reach.
SITE_YAW = {(0, 0): -90, (1, 1): -30, (1, 0): 60, (2, 1): 0, (2, 2): -60, (2, 0): 60, (3, 2): 0, (3, 1): 30,
            (3, 3): -60, (3, 0): 60, (4, 2): 180, (4, 3): 30, (4, 1): 90, (4, 4): -60, (4, 0): 60}
SITE_DROP = {(3, 1), (4, 3), (4, 1)}
H_SET, H_DROP = 0.002, 0.020


def _pair_scatter():
    """Pair each arm's scattered balls with its sites in fill order, nearest first (deterministic)."""
    pools = {"L": list(SCATTER_L), "R": list(SCATTER_R)}
    out = {}
    for site, arm in FILL:
        t = np.array(SITES[SITE_IX[site]])
        pool = pools[arm]
        k = int(np.argmin([np.hypot(*(np.array(p) - t)) for p in pool]))
        out[site] = (arm, pool.pop(k))
    return out


SCATTER_FOR_SITE = _pair_scatter()
START = [None] * 15
for _site, (_arm, _sc) in SCATTER_FOR_SITE.items():
    START[SITE_IX[_site]] = _sc
OBJECTS = [
    *kit.objects(),
    Box("cloth", pos=(X_SPOT - 0.03, 0.0, TOP + CLOTH_T / 2), size=(0.56, 0.62, CLOTH_T), color=(0.05, 0.38, 0.20)),
    # the gather ring: a 4 mm chalk circle of radius 165 mm about (X_RING, 0), drawn as a white disc under a
    # cloth-green fill (painted marks: no colliders, like the foot spot)
    Cylinder("ring_chalk", pos=(X_RING, Y_SPOT, CLOTH_TOP + 0.0001), radius=R_RING + 0.002, height=0.0002,
             color=(0.95, 0.95, 0.92)),
    Cylinder("ring_fill", pos=(X_RING, Y_SPOT, CLOTH_TOP + 0.0003), radius=R_RING - 0.002, height=0.0002,
             color=(0.05, 0.38, 0.20)),
    Cylinder("foot_spot", pos=(X_SPOT, Y_SPOT, CLOTH_TOP + 0.0005), radius=0.0127, height=0.0002, color=(0.95, 0.95, 0.92)),
    *[Sphere(name, pos=(x, y, CLOTH_TOP + R + 0.0001), radius=R, mass=BALL_KG, color=ball_color(int(name[5:])))
      for name, (x, y) in zip(NAMES, START)],
]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- materials and contact (scene setup, in memory) ----------------------------------------------------------------------
MU = {"ball": 0.06, "cloth": 0.34, "wood": 0.54}  # average combine: ball-ball 0.06, ball-cloth 0.20, ball-wood 0.30
OFFS = 0.0001  # contact offset (m) on balls, rack and cloth; rest offset 0
ANG_DAMP = 10.0  # 1/s on the balls: rolling resistance stand-in


def _descendants(prim):
    out = []
    for c in prim.GetAllChildren():
        out.append(c)
        out.extend(_descendants(c))
    return out


def _scene_setup(stage) -> None:
    from pxr import PhysxSchema, UsdPhysics

    env = mk._SIM["paths"]["rack"].rsplit("/", 1)[0]
    groups = {"wood": [f"{env}/rack"], "cloth": [f"{env}/cloth"], "ball": [f"{env}/{b}" for b in NAMES]}
    n_mat = n_col = 0
    for kind, roots in groups.items():
        for path in roots:
            root = stage.GetPrimAtPath(path)
            if not root.IsValid():
                print(f"[rk] missing prim {path}", flush=True)
                continue
            for p in [root, *_descendants(root)]:
                if p.HasAPI(UsdPhysics.MaterialAPI):
                    m = UsdPhysics.MaterialAPI(p)
                    m.GetStaticFrictionAttr().Set(MU[kind])
                    m.GetDynamicFrictionAttr().Set(MU[kind])
                    (m.GetRestitutionAttr() or m.CreateRestitutionAttr()).Set(0.0)
                    n_mat += 1
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    c = PhysxSchema.PhysxCollisionAPI.Apply(p)
                    c.CreateContactOffsetAttr().Set(OFFS)
                    c.CreateRestOffsetAttr().Set(0.0)
                    n_col += 1
            if kind == "ball":
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
                rb.CreateAngularDampingAttr().Set(ANG_DAMP)
                rb.CreateLinearDampingAttr().Set(0.0)
                rb.CreateSolverPositionIterationCountAttr().Set(32)
                rb.CreateSolverVelocityIterationCountAttr().Set(4)
                rb.CreateSleepThresholdAttr().Set(0.0)
            if kind == "wood":
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
                rb.CreateSolverPositionIterationCountAttr().Set(32)
                rb.CreateSolverVelocityIterationCountAttr().Set(4)
    for mark in ("foot_spot", "ring_chalk", "ring_fill"):  # painted marks: no colliders
        prim = stage.GetPrimAtPath(f"{env}/{mark}")
        for p in [prim, *_descendants(prim)]:
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)
    print(f"[rk] scene: {n_mat} materials, {n_col} colliders at contact offset {OFFS * 1000:.1f} mm / rest 0; mu {MU}; "
          f"ball angular damping {ANG_DAMP}; start: 15 balls scattered outside the gather ring (r {R_RING * 1000:.0f} mm "
          f"at x {X_RING * 1000:.0f}), rack waiting at back-bar x {X_BAR_START * 1000:.0f}", flush=True)


def _patch_scene(mod) -> None:
    from isaaclab.scene import interactive_scene as isc

    cls = isc.InteractiveScene
    if getattr(cls, "_rk_setup", False):
        return
    init0 = cls.__init__

    def __init__(self, *a, **k):
        init0(self, *a, **k)
        try:
            _scene_setup(self.stage)
        except Exception as e:  # noqa: BLE001
            import traceback

            traceback.print_exc()
            print(f"[rk] scene setup failed: {e!r}", flush=True)

    cls.__init__ = __init__
    cls._rk_setup = True


def _install_scene_patch() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
        return
    for f in sys.meta_path:  # after mechkit's own patch of yam_arena.scene
        if isinstance(f, mk._AfterImport) and f.name == "yam_arena.scene":
            p0 = f.patch

            def both(mod, p0=p0):
                p0(mod)
                _patch_scene(mod)

            f.patch = both
            return


_install_scene_patch()

# -- readouts ------------------------------------------------------------------------------------------------------------
GATE_RISE = 0.002  # m: the rack counts as rising once its lowest point is this far above the cloth
GATE_REST = 0.0005  # m: back down (lowest point within this of the cloth, still, not away): the gate re-arms
GATE_MOVE = 0.003  # m: any ball's centre moving more than this horizontally while the rack rises: latched 0
CLEAR = 0.050  # m: rack clear: every part this far horizontally (seen from above) from every ball's outline...
DOWN_Z = 0.005  # m: ...and set down: its lowest point within this of the table top (on the table or the cloth), still
GAP_OK = 0.0005  # m: a pair counts as tight at <= 0.5 mm (feeler gauge)
SPOT_25, SPOT_100 = 0.010, 0.005  # m: head-ball centre to the spot's centre
HAND_CLEAR = 0.030  # m: every robot point this far from every ball surface
# Hands off after the rise: from the valid 2 mm rise to the end of the trial no gripper point (TCPs, pad tips, pad
# middles) may come within 10 mm of any ball surface, or the score is capped at collection credit (0.25).  Packing
# needs contact, so this is what stops a pack being tightened after the rack has gone.  10 mm is several times the
# real rig's error (logged joints + forward kinematics against ball centres read to ~1 mm overhead); the demo's
# pads, which hold the rack's back bar 14.8 mm behind the rear row, stay at least 12 mm away while lifting it.
HANDS_OFF = 0.010
# Rack stays away: once set down clear, every rack part must stay at least CLEAR (50 mm) horizontally from every ball
# to the end of the trial, or the score is capped at 0.25.  Otherwise the rack could be picked up again and its bars
# used to push a loose pack tight after the scored lift-off.
STILL = 0.005  # m/s
# Orientation (for 0.75 and 1.0). The table's long axis is x; the head of the table (head string) is +x, the way the
# apex points in the setup. APEX_TOL: the direction from the pack's centroid (132 mm behind the head ball) to the head
# ball within +-5 deg of +x, i.e. the rear row's centre within 17 mm (under a third of a ball) of the long string; this
# is what rejects a pack facing the wrong way (180 deg), which the rear-row line alone cannot tell apart. REAR_TOL: the
# line fitted through the five rear-row centres within +-3 deg of square to the long axis, i.e. its end balls within
# 6.0 mm (a tenth of a ball) of square. A tight pack is rigid, so this also bounds its rotation to +-3 deg. Overhead
# grid photo, centres to ~1 mm: angle error about 0.6 deg (apex) and 0.3 deg (rear-row fit), well inside both.
APEX_TOL, REAR_TOL = 5.0, 3.0  # deg
def centres(objects) -> np.ndarray:
    return np.array([np.asarray(objects[b].pos, float) for b in NAMES])


def pair_gaps(c) -> np.ndarray:
    """The 30 smallest pair gaps, independent of ball identity.

    Fifteen equal balls in a complete triangular pack have exactly 30 neighbouring pairs.  Every non-neighbour is
    at least sqrt(3) diameters away, so selecting the 30 shortest centre distances finds the physical contacts while
    accepting any legal permutation of the non-head balls.
    """
    d = [float(np.linalg.norm(c[a] - c[b]) - D) for a in range(15) for b in range(a + 1, 15)]
    return np.sort(np.asarray(d))[:30]


def _hull(p) -> np.ndarray:
    """Convex hull (counter-clockwise) of 2D points."""
    p = sorted(map(tuple, np.round(p, 9)))
    lower, upper = [], []
    for seq, out in ((p, lower), (p[::-1], upper)):
        for q in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (q[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (q[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(q)
    return np.array(lower[:-1] + upper[:-1])


def _dist_to_poly(q, poly) -> np.ndarray:
    """Distance (m) from each 2D point in q to the convex polygon poly (CCW); 0 inside."""
    a, b = poly, np.roll(poly, -1, axis=0)
    e = b - a
    rel = q[:, None, :] - a[None, :, :]
    inside = np.all(e[None, :, 0] * rel[:, :, 1] - e[None, :, 1] * rel[:, :, 0] >= 0, axis=1)
    s = np.clip((rel * e[None]).sum(-1) / (e * e).sum(-1)[None], 0.0, 1.0)
    d = np.linalg.norm(rel - s[..., None] * e[None], axis=2).min(axis=1)
    return np.where(inside, 0.0, d)


def rack_clearance(objects, c) -> float:
    """Least horizontal distance (m), seen from above, from any ball's outline to the rack: each bar's footprint (its
    oriented box projected on the table) against each ball's circle."""
    t = kit._pose_T(objects, "rack")
    best = 9.0
    for b in RACK_BOXES:
        rw = t[:3, :3] @ mk.quat_to_mat(b.quat)
        cw = t[:3, :3] @ np.asarray(b.at) + t[:3, 3]
        h = np.asarray(b.size) / 2
        corners = np.array([cw + rw @ (h * np.array([i, j, k])) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
        best = min(best, float((_dist_to_poly(c[:, :2], _hull(corners[:, :2])) - R).min()))
    return best


def rack_down(objects) -> bool:
    """The rack set down: its lowest point within 5 mm of the table top and moving under 5 mm/s."""
    st = objects["rack"]
    return bool(float(np.asarray(st.low)[2]) - TOP <= DOWN_Z and float(np.linalg.norm(np.asarray(st.lin_vel))) < STILL)


def rack_away(objects, c) -> bool:
    return rack_down(objects) and rack_clearance(objects, c) >= CLEAR


def rack_low(objects) -> float:
    return float(np.asarray(objects["rack"].low)[2] - CLOTH_TOP)


def rack_contains(objects, c) -> bool:
    """All ball outlines lie inside the three inner rack faces, independent of rack height and velocity.

    In the rack frame the inner apex is (0, 0), the rear face is x=-L_IN and the side faces are the two
    60-degree V lines.  Distances are reduced by one ball radius, so a centre outside a face cannot count.
    """
    t = kit._pose_T(objects, "rack")
    q = (c - t[:3, 3]) @ t[:3, :3]
    rear_in = q[:, 0] + L_IN >= R
    sides_in = -0.5 * q[:, 0] - math.cos(math.radians(30)) * np.abs(q[:, 1]) >= R
    return bool(np.all(rear_in & sides_in))


def rack_encloses(objects, c) -> bool:
    """All ball outlines are inside the rack while it is down and still."""
    return rack_low(objects) <= GATE_REST and _rack_still(objects) and rack_contains(objects, c)


# robot points: TCP, both pad tips and both pad middles of each gripper (pad points from the URDF collision meshes, in
# each finger link's frame, m)
PAD_LOCAL = {"tip_left": [(0.0237, -0.04673, -0.09016), (0.01703, -0.04519, -0.06016)],
             "tip_right": [(-0.0237, 0.04674, -0.09016), (-0.01526, 0.04518, -0.06016)]}
_TEST_POINTS = None


def _quat_xyzw_from(q4, wxyz) -> np.ndarray:
    return np.array([q4[1], q4[2], q4[3], q4[0]]) if wxyz else np.asarray(q4)


def robot_points() -> list:
    """World points on both grippers (TCP, pad tips, pad middles); [] offline."""
    if _TEST_POINTS is not None:
        return _TEST_POINTS
    env = mk._EP["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = mk._arr(base.scene.env_origins)[0] if hasattr(mk, "_arr") else _arr(base.scene.env_origins)[0]
    out = list(mk.tcps())
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = _arr(art.data.body_pos_w)[0] - origin
        quat = _arr(art.data.body_quat_w)[0]
        for link, pts in PAD_LOCAL.items():
            k = names.index(link)
            r = mk.quat_to_mat(_quat_xyzw_from(quat[k], _RT.get("wxyz", True)))
            out += [pos[k] + r @ np.asarray(p) for p in pts]
    return out


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def hand_clearance(c) -> float:
    pts = robot_points()
    if not pts:
        return 9.0
    p = np.asarray(pts)
    return float((np.linalg.norm(p[:, None, :] - c[None, :, :], axis=2) - R).min())


_RT: dict = {}


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"n": 0, "ref": None, "gate": None, "max_move": 0.0, "enclosed": None,
                            "cleared": None, "lift_ready": False, "rise_pairs": 0, "rise_head": float("inf"),
                            "rise_oriented": False, "touched": None, "rack_back": None, "tiers": "", "label": ""}
    return m


def _check_quat_convention() -> None:
    """Pick the body-quaternion order (wxyz or xyzw) for which each pad tip sits within 6 cm of its own TCP."""
    if "wxyz" in _RT:
        return
    env = mk._EP["env"]
    if env is None:
        return
    errs = {}
    for wxyz in (True, False):
        _RT["wxyz"] = wxyz
        pts = robot_points()
        tcps = pts[:2]
        tips = [pts[2], pts[4], pts[6], pts[8]]
        errs[wxyz] = max(min(np.linalg.norm(t - c) for c in tcps) for t in tips)
    _RT["wxyz"] = errs[True] <= errs[False]
    print(f"[rk] body quaternions read as {'wxyz' if _RT['wxyz'] else 'xyzw'} (pad tip to TCP: wxyz "
          f"{errs[True] * 1000:.1f} mm, xyzw {errs[False] * 1000:.1f} mm)", flush=True)


def _rack_still(objects) -> bool:
    return float(np.linalg.norm(np.asarray(objects["rack"].lin_vel))) < STILL


def _gate_step(m, c, low, away, still, valid_enclosure=False):
    """The rack-use sequence and lift gate, one control step.

    All fifteen balls must be inside the three inner faces in the rack's last down state immediately before the rise.
    If the rack leaves that enclosure while still down, the episode evidence is invalidated.  Until the valid rise is
    set down away, moving any ball more than 3 mm sideways latches 0.  Carrying the empty waiting rack therefore
    neither fails nor satisfies the sequence.
    """
    if m["gate"] is not None or m["cleared"] is not None:
        return None
    if m["ref"] is None:
        # Enclosure must remain geometrically true on every step from the last down observation through the 2 mm
        # crossing.  This closes both low horizontal departures and the 0.5--2 mm pre-rise deadband.
        if not valid_enclosure:
            m["enclosed"] = None
            m["lift_ready"] = False
            m["max_move"] = 0.0
            m["rise_pairs"] = 0
            m["rise_head"] = float("inf")
            m["rise_oriented"] = False
            return None
        if low <= GATE_REST and still:
            first = m.get("enclosed") is None or not m.get("lift_ready", False)
            m["enclosed"] = m["n"]
            m["lift_ready"] = True
            if first:
                return "enclosed"
        if low > GATE_RISE and m.get("lift_ready", False) and m.get("enclosed") is not None:
            m["ref"] = c.copy()
            gaps = pair_gaps(c)
            m["rise_pairs"] = int((gaps <= GAP_OK).sum())
            m["rise_head"] = _head_off(c)
            m["rise_oriented"] = _oriented(c)
            return "rising"
        return None
    mv = float(np.linalg.norm((c - m["ref"])[:, :2], axis=1).max())
    m["max_move"] = max(m["max_move"], mv)
    if mv > GATE_MOVE:
        m["gate"] = m["n"]
        return "latched"
    if away:
        m["cleared"] = m["n"]
        return "cleared"
    # A retry means returning the rack flat around the pack.  Touching empty cloth near the pack during carry-out is
    # still part of the same removal episode; keep monitoring until that rack reaches the clear set-down region.
    if low <= GATE_REST and still and valid_enclosure:
        m["ref"] = None
        m["max_move"] = 0.0
        m["enclosed"] = None
        m["lift_ready"] = False
        m["rise_pairs"] = 0
        m["rise_head"] = float("inf")
        m["rise_oriented"] = False
        m["touched"] = None
        return "rearmed"
    return None


def _hands_step(m, hand_clear) -> bool:
    """Hands off from the valid rise to the end of the trial (after a clear set-down too).

    Returns True on the step a gripper point first comes within HANDS_OFF of a ball.  A retry (the rack back flat
    around the pack, `rearmed`) starts a new removal and clears this, because touching balls while racking is allowed.
    """
    if m.get("touched") is not None or m["gate"] is not None:
        return False
    if (m["ref"] is not None or m["cleared"] is not None) and hand_clear < HANDS_OFF:
        m["touched"] = m["n"]
        return True
    return False


def _rack_back_step(m, clear) -> bool:
    """After the clear set-down, the rack coming back within CLEAR of any ball is recorded (True on that step)."""
    if m.get("rack_back") is not None or m["cleared"] is None or m["gate"] is not None:
        return False
    if clear < CLEAR:
        m["rack_back"] = m["n"]
        return True
    return False


def _on_step(objects) -> None:
    """Every control step: the lift gate (`_gate_step`), rack clearance, and a trace."""
    if mk._EP.get("env") is None:
        return
    m = _mem()
    m["n"] += 1
    if m["n"] == 2:
        _check_quat_convention()
    c = centres(objects)
    low = rack_low(objects)
    clear = rack_clearance(objects, c)
    # Height and stillness are handled by the state machine.  Keeping geometric containment separate lets a valid
    # observation survive the first fraction of a millimetre of upward motion, but never a low horizontal departure.
    valid_enclosure = rack_contains(objects, c)
    ev = _gate_step(m, c, low, rack_away(objects, c), _rack_still(objects), valid_enclosure)
    hc = hand_clearance(c)
    m["min_hand_after_rise"] = min(m.get("min_hand_after_rise", 9.0), hc) if m["ref"] is not None else m.get(
        "min_hand_after_rise", 9.0)
    if _hands_step(m, hc):
        print(f"[rk] HANDS-OFF broken: a gripper point came {hc * 1000:.1f} mm from a ball after the rack rose "
              f"(limit {HANDS_OFF * 1000:.0f} mm), control step {m['n']}: score capped at collection credit", flush=True)
    if _rack_back_step(m, clear):
        print(f"[rk] RACK-BACK: after its clear set-down the rack came {clear * 1000:.0f} mm from a ball (limit "
              f"{CLEAR * 1000:.0f} mm), control step {m['n']}: score capped at collection credit", flush=True)
    if ev == "enclosed":
        print(f"[rk] rack enclosed all fifteen balls and lift gate armed at control step {m['n']}", flush=True)
    elif ev == "rising":
        g = pair_gaps(c)
        print(f"[rk] rack rising (lowest point {low * 1000:.1f} mm up) at control step {m['n']}: pairs <= 0.5 mm "
              f"{int((g <= GAP_OK).sum())}/30, widest gap {g.max() * 1000:.2f} mm, head ball "
              f"{_head_off(c) * 1000:.1f} mm from the spot", flush=True)
    elif ev == "latched":
        print(f"[rk] LATCH lift-disturbed: a ball moved {m['max_move'] * 1000:.2f} mm sideways while the rack was rising "
              f"(limit {GATE_MOVE * 1000:.0f} mm), control step {m['n']}, rack lowest point {low * 1000:.1f} mm", flush=True)
    elif ev == "rearmed":
        print(f"[rk] rack back down on the cloth at control step {m['n']}: lift gate re-armed", flush=True)
    elif ev == "cleared":
        apex, rear = orientation(c)
        g = pair_gaps(c)
        print(f"[rk] rack set down {clear * 1000:.0f} mm (horizontally) from the nearest ball at control step {m['n']}: "
              f"pairs <= 0.5 mm {int((g <= GAP_OK).sum())}/30, widest gap {g.max() * 1000:.2f} mm, head ball "
              f"{_head_off(c) * 1000:.1f} mm from the spot; most any ball moved "
              f"while it rose: {m['max_move'] * 1000:.2f} mm; orientation: apex {apex:.2f} deg off +x (limit "
              f"{APEX_TOL:.0f}), rear row {rear:.2f} deg off square (limit {REAR_TOL:.0f}); nearest gripper point to a "
              f"ball since the rise: {m.get('min_hand_after_rise', 9.0) * 1000:.1f} mm (hands-off limit "
              f"{HANDS_OFF * 1000:.0f} mm)", flush=True)
    if m["label"] != m.get("phase"):
        m["phase"] = m["label"]
        g = pair_gaps(c)
        apex, rear = orientation(c)
        print(f"[rk] k={m['n']} phase '{m['label']}': pairs <= 0.5 mm {int((g <= GAP_OK).sum())}/30, widest gap "
              f"{g.max() * 1000:.2f} mm, head ball {_head_off(c) * 1000:.1f} mm; apex {apex:.2f} deg, rear row "
              f"{rear:.2f} deg; rack low {low * 1000:+.1f} mm, at x {kit.pose(objects, 'rack')[0][0] * 1000:.1f} mm, "
              f"{clear * 1000:+.0f} mm from the balls", flush=True)
    tiers = _tiers(objects, c)
    if tiers != m["tiers"]:
        apex, rear = orientation(c)
        print(f"[rk] k={m['n']} tiers {tiers} (score {score(objects):.2f}); apex {apex:.2f} deg, rear row {rear:.2f} deg",
              flush=True)
        m["tiers"] = tiers
    if DEBUG and m["n"] % 100 == 0:
        err = np.linalg.norm(c[:, :2] - np.asarray(SITES), axis=1) * 1000
        print(f"[rk] k={m['n']} ball-to-site (mm): " + " ".join(f"{e:.1f}" if e < 99 else "--" for e in err)
              + f"; collected {collected(c)}/15", flush=True)
        vec = [(NAMES[i], (c[i, :2] - np.asarray(SITES[i])) * 1000) for i in range(15) if err[i] < 45]
        if vec:
            print("[rk]    site err vectors (mm): " + "; ".join(f"{n[5:]}: {v[0]:+.1f},{v[1]:+.1f}" for n, v in vec), flush=True)
    if DEBUG and m["n"] % 20 == 0 and str(m.get("label", "")).startswith("[collect]"):
        pts = robot_points()
        if pts:
            for sg, tcp in zip((1, -1), pts[:2]):
                d = np.linalg.norm(c - np.asarray(tcp), axis=1)
                i = int(d.argmin())
                if d[i] < 0.065:
                    rel = (c[i] - np.asarray(tcp)) * 1000
                    v = np.asarray(_objects_vel(objects, NAMES[i]))
                    print(f"[rk] k={m['n']} {'L' if sg == 1 else 'R'} near {NAMES[i]}: rel {rel[0]:+.1f},{rel[1]:+.1f},"
                          f"{rel[2]:+.1f} mm; v {np.linalg.norm(v[:3]) * 1000:.1f} mm/s w {np.linalg.norm(v[3:]):.2f}", flush=True)
    if DEBUG and m["n"] % 10 == 0:
        g = pair_gaps(c)
        print(f"[rk] k={m['n']} rack {np.round(kit.pose(objects, 'rack')[0][:2] * 1000, 1).tolist()} low {low * 1000:+.1f} mm "
              f"clear {clear * 1000:+.1f}; pairs ok {int((g <= GAP_OK).sum())}/30 widest {g.max() * 1000:.2f} mm; head "
              f"{_head_off(c) * 1000:.1f} mm; hands {hand_clearance(c) * 1000:+.1f} mm; moved {m['max_move'] * 1000:.2f}",
              flush=True)
        pts = robot_points()
        if pts:
            bx = kit.point(objects, "rack", (-L_IN - WALL_T / 2, 0.0, 0.0))
            rel = lambda p: np.round((np.asarray(p) - bx) * 1000, 1).tolist()
            print(f"[rk]    vs bar centre (mm): TCP L {rel(pts[0])} R {rel(pts[1])}; L pad tips {rel(pts[2])} {rel(pts[4])}; "
                  f"R pad tips {rel(pts[6])} {rel(pts[8])}; corr L {np.round(_cl()['corr'][1] * 1000, 1).tolist()} R "
                  f"{np.round(_cl()['corr'][-1] * 1000, 1).tolist()} | {m['label']}", flush=True)


kit.on_step(_on_step)


def _objects_vel(objects, name):
    st = objects[name]
    return [*np.asarray(st.lin_vel, float), *np.asarray(st.ang_vel, float)]


def _head_off(c) -> float:
    return float(math.dist(c[0][:2], (X_SPOT, Y_SPOT)))


def collected(c) -> int:
    """How many ball centres lie inside the gather ring (radius R_RING about (X_RING, 0))."""
    return int((np.hypot(c[:, 0] - X_RING, c[:, 1] - Y_SPOT) <= R_RING).sum())


def orientation(c) -> tuple:
    """(apex error, rear-row error) in degrees. Apex: the angle between the vector from the 15 ball centres' centroid
    to the head ball and +x (toward the head of the table), in [0, 180]. Rear row: the angle between the least-squares
    line through the five rear-row centres and the y axis (square to the long axis), in [0, 90]."""
    xy = c[:, :2]
    v = xy[0] - xy.mean(axis=0)
    apex = abs(math.degrees(math.atan2(v[1], v[0])))
    # The actual rear row is the five centres farthest behind the head ball along the pack's own axis, not five
    # named balls.  Using the pack axis (rather than world x) also finds that row when testing a reversed pack.
    rear_axis = xy.mean(axis=0) - xy[0]
    rear_axis /= np.linalg.norm(rear_axis)
    rear_ix = np.argsort((xy - xy[0]) @ rear_axis, kind="stable")[-5:]
    r = xy[rear_ix] - xy[rear_ix].mean(axis=0)
    d = np.linalg.svd(r)[2][0]  # principal direction of the rear-row centres
    rear = math.degrees(math.atan2(abs(d[0]), abs(d[1])))
    return apex, rear


def _oriented(c) -> bool:
    apex, rear = orientation(c)
    return apex <= APEX_TOL and rear <= REAR_TOL


def _still(objects) -> bool:
    return all(float(np.linalg.norm(np.asarray(objects[b].lin_vel, float))) < STILL for b in NAMES)


def _tiers(objects, c) -> str:
    g = pair_gaps(c)
    return "".join([
        "F" if collected(c) == 15 else format(collected(c), "x"),  # collected count (hex digit; F = all fifteen)
        "S" if _head_off(c) <= SPOT_25 else "-",
        "C" if rack_away(objects, c) and _mem()["cleared"] is not None else "-",
        "T" if (g <= GAP_OK).sum() >= 24 else "-",
        "A" if (g <= GAP_OK).all() else "-",
        "O" if _oriented(c) else "-",
        "5" if _head_off(c) <= SPOT_100 else "-",
        "H" if hand_clearance(c) >= HAND_CLEAR else "-",
        "G" if _mem()["gate"] is not None else "-",
    ])


def score(objects) -> float:
    """0 if any ball moved more than 3 mm sideways while the rack was rising (latched). Otherwise, collection first:
    with n of the 15 ball centres inside the gather ring, fewer than 15 collected scores 0.25 * n / 15. All fifteen
    collected: 0.25; 0.5 that, the scorer observed the rack enclosing all balls, rising around them and being set down
    clear, no gripper point came within 10 mm of a ball from that rise on, the rack stayed 50 mm clear after its
    set-down, and the head ball is within 10 mm of the spot; 0.75 that, at least 24 of the 30 physically closest pairs are within 0.5 mm and the pack is oriented; 1.0
    that, all 30 pass, the head is within 5 mm, every ball is still and both grippers are at least 30 mm clear.
    Tightness is read on the final state only: the hands-off rule is what rules out tightening after the rack has
    gone. Non-head identities do not select contacts or the actual rear row."""
    m = _mem()
    if m["gate"] is not None:
        return 0.0
    c = centres(objects)
    n = collected(c)
    if n < 15:
        return 0.25 * n / 15
    if (_head_off(c) > SPOT_25 or not rack_away(objects, c) or m["cleared"] is None
            or m.get("touched") is not None or m.get("rack_back") is not None):
        return 0.25
    g = pair_gaps(c)
    final_pairs = int((g <= GAP_OK).sum())
    if final_pairs < 24 or not _oriented(c):
        return 0.5
    if final_pairs == 30 and _head_off(c) <= SPOT_100 and _still(objects) and hand_clearance(c) >= HAND_CLEAR:
        return 1.0
    return 0.75


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    c = centres(objects)
    g = pair_gaps(c)
    out = np.clip(np.hypot(c[:, 0] - X_RING, c[:, 1] - Y_SPOT) - R_RING, 0, None).sum()  # uncollected shortfall
    return float(out + _head_off(c) + np.clip(g, 0, None).sum() + max(0.0, CLEAR - rack_clearance(objects, c))
                 + max(0.0, float(np.asarray(objects["rack"].low)[2]) - TOP))


# -- the scripted demo -----------------------------------------------------------------------------------------------------
# Three phases. 1) Collection: the arms pick the scattered balls off the cloth one by one (left arm the y > 0 side,
# right the y < 0 side) and build the loose cluster on its sites, rows from the apex, centre-out; only one hand is
# over the cluster at a time, the other fetches its next ball and waits at its staging hover. 2) The rack: both
# hands pinch the waiting rack's back bar, carry it over the cluster and lower it around the balls, 3.5 mm forward
# of the apex-touching pose, and let go. 3) As in the previous revision: pinch the back bar in the cusps between
# back-row balls 1-2 and 4-5, lift the rear bar about 1 mm, pull the rack back (its apex squeezes the pack tight and
# walks the head ball onto the spot; the pull is longer by the same 3.5 mm, so it ends at the same place), the last
# 3.5 mm slowly; settle; ease 1.5 mm forward; lift straight up, slowly past the equators; then carry it forward to
# its waiting spot beyond the pack, set it down, let go and raise the hands.
SP0 = D + G_LOOSE
Y_HAND = 1.5 * SP0
X_BAR0 = VX0 - L_IN - WALL_T / 2  # back-bar centre of the apex-touching rack pose (where the pull ends relative to)
TIP_Z = 0.018  # pad tips this far above the cloth (the bar is 31.75 mm tall)
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402

Z_PINCH = CLOTH_TOP + TIP_Z + PAD_TIP_PAST_TCP
YAW = {1: -math.pi / 2, -1: math.pi / 2}  # per arm (left +y, right -y): fingers close along x, across the bar
W_OPEN, W_HOLD = 0.021, WALL_T - 0.004  # 3 mm clear of each face going down
RAISE = float(os.environ.get("RK_RAISE", "0.001"))  # hands 1 mm up while pulling (rear bar up, apex on the cloth)
PULL = float(os.environ.get("RK_PULL", "0.016"))  # pull back past the apex-touching pose (the tuned tightening pull)
PULL_SLOW = 0.0035  # the last part of the pull, at about 1 mm/s
SETTLE = 3.0  # s
# The lift-disturbance gate arms when phase 2 first raises the waiting rack.  Let the last
# placed ball dissipate its residual contact motion before that independent lift begins.
# This is demo timing only; scoring thresholds and the rack/pack geometry are unchanged.
PRE_PICKUP_SETTLE = 15.0  # s
EASE = float(os.environ.get("RK_EASE", "0.0015"))  # forward ease-off before the lift: the side bars stand off 0.75 mm
LIFT_SLOW, LIFT = 0.035, 0.100  # the first 35 mm at 5 mm/s (bars past the balls' equators), then at 2 cm/s
# the rack is lowered this far forward of the apex-touching pose, so its V walls clear the head ball by ~1.8 mm and
# its back bar stays ~4.4 mm clear of the rear row on the way down; the pull is longer by the same amount
PLACE_FWD = float(os.environ.get("RK_PLACE_FWD", "0.0035"))
DRIFT = float(os.environ.get("RK_DRIFT", "0.0"))  # controls only: sideways drift over the first 20 mm of the lift
ONLY_N = int(os.environ.get("RK_ONLY_N", "0"))  # controls only: collect only the first N balls, then park
# collection geometry
W_BALL_OPEN = 0.065  # pad gap descending around a ball (3.9 mm a side; a neighbour 80 mm away stays clear)
W_BALL_HOLD = D - 0.004  # squeeze 2 mm a side to hold the ball
Z_GRASP = CLOTH_TOP + R  # TCP at the held ball's centre
Z_CARRY = TOP + 0.108  # carrying height: the held ball's bottom 76 mm up, clear of standing balls (60) and the rack
STAGE_Y = 0.21  # each arm waits here (its own side) while the other hand is over the cluster


def fw(w):
    return -w / 2


def _go(plan, dur, pos, w, label, sg, yaw=None):
    plan.append(Waypoint(float(dur), tuple(float(v) for v in pos), fw(w), label, YAW[sg] if yaw is None else yaw))


def _collection():
    """Both arms' collection waypoints, serialized over the cluster; returns {1: plan, -1: plan} (equal durations)."""
    plans = {1: [], -1: []}
    t = {1: 0.0, -1: 0.0}
    pose = {1: None, -1: None}

    def add(sg, dur, pos, w, label, yaw=0.0):
        _go(plans[sg], dur, pos, w, label, sg, yaw)
        t[sg] += dur
        pose[sg] = (pos, w, yaw)

    def hold(sg, until, label):
        if until > t[sg] + 1e-9:
            pos, w, yaw = pose[sg]
            add(sg, until - t[sg], pos, w, label, yaw)

    cluster_free = 0.0
    fill = FILL[:ONLY_N] if ONLY_N else FILL
    first = {1: True, -1: True}
    for site, arm in fill:
        sg = 1 if arm == "L" else -1
        tx, ty = SITES[SITE_IX[site]]
        sx, sy = SCATTER_FOR_SITE[site][1]
        yaw_t = math.radians(SITE_YAW[site])
        h = H_DROP if site in SITE_DROP else H_SET
        # fetch (own timeline)
        lbl = "[collect] to the first ball" if first[sg] else "[collect] to the next ball"
        add(sg, 2.6 if first[sg] else 2.0, (sx, sy, Z_CARRY), W_BALL_OPEN, lbl)
        first[sg] = False
        add(sg, 1.2, (sx, sy, Z_GRASP), W_BALL_OPEN, "[collect] down around the ball")
        add(sg, 0.6, (sx, sy, Z_GRASP), W_BALL_HOLD, "[collect] hold the ball")
        add(sg, 1.0, (sx, sy, Z_CARRY), W_BALL_HOLD, "[collect] lift it")
        add(sg, 1.6, (X_RING, sg * STAGE_Y, Z_CARRY), W_BALL_HOLD, "[collect] to the staging hover")
        # placement (cluster slot)
        start = max(t[sg], cluster_free)
        hold(sg, start, "[collect] wait for the cluster")
        add(sg, 1.2, (tx, ty, Z_CARRY), W_BALL_HOLD, "[collect] over the site", yaw_t)
        add(sg, 1.2, (tx, ty, CLOTH_TOP + R + h), W_BALL_HOLD, f"[collect] down to {h * 1000:.0f} mm", yaw_t)
        add(sg, 0.5, (tx, ty, CLOTH_TOP + R + h), W_BALL_OPEN, "[collect] let it go", yaw_t)
        add(sg, 0.8, (tx, ty, Z_CARRY), W_BALL_OPEN, "[collect] back up", yaw_t)
        cluster_free = t[sg] + 0.3
    end = max(t[1], t[-1]) + 0.4 + PRE_PICKUP_SETTLE
    for sg in (1, -1):
        hold(sg, end, "[collect] let the placed balls settle" if not ONLY_N else "[ctl-c] parked")
    return plans, end


def _rack_in(plans, sg):
    """Phase 2 for one arm: carry the waiting rack in and lower it around the cluster (equal durations per arm).
    The waiting spot (x 0.440) is reachable only low over the table, so the hands come in and leave it at x 0.40 and
    stay below 55 mm while out there; every ball is already in the cluster, so the low pass crosses empty cloth."""
    y = sg * Y_HAND
    xl = X_BAR0 + PLACE_FWD
    zlow = TOP + 0.055  # bar bottom 21 mm over empty cloth
    p = plans[sg]
    _go(p, 2.0, (0.40, sg * 0.17, Z_CARRY), W_OPEN, "[rack] around the cluster, high", sg)
    _go(p, 1.4, (0.40, y, zlow), W_OPEN, "[rack] low, short of the waiting rack", sg)
    _go(p, 1.2, (X_BAR_START, y, zlow), W_OPEN, "[rack] out over its back bar", sg)
    _go(p, 1.2, (X_BAR_START, y, Z_PINCH), W_OPEN, "[rack] down astride the bar", sg)
    _go(p, 1.0, (X_BAR_START, y, Z_PINCH), W_HOLD, "[rack] pinch the bar", sg)
    _go(p, 1.0, (X_BAR_START, y, zlow), W_HOLD, "[rack] raise it a little", sg)
    _go(p, 1.4, (0.40, y, zlow), W_HOLD, "[rack] bring it to x 0.40", sg)
    _go(p, 1.6, (0.40, y, Z_PINCH + LIFT), W_HOLD, "[rack] up to carrying height", sg)
    _go(p, 5.5, (xl, y, Z_PINCH + LIFT), W_HOLD, "[rack] carry it over the cluster", sg)
    _go(p, 2.2, (xl, y, Z_PINCH + 0.030), W_HOLD, "[rack] lower it", sg)
    _go(p, 5.0, (xl, y, Z_PINCH + 0.0005), W_HOLD, "[rack] down around the balls, slowly", sg)
    _go(p, 0.8, (xl, y, Z_PINCH + 0.0005), W_OPEN, "[rack] let go of the bar", sg)
    _go(p, 1.6, (xl, y, Z_PINCH + 0.10), W_OPEN, "[rack] hands up off the bar", sg)


def _arm(sg, pull=PULL):
    y = sg * Y_HAND
    x0 = X_BAR0 + PLACE_FWD  # the rack stands where phase 2 lowered it
    zr = Z_PINCH + RAISE
    p = []
    _go(p, 3.0, (x0, y, Z_PINCH + 0.10), W_OPEN, "over the back bar", sg)
    _go(p, 2.0, (x0, y, Z_PINCH), W_OPEN, "down astride the back bar", sg)
    _go(p, 1.0, (x0, y, Z_PINCH), W_HOLD, "pinch the back bar", sg)
    _go(p, 0.5, (x0, y, zr), W_HOLD, "lift the rear bar 1 mm", sg)
    if pull > 0:
        pull = pull + PLACE_FWD  # the extra placement offset is taken up in the fast stage; the end point is the same
        fast = pull - PULL_SLOW
        _go(p, fast / 0.003, (x0 - fast, y, zr), W_HOLD, "[tighten] pull the rack back: its apex squeezes the pack", sg)
        _go(p, PULL_SLOW / 0.001, (x0 - pull, y, zr), W_HOLD, "[tighten] the last 3.5 mm slowly", sg)
    _go(p, SETTLE, (x0 - pull, y, zr), W_HOLD, "let the pack settle", sg)
    xl = x0 - pull + EASE
    _go(p, 2.5, (xl, y, zr), W_HOLD, "ease the rack 1.5 mm forward, bars off the balls", sg)
    if DRIFT:
        _go(p, 1.0, (xl, y + DRIFT, Z_PINCH + 0.020), W_HOLD, "[ctl-a] lift with sideways drift", sg)
        _go(p, (LIFT_SLOW - 0.020) / 0.005, (xl, y + DRIFT, Z_PINCH + LIFT_SLOW), W_HOLD, "[lift] lift slowly past the equators", sg)
    else:
        _go(p, LIFT_SLOW / 0.005, (xl, y, Z_PINCH + LIFT_SLOW), W_HOLD, "[lift] lift slowly past the equators", sg)
    _go(p, (LIFT - LIFT_SLOW) / 0.02, (xl + 0, p[-1].pos[1], Z_PINCH + LIFT), W_HOLD, "[lift] lift it clear", sg)
    xs = X_BAR_START
    _go(p, 4.5, (0.40, y, Z_PINCH + LIFT), W_HOLD, "[carry] carry the rack forward, over and past the pack", sg)
    _go(p, 1.8, (0.40, y, TOP + 0.055), W_HOLD, "[carry] down low over empty cloth", sg)
    _go(p, 1.4, (xs, y, TOP + 0.055), W_HOLD, "[carry] out to its waiting spot", sg)
    _go(p, 1.4, (xs, y, Z_PINCH + 0.0015), W_HOLD, "[set down] set it down there", sg)
    _go(p, 1.0, (xs, y, Z_PINCH + 0.0015), W_OPEN, "let go", sg)
    _go(p, 1.2, (xs, y, TOP + 0.055), W_OPEN, "hands off the bar", sg)
    _go(p, 1.5, (0.40, y, TOP + 0.098), W_OPEN, "hands up", sg)
    return p


# The demo may read sim state (the Guide; v21): each hand's target is corrected live by an integrator on the measured
# TCP error in x and z (the arms sit ~5 mm toward the robot when low over the table), from the descent onward.
CL_GAIN, CL_MAX = 0.3, 0.012
CL_KEY = "rk_cl"


def _cl() -> dict:
    return mk.MEMORY.setdefault(CL_KEY, {"corr": {1: np.zeros(3), -1: np.zeros(3)}, "target": {}, "label": {},
                                         "on": {1: False, -1: False}})


def _patch_arm_plan() -> None:
    from yam_arena import demo as _demo

    cls = _demo.ArmPlan
    if getattr(cls, "_rk_cl", False):
        return
    target0 = cls.target

    def target(self, t):
        pos, rot, finger, label = target0(self, t)
        if mk._EP.get("env") is None:
            return pos, rot, finger, label
        sg = 1 if self.base_pos[1] > 0 else -1
        cl = _cl()
        cl["target"][sg] = np.asarray(pos, float).copy()
        cl["label"][sg] = label
        # the correction keeps applying but stops UPDATING while a ball is being released and the hand backs off:
        # a correction step at those moments jerks the just-opened pads into the placed ball
        cl["on"][sg] = label not in ("[collect] to the first ball", "[collect] let it go", "[collect] back up")
        if sg == 1:
            _mem()["label"] = label
        return pos + cl["corr"][sg], rot, finger, label

    cls.target = target
    cls._rk_cl = True


_patch_arm_plan()


def _cl_step(_objects) -> None:
    if mk._EP.get("env") is None:
        return
    cl = _cl()
    t = mk.tcps()
    for sg, tcp in zip((1, -1), t):
        if sg in cl["target"] and cl["on"][sg]:
            err = cl["target"][sg] - tcp
            if not str(cl["label"].get(sg, "")).startswith("[collect]"):
                err[1] = 0.0  # rack phases: x and z only (along the bar a few mm do not matter; a y loop once
                # pushed the rack sideways during the pull); placing balls, y accuracy matters, so it stays on
            cl["corr"][sg] = np.clip(cl["corr"][sg] + CL_GAIN * err, -CL_MAX, CL_MAX)


kit.on_step(_cl_step)


def _full(pull=PULL):
    plans, _ = _collection()
    if not ONLY_N:
        for sg in (1, -1):
            _rack_in(plans, sg)
            plans[sg] += _arm(sg, pull=pull)
    return {"left": plans[1], "right": plans[-1]}


def demo():
    """Collect the fifteen scattered balls into the loose cluster (one hand over the cluster at a time); carry the
    waiting rack in and lower it around them; then as before: pinch the back bar between back-row balls, lift it
    about 1 mm, pull back to the tuned end point (12.5 mm at 3 mm/s, the last 3.5 mm at 1 mm/s) to tighten, settle
    3 s, ease 1.5 mm forward, lift straight up (35 mm at 5 mm/s, then to 100 mm at 2 cm/s), carry it forward to its
    waiting spot beyond the pack, set it down, let go and raise the hands. With RK_NO_PULL set (control b, run with
    RK_START_PAST=0) the pull is skipped and the rack is lifted straight off the loose pack."""
    return _full(pull=0.0 if os.environ.get("RK_NO_PULL") else PULL)


def controls_no_tighten():
    """Control (b): the whole demo with no tightening pull (run with RK_START_PAST=0 so the cluster is built with the
    head ball on the spot): the rack is lowered, pinched and lifted straight off, and the pack keeps its 2 mm gaps."""
    return _full(pull=0.0)


# -- offline self-test --------------------------------------------------------------------------------------------------
def _states(ball_xy, rack_shift=(0.0, 0.0, 0.0), moving=False, rack_moving=False):
    import dataclasses

    from yam_arena.task_file import ObjectState

    pos0, q0 = kit.pose(kit.fake_states(), "rack")
    st = kit.fake_states(moved={"rack": (np.asarray(pos0) + np.asarray(rack_shift), q0)})
    if rack_moving:
        st["rack"] = dataclasses.replace(st["rack"], lin_vel=np.array([-0.02, 0.0, 0.0]))
    z = np.zeros(3)
    for name, (x, y) in zip(NAMES, ball_xy):
        p = np.array([x, y, CLOTH_TOP + R])
        st[name] = ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]),
                               lin_vel=np.array([0.02, 0, 0]) if moving else z, ang_vel=z, low=p - R, high=p + R)
    return st


def self_test():
    global _TEST_POINTS
    tight = [p for row in lattice(X_SPOT, Y_SPOT, D) for p in row]
    tight_off7 = [p for row in lattice(X_SPOT + 0.007, Y_SPOT, D) for p in row]
    loose_on = [p for row in lattice(X_SPOT, Y_SPOT, D + G_LOOSE) for p in row]
    some = [list(p) for p in tight]
    for k in range(10, 15):  # back row pulled 1 mm back: 5 pairs of the back row + 4 diagonals... count below
        some[k][0] -= 0.001
    partial5 = [list(p) for p in START]
    for site, _arm_ in FILL[:5]:  # the first five balls collected to their sites, the rest still scattered
        partial5[SITE_IX[site]] = list(SITES[SITE_IX[site]])
    # rack shifts from its default (waiting) pose, which is also where the demo finally sets it down
    on_pack = (VX0 - PULL - VX_START, 0.0, 0.0)  # lowered around the pack, at the pull's end point
    up = (on_pack[0], 0.0, 0.10)  # lifted 100 mm straight up: above the pack, not taken away
    away = (0.0, 0.0, 0.0)
    near = (-0.0529, 0.0, 0.0)  # set down with its bar face 6 mm from the head ball (too close)
    removed = {"enclosed": 10, "cleared": 20, "lift_ready": True, "rise_pairs": 30,
               "rise_head": 0.0, "rise_oriented": True}
    removed_loose = {"enclosed": 10, "cleared": 20, "lift_ready": True, "rise_pairs": 20,
                     "rise_head": 0.0, "rise_oriented": True}

    # A legal alternative arrangement: ball 1 remains the head ball, while non-head identities are permuted across
    # rows.  Its geometry, tightness and orientation must be identical.
    permuted = list(tight)
    for a, b in ((1, 11), (2, 7), (5, 14), (8, 10)):
        permuted[a], permuted[b] = permuted[b], permuted[a]

    def turned(deg):  # the tight pack rotated about the head ball, which stays on the spot
        a = math.radians(deg)
        return [(X_SPOT + (x - X_SPOT) * math.cos(a) - (y - Y_SPOT) * math.sin(a),
                 Y_SPOT + (x - X_SPOT) * math.sin(a) + (y - Y_SPOT) * math.cos(a)) for x, y in tight]
    FRAC, ORIENT = "collected fraction", "0.5 if all collected"
    cases = {
        "start (15 scattered outside the ring, rack waiting)": (_states(START), {}, [], 0.0),
        "five collected to their sites, ten still scattered": (_states(partial5), {}, [], 0.25 * 5 / 15),
        "cluster complete (loose, 15 mm past the spot), rack still waiting": (_states(SITES), {}, [], 0.25),
        "tight on the spot, rack still on it": (_states(tight, on_pack), {}, [], 0.25),
        "tight on the spot, rack lifted 100 mm straight up (not taken away)": (_states(tight, up), {}, [], 0.25),
        "tight on the spot, rack carried away but still 60 mm up": (_states(tight, (0.0, 0.0, 0.06)), {}, [], 0.25),
        "tight on the spot, rack set down 6 mm from the head ball": (_states(tight, near), {}, [], 0.25),
        "tight on the spot, rack at its spot but still sliding": (_states(tight, away, rack_moving=True), {}, [], 0.25),
        "perfect pack beside untouched rack cannot claim removal": (_states(tight, away), {}, [], 0.25),
        "loose on the spot after a valid rack removal": (_states(loose_on, away), removed, [], 0.5),
        "tight on the spot after a valid rack removal": (_states(tight, away), removed, [], 1.0),
        "tight after a valid removal, hands came within 10 mm of a ball after the rise (tightened by hand)": (
            _states(tight, away), {**removed, "touched": 25}, [], 0.25),
        "rack set down clear, then brought back to push the pack tight and set down clear again": (
            _states(tight, away), {**removed, "rack_back": 30}, [], 0.25),
        "loose at the rise, tight finally, hands kept off (rack-free tightening is impossible)": (
            _states(tight, away), removed_loose, [], 1.0),
        "legal non-head permutation after a valid rack removal": (_states(permuted, away), removed, [], 1.0),
        "tight, head 7 mm off the spot, rack away": (_states(tight_off7, away), removed, [], 0.75),
        "tight, rack away, balls rolling": (_states(tight, away, moving=True), removed, [], 0.75),
        "tight, rack away, a pad touching the head ball": (_states(tight, away), removed, [np.array([X_SPOT + R + 0.005, 0, CLOTH_TOP + R])], 0.75),
        "tight but the lift disturbed it (latched)": (_states(tight, away), {"gate": 5}, [], 0.0),
        "back row 1 mm off, rack away": (_states(some, away), removed, [], None),
        "orientation: correct pack, rotated 2 deg (inside both limits)": (_states(turned(2.0), away), removed, [], 1.0),
        "orientation: rotated 180 deg, head ball on the spot (apex faces the foot)": (_states(turned(180.0), away), removed, [], FRAC),
        "orientation: rotated 10 deg, head ball on the spot": (_states(turned(10.0), away), removed, [], ORIENT),
        "orientation: rotated 4 deg (apex inside 5, rear row outside 3)": (_states(turned(4.0), away), removed, [], ORIENT),
    }
    try:
        for name, (st, flags, pts, want) in cases.items():
            mk.new_episode()
            _mem().update(flags)
            _TEST_POINTS = pts
            got = score(st)
            if want is None:
                g = pair_gaps(centres(st))
                want = 0.75 if (g <= GAP_OK).sum() >= 24 else 0.5
                print(f"   ({name}: {int((g <= GAP_OK).sum())}/30 pairs tight -> {want})", file=sys.stderr)
            elif want in (FRAC, ORIENT):
                # a turned pack swings its rear corners toward the ring's edge: if a corner leaves the ring the
                # score is collection credit; still inside, the orientation check stops it at 0.5
                n = collected(centres(st))
                if want == FRAC:
                    assert n < 15, (name, n)
                want = 0.5 if n == 15 else 0.25 * n / 15
                print(f"   ({name}: {n}/15 inside the ring -> {want:.3f})", file=sys.stderr)
            assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0 - 1e-9), (name, got, want)
        g = pair_gaps(np.array([[x, y, 0] for x, y in tight]))
        assert np.abs(g).max() < 1e-9
        assert np.allclose(pair_gaps(np.array([[x, y, 0] for x, y in tight])),
                           pair_gaps(np.array([[x, y, 0] for x, y in permuted])))
        assert np.allclose(orientation(np.array([[x, y, 0] for x, y in tight])),
                           orientation(np.array([[x, y, 0] for x, y in permuted])))
        # Whole rack-use sequences: (lowest point m, ball shift m, away, still, valid tight enclosure) per step.
        # Expected tuple is (latched, cleared).
        c0 = np.array([[x, y, CLOTH_TOP + R] for x, y in tight])
        seqs = {
            "valid enclosure, rise, ball moves 4 mm": (
                [(0.0, 0, 0, 1, 1), (0.003, 0, 0, 0, 1), (0.02, 0.004, 0, 0, 0)], (True, False)),
            "valid enclosure, clean rise, set down away": (
                [(0.0, 0, 0, 1, 1), (0.003, 0, 0, 0, 1), (0.05, 0.001, 0, 0, 0),
                 (-0.003, 0.001, 1, 1, 0)], (False, True)),
            "valid rise, flat touch on empty near cloth, then slide to clear": (
                [(0.0, 0, 0, 1, 1), (0.003, 0, 0, 0, 1), (0.05, 0.001, 0, 0, 0),
                 (0.0, 0.001, 0, 1, 0), (0.0, 0.001, 1, 1, 0)], (False, True)),
            "empty rack lift then final pack beside it cannot count": (
                [(0.003, 0, 0, 0, 0), (0.05, 0, 0, 0, 0), (-0.003, 0, 1, 1, 0)], (False, False)),
            "old loose enclosure, rack removed low, hand-packed balls, empty rack lifted cannot count": (
                [(0.0, 0, 0, 1, 1), (0.0, 0, 1, 0, 0), (0.0, 0, 1, 1, 0),
                 (0.003, 0, 0, 0, 0), (-0.003, 0, 1, 1, 0)], (False, False)),
            "enclosure lost in the 0.5--2 mm deadband cannot start a later rise": (
                [(0.0, 0, 0, 1, 1), (0.001, 0, 0, 0, 0), (0.003, 0, 0, 0, 1),
                 (-0.003, 0, 1, 1, 0)], (False, False)),
            "rise, back flat on the cloth, a ball moves 4 mm, rise again": (
                [(0.0, 0, 0, 1, 1), (0.003, 0, 0, 0, 1), (0.0, 0, 0, 1, 1),
                 (0.0, 0.004, 0, 1, 1), (0.003, 0.004, 0, 0, 1), (0.05, 0.0045, 0, 0, 0)], (False, False)),
            "rise, back down but still moving (not re-armed), a ball moves 4 mm": (
                [(0.0, 0, 0, 1, 1), (0.003, 0, 0, 0, 1), (0.0, 0, 0, 0, 0),
                 (0.0, 0.004, 0, 0, 0)], (True, False)),
            "set down away, then a ball moves 4 mm (gate over)": (
                [(0.0, 0, 0, 1, 1), (0.003, 0, 0, 0, 1), (-0.003, 0, 1, 1, 0),
                 (-0.003, 0.004, 1, 1, 0)], (False, True)),
        }
        for name, (steps, expected) in seqs.items():
            m = {"n": 0, "ref": None, "gate": None, "max_move": 0.0, "enclosed": None,
                 "cleared": None, "lift_ready": False, "rise_pairs": 0, "rise_head": float("inf"),
                 "rise_oriented": False}
            for low_s, dx, away_s, still_s, enclosed_s in steps:
                m["n"] += 1
                cc = c0.copy()
                cc[7, 0] += dx  # ball 7 (fourth row)
                _gate_step(m, cc, low_s, bool(away_s), bool(still_s), bool(enclosed_s))
            got = (m["gate"] is not None, m["cleared"] is not None)
            assert got == expected, (name, got, expected, m)
        print(f"lift gate: {len(seqs)} sequences pass", file=sys.stderr)
        # Hands-off sequences: (rack lowest point m, away, still, valid enclosure, gripper-to-ball clearance m) per
        # step; expected: (hands-off broken, removal completed) at the end.
        hseqs = {
            "racking touches balls before the rise (allowed)": (
                [(0.0, 0, 1, 1, 0.0), (0.0, 0, 1, 1, 0.002), (0.003, 0, 0, 1, 0.013), (-0.003, 1, 1, 0, 0.10)],
                (False, True)),
            "pads 12 mm from the pack while lifting, then withdrawn (the demo)": (
                [(0.0, 0, 1, 1, 0.011), (0.003, 0, 0, 1, 0.012), (0.05, 0, 0, 0, 0.03), (-0.003, 1, 1, 0, 0.10)],
                (False, True)),
            "a pad pokes the pack while the rack is in the air": (
                [(0.0, 0, 1, 1, 0.011), (0.003, 0, 0, 1, 0.012), (0.05, 0, 0, 0, 0.004), (-0.003, 1, 1, 0, 0.10)],
                (True, True)),
            "rack set down clear, then a hand nudges the pack tighter": (
                [(0.0, 0, 1, 1, 0.011), (0.003, 0, 0, 1, 0.012), (-0.003, 1, 1, 0, 0.10), (-0.003, 1, 1, 0, 0.0)],
                (True, True)),
            "a touch after the rise, then a full retry with the rack flat around the pack": (
                [(0.0, 0, 1, 1, 0.011), (0.003, 0, 0, 1, 0.012), (0.01, 0, 0, 1, 0.003), (0.0, 0, 1, 1, 0.0),
                 (0.0, 0, 1, 1, 0.011), (0.003, 0, 0, 1, 0.013), (-0.003, 1, 1, 0, 0.10)], (False, True)),
        }
        for name, (steps, expected) in hseqs.items():
            m = {"n": 0, "ref": None, "gate": None, "max_move": 0.0, "enclosed": None, "cleared": None,
                 "lift_ready": False, "rise_pairs": 0, "rise_head": float("inf"), "rise_oriented": False,
                 "touched": None}
            for low_s, away_s, still_s, enclosed_s, hc in steps:
                m["n"] += 1
                _gate_step(m, c0.copy(), low_s, bool(away_s), bool(still_s), bool(enclosed_s))
                _hands_step(m, hc)
            got = (m["touched"] is not None, m["cleared"] is not None)
            assert got == expected, (name, got, expected, m)
        print(f"hands off: {len(hseqs)} sequences pass", file=sys.stderr)
        # Rack-stays-away sequences: (lowest point m, away, still, valid enclosure, rack clearance m); expected:
        # (rack came back, removal completed).
        rseqs = {
            "set down 61 mm clear and left there (the demo)": (
                [(0.0, 0, 1, 1, 0.0), (0.003, 0, 0, 1, 0.0), (0.05, 0, 0, 0, 0.02), (-0.003, 1, 1, 0, 0.061),
                 (-0.003, 1, 1, 0, 0.062)], (False, True)),
            "set down clear, picked up again and pushed against the pack, set down clear again": (
                [(0.0, 0, 1, 1, 0.0), (0.003, 0, 0, 1, 0.0), (-0.003, 1, 1, 0, 0.061), (0.01, 0, 0, 0, 0.0),
                 (-0.003, 1, 1, 0, 0.061)], (True, True)),
            "rack passes over the pack before its set-down (carry-out, allowed)": (
                [(0.0, 0, 1, 1, 0.0), (0.003, 0, 0, 1, 0.0), (0.08, 0, 0, 0, 0.0), (-0.003, 1, 1, 0, 0.061)],
                (False, True)),
        }
        for name, (steps, expected) in rseqs.items():
            m = {"n": 0, "ref": None, "gate": None, "max_move": 0.0, "enclosed": None, "cleared": None,
                 "lift_ready": False, "rise_pairs": 0, "rise_head": float("inf"), "rise_oriented": False,
                 "touched": None, "rack_back": None}
            for low_s, away_s, still_s, enclosed_s, cl in steps:
                m["n"] += 1
                _gate_step(m, c0.copy(), low_s, bool(away_s), bool(still_s), bool(enclosed_s))
                _rack_back_step(m, cl)
            got = (m["rack_back"] is not None, m["cleared"] is not None)
            assert got == expected, (name, got, expected, m)
        print(f"rack stays away: {len(rseqs)} sequences pass", file=sys.stderr)
        for deg, want in ((0.0, (0.0, 0.0)), (180.0, (180.0, 0.0)), (10.0, (10.0, 10.0)), (-4.0, (4.0, 4.0))):
            o = orientation(np.array([[x, y, 0] for x, y in turned(deg)]))
            assert abs(o[0] - want[0]) < 1e-6 and abs(o[1] - want[1]) < 1e-6, (deg, o)
            assert np.abs(pair_gaps(np.array([[x, y, 0] for x, y in turned(deg)]))).max() < 1e-9  # still tight
            print(f"   (pack turned {deg:+.0f} deg about the head ball: apex {o[0]:.2f} deg, rear row {o[1]:.2f} deg, "
                  f"oriented {_oriented(np.array([[x, y, 0] for x, y in turned(deg)]))})", file=sys.stderr)
        assert len(pair_gaps(c0)) == 30
        # scatter and ring geometry
        assert all(p is not None for p in START)
        sc = np.array(START)
        d_ring = np.hypot(sc[:, 0] - X_RING, sc[:, 1] - Y_SPOT)
        pair_d = min(np.hypot(*(sc[i] - sc[j])) for i in range(15) for j in range(i + 1, 15))
        print(f"scatter: 15 balls, min spacing {pair_d * 1000:.0f} mm, nearest {d_ring.min() * 1000:.0f} mm from the "
              f"ring centre (ring r {R_RING * 1000:.0f} + 20 mm clear zone)", file=sys.stderr)
        assert pair_d >= 0.080 - 1e-9 and d_ring.min() >= R_RING + 0.020 - 1e-9
        cx, cy = X_SPOT - 0.03, 0.0  # the cloth
        assert np.all(sc[:, 0] >= cx - 0.28 + R) and np.all(sc[:, 0] <= cx + 0.28 - R)
        assert np.all(np.abs(sc[:, 1]) <= 0.31 - R)  # every scattered ball fully on the cloth
        assert np.all(sc[:, 0] + R <= X_BAR_START - WALL_T / 2 - 0.010)  # clear of the waiting rack
        for pts, lbl in ((SITES, "loose"), (tight, "tight")):
            dd = np.hypot(np.array(pts)[:, 0] - X_RING, np.array(pts)[:, 1] - Y_SPOT)
            assert dd.max() <= R_RING - 0.005, (lbl, dd.max())  # the whole cluster/pack inside the ring
        st = _states(SITES)
        print(f"cluster built: head ball {_head_off(centres(st)) * 1000:.1f} mm past the spot, widest gap "
              f"{pair_gaps(centres(st)).max() * 1000:.2f} mm; rack lowering pose back-bar x {X_BAR0 + PLACE_FWD:.4f} "
              f"(apex-touching {X_BAR0:.4f} + {PLACE_FWD * 1000:.1f} mm forward)", file=sys.stderr)
        sl = _states(SITES, (VX0 + PLACE_FWD - VX_START, 0.0, 0.0))  # rack lowered around the loose cluster
        assert rack_clearance(sl, centres(sl)) > 1e-4, rack_clearance(sl, centres(sl))  # bars clear of every ball
        assert rack_encloses(sl, centres(sl)), "the lowered rack must contain all fifteen loose-cluster balls"
        sa = _states(tight, away)
        print(f"rack at its waiting spot: {rack_clearance(sa, centres(sa)) * 1000:.1f} mm horizontally from the tight "
              f"pack, lowest point {(float(sa['rack'].low[2]) - TOP) * 1000:.1f} mm above the table, rack x "
              f"{float(sa['rack'].low[0]):.3f}..{float(sa['rack'].high[0]):.3f}", file=sys.stderr)
        assert rack_clearance(sa, centres(sa)) >= CLEAR + 0.005
        th = _states([p for row in lattice(X_SPOT - 0.0, Y_SPOT, D) for p in row], away)  # head DEAD on the spot
        assert rack_clearance(th, centres(th)) >= CLEAR + 0.005  # a perfect rack still clears the waiting spot
        assert float(sa["rack"].low[0]) > TABLE_X0 + 0.005 and float(sa["rack"].high[0]) < 0.36 + 0.55 - 0.005
    finally:
        mk.new_episode()
        _TEST_POINTS = None
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan

    print(f"scoring: {self_test()} cases pass")
    hz = 20.0
    for kind, fn in (("demo", demo), ("control b (not the demo)", controls_no_tighten)):
        for arm, waypoints in fn().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (kind, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (kind, arm)
            print(f"{kind} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
