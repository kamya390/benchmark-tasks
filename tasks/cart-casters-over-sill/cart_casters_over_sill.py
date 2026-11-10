"""On the station table, in front of the right arm, a small platform cart stands on the floor (the table) facing a
loading bay: a steel platform 10 mm high whose near edge is a square sill, painted yellow, with a back stop at its
far end. The cart has two large rear wheels (56 mm across) and two small front casters (15 mm across), a deck with
a cardboard carton on it, and a push handle on two posts at the rear. Its casters stand 3 cm short of the sill.
The job is to get the cart into the bay, up against the back stop, with the carton still on it.

Physics (mechkit.py, in this folder, adds the axles and the handle's swivel to the stock runtime in memory): every
wheel is a 12-sided disc on a free axle. A 10 mm step is higher than a caster's 7 mm radius, so the casters cannot
roll up it, however hard the cart is pushed: they stop dead against it, and a harder push pitches the cart over
them. The rear wheels (27 mm radius) do climb it under a push of about 2 N. So the casters have to be lifted onto
the platform first, by tipping the cart back on its rear wheels (or lifting its front), and the rear wheels then
driven up the sill. Lifting the whole cart clear of the ground is not allowed.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/cart-casters-over-sill/cart_casters_over_sill.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (installs the axle/swivel hooks and the per-trial memory mk.MEMORY)

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, ObjectState  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
INF = math.inf

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
CY = -0.100  # the cart's and the bay's centre line
X_SILL = 0.320  # the platform's near edge (the sill)
SILL_H = 0.010
X_STOP = 0.470  # the back stop's face
PLAT_TOP = TOP + SILL_H
R_A = 0.027  # rear wheel apothem (12-gon, 56 mm across the corners)
C_A = 0.007  # caster apothem
WHEELBASE = 0.080
REAR_Y, CASTER_Y = 0.062, 0.035
DECK = (-0.040, 0.100, 0.050, 0.036, 0.044)  # deck x0, x1 (from the rear axle), half-width, bottom, top
POST_X = -0.036  # the handle posts, from the rear axle
HANDLE_Z = 0.110  # the handle's swivel axis above the floor
CARTON = (0.070, 0.070, 0.050)
CARTON_X = 0.035  # the carton's centre, ahead of the rear axle
CARTON_KG = 0.060
X0 = X_SILL - 0.030 - WHEELBASE - C_A  # the rear axle's start: casters 3 cm short of the sill

LIFT_MM = 0.004  # a wheel counts as off the ground 4 mm above the surface under it
TIP_PITCH, TIP_ROLL = 25.0, 15.0
LOAD_DZ = 0.015  # the carton counts as lost 15 mm off the deck, or with its centre off the deck outline
AT_STOP = 0.010  # the deck's front within 10 mm of the stop
MAX_YAW = 10.0

BLUE = (0.20, 0.35, 0.70)
RUBBER = (0.12, 0.12, 0.12)
STEEL = (0.55, 0.57, 0.60)
YELLOW = (0.95, 0.80, 0.10)
KRAFT = (0.62, 0.46, 0.30)
GRIP = (0.85, 0.30, 0.10)

kit = mk.Kit()


def _wheel(a, width):
    """A 12-sided disc about y: six bars of the disc's width turned in 30 deg steps (their ends are the faces)."""
    s = 2 * a * math.tan(math.radians(15.0))
    return [mk.box((2 * a, width, s), quat=(0.0, math.sin(math.radians(15.0 * k)), 0.0, math.cos(math.radians(15.0 * k))))
            for k in range(6)]


_deck_boxes = [
    mk.box((DECK[1] - DECK[0], 2 * DECK[2], DECK[4] - DECK[3]), at=((DECK[0] + DECK[1]) / 2, 0.0, (DECK[3] + DECK[4]) / 2)),
    *(mk.box((0.008, 0.008, HANDLE_Z - 0.006 - DECK[4]), at=(POST_X, sy * 0.045, (HANDLE_Z - 0.006 + DECK[4]) / 2))
      for sy in (-1, 1)),
    *(mk.box((0.012, 0.011, 0.012), at=(0.0, sy * 0.0505, 0.030)) for sy in (-1, 1)),  # rear axle brackets
    *(mk.box((0.012, 0.003, 0.029), at=(WHEELBASE, sy * CASTER_Y + side * 0.0055, 0.0215))  # caster forks
      for sy in (-1, 1) for side in (-1, 1)),
]
kit.part("cart", (X0, CY, TOP), _deck_boxes, mass=0.130, color=BLUE)
kit.part("handle", (X0 + POST_X, CY, TOP + HANDLE_Z), [mk.box((0.012, 0.110, 0.012))], mass=0.020, color=GRIP)
kit.hinge("swivel", "cart", "handle", anchor=(X0 + POST_X, CY, TOP + HANDLE_Z), axis=(0, 1, 0), lower_deg=-45.0,
          upper_deg=45.0, friction=0.004)
WHEELS = {}
for sy, tag in ((1, "l"), (-1, "r")):
    for kind, dx, dy, a, w, kg in (("rear", 0.0, REAR_Y, R_A, 0.010, 0.012), ("caster", WHEELBASE, CASTER_Y, C_A, 0.008, 0.004)):
        name = f"{kind}_{tag}"
        at = (X0 + dx, CY + sy * dy, TOP + a)
        kit.part(name, at, _wheel(a, w), mass=kg, color=RUBBER)
        kit.hinge(f"axle_{name}", "cart", name, anchor=at, axis=(0, 1, 0), lower_deg=-INF, upper_deg=INF)
        WHEELS[name] = kind
CART_PARTS = ("cart", "handle", *WHEELS)

FIXTURES = [
    Box("sill_nosing", pos=(X_SILL + 0.006, CY, TOP + SILL_H / 2), size=(0.012, 0.160, SILL_H), color=YELLOW),
    Box("bay_floor", pos=((X_SILL + 0.012 + X_STOP) / 2, CY, TOP + SILL_H / 2), size=(X_STOP - X_SILL - 0.012, 0.160, SILL_H),
        color=STEEL),
    Box("back_stop", pos=(X_STOP + 0.006, CY, TOP + 0.030), size=(0.012, 0.160, 0.060), color=STEEL),
]
_carton0 = (X0 + CARTON_X, CY, TOP + DECK[4] + CARTON[2] / 2)
OBJECTS = [*kit.objects(), Box("carton", pos=_carton0, size=CARTON, color=KRAFT, mass=CARTON_KG), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- readouts (all from the parts' poses) -----------------------------------------------------------------------

_TEST_TCPS = None  # self-test override for the grippers' TCPs


def _released(st, margin=0.025) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def _axes(objects):
    pos, q = kit.pose(objects, "cart")
    r = mk.quat_to_mat(q)
    return pos, r


def attitude(objects):
    """(pitch, roll, yaw) of the deck in degrees; pitch > 0 is nose up."""
    _, r = _axes(objects)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(r[2, 0])))))
    roll = math.degrees(math.asin(max(-1.0, min(1.0, float(r[2, 1])))))
    yaw = math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0])))
    return pitch, roll, yaw


def _ground(x) -> float:
    """The surface height under a point at x (the floor, or the bay platform past the sill)."""
    return PLAT_TOP if X_SILL <= x <= X_STOP else TOP


def _axle(objects, name):
    """The wheel's axle centre (its part origin) and its apothem: a 12-gon's lowest point is between the two, within
    1 mm, whatever its turn (the bounding box of a turned wheel reads up to 10 mm low)."""
    pos, _ = kit.pose(objects, name)
    return pos, (R_A if WHEELS[name] == "rear" else C_A)


def _wheel_up(objects, name) -> float:
    """How far the wheel's rim is above the surface under its centre (m)."""
    pos, a = _axle(objects, name)
    return float(pos[2]) - a - _ground(float(pos[0]))


def _on_platform(objects, name) -> bool:
    """The wheel's centre at least 5 mm past the sill and its rim on the platform (within 4 mm)."""
    pos, a = _axle(objects, name)
    return bool(float(pos[0]) >= X_SILL + 0.005 and abs(float(pos[2]) - a - PLAT_TOP) <= LIFT_MM)


def _carton_on(objects) -> bool:
    """The carton's centre over the deck outline and its bottom within 15 mm of the deck top, level within 20 deg."""
    pos, r = _axes(objects)
    c = objects["carton"]
    local = r.T @ (np.asarray(c.pos, float) - pos)
    deck_top = kit.point(objects, "cart", (float(local[0]), float(local[1]), DECK[4]))[2]
    x, y, z, w = (float(v) for v in c.quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    return bool(DECK[0] <= local[0] <= DECK[1] and abs(local[1]) <= DECK[2]
                and abs(float(c.low[2]) - deck_top) <= LOAD_DZ and tilt < 20.0)


def front_gap(objects) -> float:
    """The deck's front face to the back stop (m), at its nearer corner."""
    return float(min(X_STOP - kit.point(objects, "cart", (DECK[1], sy * DECK[2], DECK[4]))[0] for sy in (-1, 1)))


def _still(objects) -> bool:
    """Nothing moving: from one control step to the next the deck, handle and carton move under 0.3 mm and turn
    under 0.5 deg (the free wheels' own spin is not counted)."""
    names = ("cart", "handle")
    now = {n: kit.pose(objects, n) for n in names}
    c = objects["carton"]
    now["carton"] = (np.asarray(c.pos, float), np.asarray(c.quat_xyzw, float))
    frame = mk._EP.get("step") if mk._EP.get("env") is not None else id(objects)
    prev = mk.MEMORY.get("cart_prev")
    if prev is not None and prev[0] == frame:
        prev = mk.MEMORY.get("cart_prev_before")
    else:
        mk.MEMORY["cart_prev_before"] = prev
        mk.MEMORY["cart_prev"] = (frame, now)
    if prev is None:
        return False
    for n, (p, q) in now.items():
        p0, q0 = prev[1][n]
        turn = 2 * math.degrees(math.acos(min(1.0, abs(float(np.dot(q, q0))))))
        if np.linalg.norm(p - p0) > 0.0003 or turn > 0.5:
            return False
    return True


def _update(objects) -> dict:
    """The trial's latches (mk.MEMORY, cleared at every reset): the cart lifted clear (all four wheels off the
    ground at once), tipped (pitch over 25 deg or roll over 15 deg), or the carton lost, ever."""
    m = mk.MEMORY
    if all(_wheel_up(objects, n) > LIFT_MM for n in WHEELS):
        m["cart_lifted"] = True
    pitch, roll, _ = attitude(objects)
    if abs(pitch) > TIP_PITCH or abs(roll) > TIP_ROLL:
        m["cart_tipped"] = True
    if not _carton_on(objects):
        m["cart_load_lost"] = True
    return m


def score(objects) -> float:
    """1.0: all four wheels on the platform, the deck's front within 10 mm of the back stop and square to it (yaw
    within 10 deg), the carton on the deck, the cart and carton let go and still. 0.6: all four wheels on the
    platform. 0.3: both casters on the platform. 0 otherwise, and 0 for the rest of the trial once the cart has been
    lifted clear of the ground, tipped, or has lost the carton."""
    m = _update(objects)
    if m.get("cart_lifted") or m.get("cart_tipped") or m.get("cart_load_lost"):
        return 0.0
    casters = all(_on_platform(objects, n) for n in WHEELS if WHEELS[n] == "caster")
    rears = all(_on_platform(objects, n) for n in WHEELS if WHEELS[n] == "rear")
    still = _still(objects)
    if casters and rears:
        _, _, yaw = attitude(objects)
        done = (front_gap(objects) <= AT_STOP and abs(yaw) <= MAX_YAW and still
                and all(_released(objects[n]) for n in (*CART_PARTS, "carton")))
        return 1.0 if done else 0.6
    if casters:
        return 0.3
    return 0.0


def success(objects) -> bool:
    """In the bay against the stop with the carton on, never lifted, tipped or unloaded, let go; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Metres to go: the deck's front to the back stop."""
    return max(0.0, front_gap(objects))


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
GRIP_DZ = 0.009  # TCP above the handle's axis: the pad tips 4 mm under it
OPEN_HALF = -0.022  # fingers opened to 44 mm round the 12 mm handle (clear of the carton)
SQUEEZE = -(0.012 - 0.008) / 2
YAW = math.pi / 2  # fingers close along x, across the handle
TILT = 12.0  # tip the cart back 12 deg on its rear wheels: the casters rise about 17 mm
ROLL = 0.060  # roll forward this far on the rear wheels, casters in the air
PUSH_DOWN = 0.002  # carry the handle 2 mm low while pushing (never lift the cart)


def _handle_at(axle_x, axle_z, pitch_deg):
    """The handle's axis with the rear axle at (axle_x, axle_z) and the deck pitched nose-up by pitch_deg."""
    t = math.radians(pitch_deg)
    dx, dz = POST_X, HANDLE_Z - R_A
    return axle_x + dx * math.cos(t) - dz * math.sin(t), axle_z + dx * math.sin(t) + dz * math.cos(t)


def _tcp(axle_x, axle_z, pitch_deg, dz=0.0):
    x, z = _handle_at(axle_x, axle_z, pitch_deg)
    return (x, CY, z + GRIP_DZ + dz)


def demo():
    """Right arm, open loop from the cart's and the bay's geometry: grip the handle; tip the cart back 12 deg about
    its rear axle (the casters rise 17 mm); roll it 6 cm forward on its rear wheels, which puts the casters over the
    platform; lower them onto it; push on, the rear wheels climbing the sill, to the back stop; let go."""
    za = TOP + R_A  # rear axle height on the floor
    pitch_on = math.degrees(math.asin(SILL_H / WHEELBASE))  # casters on the platform, rear wheels on the floor
    x1 = X0 + ROLL
    x_climb = X_SILL - R_A + 0.004  # the rear wheels touch the sill's edge about here
    x_up = X_SILL + 0.012  # ... and are up on it here
    x_end = X_STOP - DECK[1] + 0.003  # commanded 3 mm into the stop
    seg = [
        Waypoint(2.0, _tcp(X0, za, 0.0, 0.050), OPEN_HALF, "over the handle", YAW),
        Waypoint(1.5, _tcp(X0, za, 0.0, 0.015), OPEN_HALF, "descend", YAW),
        Waypoint(1.0, _tcp(X0, za, 0.0), OPEN_HALF, "down round the handle", YAW),
        Waypoint(0.5, _tcp(X0, za, 0.0), OPEN_HALF, "settle", YAW),
        Waypoint(1.0, _tcp(X0, za, 0.0), SQUEEZE, "grip the handle", YAW),
        Waypoint(0.5, _tcp(X0, za, 0.0), SQUEEZE, "settle closed", YAW),
    ]
    for k in (1, 2, 3, 4):
        seg.append(Waypoint(0.6, _tcp(X0, za, TILT * k / 4), SQUEEZE, f"tip back to {TILT * k / 4:.0f} deg", YAW))
    seg += [
        Waypoint(0.5, _tcp(X0, za, TILT), SQUEEZE, "hold tipped", YAW),
        Waypoint(3.0, _tcp(x1, za, TILT), SQUEEZE, "roll forward on the rear wheels", YAW),
        Waypoint(0.5, _tcp(x1, za, TILT), SQUEEZE, "settle", YAW),
        Waypoint(1.5, _tcp(x1, za, pitch_on, -PUSH_DOWN), SQUEEZE, "lower the casters onto the platform", YAW),
        Waypoint(0.5, _tcp(x1, za, pitch_on, -PUSH_DOWN), SQUEEZE, "settle", YAW),
        Waypoint(2.0, _tcp(x_climb, za, pitch_on, -PUSH_DOWN), SQUEEZE, "roll to the sill", YAW),
        Waypoint(2.0, _tcp(x_up, za + SILL_H, 0.0, -PUSH_DOWN), SQUEEZE, "drive the rear wheels up the sill", YAW),
        Waypoint(3.5, _tcp(x_end, za + SILL_H, 0.0, -PUSH_DOWN), SQUEEZE, "push to the back stop", YAW),
        Waypoint(0.5, _tcp(x_end, za + SILL_H, 0.0, -PUSH_DOWN), SQUEEZE, "settle against the stop", YAW),
        Waypoint(1.0, _tcp(x_end - 0.003, za + SILL_H, 0.0), OPEN_HALF, "let go", YAW),
        Waypoint(1.0, _tcp(x_end - 0.003, za + SILL_H, 0.0, 0.035), OPEN_HALF, "lift clear", YAW),
        Waypoint(1.5, _tcp(x_end - 0.060, za + SILL_H, 0.0, 0.045), OPEN_HALF, "retreat", YAW),
    ]
    return {"right": seg}


# -- offline self-test ------------------------------------------------------------------------------------------


def _rot_y(deg):
    t = math.radians(-deg) / 2  # nose up = rotation about -y
    return (0.0, math.sin(t), 0.0, math.cos(t))


def self_test():
    """The scoring rules on hand-built states (the kit's forward kinematics for the cart), one trial each."""
    global _TEST_TCPS

    def st(axle_x=X0, axle_dz=0.0, pitch=0.0, yaw=0.0, carton=None, carton_dz=0.0, lift=0.0):
        """The cart with its rear axle at axle_x (raised axle_dz), pitched nose-up about the rear axle, and the
        carton riding on it unless ``carton`` (centre x, y, bottom z) is given."""
        qp = np.array(_rot_y(pitch))
        hy = math.radians(yaw) / 2
        qy = np.array([0.0, 0.0, math.sin(hy), math.cos(hy)])
        rot = mk.quat_to_mat(qy) @ mk.quat_to_mat(qp)
        axle = np.array([axle_x, CY, TOP + R_A + axle_dz + lift])
        origin = axle + rot @ np.array([0.0, 0.0, -R_A])
        s = kit.fake_states(moved={"cart": (origin, mk.mat_to_quat(rot))})
        if carton is None:
            c = origin + rot @ np.array([CARTON_X, 0.0, DECK[4] + CARTON[2] / 2]) + np.array([0, 0, carton_dz])
            q = mk.mat_to_quat(rot)
        else:
            c = np.array([carton[0], carton[1], carton[2] + CARTON[2] / 2])
            q = np.array([0.0, 0.0, 0.0, 1.0])
        half = np.array(CARTON) / 2
        ext = np.abs(mk.quat_to_mat(q)) @ half
        s["carton"] = ObjectState(c, q, np.zeros(3), np.zeros(3), c - ext, c + ext)
        return s

    def again(s):
        return dict(s)

    pitch_on = math.degrees(math.asin(SILL_H / WHEELBASE))
    start = st()
    tipped_back = st(pitch=TILT)
    rolled = st(axle_x=X0 + ROLL, pitch=TILT)
    casters_on = st(axle_x=X0 + ROLL, pitch=pitch_on)
    at_sill = st(axle_x=X_SILL - R_A, pitch=pitch_on)
    all_on = st(axle_x=X_SILL + 0.020, axle_dz=SILL_H)
    end_x = X_STOP - DECK[1]
    at_stop = st(axle_x=end_x, axle_dz=SILL_H)
    rammed = st(axle_x=X_SILL - WHEELBASE - C_A)  # casters against the sill face
    rammed_over = st(axle_x=X_SILL - WHEELBASE - C_A, pitch=-30.0)  # pushed on: pitched over the casters
    carried = st(axle_x=X0 + 0.08, axle_dz=0.03)  # the whole cart lifted 3 cm and carried
    carried_in = st(axle_x=end_x, axle_dz=SILL_H)
    load_slid = st(axle_x=X_SILL - WHEELBASE - C_A, carton=(X_SILL + 0.02, CY, PLAT_TOP))
    tipped_far = st(pitch=30.0)
    skewed = st(axle_x=end_x - 0.004, axle_dz=SILL_H, yaw=14.0)
    short = st(axle_x=end_x - 0.03, axle_dz=SILL_H)
    tcp_on_handle = [np.array(_tcp(end_x, TOP + R_A + SILL_H, 0.0))]
    lifted_carton = st(axle_x=end_x, axle_dz=SILL_H, carton_dz=0.03)

    cases = {  # name: (states in one trial, TCPs at the last state, final score, success)
        "start": ([start, again(start)], [], 0.0, False),
        "tipped back on the rear wheels": ([start, tipped_back], [], 0.0, False),
        "tipped and rolled, casters over the platform": ([start, tipped_back, rolled], [], 0.0, False),
        "casters set down on the platform": ([start, tipped_back, rolled, casters_on], [], 0.3, False),
        "casters on, rear wheels at the sill": ([start, tipped_back, rolled, casters_on, at_sill], [], 0.3, False),
        "all wheels on, short of the stop": ([start, casters_on, all_on, again(all_on)], [], 0.6, False),
        "demo: in against the stop, let go, still":
            ([start, tipped_back, rolled, casters_on, at_sill, all_on, at_stop, again(at_stop)], [], 1.0, True),
        "at the stop, still gripping the handle": ([start, casters_on, all_on, at_stop, again(at_stop)], tcp_on_handle,
                                                   0.6, False),
        "at the stop, still rolling (one step only)": ([start, casters_on, all_on, at_stop], [], 0.6, False),
        "3 cm short of the stop": ([start, casters_on, all_on, short, again(short)], [], 0.6, False),
        "at the stop but skewed 14 deg": ([start, casters_on, all_on, skewed, again(skewed)], [], 0.6, False),
        "naive: rammed into the sill": ([start, rammed, again(rammed)], [], 0.0, False),
        "rammed harder: pitched over the casters, then set right and pushed in":
            ([start, rammed, rammed_over, casters_on, all_on, at_stop, again(at_stop)], [], 0.0, False),
        "carried in clear of the ground": ([start, carried, carried_in, again(carried_in)], [], 0.0, False),
        "the carton slid off at the sill": ([start, rammed, load_slid, again(load_slid)], [], 0.0, False),
        "tipped back past 25 deg, then done right":
            ([start, tipped_far, casters_on, all_on, at_stop, again(at_stop)], [], 0.0, False),
        "carton lifted off and held over the deck": ([start, at_stop, lifted_carton, again(lifted_carton)], [], 0.0,
                                                     False),
    }
    try:
        for name, (seq, tcps, want, want_success) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for s in seq[:-1]:
                score(s)
            _TEST_TCPS = tcps
            got, got_success = score(seq[-1]), success(seq[-1])
            assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
        mk.new_episode()  # a new trial forgets a lift
        _TEST_TCPS = []
        score(carried)
        mk.new_episode()
        for s in (start, casters_on, all_on, at_stop):
            score(s)
        assert score(again(at_stop)) == 1.0, "reset"
        # geometry: tipped 12 deg the casters clear the sill; a caster cannot climb a step above its radius
        assert min(float(tipped_back[n].low[2]) for n in ("caster_l", "caster_r")) > PLAT_TOP + 0.004
        assert C_A < SILL_H < R_A and abs(front_gap(at_stop)) < 1e-6
    finally:
        mk.new_episode()
        _TEST_TCPS = None
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
