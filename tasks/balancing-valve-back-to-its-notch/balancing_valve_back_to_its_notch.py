"""On the station table stands a training rig of a fan-coil's water supply: a copper pipe run along y on low stands,
fed from the right. On it, in front of the right arm, sits a lever-operated balancing butterfly valve; in front of
the left arm, downstream of it, a basket strainer with its screen cartridge (brass cap over a rust-brown, clogged
mesh screen) seated in the strainer's bore. A clean cartridge hangs in a parts rack near the left arm, and an empty
grey drip tray waits beside the strainer.

The valve's lever (red grip, 15 cm from the stem) swings in the horizontal plane over a notch plate: closed points
the lever at the robot (0 deg), fully open lays it along the pipe (90 deg). The plate has 10 notches, every 10 deg,
marked by plain ticks with no numbers, and the lever's spring detent clicks into each one (a spring pulls the lever
to the nearest notch; a light push moves it on). The valve was balanced at the third notch (30 deg) and stands
there. Nothing in the scene records that setting except the lever itself.

Line pressure holds the strainer's cartridge in its bore while the valve is open: the cartridge cannot be lifted
out until the valve is shut, and it is held again if the valve is reopened with the cartridge still home. Opening
the valve while the bore is empty, or lifting a cartridge out while the valve is open, sprays the coil cabinet.

The job: swap the clogged cartridge for the clean one (shut the valve first, or the cap won't come off), drop the
clogged one in the tray, and put the coil back in service. A balancing valve's setting is the balance: it goes back
to the notch it was in, not wide open.

Run it:  inspect-robots run --embodiment yam_arena \\
           -E task=tasks/balancing-valve-back-to-its-notch/balancing_valve_back_to_its_notch.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (detent hinge, pressure-lock bond, per-step rules, per-episode memory)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: print the valve, the lock and the cartridges every 10 steps

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the valve ----------------------------------------------------------------------------------------------------
HX, HY = 0.340, -0.100  # valve stem axis (vertical)
PIPE_Z, PIPE_W = TOP + 0.040, 0.034  # pipe centre height, square section (modelled square)
BODY_TOP = TOP + 0.078  # valve body (on the pipe) top
PLATE_Z = TOP + 0.0945  # notch plate centre height (3 mm thick: top at TOP + 0.096)
PLATE_R = 0.062
LEVER_Z = TOP + 0.105  # lever bar centre height (bar 10 mm tall, bottom at TOP + 0.100)
LEVER_L = 0.150  # stem axis to the lever's tip
SLEEVE = (0.075, 0.024, 0.016)  # red vinyl grip: from 7.5 cm out to the tip, 24 mm wide, 16 mm tall
SLEEVE_R0 = LEVER_L - SLEEVE[0]
LEVER_KG = 0.15
PITCH = 10.0  # deg between notches
N_NOTCH = 10  # notches 0 (shut) .. 9 (90 deg, wide open)
START_NOTCH = 3  # the balance setting the valve stands at
DETENT_K = 0.8  # N*m/rad: spring to the nearest notch (0.07 N*m at the midpoint between notches)
DETENT_D = 0.03  # N*m*s/rad

# -- the strainer and the cartridges ------------------------------------------------------------------------------
SX, SY = 0.340, 0.100  # strainer bore axis
BORE = 0.040  # square bore, inside
S_WALL = 0.013
S_FLOOR = TOP + 0.008
RIM = TOP + 0.072  # strainer rim (the cap rests on it)
TUBE = (0.030, 0.030, 0.060)  # screen: stainless mesh tube (square in the sim)
CAP = (0.044, 0.044, 0.024)  # brass cap: the grip
CART_KG = 0.10
SEAT_Z = RIM - TUBE[2]  # a seated cartridge's origin (tube bottom centre)
RX, RY = 0.235, 0.205  # parts rack: the clean cartridge hangs in its bore
R_RIM = TOP + 0.065
R_WALL = 0.012
TX, TY = 0.215, 0.060  # drip tray (fixed, grey polypropylene), inside 11 x 11 cm, 3 cm walls
TRAY_IN, TRAY_WALL, TRAY_H, TRAY_BASE = 0.110, 0.006, 0.030, 0.004
TRAY_FLOOR = TOP + TRAY_BASE

STEEL, RED, ZINC, INK = (0.55, 0.56, 0.58), (0.78, 0.10, 0.08), (0.70, 0.71, 0.72), (0.06, 0.06, 0.07)
IRON, COPPER, BRONZE = (0.22, 0.26, 0.32), (0.72, 0.45, 0.22), (0.62, 0.48, 0.26)
BRASS, RUST, MESH = (0.82, 0.66, 0.26), (0.42, 0.26, 0.14), (0.85, 0.86, 0.88)
RACK_C, TRAY_C, ALU = (0.20, 0.35, 0.60), (0.45, 0.46, 0.48), (0.78, 0.80, 0.82)

kit = mk.Kit()
# lever, reference pose = shut (pointing at the robot, along -x); origin on the stem axis at the bar's centre height
kit.part("lever", (HX, HY, LEVER_Z), [
    mk.box((LEVER_L + 0.020, 0.016, 0.010), at=(-(LEVER_L - 0.020) / 2, 0, 0)),
    mk.box(SLEEVE, at=(-(SLEEVE_R0 + SLEEVE[0] / 2), 0, 0), color=RED),
    mk.box((0.026, 0.026, 0.014), at=(0, 0, 0.0005)),  # hub on the stem
], mass=LEVER_KG, color=STEEL)
kit.hinge("valve", None, "lever", anchor=(HX, HY, LEVER_Z), axis=(0, 0, 1), lower_deg=-2.0, upper_deg=92.0,
          start_deg=START_NOTCH * PITCH, stiffness=DETENT_K, rest_deg=START_NOTCH * PITCH, damping=DETENT_D)


def _cartridge(name, pos, tube_color):
    return kit.part(name, pos, [
        mk.box(TUBE, at=(0, 0, TUBE[2] / 2), color=tube_color),
        mk.box(CAP, at=(0, 0, TUBE[2] + CAP[2] / 2)),
        mk.box((0.020, 0.020, 0.003), at=(0, 0, TUBE[2] + CAP[2] + 0.0015), color=BRONZE),  # plug boss
    ], mass=CART_KG, color=BRASS)


_cartridge("cart_dirty", (SX, SY, SEAT_Z), RUST)
_cartridge("cart_clean", (RX, RY, R_RIM - TUBE[2]), MESH)
# line pressure on the strainer cap: a lock joint that the rule below frees only while the valve is shut
kit.bond("line_pressure", None, "cart_dirty", anchor=(SX, SY, SEAT_Z))


def _walls(name, cx, cy, inner, wall, z0, z1, color):
    zc, h, out = (z0 + z1) / 2, z1 - z0, []
    for s in (-1, 1):
        out.append(Box(f"{name}_wx{'np'[s > 0]}", pos=(cx + s * (inner / 2 + wall / 2), cy, zc),
                       size=(wall, inner + 2 * wall, h), color=color))
        out.append(Box(f"{name}_wy{'np'[s > 0]}", pos=(cx, cy + s * (inner / 2 + wall / 2), zc),
                       size=(inner, wall, h), color=color))
    return out


def _pipe(name, y0, y1):
    return Box(name, pos=(HX, (y0 + y1) / 2, PIPE_Z), size=(PIPE_W, y1 - y0, PIPE_W), color=COPPER)


_S_OUT = BORE + 2 * S_WALL
FIXTURES = [
    # pipe run, fed from the right; stands under it
    _pipe("pipe_in", -0.260, HY - 0.026),
    _pipe("pipe_mid", HY + 0.026, SY - _S_OUT / 2),
    _pipe("pipe_out", SY + _S_OUT / 2, 0.205),
    Box("pipe_elbow", pos=(HX + 0.060, 0.205 - PIPE_W / 2, PIPE_Z), size=(0.120 + PIPE_W, PIPE_W, PIPE_W),
        color=COPPER),
    Box("stand_in", pos=(HX, -0.230, TOP + 0.0115), size=(0.040, 0.020, 0.023), color=IRON),
    Box("stand_mid", pos=(HX, 0.000, TOP + 0.0115), size=(0.040, 0.020, 0.023), color=IRON),
    Box("coil", pos=(0.475, 0.110, TOP + 0.090), size=(0.040, 0.300, 0.180), color=ALU),  # fan-coil fin block
    # valve body (cast iron, lugged butterfly), bonnet and the notch plate
    Box("valve_body", pos=(HX, HY, (TOP + BODY_TOP) / 2), size=(0.060, 0.052, BODY_TOP - TOP), color=IRON),
    Box("valve_bonnet", pos=(HX, HY, (BODY_TOP + PLATE_Z - 0.0015) / 2), size=(0.030, 0.030,
                                                                              PLATE_Z - 0.0015 - BODY_TOP),
        color=IRON),
    *[Box(f"plate_{k}", pos=(HX - PLATE_R / 2 * math.cos(math.radians(a)), HY - PLATE_R / 2 * math.sin(math.radians(a)),
                             PLATE_Z), size=(PLATE_R, 0.0070, 0.003), yaw=math.radians(a), color=ZINC)
      for k, a in enumerate(range(-5, 100, 5))],
    *[Box(f"tick_{n}", pos=(HX - 0.056 * math.cos(math.radians(n * PITCH)), HY - 0.056 * math.sin(math.radians(n * PITCH)),
                            PLATE_Z + 0.0019), size=(0.011, 0.0022, 0.0008), yaw=math.radians(n * PITCH), color=INK)
      for n in range(N_NOTCH)],
    # strainer (bronze): floor and four walls round the bore
    Box("strainer_floor", pos=(SX, SY, (TOP + S_FLOOR) / 2), size=(_S_OUT, _S_OUT, S_FLOOR - TOP), color=BRONZE),
    *_walls("strainer", SX, SY, BORE, S_WALL, S_FLOOR, RIM, BRONZE),
    # parts rack (painted steel): a bore the clean cartridge hangs in by its cap
    *_walls("rack", RX, RY, BORE, R_WALL, TOP, R_RIM, RACK_C),
    # drip tray
    Box("tray_base", pos=(TX, TY, TOP + TRAY_BASE / 2), size=(TRAY_IN + 2 * TRAY_WALL, TRAY_IN + 2 * TRAY_WALL,
                                                               TRAY_BASE), color=TRAY_C),
    *_walls("tray", TX, TY, TRAY_IN, TRAY_WALL, TRAY_FLOOR, TRAY_FLOOR + TRAY_H, TRAY_C),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the per-step rules (sim) and the scoring ---------------------------------------------------------------------
SHUT_DEG = 2.0  # valve shut: the pressure is off the strainer
OPEN_DEG = 5.0  # valve open: the strainer is under pressure
IN_NOTCH_DEG = 3.0  # the lever sits in a notch (the detent holds it within this when let go)
SEAT_XY, SEAT_DZ, TILT_MAX = 0.007, 0.0025, 6.0  # a cartridge seated in the strainer (the bore allows 5 mm play)
BORE_XY, BORE_DZ = 0.009, 0.010  # a cartridge still in the bore (not yet lifted out)
RELOCK_DZ = 0.0015  # the lock takes hold again only on a cartridge this close to home
CLEAR = 0.030  # both grippers this far from the lever (and 2.5 cm from the cartridges) when done
CARTS = ("cart_dirty", "cart_clean")
M = "bvbn"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"init": False, "locked": True, "in_bore": {"cart_dirty": True, "cart_clean": False},
                            "fail": None, "shut_seen": False, "n": 0}
    return m


def valve_deg(objects) -> float:
    """How far the valve is open (deg; 0 = shut, 90 = wide open), from the lever's pose about the stem."""
    return kit.q_deg(objects, "valve")


def notch(objects) -> int:
    """The notch nearest the lever (0 = shut .. 9)."""
    return int(min(N_NOTCH - 1, max(0, round(valve_deg(objects) / PITCH))))


def in_notch(objects) -> bool:
    return abs(valve_deg(objects) - notch(objects) * PITCH) <= IN_NOTCH_DEG


def _tilt(objects, part) -> float:
    _, q = kit.pose(objects, part)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(mk.quat_to_mat(q)[2, 2])))))


def _bore_offset(objects, part):
    """(xy distance from the bore axis, height above the seated position) of a cartridge's origin, world frame."""
    p, _ = kit.pose(objects, part)
    return math.hypot(p[0] - SX, p[1] - SY), p[2] - SEAT_Z


def seated(objects, part) -> bool:
    d, dz = _bore_offset(objects, part)
    return bool(d <= SEAT_XY and abs(dz) <= SEAT_DZ and _tilt(objects, part) <= TILT_MAX)


def in_bore(objects, part) -> bool:
    d, dz = _bore_offset(objects, part)
    return bool(d <= BORE_XY and dz <= BORE_DZ)


def in_tray(objects, part) -> bool:
    """The cartridge lies or stands inside the drip tray, on its floor."""
    st = objects[part]
    half = TRAY_IN / 2
    return bool(abs(st.pos[0] - TX) <= half and abs(st.pos[1] - TY) <= half and abs(st.low[2] - TRAY_FLOOR) <= 0.006)


def _drive_valve(target_deg) -> None:
    """Sim only: the detent spring's set point (the notch it pulls the lever into)."""
    path = mk._SIM["joint_paths"].get((kit.id, "valve"))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "angular").GetTargetPositionAttr().Set(float(target_deg))


def _lock(on: bool) -> None:
    """Sim only: line pressure on (cap held in the bore) or off."""
    if on:
        kit._released.discard("line_pressure")
        mk._set_joint_enabled(kit, "line_pressure", True)
    else:
        kit.release("line_pressure")


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit bvbn] step {mk._EP.get('step')}: {msg}", flush=True)


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


def _released(st, margin=0.025) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def _rules(objects) -> None:
    """One control step: the lever's detent, the line-pressure lock, and the spray watch."""
    m = _mem()
    m["n"] += 1
    a = valve_deg(objects)
    _drive_valve(notch(objects) * PITCH)  # spring detent: pull toward the nearest notch
    if a <= SHUT_DEG:
        m["shut_seen"] = True
    # line pressure: off while the valve is shut; on again if it is reopened over a cartridge still home
    if m["locked"] and a <= SHUT_DEG:
        m["locked"] = False
        _lock(False)
        _say(f"valve shut ({a:.1f} deg): strainer cap free")
    elif not m["locked"] and a > OPEN_DEG and in_bore(objects, "cart_dirty") \
            and _bore_offset(objects, "cart_dirty")[1] <= RELOCK_DZ and _bore_offset(objects, "cart_dirty")[0] <= 0.002:
        m["locked"] = True
        _lock(True)
        _say(f"valve reopened ({a:.1f} deg) over the old cartridge: cap held again")
    # spray: a cartridge lifted out of the bore while the valve is open, or the valve opened on an empty bore
    for c in CARTS:
        now = in_bore(objects, c)
        if m["in_bore"][c] and not now and a > SHUT_DEG and m["fail"] is None:
            m["fail"] = f"{c} lifted out with the valve {a:.1f} deg open"
        m["in_bore"][c] = now
    if a > OPEN_DEG and not any(m["in_bore"].values()) and m["fail"] is None:
        m["fail"] = f"valve opened to {a:.1f} deg with the strainer open"
    if m["fail"] and not m.get("said"):
        m["said"] = True
        _say("FAIL: " + m["fail"])
    if DEBUG and mk._EP.get("env") is not None and m["n"] % 10 == 0:
        tc = _tcps()
        tcs = " ".join(f"({t[0]:.3f},{t[1]:.3f},{t[2] - TOP:.3f})" for t in tc)
        bd, bc = _bore_offset(objects, "cart_dirty"), _bore_offset(objects, "cart_clean")
        pd, pc = kit.pose(objects, "cart_dirty")[0], kit.pose(objects, "cart_clean")[0]
        _say(f"valve {a:.2f} deg notch {notch(objects)} locked {m['locked']} | dirty ({pd[0]:.4f},{pd[1]:.4f},"
             f"{pd[2] - TOP:.4f}) bore {bd[0] * 1000:.1f},{bd[1] * 1000:.1f} mm | clean ({pc[0]:.4f},{pc[1]:.4f},"
             f"{pc[2] - TOP:.4f}) bore {bc[0] * 1000:.1f},{bc[1] * 1000:.1f} mm tilt {_tilt(objects, 'cart_clean'):.1f}"
             f" | tcps {tcs}")


kit.on_step(_rules)


def score(objects) -> float:
    """0 once the strainer has sprayed (a cartridge lifted out with the valve open, or the valve opened on an empty
    bore). Otherwise: 0.25 for the clean cartridge seated and let go; with it, 0.5 once the valve is open again;
    0.75 with the lever in a notch within one of where it was; 1.0 in exactly that notch with the clogged
    cartridge in the drip tray, both grippers clear and everything still."""
    if mk._EP.get("env") is None:  # offline; in sim the per-step rule has already run
        _rules(objects)
    m = _mem()
    if m["fail"]:
        return 0.0
    if not (seated(objects, "cart_clean") and _released(objects["cart_clean"])):
        return 0.0
    a = valve_deg(objects)
    if a < OPEN_DEG:
        return 0.25
    n = notch(objects)
    if not (in_notch(objects) and abs(n - START_NOTCH) <= 1):
        return 0.5
    done = (n == START_NOTCH and in_tray(objects, "cart_dirty") and _released(objects["cart_dirty"])
            and _released(objects["lever"], CLEAR)
            and all(objects[p].resting() for p in ("lever", *CARTS)))
    return 1.0 if done else 0.75


def success(objects) -> bool:
    """Clean cartridge seated, valve back in its starting notch, clogged cartridge in the tray, let go, still."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Stages still to do plus the lever's distance from its starting notch (in notches)."""
    left = (not seated(objects, "cart_clean")) + (not in_tray(objects, "cart_dirty"))
    return float(left + abs(valve_deg(objects) - START_NOTCH * PITCH) / PITCH)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry and the starting notch seen at the start (30 deg); it reads no joint or lock
# state. Right arm: fingers closed, pad tips just above the notch plate's height, it pushes the lever's red grip
# sideways along its arc (fingers turned with the lever, so the pads' flat face meets the grip), first shut, later
# back to 30 deg; the detent takes the lever into the notch. Left arm: grips each cartridge by its cap's sides.

TIP = 0.013  # pad tips below the TCP
PAD_T = 0.0064  # one pad's thickness: the closed fingers are 12.8 mm thick
PUSH_R = 0.115  # push the grip this far out from the stem
PUSH_IN = 0.002  # command the pads this far into the grip's side (the arm is compliant)
OFF = SLEEVE[1] / 2 + PAD_T - PUSH_IN  # finger centre from the lever's centre line while pushing
BACK = 0.012  # extra side clearance to come down / back off
PUSH_Z = TOP + 0.099 + TIP  # pad tips 3 mm over the plate's height, under the grip's mid-height
SAFE_Z = TOP + 0.170
SQUEEZE = 0.008
GRIP = -(CAP[0] - SQUEEZE) / 2
CAP_UP = 0.004  # pad tips this far above the cap's underside
G = TUBE[2] + CAP_UP + TIP  # TCP height over a cartridge's origin while gripping it
CARRY_Z = RIM + 0.018  # carry with the tube's bottom this high (clears the strainer and rack rims)
SET_TRIM = (0.0, 0.0)  # set-down correction into the strainer (none needed once it is let stop swinging over the
# rim: sim run 2 landed it where commanded)
R_PARK = (0.200, -0.250, TOP + 0.200)
L_PARK = (0.200, 0.250, TOP + 0.200)


def _d(deg):
    a = math.radians(deg)
    return np.array([-math.cos(a), -math.sin(a), 0.0])


def _t(deg):
    a = math.radians(deg)
    return np.array([math.sin(a), -math.cos(a), 0.0])


def _finger(deg, side, off):
    """Closed-finger TCP beside the lever's grip at lever angle ``deg``; ``side`` +1 = on the opening side (to push
    it shut), -1 = on the shut side (to push it open)."""
    p = np.array([HX, HY, 0.0]) + PUSH_R * _d(deg) + side * off * _t(deg)
    return (float(p[0]), float(p[1]), PUSH_Z)


def _sweep(a0, a1, side, step=5.0, per=0.7):
    """Waypoints that push the lever from ``a0`` to ``a1`` deg, fingers turned with it."""
    c = FINGER_CLOSED
    out = []
    up = _finger(a0, side, OFF + BACK)
    out.append(Waypoint(2.0, (up[0], up[1], SAFE_Z), c, "over the lever's grip, beside it", math.radians(a0)))
    out.append(Waypoint(1.2, up, c, "fingers down beside the grip", math.radians(a0)))
    out.append(Waypoint(0.8, _finger(a0, side, OFF), c, "onto its side", math.radians(a0)))
    n = max(1, int(math.ceil(abs(a1 - a0) / step)))
    for k in range(1, n + 1):
        a = a0 + (a1 - a0) * k / n
        out.append(Waypoint(per, _finger(a, side, OFF), c, "push it along its arc", math.radians(a)))
    out.append(Waypoint(0.6, _finger(a1, side, OFF), c, "hold in the notch", math.radians(a1)))
    back = _finger(a1, side, OFF + BACK)
    out.append(Waypoint(0.8, back, c, "back off sideways", math.radians(a1)))
    out.append(Waypoint(1.0, (back[0], back[1], SAFE_Z), c, "up", math.radians(a1)))
    return out


def _pick_place(src, dst_origin, label, trim=(0.0, 0.0)):
    """Left arm: grip a cartridge by its cap at ``src`` (origin), carry it over, set it at ``dst_origin``."""
    o, c = FINGER_OPEN, GRIP
    sx, sy, sz = src
    dx, dy, dz = dst_origin[0] + trim[0], dst_origin[1] + trim[1], dst_origin[2]
    cz = CARRY_Z + G
    return [
        Waypoint(2.0, (sx, sy, sz + G + 0.050), o, f"over the {label} cartridge, fingers open"),
        Waypoint(1.5, (sx, sy, sz + G), o, "down round its cap"),
        Waypoint(0.4, (sx, sy, sz + G), o, "settle"),
        Waypoint(0.8, (sx, sy, sz + G), c, "grip the cap"),
        Waypoint(0.4, (sx, sy, sz + G), c, "settle closed"),
        Waypoint(2.0, (sx, sy, cz), c, "lift it straight out"),
        Waypoint(2.0, (dx, dy, cz), c, "carry"),
        Waypoint(1.2, (dx, dy, dz + G + TUBE[2] + 0.006), c, "down to just over the rim"),
        Waypoint(0.8, (dx, dy, dz + G + TUBE[2] + 0.006), c, "let it stop swinging"),
        Waypoint(1.8, (dx, dy, dz + G + 0.002), c, "lower"),
        Waypoint(0.5, (dx, dy, dz + G + 0.002), c, "settle"),
        Waypoint(0.8, (dx, dy, dz + G + 0.002), o, "let go"),
        Waypoint(1.2, (dx, dy, dz + G + 0.060), o, "up"),
    ]


def demo():
    """Push the lever shut (it was at the third notch, 30 deg); lift the clogged cartridge out and drop it in the
    tray; take the clean one from the rack and seat it; push the lever back to the third notch; let go."""
    start = START_NOTCH * PITCH  # seen at the start: the lever stands three notches open
    close = _sweep(start, -1.5, +1) + [Waypoint(1.5, R_PARK, FINGER_CLOSED, "park clear")]
    swap = (_pick_place((SX, SY, SEAT_Z), (TX, TY, TRAY_FLOOR), "clogged")
            + _pick_place((RX, RY, R_RIM - TUBE[2]), (SX, SY, SEAT_Z), "clean", SET_TRIM)
            + [Waypoint(1.5, L_PARK, FINGER_OPEN, "park clear")])
    reopen = _sweep(0.0, start, -1) + [Waypoint(1.5, R_PARK, FINGER_CLOSED, "park clear")]
    ta, tb, tc = (sum(w.duration for w in ws) for ws in (close, swap, reopen))
    end = 3.0
    right = [Waypoint(1.5, R_PARK, FINGER_CLOSED, "start"), *close,
             Waypoint(tb, R_PARK, FINGER_CLOSED, "wait while the strainer is serviced"), *reopen,
             Waypoint(end, R_PARK, FINGER_CLOSED, "done")]
    left = [Waypoint(1.5, L_PARK, FINGER_OPEN, "start"), Waypoint(ta, L_PARK, FINGER_OPEN, "wait for the valve"),
            *swap, Waypoint(tc + end, L_PARK, FINGER_OPEN, "done")]
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of poses (one trial each, fed step by step)."""
    global _TEST_TCPS
    up = (0.0, 0.0, 0.0, 1.0)
    d_home = (SX, SY, SEAT_Z)
    c_home = (RX, RY, R_RIM - TUBE[2])
    d_tray = (TX, TY, TRAY_FLOOR)
    c_seat = (SX, SY, SEAT_Z)

    def fr(deg=30.0, d=d_home, c=c_home, dq=up):
        return kit.fake_states(q={"valve": math.radians(deg)}, moved={"cart_dirty": (d, dq), "cart_clean": (c, up)})

    def lin(a, b, n=8):
        return [a + (b - a) * k / n for k in range(n + 1)]

    def path(a, b, n=8, lift=0.09):
        a, b = np.asarray(a, float), np.asarray(b, float)
        return [tuple(a + (b - a) * k / n + np.array([0, 0, lift * math.sin(math.pi * k / n)])) for k in range(n + 1)]

    shut = [fr(a) for a in lin(30.0, 0.0)]
    out_d = [fr(0.0, d=p) for p in path(d_home, d_tray)]
    in_c = [fr(0.0, d=d_tray, c=p) for p in path(c_home, c_seat)]
    swapped = shut + out_d + in_c

    def reopen(to):
        return [fr(a, d=d_tray, c=c_seat) for a in lin(0.0, to)]

    good = swapped + reopen(30.0)
    lever_tcp = tuple(np.array([HX, HY, LEVER_Z]) + PUSH_R * _d(30.0))
    cap_tcp = (SX, SY, SEAT_Z + G)
    tipped = tuple(mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.radians(90))))
    cases = {  # name: (frames, TCPs at the end, score)
        "start": ([fr()], [], 0.0),
        "shut, swap, back to the third notch, let go (demo)": (good, [], 1.0),
        "the same, clogged cartridge lying on its side in the tray": (
            swapped[:-1] + [fr(0.0, d=(TX, TY, TRAY_FLOOR + CAP[1] / 2), c=c_seat, dq=tipped)]
            + [fr(a, d=(TX, TY, TRAY_FLOOR + CAP[1] / 2), c=c_seat, dq=tipped) for a in lin(0.0, 30.0)], [], 1.0),
        "naive: reopened wide (notch 9)": (swapped + reopen(90.0), [], 0.5),
        "reopened one notch short (notch 2)": (swapped + reopen(20.0), [], 0.75),
        "reopened one notch past (notch 4)": (swapped + reopen(40.0), [], 0.75),
        "lever held between notches (25 deg)": (swapped + reopen(25.0), [lever_tcp], 0.5),
        "right notch but still holding the lever": (good, [lever_tcp], 0.75),
        "right notch, clogged cartridge left on the table": (
            shut + [fr(0.0, d=p) for p in path(d_home, (0.22, -0.03, TOP))]
            + [fr(0.0, d=(0.22, -0.03, TOP), c=p) for p in path(c_home, c_seat)]
            + [fr(a, d=(0.22, -0.03, TOP), c=c_seat) for a in lin(0.0, 30.0)], [], 0.75),
        "swapped, valve left shut": (swapped, [], 0.25),
        "clean cartridge seated but still held": (good, [cap_tcp], 0.0),
        "valve opened before the clean cartridge went in (spray)": (
            shut + out_d + [fr(a, d=d_tray) for a in lin(0.0, 30.0)]
            + [fr(30.0, d=d_tray, c=p) for p in path(c_home, c_seat)], [], 0.0),
        "cartridge pulled with the valve open (if the lock failed)": (
            [fr(30.0, d=p) for p in path(d_home, d_tray)] + [fr(30.0, d=d_tray, c=p) for p in path(c_home, c_seat)],
            [], 0.0),
        "clean pulled out again after reopening, then reseated": (
            good + [fr(30.0, d=d_tray, c=p) for p in path(c_seat, c_home)]
            + [fr(30.0, d=d_tray, c=p) for p in path(c_home, c_seat)], [], 0.0),
        "lock attempt only: valve never shut, nothing moved": ([fr(30.0)] * 5, [], 0.0),
        "valve shut and reopened, nothing swapped": (shut + [fr(a) for a in lin(0.0, 30.0)], [], 0.0),
        "clean cartridge cocked on the rim": (
            swapped[:-1] + [fr(0.0, d=d_tray, c=(SX + 0.012, SY, SEAT_Z + 0.008))]
            + [fr(a, d=d_tray, c=(SX + 0.012, SY, SEAT_Z + 0.008)) for a in lin(0.0, 30.0)], [], 0.0),
    }
    try:
        for name, (frames, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for f in frames[:-1]:
                score(f)
            _TEST_TCPS = tcps
            got = score(frames[-1])
            assert abs(got - want) < 1e-9, (name, got, want, _mem())
            assert success(frames[-1]) == (want >= 1.0), name
        # the lock: freed once shut, held again if reopened over the old cartridge, not over an empty bore
        mk.new_episode()
        _TEST_TCPS = []
        for f in shut:
            score(f)
        assert not _mem()["locked"]
        for a in lin(0.0, 30.0):
            score(fr(a))
        assert _mem()["locked"] and _mem()["fail"] is None
        mk.new_episode()
        for f in swapped + reopen(30.0):
            score(f)
        assert not _mem()["locked"]
        mk.new_episode()
        assert score(fr()) == 0.0 and _mem()["fail"] is None and _mem()["locked"], "a new trial forgets the last one"
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

    n = self_test()
    print(f"scoring: {n} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
