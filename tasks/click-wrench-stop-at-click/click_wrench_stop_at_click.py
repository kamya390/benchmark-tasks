"""On the station table a copper busbar stands on two insulators with a cable lug on it. The lug's bolt has been
run down snug by hand; its M5 hex head (with its washer) stands proud of the lug. A click-type torque wrench,
already set to the bolt's torque, stands to the robot's left ON ITS REST: socket down on the bench, its rubber
grip on a block, its handle pointing toward the right arm. The robot is responsible for the whole job: pick the
wrench up, carry it to the joint, seat the socket fully over the bolt head (a real hex fit with chamfered
lead-ins and 0.16 mm of clearance per flat), and only then torque the bolt to the click.

The wrench works as a real click wrench does. Inside it a sprung detent holds the handle in line with the head;
the torque you put through the grip bends that detent a little (1.5 deg at the set torque). At the set torque it
breaks over: the handle swings 6 deg on its pivot to a stop, a visible kink, and the load drops for a moment (the
click). Then nothing stops you: on the stop, the handle drives the head again, and pulling on turns the bolt
further. Let go, and the detent pulls the handle back in line and re-arms.

The bolt: its turning torque rises with every degree past snug (the joint clamping up), and it stays where it is
left (thread friction). At the set torque it is about 5 deg past snug. Pulled on to twice the set torque, about
15 deg past snug, it yields (the stud stretches): it then turns on under a steady pull and is ruined. Backing it
off undoes the torque. The socket drives the bolt only through the physical hex contact: it must be seated (at
least 2.8 mm of the head's 3.5 mm engaged, in line) to transmit the torque cleanly. Once the socket has entered
the head by 1 mm, rotating the visible wrench-head marker by 1 deg before the full seat holds for five frames
scores 0 (latched): the tool must be seated before tightening motion begins.

The job: seat the wrench on the bolt, tighten the bolt to its torque, then let go. Nothing about the bolt says
when to stop; the wrench does, once, and pulling on is still possible. Full credit needs the click with the
socket seated and the peak torque inside the joint's tolerance, 90-110 % of the set torque (the bolt 3.9-5.9 deg
past snug); a pull on past 110 % scores 0.

The real joint this models (README, "Real hardware and calibration"): an M5 x 0.8 class 8.8 hex bolt (ISO 4017,
7.88 mm across flats, head 3.5 mm) with a flat washer (ISO 7089), through a 16 mm2 tinned-copper tube lug and a
20 x 6 mm copper bar into a self-clinching M5 nut pressed into the bar; marked 3 N*m, snug 1.5 N*m; a 1/4-inch
drive click wrench of 2-10 N*m range, a low-cost 1/4-inch digital peak-torque adapter under its drive, and an
8 mm 6-point socket (ISO 2725-1 / ISO 1174-1 drive, opening 8.2 mm). At the grip's load mark 3 N*m is 14 N.
The simulation uses that full force and torque; its angles are estimated from the joint's stiffness. On the
real rig the adapter supplies peak torque and the existing camera tracks angle from a marker on the wrench head.
Steel-on-steel fit surfaces have mu 0.3 (zinc-plated/phosphated, lightly oiled); only the rubber grip is grippy.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/click-wrench-stop-at-click/click_wrench_stop_at_click.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (hinges with springs and friction, hull pieces, per-step rules, memory)

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False
VARIANT = "demo"  # sim runs: "demo"; "half" (the socket set down cocked, off the axis, never seated);
# "through" (pulls on to 45 deg); "short" (stops at 6 deg)

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
# -- the bolted joint: busbar on two insulators, a cable lug on it, the bolt's axis vertical -----------------------
BX, BY = 0.240, 0.000  # bolt axis (within both arms' reach: the left seats the wrench, the right pulls)
ZLUG = TOP + 0.033  # lug top (insulators 24 mm, bar 6 mm, lug palm 3 mm)
# -- the bolt: its own free body on the thread hinge; the socket drives it only through the hex contact ------------
HEAD_AF = 0.00788  # hex head across flats (ISO 4017 M5: 7.85-8.00 mm)
HEAD_K = 0.0035  # head height
HEAD_CH = 0.0012  # top chamfer: the lead-in a real head's chamfered crown gives the socket
WASHER_T, WASHER_R = 0.001, 0.005  # flat washer ISO 7089-5 (10 mm OD)
Z_W0 = 0.0002  # washer spawned a hair above the lug (no parasitic contact torque at rest)
HEAD_TOP = Z_W0 + WASHER_T + HEAD_K  # local z of the head's top face in the bolt frame (frame origin at ZLUG)
BOLT_KG = 0.030  # sim mass (the real bolt+washer is ~8 g; heavier for solver stability; the hinge sets dynamics)
# -- the socket end of the wrench: a real female hex with chamfered mouth -----------------------------------------
SOCK_AF = 0.0082  # socket opening across flats (8 mm 6-point socket, ISO 2725-1 range): 0.16 mm play per flat
SOCK_R = 0.006  # socket body radius (12 mm OD)
SOCKET_H = 0.025  # socket length
CAV = 0.0060  # hex cavity depth to its ceiling (ISO 2725 sockets cut the hex well deeper than this head's
# 3.5 mm; max engagement is still the head's 3.5 mm, rim on the washer, so 2.5 mm of wall stand above the
# crown: the hanging wrench binds wall-on-corner, with no ceiling-on-chamfer ramp to cam it off)
MOUTH_CH = 0.0008  # 45 deg mouth chamfer
XDCR = (0.026, 0.040)  # low-cost digital peak-torque adapter body between drive and socket: width, height
HEAD_T = 0.014  # ratchet head thickness
SOCK_BOT = -(HEAD_T / 2 + XDCR[1] + SOCKET_H)  # local z of the socket's bottom rim in the wrench-head frame
# -- the wrench: head (free body) and handle (on the head's pivot); it starts standing on its rest -----------------
R_PIVOT = 0.045  # handle pivot, from the socket axis
R_GRIP = 0.215  # grip centre, from the socket axis
GRIP = (0.018, 0.070, 0.018)  # rubber grip: across (x at the start), along the handle, height
TUBE = (0.014, 0.012)  # handle tube: width, height
HEAD_KG, HANDLE_KG = 0.45, 0.10  # conservative head-end mass: ratchet head + digital adapter + square-drive
# interfaces + socket; handle. This puts the tool's balance point 39 mm from the socket axis, on the neck,
# where the demo pinches it. Available adapters are no heavier, so the real carry is not harder than the sim.
SX, SY = 0.300, 0.060  # the socket axis at the start: the wrench stands socket-down on the bench
SPAWN_YAW = 0.0  # the wrench rests square on its blocks: the two-hand supported carry (see the demo)
# holds the tool's clocking, so no rest-pose pre-bias is needed
_SPC, _SPS = math.cos(SPAWN_YAW), math.sin(SPAWN_YAW)


def _spun(x, y):
    """A point of the resting wrench (part frame) in the world, after the spawn yaw about the socket axis."""
    return (SX + x * _SPC - y * _SPS, SY + x * _SPS + y * _SPC)
ZH_START = TOP + 0.0005 - SOCK_BOT  # wrench centre plane at the start (socket rim a hair above the bench)
ZH = ZLUG + Z_W0 + WASHER_T - SOCK_BOT  # wrench centre plane, socket fully seated (rim on the washer)
MU_METAL = 0.3  # fit surfaces: zinc-plated or phosphated steel, lightly oiled
# -- the seating gate ----------------------------------------------------------------------------------------------
ENG_SEAT = 0.0028  # seated: at least this much of the head engaged (of ENG_MAX)
ENG_MAX = HEAD_K  # 3.5 mm: rim on the washer, 2.5 mm of cavity wall above the head
TILT_MAX = 8.0  # deg, socket axis off vertical
COAX_MAX = 0.001  # m, socket axis off the bolt axis at the head
SEAT_STEPS = 5  # seated held this many consecutive control steps to latch SEATED
# Observable seat-before-load gate.  Once the socket has actually entered the head, the wrench-head marker must
# stay within this angle until the full seating geometry has held for SEAT_STEPS.  This measures wrench motion,
# not bolt motion: the same quantity is visible against the fixed busbar marker in a real trial even when the
# socket is not yet keyed to the bolt.
PRESEAT_ENG = 0.0010  # start the gate after 1 mm engagement (past the 0.8 mm mouth chamfer)
PRESEAT_TILT_MAX = 12.0  # broad contact envelope; full seating still requires TILT_MAX
PRESEAT_COAX_MAX = 0.003  # m; broad contact envelope catches the 2.5 mm half-seated control
PRESEAT_TURN_DEG = 1.0  # any head rotation this large before the five-step seat latch is a latched fail
# -- the mechanism (full force at the grip for the real 3 N*m joint) ---------------------------------------------
F0, F_CLICK, F_OVER = 7.0, 14.0, 32.2  # N at grip: bolt starts to turn; wrench set torque; joint yields
T0, T_CLICK, T_OVER = F0 * R_GRIP, F_CLICK * R_GRIP, F_OVER * R_GRIP  # N*m
SLOPE = 17.5  # N*m/rad: the real joint's estimated torque-angle slope above snug
CLICK_DEG = 1.5  # the detent breaks over at this deflection (= the set torque, pulled at the grip's centre)
K_DETENT = F_CLICK * (R_GRIP - R_PIVOT) / math.radians(CLICK_DEG)  # N*m/rad about the pivot, the armed detent
K_RETURN = 0.3  # N*m/rad, the reset spring once it has broken over
# PhysX's realised joint-drive stiffness is strongly super-linear in its command.  The original 2 N model used
# 0.59 N*m/rad; scaling that command by seven made the 14 N detent about eight times too stiff and prevented a
# click.  This command is calibrated directly against the restored 14 N joint; K_DETENT and the scored 1.5-deg
# physical breakover threshold remain unchanged.
DETENT_DRIVE_CMD = 1.205  # N*m/rad; validated on the current Box 1 PhysX build (93% peak, one click)
BREAKOVER_DRIVE = K_DETENT / DETENT_DRIVE_CMD
SWEEP_CMDS = (0.20, 0.481, 0.80, 1.30)  # VARIANT "sweep": commanded breakover stiffness per 6 s window of a
# slow pull (soft to stiff as the torque rises), to characterise the backend's super-linear stiffness mapping
SWEEP_WIN_S = 6.0
SWEEP_T0 = [None]  # set by demo() for the sweep variant: when the slow pull's arc begins (s)
BREAK_DEG = 6.0  # the handle's stop: how far it kinks at the click
REARM_DEG = 0.5  # back within this of in line (load off): the detent re-arms
SPEC_DEG = math.degrees((T_CLICK - T0) / SLOPE)  # bolt angle at the set torque (4.9 deg)
YIELD_DEG = math.degrees((T_OVER - T0) / SLOPE)  # bolt angle at the yield torque (17.7 deg, 230 %)
# The real joint and simulation use the same torque scale and torque-angle curve:
REAL_SET_NM = 3.0  # M5 class 8.8 steel bolt on copper, the marked torque (14 N at the load mark); snug = half
TORQUE_SCALE = T_CLICK / REAL_SET_NM  # 1.003 (3.01 N*m represented versus the 3 N*m marked value)
# The joint's documented tolerance on the peak torque: 90-110 % of the specified torque.
TOL = 0.10
TOL_LO_DEG = math.degrees(((1 - TOL) * T_CLICK - T0) / SLOPE)  # 3.9 deg past snug: 90 % of the set torque
TOL_HI_DEG = math.degrees(((1 + TOL) * T_CLICK - T0) / SLOPE)  # 5.9 deg past snug: 110 %, above it over-torqued
DAMP = 0.02  # N*m*s/rad on both hinges (a quiet solver)

STEEL, CHROME, RUBBER, RED = (0.30, 0.31, 0.33), (0.70, 0.72, 0.75), (0.08, 0.08, 0.09), (0.75, 0.12, 0.10)
COPPER, TIN, EPOXY, PVC = (0.78, 0.46, 0.25), (0.80, 0.80, 0.78), (0.85, 0.80, 0.20), (0.10, 0.10, 0.10)
BLOCK = (0.20, 0.20, 0.22)

# -- hull geometry: hexes as real convex meshes (PhysX convex hulls, tight contact offsets) -----------------------
_COS30 = math.cos(math.pi / 6)


def _rings_mesh(rings):
    """A watertight convex solid from horizontal outlines (counter-clockwise, equal counts), bottom to top:
    (points, face counts, face indices) for mk.hull."""
    n = len(rings[0][1])
    m = len(rings)
    pts = [(x, y, z) for z, ring in rings for x, y in ring]
    counts = [n, n]
    idx = list(range(n))[::-1] + [(m - 1) * n + k for k in range(n)]
    for r in range(m - 1):
        for k in range(n):
            k2 = (k + 1) % n
            counts.append(4)
            idx += [r * n + k, r * n + k2, (r + 1) * n + k2, (r + 1) * n + k]
    return pts, counts, idx


def _hex(d_flat):
    """A hexagon with its flats ``d_flat`` from the axis, flats facing k*60 deg (the bolt and socket agree)."""
    rv = d_flat / _COS30
    return [(rv * math.cos(math.pi / 6 + k * math.pi / 3), rv * math.sin(math.pi / 6 + k * math.pi / 3))
            for k in range(6)]


def _ngon(r, n=12):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def _bolt_pieces():
    z1 = Z_W0 + WASHER_T
    washer = _rings_mesh([(Z_W0, _ngon(WASHER_R)), (z1, _ngon(WASHER_R))])
    crown = (HEAD_AF - 2 * 0.0007) / HEAD_AF  # chamfer: the top ring pulled in 0.7 mm radially over HEAD_CH
    hx = _hex(HEAD_AF / 2)
    head = _rings_mesh([(z1, hx), (z1 + HEAD_K - HEAD_CH, hx),
                        (z1 + HEAD_K, [(x * crown, y * crown) for x, y in hx])])
    return [mk.hull(*washer, color=CHROME, mu=MU_METAL), mk.hull(*head, color=STEEL, mu=MU_METAL)]


def _socket_pieces():
    """Six cavity walls (each convex: outer arc, radial sides, one hex flat with a 45 deg mouth chamfer) and the
    solid above the cavity's ceiling."""
    pieces = []
    for k in range(6):
        a = k * math.pi / 3

        def ring(d):
            rv = d / _COS30
            outer = [(SOCK_R * math.cos(a + s), SOCK_R * math.sin(a + s)) for s in (-math.pi / 6, 0.0, math.pi / 6)]
            inner = [(rv * math.cos(a + math.pi / 6), rv * math.sin(a + math.pi / 6)),
                     (rv * math.cos(a - math.pi / 6), rv * math.sin(a - math.pi / 6))]
            return outer + inner

        wall = _rings_mesh([(SOCK_BOT, ring(SOCK_AF / 2 + MOUTH_CH)), (SOCK_BOT + MOUTH_CH, ring(SOCK_AF / 2)),
                            (SOCK_BOT + CAV, ring(SOCK_AF / 2))])
        pieces.append(mk.hull(*wall, color=CHROME, mu=MU_METAL))
    body = _rings_mesh([(SOCK_BOT + CAV, _ngon(SOCK_R)), (SOCK_BOT + SOCKET_H, _ngon(SOCK_R))])
    pieces.append(mk.hull(*body, color=CHROME, mu=MU_METAL))
    return pieces


kit = mk.Kit()
# the bolt: a free hex-head body on the thread hinge; +angle = clockwise seen from above = tightening
kit.part("lug_bolt", (BX, BY, ZLUG), _bolt_pieces(), mass=BOLT_KG, color=STEEL)
kit.hinge("bolt", None, "lug_bolt", anchor=(BX, BY, ZLUG), axis=(0, 0, -1), lower_deg=-30.0, upper_deg=90.0,
          friction=T0, damping=DAMP)
# the wrench head: socket + digital torque adapter + ratchet head + neck, one free body, socket-down on the bench
_hy = R_PIVOT  # head frame: origin on the socket axis at the centre plane, the handle along -y
kit.part("wrench_head", (SX, SY, ZH_START), yaw=SPAWN_YAW, boxes=[
    *_socket_pieces(),
    mk.box((XDCR[0], XDCR[0], XDCR[1]), at=(0, 0, -HEAD_T / 2 - XDCR[1] / 2), color=(0.15, 0.25, 0.45),
           mu=MU_METAL),  # digital peak-torque adapter body
    mk.box((0.032, 0.032, HEAD_T), mu=MU_METAL),  # ratchet head
    mk.box((0.016, _hy - 0.016, 0.012), at=(0, -(_hy + 0.016) / 2, 0), mu=MU_METAL),  # neck to the pivot
    mk.box((0.010, 0.004, 0.002), at=(0, 0.004, HEAD_T / 2 - 0.0005), color=RED),  # selector + angle marker:
    # a flush ring-style reverse selector (the low-profile pattern common on quarter-inch click
    # wrenches), 0.5 mm proud - visible, but nothing for a hold-down press to snag or pin
], mass=HEAD_KG, color=CHROME)
# handle frame: origin on the pivot, the tube running out along -y; the grip rests on the bench block
_tube_len = R_GRIP - GRIP[1] / 2 - R_PIVOT
kit.part("handle", (*_spun(0.0, -R_PIVOT), ZH_START), yaw=SPAWN_YAW, boxes=[
    mk.box((TUBE[0], _tube_len, TUBE[1]), at=(0, -_tube_len / 2, 0), mu=MU_METAL),
    mk.box(GRIP, at=(0, -(R_GRIP - R_PIVOT), 0), color=RUBBER),
    mk.box((0.012, 0.006, 0.012), at=(0, -(R_GRIP + GRIP[1] / 2 + 0.003 - R_PIVOT), 0), color=RED),  # lock knob
], mass=HANDLE_KG, color=STEEL)
# the detent pivot: handle on the head, held in line by the armed detent spring, its stop at 6 deg
kit.hinge("breakover", "wrench_head", "handle", anchor=(*_spun(0.0, -R_PIVOT), ZH_START), axis=(0, 0, -1),
          lower_deg=-1.0, upper_deg=BREAK_DEG, stiffness=K_DETENT / BREAKOVER_DRIVE, rest_deg=0.0, damping=DAMP)

FIXTURES = [
    Box("insulator_a", pos=(BX - 0.020, BY, TOP + 0.012), size=(0.020, 0.020, 0.024), color=EPOXY),
    Box("clinch_nut", pos=(BX, BY, TOP + 0.0215), size=(0.008, 0.008, 0.005), color=CHROME),  # M5, under the bar
    Box("insulator_b", pos=(BX + 0.075, BY, TOP + 0.012), size=(0.020, 0.020, 0.024), color=EPOXY),
    Box("busbar", pos=(BX + 0.030, BY, TOP + 0.027), size=(0.120, 0.020, 0.006), color=COPPER),
    Box("angle_reference", pos=(BX + 0.030, BY + 0.009, TOP + 0.0301),
        size=(0.030, 0.001, 0.0002), color=(0.95, 0.95, 0.95)),  # fixed camera-angle marker on busbar edge
    Box("lug_palm", pos=(BX + 0.003, BY, ZLUG - 0.0015), size=(0.018, 0.012, 0.003), color=TIN),
    Box("lug_barrel", pos=(BX + 0.023, BY, ZLUG + 0.001), size=(0.022, 0.008, 0.008), color=TIN),
    Box("cable", pos=(BX + 0.074, BY, ZLUG + 0.001), size=(0.080, 0.008, 0.008), color=PVC),
    Box("bench_rest", pos=(*_spun(0.0, -R_GRIP), TOP + 0.0315), size=(0.030, 0.030, 0.063), color=BLOCK),  # grip block
    # the handle rest: carries the grip while the wrench is on the bolt (the instrumented wrench cannot
    # balance unattended on a 3.5 mm head - a fitter rests the handle on the work). Its top sits half a
    # millimetre under the seated grip; it spans the grip's whole pull arc, so the grip rides along it, and
    # it reaches 30 mm past the grip on the +x side so the resting tool cannot roll over its edge (the
    # bolt-rim-plus-grip support line runs along y: the rest's width is the tool's lateral stance).
    Box("handle_rest", pos=(BX - 0.019, BY - 0.211, TOP + (ZH - GRIP[2] / 2 - TOP - 0.0005) / 2),
        size=(0.098, 0.040, ZH - GRIP[2] / 2 - TOP - 0.0005), color=BLOCK),
]
OBJECTS = [*kit.objects(), *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the mechanism's per-step rules (sim) and the scoring ---------------------------------------------------------
TURNED_DEG = 1.5  # partial credit: the bolt turned on from snug (under-torqued; 65 % of the set torque)
SETBACK_DEG = 1.0  # torqued: the bolt still within this of its tightest (it springs back up to 0.5 deg when the
# load comes off)
BACKOFF_DEG = 3.0  # turned back more than this from its tightest: loosened (latched fail)
REARM_STEPS = 4  # the detent re-arms no sooner than this many control steps after a click
STILL_DEG, STILL_M = 0.05, 0.0003  # still: the bolt and the grip moved under this in a control step
M = "cwsc"  # key of this task's per-episode memory in mk.MEMORY
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
_STICK = mk.STICK_RAD


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"n": 0, "init": False, "armed": True, "clicks": 0, "t_click": None, "peak": 0.0,
                            "over": False, "yielded": False, "loose": False, "still": True, "prev": None,
                            "k_bolt": None, "seat_steps": 0, "seated": False, "seated_now": False,
                            "click_seated": False, "preseat_yaw0": None, "preseat_turn": False}
    return m


def bolt_deg(objects) -> float:
    """How far the bolt has turned past snug (deg, + = tightened), from the bolt body's pose on its hinge."""
    return kit.q_deg(objects, "bolt")


def _cw_deg(r) -> float:
    """Clockwise turn (deg, seen from above) of a rotation matrix about the vertical."""
    return -math.degrees(math.atan2(float(r[1, 0]), float(r[0, 0])))


def wrench_yaw_deg(objects) -> float:
    """Clockwise yaw of the visible wrench-head marker against the fixed busbar/camera frame."""
    return _cw_deg(kit._pose_T(objects, "wrench_head")[:3, :3])


def _angle_delta_deg(a: float, b: float) -> float:
    """Shortest signed angular difference, robust across the +/-180 degree wrap."""
    return (a - b + 180.0) % 360.0 - 180.0


def break_deg(objects) -> float:
    """How far the handle is kinked on its pivot (deg, + = the tightening way), in the head's own frame."""
    rel = mk._inv(kit._pose_T(objects, "wrench_head")) @ kit._pose_T(objects, "handle")
    return _cw_deg(rel[:3, :3])


def grip_point(objects) -> np.ndarray:
    """The grip's centre (world)."""
    return kit.point(objects, "handle", (0.0, -(R_GRIP - R_PIVOT), 0.0))


def seat_geom(objects) -> tuple[float, float, float]:
    """(engagement m, tilt deg, coaxial offset m): how far the socket is down over the head (head top face above
    the socket's bottom rim), the socket axis off vertical, and the socket axis off the bolt axis at the head.
    All from the two bodies' poses: geometry a reviewer could measure on a photo."""
    tw = kit._pose_T(objects, "wrench_head")
    tb = kit._pose_T(objects, "lug_bolt")
    rim = tw[:3, :3] @ np.array([0.0, 0.0, SOCK_BOT]) + tw[:3, 3]
    head_top_z = float(tb[2, 3] + tb[2, 2] * HEAD_TOP)  # the bolt only yaws: its top face stays horizontal
    eng = head_top_z - float(rim[2])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(tw[2, 2])))))
    axis_pt = rim + tw[:3, 2] * max(eng, 0.0)  # the socket axis back up at the head's plane
    coax = float(math.hypot(axis_pt[0] - tb[0, 3], axis_pt[1] - tb[1, 3]))
    return eng, tilt, coax


def seated_now(objects) -> bool:
    eng, tilt, coax = seat_geom(objects)
    return bool(eng >= ENG_SEAT and tilt <= TILT_MAX and coax <= COAX_MAX)


def bolt_torque(deg: float) -> float:
    """The torque (N*m) that turns the bolt on at ``deg`` past snug: rising, flat once it yields."""
    return T0 + SLOPE * math.radians(min(max(deg, 0.0), YIELD_DEG))


def peak_pct(deg: float) -> float:
    """The torque that turned the bolt to ``deg`` past snug, in % of the set torque (what a peak-hold torque
    digital peak-torque adapter between the wrench and the socket would read on the real joint)."""
    return 100.0 * bolt_torque(deg) / T_CLICK


def _set_drive(name, target=None, stiffness=None) -> None:
    """Sim only: set joint ``name``'s drive target (rad) and/or stiffness (N*m/rad) now."""
    path = mk._SIM["joint_paths"].get((kit.id, name))
    if not path or mk._SIM.get("stage") is None:
        return
    from pxr import UsdPhysics

    scale = 180.0 / math.pi
    d = UsdPhysics.DriveAPI(mk._SIM["stage"].GetPrimAtPath(path), "angular")
    if target is not None:
        d.GetTargetPositionAttr().Set(float(target * scale))
    if stiffness is not None:
        d.GetStiffnessAttr().Set(float(stiffness / scale))


def _say(msg) -> None:
    if DEBUG and mk._EP.get("env") is not None:
        print(f"[mechkit cwsc] step {mk._EP.get('step')}: {msg}", flush=True)


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else _TEST_TCPS


CLEAR_M = 0.10  # let go: both grippers this far outside the wrench's bounding box


def _released(st, margin=CLEAR_M) -> bool:
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in _tcps())


def _rules(objects) -> None:
    """One control step of the bolt (its rising turning torque), the seating gate and the wrench's detent."""
    m = _mem()
    sim = mk._EP.get("env") is not None
    m["n"] += 1
    if not m["init"]:  # a new episode: the detent armed, the bolt at snug
        m["init"] = True
        _set_drive("breakover", target=0.0, stiffness=K_DETENT / BREAKOVER_DRIVE)
    if VARIANT == "sweep" and sim and SWEEP_T0[0] is not None:  # the stiffness-sweep probe drives the detent
        w_i = int(max(0.0, m["n"] / 20.0 - SWEEP_T0[0]) // SWEEP_WIN_S)
        if w_i != m.get("sweep_w"):
            m["sweep_w"] = w_i
            cmd = SWEEP_CMDS[min(w_i, len(SWEEP_CMDS) - 1)]
            _set_drive("breakover", target=0.0, stiffness=cmd)
            _say(f"SWEEP window {w_i}: commanded breakover stiffness {cmd:.3f} N*m/rad")
    th, d = bolt_deg(objects), break_deg(objects)
    head_yaw = wrench_yaw_deg(objects)
    eng, tilt, coax = seat_geom(objects)
    s_now = bool(eng >= ENG_SEAT and tilt <= TILT_MAX and coax <= COAX_MAX)
    m["seated_now"] = s_now
    # Begin at actual socket/head overlap, not during the free-space carry.  From first overlap until the full
    # seat has held for five frames, the independently visible wrench-head marker may not be turned.  Unlike the
    # old bolt-angle gate, this remains observable while the socket is only partly engaged.
    contact_envelope = bool(eng >= PRESEAT_ENG and tilt <= PRESEAT_TILT_MAX and coax <= PRESEAT_COAX_MAX)
    if not m["seated"]:
        if m["preseat_yaw0"] is None and contact_envelope:
            m["preseat_yaw0"] = head_yaw
        if (m["preseat_yaw0"] is not None
                and abs(_angle_delta_deg(head_yaw, m["preseat_yaw0"])) >= PRESEAT_TURN_DEG
                and not m["preseat_turn"]):
            m["preseat_turn"] = True
            _say(f"PRESEAT-TURN: wrench head moved "
                 f"{_angle_delta_deg(head_yaw, m['preseat_yaw0']):+.2f} deg before seating held "
                 f"{SEAT_STEPS} steps (engagement {eng * 1000:.2f} mm, tilt {tilt:.2f} deg, "
                 f"coax {coax * 1000:.2f} mm)")
    if s_now:
        m["seat_steps"] += 1
        if m["seat_steps"] >= SEAT_STEPS and not m["seated"]:
            m["seated"] = True
            _say(f"SEATED: engagement {eng * 1000:.2f} mm, tilt {tilt:.2f} deg, coax {coax * 1000:.2f} mm, "
                 f"held {SEAT_STEPS} steps")
    else:
        m["seat_steps"] = 0
    # the bolt: thread friction rising with the turn (the kit's friction hinge, its limit set from the angle)
    k = bolt_torque(th) / _STICK
    if m["k_bolt"] is None or abs(k - m["k_bolt"]) > 0.002 * k:
        m["k_bolt"] = k
        _set_drive("bolt", stiffness=k)
    m["peak"] = max(m["peak"], th)
    if th > TOL_HI_DEG and not m["over"]:
        m["over"] = True
        _say(f"OVER-TORQUED: bolt {th:.2f} deg past snug, {peak_pct(th):.0f} % of the set torque "
             f"(tolerance ends at 110 %, {TOL_HI_DEG:.1f} deg)")
    if th >= YIELD_DEG and not m["yielded"]:
        m["yielded"] = True
        _say(f"YIELDED: bolt {th:.2f} deg past snug (yields at {YIELD_DEG:.1f})")
    # Latch a back-off only after the bolt has first reached the torque-tolerance band.  Hex contact can nudge
    # the snug bolt in both directions while the socket's chamfers settle; that is not an undone torque if the
    # joint is subsequently tightened correctly.  A bolt left turned backwards is rejected directly by score().
    if m["peak"] >= TOL_LO_DEG and th < m["peak"] - BACKOFF_DEG and not m["loose"]:
        m["loose"] = True
        _say(f"LOOSENED: bolt {th:.2f} deg, its tightest was {m['peak']:.2f}")
    # the detent: breaks over at the set torque, re-arms once the handle is back in line
    if m["armed"] and d >= CLICK_DEG:
        m["armed"], m["t_click"] = False, m["n"]
        m["clicks"] += 1
        m["click_seated"] = m["click_seated"] or s_now
        if VARIANT != "sweep":
            _set_drive("breakover", target=0.0, stiffness=K_RETURN / BREAKOVER_DRIVE)
        _say(f"CLICK {m['clicks']}: handle {d:.2f} deg, bolt {th:.2f} deg past snug, seated {s_now}")
    elif not m["armed"] and d <= REARM_DEG and m["n"] - m["t_click"] >= REARM_STEPS:
        m["armed"] = True
        if VARIANT != "sweep":
            _set_drive("breakover", target=0.0, stiffness=K_DETENT / BREAKOVER_DRIVE)
        _say(f"re-armed: handle {d:.2f} deg, bolt {th:.2f} deg")
    g = grip_point(objects)
    if m["prev"] is not None:
        th0, g0 = m["prev"]
        m["still"] = bool(abs(th - th0) < STILL_DEG and np.linalg.norm(g - g0) < STILL_M)
    m["prev"] = (th, g)
    if DEBUG and sim:
        tc = mk.tcps()
        if tc:
            t = np.asarray(tc[1])
            a = math.radians(th)  # the hand's lead on the grip, along the grip's path (+ = pulling)
            tang = np.array([-math.cos(a), math.sin(a), 0.0])
            lead = float((t - g) @ tang)
            _say(f"R tcp ({t[0]:.4f},{t[1]:.4f},{t[2] - ZH:.4f}) lead {lead * 1000:.1f} mm; bolt {th:.2f} deg, "
                 f"handle {d:.2f} deg, eng {eng * 1000:.1f} mm tilt {tilt:.1f} coax {coax * 1000:.1f} mm "
                 f"seated {s_now}/{m['seated']}, armed {m['armed']}, clicks {m['clicks']}, peak {m['peak']:.2f} "
                 f"({peak_pct(m['peak']):.0f} %), over {m['over']} loose {m['loose']} "
                 f"preseat-turn {m['preseat_turn']} "
                 f"still {m['still']}")


kit.on_step(_rules)


def torqued(objects) -> bool:
    """In tolerance: the wrench clicked with the socket seated, the peak torque was 90-110 % of the set torque
    (the bolt's tightest 3.9-5.9 deg past snug) and the bolt is still within 1 deg of its tightest."""
    m = _mem()
    return bool(m["clicks"] > 0 and m["click_seated"] and TOL_LO_DEG <= m["peak"] <= TOL_HI_DEG
                and bolt_deg(objects) >= m["peak"] - SETBACK_DEG)


def score(objects) -> float:
    """0 (latched) if the wrench head ever moved 1 deg or more after partial engagement but before the full seat
    held for five steps, if the peak torque went over 110 % of the set torque (the bolt more than
    5.9 deg past snug), or if the bolt was turned back more than 3 deg from its tightest. Otherwise 1.0 for
    torqued (clicked while seated, peak 90-110 %, the bolt still within 1 deg of its tightest), else 0.25 for
    turned on at least 1.5 deg (under-torqued), else 0.1 for the socket seated (5 steps) but the bolt not turned.
    Halved unless both grippers are 10 cm outside the wrench's bounding box and the bolt and grip are still."""
    if mk._EP.get("env") is None and _mem().get("last") is not objects:  # offline (once per state); in sim the
        _mem()["last"] = objects  # per-step rule has already run
        _rules(objects)
    m = _mem()
    if m["over"] or m["loose"] or m["preseat_turn"] or bolt_deg(objects) <= -BACKOFF_DEG:
        return 0.0
    th = bolt_deg(objects)
    base = 1.0 if torqued(objects) else (0.25 if th >= TURNED_DEG else (0.1 if m["seated"] else 0.0))
    clear = _released(objects["wrench_head"]) and _released(objects["handle"]) and m["still"]
    return round(base if clear else base / 2, 6)


def success(objects) -> bool:
    """Torqued in tolerance (seated click, peak 90-110 %, not backed off), let go and still; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """3 on a latched fail; else the seating, the click and the turn still missing."""
    m = _mem()
    if m["over"] or m["loose"] or m["preseat_turn"] or bolt_deg(objects) <= -BACKOFF_DEG:
        return 3.0
    return float((not m["seated"]) + (m["clicks"] == 0)
                 + max(0.0, TOL_LO_DEG - bolt_deg(objects)) / TOL_LO_DEG)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry and the wrench's set torque, built around two rules a fitter follows
# with a heavy-headed wrench on a small vertical bolt: NEVER carry or leave it one-handed (the head's weight
# twists a single grip - measured, repeatably - and the instrumented wrench cannot balance unattended on
# a 3.5 mm head), and press the head down while pulling (without a hold-down the pull cams the loose hex fit
# over the head's corner before the bolt's snug friction gives). So: both hands lift the tool off its rest -
# the left pinching the steel neck, the right gripping the rubber grip - and carry it level over the joint,
# holding its clocking, then lower it together until the chamfers meet, the hex is home (no overdrive) and
# the grip is down on the HANDLE REST. The right keeps hold while the left swaps to pressing the head down
# on the drive axis (flush ring selector: nothing to snag; sliding friction at a few millimetres' radius
# costs a few hundredths of a N*m on the click). Only then does the right let go, straddle the grip (pads
# 36 mm apart: side pushes only, so no finger moment bypasses the breakover and the detent carries the full
# drive torque; rest friction acts at the pull radius and cancels) and pull it round toward the robot. The
# spec angle (4.9 deg), breakover deflection (1.5 deg), straddle slack (~2.4 deg) and the arm's lag put the
# click at a commanded ~11-13 deg; the hand stops at 15 deg - inside click + the handle's 6 deg free swing,
# so the bolt is never driven past - eases back 1.5 deg, opens and parks; the left lifts off the head and
# parks; the torqued wrench stays seated on the bolt with its grip on the rest through the success hold.
# VARIANT "pretest" is a carry-robustness self-check: two grasp-lift-carry-hold-return cycles, the second
# with both grasp points deliberately offset, never touching the bolt.

TIP = 0.013  # pad tips below the TCP
PAD_DOWN = 0.004  # right grip: pad tips this far below the grip's centre plane - the proven pushing height
PAD_DOWN_L = 0.007  # left pinch: pads a little lower round the 12 mm neck (full-height contact patch)
R_NECK = 0.040  # the left pinch point on the neck, just outboard of the head
NECK_CLOSE = -0.004  # fingers closed 8 mm inside the 16 mm neck: a firm pinch
SEAT_AIM = (0.0034, 0.0059)  # carry aim: the two laden wrists settle this far off the commanded point
# (carry-pretest telemetry, repeatable across cycles); the bolt-side targets aim the same distance the other
# way so the socket arrives centred over the head
SAG_Z = 0.004  # the seat heights command this much low as a minimum-sag allowance. Any extra wrist sag just
# becomes command-overdrive, which is safe here: the supported tool cannot pitch or skate (both hands and the
# handle rest hold it level - the rest squares the handle as the grip lands), so surplus command turns into a
# firm, aligned preload of the rim on the washer
GRIP_CLOSE = -0.006  # right hand on the 18 mm rubber grip for the carry: pads 3 mm into the rubber, firm
STRADDLE = -0.018  # right finger target: pads 36 mm apart round the 18 mm grip (9 mm play each side). The
# straddle pushes only on the grip's side, so no finger moment reaches the handle: a firm pinch would couple
# the wrist's stiffness straight past the breakover (measured: the detent never fires through a pinch).
WIDE = -0.030
PULL_DEG = {"demo": 20.5, "half": 8.0, "through": 45.0, "short": 6.0, "sweep": 16.0}  # the demo stops at
# 20.5: the click is torque-driven at the bolt's ~4.9 deg (it fired at bolt 4.41-4.71 in the recorded runs,
# two to three degrees of hand travel before the stop) and the handle's 6 deg free swing absorbs the rest of
# the hand's travel, so 20.5 cannot drive the bolt past the click
PULL_S = {"demo": 6.0, "half": 6.0, "through": 12.0, "short": 3.0, "sweep": 24.0}
EASE_DEG = 1.5
STEP_DEG = 1.5
PARK_L = (0.150, 0.250, 0.950)
PARK_R = (0.150, -0.350, TOP + 0.200)  # 5 cm further out: the pulled grip must end over 10 cm from the hand
L_OPEN = -0.04695
LIFT = 0.070  # straight up off the grip after letting go, before moving away to PARK_R




def _grip_xy(deg):
    a = math.radians(deg)
    return (BX - R_GRIP * math.sin(a), BY - R_GRIP * math.cos(a))


def demo():
    """Both hands move the tool: the left pinches the neck, the right grips the handle; together they lift
    the wrench off its rest, carry it level and lower it until the hex is home and the grip is on the handle
    rest. The right keeps hold while the left swaps to pressing the head down on the drive axis; only then
    does the right let go, straddle the grip and pull it round to the click."""
    tzs = ZH_START - PAD_DOWN_L + TIP  # left TCP, neck grasp, tool on its rest
    tgs = ZH_START - PAD_DOWN + TIP  # right TCP, grip grasp, tool on its rest
    tzb = ZH - PAD_DOWN_L + TIP  # left TCP, socket seated
    tzr = ZH - PAD_DOWN + TIP  # right TCP, socket seated: the pull height
    n0, g0 = _spun(0.0, -R_NECK), _spun(0.0, -R_GRIP)
    nb = (BX + SEAT_AIM[0], BY - R_NECK + SEAT_AIM[1])
    gb = (BX + SEAT_AIM[0], BY - R_GRIP + SEAT_AIM[1])
    YL, YR = -math.pi / 2, math.pi / 2
    L, R = [], []

    def two(dur, lpos, lfin, rpos, rfin, label):
        L.append(Waypoint(dur, lpos, lfin, label, YL))
        R.append(Waypoint(dur, rpos, rfin, label, YR))

    def take(n, g, tag=""):
        two(2.0, (*n, tzs + 0.055), WIDE, (*g, tgs + 0.055), WIDE, tag + "over the neck and the grip")
        two(1.5, (*n, tzs), WIDE, (*g, tgs), WIDE, tag + "down round them")
        two(0.5, (*n, tzs), WIDE, (*g, tgs), WIDE, tag + "settle")
        two(0.8, (*n, tzs), NECK_CLOSE, (*g, tgs), GRIP_CLOSE, tag + "both hands take the tool")

    if VARIANT == "pretest":  # carry-robustness self-check: never touches the bolt
        for k, (dn, dg) in enumerate([((0.0, 0.0), (0.0, 0.0)), ((0.002, 0.0), (0.0, 0.003))]):
            t = f"cycle {k}: "
            n = (n0[0] + dn[0], n0[1] + dn[1])
            g = (g0[0] + dg[0], g0[1] + dg[1])
            take(n, g, t)
            two(1.2, (*n, tzs + 0.020), NECK_CLOSE, (*g, tgs + 0.020), GRIP_CLOSE, t + "lift 2 cm")
            two(1.0, (*n, tzs + 0.020), NECK_CLOSE, (*g, tgs + 0.020), GRIP_CLOSE, t + "hold")
            two(1.2, (*n, tzs + 0.040), NECK_CLOSE, (*g, tgs + 0.040), GRIP_CLOSE, t + "higher")
            two(3.0, (*nb, tzb + 0.030), NECK_CLOSE, (*gb, tzr + 0.030), GRIP_CLOSE, t + "carry over the bolt")
            two(1.5, (*nb, tzb + 0.030), NECK_CLOSE, (*gb, tzr + 0.030), GRIP_CLOSE, t + "hold over the bolt")
            two(3.0, (*n, tzs + 0.040), NECK_CLOSE, (*g, tgs + 0.040), GRIP_CLOSE, t + "carry back")
            two(1.5, (*n, tzs + 0.003), NECK_CLOSE, (*g, tgs + 0.003), GRIP_CLOSE, t + "set it down")
            two(0.8, (*n, tzs + 0.003), WIDE, (*g, tgs + 0.003), WIDE, t + "let go")
            two(1.0, (*n, tzs + 0.045), WIDE, (*g, tgs + 0.045), WIDE, t + "lift away")
        L.append(Waypoint(2.0, PARK_L, L_OPEN, "park", 0.0))
        R.append(Waypoint(2.0, PARK_R, WIDE, "park", YR))
        return {"left": L, "right": R}

    if VARIANT == "half":  # the seating ablation aims 2.5 mm off the bolt axis - outside the chamfers'
        # capture - so the rim jams on the head cocked and part-engaged, never seated (coax > 1 mm)
        nb = (nb[0], nb[1] + 0.0025)
        gb = (gb[0], gb[1] + 0.0025)
    take(n0, g0)
    two(1.5, (*n0, tzs + 0.040), NECK_CLOSE, (*g0, tgs + 0.040), GRIP_CLOSE, "lift the tool off its rest")
    two(3.0, (*nb, tzb + 0.030), NECK_CLOSE, (*gb, tzr + 0.030), GRIP_CLOSE, "carry it level over the bolt")
    two(1.0, (*nb, tzb + 0.030), NECK_CLOSE, (*gb, tzr + 0.030), GRIP_CLOSE, "settle over the head")
    two(2.5, (*nb, tzb + 0.004 + SAG_Z), NECK_CLOSE, (*gb, tzr + 0.004 + SAG_Z), GRIP_CLOSE,
        "lower together: the chamfers meet" if VARIANT != "half" else "lower together, aimed off the axis")
    pull = PULL_DEG[VARIANT]

    def w(dur, deg, z, f, label):
        x, y = _grip_xy(deg)
        R.append(Waypoint(dur, (x, y, z), f, label, math.pi / 2 - math.radians(deg)))

    zarc = tzr + (0.0045 if VARIANT == "half" else 0.0)  # half pulls at its cocked-hold height

    def arc(a, b, dur, label):
        n = max(1, math.ceil(abs(b - a) / STEP_DEG))
        for k in range(1, n + 1):
            w(dur / n, a + (b - a) * k / n, zarc, STRADDLE, label)

    if VARIANT == "half":  # the cocked, part-engaged hold; the left keeps it there while the right pulls
        two(2.0, (*nb, tzb + 0.0045), NECK_CLOSE, (*gb, tzr + 0.0045), GRIP_CLOSE,
            "press it down cocked on the head: part-engaged, never seated")
        L.append(Waypoint(1.0, (*nb, tzb + 0.0045), NECK_CLOSE, "hold it there", YL))
        w(0.6, 0.0, tzr + 0.0045, STRADDLE, "open to a straddle")
        w(0.4, 0.0, tzr + 0.0045, STRADDLE, "settle")
    else:
        two(2.0, (*nb, tzb + 0.0005 + SAG_Z), NECK_CLOSE, (*gb, tzr + 0.0005 + SAG_Z), GRIP_CLOSE,
            "ease the hex home: the grip comes down on the handle rest")
        two(0.5, (*nb, tzb + 0.0005 + SAG_Z), NECK_CLOSE, (*gb, tzr + 0.0005 + SAG_Z), GRIP_CLOSE, "hold")
        zpress = ZH + HEAD_T / 2 + TIP - 0.004  # closed pad tips pressed firmly into the head's top face
        L.append(Waypoint(1.0, (*nb, tzb + 0.0005 + SAG_Z), WIDE, "left lets go: the right still holds", YL))
        L.append(Waypoint(1.0, (*nb, tzb + 0.045), WIDE, "left lifts clear", YL))
        L.append(Waypoint(1.5, (BX, BY, zpress + 0.030), 0.0, "fingers closed, over the wrench head", YL))
        L.append(Waypoint(1.0, (BX, BY, zpress), 0.0, "press the head down onto the joint, firmly", YL))
        R.append(Waypoint(4.5, (*gb, tzr + 0.0005 + SAG_Z), GRIP_CLOSE,
                          "keep the tool supported until the press is on", YR))
        arcT = {"demo": 6.0, "through": 19.0, "short": 3.0, "sweep": 24.0}[VARIANT]
        L.append(Waypoint(arcT + 3.0, (BX, BY, zpress), 0.0, "hold the head down while the right pulls", YL))
        w(0.6, 0.0, tzr, STRADDLE, "open to a straddle")
        w(0.4, 0.0, tzr, STRADDLE, "settle")
    if VARIANT == "sweep":
        SWEEP_T0[0] = sum(x.duration for x in R)
    if VARIANT == "through":  # the naive ablation: a pause where the demo stops, then on round to 45 deg
        arc(0.0, PULL_DEG["demo"], PULL_S["demo"], "pull the grip round, steadily")
        w(1.0, PULL_DEG["demo"], tzr, STRADDLE, "pause where the demo stops")
        arc(PULL_DEG["demo"], pull, PULL_S[VARIANT], "pull on through the click")
    else:
        arc(0.0, pull, PULL_S[VARIANT], "pull the grip round, steadily")
    w(0.6, pull, tzr, STRADDLE, "stop: it clicked")
    w(0.6, pull - EASE_DEG, tzr, STRADDLE, "ease off")
    w(0.5, pull - EASE_DEG, tzr, WIDE, "open the fingers")
    w(1.0, pull - EASE_DEG, tzr + LIFT, WIDE, "lift clear")
    R.append(Waypoint(2.0, PARK_R, WIDE, "move away (over 10 cm from the wrench)", math.pi / 2))
    R.append(Waypoint(2.5, PARK_R, WIDE, "wait", math.pi / 2))
    if VARIANT != "half":
        L.append(Waypoint(0.8, (BX, BY, ZH + HEAD_T / 2 + TIP + 0.036), 0.0, "lift off the head", YL))
        L.append(Waypoint(2.0, PARK_L, L_OPEN, "park", 0.0))
    tl, tr = sum(x.duration for x in L), sum(x.duration for x in R)
    if tr > tl:
        last = L[-1]
        L.append(Waypoint(tr - tl, last.pos, last.finger, "wait", last.yaw))
    elif tl > tr:
        last = R[-1]
        R.append(Waypoint(tl - tr, last.pos, last.finger, "wait", last.yaw))
    return {"left": L, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built sequences of poses (one trial each, fed one control step at a time)."""
    global _TEST_TCPS
    on = (*_grip_xy(0.0), ZH - PAD_DOWN + TIP)  # a gripper still round the grip

    def state(th, d, eng=0.0034, tilt=0.0, coax=0.0, head_deg=None):
        """The wrench posed on (or over) the bolt with ``eng`` engagement, turned with the bolt to ``th``."""
        rim_z = ZLUG + HEAD_TOP - eng
        head_deg = th if head_deg is None else head_deg
        r = mk.axis_angle((1, 0, 0), math.radians(tilt)) @ mk.axis_angle((0, 0, -1), math.radians(head_deg))
        pos = (BX + coax, BY, rim_z - SOCK_BOT)
        return kit.fake_states(q={"bolt": math.radians(th), "breakover": math.radians(d)},
                               moved={"wrench_head": (np.array(pos), mk.mat_to_quat(r))})

    def bench(th=0.0, d=0.0):
        """The wrench untouched on its bench rest."""
        return kit.fake_states(q={"bolt": math.radians(th), "breakover": math.radians(d)})

    def lin(a, b, n=8):
        return [a + (b - a) * k / n for k in range(n + 1)]

    def load(th):  # the detent's bend while the bolt turns at ``th`` (armed, pulled at the grip's centre)
        return math.degrees(bolt_torque(th) * (R_GRIP - R_PIVOT) / R_GRIP / K_DETENT)

    def pull(a, b):  # the bolt turned on from a to b through the armed detent (no click on the way)
        return [(t, min(load(t), CLICK_DEG - 0.01)) for t in lin(a, b)]

    def click(th):  # the detent breaks over at the set torque, the handle swings to its stop
        return [(th, CLICK_DEG + 0.05), (th, 4.0), (th, BREAK_DEG), (th, BREAK_DEG)]

    def let_go(th, d0=BREAK_DEG, n=6):  # load off: the handle springs back in line from d0, then all still
        return [(th, d) for d in lin(d0, 0.0, 4)[1:]] + [(th, 0.0)] * n

    APPROACH = [("bench", 0.0, 0.0, 0.0), ("hover", 0.0, 0.0, -0.010), ("lower", 0.0, 0.0, 0.0010),
                ("lower", 0.0, 0.0, 0.0022)] + [("seat", 0.0, 0.0, 0.0034)] * (SEAT_STEPS + 1)

    def frames(seq, eng=0.0034):  # torque frames (th, d), played out seated after the approach
        return APPROACH + [("seat", th, d, eng) for th, d in seq]

    s = SPEC_DEG
    good = frames(pull(0.0, s) + click(s + 0.1) + let_go(s + 0.1))
    cases = {  # name: (frames, TCPs at the end, score)
        "on its bench rest, untouched": ([("bench", 0.0, 0.0, 0.0)] * 5, [], 0.0),
        "carried over the bolt, hovering, never seated": (APPROACH[:3], [], 0.0),
        "seated and left alone (no turn, no click)": (frames([]), [], 0.1),
        "seated and left alone, hand still on the grip": (frames([]), [on], 0.05),
        "seated, pulled until it clicked, stopped, let go (demo)": (good, [], 1.0),
        "the same, the hand still on the grip": (good, [on], 0.5),
        "the same, the open hand 5 cm above the grip": (good, [(on[0], on[1], on[2] + 0.05)], 0.5),
        "the same, the grip still swinging back at the end": (frames(pull(0.0, s) + click(s + 0.1)
                                                                     + [(s + 0.1, 3.0)]), [], 0.5),
        "half-seated (1.5 mm): wrench turns 2 deg before the bolt moves": (
            APPROACH[:4] + [("seat", 0.0, 0.5, 0.0015, 0.0, 0.0, t) for t in lin(0.0, 2.0)], [], 0.0),
        "socket 1.5 mm off axis, pushed and twisted 1.5 deg": (
            APPROACH[:3] + [("miss", 0.0, 0.0, 0.003, 0.0, 0.0015, t) for t in lin(0.0, 1.5)], [], 0.0),
        "socket tilted 10 deg, wrench twisted 1.5 deg before seating": (
            APPROACH[:3] + [("tilt", 0.0, 0.0, 0.0030, 10.0, 0.0, t) for t in lin(0.0, 1.5)], [], 0.0),
        "stopped short: bolt 3.5 deg on, no click (under-torqued)": (frames(pull(0.0, 3.5)
                                                                            + let_go(3.5, load(3.5))), [], 0.25),
        "pulled on through the click to 20 deg (yielded)": (frames(pull(0.0, s) + click(s + 0.1)
                                                                   + [(t, BREAK_DEG) for t in lin(s + 0.1, 20.0)]
                                                                   + let_go(20.0)), [], 0.0),
        "naive: swung the handle round 80 deg": (frames(pull(0.0, s) + click(s)
                                                        + [(t, BREAK_DEG) for t in lin(s, 80.0)]
                                                        + let_go(80.0)), [], 0.0),
        "clicked, springs back 0.5 deg as the hand lets go": (frames(pull(0.0, s) + click(s + 0.1)
                                                                     + let_go(s - 0.4)), [], 1.0),
        "pulled on 0.8 deg past the click (106 %, inside the tolerance)": (
            frames(pull(0.0, s) + click(s) + [(t, BREAK_DEG) for t in lin(s, s + 0.8)] + let_go(s + 0.8)), [], 1.0),
        "pulled on 1.2 deg past the click (112 %, over the tolerance)": (
            frames(pull(0.0, s) + click(s) + [(t, BREAK_DEG) for t in lin(s, s + 1.2)] + let_go(s + 1.2)), [], 0.0),
        "pulled on 4 deg past the click (128 %)": (
            frames(pull(0.0, s) + click(s) + [(t, BREAK_DEG) for t in lin(s, s + 4.0)] + let_go(s + 4.0)), [], 0.0),
        "pulled on to 14.5 deg, just under the yield (197 %)": (
            frames(pull(0.0, s) + click(s) + [(t, BREAK_DEG) for t in lin(s, 14.5)] + let_go(14.5)), [], 0.0),
        "over by 1.2 deg, then eased back into the band (latched)": (
            frames(pull(0.0, s) + click(s) + [(t, BREAK_DEG) for t in lin(s, s + 1.2)]
                   + [(t, -1.0) for t in lin(s + 1.2, s + 0.4)] + let_go(s + 0.4, -1.0)), [], 0.0),
        "clicked, then backed off to 1.5 deg (torque undone)": (
            good + [("seat", t, -1.0, 0.0034) for t in lin(s + 0.1, 1.5)]
            + [("seat", t, d, 0.0034) for t, d in let_go(1.5, -1.0)], [], 0.0),
        "clicked, then eased back 2 deg (now under the torque)": (
            good + [("seat", t, -1.0, 0.0034) for t in lin(s + 0.1, s - 1.9)]
            + [("seat", t, d, 0.0034) for t, d in let_go(s - 1.9, -1.0)], [], 0.25),
        "loosened from snug: turned 4 deg the wrong way": (frames([(t, -1.0) for t in lin(0.0, -4.0)]
                                                                  + let_go(-4.0, -1.0)), [], 0.0),
        "turned 4.5 deg by the head, no click (torque not read)": (frames([(t, 0.0) for t in lin(0.0, 4.5)]
                                                                          + let_go(4.5, 0.0)), [], 0.25),
        "turned 8 deg by the head, no click (126 %, over)": (frames([(t, 0.0) for t in lin(0.0, 8.0)]
                                                                    + let_go(8.0, 0.0)), [], 0.0),
        "clicked, let go, clicked again to check (bolt 5.3 deg)": (good + [("seat", t, d, 0.0034) for t, d in
                                                                           pull(s + 0.1, s + 0.4)]
                                                                   + [("seat", t, d, 0.0034) for t, d in
                                                                      click(s + 0.4) + let_go(s + 0.4)], [], 1.0),
        "a jerk kinks the handle with the bolt only 2 deg on": (frames(pull(0.0, 2.0) + click(2.0)
                                                                       + let_go(2.0)), [], 0.25),
        "clicked, then the wrench lifted back off (bolt stays tight)": (good + [("hover", s + 0.1, 0.0, -0.010)] * 3,
                                                                        [], 1.0),
    }

    def to_state(f):
        tag, th, d, eng = f[0], f[1], f[2], f[3]
        if tag == "bench":
            return bench(th, d)
        tilt = f[4] if len(f) > 4 else 0.0
        coax = f[5] if len(f) > 5 else 0.0
        head_deg = f[6] if len(f) > 6 else None
        return state(th, d, eng=eng, tilt=tilt, coax=coax, head_deg=head_deg)

    def norm(fr):  # frames may be (tag, th, d, eng[, tilt[, coax]]) or plain torque frames (th, d)
        return [f if isinstance(f[0], str) else ("seat", f[0], f[1], 0.0034) for f in fr]

    try:
        st = state(3.0, 2.0)
        assert abs(break_deg(st) - 2.0) < 1e-6 and abs(bolt_deg(st) - 3.0) < 1e-6
        eng, tilt, coax = seat_geom(st)
        assert abs(eng - 0.0034) < 1e-6 and tilt < 1e-6 and coax < 1e-6, (eng, tilt, coax)
        assert not seated_now(state(0.0, 0.0, eng=0.0015)) and seated_now(state(0.0, 0.0, eng=0.0030))
        assert not seated_now(state(0.0, 0.0, tilt=10.0)) and not seated_now(state(0.0, 0.0, coax=0.005))
        assert not seated_now(bench())
        for name, (fr, tcps, want) in cases.items():
            fr = norm(fr)
            mk.new_episode()
            _TEST_TCPS = []
            for f in fr[:-1]:
                score(to_state(f))
            _TEST_TCPS = list(tcps)
            st = to_state(fr[-1])
            got = score(st)
            assert abs(got - want) < 1e-9, (name, got, want, _mem())
            assert success(st) == (want >= 1.0), name
        mk.new_episode()
        _TEST_TCPS = []
        for f in norm(good):
            score(to_state(f))
        clicks = _mem()["clicks"]
        mk.new_episode()
        assert score(bench()) == 0.0 and _mem()["clicks"] == 0, "a new trial forgets the last one"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1, clicks


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    n, clicks = self_test()
    print(f"scoring: {n} cases pass (set torque {T_CLICK:.2f} N*m = {F_CLICK:.1f} N at the grip, reached "
          f"{SPEC_DEG:.1f} deg past snug; tolerance 90-110 % = {TOL_LO_DEG:.1f}-{TOL_HI_DEG:.1f} deg; yield "
          f"{T_OVER:.2f} N*m = {F_OVER:.1f} N at {YIELD_DEG:.1f} deg; real joint x{1 / TORQUE_SCALE:.1f}: "
          f"{REAL_SET_NM:.0f} N*m set; seated = {ENG_SEAT * 1000:.1f} of {ENG_MAX * 1000:.1f} mm engaged, "
          f"tilt <= {TILT_MAX:.0f} deg, coax <= {COAX_MAX * 1000:.0f} mm)")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
