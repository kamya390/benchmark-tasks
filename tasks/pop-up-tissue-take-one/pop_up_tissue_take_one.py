"""On the station table stands a long flat pop-up tissue box (printed paperboard, 28 x 14 x 5.5 cm, 40 g with its last
few sheets) and, to the right, a shallow tray. The box's slot is a plastic film slit across the top: one edge of the
slit is the board, the other a light sprung film strip that presses whatever passes through it against the board
(about 0.14 N). The top tissue stands 25 mm up out of the slit. Inside, it runs down through the slit, bends round
the leading edge of the next tissue and lies under it, interleaved as in a real pop-up box; the next tissue's
leading edge waits just inside the slit, pressed against the top one.

Both tissues are real soft bodies (PhysX surface-deformable cloth, about 700 mesh nodes each, creased as folded),
not rigid parts, and nothing in the box is scripted: pull the top tissue up and the film slit squeezes it against
the next one, so friction drags the next tissue's edge up through the slit after it; once the top one's end has
passed, the film holds the next one standing there, ready. The same squeeze drags on the box: it weighs only 40 g,
so if nobody holds it down, pulling the tissue lifts the box off the table with it and tips it.

The job: take one tissue out and put it on the tray, leaving the box where it is with the next tissue ready to pull.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/pop-up-tissue-take-one/pop_up_tissue_take_one.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (the box and its sprung film strip, per-episode memory, the grippers' TCPs)
import softkit as sk  # noqa: E402  (the two tissues: PhysX surface-deformable cloth, read as mesh nodes)

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = False  # sim only: log the box pose and both tissues' heights every 10 steps to ~/dbg/b18-tissue.jsonl

# Frame: x forward from the arm bases, y left, z up, metres. Left arm base (0, +0.305, 0.75), right (0, -0.305).
XB, YB = 0.280, -0.100  # the slit's centre on the table (the box's origin)
# box-local geometry: origin on the box's bottom under the slit's centre, the slit along x at y = 0, 6 cm from the
# box's right end
BX, BH, WALL, LID = 0.140, 0.055, 0.003, 0.003
Y0, Y1 = -0.060, 0.220  # the box's ends (28 cm long)
BY, YM = Y1 - Y0, (Y0 + Y1) / 2
ZT, ZL = BH, BH - LID  # lid top, lid underside
SLX = 0.040  # slit half length
FACE = -0.003  # the board edge of the slit (fixed)
LIP_T, LIP_H = 0.004, 0.010  # film strip: thickness (y), depth (z, from the lid top down)
D0 = 0.006  # film strip travel at spawn (both tissues in the slit)
STACK_TOP = 0.035  # top of the remaining stack (rigid, part of the box)
BOX_KG = 0.040
CARD, WHITE, FILM, TRAY_C = (0.30, 0.50, 0.72), (0.97, 0.97, 0.95), (0.80, 0.84, 0.88), (0.55, 0.55, 0.58)


def _box_boxes():
    b = [mk.box((BX - 2 * WALL, BY - 2 * WALL, STACK_TOP), at=(0, YM, STACK_TOP / 2), color=WHITE)]
    for s in (-1, 1):
        b.append(mk.box((WALL, BY, ZL), at=(s * (BX / 2 - WALL / 2), YM, ZL / 2)))
        b.append(mk.box((BX - 2 * WALL, WALL, ZL), at=(0, YM + s * (BY / 2 - WALL / 2), ZL / 2)))
    ym0, yp0 = -0.006, 0.016  # lid opening in y
    b.append(mk.box((BX, ym0 - Y0, LID), at=(0, (Y0 + ym0) / 2, ZL + LID / 2)))
    b.append(mk.box((BX, Y1 - yp0, LID), at=(0, (Y1 + yp0) / 2, ZL + LID / 2)))
    for s in (-1, 1):
        b.append(mk.box((BX / 2 - SLX, yp0 - ym0, LID), at=(s * (BX / 2 + SLX) / 2, (yp0 + ym0) / 2, ZL + LID / 2)))
    b.append(mk.box((2 * SLX, FACE - ym0, LIP_H), at=(0, (FACE + ym0) / 2, ZT - LIP_H / 2), color=FILM))
    return b


def _arc(cy, cz, r, a0, a1, n=8):
    return [(cy + r * math.cos(a0 + (a1 - a0) * k / n), cz + r * math.sin(a0 + (a1 - a0) * k / n))
            for k in range(n + 1)]


ZC = ZT - LIP_H  # centre of both bends
TAB = 0.025  # the top tissue stands this far above the lid
# cross-sections (box-local y, z): top tissue = tab, down the slit, round the next one's edge, flat under it
PATH_1 = tuple([(-0.001, ZT + TAB)] + _arc(0.007, ZC, 0.008, math.pi, 1.5 * math.pi) + [(0.037, ZC - 0.008)])
PATH_2 = tuple([(0.003, ZT - 0.001)] + _arc(0.007, ZC, 0.004, math.pi, 1.5 * math.pi) + [(0.073, ZC - 0.004)])
SHEET_W = 0.070
CLOTH = dict(kind="cloth", friction=3.0, edge=0.003, color=WHITE)

kit = mk.Kit()
kit.part("box", (XB, YB, TOP), _box_boxes(), mass=BOX_KG, color=CARD)
kit.part("film", (XB, YB + FACE + 0.002 + LIP_T / 2, TOP + ZT - LIP_H / 2), [mk.box((2 * SLX, LIP_T, LIP_H))],
         mass=0.003, color=FILM)
kit.slider("film_slide", "box", "film", (0, 1, 0), 0.0, 0.012, start=D0, stiffness=20.0, rest=0.0)
TISSUE_1 = sk.Soft("tissue_1", pos=(XB, YB, TOP), size=(SHEET_W, sk.path_length(PATH_1), 0.001), profile=PATH_1,
                   **CLOTH)
TISSUE_2 = sk.Soft("tissue_2", pos=(XB, YB, TOP), size=(SHEET_W, sk.path_length(PATH_2), 0.001), profile=PATH_2,
                   **CLOTH)

# the tray, right of the box
TRX, TRY = 0.260, -0.250
TRAY_IN = (0.150, 0.120)
TRAY_T, RIM_H, RIM_W = 0.006, 0.012, 0.004
TRAY_TOP = TOP + TRAY_T


def _tray():
    out = [Box("tray_base", pos=(TRX, TRY, TOP + TRAY_T / 2), size=(TRAY_IN[0] + 2 * RIM_W, TRAY_IN[1] + 2 * RIM_W,
                                                                    TRAY_T), color=TRAY_C)]
    zc = TRAY_TOP + RIM_H / 2
    for s in (-1, 1):
        out.append(Box(f"tray_rim_x{'np'[s > 0]}", pos=(TRX + s * (TRAY_IN[0] / 2 + RIM_W / 2), TRY, zc),
                       size=(RIM_W, TRAY_IN[1] + 2 * RIM_W, RIM_H), color=TRAY_C))
        out.append(Box(f"tray_rim_y{'np'[s > 0]}", pos=(TRX, TRY + s * (TRAY_IN[1] / 2 + RIM_W / 2), zc),
                       size=(TRAY_IN[0], RIM_W, RIM_H), color=TRAY_C))
    return out


OBJECTS = [*kit.objects(), TISSUE_1, TISSUE_2, *_tray()]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring, from the box's pose and the tissues' mesh nodes ---------------------------------------------------
LIFT_TOL = 0.005  # the box's bottom never more than 5 mm off the table ...
TILT_TOL = 5.0  # ... nor tilted more than 5 deg, at any step (latched)
SPOT_TOL = 0.015  # at the end, the box within 1.5 cm of where it stood
READY_H = 0.010  # the next tissue stands at least 10 mm above the lid ...
IN_FRAC = 0.25  # ... with at least a quarter of its nodes still under the lid (it is still in the box)
TRAY_FRAC = 0.80  # on the tray: 80 % of the tissue's nodes over the tray's floor, none higher than ...
TRAY_ABOVE = 0.040  # ... 4 cm over it (lying there, not held)
CLEAR = 0.03  # let go: both TCPs at least 3 cm from every tissue node and from the box
STILL = 0.02  # m/s, every node (from node displacement between control steps)
M = "puto"  # this task's key in the per-episode memory
_TEST_TCPS = None  # self-test stand-in for the TCPs


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"lifted": False, "two_out": False}
    return m


def _box_frame(objects):
    pos, q = kit.pose(objects, "box")
    r = mk.quat_to_mat(q)
    return np.asarray(pos, dtype=float), r


def box_tilt_deg(objects) -> float:
    _, r = _box_frame(objects)
    return float(math.degrees(math.acos(max(-1.0, min(1.0, r[2, 2])))))


def box_lift(objects) -> float:
    """How far the box's lowest corner is above the table (m)."""
    pos, r = _box_frame(objects)
    corners = np.array([(sx * BX / 2, sy, 0.0) for sx in (-1, 1) for sy in (Y0, Y1)]) @ r.T + pos
    return float(corners[:, 2].min() - TOP)


def local(objects, name) -> np.ndarray:
    """A tissue's nodes in the box frame (origin on its bottom under the slit's centre, slit along x)."""
    pos, r = _box_frame(objects)
    return (sk.nodes(objects[name]) - pos) @ r


def in_box(p) -> np.ndarray:
    """Nodes inside the box, under the lid."""
    return (np.abs(p[:, 0]) < BX / 2) & (p[:, 1] > Y0) & (p[:, 1] < Y1) & (p[:, 2] < ZL)


def first_out(objects) -> bool:
    """The top tissue is out: none of its nodes is inside the box under the lid."""
    return not in_box(local(objects, "tissue_1")).any()


def on_tray(objects, name) -> bool:
    p = sk.nodes(objects[name])
    over = (np.abs(p[:, 0] - TRX) <= TRAY_IN[0] / 2) & (np.abs(p[:, 1] - TRY) <= TRAY_IN[1] / 2)
    return bool(over.mean() >= TRAY_FRAC and p[:, 2].max() <= TRAY_TOP + TRAY_ABOVE)


def next_height(objects) -> float:
    """How far the next tissue's highest node stands above the lid (m, box frame)."""
    return float(local(objects, "tissue_2")[:, 2].max() - ZT)


def next_ready(objects) -> bool:
    """The next tissue stands at least 10 mm above the lid and is still in the box (a quarter of it under the lid)."""
    p = local(objects, "tissue_2")
    return bool(p[:, 2].max() - ZT >= READY_H and in_box(p).mean() >= IN_FRAC)


def next_out(objects) -> bool:
    """The next tissue has left the box: none of its nodes inside under the lid."""
    return not in_box(local(objects, "tissue_2")).any()


def released(objects) -> bool:
    """Both TCPs at least 3 cm from every node of both tissues and from the box."""
    tc = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    return (sk.clear_of_tcps(objects["tissue_1"], CLEAR, tc) and sk.clear_of_tcps(objects["tissue_2"], CLEAR, tc)
            and _box_clear(objects, tc))


def _box_clear(objects, tc) -> bool:
    st = objects["box"]
    lo, hi = np.asarray(st.low) - CLEAR, np.asarray(st.high) + CLEAR
    return not any(np.all((np.asarray(t) >= lo) & (np.asarray(t) <= hi)) for t in tc)


def still(objects) -> bool:
    b = objects["box"]
    return (sk.motion(objects["tissue_1"], "tissue_1") < STILL and sk.motion(objects["tissue_2"], "tissue_2") < STILL
            and float(np.linalg.norm(b.lin_vel)) < STILL)


def score(objects) -> float:
    """0, latched, once the box has been lifted (its lowest corner > 5 mm off the table) or tilted > 5 deg, or the
    next tissue has left the box too. 0 at the end if the box stands more than 1.5 cm from its spot or tilted > 5
    deg, or the top tissue is not out. Otherwise 0.25 (top tissue out), + 0.25 if it lies on the tray, + 0.5 if the
    next tissue stands >= 10 mm above the lid with a quarter of it still under the lid. Halved unless both grippers
    are 3 cm clear of the tissues and the box and everything is still."""
    m = _mem()
    if box_lift(objects) > LIFT_TOL or box_tilt_deg(objects) > TILT_TOL:
        m["lifted"] = True
    if next_out(objects):
        m["two_out"] = True
    if DEBUG and mk._EP.get("env") is not None and mk._EP.get("step", 0) % 10 == 0:
        _debug(objects)
    if m["lifted"] or m["two_out"]:
        return 0.0
    pos, _ = _box_frame(objects)
    if math.hypot(pos[0] - XB, pos[1] - YB) > SPOT_TOL or not first_out(objects):
        return 0.0
    base = 0.25 + (0.25 if on_tray(objects, "tissue_1") else 0.0) + (0.5 if next_ready(objects) else 0.0)
    ok = released(objects) and still(objects)
    return round(base if ok else base / 2, 6)


def _debug(objects) -> None:
    import json
    import time

    pos, _ = _box_frame(objects)
    p1, p2 = local(objects, "tissue_1"), local(objects, "tissue_2")
    rec = {"k": mk._EP.get("step"), "t": round(time.time(), 3), "box": np.round(pos, 4).tolist(),
           "lift_mm": round(box_lift(objects) * 1000, 1), "tilt": round(box_tilt_deg(objects), 2),
           "film_mm": round(float(((objects["film"].pos - pos) @ _box_frame(objects)[1])[1] - 0.001) * 1000, 2),
           "t1": {"zmax": round(float(p1[:, 2].max() - ZT) * 1000, 1), "zmin": round(float(p1[:, 2].min() - ZT) * 1000,
                                                                                    1),
                  "in": round(float(in_box(p1).mean()), 3), "tray": on_tray(objects, "tissue_1"),
                  "xy": np.round(np.r_[sk.nodes(objects["tissue_1"])[:, :2].min(0),
                                       sk.nodes(objects["tissue_1"])[:, :2].max(0)], 3).tolist()},
           "t2": {"zmax": round(float(p2[:, 2].max() - ZT) * 1000, 1), "in": round(float(in_box(p2).mean()), 3),
                  "ylo": round(float(p2[:, 1].min()) * 1000, 1), "yhi": round(float(p2[:, 1].max()) * 1000, 1)},
           "m1": round(sk.motion(objects["tissue_1"], "tissue_1"), 4),
           "m2": round(sk.motion(objects["tissue_2"], "tissue_2"), 4),
           "rel": released(objects), "mem": dict(_mem()), "tcps": [np.round(t, 4).tolist() for t in mk.tcps()]}
    with open(str(Path.home() / "dbg" / "b18-tissue.jsonl"), "a") as fh:
        fh.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    """Top tissue on the tray, next one ready, box on its spot and never lifted, let go and still; held 1 s."""
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    """Top-tissue nodes still in the box, plus how far the next tissue is short of standing 10 mm up (m)."""
    return float(in_box(local(objects, "tissue_1")).sum() + max(0.0, READY_H - next_height(objects)))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the scene's geometry. The left hand comes down first, fingers closed, and presses the lid near
# its left end (200 mm left of the slit) so the box cannot ride up. The right hand comes over the standing tab with the
# fingers part open (30 mm), goes down until the pad tips are 6 mm above the lid (the pads then cover the tab from
# 6 to 22 mm up) and closes to a 2 mm jaw gap, a firm pinch on the thin tissue (closing fully on a single sheet
# squeezes it out of the jaws in this simulator). It pulls straight up 10.6 cm in 7 s, which draws the whole tissue
# out of the slit and, through the slit's squeeze, the next tissue's edge up after it. Then the left hand lets go of
# the box, the right carries the tissue to the tray, lowers it until it touches and opens; both arms park.

TIP = 0.013  # pad tips below the TCP
GRIP = -0.0010  # finger target: 2 mm jaw gap
APPROACH = -0.015  # 30 mm jaw gap coming down round the tab
PRESS = (XB, YB + 0.200)
DROP_Y = TRY - 0.025  # the sheet's creased lower end folds over toward +y as it lands
PULL = 0.106
L_PARK, R_PARK = (0.150, 0.250, 0.950), (0.200, -0.250, TOP + 0.160)
NAIVE = False  # True: the naive policy, the right hand pulls the same way and nobody holds the box (sim check)


def demo():
    """Hold the box down with the left hand; pull the top tissue straight up and out with the right; lay it on the
    tray."""
    c = FINGER_CLOSED
    g = (XB, YB - 0.001, TOP + ZT + 0.006 + TIP)
    R, L = [], []

    def w(arm, dur, pos, f, label, yaw=0.0):
        arm.append(Waypoint(dur, tuple(float(v) for v in pos), f, label, yaw))

    w(R, 3.0, (g[0], g[1], g[2] + 0.06), APPROACH, "over the tab, fingers part open")
    w(R, 1.5, (g[0], g[1], g[2] + 0.06), APPROACH, "wait for the left hand")
    w(R, 1.5, g, APPROACH, "down round the tab")
    w(R, 0.5, g, APPROACH, "settle")
    w(R, 1.0, g, GRIP, "pinch the tab")
    w(R, 0.5, g, GRIP, "settle")
    w(R, 7.0, (g[0], g[1], g[2] + PULL), GRIP, "pull straight up, out of the slit")
    w(R, 1.0, (g[0], g[1], g[2] + PULL), GRIP, "hold")
    w(R, 2.5, (TRX, DROP_Y, g[2] + PULL), GRIP, "carry to the tray")
    w(R, 1.5, (TRX, DROP_Y, TRAY_TOP + 0.078), GRIP, "lower until it touches")
    w(R, 1.0, (TRX, DROP_Y, TRAY_TOP + 0.078), FINGER_OPEN, "let go")
    w(R, 1.0, (TRX, DROP_Y, TRAY_TOP + 0.150), FINGER_OPEN, "up")
    w(R, 2.0, R_PARK, FINGER_OPEN, "park")
    total = sum(x.duration for x in R) + 3.0
    w(R, total - sum(x.duration for x in R), R_PARK, FINGER_OPEN, "wait")
    if NAIVE:
        w(L, 2.0, L_PARK, FINGER_OPEN, "park")
        w(L, total - 2.0, L_PARK, FINGER_OPEN, "park")
        return {"left": L, "right": R}
    lid = TOP + ZT + TIP
    w(L, 2.5, (*PRESS, lid + 0.05), c, "over the lid's left side, fingers closed")
    w(L, 1.5, (*PRESS, lid - 0.004), c, "press the box down")
    hold = 3.0 + 1.5 + 1.5 + 0.5 + 1.0 + 0.5 + 7.0 + 1.0 - 4.0
    w(L, hold, (*PRESS, lid - 0.004), c, "hold the box while the tissue is pulled")
    w(L, 1.0, (*PRESS, lid + 0.05), c, "lift off")
    w(L, 2.0, L_PARK, FINGER_OPEN, "park")
    w(L, total - sum(x.duration for x in L), L_PARK, FINGER_OPEN, "wait")
    return {"left": L, "right": R}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The scoring rules on hand-built states (one trial each, fed frame by frame)."""
    global _TEST_TCPS
    r1, r2 = sk.rest_nodes(TISSUE_1), sk.rest_nodes(TISSUE_2)
    o = np.array([XB, YB, TOP])

    def on_the_tray(p, dx=0.0):  # the tissue lying crumpled flat in the tray
        q = p - p.mean(0)
        q = q[:, [0, 1, 2]] * np.array([1.0, 0.0, 0.0]) + np.c_[np.zeros(len(q)), (q[:, 1] + q[:, 2]) * 0.8,
                                                             np.zeros(len(q))]
        return q + np.array([TRX + dx, TRY, TRAY_TOP + 0.004])

    def presented(p, up=0.016):  # the next tissue drawn up `up` along its path through the slit
        q = p - o
        lead = q[:, 1] < 0.0035
        q[lead, 2] += up
        near = (q[:, 1] >= 0.0035) & (q[:, 1] < 0.007 + up)  # what followed it now runs up the slit too
        q[near, 1] = 0.003
        q[near, 2] = ZT - 0.001 + up - (q[near, 1] - 0.003)
        q[:, 1] = np.where(q[:, 1] > 0.007 + up, q[:, 1] - up, q[:, 1])
        return q + o

    def state(p1, p2, box_dz=0.0, box_tilt=0.0, box_dxy=(0.0, 0.0), moving=False):
        h = math.radians(box_tilt) / 2
        qb = (math.sin(h), 0.0, 0.0, math.cos(h))
        st = kit.fake_states(moved={"box": ((XB + box_dxy[0], YB + box_dxy[1], TOP + box_dz), qb)})
        shift = np.array([box_dxy[0], box_dxy[1], box_dz])
        v = np.zeros_like(p1)
        if moving:
            v[:, 2] = -0.05
        st["tissue_1"], st["tissue_2"] = sk.fake(TISSUE_1, p1, v), sk.fake(TISSUE_2, p2 + shift)
        return st

    def lin(a, b, n=4):
        return [a + (b - a) * k / n for k in range(n + 1)]

    out_held = r1 + np.array([0.0, 0.0, 0.110])  # pulled straight up, still in the jaws
    tray = on_the_tray(r1)
    up2 = presented(r2)
    on_lid = r1 - o + np.array([0.0, -0.040, 0.0])
    on_lid[:, 2] = ZT + 0.003
    on_lid += o
    tcp_tab = [(XB, YB, TOP + ZT + 0.13)]
    cases = {  # name: (frames [(t1, t2, kwargs)], TCPs at the end, score)
        "start: top tissue standing in the slit": ([(r1, r2, {})], [], 0.0),
        "demo: out, on the tray, next one standing 16 mm, box held": (
            [(r1, r2, {}), (out_held, up2, {}), (tray, up2, {})], [], 1.0),
        "out and on the tray but the next fell back into the box": ([(r1, r2, {}), (tray, r2, {})], [], 0.5),
        "out, dropped on the lid, next one ready": ([(r1, r2, {}), (on_lid, up2, {})], [], 0.5 + 0.25),
        "out and still held in the jaws over the box": ([(r1, r2, {}), (out_held, up2, {})], tcp_tab, 0.375),
        "on the tray, still settling": ([(r1, r2, {}), (tray, up2, {"moving": True})], [], 0.5),
        "pulled half out, end still in the box": ([(r1, r2, {}), (r1 + np.array([0.0, -0.02, 0.005]), up2, {})],
                                                   [], 0.0),
        "nobody held the box: lifted 13 mm with the tissue, fell back": (
            [(r1, r2, {}), (out_held, up2, {"box_dz": 0.013, "box_tilt": 8.5}), (tray, up2, {})], [], 0.0),
        "box tipped 6 deg mid-pull (one side lifted)": (
            [(r1, r2, {}), (out_held, up2, {"box_tilt": 6.0}), (tray, up2, {})], [], 0.0),
        "box dragged 3 cm off its spot": ([(r1, r2, {}), (tray, up2, {"box_dxy": (0.0, -0.03)})], [], 0.0),
        "both tissues pulled out (pinched together), both on the tray": (
            [(r1, r2, {}), (tray, on_the_tray(r2, 0.01), {})], [], 0.0),
        "next one dragged out, then stuffed back in (latched)": (
            [(r1, r2, {}), (tray, on_the_tray(r2, 0.01), {}), (tray, up2, {})], [], 0.0),
    }
    try:
        for name, (frames, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = []
            for p1, p2, kw in frames[:-1]:
                score(state(p1, p2, **kw))
            _TEST_TCPS = tcps
            p1, p2, kw = frames[-1]
            got = score(state(p1, p2, **kw))
            assert abs(got - want) < 1e-9, (name, got, want, first_out(state(p1, p2, **kw)),
                                            next_height(state(p1, p2, **kw)))
            assert success(state(p1, p2, **kw)) == (want >= 1.0), name
        mk.new_episode()
        _TEST_TCPS = []
        score(state(out_held, up2, box_dz=0.02))
        mk.new_episode()
        assert score(state(tray, up2)) == 1.0, "a new trial forgets the last one's lift"
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases) + 1


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
