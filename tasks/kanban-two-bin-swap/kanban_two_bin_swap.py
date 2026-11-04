"""On the station table, in front of the left arm, a two-bin kanban lane at a line-side rack. The grey tote in the pick
slot (nearest the arms, between two low guide rails) is empty, and its yellow kanban card lies on its lid. The full
blue tote waits in the reserve slot behind it, in the same lane. To the right of the lane stands the card post (a
small white open frame) and further left a black mat marks the return shelf for empty totes. The replenishment
loop: the card goes to the post, the empty tote to the return mat, and the full tote forward into the pick slot.
Scored from the poses at the end.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/kanban-two-bin-swap/kanban_two_bin_swap.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
TOTE = (0.075, 0.110, 0.060)  # closed parts tote: 7.5 cm deep (x), 11 cm wide (y), 6 cm tall
EMPTY_KG, FULL_KG = 0.120, 0.450
LANE_Y = 0.300
PICK_X = 0.190  # slot centres along the lane
RESERVE_X = 0.300
RAIL = (0.230, 0.006, 0.010)  # the two guide rails, along x, 3 mm outside the totes
RAIL_X = 0.245
RAIL_DY = TOTE[1] / 2 + 0.003 + RAIL[1] / 2

CARD = (0.050, 0.035, 0.006)  # kanban card holder, a stiff plastic sleeve
CARD_KG = 0.010
POST = (0.190, 0.150)  # the card post's pocket centre
POST_INNER = (0.070, 0.050)
POST_WALL = (0.006, 0.030)  # thickness, height
RETURN = (0.300, 0.440)  # the return mat's centre
RETURN_MAT = (0.120, 0.140, 0.002)

GREY = (0.60, 0.61, 0.63)
BLUE = (0.15, 0.32, 0.80)
YELLOW = (0.95, 0.80, 0.10)
WHITE = (0.92, 0.92, 0.92)
BLACK = (0.08, 0.08, 0.08)
STEEL = (0.75, 0.76, 0.78)


def _pocket(name, centre, inner, wall, color):
    cx, cy = centre
    ix, iy = inner
    t, h = wall
    z = TOP + h / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, h), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, h), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, h), color=color),
    ]


EMPTY = Box("empty_tote", pos=(PICK_X, LANE_Y, TOP + TOTE[2] / 2), size=TOTE, mass=EMPTY_KG, color=GREY)
FULL = Box("full_tote", pos=(RESERVE_X, LANE_Y, TOP + TOTE[2] / 2), size=TOTE, mass=FULL_KG, color=BLUE)
KANBAN = Box("kanban_card", pos=(PICK_X, LANE_Y, TOP + TOTE[2] + CARD[2] / 2), size=CARD, mass=CARD_KG, color=YELLOW)
FIXTURES = [
    Box("rail_left", pos=(RAIL_X, LANE_Y + RAIL_DY, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    Box("rail_right", pos=(RAIL_X, LANE_Y - RAIL_DY, TOP + RAIL[2] / 2), size=RAIL, color=STEEL),
    *_pocket("card_post", POST, POST_INNER, POST_WALL, WHITE),
    Box("return_mat", pos=(*RETURN, TOP + RETURN_MAT[2] / 2), size=RETURN_MAT, color=BLACK),
]
OBJECTS = [EMPTY, FULL, KANBAN, *FIXTURES]
MAT_TOP = TOP + RETURN_MAT[2]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SLOT_X = 0.010  # a tote is in the pick slot with its centre within 1 cm of the slot centre along the lane
SLOT_DEG = 8.0  # and square to the lane


def _yaw_tilt(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    up_z = 1.0 - 2.0 * (x * x + y * y)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _upright_on(state, z) -> bool:
    _, tilt = _yaw_tilt(state.quat_xyzw)
    return bool(abs(float(state.low[2]) - z) <= 0.003 and tilt <= 8.0 and state.resting())


def _in_pick_slot(state) -> bool:
    """Standing upright on the table between the guide rails, centre within 1 cm of the pick slot's centre."""
    yaw, _ = _yaw_tilt(state.quat_xyzw)
    return bool(
        _upright_on(state, TOP)
        and abs(float(state.pos[0]) - PICK_X) <= SLOT_X
        and abs(float(state.pos[1]) - LANE_Y) <= 0.006
        and _square(yaw) <= SLOT_DEG
    )


def _on_return_mat(state) -> bool:
    """Standing upright on the black mat with its whole footprint (centre at least half a tote inside its edges)."""
    yaw, _ = _yaw_tilt(state.quat_xyzw)
    ex = abs(math.cos(math.radians(yaw))) * TOTE[0] / 2 + abs(math.sin(math.radians(yaw))) * TOTE[1] / 2
    ey = abs(math.sin(math.radians(yaw))) * TOTE[0] / 2 + abs(math.cos(math.radians(yaw))) * TOTE[1] / 2
    return bool(
        _upright_on(state, MAT_TOP)
        and abs(float(state.pos[0]) - RETURN[0]) + ex <= RETURN_MAT[0] / 2
        and abs(float(state.pos[1]) - RETURN[1]) + ey <= RETURN_MAT[1] / 2
    )


def _in_post(state) -> bool:
    """At rest with its centre inside the post's inner walls, below the top of the walls."""
    return bool(
        abs(float(state.pos[0]) - POST[0]) < POST_INNER[0] / 2
        and abs(float(state.pos[1]) - POST[1]) < POST_INNER[1] / 2
        and float(state.low[2]) < TOP + POST_WALL[1]
        and state.resting()
    )


def _steps(objects):
    return (
        _in_post(objects["kanban_card"]),
        _on_return_mat(objects["empty_tote"]),
        _in_pick_slot(objects["full_tote"]),
    )


def success(objects) -> bool:
    """Card in the post, empty tote on the return mat, full tote in the pick slot; holds for 1 s."""
    return all(_steps(objects))


def score(objects) -> float:
    """1/3 each for the card in the post, the empty tote on the return mat and the full tote in the pick slot."""
    return sum(_steps(objects)) / 3.0


def distance(objects) -> float:
    c, e, f = objects["kanban_card"], objects["empty_tote"], objects["full_tote"]
    return float(
        math.hypot(float(c.pos[0]) - POST[0], float(c.pos[1]) - POST[1])
        + math.hypot(float(e.pos[0]) - RETURN[0], float(e.pos[1]) - RETURN[1])
        + math.hypot(float(f.pos[0]) - PICK_X, float(f.pos[1]) - LANE_Y)
    )


# -- the scripted demo ------------------------------------------------------------------------------------------

TIP = 0.013
TRAVEL = TOP + 0.150
SQUEEZE = 0.008


def _move(pick, place, *, width, height, depth, yaw, label, pick_bottom, place_bottom, open_=FINGER_OPEN):
    """Top-down pick and place; the pads close ``SQUEEZE`` narrower than ``width``, their tips ``depth`` under the
    part's top. 17.9 s."""
    (px, py), (qx, qy) = pick, place
    gz = pick_bottom + height - depth + TIP
    qz = place_bottom + height - depth + TIP + 0.002
    grip = -(width - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), open_, f"{label}: over", yaw),
        Waypoint(1.5, (px, py, gz + 0.020), open_, f"{label}: descend", yaw),
        Waypoint(1.0, (px, py, gz), open_, f"{label}: down slowly", yaw),
        Waypoint(0.5, (px, py, gz), open_, f"{label}: settle", yaw),
        Waypoint(1.5, (px, py, gz), grip, f"{label}: close", yaw),
        Waypoint(1.5, (px, py, TRAVEL), grip, f"{label}: lift", yaw),
        Waypoint(2.0, (qx, qy, TRAVEL), grip, f"{label}: carry", yaw),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", yaw),
        Waypoint(1.5, (qx, qy, qz + 0.020), grip, f"{label}: lower", yaw),
        Waypoint(1.5, (qx, qy, qz), grip, f"{label}: set down slowly", yaw),
        Waypoint(0.6, (qx, qy, qz), grip, f"{label}: settle", yaw),
        Waypoint(1.5, (qx, qy, qz), open_, f"{label}: release", yaw),
        Waypoint(1.0, (qx, qy, TRAVEL), open_, f"{label}: retreat", yaw),
    ]


def demo():
    """Left arm only: the card off the empty tote and dropped into the post from just above its walls (fingers across
    its 35 mm width), the empty tote to the
    return mat, then the full tote forward into the pick slot (fingers across the totes' 75 mm depth, from above the
    guide rails). 53.7 s."""
    card_open = -(CARD[1] + 0.024) / 2
    drop = TOP + POST_WALL[1] + 0.006  # the card is let go with the pads above the post's walls and drops in
    card = _move((PICK_X, LANE_Y), POST, width=CARD[1], height=CARD[2], depth=0.004, yaw=0.0, label="card",
                 pick_bottom=TOP + TOTE[2], place_bottom=drop, open_=card_open)
    empty = _move((PICK_X, LANE_Y), RETURN, width=TOTE[0], height=TOTE[2], depth=0.025, yaw=math.pi / 2,
                  label="empty tote", pick_bottom=TOP, place_bottom=MAT_TOP)
    full = _move((RESERVE_X, LANE_Y), (PICK_X, LANE_Y), width=TOTE[0], height=TOTE[2], depth=0.025, yaw=math.pi / 2,
                 label="full tote", pick_bottom=TOP, place_bottom=TOP)
    return {"left": card + empty + full}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        q = np.array([cy * st, sy * st, sy * ct, cy * ct])
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    start = {
        "empty_tote": box(TOTE, PICK_X, LANE_Y, TOP),
        "full_tote": box(TOTE, RESERVE_X, LANE_Y, TOP),
        "kanban_card": box(CARD, PICK_X, LANE_Y, TOP + TOTE[2]),
    }
    card_in = box(CARD, *POST, TOP)
    empty_back = box(TOTE, *RETURN, MAT_TOP)
    full_fwd = box(TOTE, PICK_X, LANE_Y, TOP)
    done = {"kanban_card": card_in, "empty_tote": empty_back, "full_tote": full_fwd}
    cases = {
        "start": (start, 0.0, False),
        "done": (done, 1.0, True),
        "done, empty tote turned 90 degrees on the mat": (done | {"empty_tote": box(TOTE, *RETURN, MAT_TOP, 90)}, 1.0, True),
        "card in the post only": (start | {"kanban_card": card_in}, 1 / 3, False),
        "card left on the empty tote, totes swapped": (
            {"kanban_card": box(CARD, *RETURN, MAT_TOP + TOTE[2]), "empty_tote": empty_back, "full_tote": full_fwd}, 2 / 3, False),
        "card on the table beside the post": (done | {"kanban_card": box(CARD, POST[0], POST[1] - 0.06, TOP)}, 2 / 3, False),
        "card lying on the post wall": (done | {"kanban_card": box(CARD, POST[0], POST[1] + 0.028, TOP + POST_WALL[1])}, 2 / 3, False),
        "empty tote half off the mat": (done | {"empty_tote": box(TOTE, RETURN[0] + 0.040, RETURN[1], MAT_TOP)}, 2 / 3, False),
        "empty tote on its side on the mat": (done | {"empty_tote": box(TOTE, *RETURN, MAT_TOP, 0, 90)}, 2 / 3, False),
        "full tote 2 cm short of the slot": (done | {"full_tote": box(TOTE, PICK_X + 0.020, LANE_Y, TOP)}, 2 / 3, False),
        "full tote turned 15 degrees in the slot": (done | {"full_tote": box(TOTE, PICK_X, LANE_Y, TOP, 15)}, 2 / 3, False),
        "full tote still sliding": (done | {"full_tote": box(TOTE, PICK_X, LANE_Y, TOP, vel=(-0.03, 0, 0))}, 2 / 3, False),
        "empty tote put back in the pick slot": (start | {"kanban_card": card_in}, 1 / 3, False),
        "full tote on the return mat, empty tote in the slot": (
            {"kanban_card": card_in, "empty_tote": full_fwd, "full_tote": box(TOTE, *RETURN, MAT_TOP)}, 1 / 3, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(done) < 1e-9
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
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
