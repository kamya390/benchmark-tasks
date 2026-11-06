"""On the station table, facing the arms, stands the front of a withdrawable air circuit breaker (a scaled-down
stored-energy power breaker of the kind that feeds a data-centre switchboard): a grey steel facia 26 cm wide and
15.5 cm tall with the breaker body behind it. On the facia: a green "I" (close) push button and a red "O" (open) push
button near the top on the left, two indicator windows in the middle (left: the main contacts, showing green for
open; right: the closing spring, showing white for discharged), and, on the right, the black manual charging handle,
sticking out level toward the arms on a sprung pivot. The breaker is open and its closing spring is discharged.

How the mechanism behaves (mechkit.py, in this folder, adds the joints to the stock runtime in memory):
- each full stroke of the charging handle (pressed down past 50 of its 65 degrees, then let back up within 10 degrees of level) winds the
  closing spring a step; after five full strokes the spring is charged and the charge flag drops from white to yellow;
- pressing the "I" button 5 mm or more closes the breaker only if the spring is charged: the contact flag drops from
  green to red and the charge flag drops on to white again (the spring has discharged into the contacts); pressed
  while the spring is discharged, the button does nothing;
- the "O" button is not modelled (it is part of the fixed facia).
Scored from the indicator flags' poses (what a photo of the facia shows) and, for partial credit, the counted strokes.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/acb-charge-before-close/acb_charge_before_close.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (installs the joint hooks; mk.MEMORY is cleared at every episode reset)

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
FACE_X = 0.380  # the facia's front face
FACIA_T = 0.006
CAV_X = FACE_X + FACIA_T  # 2 cm cavity behind the facia, where the indicator flags run
BODY_X = CAV_X + 0.020  # front of the breaker body
BODY_D = 0.130
HALF_W = 0.130  # facia and body 26 cm wide
TOP_Z = TOP + 0.155  # facia and body top

# push buttons (upper left)
BTN_Y, BTN_O_Y, BTN_Z = 0.070, 0.102, TOP + 0.125
BTN = 0.022  # square face
BTN_PROUD = 0.020  # the buttons stand 20 mm proud of the facia
BTN_LEN = 0.026
BTN_TRAVEL = 0.010
CLOSE_PRESS = 0.005  # the "I" button acts once pressed 5 mm in
HOLE_CLEAR = 0.001

# indicator windows (middle) and their flags in the cavity
WIN_W, WIN_H, WIN_Z = 0.030, 0.020, TOP + 0.070
CONTACT_Y, CHARGE_Y = 0.055, -0.005
BAND = 0.025  # each colour band on a flag; a flag drops one band per step
FLAG_W, FLAG_T = 0.044, 0.004
FLAG_X = CAV_X + 0.006  # flag plate centre: the plate runs 4 to 8 mm behind the facia's back face
PAWL = 0.008
PAWL_Z_CHARGE = TOP + 0.125  # the charge flag's two pawls
PAWL_Z_CONTACT = TOP + 0.100

# charging handle (right)
HANDLE_Y = -0.075
PIVOT = (FACE_X - 0.012, HANDLE_Y, TOP + 0.105)
BAR_L, BAR_W, BAR_T = 0.100, 0.024, 0.012  # from 6 mm behind the pivot to 9.4 cm in front of it
LIP = (0.012, 0.030, 0.020)  # the grip's raised end
HANDLE_KG = 0.050
HANDLE_K = 0.25  # N*m/rad return spring
HANDLE_PRELOAD_DEG = 10.0  # the spring holds the handle up against its level stop
HANDLE_MAX_DEG = 65.0
STROKE_DOWN_DEG, STROKE_UP_DEG = 50.0, 10.0  # a full stroke: past 50 degrees down, then back above 10
STROKES_TO_CHARGE = 5

GREY = (0.55, 0.57, 0.60)
DARK = (0.30, 0.31, 0.33)
BLACK = (0.08, 0.08, 0.09)
GREEN = (0.10, 0.62, 0.22)
RED = (0.82, 0.10, 0.08)
WHITE = (0.93, 0.93, 0.91)
YELLOW = (0.98, 0.82, 0.08)


def _plate(prefix, x0, x1, y0, y1, z0, z1, holes, color):
    """A plate (x0..x1 thick) over y0..y1, z0..z1 with rectangular holes [(yc, zc, w, h)], as plain boxes."""
    ys = sorted({y0, y1, *(h[0] - h[2] / 2 for h in holes), *(h[0] + h[2] / 2 for h in holes)})
    out = []
    for a, b in zip(ys, ys[1:]):
        cuts = sorted((h[1] - h[3] / 2, h[1] + h[3] / 2) for h in holes
                      if h[0] - h[2] / 2 <= a + 1e-9 and h[0] + h[2] / 2 >= b - 1e-9)
        lo = z0
        for c0, c1 in [*cuts, (z1, z1)]:
            if c0 - lo > 1e-6:
                out.append(Box(f"{prefix}{len(out)}", pos=((x0 + x1) / 2, (a + b) / 2, (lo + c0) / 2),
                               size=(x1 - x0, b - a, c0 - lo), color=color))
            lo = max(lo, c1)
    return out


_HOLES = [(BTN_Y, BTN_Z, BTN + 2 * HOLE_CLEAR, BTN + 2 * HOLE_CLEAR),
          (CONTACT_Y, WIN_Z, WIN_W, WIN_H), (CHARGE_Y, WIN_Z, WIN_W, WIN_H)]
FACIA = _plate("facia_", FACE_X, CAV_X, -HALF_W, HALF_W, TOP, TOP_Z, _HOLES, GREY)
BODY = Box("breaker_body", pos=(BODY_X + BODY_D / 2, 0.0, (TOP + TOP_Z) / 2), size=(BODY_D, 2 * HALF_W, TOP_Z - TOP),
           color=DARK)
CAVITY_LID = Box("cavity_lid", pos=((CAV_X + BODY_X) / 2, 0.0, TOP_Z - 0.003), size=(BODY_X - CAV_X, 2 * HALF_W, 0.006),
                 color=DARK)
O_BUTTON = Box("open_button", pos=(FACE_X - BTN_PROUD / 2, BTN_O_Y, BTN_Z), size=(BTN_PROUD, BTN, BTN), color=RED)
# the handle's pivot bracket: two cheeks either side of the bar's root
CHEEKS = [Box(f"handle_cheek_{n}", pos=((PIVOT[0] - 0.008 + FACE_X) / 2, HANDLE_Y + s * 0.018, PIVOT[2]),
              size=(FACE_X - PIVOT[0] + 0.008, 0.008, 0.026), color=DARK) for n, s in (("l", 1), ("r", -1))]
# ledges in the cavity that the flags' pawls rest on (fixed to the breaker body)
_LEDGE_X = (CAV_X + 0.0015, BODY_X)

kit = mk.Kit()

# -- the "I" (close) button: a sprung plunger through the facia --------------------------------------------------
kit.part("close_button", (FACE_X - BTN_PROUD + BTN_LEN / 2, BTN_Y, BTN_Z), [mk.box((BTN_LEN, BTN, BTN))],
         mass=0.015, color=GREEN)
kit.slider("close_button_travel", None, "close_button", axis=(1, 0, 0), lower=0.0, upper=BTN_TRAVEL,
           stiffness=40.0, rest=-0.004, damping=0.5)

# -- the indicator flags: plates on vertical slides, each held up by pawls glued to it that rest on fixed ledges ---
# q = how far a flag has dropped (m). Charge flag bands, bottom to top: white (discharged) / yellow (charged) / white.
kit.part("charge_flag", (FLAG_X, CHARGE_Y, WIN_Z), [
    mk.box((FLAG_T, FLAG_W, BAND), at=(0, 0, 0), color=WHITE),
    mk.box((FLAG_T, FLAG_W, BAND), at=(0, 0, BAND), color=YELLOW),
    mk.box((FLAG_T, FLAG_W, BAND), at=(0, 0, 2 * BAND), color=WHITE),
], mass=0.010, color=WHITE)
kit.slider("charge_flag_drop", None, "charge_flag", axis=(0, 0, -1), lower=0.0, upper=2 * BAND)
# contact flag bands, bottom to top: green (O, open) / red (I, closed)
kit.part("contact_flag", (FLAG_X, CONTACT_Y, WIN_Z), [
    mk.box((FLAG_T, FLAG_W, BAND), at=(0, 0, 0), color=GREEN),
    mk.box((FLAG_T, FLAG_W, BAND), at=(0, 0, BAND), color=RED),
], mass=0.010, color=GREEN)
kit.slider("contact_flag_drop", None, "contact_flag", axis=(0, 0, -1), lower=0.0, upper=BAND)

# pawls: (name, flag, y, z, drop before it lands on its ledge)
_PAWLS = {
    "charge_pawl": ("charge_flag", CHARGE_Y - FLAG_W / 2 - 0.003 - PAWL / 2, PAWL_Z_CHARGE, 0.0),
    "discharge_pawl": ("charge_flag", CHARGE_Y + FLAG_W / 2 + 0.003 + PAWL / 2, PAWL_Z_CHARGE, BAND),
    "contact_pawl": ("contact_flag", CONTACT_Y + FLAG_W / 2 + 0.003 + PAWL / 2, PAWL_Z_CONTACT, 0.0),
}
LEDGES = []
for _name, (_flag, _y, _z, _drop) in _PAWLS.items():
    kit.part(_name, (FLAG_X, _y, _z), [mk.box((PAWL, PAWL, PAWL))], mass=0.004, color=DARK)
    kit.bond(f"{_name}_glue", _flag, _name)
    _top = _z - PAWL / 2 - _drop
    LEDGES.append(Box(f"ledge_{_name}", pos=((_LEDGE_X[0] + _LEDGE_X[1]) / 2, _y, _top - 0.005),
                      size=(_LEDGE_X[1] - _LEDGE_X[0], PAWL, 0.010), color=DARK))

# -- the manual charging handle: a sprung lever, level at rest, pressed down through each stroke -------------------
kit.part("charge_handle", PIVOT, [
    mk.box((BAR_L, BAR_W, BAR_T), at=(-(BAR_L / 2 - 0.006), 0, 0), color=DARK),
    mk.box(LIP, at=(-(BAR_L - 0.006 - LIP[0] / 2), 0, BAR_T / 2 + LIP[2] / 2 - 0.006), color=BLACK),
], mass=HANDLE_KG, color=DARK)
# axis -y: a positive angle swings the handle's free end down
kit.hinge("charge_handle_pivot", None, "charge_handle", anchor=PIVOT, axis=(0, -1, 0), lower_deg=0.0,
          upper_deg=HANDLE_MAX_DEG, stiffness=HANDLE_K, rest_deg=-HANDLE_PRELOAD_DEG, damping=0.002)

OBJECTS = [*kit.objects(), *FACIA, BODY, CAVITY_LID, O_BUTTON, *CHEEKS, *LEDGES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- the breaker's logic (runs every control step in sim, and inside score() offline) ------------------------------
_TEST_TCPS = None  # self-test stand-in for the two grippers' TCPs
SHOWS = 0.018  # a flag counts as dropped one band when it is at least 18 of its 25 mm down


def _tcps():
    return mk.tcps() if _TEST_TCPS is None else [np.asarray(t, float) for t in _TEST_TCPS]


def _released(st, margin=0.025) -> bool:
    """Neither gripper's TCP within 2.5 cm of the part's bounding box."""
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps())


def _mechanism(objects) -> None:
    """Count full charging strokes; charge the spring after five; close the breaker when "I" is pressed while
    charged. State lives in mk.MEMORY (cleared at every episode reset); repeated calls on one state are harmless."""
    m = mk.MEMORY
    angle = kit.q_deg(objects, "charge_handle_pivot")
    if angle >= STROKE_DOWN_DEG:
        m["acb_handle_down"] = True
    elif angle <= STROKE_UP_DEG and m.get("acb_handle_down"):
        m["acb_handle_down"] = False
        m["acb_strokes"] = m.get("acb_strokes", 0) + 1
    if m.get("acb_strokes", 0) >= STROKES_TO_CHARGE and not m.get("acb_charged"):
        m["acb_charged"] = True
        kit.release("charge_pawl_glue")  # the charge flag drops to yellow
    if (m.get("acb_charged") and not m.get("acb_closed")
            and kit.q(objects, "close_button_travel") >= CLOSE_PRESS):
        m["acb_closed"] = True
        kit.release("contact_pawl_glue")  # contacts close: green -> red
        kit.release("discharge_pawl_glue")  # the spring has discharged: yellow -> white


kit.on_step(_mechanism)


def _contact_closed(objects) -> bool:
    """The contact flag shows red ("I"): dropped a band."""
    return kit.q(objects, "contact_flag_drop") >= SHOWS


def _spring_charged(objects) -> bool:
    """The charge flag shows yellow: dropped one band, not two."""
    return SHOWS <= kit.q(objects, "charge_flag_drop") <= BAND + (BAND - SHOWS)


def _hands_off(objects) -> bool:
    return all(_released(objects[p]) and objects[p].resting() for p in ("charge_handle", "close_button"))


def score(objects) -> float:
    """1.0: the contact flag shows red (breaker closed), both grippers clear of the handle and the buttons, the
    handle and the button still. 0.5: closed but a gripper still on the handle or button (or either still moving),
    or the closing spring charged (yellow flag) and the breaker not closed. 0.25: at least two full charging strokes
    counted. Otherwise 0. Pressing "I" with the spring discharged does nothing."""
    _mechanism(objects)
    if _contact_closed(objects):
        return 1.0 if _hands_off(objects) else 0.5
    if _spring_charged(objects):
        return 0.5
    return 0.25 if mk.MEMORY.get("acb_strokes", 0) >= 2 else 0.0


def success(objects) -> bool:
    """The breaker shows closed with the hands off it; holds 1 s."""
    return score(objects) >= 1.0


def distance(objects) -> float:
    """Charging strokes still to do (as 2 cm each) plus the contact flag's drop still to come."""
    left = max(0, STROKES_TO_CHARGE - mk.MEMORY.get("acb_strokes", 0))
    return float(0.02 * left + max(0.0, BAND - kit.q(objects, "contact_flag_drop")))


# -- the scripted demo ------------------------------------------------------------------------------------------
# Open loop from the facia's fixed geometry; it reads no joint or simulator state. The right arm pumps the charging
# handle six times, each to 62 degrees commanded (about 57 reached in sim) (the manual: pump until the indicator shows charged; five strokes charge this spring, the sixth
# is a spare that a charged mechanism ignores), then the left arm presses "I". Closed fingers throughout.

C = FINGER_CLOSED
TIP = 0.013  # pad tips below the TCP
PRESS_R = 0.066  # the pad presses the handle 6.6 cm out from the pivot, inboard of the grip's raised end
PRESS_IN = 0.003  # the pad tip runs 3 mm into the handle's top face line, so it keeps contact
STROKE_DEGS = (0, 12, 24, 36, 48, 56, 62)
N_STROKES = 6
PAD_FACE = 0.012  # the closed pads' face is ~12 mm from the TCP along x (measured in the kit smoke test)
BTN_PUSH = 0.008  # commanded press of the "I" button
PARK = {"left": (0.200, 0.200, TOP + 0.200), "right": (0.200, -0.200, TOP + 0.200)}


def _handle_tcp(deg, gap):
    """TCP that puts the pad tips ``gap`` above the handle's top face, PRESS_R out, with the handle at ``deg``."""
    a = math.radians(deg)
    along = np.array([-math.cos(a), 0.0, -math.sin(a)])
    up = np.array([-math.sin(a), 0.0, math.cos(a)])
    p = np.asarray(PIVOT, float) + PRESS_R * along + (BAR_T / 2 + gap) * up
    return (float(p[0]), float(p[1]), float(p[2] + TIP))


def _pump():
    hover = _handle_tcp(0, 0.030)
    wps = [Waypoint(2.0, hover, C, "over the charging handle"),
           Waypoint(1.0, _handle_tcp(0, 0.002), C, "onto the handle")]
    for k in range(N_STROKES):
        wps += [Waypoint(0.25, _handle_tcp(d, -PRESS_IN), C, f"stroke {k + 1}: down to {d} deg")
                for d in STROKE_DEGS[1:]]
        wps += [Waypoint(0.3, _handle_tcp(STROKE_DEGS[-1], -PRESS_IN), C, f"stroke {k + 1}: bottom"),
                Waypoint(0.8, _handle_tcp(20, 0.012), C, f"stroke {k + 1}: ease up"),
                Waypoint(0.5, _handle_tcp(0, 0.020), C, f"stroke {k + 1}: let it spring back"),
                Waypoint(0.3, _handle_tcp(0, 0.020), C, f"stroke {k + 1}: wait")]
        if k < N_STROKES - 1:
            wps.append(Waypoint(0.4, _handle_tcp(0, 0.002), C, f"stroke {k + 2}: onto the handle"))
    wps += [Waypoint(0.8, hover, C, "lift off"), Waypoint(1.5, PARK["right"], C, "park")]
    return wps


def _press_close(wait_s):
    front = FACE_X - BTN_PROUD  # the button's face at rest
    off = (front - PAD_FACE - 0.035, BTN_Y, BTN_Z)
    push = (front - PAD_FACE + BTN_PUSH, BTN_Y, BTN_Z)
    return [Waypoint(wait_s, PARK["left"], C, "wait for the spring to be charged"),
            Waypoint(1.5, (off[0], off[1], BTN_Z + 0.040), C, "in front of the I button, above"),
            Waypoint(1.0, off, C, "down level with it"),
            Waypoint(1.2, push, C, "press I"),
            Waypoint(0.6, push, C, "hold"),
            Waypoint(0.8, off, C, "back off"),
            Waypoint(1.0, (off[0], off[1], BTN_Z + 0.040), C, "up"),
            Waypoint(1.5, PARK["left"], C, "park")]


def _plan_s(wps) -> float:
    return sum(w.duration for w in wps)


def demo():
    """Right arm: six full strokes of the charging handle (closed fingers pressing its grip down along its arc to
    62 degrees, then easing up so the spring returns it). Left arm, once the right is done: press the green "I"
    button 8 mm, back off. Both park clear."""
    right = _pump()
    left = _press_close(_plan_s(right) - 1.5)
    right.append(Waypoint(max(0.5, _plan_s(left) - _plan_s(right)), PARK["right"], C, "wait"))
    return {"left": left, "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    """The rules on hand-built states: handle angles fed through one trial, flags posed by the kit."""
    global _TEST_TCPS
    import dataclasses

    def st(handle=0.0, button=0.0, charge=0.0, contact=0.0):
        return kit.fake_states(q={"charge_handle_pivot": math.radians(handle), "close_button_travel": button,
                                  "charge_flag_drop": charge, "contact_flag_drop": contact})

    def strokes(n, low=0.0, high=62.0, **kw):
        return [s for _ in range(n) for s in (st(high, **kw), st(low, **kw))]

    rest = st()
    charged = st(charge=BAND)
    closed = st(charge=2 * BAND, contact=BAND)
    on_button = kit.point(closed, "close_button", (-BTN_LEN / 2 - 0.012, 0, 0))
    moving = dict(closed)
    moving["close_button"] = dataclasses.replace(closed["close_button"], lin_vel=np.array([0.05, 0.0, 0.0]))

    cases = {  # name: (states fed in order in one trial, gripper TCPs at the end, final score)
        "start": ([rest], [], 0.0),
        "naive: press I with the spring discharged": ([rest, st(button=0.009), rest], [], 0.0),
        "one full stroke": ([rest, *strokes(1)], [], 0.0),
        "two full strokes": ([rest, *strokes(2)], [], 0.25),
        "ten half strokes (never past 50 degrees)": ([rest, *strokes(10, high=45.0)], [], 0.0),
        "ten strokes that never return (held between 30 and 62 degrees)": ([rest, *strokes(10, low=30.0)], [], 0.0),
        "spring charged (yellow flag), not closed": ([rest, *strokes(5), charged], [], 0.5),
        "charged, then I pressed: closed, hands off": ([rest, *strokes(5), charged, closed], [], 1.0),
        "closed, a gripper still on the I button": ([rest, *strokes(5), charged, closed], [on_button], 0.5),
        "closed, the button still springing back": ([rest, *strokes(5), charged, moving], [], 0.5),
        "end state closed without the history (flags alone)": ([closed], [], 1.0),
        "both flags at rest after eight strokes (no flag drop seen)": ([rest, *strokes(8)], [], 0.25),
    }
    try:
        for name, (seq, tcps, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            for s in seq[:-1]:
                score(s)
            got = score(seq[-1])
            assert abs(got - want) < 1e-9 and success(seq[-1]) == (want == 1.0), (name, got, want)
        # the mechanism: five strokes release the charge pawl, I before that does nothing, I after it closes
        mk.new_episode()
        _TEST_TCPS = []
        for s in [st(button=0.009), rest, *strokes(4)]:
            _mechanism(s)
        assert not kit._released and mk.MEMORY["acb_strokes"] == 4, (kit._released, mk.MEMORY)
        _mechanism(st(62.0))
        _mechanism(rest)
        assert kit._released == {"charge_pawl_glue"}, kit._released
        _mechanism(st(button=0.004, charge=BAND))
        assert not mk.MEMORY.get("acb_closed")
        _mechanism(st(button=0.006, charge=BAND))
        assert kit._released == {"charge_pawl_glue", "contact_pawl_glue", "discharge_pawl_glue"}, kit._released
        # a new trial forgets the strokes
        mk.new_episode()
        assert score(rest) == 0.0 and not kit._released and "acb_strokes" not in mk.MEMORY
        # geometry: a stroke press reaches 62 degrees with the pad on the handle, the fingers clear of the facia
        tcp = _handle_tcp(62, -PRESS_IN)
        assert tcp[0] + 0.034 < FACE_X and tcp[2] - TIP > TOP + 0.02, tcp
        assert FACE_X - BTN_PROUD - PAD_FACE + BTN_PUSH + 0.020 < FACE_X  # fingers clear of the facia at I
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

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
