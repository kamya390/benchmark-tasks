"""Pure scoring policy and episode tracker for the turn-a-cake-out-of-its-tin task.

Nothing here imports YAM Arena, so the whole acceptance-critical state machine (plate on the tin, the clamped
inversion, the cake's release from the tin, the lift-off, the failure latches, release and stillness) is testable
offline by feeding synthetic :class:`Frame` poses.  Positions are metres in the station frame, z up.

Body frames
  tin    origin = centre of the floor's underside, +z toward the open rim.
  cake   origin = centre of its base (the parchment side, on the tin floor), +z toward its baked top.
  plate  origin = centre of its underside, +z = the eating face.

The first two milestones are episode history (latched); everything above them is read from the final state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# Geometry shared with the scene file.
TIN_H = 0.050            # outer height, floor underside to rim
TIN_FLOOR = 0.003        # floor thickness
TIN_R_FLOOR = 0.0975     # inner radius at the floor
TIN_R_RIM = 0.1000       # inner radius at the rim (1.5 mm of draft per side)
TIN_WALL = 0.002
TIN_R_OUT = 0.1035       # over the rolled rim
CAKE_H = 0.044
CAKE_R = 0.0976          # widest step (the baked top)
CAKE_MASS = 0.50
PLATE_R = 0.125
PLATE_T = 0.008
STAND_R = 0.055
STAND_H = 0.140

# The cake's light sticking to the tin: it lets go once the pull of its own weight along the tin axis, out of the
# opening, exceeds STICK_N (the tin tipped more than about 128 degrees from upright).
STICK_N = 3.0
G = 9.81

# Scoring thresholds.
PLATE_ON_TIN_RADIAL = 0.020
PLATE_ON_TIN_GAP = 0.005
UPRIGHT_DEG = 10.0
INVERTED_DEG = 15.0
CAKE_ON_PLATE_GAP = 0.005
CAKE_FLAT_DEG = 5.0
CAKE_CENTRE = 0.020
TIN_CLEAR = 0.020
TIN_ON_STAND = 0.030
TIN_ON_STAND_Z = STAND_H + 0.008
TABLE_TOUCH = 0.004
RELEASE_M = 0.030
PAIR_STEP_DEG = 15.0             # maximum observed axis change per 20 Hz frame
SUPPORT_GAP = 0.005
HOLD_STEPS = 20                   # 1.0 s at 20 Hz


@dataclass(frozen=True)
class Evidence:
    plate_on_tin: bool = False       # latched
    pair_inverted: bool = False      # latched
    tin_clear: bool = False
    cake_on_plate: bool = False
    cake_centred: bool = False
    cake_flat: bool = False
    plate_down: bool = False
    tin_on_stand: bool = False
    released: bool = False
    still_hold: bool = False
    cake_on_table: bool = False      # latched violation
    dropped: bool = False            # latched violation

    @property
    def violated(self) -> bool:
        return self.cake_on_table or self.dropped


def score_evidence(e: Evidence) -> float:
    """Highest completed milestone; a latched violation zeroes the episode."""
    if e.violated:
        return 0.0
    score = 0.0
    if e.plate_on_tin:
        score = 0.25
        if e.pair_inverted:
            score = 0.50
            if e.tin_clear and e.cake_on_plate and e.plate_down:
                score = 0.75
                if e.cake_centred and e.cake_flat and e.tin_on_stand and e.released and e.still_hold:
                    score = 1.0
    return score


@dataclass
class Frame:
    tin_pos: np.ndarray
    tin_rot: np.ndarray
    cake_pos: np.ndarray
    cake_rot: np.ndarray
    plate_pos: np.ndarray
    plate_rot: np.ndarray
    points: list = field(default_factory=list)   # gripper TCPs and finger-pad centres (world)
    still: bool = True                           # tin, cake and plate at rest


@dataclass
class Command:
    release_stick: bool = False


def _cos(deg: float) -> float:
    return math.cos(math.radians(deg))


def _z_range(centre, axis, radius: float, half: float) -> tuple[float, float]:
    """Lowest and highest z of a cylinder."""
    az = float(axis[2])
    reach = half * abs(az) + radius * math.sqrt(max(0.0, 1.0 - az * az))
    return float(centre[2]) - reach, float(centre[2]) + reach


def _dist_to_cylinder(p, centre, axis, radius: float, half: float) -> float:
    """Distance from a point to a solid cylinder (0 inside)."""
    d = np.asarray(p, float) - centre
    along = float(d @ axis)
    radial = float(np.linalg.norm(d - along * axis))
    da, dr = max(0.0, abs(along) - half), max(0.0, radial - radius)
    return math.hypot(da, dr)


class Tracker:
    """Accumulates one episode.  ``mat_xy`` is the centre of the stand the empty tin goes on, ``table_top`` the bench height."""

    def __init__(self, mat_xy, table_top: float, stand_xy=(0.450, 0.0)):
        self.mat = np.asarray(mat_xy, float)[:2]
        self.top = float(table_top)
        self.supports = (self.mat, np.asarray(stand_xy, float)[:2])
        self.reset()

    def reset(self, stuck: bool = True, plate_on_tin: bool = False, pair_inverted: bool = False):
        self.flags = dict(plate_on_tin=plate_on_tin, pair_inverted=pair_inverted, cake_on_table=False, dropped=False)
        self.inversion_active = False
        self.saw_sideways = False
        self.previous_axes = None
        self.stuck = stuck
        self.still_steps = 0
        self.evidence = Evidence(plate_on_tin=plate_on_tin, pair_inverted=pair_inverted)
        self.detail: dict = {}

    def update(self, f: Frame) -> Command:
        fl = self.flags
        tin, cake, plate = (np.asarray(v, float) for v in (f.tin_pos, f.cake_pos, f.plate_pos))
        ta, ca, pa = (np.asarray(r, float)[:, 2] for r in (f.tin_rot, f.cake_rot, f.plate_rot))
        tin_c = tin + ta * TIN_H / 2
        cake_c = cake + ca * CAKE_H / 2
        plate_c = plate + pa * PLATE_T / 2
        face = plate + pa * PLATE_T          # centre of the eating face
        rim = tin + ta * TIN_H               # centre of the rim plane

        # The cake's sticking gives way to its own weight once the tin is tipped far enough.
        command = Command()
        if self.stuck and CAKE_MASS * G * -float(ta[2]) >= STICK_N:
            self.stuck = False
            command.release_stick = True

        # Cake relative to the tin and to the plate.
        q = cake_c - tin_c
        in_tin_axial = float(q @ ta)
        in_tin_radial = float(np.linalg.norm(q - in_tin_axial * ta))
        cake_in_tin = in_tin_radial <= 0.006 and abs(in_tin_axial) <= 0.012
        cake_under_tin = in_tin_radial <= 0.006 and abs(in_tin_axial) <= TIN_H
        d = cake_c - face
        over_axial = float(d @ pa)
        over_radial = float(np.linalg.norm(d - over_axial * pa))
        tilt_on_plate = abs(float(ca @ pa))
        cake_gap = over_axial - (CAKE_H / 2 * tilt_on_plate + CAKE_R * math.sqrt(max(0.0, 1 - tilt_on_plate ** 2)))
        cake_on_plate = (float(pa[2]) >= _cos(UPRIGHT_DEG) and -0.002 <= cake_gap <= CAKE_ON_PLATE_GAP
                         and over_radial <= PLATE_R - CAKE_R and tilt_on_plate >= _cos(UPRIGHT_DEG))
        plate_low, _ = _z_range(plate_c, pa, PLATE_R, PLATE_T / 2)
        # A level plate must touch a real support plane with its footprint over
        # that support. A height ceiling alone also accepts a hovering plate.
        on_table = -0.002 <= plate_low - self.top <= SUPPORT_GAP
        on_stand = (-0.002 <= plate_low - self.top - STAND_H <= SUPPORT_GAP
                    and any(np.linalg.norm(plate[:2] - xy) <= PLATE_R - STAND_R
                            for xy in self.supports))
        plate_down = float(pa[2]) >= _cos(CAKE_FLAT_DEG) and (on_table or on_stand)

        # 0.25: the plate lies face-down on the upright tin's rim, centred, with the cake still in the tin.
        r = face - rim
        r_axial = float(r @ ta)
        r_radial = float(np.linalg.norm(r - r_axial * ta))
        pair_contact = (r_radial <= PLATE_ON_TIN_RADIAL and abs(r_axial) <= PLATE_ON_TIN_GAP
                        and float(ta @ pa) <= -_cos(UPRIGHT_DEG))
        covered = (float(ta[2]) >= _cos(UPRIGHT_DEG)
                   and float(pa[2]) <= -_cos(UPRIGHT_DEG) and cake_in_tin and pair_contact)
        if covered:
            fl["plate_on_tin"] = True

        # The 0.25 milestone survives a broken attempt. A new qualifying cover
        # starts another attempt. Higher credit needs an unbroken observed turn.
        if not fl["pair_inverted"]:
            continuous = pair_contact
            if self.previous_axes is not None:
                continuous = continuous and all(
                    float(old @ new) >= _cos(PAIR_STEP_DEG)
                    for old, new in zip(self.previous_axes, (ta, pa)))
            if not continuous:
                self.inversion_active = False
                self.saw_sideways = False
            if covered and not self.inversion_active:
                self.inversion_active = True
                self.saw_sideways = False
            if self.inversion_active:
                self.saw_sideways |= abs(float(ta[2])) <= 0.5
                if (self.saw_sideways and float(ta[2]) <= -_cos(INVERTED_DEG)
                        and plate_down and cake_under_tin and cake_on_plate and f.still):
                    fl["pair_inverted"] = True
        self.previous_axes = (ta.copy(), pa.copy())

        # 0.75: the tin is clear of the cake, which rests on the set-down plate.
        tin_low, _ = _z_range(tin_c, ta, TIN_R_OUT, TIN_H / 2)
        _, cake_high = _z_range(cake_c, ca, CAKE_R, CAKE_H / 2)
        apart = float(np.linalg.norm((tin_c - cake_c)[:2]))
        tin_clear = tin_low >= cake_high + TIN_CLEAR or apart >= TIN_R_OUT + CAKE_R + TIN_CLEAR

        # 1.0: cake centred and flat, turned out (its base uppermost), tin on the other stand, grippers clear, all still.
        cake_centred = over_radial <= CAKE_CENTRE
        cake_flat = float(ca[2]) <= -_cos(CAKE_FLAT_DEG)
        tin_on_stand = (float(np.linalg.norm(tin_c[:2] - self.mat)) <= TIN_ON_STAND
                      and abs(float(ta[2])) >= _cos(UPRIGHT_DEG) and tin_low <= self.top + TIN_ON_STAND_Z)
        solids = ((tin_c, ta, TIN_R_OUT, TIN_H / 2), (cake_c, ca, CAKE_R, CAKE_H / 2),
                  (plate_c, pa, PLATE_R, PLATE_T / 2))
        clearance = min((_dist_to_cylinder(p, *s) for p in f.points for s in solids), default=1.0)
        released = clearance >= RELEASE_M
        self.still_steps = self.still_steps + 1 if f.still else 0

        # Latched violations.
        cake_low, _ = _z_range(cake_c, ca, CAKE_R, CAKE_H / 2)
        if cake_low <= self.top + TABLE_TOUCH:
            fl["cake_on_table"] = True
        if min(tin[2], cake[2], plate[2]) < self.top - 0.020:
            fl["dropped"] = True

        self.evidence = Evidence(
            plate_on_tin=fl["plate_on_tin"], pair_inverted=fl["pair_inverted"], tin_clear=tin_clear,
            cake_on_plate=cake_on_plate, cake_centred=cake_centred, cake_flat=cake_flat, plate_down=plate_down,
            tin_on_stand=tin_on_stand, released=released, still_hold=self.still_steps >= HOLD_STEPS,
            cake_on_table=fl["cake_on_table"], dropped=fl["dropped"])
        self.detail = dict(plate_rim_mm=(round(r_radial * 1000, 1), round(r_axial * 1000, 1)),
                           cake_in_tin_mm=(round(in_tin_radial * 1000, 1), round(in_tin_axial * 1000, 1)),
                           cake_plate_mm=(round(over_radial * 1000, 1), round(cake_gap * 1000, 1)),
                           clearance_mm=round(clearance * 1000, 1), stuck=self.stuck, still=self.still_steps,
                           inversion_active=self.inversion_active, saw_sideways=self.saw_sideways,
                           pair_contact=pair_contact, plate_supported=plate_down)
        return command

    def score(self) -> float:
        return score_evidence(self.evidence)
