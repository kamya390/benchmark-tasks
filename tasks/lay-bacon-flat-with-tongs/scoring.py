"""Pure scoring for the bacon lay-out task (NumPy only, no simulator).

Every length is in metres in the PAN FRAME: origin at the centre of the pan floor's top face, x and y along the
pan's short and long sides, z up.  A rasher is a list of links; a link is ``(centre, rotation 3x3, half sizes)``.
The task file feeds one :class:`Obs` per control step to a :class:`Tracker`.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# -- thresholds (the same numbers are quoted in task.json) --------------------------------------------------------
PAN_HALF = (0.100, 0.150)   # the pan's flat floor inside the rim: 200 x 300 mm
RASHER_LEN = 0.250          # stand-in strip, 250 x 35 x 3 mm
FLAT_MAX = 0.005            # no point of a laid rasher higher than 5 mm above the pan floor (a doubled part is >= 6)
STRAIGHT_MIN = 0.235        # end-to-end length of a laid rasher: at least 235 of its 250 mm
OVERLAP_SLACK = 0.001       # footprints closer than this (top view) do not count as overlapping
TOUCH = 0.010               # a gripper point within 10 mm of a rasher = the gripper touched it (latched)
PINCH = 0.008               # both tong jaws within 8 mm of one link ...
CARRY_Z = 0.060             # ... while that link is 60 mm above the pan floor = carried in the tongs (latched)
REST_ERR = 0.030            # tongs back on their rest: within 30 mm of their starting place
CLEAR = 0.040               # both gripper centre points at least 40 mm from the tongs and every rasher
HOLD_S = 1.0                # everything still for 1 s
N_RASHERS = 3


def corners(link) -> np.ndarray:
    c, r, h = (np.asarray(v, float) for v in link)
    signs = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)], float)
    return c + (signs * h) @ r.T


def point_box_distance(p, link) -> float:
    c, r, h = (np.asarray(v, float) for v in link)
    q = r.T @ (np.asarray(p, float) - c)
    return float(np.linalg.norm(np.maximum(np.abs(q) - h, 0.0)))


def _hull(pts: np.ndarray) -> np.ndarray:
    """Convex hull (counter-clockwise) of 2-D points: Andrew's monotone chain."""
    pts = sorted(map(tuple, np.round(pts, 9)))
    if len(pts) < 3:
        return np.array(pts)

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]

    return np.array(half(pts) + half(pts[::-1]))


def _convex_gap(a: np.ndarray, b: np.ndarray) -> float:
    """Separation of two convex polygons along their best separating axis (negative: they overlap by that much)."""
    best = -np.inf
    for poly in (a, b):
        for k in range(len(poly)):
            e = poly[(k + 1) % len(poly)] - poly[k]
            n = np.array([e[1], -e[0]])
            ln = np.linalg.norm(n)
            if ln < 1e-12:
                continue
            n /= ln
            pa, pb = a @ n, b @ n
            best = max(best, pb.min() - pa.max(), pa.min() - pb.max())
    return float(best)


def footprints(rasher) -> list[np.ndarray]:
    return [_hull(corners(link)[:, :2]) for link in rasher]


def overlaps(rasher_a, rasher_b) -> bool:
    """Seen from above, does any part of one rasher cover any part of the other?"""
    fb = footprints(rasher_b)
    return any(_convex_gap(pa, pb) < -OVERLAP_SLACK for pa in footprints(rasher_a) for pb in fb)


def end_to_end(rasher) -> float:
    """Distance between the mid-points of the rasher's two end edges (its length when straight)."""
    def end(link, sign):
        c, r, h = (np.asarray(v, float) for v in link)
        return c + r @ np.array([0.0, sign * h[1], 0.0])   # links run along their local y
    return float(np.linalg.norm(end(rasher[-1], 1.0) - end(rasher[0], -1.0)))


def inside_pan(rasher) -> bool:
    pts = np.vstack([corners(link) for link in rasher])
    return bool(np.all(np.abs(pts[:, 0]) <= PAN_HALF[0]) and np.all(np.abs(pts[:, 1]) <= PAN_HALF[1])
                and pts[:, 2].min() > -0.002)


def flat(rasher) -> bool:
    return bool(max(corners(link)[:, 2].max() for link in rasher) <= FLAT_MAX)


@dataclass
class Obs:
    rashers: list                      # N_RASHERS lists of links
    gripper_points: np.ndarray         # (k, 3): both gripper centre points and all four finger pads
    jaws: np.ndarray                   # (2, 3): the two tong jaw pads
    tcps: np.ndarray                   # (2, 3): the two gripper centre points
    tongs_points: np.ndarray           # (k, 3): points along the tongs (for the release clearance)
    tongs_rest_err: float              # how far the tongs are from where they started on the rest
    still: bool                        # every loose object at rest this step
    dt: float = 0.05


@dataclass
class Tracker:
    touched: list = field(default_factory=lambda: [False] * N_RASHERS)
    carried: list = field(default_factory=lambda: [False] * N_RASHERS)
    laid: list = field(default_factory=lambda: [False] * N_RASHERS)
    done_s: float = 0.0
    detail: dict = field(default_factory=dict)

    def update(self, o: Obs) -> float:
        n = len(o.rashers)
        for i, rasher in enumerate(o.rashers):
            if any(point_box_distance(p, link) < TOUCH for p in o.gripper_points for link in rasher):
                self.touched[i] = True
            for link in rasher:
                if (float(np.asarray(link[0])[2]) >= CARRY_Z
                        and all(point_box_distance(j, link) < PINCH for j in o.jaws)):
                    self.carried[i] = True
        ok_geom = []
        for i, rasher in enumerate(o.rashers):
            ok = inside_pan(rasher) and flat(rasher) and end_to_end(rasher) >= STRAIGHT_MIN
            ok = ok and not any(overlaps(rasher, o.rashers[j]) for j in range(n) if j != i)
            ok_geom.append(ok)
        self.laid = [ok_geom[i] and self.carried[i] and not self.touched[i] for i in range(n)]
        count = sum(self.laid)
        things = np.vstack([np.asarray(o.tongs_points, float)]
                           + [np.asarray(link[0], float)[None] for r in o.rashers for link in r])
        clear = min(float(np.linalg.norm(things - np.asarray(t, float), axis=1).min()) for t in o.tcps)
        finished = (count == n and o.tongs_rest_err <= REST_ERR and clear >= CLEAR and o.still)
        self.done_s = self.done_s + o.dt if finished else 0.0
        self.detail = {"geom": ok_geom, "carried": list(self.carried), "touched": list(self.touched),
                       "rest_err": round(float(o.tongs_rest_err), 4), "clear": round(clear, 4), "still": o.still,
                       "done_s": round(self.done_s, 2)}
        if finished and self.done_s >= HOLD_S - 1e-9:
            return 1.0
        return 0.25 * min(count, 3)

