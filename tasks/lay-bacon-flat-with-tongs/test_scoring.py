import unittest

import numpy as np

import scoring
from scoring import Obs, Tracker

EYE = np.eye(3)
HALF = np.array([0.0175, 0.125 / 6, 0.0015])   # one of six links of a 250 x 35 x 3 mm strip


def strip(x, y0=-0.125, z=0.0015, n=6):
    """A flat rasher lying along y on the pan floor, its first end at y0."""
    return [(np.array([x, y0 + (2 * k + 1) * HALF[1], z]), EYE, HALF) for k in range(n)]


def folded(x):
    """The last two links doubled back on top of the strip."""
    s = strip(x)
    for k, under in ((4, 3), (5, 2)):
        c = s[under][0] + [0, 0, 0.003]
        s[k] = (c, EYE, HALF)
    return s


def in_tray():
    return strip(0.0, y0=-0.60, z=0.03)


FAR = np.array([[0.0, 0.5, 0.3]])
TONGS_AWAY = np.array([[0.3, 0.0, 0.05], [0.3, 0.1, 0.05]])


def obs(rashers, **kw):
    base = dict(rashers=rashers, gripper_points=np.array([[0.0, 0.5, 0.3], [0.1, 0.5, 0.3]]),
                jaws=np.array([[0.3, 0.2, 0.05], [0.3, 0.2, 0.06]]),
                tcps=np.array([[0.0, 0.5, 0.3], [0.1, 0.5, 0.3]]), tongs_points=TONGS_AWAY,
                tongs_rest_err=0.0, still=True, dt=0.05)
    base.update(kw)
    return Obs(**base)


def carry(tracker, rashers, i):
    """One step in which rasher i is held 10 cm up between the tong jaws."""
    lifted = [list(r) for r in rashers]
    c = np.array([0.0, -0.3, 0.10])
    lifted[i] = [(c + [0, (2 * k) * HALF[1], 0], EYE, HALF) for k in range(6)]
    jaws = np.array([c + [0, 0, 0.004], c - [0, 0, 0.004]])
    tracker.update(obs(lifted, jaws=jaws))


GOOD = [strip(-0.06), strip(0.0), strip(0.06)]


class ScoringTests(unittest.TestCase):
    def tracker_all_carried(self):
        t = Tracker()
        for i in range(3):
            carry(t, [in_tray(), in_tray(), in_tray()], i)
        return t

    def run_for(self, t, o, steps):
        s = 0.0
        for _ in range(steps):
            s = t.update(o)
        return s

    def test_untouched_scene_scores_zero(self):
        self.assertEqual(Tracker().update(obs([in_tray(), in_tray(), in_tray()])), 0.0)

    def test_ladder(self):
        t = self.tracker_all_carried()
        self.assertEqual(t.update(obs([strip(-0.06), in_tray(), in_tray()])), 0.25)
        self.assertEqual(t.update(obs([strip(-0.06), strip(0.0), in_tray()])), 0.5)
        self.assertEqual(t.update(obs(GOOD, tongs_rest_err=0.2)), 0.75)

    def test_full_score_needs_one_second(self):
        t = self.tracker_all_carried()
        self.assertEqual(self.run_for(t, obs(GOOD), 19), 0.75)
        self.assertEqual(t.update(obs(GOOD)), 1.0)

    def test_full_score_conditions(self):
        for kw in (dict(tongs_rest_err=0.05), dict(still=False),
                   dict(tcps=np.array([[0.3, 0.0, 0.07], [0.1, 0.5, 0.3]])),       # still holding the tongs
                   dict(tcps=np.array([[-0.06, 0.0, 0.03], [0.1, 0.5, 0.3]]))):    # hovering on a rasher
            t = self.tracker_all_carried()
            self.assertEqual(self.run_for(t, obs(GOOD, **kw), 40), 0.75, kw)

    def test_stillness_restarts(self):
        t = self.tracker_all_carried()
        self.run_for(t, obs(GOOD), 15)
        t.update(obs(GOOD, still=False))
        self.assertEqual(self.run_for(t, obs(GOOD), 19), 0.75)
        self.assertEqual(t.update(obs(GOOD)), 1.0)

    def test_folded_rasher_does_not_count(self):
        t = self.tracker_all_carried()
        self.assertEqual(t.update(obs([folded(-0.06), strip(0.0), strip(0.06)])), 0.5)

    def test_raised_end_does_not_count(self):
        t = self.tracker_all_carried()
        r = strip(-0.06)
        r[5] = (r[5][0] + [0, 0, 0.004], EYE, HALF)   # top face 7 mm up
        self.assertEqual(t.update(obs([r, strip(0.0), strip(0.06)])), 0.5)

    def test_overlap_costs_both(self):
        t = self.tracker_all_carried()
        over = [(c + [0, 0, 0.0], r, h) for c, r, h in strip(-0.03)]
        self.assertEqual(t.update(obs([over, strip(0.0), strip(0.06)])), 0.25)

    def test_touching_edges_is_not_overlap(self):
        t = self.tracker_all_carried()
        self.assertEqual(t.update(obs([strip(-0.035), strip(0.0), strip(0.035)])), 0.75)

    def test_over_the_rim_does_not_count(self):
        t = self.tracker_all_carried()
        self.assertEqual(t.update(obs([strip(-0.09), strip(0.0), strip(0.06)])), 0.5)
        self.assertEqual(t.update(obs([strip(-0.06, y0=-0.09), strip(0.0), strip(0.06)])), 0.5)

    def test_gripper_touch_is_latched_per_rasher(self):
        t = self.tracker_all_carried()
        pts = np.array([[-0.06, 0.0, 0.008]])
        t.update(obs(GOOD, gripper_points=pts))
        self.assertEqual(t.touched, [True, False, False])
        self.assertEqual(self.run_for(t, obs(GOOD), 40), 0.5)

    def test_not_carried_in_tongs_does_not_count(self):
        t = Tracker()
        carry(t, [in_tray(), in_tray(), in_tray()], 0)
        self.assertEqual(self.run_for(t, obs(GOOD), 40), 0.25)   # e.g. the tray tipped out over the pan

    def test_jaws_on_a_low_rasher_is_not_carrying(self):
        t = Tracker()
        c = GOOD[0][2][0]
        t.update(obs(GOOD, jaws=np.array([c + [0, 0, 0.004], c - [0, 0, 0.004]])))
        self.assertEqual(t.carried, [False, False, False])

    def test_diagonal_rasher_overlap_uses_true_footprint(self):
        t = self.tracker_all_carried()
        a = np.radians(20)
        rot = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        diag = [(rot @ c, rot, h) for c, r, h in strip(0.0)]
        self.assertTrue(scoring.overlaps(diag, strip(0.035)))
        self.assertFalse(scoring.overlaps(strip(-0.06), strip(0.06)))


if __name__ == "__main__":
    unittest.main()
