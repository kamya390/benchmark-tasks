"""Offline tests of the scoring state machine: every tier, every full-score condition, every failure latch."""

import unittest

import numpy as np

import scoring as s

TOP = 0.7475
MAIN = np.array([0.45, 0.0])
MAT = np.array([0.20, 0.0])
UP = np.eye(3)
FLIP = np.diag([1.0, -1.0, -1.0])      # half a turn about x: +z points down
SIDE = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])   # +z along world x
FAR = [np.array([0.2, 0.4, TOP + 0.3])]


def at(xy, z):
    return np.array([xy[0], xy[1], TOP + z])


def start(**kw):
    """Tin with the cake on the stand, plate elsewhere."""
    f = dict(tin_pos=at(MAIN, s.STAND_H), tin_rot=UP, cake_pos=at(MAIN, s.STAND_H + 0.0035), cake_rot=UP,
             plate_pos=at(MAT, s.STAND_H), plate_rot=UP, points=FAR, still=True)
    f.update(kw)
    return s.Frame(**f)


def covered(**kw):
    """Plate face-down on the tin's rim."""
    return start(plate_pos=at(MAIN, s.STAND_H + s.TIN_H + s.PLATE_T), plate_rot=FLIP, **kw)


def turned(**kw):
    """Pair turned over and set down: plate on the stand, cake on it, tin over the cake."""
    f = dict(plate_pos=at(MAIN, s.STAND_H), plate_rot=UP,
             cake_pos=at(MAIN, s.STAND_H + s.PLATE_T + s.CAKE_H), cake_rot=FLIP,
             tin_pos=at(MAIN, s.STAND_H + s.PLATE_T + s.TIN_H), tin_rot=FLIP)
    f.update(kw)
    return start(**f)


def done(**kw):
    f = dict(tin_pos=at(MAT, s.STAND_H + s.TIN_H), tin_rot=FLIP)
    f.update(kw)
    return turned(**f)


class Ladder(unittest.TestCase):
    def run_frames(self, frames, repeat_last=25):
        t = s.Tracker(MAT, TOP)
        for f in frames:
            t.update(f)
        for _ in range(repeat_last):
            t.update(frames[-1])
        return t

    def test_untouched_scores_zero(self):
        self.assertEqual(self.run_frames([start()]).score(), 0.0)

    def test_plate_on_tin(self):
        self.assertEqual(self.run_frames([start(), covered()]).score(), 0.25)

    def test_plate_right_way_up_on_tin_earns_nothing(self):
        f = start(plate_pos=at(MAIN, s.STAND_H + s.TIN_H), plate_rot=UP)
        self.assertEqual(self.run_frames([f]).score(), 0.0)

    def test_plate_off_centre_earns_nothing(self):
        f = covered()
        f.plate_pos = f.plate_pos + [0.025, 0, 0]
        self.assertEqual(self.run_frames([f]).score(), 0.0)

    def test_pair_inverted(self):
        self.assertEqual(self.run_frames([covered(), turned()]).score(), 0.5)

    def test_inverted_while_still_held_high_is_not_set_down(self):
        lift = np.array([0, 0, 0.06])
        f = turned()
        for k in ("tin_pos", "cake_pos", "plate_pos"):
            setattr(f, k, getattr(f, k) + lift)
        self.assertEqual(self.run_frames([covered(), f]).score(), 0.25)

    def test_inversion_without_the_plate_step_earns_nothing(self):
        self.assertEqual(self.run_frames([turned()]).score(), 0.0)

    def test_tin_lifted_clear(self):
        f = turned(tin_pos=at(MAIN, s.STAND_H + s.PLATE_T + s.TIN_H + 0.08))
        self.assertEqual(self.run_frames([covered(), turned(), f]).score(), 0.75)

    def test_tin_raised_only_1cm_is_not_clear(self):
        f = turned(tin_pos=at(MAIN, s.STAND_H + s.PLATE_T + s.TIN_H + 0.01))
        self.assertEqual(self.run_frames([covered(), turned(), f]).score(), 0.5)

    def test_full_score(self):
        self.assertEqual(self.run_frames([covered(), turned(), done()]).score(), 1.0)

    def test_tin_may_rest_either_way_up_on_its_stand(self):
        f = done(tin_pos=at(MAT, s.STAND_H), tin_rot=UP)
        self.assertEqual(self.run_frames([covered(), turned(), f]).score(), 1.0)

    def test_cake_off_centre(self):
        f = done()
        f.cake_pos = f.cake_pos + [0.025, 0, 0]
        self.assertEqual(self.run_frames([covered(), turned(), f]).score(), 0.75)

    def test_tin_off_its_stand(self):
        f = done(tin_pos=at(MAT + [-0.02, 0.05], s.STAND_H + s.TIN_H))
        self.assertEqual(self.run_frames([covered(), turned(), f]).score(), 0.75)

    def test_gripper_still_at_the_tin(self):
        f = done(points=[at(MAT + [0.0, 0.12], s.STAND_H + 0.03)])
        self.assertEqual(self.run_frames([covered(), turned(), f]).score(), 0.75)

    def test_needs_one_second_of_stillness(self):
        t = self.run_frames([covered(), turned(), done(still=False)], repeat_last=3)
        self.assertEqual(t.score(), 0.75)
        for _ in range(s.HOLD_STEPS - 1):
            t.update(done())
        self.assertEqual(t.score(), 0.75)
        t.update(done())
        self.assertEqual(t.score(), 1.0)

    def test_cake_slid_off_plate_onto_table_zeroes(self):
        f = done(cake_pos=at((0.45, 0.3), s.CAKE_H), cake_rot=FLIP)
        self.assertEqual(self.run_frames([covered(), turned(), f]).score(), 0.0)

    def test_table_touch_is_latched(self):
        f = done(cake_pos=at((0.45, 0.3), s.CAKE_H), cake_rot=FLIP)
        self.assertEqual(self.run_frames([covered(), turned(), f, done()]).score(), 0.0)

    def test_cake_dumped_without_plate_zeroes(self):
        f = start(tin_rot=FLIP, tin_pos=at(MAIN, 0.3), cake_pos=at(MAIN, 0.0), cake_rot=UP)
        self.assertEqual(self.run_frames([covered(), f]).score(), 0.0)

    def test_dropped_off_the_bench_zeroes(self):
        f = done(tin_pos=at((0.9, 0.9), -0.5))
        self.assertEqual(self.run_frames([covered(), turned(), f]).score(), 0.0)

    def test_cake_tilted_on_plate_is_not_out(self):
        f = done(cake_rot=SIDE, cake_pos=at(MAIN, s.STAND_H + s.PLATE_T + s.CAKE_R))
        self.assertEqual(self.run_frames([covered(), turned(), f]).score(), 0.5)


class Stick(unittest.TestCase):
    def test_holds_upright_and_on_its_side(self):
        t = s.Tracker(MAT, TOP)
        self.assertFalse(t.update(start()).release_stick)
        self.assertFalse(t.update(start(tin_rot=SIDE)).release_stick)
        self.assertTrue(t.stuck)

    def test_lets_go_once_turned_over_and_only_once(self):
        t = s.Tracker(MAT, TOP)
        self.assertTrue(t.update(turned()).release_stick)
        self.assertFalse(t.stuck)
        self.assertFalse(t.update(turned()).release_stick)

    def test_release_angle(self):
        t = s.Tracker(MAT, TOP)
        for deg, want in ((120, False), (126, False), (130, True)):
            a = np.radians(deg)
            rot = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
            self.assertEqual(t.update(start(tin_rot=rot)).release_stick, want, deg)


if __name__ == "__main__":
    unittest.main()
