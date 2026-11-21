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


def inversion():
    """One observed, clamped turn, finishing inverted above the support."""
    frames = [covered()]
    centre = at(MAIN, 0.320)
    for angle in range(0, 181, 10):
        a = np.radians(angle)
        r = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
        axis = r[:, 2]
        tin = centre - axis * (s.TIN_H + s.PLATE_T) / 2
        frames.append(start(tin_pos=tin, tin_rot=r, cake_pos=tin + axis * 0.006,
                            cake_rot=r, plate_pos=tin + axis * (s.TIN_H + s.PLATE_T),
                            plate_rot=r @ FLIP, still=False))
    return frames


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
        self.assertEqual(self.run_frames([*inversion(), turned()]).score(), 0.5)

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
        self.assertEqual(self.run_frames([*inversion(), turned(), f]).score(), 0.75)

    def test_tin_raised_only_1cm_is_not_clear(self):
        f = turned(tin_pos=at(MAIN, s.STAND_H + s.PLATE_T + s.TIN_H + 0.01))
        self.assertEqual(self.run_frames([*inversion(), turned(), f]).score(), 0.5)

    def test_full_score(self):
        self.assertEqual(self.run_frames([*inversion(), turned(), done()]).score(), 1.0)

    def test_tin_may_rest_either_way_up_on_its_stand(self):
        f = done(tin_pos=at(MAT, s.STAND_H), tin_rot=UP)
        self.assertEqual(self.run_frames([*inversion(), turned(), f]).score(), 1.0)

    def test_cake_off_centre(self):
        f = done()
        f.cake_pos = f.cake_pos + [0.025, 0, 0]
        self.assertEqual(self.run_frames([*inversion(), turned(), f]).score(), 0.75)

    def test_tin_off_its_stand(self):
        f = done(tin_pos=at(MAT + [-0.02, 0.05], s.STAND_H + s.TIN_H))
        self.assertEqual(self.run_frames([*inversion(), turned(), f]).score(), 0.75)

    def test_gripper_still_at_the_tin(self):
        f = done(points=[at(MAT + [0.0, 0.12], s.STAND_H + 0.03)])
        self.assertEqual(self.run_frames([*inversion(), turned(), f]).score(), 0.75)

    def test_needs_one_second_of_stillness(self):
        t = self.run_frames([*inversion(), turned(), done(still=False)], repeat_last=3)
        self.assertEqual(t.score(), 0.75)
        for _ in range(s.HOLD_STEPS - 1):
            t.update(done())
        self.assertEqual(t.score(), 0.75)
        t.update(done())
        self.assertEqual(t.score(), 1.0)

    def test_cake_slid_off_plate_onto_table_zeroes(self):
        f = done(cake_pos=at((0.45, 0.3), s.CAKE_H), cake_rot=FLIP)
        self.assertEqual(self.run_frames([*inversion(), turned(), f]).score(), 0.0)

    def test_table_touch_is_latched(self):
        f = done(cake_pos=at((0.45, 0.3), s.CAKE_H), cake_rot=FLIP)
        self.assertEqual(self.run_frames([*inversion(), turned(), f, done()]).score(), 0.0)

    def test_cake_dumped_without_plate_zeroes(self):
        f = start(tin_rot=FLIP, tin_pos=at(MAIN, 0.3), cake_pos=at(MAIN, 0.0), cake_rot=UP)
        self.assertEqual(self.run_frames([covered(), f]).score(), 0.0)

    def test_dropped_off_the_bench_zeroes(self):
        f = done(tin_pos=at((0.9, 0.9), -0.5))
        self.assertEqual(self.run_frames([*inversion(), turned(), f]).score(), 0.0)

    def test_cake_tilted_on_plate_is_not_out(self):
        f = done(cake_rot=SIDE, cake_pos=at(MAIN, s.STAND_H + s.PLATE_T + s.CAKE_R))
        self.assertEqual(self.run_frames([*inversion(), turned(), f]).score(), 0.5)


class RevisionRegressions(unittest.TestCase):
    run_frames = Ladder.run_frames

    def test_endpoint_jump_does_not_prove_inversion(self):
        self.assertEqual(self.run_frames([covered(), turned()]).score(), 0.25)

    def test_stationary_plate_with_tin_inverted_onto_it(self):
        uncovered = start(plate_pos=at(MAIN, s.STAND_H), plate_rot=UP,
                          tin_pos=at(MAT, s.STAND_H), cake_pos=at(MAT, s.STAND_H + 0.0035))
        self.assertEqual(self.run_frames([covered(), uncovered, turned(), done()]).score(), 0.25)

    def test_continuous_tin_only_turn_onto_stationary_plate(self):
        frames = [covered()]
        plate_start = frames[0].plate_pos.copy()
        plate_high = at(MAT, s.STAND_H + 0.250)
        plate_final = at(MAT, s.STAND_H)

        # Lift the plate clear before changing its orientation or moving it aside.
        for fraction in np.linspace(0, 1, 21)[1:]:
            frames.append(start(plate_pos=plate_start + [0, 0, 0.250 * fraction], plate_rot=FLIP))
        raised_plate = frames[-1].plate_pos.copy()
        for fraction in np.linspace(0, 1, 19)[1:]:
            angle = np.pi * (1 - fraction)
            rot = np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)],
                            [0, np.sin(angle), np.cos(angle)]])
            frames.append(start(plate_pos=raised_plate + fraction * (plate_high - raised_plate),
                                plate_rot=rot, still=False))
        for fraction in np.linspace(0, 1, 21)[1:]:
            frames.append(start(plate_pos=plate_high + fraction * (plate_final - plate_high),
                                plate_rot=UP, still=False))

        # The plate stays on its own stand throughout the tin-only lift and turn.
        tin_start = at(MAIN, s.STAND_H)
        tin_high = tin_start + [0, 0, 0.250]
        for fraction in np.linspace(0, 1, 21)[1:]:
            tin = tin_start + fraction * (tin_high - tin_start)
            frames.append(start(tin_pos=tin, cake_pos=tin + [0, 0, 0.0035],
                                plate_pos=plate_final, plate_rot=UP, still=False))
        for fraction in np.linspace(0, 1, 19)[1:]:
            angle = np.pi * fraction
            rot = np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)],
                            [0, np.sin(angle), np.cos(angle)]])
            tin = tin_high + np.array([MAT[0] - MAIN[0], 0, 0]) * fraction
            cake = tin + rot[:, 2] * (0.0035 + 0.0025 * fraction)
            frames.append(start(tin_pos=tin, tin_rot=rot, cake_pos=cake, cake_rot=rot,
                                plate_pos=plate_final, plate_rot=UP, still=False))
        tin_high_inverted = frames[-1].tin_pos.copy()
        tin_final = at(MAT, s.STAND_H + s.PLATE_T + s.TIN_H)
        for fraction in np.linspace(0, 1, 21)[1:]:
            tin = tin_high_inverted + fraction * (tin_final - tin_high_inverted)
            frames.append(start(tin_pos=tin, tin_rot=FLIP, cake_pos=tin - [0, 0, 0.006],
                                cake_rot=FLIP, plate_pos=plate_final, plate_rot=UP,
                                still=fraction == 1))

        # The shortcut must fail even though every orientation change is small.
        for previous, current in zip(frames, frames[1:]):
            for name in ("tin_rot", "plate_rot"):
                dot = float(getattr(previous, name)[:, 2] @ getattr(current, name)[:, 2])
                self.assertGreaterEqual(dot, np.cos(np.radians(s.PAIR_STEP_DEG)))
        tracker = self.run_frames(frames)
        self.assertFalse(tracker.evidence.violated)
        self.assertTrue(tracker.evidence.plate_on_tin)
        self.assertFalse(tracker.evidence.pair_inverted)
        self.assertEqual(tracker.score(), 0.25)

    def test_inversion_with_plate_detached_midway(self):
        frames = inversion()
        frames[10].plate_pos = frames[10].plate_pos + [0.08, 0, 0]
        self.assertEqual(self.run_frames([*frames, turned(), done()]).score(), 0.25)

    def test_retry_after_broken_attempt(self):
        broken = start(plate_pos=at(MAT, s.STAND_H), plate_rot=UP)
        self.assertEqual(self.run_frames([*inversion()[:10], broken,
                                          *inversion(), turned(), done()]).score(), 1.0)

    def test_pair_hovering_below_support_height(self):
        f = turned()
        for name in ("tin_pos", "cake_pos", "plate_pos"):
            setattr(f, name, getattr(f, name) - [0, 0, 0.020])
        self.assertEqual(self.run_frames([*inversion(), f]).score(), 0.25)

    def test_pair_at_stand_height_without_stand_underneath(self):
        f = turned()
        for name in ("tin_pos", "cake_pos", "plate_pos"):
            setattr(f, name, getattr(f, name) + [0, 0.20, 0])
        self.assertEqual(self.run_frames([*inversion(), f]).score(), 0.25)

    def test_cake_not_resting_on_plate_at_inversion_milestone(self):
        f = turned(cake_pos=at(MAIN, s.STAND_H + s.PLATE_T + s.CAKE_H + 0.010))
        self.assertEqual(self.run_frames([*inversion(), f]).score(), 0.25)

    def test_reset_clears_turn_history(self):
        t = self.run_frames([*inversion(), turned()])
        self.assertEqual(t.score(), 0.5)
        t.reset()
        t.update(turned())
        self.assertEqual(t.score(), 0.0)


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
