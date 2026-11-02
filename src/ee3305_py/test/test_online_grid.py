"""Run without ROS: python -m unittest discover -s src/ee3305_py/test -v."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ee3305_py.online_grid import Grid, inflate, reachable, select_target, trajectory_clear


class GridTests(unittest.TestCase):
    def test_unknown_and_uncertain_cells_remain_blocked(self):
        raw = Grid(21, 21, .1, 0., 0., [0] * 441)
        raw.data[raw.index(10, 10)] = -1
        raw.data[raw.index(15, 15)] = 50
        raw.data[raw.index(5, 5)] = 100
        cost = inflate(raw, .12, .4)
        self.assertEqual(cost.data[cost.index(10, 10)], -1)
        self.assertFalse(cost.free(11, 10))  # footprint cannot overlap unknown
        self.assertEqual(cost.data[cost.index(15, 15)], -1)
        self.assertFalse(cost.free(5, 5))
        self.assertFalse(cost.free(6, 5))
        self.assertGreater(cost.data[cost.index(8, 5)], 0)

    def test_diagonal_cannot_cross_blocked_corner(self):
        g = Grid(2, 2, 1., 0., 0., [0, 100, 100, 0])
        self.assertNotIn((1, 1), reachable(g, (0, 0)))

    def test_map_growth_changes_indices_but_not_world_position(self):
        old = Grid(10, 10, .1, -1., -1., [0] * 100)
        new = Grid(30, 30, .1, -2., -2., [0] * 900)
        for g in [old, new]:
            self.assertAlmostEqual(g.xy(*g.cell(-.45, -.45))[0], -.45)
        self.assertNotEqual(old.cell(-.45, -.45), new.cell(-.45, -.45))
        self.assertEqual(old.cell(-1.01, -1.01), (-1, -1))

    def partial_map(self):
        raw = Grid(40, 30, .1, -2., -1.5, [-1] * 1200)
        for r in range(4, 26):
            for c in range(4, 23):
                raw.data[raw.index(c, r)] = 0
        return raw

    def test_unknown_goal_selects_reachable_standoff_frontier(self):
        raw = self.partial_map()
        cost = inflate(raw)
        target, final = select_target(raw, cost, (-.8, 0.), (3., 0.), stand_off=.45)
        self.assertFalse(final)
        self.assertIn(cost.cell(*target), reachable(cost, cost.cell(-.8, 0.)))
        self.assertGreater(target[0], -.8)
        self.assertTrue(cost.free(*cost.cell(*target)))

    def test_reachable_goal_supersedes_exploration(self):
        raw = self.partial_map()
        self.assertEqual(select_target(raw, inflate(raw), (-.8, 0.), (-.5, 0.)),
                         ((-.5, 0.), True))

    def test_probabilistic_slam_boundary_does_not_hide_frontiers(self):
        raw = self.partial_map()
        for r in range(4, 26):
            raw.data[raw.index(22, r)] = 45
        result = select_target(raw, inflate(raw), (-.8, 0.), (3., 0.), stand_off=.45)
        self.assertIsNotNone(result)
        self.assertGreater(result[0][0], -.8)

    def test_exclusion_selects_different_frontier(self):
        raw = self.partial_map()
        cost = inflate(raw)
        first = select_target(raw, cost, (-.8, 0.), (3., 0.), stand_off=.45)[0]
        second = select_target(raw, cost, (-.8, 0.), (3., 0.), [first], stand_off=.45)[0]
        self.assertGreaterEqual(((first[0]-second[0])**2 + (first[1]-second[1])**2)**.5, .4)

    def test_enclosed_room_has_no_frontier(self):
        raw = Grid(20, 20, .1, 0., 0., [0] * 400)
        for c in range(20):
            raw.data[c] = raw.data[380 + c] = 100
        for r in range(20):
            raw.data[r * 20] = raw.data[r * 20 + 19] = 100
        self.assertIsNone(select_target(raw, inflate(raw), (1., 1.), (3., 3.)))

    def test_command_arc_rejects_new_obstacle_and_unknown(self):
        g = Grid(30, 30, .05, -.5, -.5, [0] * 900)
        self.assertTrue(trajectory_clear(g, (0., 0., 0.), .2, 0.))
        c, r = g.cell(.1, 0.)
        for value in [100, -1]:
            g.data[g.index(c, r)] = value
            self.assertFalse(trajectory_clear(g, (0., 0., 0.), .2, 0.))
        self.assertFalse(trajectory_clear(g, (9., 9., 0.), 0., 0.))

    def test_invalid_grid_is_rejected(self):
        with self.assertRaises(ValueError):
            Grid(3, 3, .1, 0., 0., [0])


if __name__ == '__main__':
    unittest.main()
