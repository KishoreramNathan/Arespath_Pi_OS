import unittest

from app import config
from app.robot.mapping import OccupancyGridMap


class MergeConfigTests(unittest.TestCase):
    def test_large_area_grid_defaults(self):
        self.assertEqual(config.MAP_SIZE_CELLS, 250)
        self.assertAlmostEqual(config.MAP_RESOLUTION_M, 0.25, places=5)
        self.assertAlmostEqual(config.MAP_ORIGIN_X_M, -31.25, places=2)
        self.assertAlmostEqual(config.MAP_ORIGIN_Y_M, -31.25, places=2)

    def test_grid_shape_matches_config(self):
        grid = OccupancyGridMap()
        self.assertEqual(grid.log_odds.shape, (250, 250))


if __name__ == '__main__':
    unittest.main()
