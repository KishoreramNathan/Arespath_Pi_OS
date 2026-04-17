import unittest

import numpy as np

from app import config
from app.robot import runtime_cfg as runtime_cfg_mod

runtime_cfg_mod.init(config.RUNTIME_SETTINGS_FILE)

from app.robot.control import RobotRuntime  # noqa: E402
from app.robot.mapping import OccupancyGridMap  # noqa: E402
from app.robot.state import Pose  # noqa: E402


class MappingRegressionTests(unittest.TestCase):
    def test_png_cache_refreshes_immediately_when_map_is_dirty(self):
        occ = OccupancyGridMap()
        before = occ.map_payload()["image_png_b64"]
        occ.update_from_scan(Pose(0.0, 0.0, 0.0), [(0.0, 1.0)])
        after = occ.map_payload(Pose(0.0, 0.0, 0.0))["image_png_b64"]
        self.assertNotEqual(before, after)

    def test_live_scan_overlay_does_not_mutate_saved_map(self):
        runtime = RobotRuntime()
        try:
            runtime.state.pose = Pose(0.0, 0.0, 0.0)
            runtime.state.nav.goal = (2.0, 0.0)
            runtime.state.nav.active = True
            before = runtime.map.log_odds.copy()
            runtime._plan_path(scan=[(0.0, 0.8), (0.1, 0.9), (-0.1, 0.9)])
            after = runtime.map.log_odds
            self.assertTrue(np.array_equal(before, after))
        finally:
            runtime.stop()


class ApiSmokeTests(unittest.TestCase):
    def test_core_routes_respond(self):
        from app.main import app, cameras, runtime

        client = app.test_client()
        try:
            health = client.get("/healthz")
            self.assertEqual(health.status_code, 200)
            self.assertTrue(health.get_json()["ok"])

            status = client.get("/api/status")
            self.assertEqual(status.status_code, 200)
            status_json = status.get_json()
            self.assertIn("mode", status_json)
            self.assertIn("pose", status_json)

            map_data = client.get("/api/map/data")
            self.assertEqual(map_data.status_code, 200)
            map_json = map_data.get_json()
            self.assertIn("image_png_b64", map_json)
            self.assertIn("resolution", map_json)
        finally:
            runtime.stop()
            cameras.stop()


if __name__ == "__main__":
    unittest.main()
