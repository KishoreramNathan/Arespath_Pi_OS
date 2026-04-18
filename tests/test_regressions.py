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
            self.assertEqual(map_json.get("render_scale"), 1)
        finally:
            runtime.stop()
            cameras.stop()


if __name__ == "__main__":
    unittest.main()


class LocalizationRegressionTests(unittest.TestCase):
    def test_telemetry_lidar_localizer_does_not_double_integrate_odometry(self):
        from app.robot.lidar_localization import TelemetryLidarLocalizer

        loc = TelemetryLidarLocalizer()
        pose0 = Pose(0.0, 0.0, 0.0)
        loc.reset(pose0)
        first = loc.update(pose0, 0, 0, 1.0, [], OccupancyGridMap())
        second = loc.update(first.pose, 1320, 1320, 2.0, [], OccupancyGridMap())

        wheel_circ = 2.0 * 3.141592653589793 * config.WHEEL_RADIUS_M
        expected_ds = wheel_circ * (1320 / config.TICKS_PER_WHEEL_REV)
        self.assertAlmostEqual(second.pose.x, expected_ds, places=4)
        self.assertAlmostEqual(second.pose.y, 0.0, places=4)
        self.assertAlmostEqual(second.pose.theta, 0.0, places=4)

    def test_planner_result_clears_replan_after_obstacle_flag(self):
        from app.robot.global_planner_node import PlanResult
        from app.robot.trajectory import SmoothedPath

        runtime = RobotRuntime()
        try:
            runtime._running = True
            runtime._replanning_after_obs = True
            runtime.state.nav.goal = (1.0, 0.0)
            result = PlanResult(
                goal=(1.0, 0.0),
                path=[(0.0, 0.0), (1.0, 0.0)],
                trajectory=[],
                smoothed=SmoothedPath(),
                stamp=123.0,
            )

            class _PlannerStub:
                def __init__(self, res):
                    self.res = res
                    self.used = False
                def get_latest_result(self):
                    if self.used:
                        runtime._running = False
                        return None
                    self.used = True
                    return self.res
                def stop(self):
                    return None

            runtime.global_planner = _PlannerStub(result)
            runtime._planner_loop()
            self.assertFalse(runtime._replanning_after_obs)
            self.assertTrue(runtime._path_valid)
        finally:
            runtime.stop()
