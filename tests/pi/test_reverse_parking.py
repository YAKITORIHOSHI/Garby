"""Reverse parking (owner, 2026-11-16): the Pi's rear-travel clearance R=.

Each route now ends by backing into its parking spot. Behind the robot only the
LiDAR sees, so every path packet carries R=, and the bridge grants GO between
[BACK:START] and [BACK:END] only on R=C (tests/firmware/test_bridge.cpp covers the
bridge and executor ends). Here: the rear zones, R= in every packet, the backing
route step on the Pi, the BLE mailbox priority and the moving-object rule.
"""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import math
import time
import queue
import unittest

import test_runtime as fixtures
from bridge_core import CoalescingBleQueue
from hallway_geometry import ChassisGeometry
from motion_tracker import EgoMotion, MotionTracker
from station_mode import StationMode
import test_forward_only_motion as forward_only
import test_motion_tracker as hallway_model
import test_hallway_geometry as wall_model

BACKING_AT_STATION = "[LEG:RF:12/13:K:1]"    # return route, last step, STEP_STATION
BACKING_AT_DESTINATION = "[LEG:OF:12/13:K:0]"


def at(scan, x_cm, y_cm):
    """Put one return at (forward, left) cm into a 1-degree runtime test scan."""
    angle = math.degrees(math.atan2(y_cm, x_cm))
    scan.ranges[int(round(angle + 180.0)) % 360] = math.hypot(x_cm, y_cm) / 100.0
    return scan


class RearZoneGeometryTests(unittest.TestCase):
    points = staticmethod(forward_only.SideBandGeometryTests.points)

    def test_the_rear_box_reaches_backing_stop_cm_behind_the_rear_edge(self):
        self.assertTrue(self.points((-40.0, 0.0)).rear_stop)      # 10 cm behind the edge
        self.assertTrue(self.points((-62.0, 35.0)).rear_stop)     # corridor corner
        self.assertFalse(self.points((-70.0, 0.0)).rear_stop)     # 40 cm behind: beyond it
        self.assertFalse(self.points((-40.0, 45.0)).rear_stop)    # outside the corridor

    def test_returns_inside_the_body_outline_are_the_robot_itself(self):
        self.assertFalse(self.points((-20.0, 0.0)).rear_stop)

    def test_the_rear_bands_cover_the_rear_half_only(self):
        self.assertTrue(self.points((-20.0, 30.5)).rear_side_left)
        self.assertTrue(self.points((-25.0, -31.5)).rear_side_right)
        self.assertFalse(self.points((15.0, 31.0)).rear_side_left)   # the forward band's
        # ... and the forward rule is unchanged: the rear band never holds a forward move.
        self.assertFalse(self.points((-20.0, 30.5)).side_left)

    def test_the_box_depth_is_a_chassis_setting(self):
        self.assertEqual(ChassisGeometry().backing_stop_cm, 35.0)


class RuntimeRearTravelTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fixtures.load_runtime()

    def path(self, scan):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        node.scan_callback(scan)
        return node, node.build_status_packets()[0].split("|", 1)[1]

    def test_every_path_packet_carries_the_rear_travel_code(self):
        _, path = self.path(fixtures.RuntimeTests.scan())
        self.assertEqual(path, "F=C|B=C|T=C|R=C")

    def test_something_close_behind_holds_backing_but_not_a_forward_move(self):
        _, path = self.path(at(fixtures.RuntimeTests.scan(), -40.0, 0.0))
        self.assertEqual(path, "F=C|B=C|T=O|R=O")

    def test_something_beyond_the_rear_box_does_not_hold_backing(self):
        _, path = self.path(at(fixtures.RuntimeTests.scan(), -75.0, 0.0))
        self.assertTrue(path.endswith("|R=C"), path)

    def test_beside_the_rear_half_holds_backing_only(self):
        _, path = self.path(at(fixtures.RuntimeTests.scan(), -20.0, 30.9))
        self.assertTrue(path.startswith("F=C|B=C|"), path)
        self.assertTrue(path.endswith("|R=O"), path)

    def test_read_only_back_bypasses_the_rear_box_but_never_the_rear_bands(self):
        self.runtime.back_disabled.set()
        try:
            _, path = self.path(at(fixtures.RuntimeTests.scan(), -40.0, 0.0))
            self.assertTrue(path.endswith("|R=C"), path)
            _, path = self.path(at(fixtures.RuntimeTests.scan(), -20.0, 30.9))
            self.assertTrue(path.endswith("|R=O"), path)
        finally:
            self.runtime.back_disabled.clear()

    def test_no_scan_gives_no_rear_code_which_the_bridge_never_takes_as_clear(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        self.assertEqual(node.build_status_packets()[0].split("|", 1)[1], "F=S|B=S")

    def test_a_backing_hold_is_named_and_shows_its_zone(self):
        runtime = self.runtime
        runtime.ble_connected.set()
        try:
            runtime.ble_notify_handler("[RUNNING]")
            runtime.ble_notify_handler(BACKING_AT_DESTINATION)
            node, path = self.path(at(fixtures.RuntimeTests.scan(), -40.0, 0.0))
            self.assertTrue(path.endswith("|R=O"))
            self.assertEqual(node.obstacle_hold_reason(), "Rear travel clearance blocked (REAR_BOX)")
            self.assertIn("REAR_BOX", node._active_zones())
            # The same return during a forward step: not a hold, not a shown zone.
            runtime.ble_notify_handler("[LEG:OF:10/13:S:0]")
            self.assertEqual(node.obstacle_hold_reason(), "")
            self.assertNotIn("REAR_BOX", node._active_zones())
        finally:
            runtime.ble_connected.clear()
            runtime.station_mode.invalidate()
            runtime.publish_robot_state("UNKNOWN")

    def test_backing_into_the_recorded_station_spot_is_not_held_by_its_walls(self):
        # The 2026-11-14 base spot: the wall 22.5 cm behind the LiDAR and a wall step
        # beside the rear corner. Backing into it ends exactly there.
        runtime = self.runtime
        runtime.ble_connected.set()
        try:
            for notice, expected in ((None, "|R=O"), (BACKING_AT_STATION, "|R=C")):
                with self.subTest(notice=notice):
                    runtime.ble_notify_handler("[RETURNING]")
                    if notice:
                        runtime.ble_notify_handler(notice)
                    else:
                        runtime.station_mode.invalidate()
                    node = runtime.LidarDistanceReader(None, queue.Queue())
                    paths = []
                    for scan in forward_only.recorded_scans():
                        node.scan_callback(scan)
                        paths.append(node.build_status_packets()[0])
                    self.assertEqual(len(paths), 15)
                    self.assertTrue(all(path.endswith(expected) for path in paths), paths)
        finally:
            runtime.ble_connected.clear()
            runtime.station_mode.invalidate()
            runtime.publish_robot_state("UNKNOWN")


class RearWallEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fixtures.load_runtime()
        self.runtime.LIDAR_REFLECT_LEFT_RIGHT = False
        self.runtime.LIDAR_YAW_OFFSET_DEG = 0.0

    def scan(self, segments, angle=0.0):
        msg = fixtures.RuntimeTests.scan()
        data = wall_model.world_scan(segments, heading_deg=angle, noise_cm=0.2)
        for key, value in data.items():
            setattr(msg, key, value)
        msg.ranges = [v if v > 0 else math.inf for v in msg.ranges]
        return msg

    def fields(self, node):
        _, sides = node.build_status_packets()
        return dict(item.split('=', 1) for item in sides.split('|')[1:] if '=' in item)

    def test_rear_wall_reports_its_own_heading_and_distance(self):
        for angle in (-6.0, 0.0, 4.0):
            with self.subTest(angle=angle):
                node = self.runtime.LidarDistanceReader(None, queue.Queue())
                node.scan_callback(self.scan([(-150, -220, -150, 220)], angle))
                fields = self.fields(node)
                self.assertIn('RH', fields)
                self.assertAlmostEqual(float(fields['RH']), angle, delta=0.4)
                self.assertAlmostEqual(float(fields['RD']), 150.0, delta=0.5)

    def test_station_mask_does_not_erase_the_rear_alignment_wall(self):
        runtime = self.runtime
        runtime.ble_connected.set()
        try:
            runtime.ble_notify_handler('[RETURNING]')
            runtime.ble_notify_handler(BACKING_AT_STATION)
            node = runtime.LidarDistanceReader(None, queue.Queue())
            node.scan_callback(self.scan([(-120, -220, -120, 220)], 3.0))
            fields = self.fields(node)
            self.assertTrue(node._station_walls)
            self.assertIn('RH', fields)
            self.assertAlmostEqual(float(fields['RH']), 3.0, delta=0.4)
        finally:
            runtime.ble_connected.clear()
            runtime.station_mode.invalidate()
            runtime.publish_robot_state('UNKNOWN')

    def test_the_developer_tab_sees_the_same_rear_wall_and_never_an_old_one(self):
        # Display only (owner, 2026-11-16): check the fit on the bench before parking.
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        node.scan_callback(self.scan([(-150, -220, -150, 220)], 2.0))
        view = self.runtime.rear_wall_view()
        self.assertAlmostEqual(view["distance_cm"], 150.0, delta=0.5)
        self.assertAlmostEqual(view["angle_deg"], 2.0, delta=0.4)
        self.assertIsNone(self.runtime.rear_wall_view(now=time.monotonic() + 1.1))
        node.scan_callback(self.scan([(150, -220, 150, 220)]))
        self.assertIsNone(self.runtime.rear_wall_view())

    def test_a_short_panel_or_wall_to_one_side_is_not_the_parking_wall(self):
        for segment in [(-120, -20, -120, 20), (-120, 50, -120, 200),
                        (-100, 80, 200, 80), (150, -220, 150, 220)]:
            with self.subTest(segment=segment):
                node = self.runtime.LidarDistanceReader(None, queue.Queue())
                node.scan_callback(self.scan([segment]))
                fields = self.fields(node)
                self.assertNotIn('RH', fields)
                self.assertNotIn('RD', fields)

    def test_mount_transform_is_shared_with_the_map_and_clearance(self):
        self.runtime.LIDAR_REFLECT_LEFT_RIGHT = True
        self.runtime.LIDAR_YAW_OFFSET_DEG = 7.0
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        node.scan_callback(self.scan([(-150, -220, -150, 220)], 3.0))
        self.assertAlmostEqual(float(self.fields(node)['RH']), 4.0, delta=0.4)

    def test_recorded_station_wall_remains_measurable_despite_its_corner_step(self):
        # Recorded raw mounting uses the runtime's default reflection.
        self.runtime.LIDAR_REFLECT_LEFT_RIGHT = True
        runtime = self.runtime
        runtime.ble_connected.set()
        try:
            runtime.ble_notify_handler('[RETURNING]')
            runtime.ble_notify_handler(BACKING_AT_STATION)
            node = runtime.LidarDistanceReader(None, queue.Queue())
            measured = []
            for scan in forward_only.recorded_scans():
                node.scan_callback(scan)
                fields = self.fields(node)
                measured.append(float(fields['RH']))
                self.assertAlmostEqual(float(fields['RD']), 22.4, delta=0.6)
            self.assertEqual(len(measured), 15)
            self.assertLess(max(measured) - min(measured), 1.0)
        finally:
            runtime.ble_connected.clear()
            runtime.station_mode.invalidate()
            runtime.publish_robot_state('UNKNOWN')


class BackingLegNoticeTests(unittest.TestCase):
    def test_the_backing_step_is_parsed_and_described(self):
        mode = StationMode(clock=lambda: 10.0)
        mode.update_from_line(BACKING_AT_STATION)
        leg = mode.leg()
        self.assertTrue(leg.backing)
        self.assertTrue(leg.station)
        self.assertEqual(leg.describe(),
                         "Return step 13 of 13: backing into place · station step, station walls ignored")

    def test_the_way_back_of_a_reverse_run_never_backs(self):
        mode = StationMode(clock=lambda: 10.0)
        mode.update_from_line("[LEG:RB:12/13:K:1]")
        self.assertFalse(mode.leg().backing)

    def test_a_stale_backing_notice_is_no_longer_backing(self):
        now = [10.0]
        mode = StationMode(clock=lambda: now[0])
        mode.update_from_line(BACKING_AT_DESTINATION)
        now[0] += 4.0
        self.assertFalse(mode.leg().backing)


class MailboxTests(unittest.TestCase):
    def test_a_clear_rear_keeps_an_all_clear_ordinary_and_a_blocked_rear_is_urgent(self):
        for packet, urgent in (("P:2|F=C|B=C|T=C|R=C", False), ("P:3|F=C|B=C|R=C", False),
                               ("P:4|F=C|B=C|T=C|R=O", True), ("P:5|F=C|B=C|T=C|R=S", True)):
            with self.subTest(packet=packet):
                mailbox = CoalescingBleQueue()
                mailbox.put_nowait("SENSOR:US=100")
                mailbox.put_nowait(packet)
                self.assertEqual(mailbox.get_nowait() == packet, urgent)


class BackingMoverTests(unittest.TestCase):
    """A walker coming up behind: no reason to stop while driving forward away from
    them (they catch up), every reason while backing toward them."""

    def run_robot(self, speed):
        world = hallway_model.Hallway(seed=3)
        world.add_person(-320.0, 5.0, 100.0, 0.0)    # behind the robot, walking toward it
        tracker = MotionTracker(hallway_model.GEOMETRY)
        dt = 1.0 / hallway_model.SCAN_HZ
        robot_x, stops = 0.0, []
        for k in range(int(3.0 * hallway_model.SCAN_HZ)):
            t = 100.0 + k * dt
            scan = world.scan(robot_x)
            result = tracker.update(scan, angle_min=hallway_model.ANGLE_MIN,
                                    angle_increment=hallway_model.STEP, range_min=0.10,
                                    range_max=hallway_model.RANGE_MAX_M, scan_time=t,
                                    ego=EgoMotion("F", speed, 0.0, t),
                                    heading_deg=hallway_model.wall_heading(scan))
            stops.append(result.stop)
            robot_x += speed * dt
            world.step_people(dt)
        return stops

    def test_a_walker_behind_stops_a_backing_robot(self):
        self.assertTrue(any(self.run_robot(-25.0)))

    def test_the_same_walker_does_not_stop_a_robot_driving_forward(self):
        self.assertFalse(any(self.run_robot(25.0)))


if __name__ == "__main__":
    unittest.main()
