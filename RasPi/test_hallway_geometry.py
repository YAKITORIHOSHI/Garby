"""Chassis zones and wall line fits, alone and through the production scan consumer."""
import math
import random
import unittest

import hallway_geometry as hg
from bridge_core import CoalescingBleQueue
import test_runtime as fixtures


def _ray_hit(ox, oy, dx, dy, segments):
    best = None
    for x1, y1, x2, y2 in segments:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
        if t > 0 and 0 <= u <= 1 and (best is None or t < best):
            best = t
    return best


def wall(y, x1=-2000.0, x2=2000.0):
    return (x1, y, x2, y)


def world_scan(segments, *, y0=0.0, heading_deg=0.0, rays=690, noise_cm=1.0, seed=3):
    """A robot at lateral y0 (cm, + left), rotated heading_deg (+ left)."""
    rnd = random.Random(seed)
    heading = math.radians(heading_deg)
    step = 2.0 * math.pi / rays
    ranges = []
    for index in range(rays):
        angle = -math.pi + index * step + heading
        hit = _ray_hit(0.0, y0, math.cos(angle), math.sin(angle), segments)
        ranges.append(0.0 if hit is None or hit > 800.0 else (hit + rnd.gauss(0.0, noise_cm)) / 100.0)
    return dict(ranges=ranges, angle_min=-math.pi, angle_increment=step,
                range_min=0.1, range_max=8.0)


HALLWAY = [wall(95.0), wall(-95.0)]


class WallFitTests(unittest.TestCase):
    def analyze(self, segments, **pose):
        return hg.analyze_hallway(**world_scan(segments, **pose))

    def test_centered_hallway_reads_the_measured_95_cm_walls(self):
        result = self.analyze(HALLWAY)
        self.assertAlmostEqual(result.left.distance_cm, 95.0, delta=1.0)
        self.assertAlmostEqual(result.right.distance_cm, 95.0, delta=1.0)
        self.assertAlmostEqual(result.heading_deg, 0.0, delta=0.3)
        self.assertFalse(any((result.front_stop, result.front_slow, result.side_left,
                              result.side_right, result.turn_front, result.turn_back)))

    def test_offset_and_heading_signs_are_left_positive(self):
        result = self.analyze(HALLWAY, y0=15.0, heading_deg=3.0)
        lateral = (result.right.distance_cm - result.left.distance_cm) / 2.0
        self.assertAlmostEqual(lateral, 15.0, delta=1.0)  # robot left of centre
        self.assertAlmostEqual(result.heading_deg, 3.0, delta=0.3)  # rotated left
        result = self.analyze(HALLWAY, y0=-25.0, heading_deg=-6.0)
        lateral = (result.right.distance_cm - result.left.distance_cm) / 2.0
        self.assertAlmostEqual(lateral, -25.0, delta=1.0)
        self.assertAlmostEqual(result.heading_deg, -6.0, delta=0.3)

    def test_open_door_room_does_not_become_the_right_wall(self):
        door = [wall(95.0), wall(-95.0, -2000, 20), wall(-95.0, 110, 2000),
                (20, -95, 20, -350), (110, -95, 110, -350), wall(-350.0, 20, 110)]
        result = self.analyze(door, y0=10.0)
        self.assertAlmostEqual(result.right.distance_cm, 105.0, delta=1.0)
        self.assertAlmostEqual(result.left.distance_cm, 85.0, delta=1.0)

    def test_posts_at_82_cm_do_not_replace_the_95_cm_walls(self):
        posts = HALLWAY + [(40, 82, 65, 82), (40, -82, 65, -82), (40, 82, 40, 95),
                           (65, 82, 65, 95), (40, -82, 40, -95), (65, -82, 65, -95)]
        result = self.analyze(posts)
        self.assertAlmostEqual(result.left.distance_cm, 95.0, delta=1.0)
        self.assertAlmostEqual(result.right.distance_cm, 95.0, delta=1.0)

    def test_door_leaf_swung_into_the_hallway_is_not_a_wall(self):
        tip = (60 + 90 * math.cos(math.radians(-20)), 95 + 90 * math.sin(math.radians(-20)))
        result = self.analyze(HALLWAY + [(60, 95, tip[0], tip[1])])
        self.assertAlmostEqual(result.left.distance_cm, 95.0, delta=1.5)
        self.assertAlmostEqual(result.heading_deg, 0.0, delta=0.5)

    def test_open_space_reports_no_wall_and_no_heading(self):
        result = self.analyze([])
        self.assertIsNone(result.left)
        self.assertIsNone(result.right)
        self.assertIsNone(result.heading_deg)

    def test_one_wall_still_gives_heading(self):
        result = self.analyze([wall(95.0)], heading_deg=-4.0)
        self.assertIsNone(result.right)
        self.assertAlmostEqual(result.heading_deg, -4.0, delta=0.3)

    def test_disagreeing_walls_keep_only_the_stronger_one(self):
        strong = hg.WallFit(95.0, 0.0, 120, 200.0, 1.0)
        weak = hg.WallFit(90.0, 8.0, 30, 70.0, 1.0)
        left, right, heading = hg.combine_walls(strong, weak)
        self.assertIs(left, strong)
        self.assertIsNone(right)
        self.assertEqual(heading, 0.0)


class ZoneTests(unittest.TestCase):
    @staticmethod
    def points(*points_cm, rays=720):
        """Clear 8 m scan with returns at the given robot-frame points (cm)."""
        step = 2.0 * math.pi / rays
        ranges = [0.0] * rays
        for x, y in points_cm:
            index = round((math.atan2(y, x) + math.pi) / step) % rays
            ranges[index] = math.hypot(x, y) / 100.0
        return hg.analyze_hallway(ranges, angle_min=-math.pi, angle_increment=step,
                                  range_min=0.1, range_max=8.0)

    def test_front_box_covers_the_swept_corridor_edge(self):
        # 30 deg off the nose at 69 cm: outside the 60 cm diagonal sector limit,
        # but inside the 60 cm chassis width plus its 10 cm margin.
        self.assertTrue(self.points((60.0, 35.0)).front_stop)
        self.assertFalse(self.points((60.0, 45.0)).front_stop)

    def test_slow_zone_lies_between_the_stop_and_160_cm(self):
        result = self.points((130.0, 0.0))
        self.assertFalse(result.front_stop)
        self.assertTrue(result.front_slow)
        self.assertFalse(self.points((170.0, 0.0)).front_slow)

    def test_side_band_is_two_centimetres_beyond_the_body(self):
        self.assertTrue(self.points((0.0, 31.0)).side_left)
        self.assertTrue(self.points((10.0, -32.0)).side_right)
        self.assertFalse(self.points((0.0, 33.0)).side_left)

    def test_turn_sweep_is_split_into_front_and_back_halves(self):
        result = self.points((-40.0, 20.0))  # behind the body, inside the spin circle
        self.assertTrue(result.turn_back)
        self.assertFalse(result.turn_front)
        result = self.points((35.0, 30.0))
        self.assertTrue(result.turn_front)

    def test_returns_inside_the_body_outline_are_self_returns(self):
        result = self.points((15.0, 12.0))
        self.assertFalse(any((result.front_stop, result.turn_front, result.turn_back)))

    def test_invalid_geometry_values_keep_defaults_and_warn(self):
        warnings = []
        geometry = hg.geometry_from_environment(
            {"GARBY_CHASSIS_WIDTH_CM": "nan", "GARBY_CHASSIS_FRONT_CM": "40"}, warnings.append)
        self.assertEqual(geometry.width_cm, 60.0)
        self.assertEqual(geometry.front_cm, 40.0)
        self.assertEqual(len(warnings), 1)

    def test_invalid_metadata_yields_no_analysis(self):
        self.assertIsNone(hg.analyze_hallway([1.0], angle_min=math.nan, angle_increment=0.1,
                                             range_min=0.1, range_max=8.0))


class RuntimeIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fixtures.load_runtime()
        self.node = self.runtime.LidarDistanceReader(None, None)

    @staticmethod
    def fields(packet):
        return dict(part.split("=", 1) for part in packet.split("|")[1:] if "=" in part)

    def deliver(self, ranges):
        scan = fixtures.RuntimeTests.scan(list(ranges))
        scan.angle_increment = 2.0 * math.pi / len(ranges)
        self.node.scan_callback(scan)
        path, sides = self.node.build_status_packets()
        return self.fields(path), self.fields(sides)

    def test_hallway_frames_carry_wall_fit_heading_and_age(self):
        scan = world_scan(HALLWAY, y0=10.0, heading_deg=2.0, rays=360)
        path, sides = self.deliver(scan["ranges"])
        self.assertEqual((path["F"], path["B"], path["T"]), ("C", "C", "C"))
        self.assertAlmostEqual(float(sides["WL"]), 85.0, delta=1.5)
        self.assertAlmostEqual(float(sides["WR"]), 105.0, delta=1.5)
        self.assertAlmostEqual(float(sides["H"]), 2.0, delta=0.5)
        self.assertGreaterEqual(int(sides["G"]), 0)
        self.assertIn("A", sides)

    def test_round_room_omits_wall_fields_instead_of_inventing_walls(self):
        _, sides = self.deliver([2.0] * 360)
        for key in ("WL", "WR", "H"):
            self.assertNotIn(key, sides)

    def test_turn_sweep_return_blocks_turns_and_respects_group_bypass(self):
        ranges = [2.0] * 360
        ranges[180 + 153] = math.hypot(40.0, 20.0) / 100.0  # (-40, -20): behind-right
        path, _ = self.deliver(ranges)
        self.assertEqual((path["F"], path["B"], path["T"]), ("C", "C", "O"))
        self.runtime.back_disabled.set()
        try:
            path, _ = self.deliver(ranges)
            self.assertEqual(path["T"], "C")
        finally:
            self.runtime.back_disabled.clear()

    def test_side_band_blocks_both_directions_and_is_never_bypassed(self):
        ranges = [2.0] * 360
        ranges[180 + 90] = 0.31  # 31 cm to the left: 1 cm beyond the 60 cm body
        self.runtime.front_disabled.set()
        self.runtime.back_disabled.set()
        try:
            path, _ = self.deliver(ranges)
        finally:
            self.runtime.front_disabled.clear()
            self.runtime.back_disabled.clear()
        self.assertEqual((path["F"], path["B"], path["T"]), ("O", "O", "O"))

    def test_monitor_footer_describes_the_wall_fit_without_a_display(self):
        from monitor_ui import wall_fit_text
        self.assertEqual(wall_fit_text("SIDES:L=95.0|WL=85.2|WR=104.9|H=2.01|G=120"),
                         "Walls L 85 R 105 cm, offset +10, heading +2.0 deg")
        self.assertEqual(wall_fit_text("SIDES:L=95.0|WR=104.9|H=-0.5"),
                         "Walls R 105 cm, heading -0.5 deg")
        for nothing in ("SIDES:STABLE", "SIDES:WL=abc", None):
            self.assertEqual(wall_fit_text(nothing), "Walls: no line fit")
        self.assertEqual(wall_fit_text("SIDES:WL=95.0|WR=95.0|Z=LEFT_BAND,TURN_BACK"),
                         "Walls L 95 R 95 cm, offset +0   |   Zones: left band, turn back")

    def test_zone_block_is_named_in_the_log_and_the_gui_frame(self):
        import queue
        node = self.runtime.LidarDistanceReader(None, queue.Queue(maxsize=5))
        ranges = [2.0] * 360
        ranges[180 + 90] = 0.31
        scan = fixtures.RuntimeTests.scan(ranges)
        with self.assertLogs(self.runtime.logger, level="INFO") as logs:
            node.scan_callback(scan)
        self.assertTrue(any("LiDAR chassis zones: LEFT_BAND" in line for line in logs.output))
        frame = node.data_queue.get_nowait()
        self.assertIn("|Z=LEFT_BAND", frame[4])
        self.assertNotIn("Z=", node.build_status_packets()[1])  # GUI only, never on BLE

    def test_zone_log_names_new_evidence_at_once_and_coalesces_a_flapping_boundary(self):
        # 2026-11-12 field log: TURN_FRONT toggled six times in one second for
        # an object on the zone boundary, one INFO line per scan.
        limiter = self.runtime.ZoneLogLimiter(hold_s=2.0)
        with_turn = ("FRONT_BOX", "RIGHT_BAND", "TURN_FRONT", "TURN_BACK")
        without = ("FRONT_BOX", "RIGHT_BAND", "TURN_BACK")
        logged = [now for now, zones in enumerate([without, with_turn] * 6)
                  if limiter.update(zones, now * 0.17)]
        self.assertEqual(logged, [0, 1])  # first state, then the added zone
        self.assertFalse(limiter.update(without, 1.9))
        self.assertTrue(limiter.update(without, 2.2))  # settled removal, once
        self.assertFalse(limiter.update(without, 9.0))
        self.assertTrue(limiter.update(with_turn, 9.1))  # new evidence, at once
        self.assertTrue(limiter.update((), 11.2))

    def test_wall_nearer_than_the_body_is_not_a_hallway_wall(self):
        # 60 cm body + 2 cm side margin: a "wall" at 31 cm is inside the side-band STOP.
        self.assertIsNone(hg.analyze_hallway(**world_scan([wall(31.0), wall(-95.0)])).left)
        # Just outside it, a real wall the robot has drifted close to still counts.
        near = hg.analyze_hallway(**world_scan([wall(36.0), wall(-154.0)], noise_cm=0.5))
        self.assertAlmostEqual(near.left.distance_cm, 36.0, delta=1.0)

    def test_mailbox_keeps_turn_clear_ordinary_and_anything_else_urgent(self):
        for packet, urgent in (("P:2|F=C|B=C|T=C", False), ("P:3|F=C|B=C|T=O", True),
                               ("P:4|F=W|B=C|T=C", True), ("P:5|F=C|B=C|T=S", True)):
            with self.subTest(packet=packet):
                mailbox = CoalescingBleQueue()
                mailbox.put_nowait("SENSOR:US=100")
                mailbox.put_nowait(packet)
                self.assertEqual(mailbox.get_nowait() == packet, urgent)


if __name__ == "__main__":
    unittest.main()
