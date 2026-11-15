"""Station legs: the station's walls stop holding the robot, nothing else does.

Synthetic scans of a robot parked with its back to a wall (the layout of the
2026-11-14 live run) and driving at one, plus the recorded station scans when
this checkout has them. Geometry only: it cannot show a real parking run.
"""
import math
from pathlib import Path
import unittest

import numpy as np

from hallway_geometry import ChassisGeometry, analyze_hallway
from scan_frame import build_scan_frame
import station_mode as sm
from test_hallway_geometry import world_scan

GEOMETRY = ChassisGeometry()
STATION_SCANS = (Path(__file__).resolve().parents[1] / "artifacts"
                 / "manual-disposal-hold-2026-11-14" / "scans-base.npz")


def box(cx, cy, half=12.0):
    return [(cx - half, cy - half, cx + half, cy - half), (cx + half, cy - half, cx + half, cy + half),
            (cx + half, cy + half, cx - half, cy + half), (cx - half, cy + half, cx - half, cy - half)]


def circle(cx, cy, radius, sides=24):
    points = [(cx + radius * math.cos(2 * math.pi * k / sides),
               cy + radius * math.sin(2 * math.pi * k / sides)) for k in range(sides + 1)]
    return [(*points[k], *points[k + 1]) for k in range(sides)]


# Parked with the back to the wall: the wall 36 cm behind the LiDAR (6 cm behind
# the body), the far hallway wall 154 cm ahead, an alcove side wall on the left.
PARKED = [(-36.0, -400.0, -36.0, 400.0), (154.0, -400.0, 154.0, 400.0),
          (-36.0, 55.0, 30.0, 55.0)]
# Driving at the station wall, 70 cm ahead of the front edge.
APPROACH = [(100.0, -400.0, 100.0, 400.0), (-90.0, -400.0, -90.0, 400.0)]


def analyse(segments, station, extra=(), **pose):
    scan = world_scan(list(segments) + list(extra), **pose)
    frame = build_scan_frame(lidar_y_cm=GEOMETRY.lidar_y_cm, **scan)
    ranges, taken = scan["ranges"], 0
    if station:
        ranges, frame, taken = sm.without_station_walls(ranges, frame, GEOMETRY)
    meta = {k: scan[k] for k in ("angle_min", "angle_increment", "range_min", "range_max")}
    return analyze_hallway(ranges, geometry=GEOMETRY, frame=frame, **meta), ranges, taken


def held(result):
    return [name for name in ("front_stop", "front_slow", "side_left", "side_right",
                              "turn_front", "turn_back") if getattr(result, name)]


class LegNoticeTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.station = sm.StationMode(clock=lambda: self.now)

    def test_a_station_step_is_active_only_while_its_notice_is_fresh(self):
        self.assertFalse(self.station.active())
        self.assertTrue(self.station.update_from_line("[LEG:OF:1/15:S:5]"))
        leg = self.station.leg()
        self.assertEqual((leg.route, leg.reverse, leg.step, leg.count, leg.op, leg.flags),
                         ("O", False, 1, 15, "S", 5))
        self.assertTrue(self.station.active())
        self.assertIn("Outbound step 2 of 15", leg.describe())
        self.assertIn("station walls ignored", leg.describe())
        self.now += sm.FRESH_S - 0.1
        self.assertTrue(self.station.active())
        self.now += 0.2
        self.assertFalse(self.station.active())      # the repeat did not arrive
        self.assertEqual(self.station.leg().route, "")

    def test_an_ordinary_step_the_end_notice_and_a_restart_are_not_station_mode(self):
        self.station.update_from_line("[LEG:OF:4/15:H:0]")
        self.assertFalse(self.station.active())
        self.assertIn("long hallway", self.station.leg().describe())
        self.station.update_from_line("[LEG:RB:13/15:S:3]")
        self.assertTrue(self.station.active())
        self.assertIn("way back", self.station.leg().describe())
        self.station.update_from_line("[LEG:0]")
        self.assertFalse(self.station.active())
        self.station.update_from_line("[LEG:RF:14/15:L:3]")
        self.station.invalidate()
        self.assertFalse(self.station.active())

    def test_malformed_notices_end_station_mode_and_other_lines_are_not_consumed(self):
        for line in ("[LEG:OF:1/15:S:999]", "[LEG:OF:15/15:S:1]", "[LEG:XF:1/15:S:1]",
                     "[LEG:OF:1/15:S:1", "[LEG:OF:1/15:Q:1]", "[LEG:]"):
            self.station.update_from_line("[LEG:OF:1/15:S:1]")
            self.assertTrue(self.station.update_from_line(line), line)
            self.assertFalse(self.station.active(), line)
        self.station.update_from_line("[LEG:OF:1/15:S:1]")
        self.assertFalse(self.station.update_from_line("[DRIVE:F:71]"))
        self.assertTrue(self.station.active())

    def test_a_notice_from_the_future_or_the_past_clock_is_not_fresh(self):
        self.station.update_from_line("[LEG:OF:1/15:S:1]")
        self.now -= 1.0
        self.assertFalse(self.station.active())


class StationWallTests(unittest.TestCase):
    def test_parked_against_the_wall_is_held_without_station_mode_only(self):
        ordinary, _, _ = analyse(PARKED, station=False)
        self.assertIn("turn_back", held(ordinary))
        station, ranges, taken = analyse(PARKED, station=True)
        self.assertEqual(held(station), [])
        self.assertGreater(taken, 100)
        self.assertTrue(all(math.isinf(r) or r == 0.0 or r > 0 for r in ranges))

    def test_driving_at_the_station_wall_is_held_without_station_mode_only(self):
        ordinary, _, _ = analyse(APPROACH, station=False)
        self.assertIn("front_slow", held(ordinary))
        nearer, _, _ = analyse([(70.0, -400.0, 70.0, 400.0)], station=False)
        self.assertIn("front_stop", held(nearer))
        self.assertEqual(held(analyse(APPROACH, station=True)[0]), [])
        self.assertEqual(held(analyse([(70.0, -400.0, 70.0, 400.0)], station=True)[0]), [])

    def test_a_person_or_a_bin_at_the_station_still_holds(self):
        for name, obstacle in (("bin ahead", box(70.0, 0.0)), ("person ahead", circle(80.0, 5.0, 20.0)),
                               ("bin beside", box(15.0, 44.0, 10.0)),
                               ("person behind in the spin circle", circle(0.0, -44.0, 12.0))):
            with self.subTest(obstacle=name):
                ordinary, _, _ = analyse(PARKED, station=False, extra=obstacle)
                station, _, _ = analyse(PARKED, station=True, extra=obstacle)
                self.assertTrue(held(station), name)
                # Whatever the obstacle itself held is still held.
                wall_only = set(held(analyse(PARKED, station=False)[0]))
                self.assertTrue((set(held(ordinary)) - wall_only) <= set(held(station)), name)

    def test_a_wall_corner_and_a_turned_robot_are_still_wall(self):
        for heading in (0.0, 20.0, -35.0, 90.0):
            with self.subTest(heading=heading):
                self.assertEqual(held(analyse(PARKED, station=True, heading_deg=heading)[0]), [])

    def test_short_flat_things_standing_free_are_not_wall(self):
        board = [(60.0, -20.0, 60.0, 20.0)]   # 40 cm wide, 30 cm ahead of the front edge
        self.assertIn("front_stop", held(analyse([], station=True, extra=board)[0]))
        self.assertIn("front_stop", held(analyse(PARKED, station=True, extra=board)[0]))
        self.assertIn("front_stop", held(analyse(PARKED, station=True, extra=box(70.0, 0.0, 15.0))[0]))

    def test_a_wall_step_joined_to_the_station_wall_is_station_wall(self):
        # The layout of the recorded station: the back wall, then a 13 cm step and a
        # 35 cm return right beside the body, inside the circle a turn sweeps.
        stepped = [(-36.0, -400.0, -36.0, 31.0), (-36.0, 31.0, -23.0, 31.0), (-23.0, 31.0, -23.0, 66.0),
                   (154.0, -400.0, 154.0, 400.0)]
        ordinary, _, _ = analyse(stepped, station=False, noise_cm=0.2)
        self.assertIn("turn_back", held(ordinary))
        self.assertEqual(held(analyse(stepped, station=True, noise_cm=0.2)[0]), [])

    def test_a_person_standing_against_the_station_wall_still_holds(self):
        # Touching the back wall, inside the spin circle: one continuous outline with
        # the wall, but curved.
        for radius in (14.0, 20.0):
            with self.subTest(radius=radius):
                person = circle(-36.0 + radius - 2.0, 40.0, radius)
                self.assertIn("turn_back",
                              held(analyse(PARKED[:2], station=True, extra=person, noise_cm=0.2)[0]))

    def test_the_minimum_gap_keeps_a_wall_about_to_be_touched(self):
        touching = [(32.0, -400.0, 32.0, 400.0)]   # 2 cm ahead of the front edge
        scan = world_scan(touching)
        frame = build_scan_frame(lidar_y_cm=GEOMETRY.lidar_y_cm, **scan)
        self.assertEqual(held(analyse(touching, station=True)[0]), [])
        ranges, kept, _ = sm.without_station_walls(scan["ranges"], frame, GEOMETRY, min_gap_cm=3.0)
        meta = {k: scan[k] for k in ("angle_min", "angle_increment", "range_min", "range_max")}
        self.assertIn("front_stop", held(analyze_hallway(ranges, geometry=GEOMETRY, frame=kept, **meta)))

    def test_no_wall_changes_nothing(self):
        scan = world_scan(circle(120.0, 0.0, 20.0))
        frame = build_scan_frame(lidar_y_cm=GEOMETRY.lidar_y_cm, **scan)
        ranges, same, taken = sm.without_station_walls(scan["ranges"], frame, GEOMETRY)
        self.assertEqual(taken, 0)
        self.assertIs(ranges, scan["ranges"])
        self.assertIs(same, frame)
        self.assertEqual(sm.without_station_walls([], None, GEOMETRY), ([], None, 0))

    @unittest.skipUnless(STATION_SCANS.is_file(), "station recording not in this checkout")
    def test_the_recorded_station_scans_of_2026_11_14(self):
        from lidar_geometry import robot_scan_angles
        data = np.load(STATION_SCANS)
        ordinary_held = station_held = taken_total = 0
        for row, length, meta in zip(data["ranges"], data["lengths"], data["meta"]):
            angle_min, increment, range_min, range_max = (float(v) for v in meta[:4])
            amin, inc = robot_scan_angles(angle_min, increment, yaw_offset_deg=0.0,
                                          reflect_left_right=True)
            scan = dict(angle_min=amin, angle_increment=inc, range_min=range_min,
                        range_max=range_max)
            ranges = row[:int(length)].tolist()
            frame = build_scan_frame(ranges, lidar_y_cm=GEOMETRY.lidar_y_cm, **scan)
            ordinary_held += bool(held(analyze_hallway(ranges, geometry=GEOMETRY, frame=frame, **scan)))
            cleared, kept, taken = sm.without_station_walls(ranges, frame, GEOMETRY)
            taken_total += taken
            # The far hallway wall's own kink is in the caution zone: slower, not a hold.
            station_held += bool(set(held(analyze_hallway(cleared, geometry=GEOMETRY, frame=kept,
                                                          **scan))) - {"front_slow"})
        print(f"INFO recorded station scans: {ordinary_held}/15 held by zones ordinarily, "
              f"{station_held}/15 in station mode, {taken_total / 15:.0f} wall returns taken out per scan")
        self.assertEqual(ordinary_held, 15)
        self.assertEqual(station_held, 0)


if __name__ == "__main__":
    unittest.main()
