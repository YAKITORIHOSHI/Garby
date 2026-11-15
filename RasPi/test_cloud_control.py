"""Phone control over Firebase: at most once, age limits, the monitor's own gates.

The cloud manager and the operator controller are fakes here; test_runtime.py
drives the same relay through the production functions.
"""
import base64
import struct
import unittest

import cloud_control as cc
from hallway_geometry import ChassisGeometry, analyze_hallway
from scan_frame import build_scan_frame
from test_hallway_geometry import HALLWAY, world_scan

NOW = 1_800_000_000_000


class FakeManager:
    """devices/<id>/commands/operator with the transaction semantics of production."""

    def __init__(self, command=None):
        self.command = command
        self.claim_fails = False

    def claim_operator(self, device_id, command):
        if self.claim_fails or self.command is None or self.command.get("status") != "pending":
            return False
        if self.command.get("requestId") != command.get("requestId"):
            return False
        self.command = {**self.command, "status": "claimed"}
        return True

    def finish_operator(self, device_id, command, status, reason=""):
        if (self.command is None or self.command.get("requestId") != command.get("requestId")
                or self.command.get("status") not in ("pending", "claimed")):
            return False
        self.command = {**self.command, "status": status, "reason": reason}
        return True


def request(action, request_id="a" * 32, age_ms=1_000, **extra):
    return {"version": 1, "requestId": request_id, "requestedAt": NOW - age_ms,
            "requestedBy": "uid-operator", "action": action, "status": "pending", **extra}


class RelayTests(unittest.TestCase):
    def setUp(self):
        self.requested = []
        self.outcomes = {}
        self.next_id = "501"
        self.refuse = None
        self.read_only = []

        def request_action(action):
            self.requested.append(action)
            if self.refuse:
                return None, self.refuse
            return self.next_id, ""
        self.relay = cc.OperatorRelay(request_action, self.outcomes.get,
                                      lambda group, enabled: (self.read_only.append((group, enabled))
                                                              or (True, "")))

    def run_to_end(self, manager, polls=3, step_ms=1_000):
        for k in range(polls):
            self.relay.poll(manager, "garby", manager.command, NOW + k * step_ms)
        return manager.command["status"], manager.command.get("reason", "")

    def test_an_accepted_action_is_sent_once_and_reports_the_controllers_answer(self):
        manager = FakeManager(request("SONAR_OFF"))
        self.relay.poll(manager, "garby", manager.command, NOW)
        self.assertEqual((self.requested, manager.command["status"]), (["SONAR_OFF"], "claimed"))
        self.relay.poll(manager, "garby", manager.command, NOW + 1_000)   # no answer yet
        self.assertEqual(manager.command["status"], "claimed")
        self.outcomes["501"] = (True, "SONAR_OFF")
        self.relay.poll(manager, "garby", manager.command, NOW + 2_000)
        self.assertEqual((manager.command["status"], manager.command["reason"]), ("accepted", ""))
        self.relay.poll(manager, "garby", manager.command, NOW + 3_000)
        self.assertEqual(self.requested, ["SONAR_OFF"])                   # never repeated

    def test_a_controller_refusal_and_a_local_refusal_are_reported_with_their_reason(self):
        manager = FakeManager(request("CONTINUE"))
        self.relay.poll(manager, "garby", manager.command, NOW)
        self.outcomes["501"] = (False, "NO_ROUTE")
        self.assertEqual(self.run_to_end(manager), ("failed", "NO_ROUTE"))
        self.refuse = "Controller settings confirmed"   # the gate said no: nothing was sent
        manager = FakeManager(request("RELEASE", "b" * 32))
        self.assertEqual(self.run_to_end(manager), ("failed", "Controller settings confirmed"))

    def test_no_answer_ends_unconfirmed_not_accepted(self):
        manager = FakeManager(request("RELEASE"))
        self.relay.poll(manager, "garby", manager.command, NOW)
        self.relay.poll(manager, "garby", manager.command, NOW + cc.RESULT_TIMEOUT_MS + 1)
        self.assertEqual((manager.command["status"], manager.command["reason"]),
                         ("unconfirmed", "controller_result_unverified"))
        self.assertEqual(self.requested, ["RELEASE"])

    def test_an_old_release_is_refused_but_an_old_stop_still_stops(self):
        for action in ("RELEASE", "CONTINUE", "REVERSE", "SONAR_ON", "REBOOT", "FRONT_READONLY_ON"):
            with self.subTest(action=action):
                manager = FakeManager(request(action, age_ms=cc.ACTION_MAX_AGE_MS + 1))
                self.assertEqual(self.run_to_end(manager), ("failed", "expired_request"))
        self.assertEqual((self.requested, self.read_only), ([], []))
        manager = FakeManager(request("ESTOP", age_ms=cc.ACTION_MAX_AGE_MS + 1))
        self.relay.poll(manager, "garby", manager.command, NOW)
        self.assertEqual((self.requested, manager.command["status"]), (["ESTOP"], "claimed"))
        # Older than its own limit: reported expired, yet the stop was applied.
        self.requested.clear()
        manager = FakeManager(request("ESTOP", "c" * 32, age_ms=cc.ESTOP_MAX_AGE_MS + 1))
        self.assertEqual(self.run_to_end(manager), ("failed", "expired_request"))
        self.assertEqual(self.requested, ["ESTOP"])

    def test_a_request_from_the_future_is_refused(self):
        manager = FakeManager(request("RELEASE", age_ms=-cc.FUTURE_TOLERANCE_MS - 1))
        self.assertEqual(self.run_to_end(manager), ("failed", "expired_request"))
        self.assertEqual(self.requested, [])

    def test_malformed_and_unknown_requests_are_refused_without_acting(self):
        bad = [request("RELEASE", version=2), request("RELEASE", requestId="short"),
               request("RELEASE", requestedBy=""), request("RELEASE", requestedAt="now"),
               request("LAUNCH"), {**request("RELEASE"), "action": 7}]
        for command in bad:
            manager = FakeManager(command)
            self.relay.poll(manager, "garby", manager.command, NOW)
            self.assertEqual(manager.command["status"], "failed", command)
        self.assertEqual(self.requested, [])
        self.relay.poll(FakeManager(), "garby", None, NOW)
        self.relay.poll(FakeManager(), "garby", "text", NOW)

    def test_a_lost_claim_does_not_act_except_for_a_stop(self):
        manager = FakeManager(request("RELEASE"))
        manager.claim_fails = True
        self.relay.poll(manager, "garby", manager.command, NOW)
        self.assertEqual(self.requested, [])
        manager = FakeManager(request("ESTOP"))
        manager.claim_fails = True
        self.relay.poll(manager, "garby", manager.command, NOW)
        self.assertEqual(self.requested, ["ESTOP"])

    def test_a_slow_claim_never_acts_on_a_request_that_expired_meanwhile(self):
        # The claim is a database transaction and can take seconds (2026-11-15:
        # the check after it reused the poll's own time and could never fire).
        clock = [0.0]

        class SlowClaim(FakeManager):
            def __init__(self, command, seconds):
                super().__init__(command)
                self.seconds = seconds

            def claim_operator(self, device_id, command):
                clock[0] += self.seconds
                return super().claim_operator(device_id, command)
        relay = cc.OperatorRelay(lambda action: (self.requested.append(action) or "501", ""),
                                 self.outcomes.get,
                                 lambda group, enabled: (self.read_only.append((group, enabled))
                                                         or (True, "")),
                                 clock=lambda: clock[0])
        too_slow = cc.ACTION_MAX_AGE_MS / 1000      # 1 s old when polled, 21 s when acted on
        for action, request_id in (("RELEASE", "a" * 32), ("FRONT_READONLY_ON", "b" * 32)):
            with self.subTest(action=action):
                manager = SlowClaim(request(action, request_id), too_slow)
                relay.poll(manager, "garby", manager.command, NOW)
                self.assertEqual((manager.command["status"], manager.command["reason"]),
                                 ("failed", "expired_request"))
        self.assertEqual((self.requested, self.read_only), ([], []))
        # A claim that leaves it fresh acts, and the controller's answer is awaited
        # from the moment the action was handed over, not from the start of the poll.
        manager = SlowClaim(request("SONAR_OFF", "c" * 32), 5.0)
        relay.poll(manager, "garby", manager.command, NOW)
        self.assertEqual((manager.command["status"], self.requested), ("claimed", ["SONAR_OFF"]))
        relay.poll(manager, "garby", manager.command, NOW + cc.RESULT_TIMEOUT_MS + 1)
        self.assertEqual(manager.command["status"], "claimed")
        relay.poll(manager, "garby", manager.command, NOW + 5_000 + cc.RESULT_TIMEOUT_MS + 1)
        self.assertEqual((manager.command["status"], self.requested), ("unconfirmed", ["SONAR_OFF"]))

    def test_a_request_claimed_before_a_restart_is_never_replayed(self):
        manager = FakeManager({**request("CONTINUE"), "status": "claimed"})
        self.relay.poll(manager, "garby", manager.command, NOW)
        self.assertEqual((manager.command["status"], self.requested), ("unconfirmed", []))

    def test_finished_requests_are_left_alone(self):
        for status in ("accepted", "failed", "unconfirmed"):
            manager = FakeManager({**request("RELEASE"), "status": status})
            self.relay.poll(manager, "garby", manager.command, NOW)
            self.assertEqual((manager.command["status"], self.requested), (status, []))

    def test_read_only_toggles_use_the_local_gate_and_finish_at_once(self):
        manager = FakeManager(request("FRONT_READONLY_ON"))
        self.relay.poll(manager, "garby", manager.command, NOW)
        self.assertEqual((manager.command["status"], self.read_only), ("accepted", [("front", True)]))
        relay = cc.OperatorRelay(lambda action: ("1", ""), {}.get,
                                 lambda group, enabled: (False, "not_permitted"))
        manager = FakeManager(request("BACK_READONLY_ON", "d" * 32))
        relay.poll(manager, "garby", manager.command, NOW)
        self.assertEqual((manager.command["status"], manager.command["reason"]),
                         ("failed", "not_permitted"))

    def test_a_stop_whose_acknowledgement_never_comes_says_so(self):
        manager = FakeManager(request("ESTOP"))
        self.relay.poll(manager, "garby", manager.command, NOW)
        self.relay.poll(manager, "garby", manager.command, NOW + cc.RESULT_TIMEOUT_MS + 1)
        self.assertEqual((manager.command["status"], manager.command["reason"]),
                         ("unconfirmed", "stop_requested_controller_unconfirmed"))


class MirrorTests(unittest.TestCase):
    def mirror(self, **status):
        return cc.operator_mirror(
            status, robot_state="OUTBOUND", front_read_only=False, back_read_only=True,
            read_only_allowed=True, motion_hold="LiDAR obstacle", zones=("FRONT_BOX", "SLOW"),
            route_leg="Outbound step 5 of 15: long hallway", people=1, objects=2,
            launcher={"mode": "headless"}, now_ms=NOW)

    def test_unknown_controller_state_never_reads_as_released_or_enabled(self):
        mirror = self.mirror()
        self.assertIsNone(mirror["emergencyStopped"])
        self.assertIsNone(mirror["ultrasonicEnabled"])
        self.assertFalse(any(mirror[key] for key in ("supported", "canRelease", "canContinue",
                                                    "canReverse", "canReset", "canConfigure")))

    def test_it_carries_what_the_controls_tab_shows(self):
        mirror = self.mirror(supported=True, connected=True, emergency_stopped=True,
                             ultrasonic_enabled=False, can_release=True, can_reverse=True,
                             message="Controller settings confirmed")
        self.assertEqual((mirror["emergencyStopped"], mirror["canRelease"], mirror["canReverse"],
                          mirror["canContinue"], mirror["zones"], mirror["people"],
                          mirror["backReadOnly"], mirror["robotState"]),
                         (True, True, True, False, "FRONT_BOX,SLOW", 1, True, "OUTBOUND"))

    def test_only_a_real_change_counts_as_changed(self):
        first = self.mirror()
        self.assertTrue(cc.mirror_changed(None, first))
        self.assertFalse(cc.mirror_changed(first, {**first, "updatedAt": NOW + 5_000}))
        self.assertTrue(cc.mirror_changed(first, {**first, "people": 0}))


class MapTests(unittest.TestCase):
    def frame(self, **pose):
        scan = world_scan(HALLWAY, **pose)
        frame = build_scan_frame(**scan)
        return frame, analyze_hallway(geometry=ChassisGeometry(), frame=frame, **scan)

    def test_the_lease_decides_whether_anyone_is_watching(self):
        self.assertTrue(cc.lease_fresh({"leaseAt": NOW - 20_000}, NOW))
        self.assertFalse(cc.lease_fresh({"leaseAt": NOW - cc.MAP_LEASE_MS - 1}, NOW))
        self.assertFalse(cc.lease_fresh({"leaseAt": NOW + cc.FUTURE_TOLERANCE_MS + 1}, NOW))
        for lease in (None, {}, {"leaseAt": "now"}, "x"):
            self.assertFalse(cc.lease_fresh(lease, NOW))

    def test_a_frame_holds_one_distance_per_degree_walls_and_markers(self):
        frame, hallway = self.frame()
        document = cc.encode_map_frame(
            frame, seq=7, scan_age_s=0.2, hallway=hallway, zones=("OBJECT_AHEAD",),
            path="F=W|B=C|T=C", hold="", now_ms=NOW,
            markers=((2.9, 0.1, "person", -1.2, 0.0), (1.0, -0.5, "object", 0.0, 0.0),
                     (1.0, 1.0, "ghost", 0.0, 0.0)))
        distances = struct.unpack("<360H", base64.b64decode(document["d"]))
        self.assertAlmostEqual(distances[90], 95, delta=3)     # the wall on the left
        self.assertAlmostEqual(distances[270], 95, delta=3)    # and on the right
        self.assertEqual(distances[0], 0)                      # down the hallway: no return
        self.assertAlmostEqual(distances[10], 535, delta=20)   # the wall, about 10 degrees to the left
        self.assertEqual(document["m"], [[290, 10, 1, -120, 0], [100, -50, 3, 0, 0]])
        self.assertAlmostEqual(document["wl"], 95.0, delta=1.5)
        self.assertAlmostEqual(document["h"], 0.0, delta=0.3)
        self.assertEqual((document["seq"], document["ageMs"], document["z"], document["p"]),
                         (7, 200, "OBJECT_AHEAD", "F=W|B=C|T=C"))
        import json
        self.assertLess(len(json.dumps(document)), 1500)

    def test_the_encoder_still_produces_the_frame_the_app_is_tested_against(self):
        # map_frame_golden.json is also the Android decoder's test input
        # (Garby-Mobile-App, app/src/test/resources): both sides meet on the same bytes.
        import json, math
        from pathlib import Path
        import numpy as np
        from hallway_geometry import HallwayAnalysis, WallFit
        from scan_frame import ScanFrame
        points = [(0.0, 280.0), (90.0, 95.0), (270.0, 120.0), (180.0, 36.0), (45.2, 410.0),
                  (44.9, 300.0)]
        xs = np.array([r * math.cos(math.radians(a)) for a, r in points])
        ys = np.array([r * math.sin(math.radians(a)) for a, r in points])
        frame = ScanFrame(np.arange(len(points)), xs, ys, np.hypot(xs, ys), math.radians(0.53), 690)
        hallway = HallwayAnalysis(False, False, False, False, False, False, None,
                                  WallFit(95.0, 0.5, 100, 200.0, 1.0),
                                  WallFit(120.0, 0.5, 90, 190.0, 1.1), -0.5)
        document = cc.encode_map_frame(
            frame, seq=42, scan_age_s=0.25, hallway=hallway,
            markers=((2.9, 0.1, "person", -1.2, 0.05), (1.5, -0.6, "moving", 0.0, 0.7),
                     (4.0, 0.0, "object", 0.0, 0.0)),
            zones=("OBJECT_AHEAD", "SLOW"), path="F=W|B=C|T=C", hold="LiDAR obstacle", now_ms=NOW)
        golden = json.loads(Path(__file__).with_name("map_frame_golden.json").read_text())
        self.assertEqual(document, golden)
        distances = struct.unpack("<360H", base64.b64decode(golden["d"]))
        self.assertEqual((distances[0], distances[90], distances[270], distances[180], distances[45]),
                         (280, 95, 120, 36, 300))
        self.assertEqual(sum(1 for value in distances if value), 5)

    def test_left_is_left(self):
        frame, _ = self.frame(y0=40.0)          # robot 40 cm left of centre
        distances = struct.unpack("<360H", base64.b64decode(
            cc.encode_map_frame(frame, seq=1, scan_age_s=0.1, now_ms=NOW)["d"]))
        self.assertAlmostEqual(distances[90], 55, delta=3)
        self.assertAlmostEqual(distances[270], 135, delta=3)

    def test_a_stale_or_missing_scan_is_never_published(self):
        frame, _ = self.frame()
        self.assertIsNone(cc.encode_map_frame(frame, seq=1, scan_age_s=cc.MAP_MAX_SCAN_AGE_S + 0.01))
        self.assertIsNone(cc.encode_map_frame(frame, seq=1, scan_age_s=-0.1))
        self.assertIsNone(cc.encode_map_frame(frame, seq=1, scan_age_s=float("nan")))
        self.assertIsNone(cc.encode_map_frame(None, seq=1, scan_age_s=0.1))

    def test_the_marker_list_and_the_rate_are_bounded(self):
        frame, _ = self.frame()
        many = tuple((1.0 + k * 0.1, 0.0, "object", 0.0, 0.0) for k in range(40))
        self.assertEqual(len(cc.encode_map_frame(frame, seq=1, scan_age_s=0.1, markers=many)["m"]),
                         cc.MAP_MAX_MARKERS)
        publisher = cc.MapPublisher(clock=lambda: 0.0)
        self.assertEqual([publisher.due(now=t) for t in (10.0, 10.2, 10.5, 10.6, 11.0)],
                         [True, False, True, False, True])
        self.assertEqual([publisher.due(hot=True, now=t) for t in (11.5, 12.0, 12.4, 13.0)],
                         [False, True, False, True])
        self.assertEqual(publisher.seq, 5)


if __name__ == "__main__":
    unittest.main()
