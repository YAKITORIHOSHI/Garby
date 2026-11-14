"""At-most-once cloud disposal relay; cloud intent never supplies motion clearance."""
import re
import time

REQUEST_MAX_AGE_MS = 30_000
FUTURE_TOLERANCE_MS = 5_000
RESULT_TIMEOUT_MS = 15_000
READY_MAX_AGE_S = 7.5


def valid_request(command, now_ms):
    if (type(command.get("version")) is not int or command["version"] != 1
            or not isinstance(command.get("requestId"), str)
            or not re.fullmatch(r"[0-9a-f]{32}", command["requestId"])
            or not isinstance(command.get("requestedBy"), str)
            or not 1 <= len(command["requestedBy"].strip()) <= 128
            or type(command.get("requestedAt")) is not int
            or command["requestedAt"] <= 0):
        return False
    return -FUTURE_TOLERANCE_MS <= now_ms - command["requestedAt"] <= REQUEST_MAX_AGE_MS


class DisposalRelay:
    """Persist the claim before queuing. A restart can only query that claim's ID.

    The terminal 'accepted' means START ACK, never arrival or motor movement.
    Cloud I/O runs only in the dedicated polling worker.
    """

    def __init__(self, control, enqueue, available):
        self.control = control
        self.enqueue = enqueue
        self.available = available

    def poll(self, manager, device_id, connected, blocked, now_ms=None):
        now_ms = int(time.time() * 1000) if now_ms is None else now_ms
        command = manager.get_disposal_command(device_id)
        if not isinstance(command, dict):
            return
        status = command.get("status")
        if status == "pending":
            if not valid_request(command, now_ms):
                manager.finish_disposal(device_id, command, "failed", "invalid_or_expired_request")
                return
            if blocked or not self.available():
                manager.finish_disposal(device_id, command, "failed",
                                        "emergency_stop" if blocked else "not_ready")
                return
            packet = self.control.request(connected)
            if packet is None:
                manager.finish_disposal(device_id, command, "failed", "not_ready")
                return
            controller_id = packet[7:-1]
            try:
                claimed = manager.claim_disposal(device_id, command, controller_id)
            except Exception:
                # A transaction may have committed despite a network timeout.
                # No START is sent; a later poll can only query the stored ID.
                self.control.cancel(controller_id)
                raise
            if not claimed:
                self.control.cancel(controller_id)
                return
            # Claim I/O consumes the same two-second outgoing queue deadline.
            if (not self.control.can_send(packet, connected)
                    or not valid_request(command, int(time.time() * 1000))):
                self.control.cancel(controller_id)
                manager.finish_disposal(device_id, command, "failed", "not_ready")
                return
            self.enqueue(packet)
        elif status == "dispatching":
            controller_id = command.get("controllerRequestId", "")
            claimed_at = command.get("claimedAt")
            if (not isinstance(controller_id, str)
                    or not re.fullmatch(r"[1-9][0-9]{0,15}", controller_id)
                    or type(claimed_at) is not int):
                manager.finish_disposal(device_id, command, "unconfirmed", "controller_result_unverified")
                return
            outcome = self.control.result(controller_id)
            if outcome is not None:
                accepted, reason = outcome
                manager.finish_disposal(device_id, command,
                                        "accepted" if accepted else "failed", reason)
                return
            # A known matching ACK survives a cloud outage; the timeout only
            # bounds waiting/querying when the controller result is unknown.
            if not -FUTURE_TOLERANCE_MS <= now_ms - claimed_at <= RESULT_TIMEOUT_MS:
                self.control.cancel(controller_id)
                manager.finish_disposal(device_id, command, "unconfirmed", "controller_result_unverified")
                return
            if blocked:
                self.control.cancel(controller_id)
                manager.finish_disposal(device_id, command, "unconfirmed", "emergency_stop")
                return
            if connected:
                self.control.recover_query(controller_id)
            # _drain_loop services status queries. It never repeats START.
