package com.thesis.garby.realtime

enum class DisposalStatus {
    Pending, Dispatching, Accepted, Failed, Unconfirmed, Unknown;

    companion object {
        fun fromWire(value: String?): DisposalStatus = when (value) {
            "pending" -> Pending
            "dispatching" -> Dispatching
            "accepted" -> Accepted
            "failed" -> Failed
            "unconfirmed" -> Unconfirmed
            else -> Unknown
        }
    }
}

data class DisposalCommand(
    val requestId: String,
    val requestedAtMs: Long,
    val requestedBy: String,
    val status: DisposalStatus,
    val reason: String? = null
) {
    val unresolved: Boolean get() = status == DisposalStatus.Pending ||
        status == DisposalStatus.Dispatching || status == DisposalStatus.Unknown
}

data class DisposalUiState(
    val command: DisposalCommand? = null,
    val loaded: Boolean = false,
    val submitting: Boolean = false,
    val notice: String? = null,
    val error: String? = null,
    val noticeRequestId: String? = null
) {
    /** A late server report resolves only the uncertainty for that exact request. */
    fun withCommand(incoming: DisposalCommand?): DisposalUiState {
        val resolved = incoming?.requestId == noticeRequestId &&
            incoming != null && incoming.status != DisposalStatus.Unknown
        return copy(command = incoming, loaded = true, error = null,
            notice = if (resolved) null else notice,
            noticeRequestId = if (resolved) null else noticeRequestId)
    }

    fun withSubmission(result: DisposalSubmission): DisposalUiState = when (result) {
        is DisposalSubmission.Sent -> copy(
            // A listener may already have delivered the controller's outcome.
            command = command?.takeIf { it.requestId == result.command.requestId ||
                it.requestedAtMs > result.command.requestedAtMs } ?: result.command,
            notice = null, noticeRequestId = null)
        is DisposalSubmission.NotSent -> copy(notice = result.message, noticeRequestId = null)
        is DisposalSubmission.Unconfirmed -> {
            val alreadyReported = command?.requestId == result.requestId &&
                command.status != DisposalStatus.Unknown
            copy(notice = if (alreadyReported) null else result.message,
                noticeRequestId = if (alreadyReported) null else result.requestId)
        }
    }
}

/** Sensor thresholds intentionally play no part. The Pi and MCU recheck current eligibility. */
fun disposalBlockReason(
    status: DeviceStatus?,
    cloudConnected: Boolean,
    internetAvailable: Boolean,
    disposal: DisposalUiState,
    nowMs: Long
): String? = when {
    !internetAvailable -> "Connect this phone to the internet."
    !cloudConnected -> "Waiting for the cloud connection."
    !disposal.loaded || disposal.error != null -> "Waiting for manual disposal status."
    disposal.submitting -> "Sending manual disposal request…"
    disposal.command?.unresolved == true -> "A manual disposal request is still waiting for confirmation."
    status == null || !isFreshTimestamp(status.lastSeenMs, nowMs) -> "Waiting for a fresh GARBY report."
    status.robotState != RobotState.Idle -> "GARBY must be idle at base."
    status.bleConnected != true -> "Waiting for GARBY's controller connection."
    status.manualDisposalReady != true || status.manualDisposalUpdatedAtMs <= 0L ||
        nowMs - status.manualDisposalUpdatedAtMs !in -5_000L..15_000L ->
        "Waiting for GARBY to confirm it is ready for manual disposal."
    else -> null
}

fun disposalMessage(command: DisposalCommand?, nowMs: Long): String? = command?.let {
    when (it.status) {
        DisposalStatus.Pending, DisposalStatus.Dispatching ->
            if (nowMs - it.requestedAtMs > 45_000L) {
                "No confirmation yet. Check GARBY before trying again."
            } else "Request sent. Waiting for GARBY to accept it."
        DisposalStatus.Accepted -> "Accepted by GARBY. Follow the robot status for arrival."
        DisposalStatus.Unconfirmed -> "Delivery not confirmed. Check GARBY before trying again."
        DisposalStatus.Unknown -> "Request status unavailable. Check GARBY."
        DisposalStatus.Failed -> when (it.reason) {
            "emergency_stop", "EMERGENCY_STOP" -> "Rejected: emergency stop is active."
            "not_ready", "NOT_READY", "NOT_IDLE" -> "Rejected: GARBY is no longer ready or idle."
            "invalid_or_expired_request" -> "Request expired or was invalid. Nothing was started by this request."
            "ROUTE_FAULT" -> "Rejected: GARBY needs supervised recovery at the robot."
            "MOTORS_UNAVAILABLE" -> "Rejected: motor outputs are unavailable."
            else -> "Manual disposal was rejected. Check GARBY before retrying."
        }
    }
}

sealed interface DisposalSubmission {
    data class Sent(val command: DisposalCommand) : DisposalSubmission
    data class NotSent(val message: String) : DisposalSubmission
    data class Unconfirmed(val message: String, val requestId: String) : DisposalSubmission
}

/** Used on every Firebase transaction retry, including a competing phone's write. */
fun canReplaceDisposal(
    expectedId: String?, currentId: String?, currentStatus: String?, exists: Boolean
): Boolean = currentId == expectedId && (!exists || currentStatus == "accepted" ||
    currentStatus == "failed" || currentStatus == "unconfirmed")
