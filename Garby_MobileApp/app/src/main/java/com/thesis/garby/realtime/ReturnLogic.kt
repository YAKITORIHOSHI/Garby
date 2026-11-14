package com.thesis.garby.realtime

/** Each condition the app checks before it writes a return-to-base request. */
enum class ReturnCheck {
    /** The Pi's destination flag is set and, when published, its state is AT_DESTINATION. */
    AtDestination,

    /** The Pi heartbeat is fresh. */
    RobotOnline,

    /** The Pi reports its BLE link to the bridge. */
    BleLink,

    /** The Pi reports usable LiDAR scans. */
    Lidar,

    /** Firebase reports a live connection. */
    Cloud,

    /** Android reports validated internet. */
    Internet,

    /** No earlier request is still waiting for acceptance or driving back. */
    NoReturnInFlight
}

data class ReturnCheckResult(val check: ReturnCheck, val passed: Boolean)

data class ReturnReadiness(val results: List<ReturnCheckResult>) {
    val canSend: Boolean get() = results.isNotEmpty() && results.all { it.passed }

    /** The first unmet condition, in the order the screen lists them. */
    val firstUnmet: ReturnCheck? get() = results.firstOrNull { !it.passed }?.check

    fun passed(check: ReturnCheck): Boolean = results.firstOrNull { it.check == check }?.passed == true

    companion object {
        val Blocked = ReturnReadiness(ReturnCheck.entries.map { ReturnCheckResult(it, false) })
    }
}

/**
 * The app's return gate. It never grants motion; it only decides whether the app may ask.
 *
 * Destination readiness is strict because the Pi relays any fresh request: a request reaching
 * the executor while it is still driving outbound latches a route fault (`[RETURN REJECTED]`).
 * The Pi's flag must be set and, when the Pi publishes a state, that state must agree.
 */
fun evaluateReturnReadiness(
    status: DeviceStatus?,
    cloudConnected: Boolean,
    internetAvailable: Boolean,
    returnInFlight: Boolean,
    nowMs: Long
): ReturnReadiness {
    val atDestination = status != null && status.isReadyToReturn == true &&
        (status.robotState == null || status.robotState == RobotState.AtDestination)
    return ReturnReadiness(
        listOf(
            ReturnCheckResult(ReturnCheck.AtDestination, atDestination),
            ReturnCheckResult(ReturnCheck.RobotOnline, robotPresence(status, nowMs) == RobotPresence.Live),
            ReturnCheckResult(ReturnCheck.BleLink, status?.bleConnected == true),
            ReturnCheckResult(ReturnCheck.Lidar, status?.lidarHealthy == true),
            ReturnCheckResult(ReturnCheck.Cloud, cloudConnected),
            ReturnCheckResult(ReturnCheck.Internet, internetAvailable),
            ReturnCheckResult(ReturnCheck.NoReturnInFlight, !returnInFlight)
        )
    )
}

/** Where one return request stands, from the Pi's recorded status and the elapsed time. */
enum class ReturnStage {
    /** Written; the Pi has not accepted it and it is still inside the Pi's acceptance window. */
    WaitingForAcceptance,

    /** Still pending after the Pi's two-minute acceptance window: it has most likely expired. */
    NotAccepted,

    /** The executor accepted the return (its ACK, relayed by the Pi) and is driving back. */
    Returning,

    /** Accepted, but no outcome inside the 15-minute completion window. */
    Overdue,

    /** The executor reported the matching completion. */
    BackAtBase,

    /** The executor or the Pi recorded a failure; see [ReturnCommand.reason]. */
    Failed,

    /** The status text is not one the app recognises. */
    Unknown
}

data class ReturnProgress(
    val command: ReturnCommand,
    val stage: ReturnStage,
    /** This phone sent the request shown. */
    val ownRequest: Boolean,
    /** This phone sent a request that a newer request, from any operator, has replaced. */
    val ownRequestReplaced: Boolean
) {
    /** A request the robot may still act on. The app does not send another meanwhile. */
    val inFlight: Boolean
        get() = stage == ReturnStage.WaitingForAcceptance || stage == ReturnStage.Returning
}

/**
 * Describes the latest request in Firebase. [serverNowMs] is the phone clock corrected by
 * Firebase's server offset, because `requestedAt` is a server timestamp.
 */
fun resolveReturnProgress(
    latest: ReturnCommand?,
    tracked: ReturnRequestRef?,
    serverNowMs: Long
): ReturnProgress? {
    if (latest == null || latest.requestedAtMs <= 0L) return null
    val own = tracked?.matches(latest) == true
    val replaced = tracked != null && !own && latest.requestedAtMs > tracked.requestedAtMs
    val ageMs = serverNowMs - latest.requestedAtMs
    val stage = when (latest.status) {
        ReturnStatus.Pending ->
            if (ageMs > RtdbConstants.RETURN_ACCEPT_WINDOW_MS + RtdbConstants.RETURN_STAGE_GRACE_MS) {
                ReturnStage.NotAccepted
            } else {
                ReturnStage.WaitingForAcceptance
            }
        ReturnStatus.Accepted ->
            if (ageMs > RtdbConstants.RETURN_COMPLETION_WINDOW_MS + RtdbConstants.RETURN_STAGE_GRACE_MS) {
                ReturnStage.Overdue
            } else {
                ReturnStage.Returning
            }
        ReturnStatus.Done -> ReturnStage.BackAtBase
        ReturnStatus.Failed -> ReturnStage.Failed
        ReturnStatus.Unknown -> ReturnStage.Unknown
    }
    return ReturnProgress(latest, stage, own, replaced)
}

/** Milliseconds left in the Pi's acceptance window for a pending request; never negative. */
fun acceptanceTimeLeftMs(command: ReturnCommand, serverNowMs: Long): Long =
    (command.requestedAtMs + RtdbConstants.RETURN_ACCEPT_WINDOW_MS - serverNowMs).coerceAtLeast(0L)

/**
 * Operator-facing text for the Pi's failure codes (`mark_reset_failed` in
 * `RasPi/final_w_serial.py`). Unknown codes are shown verbatim rather than guessed at.
 */
fun describeReturnFailure(reason: String?): String = when (reason?.trim()) {
    "operator_emergency_stop" ->
        "Emergency stop is latched on GARBY, so it refused the return. Releasing the stop at " +
            "the Pi monitor leaves GARBY idle where it stands; it will not drive this route afterwards."
    "return_already_in_progress" ->
        "A return was already in progress, so this request was refused. Follow the earlier one."
    "controller_rejected_or_route_incomplete" ->
        "GARBY refused the return or stopped before finishing the route. Treat its position " +
            "as unknown and check it in person; a route fault needs supervised recovery."
    "completion_unverified" ->
        "GARBY accepted the return but did not report arriving within 15 minutes. Check the " +
            "robot in person."
    "stale_or_future_reset_request" ->
        "The Pi saw this request more than 2 minutes after it was sent (or the clocks disagree), " +
            "so it ignored it. Nothing moved."
    "missing_reset_timestamp" ->
        "The request reached the Pi without a server timestamp, so it was ignored. Nothing moved."
    null, "" -> "GARBY reported that the return failed."
    else -> "GARBY reported that the return failed ($reason)."
}
