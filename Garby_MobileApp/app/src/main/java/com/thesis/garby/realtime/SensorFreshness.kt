package com.thesis.garby.realtime

/** Pure timestamp check so cached Firebase values cannot silently look live forever. */
fun isFreshTimestamp(
    updatedAtMs: Long,
    nowMs: Long = System.currentTimeMillis(),
    staleAfterMs: Long = RtdbConstants.STALE_AFTER_MS,
    futureToleranceMs: Long = RtdbConstants.FUTURE_TIMESTAMP_TOLERANCE_MS
): Boolean {
    if (updatedAtMs <= 0L || staleAfterMs < 0L || futureToleranceMs < 0L) return false
    val ageMs = nowMs - updatedAtMs
    return ageMs in -futureToleranceMs..staleAfterMs
}

/** Whether the Pi is reporting, judged from its 30-second heartbeat at display time. */
enum class RobotPresence {
    /** Heartbeat within [RtdbConstants.STALE_AFTER_MS]. */
    Live,

    /** A heartbeat exists but is too old (or too far in the future) to trust. */
    Stale,

    /** No status record, or one without a heartbeat. */
    NeverSeen
}

fun robotPresence(status: DeviceStatus?, nowMs: Long): RobotPresence = when {
    status == null || status.lastSeenMs <= 0L -> RobotPresence.NeverSeen
    isFreshTimestamp(status.lastSeenMs, nowMs = nowMs) -> RobotPresence.Live
    else -> RobotPresence.Stale
}
