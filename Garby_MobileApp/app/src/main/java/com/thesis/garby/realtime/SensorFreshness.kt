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
