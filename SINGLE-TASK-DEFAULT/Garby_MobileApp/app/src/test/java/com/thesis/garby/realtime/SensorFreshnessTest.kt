package com.thesis.garby.realtime

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class SensorFreshnessTest {
    private val now = 1_000_000L

    @Test
    fun missingTimestampIsStale() {
        assertFalse(isFreshTimestamp(0L, nowMs = now, staleAfterMs = 60_000L))
    }

    @Test
    fun currentAndBoundaryTimestampAreFresh() {
        assertTrue(isFreshTimestamp(now, nowMs = now, staleAfterMs = 60_000L))
        assertTrue(isFreshTimestamp(now - 60_000L, nowMs = now, staleAfterMs = 60_000L))
    }

    @Test
    fun oldTimestampIsStale() {
        assertFalse(isFreshTimestamp(now - 60_001L, nowMs = now, staleAfterMs = 60_000L))
    }

    @Test
    fun excessiveFutureTimestampIsStale() {
        assertFalse(
            isFreshTimestamp(
                updatedAtMs = now + 30_001L,
                nowMs = now,
                staleAfterMs = 60_000L,
                futureToleranceMs = 30_000L
            )
        )
    }
}
