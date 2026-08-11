package com.thesis.garby.realtime

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ResetStatusTest {
    @Test
    fun knownStatusesParseCaseInsensitively() {
        assertEquals(ResetStatus.Pending, ResetStatus.fromString("pending"))
        assertEquals(ResetStatus.Ack, ResetStatus.fromString("ACK"))
        assertEquals(ResetStatus.Done, ResetStatus.fromString(" done "))
        assertEquals(ResetStatus.Failed, ResetStatus.fromString("failed"))
    }

    @Test
    fun malformedStatusDoesNotBecomePending() {
        assertEquals(ResetStatus.Unknown, ResetStatus.fromString(null))
        assertEquals(ResetStatus.Unknown, ResetStatus.fromString("surprise"))
    }

    @Test
    fun requestMarkerRejectsOldCommand() {
        val marker = ResetRequestMarker("operator-1", 10_000L)
        assertFalse(marker.matches(ResetCommand(10_000L, "operator-1", ResetStatus.Done)))
        assertFalse(marker.matches(ResetCommand(10_001L, "operator-2", ResetStatus.Done)))
        assertTrue(marker.matches(ResetCommand(10_001L, "operator-1", ResetStatus.Done)))
    }

    @Test
    fun compatibilityFlagCannotCompleteStructuredCommand() {
        assertEquals(
            ResetStatus.Pending,
            resolveResetStatus("pending", appReadyToReset = false, structuredResetExists = true)
        )
        assertEquals(
            ResetStatus.Ack,
            resolveResetStatus("ack", appReadyToReset = false, structuredResetExists = true)
        )
    }

    @Test
    fun compatibilityFlagOnlySuppliesLegacyPendingState() {
        assertEquals(
            ResetStatus.Pending,
            resolveResetStatus(null, appReadyToReset = true, structuredResetExists = false)
        )
        assertEquals(
            ResetStatus.Unknown,
            resolveResetStatus(null, appReadyToReset = false, structuredResetExists = false)
        )
    }
}
