package com.thesis.garby.realtime

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ReturnCommandTest {
    private val requestedAt = 1_000_000L
    private val mine = ReturnRequestRef(requestedAt, "operator-1")

    private fun command(status: ReturnStatus, at: Long = requestedAt, by: String = "operator-1") =
        ReturnCommand(at, by, status)

    @Test
    fun knownStatusesParseCaseInsensitively() {
        assertEquals(ReturnStatus.Pending, ReturnStatus.fromWire("pending"))
        assertEquals(ReturnStatus.Accepted, ReturnStatus.fromWire("ACK"))
        assertEquals(ReturnStatus.Done, ReturnStatus.fromWire(" done "))
        assertEquals(ReturnStatus.Failed, ReturnStatus.fromWire("failed"))
    }

    @Test
    fun malformedStatusDoesNotBecomePending() {
        assertEquals(ReturnStatus.Unknown, ReturnStatus.fromWire(null))
        assertEquals(ReturnStatus.Unknown, ReturnStatus.fromWire("surprise"))
    }

    @Test
    fun requestIdentityNeedsTheExactTimestampAndOperator() {
        assertTrue(mine.matches(command(ReturnStatus.Done)))
        assertFalse(mine.matches(command(ReturnStatus.Done, at = requestedAt + 1)))
        assertFalse(mine.matches(command(ReturnStatus.Done, by = "operator-2")))
        assertFalse(mine.matches(null))
        assertFalse(ReturnRequestRef(0L, "operator-1").matches(command(ReturnStatus.Done, at = 0L)))
    }

    @Test
    fun pendingRequestWaitsThenExpiresAfterThePiAcceptanceWindow() {
        val pending = command(ReturnStatus.Pending)
        val early = resolveReturnProgress(pending, mine, requestedAt + 30_000L)!!
        assertEquals(ReturnStage.WaitingForAcceptance, early.stage)
        assertTrue(early.inFlight)
        assertTrue(early.ownRequest)

        val boundary = requestedAt + RtdbConstants.RETURN_ACCEPT_WINDOW_MS + RtdbConstants.RETURN_STAGE_GRACE_MS
        assertEquals(ReturnStage.WaitingForAcceptance, resolveReturnProgress(pending, mine, boundary)!!.stage)
        val expired = resolveReturnProgress(pending, mine, boundary + 1)!!
        assertEquals(ReturnStage.NotAccepted, expired.stage)
        assertFalse(expired.inFlight)
    }

    @Test
    fun acceptedReturnBecomesOverdueOnlyAfterTheCompletionWindow() {
        val accepted = command(ReturnStatus.Accepted)
        val driving = resolveReturnProgress(accepted, mine, requestedAt + 10 * 60_000L)!!
        assertEquals(ReturnStage.Returning, driving.stage)
        assertTrue(driving.inFlight)
        val late = requestedAt + RtdbConstants.RETURN_COMPLETION_WINDOW_MS + RtdbConstants.RETURN_STAGE_GRACE_MS + 1
        val overdue = resolveReturnProgress(accepted, mine, late)!!
        assertEquals(ReturnStage.Overdue, overdue.stage)
        assertFalse(overdue.inFlight)
    }

    @Test
    fun terminalOutcomesAreReportedAsRecorded() {
        assertEquals(ReturnStage.BackAtBase, resolveReturnProgress(command(ReturnStatus.Done), mine, requestedAt)!!.stage)
        assertEquals(ReturnStage.Failed, resolveReturnProgress(command(ReturnStatus.Failed), mine, requestedAt)!!.stage)
        assertEquals(ReturnStage.Unknown, resolveReturnProgress(command(ReturnStatus.Unknown), mine, requestedAt)!!.stage)
    }

    @Test
    fun newerRequestFromAnyoneReplacesTheTrackedOne() {
        val newer = command(ReturnStatus.Pending, at = requestedAt + 5_000L, by = "operator-2")
        val progress = resolveReturnProgress(newer, mine, requestedAt + 6_000L)!!
        assertFalse(progress.ownRequest)
        assertTrue(progress.ownRequestReplaced)

        val sameOperatorNewer = command(ReturnStatus.Done, at = requestedAt + 5_000L)
        val second = resolveReturnProgress(sameOperatorNewer, mine, requestedAt + 6_000L)!!
        assertFalse(second.ownRequest)
        assertTrue(second.ownRequestReplaced)
    }

    @Test
    fun noRecordMeansNoProgress() {
        assertNull(resolveReturnProgress(null, mine, requestedAt))
        assertNull(resolveReturnProgress(command(ReturnStatus.Pending, at = 0L), null, requestedAt))
    }

    @Test
    fun acceptanceCountdownNeverGoesNegative() {
        val pending = command(ReturnStatus.Pending)
        assertEquals(RtdbConstants.RETURN_ACCEPT_WINDOW_MS, acceptanceTimeLeftMs(pending, requestedAt))
        assertEquals(0L, acceptanceTimeLeftMs(pending, requestedAt + 10 * 60_000L))
    }

    @Test
    fun everyPiFailureCodeHasOperatorText() {
        listOf(
            "operator_emergency_stop",
            "return_already_in_progress",
            "controller_rejected_or_route_incomplete",
            "completion_unverified",
            "stale_or_future_reset_request",
            "missing_reset_timestamp"
        ).forEach { code ->
            val text = describeReturnFailure(code)
            assertFalse("$code fell through to the generic text", text.contains(code))
        }
        assertTrue(describeReturnFailure("new_reason").contains("new_reason"))
        assertEquals("GARBY reported that the return failed.", describeReturnFailure(null))
    }
}
