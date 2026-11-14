package com.thesis.garby.realtime

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test

class DisposalLogicTest {
    private val now = 1_000_000L
    private val idle = DeviceStatus(lastSeenMs = now, robotState = RobotState.Idle,
        bleConnected = true, manualDisposalReady = true, manualDisposalUpdatedAtMs = now)
    private val loaded = DisposalUiState(loaded = true)
    private fun block(status: DeviceStatus? = idle, cloud: Boolean = true, internet: Boolean = true,
        state: DisposalUiState = loaded) = disposalBlockReason(status, cloud, internet, state, now)

    @Test fun idleCanDisposeWithoutAnyBinGasWeightOrLidarReading() {
        assertNull(idle.sensorSerialConnected)
        assertNull(idle.lidarHealthy)
        assertNull(block())
    }

    @Test fun missingStaleOrFutureEligibilityFailsClosed() {
        for (status in listOf(null, idle.copy(manualDisposalReady = null),
            idle.copy(manualDisposalReady = false), idle.copy(manualDisposalUpdatedAtMs = 0L),
            idle.copy(manualDisposalUpdatedAtMs = now - 15_001L),
            idle.copy(manualDisposalUpdatedAtMs = now + 5_001L),
            idle.copy(lastSeenMs = now - 90_001L), idle.copy(bleConnected = null))) {
            assertNotNull(block(status = status))
        }
        assertNull(block(status = idle.copy(manualDisposalUpdatedAtMs = now - 15_000L)))
    }

    @Test fun offlineUnknownStateOrBusyCannotSubmit() {
        assertNotNull(block(cloud = false))
        assertNotNull(block(internet = false))
        for (robot in RobotState.entries.filter { it != RobotState.Idle }) {
            assertNotNull(block(status = idle.copy(robotState = robot)))
        }
        assertNotNull(block(status = idle.copy(robotState = null)))
        assertNotNull(block(state = loaded.copy(submitting = true)))
        assertNotNull(block(state = loaded.copy(loaded = false)))
        assertNotNull(block(state = loaded.copy(error = "permission denied")))
    }

    @Test fun unresolvedRequestBlocksRepeatedTapsEvenAfterItsDeadline() {
        for (status in listOf(DisposalStatus.Pending, DisposalStatus.Dispatching, DisposalStatus.Unknown)) {
            val command = DisposalCommand("a".repeat(32), now - 500_000L, "operator", status)
            assertNotNull(block(state = loaded.copy(command = command)))
            assertFalse(disposalMessage(command, now).orEmpty().contains("Accepted"))
        }
    }

    @Test fun cloudWriteAndControllerAcceptanceAreDifferentStates() {
        val command = DisposalCommand("a".repeat(32), now, "operator", DisposalStatus.Pending)
        assertTrue(disposalMessage(command, now).orEmpty().contains("Waiting for GARBY"))
        assertTrue(disposalMessage(command.copy(status = DisposalStatus.Accepted), now)
            .orEmpty().contains("Follow the robot status for arrival"))
        assertEquals(DisposalStatus.Unknown, DisposalStatus.fromWire("done"))
    }

    @Test fun transactionCannotReplaceAConcurrentOrUnresolvedRequest() {
        assertTrue(canReplaceDisposal(null, null, null, false))
        for (status in listOf("accepted", "failed", "unconfirmed")) {
            assertTrue(canReplaceDisposal("old", "old", status, true))
            assertFalse(canReplaceDisposal("old", "new", status, true))
        }
        for (status in listOf(null, "pending", "dispatching", "unknown")) {
            assertFalse(canReplaceDisposal("old", "old", status, true))
        }
    }

    @Test fun matchingLateReportClearsTimeoutNoticeForEveryKnownServerStage() {
        val timeout = DisposalSubmission.Unconfirmed("Delivery not confirmed", "request-a")
        val waiting = loaded.withSubmission(timeout)
        assertNotNull(waiting.notice)
        for (stage in DisposalStatus.entries.filter { it != DisposalStatus.Unknown }) {
            val updated = waiting.withCommand(DisposalCommand("request-a", now, "operator", stage))
            assertNull(updated.notice)
            assertNull(updated.noticeRequestId)
            assertEquals(stage, updated.command?.status)
        }
    }

    @Test fun anotherRequestOrAnUnknownStatusCannotResolveThisPhonesTimeout() {
        val waiting = loaded.withSubmission(DisposalSubmission.Unconfirmed("Check GARBY", "request-a"))
        for (command in listOf(null,
            DisposalCommand("request-b", now, "another-phone", DisposalStatus.Accepted),
            DisposalCommand("request-a", now, "operator", DisposalStatus.Unknown))) {
            assertEquals("Check GARBY", waiting.withCommand(command).notice)
        }
    }

    @Test fun timeoutAfterListenerReportCannotHideAnAlreadyKnownOutcome() {
        for (stage in listOf(DisposalStatus.Pending, DisposalStatus.Accepted, DisposalStatus.Failed)) {
            val reported = loaded.withCommand(DisposalCommand("request-a", now, "operator", stage))
            val updated = reported.withSubmission(DisposalSubmission.Unconfirmed("Check GARBY", "request-a"))
            assertNull(updated.notice)
            assertEquals(stage, updated.command?.status)
        }
    }

    @Test fun submissionReceiptCannotDowngradeListenerOutcomeOrReplaceNewerRequest() {
        val pending = DisposalCommand("request-a", now, "operator", DisposalStatus.Pending)
        val accepted = pending.copy(status = DisposalStatus.Accepted)
        assertEquals(accepted, loaded.withCommand(accepted)
            .withSubmission(DisposalSubmission.Sent(pending)).command)
        val newer = pending.copy(requestId = "request-b", requestedAtMs = now + 1L)
        assertEquals(newer, loaded.withCommand(newer)
            .withSubmission(DisposalSubmission.Sent(pending)).command)
        assertEquals(pending, loaded.withSubmission(DisposalSubmission.Sent(pending)).command)
    }

    @Test fun aNotSentNoticeIsNotClearedByAnUnrelatedCloudUpdate() {
        val notSent = loaded.withSubmission(DisposalSubmission.NotSent("Nothing was sent"))
        assertEquals("Nothing was sent", notSent.withCommand(
            DisposalCommand("other", now, "operator", DisposalStatus.Accepted)).notice)
    }
}
