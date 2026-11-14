package com.thesis.garby.realtime

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ReturnReadinessTest {
    private val now = 1_000_000L
    private val atDestination = DeviceStatus(
        lastSeenMs = now,
        robotState = RobotState.AtDestination,
        isReadyToReturn = true,
        bleConnected = true,
        lidarHealthy = true
    )

    private fun readiness(
        status: DeviceStatus? = atDestination,
        cloud: Boolean = true,
        internet: Boolean = true,
        inFlight: Boolean = false
    ) = evaluateReturnReadiness(status, cloud, internet, inFlight, now)

    @Test
    fun liveDestinationCanReturn() {
        val result = readiness()
        assertTrue(result.canSend)
        assertNull(result.firstUnmet)
    }

    @Test
    fun missingOrStaleRobotCannotReturn() {
        assertEquals(ReturnCheck.AtDestination, readiness(status = null).firstUnmet)
        val stale = readiness(status = atDestination.copy(lastSeenMs = now - RtdbConstants.STALE_AFTER_MS - 1))
        assertFalse(stale.canSend)
        assertEquals(ReturnCheck.RobotOnline, stale.firstUnmet)
    }

    @Test
    fun disconnectedOrUnknownLinksCannotReturn() {
        assertEquals(ReturnCheck.Cloud, readiness(cloud = false).firstUnmet)
        assertEquals(ReturnCheck.Internet, readiness(internet = false).firstUnmet)
        assertEquals(ReturnCheck.BleLink, readiness(status = atDestination.copy(bleConnected = null)).firstUnmet)
        assertEquals(ReturnCheck.Lidar, readiness(status = atDestination.copy(lidarHealthy = false)).firstUnmet)
    }

    @Test
    fun incompleteOutboundCannotReturn() {
        // A request reaching the executor mid-outbound latches a route fault.
        val outbound = atDestination.copy(robotState = RobotState.Outbound, isReadyToReturn = false)
        assertEquals(ReturnCheck.AtDestination, readiness(status = outbound).firstUnmet)
    }

    @Test
    fun readyFlagMustAgreeWithPublishedState() {
        val contradictory = atDestination.copy(robotState = RobotState.Returning, isReadyToReturn = true)
        assertFalse(readiness(status = contradictory).canSend)
        val flagCleared = atDestination.copy(isReadyToReturn = false)
        assertFalse(readiness(status = flagCleared).canSend)
    }

    @Test
    fun olderPiWithoutStateFieldStillUsesItsReadyFlag() {
        assertTrue(readiness(status = atDestination.copy(robotState = null)).canSend)
        assertFalse(readiness(status = atDestination.copy(robotState = null, isReadyToReturn = null)).canSend)
    }

    @Test
    fun requestInFlightBlocksAnother() {
        assertEquals(ReturnCheck.NoReturnInFlight, readiness(inFlight = true).firstUnmet)
    }

    @Test
    fun blockedPlaceholderNeverSends() {
        assertFalse(ReturnReadiness.Blocked.canSend)
        assertFalse(ReturnReadiness(emptyList()).canSend)
    }
}
