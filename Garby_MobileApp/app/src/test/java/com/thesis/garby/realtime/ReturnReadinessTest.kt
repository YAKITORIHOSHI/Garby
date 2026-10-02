package com.thesis.garby.realtime

import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ReturnReadinessTest {
    private val now = 1_000_000L
    private val live = DeviceStatus(true, now, null, null, bleConnected = true, lidarHealthy = true)

    @Test fun liveDestinationCanReturn() {
        assertTrue(canRequestReturn(true, live, true, now))
    }
    @Test fun missingOrStaleRobotCannotReturn() {
        assertFalse(canRequestReturn(true, null, true, now))
        assertFalse(canRequestReturn(true, live.copy(lastSeenMs = now - 90_001), true, now))
    }
    @Test fun disconnectedOrUnknownLinksCannotReturn() {
        assertFalse(canRequestReturn(true, live, false, now))
        assertFalse(canRequestReturn(true, live.copy(bleConnected = null), true, now))
        assertFalse(canRequestReturn(true, live.copy(lidarHealthy = false), true, now))
    }
    @Test fun incompleteOutboundCannotReturn() {
        assertFalse(canRequestReturn(false, live, true, now))
    }
}
