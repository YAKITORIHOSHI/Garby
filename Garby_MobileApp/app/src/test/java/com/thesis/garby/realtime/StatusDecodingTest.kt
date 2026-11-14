package com.thesis.garby.realtime

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class StatusDecodingTest {
    private val now = 1_000_000L

    @Test
    fun robotStatesParseFromThePiWireValues() {
        assertEquals(RobotState.Idle, RobotState.fromWire("IDLE"))
        assertEquals(RobotState.Outbound, RobotState.fromWire("OUTBOUND"))
        assertEquals(RobotState.AtDestination, RobotState.fromWire("at_destination"))
        assertEquals(RobotState.Returning, RobotState.fromWire(" RETURNING "))
        assertEquals(RobotState.Fault, RobotState.fromWire("FAULT"))
        assertEquals(RobotState.Unknown, RobotState.fromWire("UNKNOWN"))
        assertEquals(RobotState.Unknown, RobotState.fromWire("SOMETHING_NEW"))
        assertNull(RobotState.fromWire(null))
        assertNull(RobotState.fromWire("  "))
    }

    @Test
    fun presenceFollowsTheHeartbeat() {
        assertEquals(RobotPresence.NeverSeen, robotPresence(null, now))
        assertEquals(RobotPresence.NeverSeen, robotPresence(DeviceStatus(lastSeenMs = 0L), now))
        assertEquals(RobotPresence.Live, robotPresence(DeviceStatus(lastSeenMs = now - 30_000L), now))
        assertEquals(
            RobotPresence.Stale,
            robotPresence(DeviceStatus(lastSeenMs = now - RtdbConstants.STALE_AFTER_MS - 1), now)
        )
        assertEquals(
            RobotPresence.Stale,
            robotPresence(DeviceStatus(lastSeenMs = now + RtdbConstants.FUTURE_TIMESTAMP_TOLERANCE_MS + 1), now)
        )
    }

    @Test
    fun throttleFlagsDecodeLikeBridgeCore() {
        assertTrue(decodeThrottleFlags(null).isEmpty())
        assertTrue(decodeThrottleFlags(0L).isEmpty())
        assertEquals(
            listOf(ThrottleCondition.UndervoltageNow, ThrottleCondition.UndervoltageSinceBoot),
            decodeThrottleFlags(0x10001L)
        )
        assertEquals(
            listOf(ThrottleCondition.ThrottledSinceBoot, ThrottleCondition.SoftTemperatureLimitSinceBoot),
            decodeThrottleFlags(0xC0000L)
        )
    }

    @Test
    fun powerAndTemperatureLevels() {
        assertEquals(HealthLevel.Unknown, piPowerLevel(null))
        assertEquals(HealthLevel.Ok, piPowerLevel(0L))
        assertEquals(HealthLevel.Fault, piPowerLevel(0x1L))
        assertEquals(HealthLevel.Caution, piPowerLevel(0x50000L))
        assertEquals(HealthLevel.Caution, piPowerLevel(0x100L))

        assertEquals(HealthLevel.Unknown, piTemperatureLevel(null))
        assertEquals(HealthLevel.Ok, piTemperatureLevel(69.9f))
        assertEquals(HealthLevel.Caution, piTemperatureLevel(70f))
        assertEquals(HealthLevel.Fault, piTemperatureLevel(PI_THERMAL_WARNING_C))
    }

    @Test
    fun linksDistinguishDownFromNotReported() {
        assertEquals(HealthLevel.Ok, linkLevel(true))
        assertEquals(HealthLevel.Fault, linkLevel(false))
        assertEquals(HealthLevel.Unknown, linkLevel(null))
    }
}
