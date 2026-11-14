package com.thesis.garby.ui

import com.thesis.garby.realtime.ConnectionState
import com.thesis.garby.realtime.DeviceUiState
import com.thesis.garby.realtime.HealthLevel
import com.thesis.garby.realtime.SensorReading
import com.thesis.garby.realtime.SensorReadingUiState
import com.thesis.garby.realtime.Severity
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNull
import org.junit.Test

class DashboardIndicatorsTest {
    private val now = 1_800_000_000_000L

    private fun reading(value: Float, ageMs: Long = 0L, stale: Boolean = false) = SensorReadingUiState(
        reading = SensorReading(value, "", "", now - ageMs),
        isLoading = false,
        isStale = stale
    )

    private fun liveState() = DeviceUiState(
        connection = ConnectionState.Connected,
        weight = reading(2f),
        level = reading(30f),
        mq135 = reading(100f),
        mq137 = reading(100f),
        mq4 = reading(100f),
        nowMs = now
    )

    @Test
    fun completeFreshReadingsAreRequiredForAPositiveSummary() {
        val state = liveState()
        assertEquals(Severity.Normal, binSectionIndicator(state).severity)
        assertEquals(Severity.Normal, airSectionIndicator(state).severity)
        assertNotEquals(Severity.Normal, binSectionIndicator(state.copy(level = reading(999f))).severity)
        assertNotEquals(Severity.Normal, airSectionIndicator(state.copy(mq4 = reading(-1f))).severity)
    }

    @Test
    fun staleDangerIsNeverPresentedAsACurrentWarning() {
        val state = liveState().copy(mq4 = reading(900f, stale = true))
        val indicator = airSectionIndicator(state)
        assertEquals("2/3 live", indicator.label)
        assertEquals(Severity.Elevated, indicator.severity)
    }

    @Test
    fun freshDangerRemainsVisibleWhenAnotherSensorIsMissing() {
        val state = liveState().copy(mq4 = reading(700f), mq137 = reading(-1f))
        assertEquals(SectionIndicator("Danger band", Severity.Danger), airSectionIndicator(state))
    }

    @Test
    fun summariesRespectEachSensorsDifferentBands() {
        val state = liveState().copy(mq135 = reading(499f), mq137 = reading(399f))
        assertEquals(Severity.Elevated, airSectionIndicator(state).severity)
        assertEquals(Severity.High, airSectionIndicator(state.copy(mq135 = reading(500f))).severity)
    }

    @Test
    fun dispatchWeightIsDistinctFromAnInvalidReading() {
        val indicator = binSectionIndicator(liveState().copy(weight = reading(4f)))
        assertEquals("Dispatch weight", indicator.label)
        assertEquals(Severity.Elevated, indicator.severity)
        assertEquals("Check weight", binSectionIndicator(liveState().copy(weight = reading(20.01f))).label)
    }

    @Test
    fun offlineAndDisconnectedSnapshotsCannotLookLive() {
        val state = liveState()
        for (offline in listOf(
            state.copy(internetAvailable = false),
            state.copy(connection = ConnectionState.Disconnected)
        )) {
            assertEquals("Stale", binSectionIndicator(offline).label)
            assertNull(airSectionIndicator(offline).severity)
        }
    }

    @Test
    fun timestampExpiryIsCheckedEvenBeforeTheNextFreshnessUpdate() {
        val state = liveState().copy(
            weight = reading(4f, ageMs = 91_000L),
            level = reading(30f, ageMs = -31_000L)
        )
        assertEquals(SectionIndicatorKind.Stale, binSectionIndicator(state).kind)
    }

    @Test
    fun unavailableAndNonFiniteValuesNeverCountAsLive() {
        val state = liveState().copy(weight = reading(Float.NaN), level = reading(999f))
        assertEquals(SectionIndicatorKind.Unavailable, binSectionIndicator(state).kind)
        val air = state.copy(mq135 = reading(Float.POSITIVE_INFINITY), mq137 = reading(-1f), mq4 = reading(-1f))
        assertEquals(SectionIndicatorKind.Unavailable, airSectionIndicator(air).kind)
    }

    @Test
    fun loadingIsDistinctFromUnavailable() {
        assertEquals(SectionIndicatorKind.Loading, airSectionIndicator(DeviceUiState(nowMs = now)).kind)
        val missing = SensorReadingUiState(isLoading = false)
        assertEquals(SectionIndicatorKind.Unavailable, binSectionIndicator(liveState().copy(weight = missing, level = missing)).kind)
    }

    @Test
    fun systemIssuesTakePriorityAndUnknownHealthCannotProduceAnAllClear() {
        assertEquals(
            SectionIndicator("1 issue", Severity.Danger),
            systemSectionIndicator(listOf(HealthLevel.Fault, HealthLevel.Caution, HealthLevel.Unknown))
        )
        assertEquals(
            SectionIndicator("1 to check", Severity.Elevated),
            systemSectionIndicator(listOf(HealthLevel.Caution, HealthLevel.Unknown))
        )
        assertEquals(SectionIndicator("1/2 healthy"), systemSectionIndicator(listOf(HealthLevel.Ok, HealthLevel.Unknown)))
        assertEquals(
            SectionIndicator("2/2 healthy", Severity.Normal),
            systemSectionIndicator(listOf(HealthLevel.Ok, HealthLevel.Ok))
        )
    }
}
