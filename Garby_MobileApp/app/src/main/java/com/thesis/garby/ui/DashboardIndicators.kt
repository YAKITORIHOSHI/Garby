package com.thesis.garby.ui

import com.thesis.garby.realtime.DeviceUiState
import com.thesis.garby.realtime.GasSensor
import com.thesis.garby.realtime.HealthLevel
import com.thesis.garby.realtime.LoadLevel
import com.thesis.garby.realtime.SensorReadingUiState
import com.thesis.garby.realtime.Severity
import com.thesis.garby.realtime.gasBand
import com.thesis.garby.realtime.isFreshTimestamp
import com.thesis.garby.realtime.loadLevel

/** A display summary only. It never participates in dispatch or return readiness. */
data class SectionIndicator(
    val label: String,
    val severity: Severity? = null,
    val kind: SectionIndicatorKind = SectionIndicatorKind.Summary
)

/** Keeps the status symbol independent of its displayed wording. */
enum class SectionIndicatorKind { Summary, Loading, Stale, Unavailable }

fun binSectionIndicator(state: DeviceUiState): SectionIndicator {
    val readings = listOf(state.weight, state.level)
    val values = listOf(
        state.weight.liveValue(state) { it >= 0f },
        state.level.liveValue(state) { it >= 0f && it < 999f }
    )
    val attention = when (values[0]?.let(::loadLevel)) {
        LoadLevel.OutOfRange -> SectionIndicator("Check weight", Severity.High)
        LoadLevel.AtTrigger -> SectionIndicator("Dispatch weight", Severity.Elevated)
        LoadLevel.Filling -> SectionIndicator("Filling up", Severity.Elevated)
        else -> null
    }
    return attention ?: readingAvailability(readings, values, state)
}

fun airSectionIndicator(state: DeviceUiState): SectionIndicator {
    val readings = listOf(state.mq135, state.mq137, state.mq4)
    val sensors = listOf(GasSensor.Mq135, GasSensor.Mq137, GasSensor.Mq4)
    val values = readings.map { it.liveValue(state) { value -> value >= 0f } }
    val severity = sensors.zip(values).mapNotNull { (sensor, value) ->
        value?.let { gasBand(sensor, it)?.severity }
    }.maxOrNull()
    // A fresh warning remains visible even if another sensor has stopped reporting.
    return when (severity) {
        Severity.Danger -> SectionIndicator("Danger band", severity)
        Severity.High -> SectionIndicator("High readings", severity)
        Severity.Elevated -> SectionIndicator("Elevated", severity)
        else -> readingAvailability(readings, values, state)
    }
}

/** Uses the very same levels shown on the system tiles, including unknown Pi reports. */
fun systemSectionIndicator(levels: List<HealthLevel>): SectionIndicator {
    if (levels.isEmpty()) return SectionIndicator("No data", kind = SectionIndicatorKind.Unavailable)
    val faults = levels.count { it == HealthLevel.Fault }
    val cautions = levels.count { it == HealthLevel.Caution }
    val healthy = levels.count { it == HealthLevel.Ok }
    return when {
        faults > 0 -> SectionIndicator("$faults ${if (faults == 1) "issue" else "issues"}", Severity.Danger)
        cautions > 0 -> SectionIndicator("$cautions to check", Severity.Elevated)
        healthy == levels.size -> SectionIndicator("$healthy/${levels.size} healthy", Severity.Normal)
        else -> SectionIndicator("$healthy/${levels.size} healthy")
    }
}

private fun SensorReadingUiState.liveValue(state: DeviceUiState, valid: (Float) -> Boolean): Float? {
    val sample = reading ?: return null
    return sample.value.takeIf {
        !isLoading && error == null && !isStale && state.cloudConnected && state.internetAvailable &&
            isFreshTimestamp(sample.updatedAtMs, state.nowMs) && it.isFinite() && valid(it)
    }
}

private fun readingAvailability(
    readings: List<SensorReadingUiState>,
    liveValues: List<Float?>,
    state: DeviceUiState
): SectionIndicator {
    val live = liveValues.count { it != null }
    return when {
        live == readings.size -> SectionIndicator("$live/${readings.size} live", Severity.Normal)
        live > 0 -> SectionIndicator("$live/${readings.size} live", Severity.Elevated)
        readings.all { it.isLoading } -> SectionIndicator("Loading", kind = SectionIndicatorKind.Loading)
        readings.any { sample ->
            sample.reading != null && (sample.isStale || !state.cloudConnected ||
                !state.internetAvailable || !isFreshTimestamp(sample.reading.updatedAtMs, state.nowMs))
        } -> SectionIndicator("Stale", kind = SectionIndicatorKind.Stale)
        else -> SectionIndicator("No data", kind = SectionIndicatorKind.Unavailable)
    }
}
