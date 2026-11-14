package com.thesis.garby.realtime

/**
 * Bits of the Raspberry Pi firmware `get_throttled` field, as named by
 * `describe_throttle_flags` in `RasPi/bridge_core.py`.
 */
enum class ThrottleCondition(val bit: Long, val label: String, val activeNow: Boolean) {
    UndervoltageNow(0x1L, "Undervoltage now", true),
    FrequencyCappedNow(0x2L, "CPU frequency capped now", true),
    ThrottledNow(0x4L, "Throttled now", true),
    SoftTemperatureLimitNow(0x8L, "Soft temperature limit now", true),
    UndervoltageSinceBoot(0x10000L, "Undervoltage since boot", false),
    FrequencyCappedSinceBoot(0x20000L, "Frequency capped since boot", false),
    ThrottledSinceBoot(0x40000L, "Throttled since boot", false),
    SoftTemperatureLimitSinceBoot(0x80000L, "Soft temperature limit since boot", false)
}

fun decodeThrottleFlags(flags: Long?): List<ThrottleCondition> =
    if (flags == null) emptyList() else ThrottleCondition.entries.filter { flags and it.bit != 0L }

enum class HealthLevel { Ok, Caution, Fault, Unknown }

/** `bridge_core.build_health_status` raises the Pi's thermal warning at 80 °C. */
const val PI_THERMAL_WARNING_C = 80f

/** Presentation only: a warm Pi is shown amber before the Pi's own warning. */
const val PI_WARM_C = 70f

fun piTemperatureLevel(temperatureC: Float?): HealthLevel = when {
    temperatureC == null || !temperatureC.isFinite() -> HealthLevel.Unknown
    temperatureC >= PI_THERMAL_WARNING_C -> HealthLevel.Fault
    temperatureC >= PI_WARM_C -> HealthLevel.Caution
    else -> HealthLevel.Ok
}

/**
 * Anything happening now is a fault. Something that only happened earlier since boot, or a bit
 * without a name, is a caution.
 */
fun piPowerLevel(flags: Long?): HealthLevel = when {
    flags == null -> HealthLevel.Unknown
    flags == 0L -> HealthLevel.Ok
    decodeThrottleFlags(flags).any { it.activeNow } -> HealthLevel.Fault
    else -> HealthLevel.Caution
}

fun linkLevel(connected: Boolean?): HealthLevel = when (connected) {
    true -> HealthLevel.Ok
    false -> HealthLevel.Fault
    null -> HealthLevel.Unknown
}
