package com.thesis.garby.realtime

/** Values are already converted to engineering units by the Raspberry Pi. */
data class SensorReading(
    val value: Float,
    val unit: String,
    val sensorType: String,
    val updatedAtMs: Long
)

data class HistoryPoint(
    val value: Float,
    val atMs: Long
)

data class DeviceStatus(
    val online: Boolean,
    val lastSeenMs: Long,
    val batteryPercent: Int?,
    val wifiRssi: Int?,
    val recentUptimeStr: String? = null,
    val cpuTemperatureC: Float? = null,
    val thermalWarning: Boolean? = null,
    val throttledFlags: Long? = null,
    val bleConnected: Boolean? = null,
    val lidarHealthy: Boolean? = null,
    val sensorSerialConnected: Boolean? = null
)

enum class ResetStatus {
    Pending,
    Ack,
    Done,
    Failed,
    Unknown;

    companion object {
        fun fromString(value: String?): ResetStatus = when (value?.trim()?.lowercase()) {
            "pending" -> Pending
            "ack" -> Ack
            "done" -> Done
            "failed" -> Failed
            else -> Unknown
        }
    }
}

data class ResetCommand(
    val requestedAtMs: Long,
    val requestedBy: String,
    val status: ResetStatus
)

/** Marker used to reject stale terminal states from a previous reset request. */
data class ResetRequestMarker(
    val requestedBy: String,
    val previousRequestedAtMs: Long
) {
    fun matches(command: ResetCommand): Boolean =
        command.requestedBy == requestedBy &&
            command.requestedAtMs > previousRequestedAtMs
}

/**
 * Resolves the deployed compatibility flag without letting it override the
 * authoritative structured command. In particular, APP=false is an idle
 * signal, not proof that a new structured pending/ack command completed.
 */
internal fun resolveResetStatus(
    structuredStatus: String?,
    appReadyToReset: Boolean?,
    structuredResetExists: Boolean
): ResetStatus {
    val structured = ResetStatus.fromString(structuredStatus)
    if (structuredResetExists) return structured
    return if (appReadyToReset == true) ResetStatus.Pending else ResetStatus.Unknown
}

sealed interface SensorUiState<out T> {
    object Loading : SensorUiState<Nothing>
    data class Value<T>(val data: T) : SensorUiState<T>
    data class Error(val message: String) : SensorUiState<Nothing>
}

sealed interface ConnectionState {
    object Connected : ConnectionState
    object Reconnecting : ConnectionState
    object Disconnected : ConnectionState
}

enum class SensorKey(
    val storageKey: String,
    val displayName: String,
    val unit: String,
    /** Exact legacy node in the deployed GARBY RTDB export. */
    val legacyPath: String,
    /** Exact value field inside [legacyPath]. */
    val legacyValueKey: String
) {
    Level(
        "level",
        "Ultrasonic Distance",
        "cm",
        "RASPI/VALUES/ULTRASONIC_SENSOR",
        "CM_DISTANCE"
    ),
    Weight(
        "weight",
        "Current Weight",
        "kg",
        "RASPI/VALUES/LOAD_CELL",
        "WEIGHT_IN_KG"
    ),
    Mq135(
        "mq135",
        "MQ135 Air Quality",
        "ppm",
        "RASPI/VALUES/MQ135_SENSOR",
        "AIR_QUALITY"
    ),
    Mq137(
        "mq137",
        "MQ137 Ammonia",
        "ppm",
        "RASPI/VALUES/MQ137",
        "AMMONIA"
    ),
    Mq4(
        "mq4",
        "MQ4 Methane",
        "ppm",
        "RASPI/VALUES/MQ4_SENSOR",
        "METHANE"
    )
}
