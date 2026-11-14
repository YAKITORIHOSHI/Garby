package com.thesis.garby.realtime

/** Values are already converted to engineering units by the Raspberry Pi. */
data class SensorReading(
    val value: Float,
    val unit: String,
    val sensorType: String,
    val updatedAtMs: Long
)

/**
 * The executor's route state as the Pi publishes it in `devices/<id>/status/robotState`.
 *
 * The Pi maps controller lines onto these values: `[MCU READY]` and a lost BLE link -> UNKNOWN,
 * `[IDLE]` -> IDLE, `[RUNNING]` -> OUTBOUND, `[OUTBOUND COMPLETE]` -> AT_DESTINATION,
 * `[RETURNING]` -> RETURNING, `[ROUTE FAULT]` and `[RETURN REJECTED]` -> FAULT.
 */
enum class RobotState(val wireValue: String) {
    Idle("IDLE"),
    Outbound("OUTBOUND"),
    AtDestination("AT_DESTINATION"),
    Returning("RETURNING"),
    Fault("FAULT"),
    Unknown("UNKNOWN");

    companion object {
        /** Null when the field is absent (an older Pi); unrecognised text is [Unknown]. */
        fun fromWire(value: String?): RobotState? {
            val text = value?.trim()?.uppercase().orEmpty()
            if (text.isEmpty()) return null
            return entries.firstOrNull { it.wireValue == text } ?: Unknown
        }
    }
}

/**
 * The Pi's status record. Every field is nullable or zero when the Pi has not reported it, so
 * the app never shows an invented value. Freshness is judged from [lastSeenMs] at display time.
 */
data class DeviceStatus(
    /** Pi heartbeat in epoch milliseconds (Pi clock), written every 30 s; 0 when never reported. */
    val lastSeenMs: Long,
    /** Null when the Pi does not publish a state (versions before the state field). */
    val robotState: RobotState? = null,
    /** The Pi's destination flag: true only after a completed outbound route. Null when absent. */
    val isReadyToReturn: Boolean? = null,
    val recentUptimeStr: String? = null,
    val cpuTemperatureC: Float? = null,
    val thermalWarning: Boolean? = null,
    val throttledFlags: Long? = null,
    val bleConnected: Boolean? = null,
    val lidarHealthy: Boolean? = null,
    val sensorSerialConnected: Boolean? = null,
    /** Fresh Pi + executor eligibility for threshold-free disposal, never motion permission. */
    val manualDisposalReady: Boolean? = null,
    val manualDisposalUpdatedAtMs: Long = 0L
)

/** Wire statuses of `devices/<id>/commands/reset`, written by the app (pending) and the Pi. */
enum class ReturnStatus {
    Pending,
    Accepted,
    Done,
    Failed,
    Unknown;

    companion object {
        fun fromWire(value: String?): ReturnStatus = when (value?.trim()?.lowercase()) {
            "pending" -> Pending
            "ack" -> Accepted
            "done" -> Done
            "failed" -> Failed
            else -> Unknown
        }
    }
}

/** One return-to-base request and the outcome the Pi recorded for it. */
data class ReturnCommand(
    val requestedAtMs: Long,
    val requestedBy: String,
    val status: ReturnStatus,
    val ackAtMs: Long? = null,
    val doneAtMs: Long? = null,
    val failedAtMs: Long? = null,
    /** The Pi's failure reason code, for example `operator_emergency_stop`. */
    val reason: String? = null
)

/**
 * Identity of one request: the server-issued `requestedAt` plus the requesting operator. A newer
 * request, even from the same operator, never matches, so an old outcome cannot complete it.
 */
data class ReturnRequestRef(
    val requestedAtMs: Long,
    val requestedBy: String
) {
    fun matches(command: ReturnCommand?): Boolean =
        command != null && requestedAtMs > 0L &&
            command.requestedAtMs == requestedAtMs && command.requestedBy == requestedBy
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
        "Bin distance",
        "cm",
        "RASPI/VALUES/ULTRASONIC_SENSOR",
        "CM_DISTANCE"
    ),
    Weight(
        "weight",
        "Bin weight",
        "kg",
        "RASPI/VALUES/LOAD_CELL",
        "WEIGHT_IN_KG"
    ),
    Mq135(
        "mq135",
        "Air quality (MQ135)",
        "ppm",
        "RASPI/VALUES/MQ135_SENSOR",
        "AIR_QUALITY"
    ),
    Mq137(
        "mq137",
        "Ammonia (MQ137)",
        "ppm",
        "RASPI/VALUES/MQ137",
        "AMMONIA"
    ),
    Mq4(
        "mq4",
        "Methane (MQ4)",
        "ppm",
        "RASPI/VALUES/MQ4_SENSOR",
        "METHANE"
    )
}
