package com.thesis.garby.realtime

/**
 * Copies of the executor's trigger rules (`MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.h` and
 * `ReceivedDatas::setMQ*` / `sensorReadDecisionMaker` in `MAIN_EXECUTOR_CODE.cpp`).
 *
 * The app only displays them. The robot applies its own copy, and only while it is idle at base;
 * update both together if the firmware changes.
 */
object ExecutorThresholds {
    /** `LOAD_TRIGGER_KG`: two filtered samples at or above this start an outbound run. */
    const val LOAD_TRIGGER_KG = 4.0f

    /** `LOAD_VALID_MAX_KG`: heavier readings are treated as invalid by the executor. */
    const val LOAD_VALID_MAX_KG = 20.0f

    /** Presentation only: the load card turns amber from 75 % of the trigger. */
    const val LOAD_CAUTION_KG = 3.0f

    /** Filtered load samples needed in a row before a load-triggered run. */
    const val LOAD_CONFIRMATIONS = 2

    /** Consecutive dangerous gas packets needed before a gas-triggered run. */
    const val GAS_CONFIRMATIONS = 3

    /** The level at which any one gas sensor alone counts as dangerous. */
    const val GAS_DANGER_PPM = 700f
}

/** Ordered severities, mapped to colours by the UI. */
enum class Severity { Normal, Elevated, High, Danger }

data class GasBand(
    val label: String,
    /** Exclusive upper bound in ppm, or null for the top band. */
    val upperPpm: Float?,
    val severity: Severity
)

enum class GasSensor(val key: SensorKey, val bands: List<GasBand>) {
    /** MQ4 methane: NORMAL < 400 <= WARNING < 700 <= DANGER. */
    Mq4(
        SensorKey.Mq4,
        listOf(
            GasBand("Normal", 400f, Severity.Normal),
            GasBand("Warning", 700f, Severity.High),
            GasBand("Danger", null, Severity.Danger)
        )
    ),

    /** MQ135 air quality: CLEAN < 300 <= MODERATE < 500 <= POOR < 700 <= VERY POOR. */
    Mq135(
        SensorKey.Mq135,
        listOf(
            GasBand("Clean", 300f, Severity.Normal),
            GasBand("Moderate", 500f, Severity.Elevated),
            GasBand("Poor", 700f, Severity.High),
            GasBand("Very poor", null, Severity.Danger)
        )
    ),

    /** MQ137 ammonia: NORMAL < 400 <= WARNING < 700 <= DANGER. */
    Mq137(
        SensorKey.Mq137,
        listOf(
            GasBand("Normal", 400f, Severity.Normal),
            GasBand("Warning", 700f, Severity.High),
            GasBand("Danger", null, Severity.Danger)
        )
    )
}

/** The band for a live reading. Negative values are the unavailable sentinel: no band. */
fun gasBand(sensor: GasSensor, ppm: Float): GasBand? {
    if (!ppm.isFinite() || ppm < 0f) return null
    return sensor.bands.first { band -> band.upperPpm == null || ppm < band.upperPpm }
}

/**
 * The executor's danger rule for one telemetry packet: any sensor in its top band, or all three
 * in their second-highest band. The robot also needs [ExecutorThresholds.GAS_CONFIRMATIONS]
 * such packets in a row, and acts only while idle at base.
 */
fun gasMatchesDangerRule(mq4: Float?, mq135: Float?, mq137: Float?): Boolean {
    val bands = listOf(
        mq4?.let { gasBand(GasSensor.Mq4, it) },
        mq135?.let { gasBand(GasSensor.Mq135, it) },
        mq137?.let { gasBand(GasSensor.Mq137, it) }
    )
    if (bands.any { it?.severity == Severity.Danger }) return true
    return bands.all { it?.severity == Severity.High }
}

enum class LoadLevel { Light, Filling, AtTrigger, OutOfRange }

/** Null for the -1 "unavailable" sentinel and other invalid readings. */
fun loadLevel(kg: Float): LoadLevel? = when {
    !kg.isFinite() || kg < 0f -> null
    kg > ExecutorThresholds.LOAD_VALID_MAX_KG -> LoadLevel.OutOfRange
    kg >= ExecutorThresholds.LOAD_TRIGGER_KG -> LoadLevel.AtTrigger
    kg >= ExecutorThresholds.LOAD_CAUTION_KG -> LoadLevel.Filling
    else -> LoadLevel.Light
}
