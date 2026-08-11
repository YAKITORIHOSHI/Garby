package com.thesis.garby.realtime

/** Shared RTDB and freshness limits for the Android client. */
object RtdbConstants {
    const val DATABASE_URL = "https://garby-thesis-default-rtdb.asia-southeast1.firebasedatabase.app/"
    const val DEFAULT_DEVICE_ID = "garby-bin-01"

    /** Sensor/device data older than this is marked as stale (10 minutes). */
    const val STALE_AFTER_MS: Long = 600_000L

    /** Tolerated clock skew before a future timestamp is treated as invalid (5 minutes). */
    const val FUTURE_TIMESTAMP_TOLERANCE_MS: Long = 300_000L

    /** Robot is considered offline if recent_uptime is older than 3 minutes. */
    const val ROBOT_UPTIME_TIMEOUT_MS: Long = 180_000L

    /** Frequency for re-evaluating cached values that can become stale without a callback. */
    const val FRESHNESS_TICK_MS: Long = 5_000L

    const val MAX_HISTORY = 120
    const val WRITE_TIMEOUT_MS: Long = 15_000L
    const val FIREBASE_CONNECT_TIMEOUT_MS: Long = 5_000L
    const val RESET_ACK_TIMEOUT_MS: Long = 60_000L
}

