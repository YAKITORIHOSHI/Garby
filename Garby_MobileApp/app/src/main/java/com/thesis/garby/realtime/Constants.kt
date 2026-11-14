package com.thesis.garby.realtime

/** Shared RTDB and freshness limits for the Android client. */
object RtdbConstants {
    const val DATABASE_URL = "https://garby-thesis-default-rtdb.asia-southeast1.firebasedatabase.app/"
    const val DEFAULT_DEVICE_ID = "garby-bin-01"

    /** Three missed 30-second Pi heartbeats make cached data stale and the robot offline. */
    const val STALE_AFTER_MS: Long = 90_000L

    /** Tolerated clock skew before a future timestamp is treated as invalid. */
    const val FUTURE_TIMESTAMP_TOLERANCE_MS: Long = 30_000L

    /** Frequency for re-evaluating cached values that can become stale without a callback. */
    const val FRESHNESS_TICK_MS: Long = 5_000L

    /** Return-screen clock; the acceptance window is short enough to count down in seconds. */
    const val RETURN_TICK_MS: Long = 1_000L

    const val WRITE_TIMEOUT_MS: Long = 15_000L
    const val FIREBASE_CONNECT_TIMEOUT_MS: Long = 5_000L

    /**
     * The Pi accepts a pending return only while it is at most 120 s old by its own clock
     * (`RESET_REQUEST_MAX_AGE_MS` in `RasPi/final_w_serial.py`). After that it marks it failed.
     */
    const val RETURN_ACCEPT_WINDOW_MS: Long = 120_000L

    /**
     * An accepted return has 15 minutes to report its outcome (`RESET_COMPLETION_TIMEOUT_S`);
     * after that the Pi records `completion_unverified`.
     */
    const val RETURN_COMPLETION_WINDOW_MS: Long = 900_000L

    /**
     * Allowance for the Pi's poll period, Firebase latency and clock differences before the app
     * calls a request expired or overdue. It only changes wording; the Pi decides the outcome.
     */
    const val RETURN_STAGE_GRACE_MS: Long = 15_000L
}
