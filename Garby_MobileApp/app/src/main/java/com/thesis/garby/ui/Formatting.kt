package com.thesis.garby.ui

import java.text.DateFormat
import java.util.Calendar
import java.util.Date
import java.util.Locale
import java.util.TimeZone

/** "just now", "42 s ago", "5 min ago", "3 h ago", "2 d ago". Future ages read as just now. */
fun formatAgo(ageMs: Long): String = when {
    ageMs < 5_000L -> "just now"
    ageMs < 60_000L -> "${ageMs / 1_000L} s ago"
    ageMs < 3_600_000L -> "${ageMs / 60_000L} min ago"
    ageMs < 86_400_000L -> "${ageMs / 3_600_000L} h ago"
    else -> "${ageMs / 86_400_000L} d ago"
}

/** m:ss, rounded up so a countdown never shows 0:00 while time remains. */
fun formatCountdown(remainingMs: Long): String {
    val totalSeconds = ((remainingMs.coerceAtLeast(0L) + 999L) / 1_000L)
    return "%d:%02d".format(Locale.ROOT, totalSeconds / 60L, totalSeconds % 60L)
}

/** Local clock time, with the date when it is not today. */
fun formatTimeOfDay(
    epochMs: Long,
    nowMs: Long,
    locale: Locale,
    zone: TimeZone = TimeZone.getDefault()
): String {
    val then = Calendar.getInstance(zone, locale).apply { timeInMillis = epochMs }
    val now = Calendar.getInstance(zone, locale).apply { timeInMillis = nowMs }
    val sameDay = then.get(Calendar.YEAR) == now.get(Calendar.YEAR) &&
        then.get(Calendar.DAY_OF_YEAR) == now.get(Calendar.DAY_OF_YEAR)
    val format = if (sameDay) {
        DateFormat.getTimeInstance(DateFormat.SHORT, locale)
    } else {
        DateFormat.getDateTimeInstance(DateFormat.MEDIUM, DateFormat.SHORT, locale)
    }
    format.timeZone = zone
    return format.format(Date(epochMs))
}

fun formatDecimal(value: Float, decimals: Int, locale: Locale): String =
    String.format(locale, "%.${decimals.coerceIn(0, 3)}f", value)
