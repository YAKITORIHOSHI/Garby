package com.thesis.garby.ui

import org.junit.Assert.assertEquals
import org.junit.Test
import java.util.Locale
import java.util.TimeZone

class FormattingTest {

    @Test
    fun agesRoundDownToTheirUnit() {
        assertEquals("just now", formatAgo(-2_000L))
        assertEquals("just now", formatAgo(4_999L))
        assertEquals("5 s ago", formatAgo(5_000L))
        assertEquals("59 s ago", formatAgo(59_999L))
        assertEquals("1 min ago", formatAgo(60_000L))
        assertEquals("2 h ago", formatAgo(2 * 3_600_000L + 5L))
        assertEquals("3 d ago", formatAgo(3 * 86_400_000L))
    }

    @Test
    fun countdownRoundsUpAndNeverGoesNegative() {
        assertEquals("2:00", formatCountdown(120_000L))
        assertEquals("1:42", formatCountdown(101_001L))
        assertEquals("0:01", formatCountdown(1L))
        assertEquals("0:00", formatCountdown(-5_000L))
    }

    @Test
    fun timeOfDayAddsTheDateOnlyForOtherDays() {
        val utc = TimeZone.getTimeZone("UTC")
        val noon = 1_763_035_200_000L // 2025-11-13 12:00 UTC
        val sameDay = formatTimeOfDay(noon, noon + 3_600_000L, Locale.US, utc)
        val otherDay = formatTimeOfDay(noon, noon + 2 * 86_400_000L, Locale.US, utc)
        assertEquals("12:00 PM", sameDay.replace(' ', ' '))
        assertEquals(true, otherDay.contains("2025"))
    }
}
