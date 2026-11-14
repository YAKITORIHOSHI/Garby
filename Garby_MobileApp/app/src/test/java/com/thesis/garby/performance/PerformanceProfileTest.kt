package com.thesis.garby.performance

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class PerformanceProfileTest {
    private val gib = 1024L * 1024 * 1024

    @Test
    fun sixGbEightCorePhoneStillUsesLightRendering() {
        val profile = PerformanceProfile.forDevice(false, 6 * gib, 256, 8)
        assertTrue(profile.reducedEffects)
        assertEquals(1, profile.backgroundParallelism)
        assertEquals(4 * 1024 * 1024, profile.bitmapCacheBytes)
    }

    @Test
    fun lowRamSmallHeapAndFewCoresAreIndependentSignals() {
        assertTrue(PerformanceProfile.forDevice(true, 8 * gib, 256, 8).reducedEffects)
        assertTrue(PerformanceProfile.forDevice(false, 6 * gib, 256, 8).reducedEffects)
        assertTrue(PerformanceProfile.forDevice(false, 8 * gib, 192, 8).reducedEffects)
        assertTrue(PerformanceProfile.forDevice(false, 8 * gib, 256, 4).reducedEffects)
        assertTrue(PerformanceProfile.forDevice(false, 0, 0, 0).reducedEffects)
    }

    @Test
    fun capableDeviceKeepsEffectsWithBoundedBackgroundWork() {
        val profile = PerformanceProfile.forDevice(false, 8 * gib, 256, 8)
        assertFalse(profile.reducedEffects)
        assertEquals(2, profile.backgroundParallelism)
    }

    @Test
    fun artworkSamplingCutsPixelMemoryToOneQuarterWithoutUpscaling() {
        assertEquals(2, bitmapSampleSize(941, 1672, PerformanceProfile.Conservative.backdropMaxDimensionPx))
        assertEquals(2, bitmapSampleSize(816, 599, 512))
        assertEquals(1, bitmapSampleSize(128, 128, 512))
        assertEquals(1, bitmapSampleSize(1024, 1024, 1024))
        assertEquals(4, bitmapSampleSize(4096, 2160, 1024))
    }
}
