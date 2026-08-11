package com.thesis.garby.libs

import org.junit.Assert.assertEquals
import org.junit.Test

class SensorStatusTest {
    @Test
    fun weightThresholdBoundariesAreStable() {
        assertEquals(SensorStatus.NORMAL, getCurrentWeight(0.699f))
        assertEquals(SensorStatus.MODERATE, getCurrentWeight(0.70f))
        assertEquals(SensorStatus.SEVERE, getCurrentWeight(1.00f))
    }

    @Test
    fun gasThresholdBoundariesAreStable() {
        assertEquals(GAS.LOW, getGasLevelStatus(399.999f))
        assertEquals(GAS.MEDIUM, getGasLevelStatus(400f))
        assertEquals(GAS.HIGH, getGasLevelStatus(700f))
    }
}
