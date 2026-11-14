package com.thesis.garby.realtime

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

/** Boundaries copied from MAIN_EXECUTOR_CODE (ReceivedDatas::setMQ*, LOAD_TRIGGER_KG). */
class ThresholdsTest {

    @Test
    fun methaneAndAmmoniaBandsMatchTheExecutor() {
        for (sensor in listOf(GasSensor.Mq4, GasSensor.Mq137)) {
            assertEquals("Normal", gasBand(sensor, 399f)?.label)
            assertEquals("Warning", gasBand(sensor, 400f)?.label)
            assertEquals("Warning", gasBand(sensor, 699f)?.label)
            assertEquals("Danger", gasBand(sensor, 700f)?.label)
        }
    }

    @Test
    fun airQualityHasFourBandsLikeTheExecutor() {
        assertEquals("Clean", gasBand(GasSensor.Mq135, 299f)?.label)
        assertEquals("Moderate", gasBand(GasSensor.Mq135, 300f)?.label)
        assertEquals("Poor", gasBand(GasSensor.Mq135, 500f)?.label)
        assertEquals("Very poor", gasBand(GasSensor.Mq135, 700f)?.label)
    }

    @Test
    fun unavailableSentinelHasNoBand() {
        assertNull(gasBand(GasSensor.Mq4, -1f))
        assertNull(gasBand(GasSensor.Mq135, Float.NaN))
    }

    @Test
    fun dangerRuleFollowsSensorReadDecisionMaker() {
        // Any single sensor in its top band.
        assertTrue(gasMatchesDangerRule(mq4 = 700f, mq135 = 0f, mq137 = 0f))
        assertTrue(gasMatchesDangerRule(mq4 = 0f, mq135 = 700f, mq137 = 0f))
        assertTrue(gasMatchesDangerRule(mq4 = 0f, mq135 = 0f, mq137 = 700f))
        // All three in their second-highest band.
        assertTrue(gasMatchesDangerRule(mq4 = 400f, mq135 = 500f, mq137 = 400f))
        // Two of three, or MQ135 only moderate, is not enough.
        assertFalse(gasMatchesDangerRule(mq4 = 400f, mq135 = 499f, mq137 = 400f))
        assertFalse(gasMatchesDangerRule(mq4 = 400f, mq135 = 500f, mq137 = null))
        assertFalse(gasMatchesDangerRule(mq4 = -1f, mq135 = -1f, mq137 = -1f))
    }

    @Test
    fun loadLevelsUseTheFourKilogramTrigger() {
        assertEquals(LoadLevel.Light, loadLevel(2.99f))
        assertEquals(LoadLevel.Filling, loadLevel(3.0f))
        assertEquals(LoadLevel.Filling, loadLevel(3.99f))
        assertEquals(LoadLevel.AtTrigger, loadLevel(4.0f))
        assertEquals(LoadLevel.AtTrigger, loadLevel(20.0f))
        assertEquals(LoadLevel.OutOfRange, loadLevel(20.01f))
        assertNull(loadLevel(-1f))
        assertEquals(4.0f, ExecutorThresholds.LOAD_TRIGGER_KG)
    }
}
