package com.thesis.garby.realtime

import org.junit.Assert.assertEquals
import org.junit.Test

class DatabaseSchemaTest {
    @Test
    fun sensorKeysMatchDeployedFirebaseExport() {
        assertEquals(
            "RASPI/VALUES/ULTRASONIC_SENSOR" to "CM_DISTANCE",
            SensorKey.Level.legacyPath to SensorKey.Level.legacyValueKey
        )
        assertEquals(
            "RASPI/VALUES/LOAD_CELL" to "WEIGHT_IN_KG",
            SensorKey.Weight.legacyPath to SensorKey.Weight.legacyValueKey
        )
        assertEquals(
            "RASPI/VALUES/MQ135_SENSOR" to "AIR_QUALITY",
            SensorKey.Mq135.legacyPath to SensorKey.Mq135.legacyValueKey
        )
        assertEquals(
            "RASPI/VALUES/MQ137" to "AMMONIA",
            SensorKey.Mq137.legacyPath to SensorKey.Mq137.legacyValueKey
        )
        assertEquals(
            "RASPI/VALUES/MQ4_SENSOR" to "METHANE",
            SensorKey.Mq4.legacyPath to SensorKey.Mq4.legacyValueKey
        )
    }
}
