package com.thesis.garby.libs

import androidx.compose.ui.graphics.Color

enum class SensorStatus(val color: Color) {
    NORMAL(Color(0xFF4CAF50)),
    MODERATE(Color(0xFFFFC107)),
    SEVERE(Color(0xFFF44336))
}

enum class GAS(val color: Color) {

    LOW(Color(0xFF4CAF50)),
    MEDIUM(Color(0xFFFFC107)),
    HIGH(Color(0xFFF44336))

}

fun getGasLevelStatus(value: Float): GAS = when {
    value < 400f -> GAS.LOW
    value < 700f -> GAS.MEDIUM
    else        -> GAS.HIGH
}

fun getCurrentWeight(value: Float): SensorStatus = when {
    value < 0.70f -> SensorStatus.NORMAL
    value < 1.00f -> SensorStatus.MODERATE
    else        -> SensorStatus.SEVERE
}
