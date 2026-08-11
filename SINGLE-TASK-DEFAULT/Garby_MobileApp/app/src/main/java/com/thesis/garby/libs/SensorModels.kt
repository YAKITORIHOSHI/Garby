package com.thesis.garby.libs

import androidx.compose.ui.graphics.Color

data class SensorData(
    val id: Int,
    val name: String,
    val unit: String,
    val maxValue: Float
)

data class SensorThresholds(
    val normal: Float,
    val warning: Float,
    val severe: Float,
    val normalColor: Color = Color(0xFF4CAF50),
    val warningColor: Color = Color(0xFFFFC107),
    val severeColor: Color = Color(0xFFF44336),
    val criticalColor: Color = Color(0xFF8B0000)
)
