package com.thesis.garby.libs

import androidx.compose.animation.core.Animatable
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.thesis.garby.ui.theme.Montserrat
import kotlin.math.cos
import kotlin.math.sin

/**
 * Compact, single-animation sensor gauge.
 *
 * The original implementation animated the same value twice and derived zones
 * asynchronously, which added lag and recomposition churn. This version clamps
 * input once, animates one normalized progress value, and computes the zone
 * synchronously from the authoritative reading.
 */
@Composable
fun SensorGauge(
    modifier: Modifier = Modifier,
    data: SensorData,
    currentValueProvider: () -> Float,
    sensorThresholds: SensorThresholds,
    animateDuration: Int = 900,
    gaugeSize: androidx.compose.ui.unit.Dp = 150.dp
) {
    val currentValue = currentValueProvider()

    Box(
        modifier = modifier
            .shadow(10.dp, RoundedCornerShape(22.dp))
            .background(Color.White, RoundedCornerShape(22.dp))
            .padding(horizontal = 18.dp, vertical = 16.dp)
    ) {
        Column(
            horizontalAlignment = Alignment.CenterHorizontally,
            modifier = Modifier.fillMaxWidth()
        ) {
            Text(
                text = data.name,
                color = Color(0xFFE91E1E),
                fontFamily = Montserrat,
                fontWeight = FontWeight.Black,
                fontSize = 15.sp,
                textAlign = TextAlign.Center
            )

            Spacer(modifier = Modifier.height(8.dp))

            CircularGauge(
                currentValue = currentValue,
                maxValue = data.maxValue,
                unit = data.unit,
                thresholds = sensorThresholds,
                animationDurationMs = animateDuration,
                modifier = Modifier.size(gaugeSize)
            )
        }
    }
}

@Composable
private fun CircularGauge(
    currentValue: Float,
    maxValue: Float,
    unit: String,
    thresholds: SensorThresholds,
    animationDurationMs: Int,
    modifier: Modifier = Modifier
) {
    val safeMax = maxValue.takeIf { it.isFinite() && it > 0f } ?: 1f
    val safeValue = currentValue
        .takeIf { it.isFinite() }
        ?.coerceIn(0f, safeMax)
        ?: 0f
    val targetProgress = (safeValue / safeMax).coerceIn(0f, 1f)

    val zoneColor = when {
        safeValue < thresholds.normal -> thresholds.normalColor
        safeValue < thresholds.warning -> thresholds.warningColor
        safeValue < thresholds.severe -> thresholds.severeColor
        else -> thresholds.criticalColor
    }

    val progress = remember { Animatable(0f) }
    LaunchedEffect(targetProgress) {
        progress.animateTo(
            targetValue = targetProgress,
            animationSpec = tween(animationDurationMs.coerceIn(0, 5_000))
        )
    }

    Box(modifier = modifier, contentAlignment = Alignment.Center) {
        Canvas(modifier = Modifier.matchParentSize()) {
            val strokeWidth = size.minDimension * 0.10f
            val diameter = size.minDimension - strokeWidth
            val topLeft = Offset(
                (size.width - diameter) / 2f,
                (size.height - diameter) / 2f
            )
            val arcSize = Size(diameter, diameter)
            val startAngle = 135f
            val totalSweep = 270f

            drawArc(
                color = Color(0xFFE9ECEF),
                startAngle = startAngle,
                sweepAngle = totalSweep,
                useCenter = false,
                topLeft = topLeft,
                size = arcSize,
                style = Stroke(width = strokeWidth, cap = StrokeCap.Round)
            )

            drawArc(
                color = zoneColor,
                startAngle = startAngle,
                sweepAngle = totalSweep * progress.value,
                useCenter = false,
                topLeft = topLeft,
                size = arcSize,
                style = Stroke(width = strokeWidth, cap = StrokeCap.Round)
            )

            val indicatorAngleDegrees = startAngle + totalSweep * progress.value
            val indicatorAngleRadians = Math.toRadians(indicatorAngleDegrees.toDouble())
            val radius = diameter / 2f
            val center = this.center
            val indicatorCenter = Offset(
                x = center.x + cos(indicatorAngleRadians).toFloat() * radius,
                y = center.y + sin(indicatorAngleRadians).toFloat() * radius
            )
            drawCircle(
                color = Color.White,
                radius = strokeWidth * 0.42f,
                center = indicatorCenter
            )
            drawCircle(
                color = zoneColor,
                radius = strokeWidth * 0.25f,
                center = indicatorCenter
            )
        }

        val locale = LocalConfiguration.current.locales[0]
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(
                text = String.format(locale, "%.1f", safeValue),
                color = Color(0xFF9A1D1D),
                fontFamily = Montserrat,
                fontWeight = FontWeight.Black,
                fontSize = 28.sp
            )
            Text(
                text = unit,
                color = Color(0xFFE91E1E),
                fontFamily = Montserrat,
                fontWeight = FontWeight.Bold,
                fontSize = 13.sp
            )
        }
    }
}
