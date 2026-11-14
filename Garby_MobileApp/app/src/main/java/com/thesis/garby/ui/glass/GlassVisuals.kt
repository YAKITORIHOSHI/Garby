package com.thesis.garby.ui.glass

import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxScope
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.rounded.Flag
import androidx.compose.material.icons.rounded.Home
import androidx.compose.material.icons.rounded.PriorityHigh
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.ui.Alignment
import androidx.compose.ui.BiasAlignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawWithCache
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.semantics.clearAndSetSemantics
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import com.thesis.garby.R
import com.thesis.garby.performance.LocalPerformanceProfile
import com.thesis.garby.ui.rememberResourceBitmap
import com.thesis.garby.ui.theme.GarbyColors
import kotlin.math.cos
import kotlin.math.sin

/** The round GARBY logo inside a glass ring. The source image is off-centre, hence the bias. */
@Composable
fun LogoBadge(size: Dp, modifier: Modifier = Modifier, ring: Dp = 3.dp) {
    val profile = LocalPerformanceProfile.current
    val reducedEffects = profile.reducedEffects
    val bitmap by rememberResourceBitmap(R.drawable.app_icon, profile.logoMaxDimensionPx)
    Box(
        contentAlignment = Alignment.Center,
        modifier = modifier
            .size(size)
            .then(if (reducedEffects) Modifier else Modifier.shadow(
                size / 6, CircleShape, ambientColor = GarbyColors.Ember, spotColor = GarbyColors.Ember
            ))
            .clip(CircleShape)
            .background(Color.White.copy(alpha = 0.10f))
            .border(1.dp, GlassBorderBrush, CircleShape)
            .padding(ring)
    ) {
        bitmap?.let {
            Image(
                bitmap = it,
                contentDescription = "GARBY logo",
                contentScale = ContentScale.Crop,
                alignment = BiasAlignment(horizontalBias = -0.22f, verticalBias = 0f),
                modifier = Modifier
                    .size(size - ring * 2)
                    .clip(CircleShape)
            )
        }
    }
}

/**
 * A 270° ring. [progress] is clamped for drawing only, so callers can still print a value beyond
 * the scale. [markers] are fractions of the scale drawn as ticks (thresholds).
 */
@Composable
fun RingGauge(
    progress: Float,
    color: Color,
    modifier: Modifier = Modifier,
    strokeWidth: Dp = 10.dp,
    markers: List<Float> = emptyList(),
    content: @Composable BoxScope.() -> Unit = {}
) {
    val target = if (progress.isFinite()) progress.coerceIn(0f, 1f) else 0f
    val reducedEffects = LocalPerformanceProfile.current.reducedEffects
    val animated = if (reducedEffects) rememberUpdatedState(target) else
        animateFloatAsState(target, animationSpec = tween(250), label = "ringGauge")
    Box(contentAlignment = Alignment.Center, modifier = modifier) {
        Canvas(Modifier.matchParentSize()) {
            val sweepFraction = animated.value
            val stroke = strokeWidth.toPx()
            val diameter = size.minDimension - stroke * 2f
            val topLeft = Offset((size.width - diameter) / 2f, (size.height - diameter) / 2f)
            val arcSize = Size(diameter, diameter)
            val start = 135f
            val sweep = 270f
            drawArc(
                color = Color.White.copy(alpha = 0.12f),
                startAngle = start,
                sweepAngle = sweep,
                useCenter = false,
                topLeft = topLeft,
                size = arcSize,
                style = Stroke(stroke, cap = StrokeCap.Round)
            )
            if (sweepFraction > 0f) {
                // Soft glow under the value arc.
                if (!reducedEffects) drawArc(
                    color = color.copy(alpha = 0.22f),
                    startAngle = start,
                    sweepAngle = sweep * sweepFraction,
                    useCenter = false,
                    topLeft = topLeft,
                    size = arcSize,
                    style = Stroke(stroke * 2f, cap = StrokeCap.Round)
                )
                drawArc(
                    color = color,
                    startAngle = start,
                    sweepAngle = sweep * sweepFraction,
                    useCenter = false,
                    topLeft = topLeft,
                    size = arcSize,
                    style = Stroke(stroke, cap = StrokeCap.Round)
                )
            }
            val radius = diameter / 2f
            markers.filter { it in 0f..1f }.forEach { fraction ->
                val angle = Math.toRadians((start + sweep * fraction).toDouble())
                val inner = radius - stroke * 0.9f
                val outer = radius + stroke * 0.9f
                drawLine(
                    color = Color.White.copy(alpha = 0.85f),
                    start = Offset(center.x + cos(angle).toFloat() * inner, center.y + sin(angle).toFloat() * inner),
                    end = Offset(center.x + cos(angle).toFloat() * outer, center.y + sin(angle).toFloat() * outer),
                    strokeWidth = 2.dp.toPx(),
                    cap = StrokeCap.Round
                )
            }
        }
        content()
    }
}

/** One coloured segment of a [BandBar], ending at [upper] on the bar's scale. */
data class BarBand(val upper: Float, val color: Color)

/**
 * A horizontal scale split into bands with a marker for the value. The marker is clamped to the
 * scale; the caller prints the real value.
 */
@Composable
fun BandBar(
    value: Float?,
    maxValue: Float,
    bands: List<BarBand>,
    modifier: Modifier = Modifier,
    markerColor: Color = Color.White
) {
    val fraction = value?.takeIf { it.isFinite() && maxValue > 0f }?.let { (it / maxValue).coerceIn(0f, 1f) }
    val animated = if (LocalPerformanceProfile.current.reducedEffects) rememberUpdatedState(fraction ?: 0f) else
        animateFloatAsState(fraction ?: 0f, animationSpec = tween(250), label = "bandBar")
    Canvas(
        modifier
            .fillMaxWidth()
            .height(14.dp)
    ) {
        val barHeight = 6.dp.toPx()
        val y = size.height / 2f
        var start = 0f
        bands.forEach { band ->
            val end = (band.upper / maxValue).coerceIn(0f, 1f) * size.width
            if (end > start) {
                drawLine(
                    color = band.color.copy(alpha = 0.55f),
                    start = Offset(start, y),
                    end = Offset(end, y),
                    strokeWidth = barHeight
                )
            }
            start = end
        }
        if (fraction != null) {
            val x = (animated.value * size.width).coerceIn(barHeight, size.width - barHeight)
            drawCircle(markerColor.copy(alpha = 0.3f), radius = size.height / 2f + 2.dp.toPx(), center = Offset(x, y))
            drawCircle(markerColor, radius = size.height / 2f - 1.dp.toPx(), center = Offset(x, y))
        }
    }
}

/** Where the robot is along its route, as far as its state says. */
enum class RoutePosition { AtBase, ToDestination, AtDestination, ToBase, Stopped, Unknown }

/**
 * Base on the left, the collection point on the right. The state does not report distance,
 * so a moving robot is drawn halfway with its direction. [animate] controls marker transitions;
 * dashes remain static so a parked screen does not generate continuous frames.
 */
@Composable
fun RouteTrack(
    position: RoutePosition,
    accent: Color,
    animate: Boolean,
    description: String,
    modifier: Modifier = Modifier
) {
    val travelling = position == RoutePosition.ToDestination || position == RoutePosition.ToBase
    val reducedEffects = LocalPerformanceProfile.current.reducedEffects
    val markerFraction = when (position) {
        RoutePosition.AtBase -> 0f
        RoutePosition.AtDestination -> 1f
        RoutePosition.ToDestination, RoutePosition.ToBase, RoutePosition.Stopped -> 0.5f
        RoutePosition.Unknown -> null
    }
    val animatedFraction = if (reducedEffects || !animate) rememberUpdatedState(markerFraction ?: 0.5f) else
        animateFloatAsState(markerFraction ?: 0.5f, tween(250), label = "routeMarker")

    Column(modifier.clearAndSetSemantics { contentDescription = description }) {
        BoxWithConstraints(
            Modifier
                .fillMaxWidth()
                .height(44.dp)
        ) {
            val edge = 22.dp
            val trackWidth = maxWidth - edge * 2
            val markerSize = 38.dp
            Canvas(Modifier.matchParentSize()) {
                val y = size.height / 2f
                val x0 = edge.toPx()
                val x1 = size.width - edge.toPx()
                val stroke = 4.dp.toPx()
                drawLine(
                    color = Color.White.copy(alpha = if (markerFraction == null) 0.10f else 0.16f),
                    start = Offset(x0, y),
                    end = Offset(x1, y),
                    strokeWidth = stroke,
                    cap = StrokeCap.Round,
                    pathEffect = if (markerFraction == null) PathEffect.dashPathEffect(floatArrayOf(10f, 10f)) else null
                )
                if (markerFraction != null && position != RoutePosition.AtBase) {
                    val mx = x0 + (x1 - x0) * animatedFraction.value
                    val dash = 14.dp.toPx()
                    val (from, to) = if (position == RoutePosition.ToBase) mx to x1 else x0 to mx
                    if (to > from) {
                        drawLine(
                            brush = Brush.horizontalGradient(
                                listOf(accent.copy(alpha = 0.25f), accent),
                                startX = from,
                                endX = to
                            ),
                            start = Offset(from, y),
                            end = Offset(to, y),
                            strokeWidth = stroke,
                            cap = StrokeCap.Round,
                            pathEffect = if (travelling) {
                                PathEffect.dashPathEffect(floatArrayOf(dash, dash * 0.6f))
                            } else {
                                null
                            }
                        )
                    }
                }
                drawCircle(Color.White.copy(alpha = 0.85f), radius = 5.dp.toPx(), center = Offset(x0, y))
                drawCircle(Color.White.copy(alpha = 0.85f), radius = 5.dp.toPx(), center = Offset(x1, y))
            }
            if (markerFraction != null) {
                Box(
                    contentAlignment = Alignment.Center,
                    modifier = Modifier
                        .offset {
                            IntOffset(
                                x = (edge + trackWidth * animatedFraction.value - markerSize / 2).roundToPx(),
                                y = 3.dp.roundToPx()
                            )
                        }
                        .size(markerSize)
                        .then(if (reducedEffects) Modifier else Modifier.shadow(
                            10.dp, CircleShape, ambientColor = accent, spotColor = accent
                        ))
                        .clip(CircleShape)
                        .background(Brush.linearGradient(listOf(accent, accent.copy(alpha = 0.72f))))
                        .border(1.5.dp, Color.White.copy(alpha = 0.7f), CircleShape)
                ) {
                    if (position == RoutePosition.Stopped) {
                        Icon(
                            Icons.Rounded.PriorityHigh,
                            contentDescription = null,
                            tint = GarbyColors.Night,
                            modifier = Modifier.size(20.dp)
                        )
                    } else {
                        Icon(
                            painter = painterResource(R.drawable.ic_stat_garby),
                            contentDescription = null,
                            tint = GarbyColors.Night,
                            modifier = Modifier.size(22.dp)
                        )
                    }
                }
            }
        }
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier
                .fillMaxWidth()
                .padding(top = 6.dp)
        ) {
            Icon(Icons.Rounded.Home, contentDescription = null, tint = GarbyColors.TextMuted, modifier = Modifier.size(14.dp))
            Spacer(Modifier.width(4.dp))
            Text("Base", style = MaterialTheme.typography.labelSmall, color = GarbyColors.TextMuted)
            Spacer(Modifier.weight(1f))
            Text("Collection point", style = MaterialTheme.typography.labelSmall, color = GarbyColors.TextMuted)
            Spacer(Modifier.width(4.dp))
            Icon(Icons.Rounded.Flag, contentDescription = null, tint = GarbyColors.TextMuted, modifier = Modifier.size(14.dp))
        }
    }
}

/** A static glass placeholder, shown until the first value arrives. */
@Composable
fun ShimmerPanel(label: String, modifier: Modifier = Modifier) {
    GlassPanel(modifier = modifier.semantics { contentDescription = label }) {
        Box(
            Modifier
                .fillMaxWidth()
                .height(96.dp)
                .drawWithCache {
                    val bars = listOf(
                        Triple(0f, 40.dp.toPx(), 40.dp.toPx()),
                        Triple(54.dp.toPx(), size.width * 0.55f, 12.dp.toPx()),
                        Triple(76.dp.toPx(), size.width * 0.8f, 20.dp.toPx())
                    )
                    onDrawBehind {
                        val brush = Brush.linearGradient(
                            colors = listOf(
                                Color.White.copy(alpha = 0.06f),
                                Color.White.copy(alpha = 0.18f),
                                Color.White.copy(alpha = 0.06f)
                            ),
                            start = Offset(size.width * 0.3f, 0f),
                            end = Offset(size.width * 0.9f, 0f)
                        )
                        bars.forEach { (top, width, height) ->
                            drawRoundRect(
                                brush = brush,
                                topLeft = Offset(0f, top),
                                size = Size(width, height),
                                cornerRadius = CornerRadius(height / 2f)
                            )
                        }
                    }
                }
        )
    }
}
