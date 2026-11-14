package com.thesis.garby.ui.theme

import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Shapes
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import com.thesis.garby.performance.DevicePerformance
import com.thesis.garby.performance.LocalPerformanceProfile

/**
 * The glass UI is designed for one dark look over the marble backdrop, so it does not follow
 * the system theme or wallpaper colours; status colours must stay recognisable everywhere.
 */
private val GarbyColorScheme = darkColorScheme(
    primary = GarbyColors.Ember,
    onPrimary = Color.White,
    primaryContainer = GarbyColors.Crimson,
    onPrimaryContainer = Color.White,
    secondary = GarbyColors.Rose,
    onSecondary = GarbyColors.Night,
    tertiary = GarbyColors.Info,
    onTertiary = GarbyColors.Night,
    background = GarbyColors.Night,
    onBackground = Color.White,
    surface = GarbyColors.NightRaised,
    onSurface = Color.White,
    surfaceVariant = Color(0xFF2B1418),
    onSurfaceVariant = GarbyColors.TextSecondary,
    surfaceContainerHigh = Color(0xFF261216),
    surfaceContainerHighest = Color(0xFF30171C),
    error = GarbyColors.Danger,
    onError = GarbyColors.Night,
    outline = GarbyColors.GlassStroke,
    outlineVariant = GarbyColors.GlassStrokeFaint,
    scrim = Color.Black
)

private val GarbyShapes = Shapes(
    small = RoundedCornerShape(14.dp),
    medium = RoundedCornerShape(20.dp),
    large = RoundedCornerShape(28.dp),
    extraLarge = RoundedCornerShape(32.dp)
)

@Composable
fun GarbyTheme(content: @Composable () -> Unit) {
    CompositionLocalProvider(LocalPerformanceProfile provides DevicePerformance.profile) {
        MaterialTheme(
            colorScheme = GarbyColorScheme,
            typography = GarbyTypography,
            shapes = GarbyShapes,
            content = content
        )
    }
}
