package com.thesis.garby

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.rounded.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.android.tools.screenshot.PreviewTest
import com.thesis.garby.realtime.*
import com.thesis.garby.ui.glass.SectionHeader
import com.thesis.garby.ui.glass.StatusPill
import com.thesis.garby.ui.theme.GarbyColors
import com.thesis.garby.ui.theme.GarbyTheme

private const val NOW = 1_800_000_000_000L

private fun sample(value: Float, unit: String) = SensorReadingUiState(
    reading = SensorReading(value, unit, "", NOW - 12_000),
    isLoading = false,
    isStale = false
)

private fun sampleState() = DeviceUiState(
    connection = ConnectionState.Connected,
    weight = sample(2.35f, "kg"),
    level = sample(32.5f, "cm"),
    mq135 = sample(412f, "ppm"),
    mq137 = sample(128f, "ppm"),
    mq4 = sample(236f, "ppm"),
    device = DeviceStatusUiState(
        status = DeviceStatus(
            lastSeenMs = NOW - 12_000,
            robotState = RobotState.Idle,
            cpuTemperatureC = 54.3f,
            throttledFlags = 0L,
            bleConnected = true,
            lidarHealthy = true,
            sensorSerialConnected = true
        ),
        isLoading = false,
        isStale = false
    ),
    nowMs = NOW
)

@PreviewTest
@Preview(device = "spec:width=400dp,height=2400dp,dpi=160", name = "Dashboard 400", widthDp = 400, heightDp = 2400)
@Composable
fun DashboardRender() {
    GarbyTheme {
        DashboardContent(sampleState(), ReturnSummary(), {}, {})
    }
}

@PreviewTest
@Preview(device = "spec:width=320dp,height=1600dp,dpi=160", name = "Narrow 320", widthDp = 320, heightDp = 1600)
@Preview(device = "spec:width=400dp,height=1800dp,dpi=160", name = "Large text 400", widthDp = 400, heightDp = 1800, fontScale = 1.5f)
@Preview(device = "spec:width=360dp,height=1600dp,dpi=160", name = "Large text", widthDp = 360, heightDp = 1600, fontScale = 1.5f)
@Composable
fun NarrowDashboardRender() {
    GarbyTheme {
        DashboardContent(sampleState(), ReturnSummary(), {}, {})
    }
}

@PreviewTest
@Preview(device = "spec:width=400dp,height=1100dp,dpi=160", name = "Category states 400", widthDp = 400, heightDp = 1100)
@Preview(device = "spec:width=320dp,height=1100dp,dpi=160", name = "Category states 320", widthDp = 320, heightDp = 1100)
@Preview(device = "spec:width=320dp,height=1900dp,dpi=160", name = "Category states extra large", widthDp = 320, heightDp = 1900, fontScale = 2f)
@Preview(device = "spec:width=360dp,height=1600dp,dpi=160", name = "Category states large text", widthDp = 360, heightDp = 1600, fontScale = 1.5f)
@Composable
fun CategoryStatesRender() {
    GarbyTheme {
        Box(Modifier.fillMaxSize().background(GarbyColors.Night)) {
            Image(
                painterResource(R.drawable.app_background), null,
                modifier = Modifier.fillMaxSize(), contentScale = ContentScale.Crop
            )
            Box(Modifier.fillMaxSize().background(GarbyColors.Night.copy(alpha = 0.55f)))
            Column(Modifier.fillMaxWidth().padding(16.dp)) {
                SectionHeader("Bin", Icons.Rounded.DeleteOutline, GarbyColors.Rose,
                    caption = "Weight and distance · 4\u00A0kg trigger",
                    trailing = { StatusPill("2/2 live", GarbyColors.Ok, icon = Icons.Rounded.CheckCircle) })
                SectionHeader("Air", Icons.Rounded.Air, GarbyColors.Info,
                    caption = "Three gas sensors · Threshold bands",
                    trailing = { StatusPill("Elevated", GarbyColors.Caution, icon = Icons.Rounded.Info) })
                SectionHeader("System", Icons.Rounded.Tune, GarbyColors.Neutral,
                    caption = "Connections · Pi health · 30\u00A0s updates",
                    trailing = { StatusPill("8/8 healthy", GarbyColors.Ok, icon = Icons.Rounded.CheckCircle) })
                SectionHeader("Bin", Icons.Rounded.DeleteOutline, GarbyColors.Rose,
                    caption = "Weight and distance · 4\u00A0kg trigger",
                    trailing = { StatusPill("Dispatch weight", GarbyColors.Caution, icon = Icons.Rounded.Info) })
                SectionHeader("Air", Icons.Rounded.Air, GarbyColors.Info,
                    caption = "Three gas sensors · Threshold bands",
                    trailing = { StatusPill("Danger band", GarbyColors.Danger, icon = Icons.Rounded.Warning) })
                SectionHeader("System", Icons.Rounded.Tune, GarbyColors.Neutral,
                    caption = "Connections · Pi health · 30\u00A0s updates",
                    trailing = { StatusPill("2 issues", GarbyColors.Danger, icon = Icons.Rounded.Warning) })
                SectionHeader("Bin", Icons.Rounded.DeleteOutline, GarbyColors.Rose,
                    caption = "Weight and distance · 4\u00A0kg trigger",
                    trailing = { StatusPill("Stale", GarbyColors.Neutral, icon = Icons.Rounded.Schedule) })
                SectionHeader("Air", Icons.Rounded.Air, GarbyColors.Info,
                    caption = "Three gas sensors · Threshold bands",
                    trailing = { StatusPill("No data", GarbyColors.Neutral, icon = Icons.Rounded.ErrorOutline) })
            }
        }
    }
}
