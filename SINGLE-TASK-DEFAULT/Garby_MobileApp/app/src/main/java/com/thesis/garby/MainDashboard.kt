package com.thesis.garby

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.grid.GridCells
import androidx.compose.foundation.lazy.grid.GridItemSpan
import androidx.compose.foundation.lazy.grid.LazyVerticalGrid
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.outlined.Logout
import androidx.compose.material.icons.filled.GasMeter
import androidx.compose.material.icons.filled.Scale
import androidx.compose.material.icons.outlined.LockReset
import androidx.compose.material3.CenterAlignedTopAppBar
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.viewmodel.compose.viewModel
import com.thesis.garby.libs.SensorData
import com.thesis.garby.libs.SensorGauge
import com.thesis.garby.libs.SensorThresholds
import com.thesis.garby.libs.getCurrentWeight
import com.thesis.garby.libs.getGasLevelStatus
import com.thesis.garby.realtime.ConnectionState
import com.thesis.garby.realtime.DeviceStatus
import com.thesis.garby.realtime.DeviceStatusUiState
import com.thesis.garby.realtime.DeviceUiState
import com.thesis.garby.realtime.DeviceViewModel
import com.thesis.garby.realtime.RtdbConstants
import com.thesis.garby.realtime.SensorReadingUiState
import com.thesis.garby.ui.ErrorPlaceholder
import com.thesis.garby.ui.LoadingPlaceholder
import com.thesis.garby.ui.theme.Montserrat
import kotlin.math.roundToInt

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun MainDashboard(
    onResetTrashbin: () -> Unit = {},
    onSignOut: () -> Unit = {},
    deviceViewModel: DeviceViewModel = viewModel()
) {
    val state by deviceViewModel.uiState.collectAsState()

    Scaffold(
        topBar = {
            CenterAlignedTopAppBar(
                title = {
                    Text(
                        "Garby Monitor",
                        color = Color.White,
                        fontFamily = Montserrat,
                        fontWeight = FontWeight.Black,
                        fontSize = 28.sp
                    )
                },
                navigationIcon = {
                    IconButton(onClick = onSignOut) {
                        Icon(
                            imageVector = Icons.AutoMirrored.Outlined.Logout,
                            contentDescription = "Sign out",
                            modifier = Modifier.size(25.dp),
                            tint = Color.White
                        )
                    }
                },
                actions = {
                    IconButton(onClick = onResetTrashbin) {
                        Icon(
                            imageVector = Icons.Outlined.LockReset,
                            contentDescription = "Reset trash bin",
                            modifier = Modifier.size(25.dp),
                            tint = Color.White
                        )
                    }
                },
                colors = TopAppBarDefaults.topAppBarColors(containerColor = Color.Transparent)
            )
        },
        containerColor = Color.White
    ) { innerPadding ->
        Box(modifier = Modifier.fillMaxSize()) {
            Image(
                painter = painterResource(id = R.drawable.app_background),
                contentDescription = null,
                contentScale = ContentScale.Crop,
                modifier = Modifier.fillMaxSize()
            )

            LazyVerticalGrid(
                columns = GridCells.Adaptive(minSize = 160.dp),
                modifier = Modifier
                    .fillMaxSize()
                    .padding(innerPadding),
                contentPadding = PaddingValues(16.dp),
                verticalArrangement = Arrangement.spacedBy(12.dp),
                horizontalArrangement = Arrangement.spacedBy(12.dp)
            ) {
                item(span = { GridItemSpan(maxLineSpan) }) {
                    ConnectionBanner(state)
                }

                item(span = { GridItemSpan(maxLineSpan) }) {
                    SensorLevelPanel(state.level)
                }

                item(span = { GridItemSpan(maxLineSpan) }) {
                    Text(
                        text = "SENSOR METRICS",
                        fontFamily = Montserrat,
                        fontWeight = FontWeight.Bold,
                        fontSize = 16.sp,
                        color = Color.White,
                        textAlign = TextAlign.Center,
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(top = 8.dp, bottom = 2.dp)
                    )
                }

                item {
                    MetricSensorPanel(
                        state = state.weight,
                        label = "Current Weight",
                        defaultUnit = "kg",
                        icon = MetricIcon.Weight
                    )
                }

                item {
                    MetricSensorPanel(
                        state = state.mq135,
                        label = "Air Quality (MQ135)",
                        defaultUnit = "ppm",
                        icon = MetricIcon.Gas
                    )
                }

                item {
                    MetricSensorPanel(
                        state = state.mq137,
                        label = "Ammonia (MQ137)",
                        defaultUnit = "ppm",
                        icon = MetricIcon.Gas
                    )
                }

                item {
                    MetricSensorPanel(
                        state = state.mq4,
                        label = "Methane (MQ4)",
                        defaultUnit = "ppm",
                        icon = MetricIcon.Gas
                    )
                }
            }
        }
    }
}

@Composable
private fun SensorLevelPanel(state: SensorReadingUiState) {
    val reading = state.reading
    when {
        state.isLoading -> LoadingPlaceholder(
            modifier = Modifier
                .fillMaxWidth()
                .height(170.dp)
        )
        reading != null && reading.value >= 999f -> ErrorPlaceholder(
            message = "Ultrasonic sensor offline",
            modifier = Modifier
                .fillMaxWidth()
                .height(150.dp)
        )
        reading != null -> Box(modifier = Modifier.fillMaxWidth()) {
            SensorGauge(
                modifier = Modifier.fillMaxWidth(),
                data = remember(reading.unit) {
                    SensorData(
                        id = 7,
                        name = "ULTRASONIC DISTANCE",
                        unit = reading.unit.ifBlank { "cm" },
                        maxValue = 300f
                    )
                },
                currentValueProvider = { reading.value },
                sensorThresholds = remember {
                    SensorThresholds(
                        normal = 100f,
                        warning = 200f,
                        severe = 300f
                    )
                },
                animateDuration = 900,
                gaugeSize = 150.dp
            )

            if (state.isStale) {
                Box(
                    modifier = Modifier
                        .align(Alignment.TopEnd)
                        .padding(14.dp)
                        .background(Color(0xFFFFF3CD), RoundedCornerShape(10.dp))
                        .padding(horizontal = 8.dp, vertical = 3.dp)
                ) {
                    Text(
                        text = "STALE",
                        color = Color(0xFF856404),
                        fontFamily = Montserrat,
                        fontWeight = FontWeight.Bold,
                        fontSize = 10.sp
                    )
                }
            }
        }
        state.error != null -> ErrorPlaceholder(
            message = state.error,
            modifier = Modifier
                .fillMaxWidth()
                .height(150.dp)
        )
        else -> ErrorPlaceholder(
            message = "Trash-bin level unavailable",
            modifier = Modifier
                .fillMaxWidth()
                .height(150.dp)
        )
    }
}

private enum class MetricIcon { Weight, Gas }

@Composable
private fun MetricSensorPanel(
    state: SensorReadingUiState,
    label: String,
    defaultUnit: String,
    icon: MetricIcon
) {
    val reading = state.reading
    when {
        state.isLoading -> LoadingPlaceholder(
            modifier = Modifier
                .fillMaxWidth()
                .height(125.dp)
        )
        reading != null -> {
            val status = when (icon) {
                MetricIcon.Weight -> getCurrentWeight(reading.value).let { it.name to it.color }
                MetricIcon.Gas -> getGasLevelStatus(reading.value).let { it.name to it.color }
            }
            MetricCard(
                label = label,
                value = reading.value,
                unit = reading.unit.ifBlank { defaultUnit },
                statusLabel = if (state.isStale) "STALE" else status.first,
                statusColor = if (state.isStale) Color(0xFFFF9800) else status.second,
                icon = icon,
                isStale = state.isStale
            )
        }
        state.error != null -> ErrorPlaceholder(
            message = state.error,
            modifier = Modifier
                .fillMaxWidth()
                .height(125.dp)
        )
        else -> ErrorPlaceholder(
            message = "$label unavailable",
            modifier = Modifier
                .fillMaxWidth()
                .height(125.dp)
        )
    }
}

@Composable
private fun MetricCard(
    label: String,
    value: Float,
    unit: String,
    statusLabel: String,
    statusColor: Color,
    icon: MetricIcon,
    isStale: Boolean = false
) {
    val locale = LocalConfiguration.current.locales[0]
    val valueText = String.format(locale, "%.1f", value)

    Box(
        modifier = Modifier
            .fillMaxWidth()
            .shadow(8.dp, RoundedCornerShape(22.dp))
            .background(Color.White, RoundedCornerShape(22.dp))
            .padding(16.dp)
    ) {
        Column(modifier = Modifier.fillMaxWidth()) {
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.SpaceBetween,
                modifier = Modifier.fillMaxWidth()
            ) {
                Box(
                    modifier = Modifier
                        .size(46.dp)
                        .background(Color(0xFFFFE4EE), CircleShape),
                    contentAlignment = Alignment.Center
                ) {
                    Icon(
                        imageVector = when (icon) {
                            MetricIcon.Weight -> Icons.Filled.Scale
                            MetricIcon.Gas -> Icons.Filled.GasMeter
                        },
                        contentDescription = label,
                        tint = Color(0xFFE91E1E),
                        modifier = Modifier.size(25.dp)
                    )
                }

                Row(verticalAlignment = Alignment.CenterVertically) {
                    Box(
                        modifier = Modifier
                            .size(9.dp)
                            .background(statusColor, CircleShape)
                    )
                    Spacer(modifier = Modifier.width(5.dp))
                    Text(
                        text = statusLabel,
                        fontFamily = Montserrat,
                        fontWeight = FontWeight.Bold,
                        fontSize = 11.sp,
                        color = Color.Black,
                        maxLines = 1
                    )
                }
            }

            Spacer(modifier = Modifier.height(12.dp))

            Text(
                text = label,
                fontFamily = Montserrat,
                fontWeight = FontWeight.Bold,
                fontSize = 12.sp,
                color = Color(0xFFE91E1E),
                maxLines = 1
            )

            Spacer(modifier = Modifier.height(2.dp))

            Row(verticalAlignment = Alignment.Bottom) {
                Text(
                    text = valueText,
                    fontFamily = Montserrat,
                    fontWeight = FontWeight.Black,
                    fontSize = 28.sp,
                    color = Color(0xFF9A1D1D)
                )
                Text(
                    text = " $unit",
                    fontFamily = Montserrat,
                    fontWeight = FontWeight.Bold,
                    fontSize = 13.sp,
                    color = Color(0xFFE91E1E),
                    modifier = Modifier.padding(bottom = 4.dp)
                )
            }
        }
    }
}

@Composable
private fun ConnectionBanner(state: DeviceUiState) {
    val cloud = when (state.connection) {
        ConnectionState.Connected -> Color(0xFF4CAF50) to "CLOUD CONNECTED"
        ConnectionState.Reconnecting -> Color(0xFFFFC107) to "CLOUD RECONNECTING"
        ConnectionState.Disconnected -> Color(0xFFF44336) to "CLOUD OFFLINE"
    }

    Column(
        horizontalAlignment = Alignment.CenterHorizontally,
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp)
    ) {
        StatusLine(color = cloud.first, text = cloud.second)
        Spacer(modifier = Modifier.height(4.dp))
        if (state.device.isLoading && state.device.status == null) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                CircularProgressIndicator(
                    color = Color(0xFFFFC107),
                    strokeWidth = 2.dp,
                    modifier = Modifier.size(12.dp)
                )
                Spacer(modifier = Modifier.width(8.dp))
                Text(
                    text = "LOADING ROBOT STATUS...",
                    fontFamily = Montserrat,
                    fontWeight = FontWeight.Bold,
                    fontSize = 12.sp,
                    color = Color.White
                )
            }
        } else {
            val (garbyColor, garbyText) = formatUptimeStatus(state.device)
            StatusLine(color = garbyColor, text = garbyText)
            state.device.status?.let { status ->
                SystemHealthLines(status = status, stale = state.device.isStale)
            }
        }
    }
}

@Composable
private fun SystemHealthLines(status: DeviceStatus, stale: Boolean) {
    val hasThermalData = status.cpuTemperatureC != null ||
        status.thermalWarning != null || status.throttledFlags != null
    if (hasThermalData) {
        val thermalWarning = status.thermalWarning == true ||
            (status.throttledFlags ?: 0L) != 0L
        val thermalParts = buildList {
            status.cpuTemperatureC?.let { add("PI ${it.roundToInt()}°C") }
            status.thermalWarning?.let { add(if (it) "THERMAL WARNING" else "THERMAL OK") }
            status.throttledFlags?.takeIf { it != 0L }?.let {
                add("FLAGS 0x${it.toString(16).uppercase()}")
            }
        }
        Spacer(modifier = Modifier.height(4.dp))
        StatusLine(
            color = when {
                stale -> Color(0xFFFF9800)
                thermalWarning -> Color(0xFFF44336)
                else -> Color(0xFF4CAF50)
            },
            text = thermalParts.joinToString(" • ")
        )
    }

    val links = listOfNotNull(
        status.bleConnected?.let { "BLE" to it },
        status.lidarHealthy?.let { "LIDAR" to it },
        status.sensorSerialConnected?.let { "SENSORS" to it }
    )
    if (links.isNotEmpty()) {
        Spacer(modifier = Modifier.height(4.dp))
        StatusLine(
            color = when {
                stale -> Color(0xFFFF9800)
                links.any { !it.second } -> Color(0xFFF44336)
                else -> Color(0xFF4CAF50)
            },
            text = links.joinToString(" • ") { (label, healthy) ->
                "$label ${if (healthy) "OK" else "DOWN"}"
            }
        )
    }
}

private fun formatUptimeStatus(deviceState: DeviceStatusUiState): Pair<Color, String> {
    val status = deviceState.status
    if (status == null || deviceState.error != null) {
        return Color(0xFFF44336) to "GARBY OFFLINE"
    }

    val now = System.currentTimeMillis()
    val lastSeenMs = status.lastSeenMs
    val ageMs = (now - lastSeenMs).coerceAtLeast(0L)
    val isOnline = status.online && ageMs <= RtdbConstants.ROBOT_UPTIME_TIMEOUT_MS

    return if (isOnline) {
        Color(0xFF4CAF50) to "GARBY ONLINE"
    } else {
        Color(0xFFF44336) to "GARBY OFFLINE"
    }
}

@Composable
private fun StatusLine(color: Color, text: String) {
    Row(verticalAlignment = Alignment.CenterVertically) {
        Box(
            modifier = Modifier
                .size(10.dp)
                .background(color, CircleShape)
        )
        Spacer(modifier = Modifier.width(8.dp))
        Text(
            text = text,
            fontFamily = Montserrat,
            fontWeight = FontWeight.Bold,
            fontSize = 12.sp,
            color = Color.White
        )
    }
}
