package com.thesis.garby

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.calculateEndPadding
import androidx.compose.foundation.layout.calculateStartPadding
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.lazy.grid.GridCells
import androidx.compose.foundation.lazy.grid.GridItemSpan
import androidx.compose.foundation.lazy.grid.LazyGridScope
import androidx.compose.foundation.lazy.grid.LazyVerticalGrid
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.rounded.KeyboardReturn
import androidx.compose.material.icons.automirrored.rounded.Logout
import androidx.compose.material.icons.rounded.Air
import androidx.compose.material.icons.rounded.Bluetooth
import androidx.compose.material.icons.rounded.Bolt
import androidx.compose.material.icons.rounded.CheckCircle
import androidx.compose.material.icons.rounded.Cloud
import androidx.compose.material.icons.rounded.CloudOff
import androidx.compose.material.icons.rounded.DeleteOutline
import androidx.compose.material.icons.rounded.ErrorOutline
import androidx.compose.material.icons.rounded.HourglassTop
import androidx.compose.material.icons.rounded.Info
import androidx.compose.material.icons.rounded.MonitorHeart
import androidx.compose.material.icons.rounded.Radar
import androidx.compose.material.icons.rounded.Scale
import androidx.compose.material.icons.rounded.Schedule
import androidx.compose.material.icons.rounded.Science
import androidx.compose.material.icons.rounded.Sensors
import androidx.compose.material.icons.rounded.Straighten
import androidx.compose.material.icons.rounded.Thermostat
import androidx.compose.material.icons.rounded.Tune
import androidx.compose.material.icons.rounded.Warning
import androidx.compose.material.icons.rounded.Wifi
import androidx.compose.material.icons.rounded.WifiOff
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.LocalLayoutDirection
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.semantics.stateDescription
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.compose.LocalLifecycleOwner
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.repeatOnLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import com.thesis.garby.realtime.ConnectionState
import com.thesis.garby.realtime.DeviceStatus
import com.thesis.garby.realtime.DeviceStatusUiState
import com.thesis.garby.realtime.DeviceUiState
import com.thesis.garby.realtime.DeviceViewModel
import com.thesis.garby.realtime.disposalBlockReason
import com.thesis.garby.realtime.disposalMessage
import com.thesis.garby.realtime.ExecutorThresholds
import com.thesis.garby.realtime.GasSensor
import com.thesis.garby.realtime.HealthLevel
import com.thesis.garby.realtime.LoadLevel
import com.thesis.garby.realtime.ReturnCommand
import com.thesis.garby.realtime.ReturnProgress
import com.thesis.garby.realtime.ReturnStage
import com.thesis.garby.realtime.ReturnStatus
import com.thesis.garby.realtime.RobotPresence
import com.thesis.garby.realtime.RobotState
import com.thesis.garby.realtime.SensorReading
import com.thesis.garby.realtime.SensorReadingUiState
import com.thesis.garby.realtime.Severity
import com.thesis.garby.realtime.decodeThrottleFlags
import com.thesis.garby.realtime.gasBand
import com.thesis.garby.realtime.gasMatchesDangerRule
import com.thesis.garby.realtime.linkLevel
import com.thesis.garby.realtime.loadLevel
import com.thesis.garby.realtime.piPowerLevel
import com.thesis.garby.realtime.piTemperatureLevel
import com.thesis.garby.ui.SectionIndicator
import com.thesis.garby.ui.SectionIndicatorKind
import com.thesis.garby.ui.airSectionIndicator
import com.thesis.garby.ui.binSectionIndicator
import com.thesis.garby.ui.formatAgo
import com.thesis.garby.ui.formatDecimal
import com.thesis.garby.ui.formatTimeOfDay
import com.thesis.garby.ui.glass.BandBar
import com.thesis.garby.ui.glass.BarBand
import com.thesis.garby.ui.glass.GlassButton
import com.thesis.garby.ui.glass.GlassButtonStyle
import com.thesis.garby.ui.glass.GlassDialog
import com.thesis.garby.ui.glass.GlassDock
import com.thesis.garby.ui.glass.GlassIconButton
import com.thesis.garby.ui.glass.GlassPanel
import com.thesis.garby.ui.glass.GlassScaffold
import com.thesis.garby.ui.glass.GlassShapes
import com.thesis.garby.ui.glass.GlassTier
import com.thesis.garby.ui.glass.GlassTopBar
import com.thesis.garby.ui.glass.IconOrb
import com.thesis.garby.ui.glass.LogoBadge
import com.thesis.garby.ui.glass.RoutePosition
import com.thesis.garby.ui.glass.RouteTrack
import com.thesis.garby.ui.glass.SectionHeader
import com.thesis.garby.ui.glass.ShimmerPanel
import com.thesis.garby.ui.glass.StatusDot
import com.thesis.garby.ui.glass.StatusPill
import com.thesis.garby.ui.healthColor
import com.thesis.garby.ui.returnStageUi
import com.thesis.garby.ui.robotStateUi
import com.thesis.garby.ui.severityColor
import com.thesis.garby.ui.systemSectionIndicator
import com.thesis.garby.ui.theme.GarbyColors
import com.thesis.garby.ui.theme.GarbyTheme
import java.util.Locale

@Composable
fun MainDashboard(
    returnViewModel: ReturnViewModel,
    onOpenReturn: () -> Unit,
    onSignOut: () -> Unit,
    deviceViewModel: DeviceViewModel = viewModel()
) {
    val lifecycleOwner = LocalLifecycleOwner.current
    LaunchedEffect(lifecycleOwner, deviceViewModel) {
        lifecycleOwner.lifecycle.repeatOnLifecycle(Lifecycle.State.STARTED) {
            deviceViewModel.observeWhileVisible()
        }
    }
    val state by deviceViewModel.uiState.collectAsStateWithLifecycle()
    val summary by returnViewModel.summary.collectAsStateWithLifecycle()
    var confirmSignOut by rememberSaveable { mutableStateOf(false) }

    DashboardContent(
        state = state,
        summary = summary,
        onOpenReturn = onOpenReturn,
        onSignOut = { confirmSignOut = true },
        onManualDisposal = deviceViewModel::requestDisposal,
        onDismissDisposalNotice = deviceViewModel::clearDisposalNotice
    )

    if (confirmSignOut) {
        SignOutDialog(
            onConfirm = {
                confirmSignOut = false
                onSignOut()
            },
            onDismiss = { confirmSignOut = false }
        )
    }
}

@Composable
fun DashboardContent(
    state: DeviceUiState,
    summary: ReturnSummary,
    onOpenReturn: () -> Unit,
    onSignOut: () -> Unit,
    onManualDisposal: () -> Unit = {},
    onDismissDisposalNotice: () -> Unit = {}
) {
    var confirmDisposal by rememberSaveable { mutableStateOf(false) }
    val disposalBlocked = disposalBlockReason(state.device.status, state.cloudConnected,
        state.internetAvailable, state.disposal, state.nowMs)
    val robotState = state.device.status?.robotState
    val systemTiles = healthTiles(state)
    val (subtitle, subtitleColor) = cloudSubtitle(state)
    GlassScaffold(
        topBar = {
            GlassTopBar(
                title = "GARBY",
                subtitle = subtitle,
                subtitleColor = subtitleColor,
                navigationIcon = {
                    LogoBadge(size = 44.dp, ring = 2.dp, modifier = Modifier.padding(start = 2.dp))
                },
                actions = {
                    GlassIconButton(
                        icon = Icons.AutoMirrored.Rounded.Logout,
                        contentDescription = "Sign out",
                        onClick = onSignOut
                    )
                }
            )
        },
        bottomBar = {
            ReturnDock(summary = summary, robotState = robotState, onOpenReturn = onOpenReturn)
        }
    ) { padding ->
        val layoutDirection = LocalLayoutDirection.current
        val cardScale = LocalDensity.current.fontScale.coerceAtLeast(1f)
        LazyVerticalGrid(
            columns = GridCells.Adaptive(minSize = 168.dp * cardScale),
            modifier = Modifier.fillMaxSize(),
            contentPadding = PaddingValues(
                start = padding.calculateStartPadding(layoutDirection) + 16.dp,
                end = padding.calculateEndPadding(layoutDirection) + 16.dp,
                top = padding.calculateTopPadding() + 4.dp,
                bottom = padding.calculateBottomPadding() + 20.dp
            ),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            connectivityIssue(state)?.let { issue ->
                fullWidth("connectivity") { ConnectivityBanner(issue) }
            }
            fullWidth("hero") { RobotHeroCard(state = state, summary = summary) }
            fullWidth("manual-disposal") {
                ManualDisposalCard(state, disposalBlocked,
                    onStart = { confirmDisposal = true },
                    onDismissNotice = onDismissDisposalNotice)
            }

            fullWidth("bin-header", contentType = "section-header") {
                SectionHeader(
                    title = "Bin",
                    icon = Icons.Rounded.DeleteOutline,
                    accent = GarbyColors.Rose,
                    caption = "Weight and distance · 4\u00A0kg trigger",
                    trailing = { SectionStatus(binSectionIndicator(state)) }
                )
            }
            item(key = "weight", contentType = "weight") { WeightCard(state.weight, robotState, state.nowMs) }
            item(key = "level", contentType = "bin-distance") { BinDistanceCard(state.level, state.nowMs) }

            fullWidth("air-header", contentType = "section-header") {
                SectionHeader(
                    title = "Air",
                    icon = Icons.Rounded.Air,
                    accent = GarbyColors.Info,
                    caption = "Three gas sensors · Threshold bands",
                    trailing = { SectionStatus(airSectionIndicator(state)) }
                )
            }
            item(key = "mq135", contentType = "gas") { GasCard(GasSensor.Mq135, state.mq135, state.nowMs) }
            item(key = "mq137", contentType = "gas") { GasCard(GasSensor.Mq137, state.mq137, state.nowMs) }
            item(key = "mq4", contentType = "gas") { GasCard(GasSensor.Mq4, state.mq4, state.nowMs) }
            item(key = "gas-rule", contentType = "gas-rule") { GasRuleCard(state) }

            fullWidth("system-header", contentType = "section-header") {
                SectionHeader(
                    title = "System",
                    icon = Icons.Rounded.Tune,
                    accent = GarbyColors.Neutral,
                    caption = "Connections · Pi health · 30\u00A0s updates",
                    trailing = { SectionStatus(systemSectionIndicator(systemTiles.map { it.level })) }
                )
            }
            systemTiles.forEach { tile ->
                item(key = "health-${tile.title}", contentType = "health") { HealthTile(tile) }
            }

            fullWidth("footer") { Footer() }
        }
    }
    if (confirmDisposal) {
        GlassDialog(onDismissRequest = { confirmDisposal = false }) {
            Column(Modifier.heightIn(max = 420.dp).verticalScroll(rememberScrollState()),
                verticalArrangement = Arrangement.spacedBy(14.dp)) {
                Text("Start manual disposal?", style = MaterialTheme.typography.titleLarge,
                    color = GarbyColors.TextPrimary)
                Text("Are you sure you want GARBY to go to the collection point now? " +
                    "Bin level, gas and weight thresholds do not need to be reached. " +
                    "Obstacle detection and emergency stop remain active.",
                    style = MaterialTheme.typography.bodyMedium, color = GarbyColors.TextSecondary)
                if (disposalBlocked != null) Text(disposalBlocked,
                    style = MaterialTheme.typography.bodySmall, color = GarbyColors.Caution)
                GlassButton("Yes, start disposal", onClick = {
                    confirmDisposal = false
                    onManualDisposal()
                }, enabled = disposalBlocked == null, modifier = Modifier.fillMaxWidth())
                GlassButton("Cancel", onClick = { confirmDisposal = false },
                    style = GlassButtonStyle.Secondary, modifier = Modifier.fillMaxWidth())
            }
        }
    }
}

@Composable
private fun ManualDisposalCard(
    state: DeviceUiState,
    blocked: String?,
    onStart: () -> Unit,
    onDismissNotice: () -> Unit
) {
    GlassPanel(modifier = Modifier.fillMaxWidth()) {
        Text("Manual disposal", style = MaterialTheme.typography.titleMedium,
            color = GarbyColors.TextPrimary)
        Text("Empty the bin after school, even before its thresholds are reached.",
            style = MaterialTheme.typography.bodySmall, color = GarbyColors.TextSecondary)
        val message = if (state.disposal.submitting || state.disposal.notice != null) null
            else disposalMessage(state.disposal.command, state.nowMs)
        if (message != null) Text("Latest request: $message", style = MaterialTheme.typography.bodySmall,
            color = GarbyColors.TextSecondary, modifier = Modifier.padding(top = 8.dp))
        state.disposal.notice?.let { notice ->
            Text(notice, style = MaterialTheme.typography.bodySmall, color = GarbyColors.Caution,
                modifier = Modifier.padding(top = 8.dp))
            GlassButton("Dismiss message", onClick = onDismissNotice,
                style = GlassButtonStyle.Secondary, modifier = Modifier.fillMaxWidth())
        }
        if (blocked != null) Text(blocked, style = MaterialTheme.typography.bodySmall,
            color = GarbyColors.TextMuted, modifier = Modifier.padding(top = 8.dp))
        GlassButton("Manual disposal", onClick = onStart, enabled = blocked == null,
            loading = state.disposal.submitting, icon = Icons.Rounded.DeleteOutline,
            modifier = Modifier.fillMaxWidth().padding(top = 12.dp))
    }
}

private fun LazyGridScope.fullWidth(key: String, contentType: String = key, content: @Composable () -> Unit) {
    item(key = key, contentType = contentType, span = { GridItemSpan(maxLineSpan) }) { content() }
}

@Composable
private fun SectionStatus(indicator: SectionIndicator) {
    val color = indicator.severity?.let(::severityColor) ?: GarbyColors.Neutral
    val icon = when (indicator.severity) {
        Severity.Normal -> Icons.Rounded.CheckCircle
        Severity.Elevated -> Icons.Rounded.Info
        Severity.High, Severity.Danger -> Icons.Rounded.Warning
        null -> when (indicator.kind) {
            SectionIndicatorKind.Loading -> Icons.Rounded.HourglassTop
            SectionIndicatorKind.Stale -> Icons.Rounded.Schedule
            SectionIndicatorKind.Unavailable -> Icons.Rounded.ErrorOutline
            SectionIndicatorKind.Summary -> Icons.Rounded.Info
        }
    }
    StatusPill(indicator.label, color, icon = icon)
}

private fun cloudSubtitle(state: DeviceUiState): Pair<String, Color> = when {
    !state.internetAvailable -> "Phone offline" to GarbyColors.Danger
    state.connection is ConnectionState.Connected -> "Cloud connected" to GarbyColors.Ok
    state.connection is ConnectionState.Reconnecting -> "Connecting to the cloud…" to GarbyColors.Caution
    else -> "Cloud unavailable" to GarbyColors.Danger
}

private data class ConnectivityIssue(
    val icon: ImageVector,
    val title: String,
    val text: String,
    val color: Color
)

private fun connectivityIssue(state: DeviceUiState): ConnectivityIssue? = when {
    !state.internetAvailable -> ConnectivityIssue(
        Icons.Rounded.WifiOff,
        "This phone is offline",
        "Everything below is the last data received and is marked stale.",
        GarbyColors.Danger
    )
    state.connection is ConnectionState.Disconnected -> ConnectivityIssue(
        Icons.Rounded.CloudOff,
        "Cloud connection failed",
        "Firebase refused or dropped the connection. Check this app's access, then reopen it.",
        GarbyColors.Danger
    )
    else -> null
}

@Composable
private fun ConnectivityBanner(issue: ConnectivityIssue) {
    GlassPanel(
        modifier = Modifier.fillMaxWidth(),
        glow = issue.color,
        contentPadding = PaddingValues(16.dp)
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            IconOrb(issue.icon, issue.color)
            Spacer(Modifier.width(12.dp))
            Column(Modifier.weight(1f)) {
                Text(issue.title, style = MaterialTheme.typography.titleSmall, color = GarbyColors.TextPrimary)
                Text(issue.text, style = MaterialTheme.typography.bodySmall, color = GarbyColors.TextSecondary)
            }
        }
    }
}

@Composable
private fun RobotHeroCard(state: DeviceUiState, summary: ReturnSummary) {
    val device = state.device
    if (device.isLoading && device.status == null) {
        ShimmerPanel(label = "Loading robot status", modifier = Modifier.fillMaxWidth())
        return
    }
    val status = device.status
    val presence = state.presence
    val ui = robotStateUi(status?.robotState, status?.isReadyToReturn)
    val live = presence == RobotPresence.Live && state.cloudConnected
    val accent = if (presence == RobotPresence.NeverSeen) GarbyColors.Neutral else ui.color

    GlassPanel(
        modifier = Modifier.fillMaxWidth(),
        shape = GlassShapes.Hero,
        glow = if (live) accent else null,
        contentPadding = PaddingValues(20.dp)
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            IconOrb(ui.icon, accent, size = 44.dp)
            Spacer(Modifier.width(12.dp))
            Column(Modifier.weight(1f)) {
                Text("ROBOT", style = MaterialTheme.typography.labelSmall, color = GarbyColors.TextMuted)
                Text(
                    text = presenceDetail(presence, status, state.nowMs),
                    style = MaterialTheme.typography.bodySmall,
                    color = GarbyColors.TextSecondary,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis
                )
            }
            PresencePill(presence, state.cloudConnected)
        }
        Spacer(Modifier.height(16.dp))
        if (presence == RobotPresence.NeverSeen) {
            Text("No reports yet", style = MaterialTheme.typography.headlineSmall, color = GarbyColors.TextPrimary)
            Spacer(Modifier.height(6.dp))
            Text(
                "GARBY's Pi has not published a status to the cloud. Check that it is powered, " +
                    "online and running the GARBY service.",
                style = MaterialTheme.typography.bodyMedium,
                color = GarbyColors.TextSecondary
            )
        } else {
            Text(
                text = if (live) ui.title else "Last known: ${ui.title}",
                style = MaterialTheme.typography.headlineSmall,
                color = GarbyColors.TextPrimary
            )
            Spacer(Modifier.height(6.dp))
            Text(ui.description, style = MaterialTheme.typography.bodyMedium, color = GarbyColors.TextSecondary)
            if (!live) {
                Spacer(Modifier.height(8.dp))
                Text(
                    "Not live: GARBY may have moved on since this report.",
                    style = MaterialTheme.typography.bodySmall,
                    color = GarbyColors.Caution
                )
            }
        }
        Spacer(Modifier.height(18.dp))
        RouteTrack(
            position = if (presence == RobotPresence.NeverSeen) RoutePosition.Unknown else ui.route,
            accent = accent,
            animate = live && ui.moving,
            description = "Route position: ${ui.title}"
        )
        summary.progress?.let { progress ->
            Spacer(Modifier.height(16.dp))
            ReturnRequestLine(progress, state.nowMs)
        }
    }
}

private fun presenceDetail(presence: RobotPresence, status: DeviceStatus?, nowMs: Long): String {
    val lastSeen = status?.lastSeenMs ?: 0L
    return when (presence) {
        RobotPresence.Live -> "Updated ${formatAgo(nowMs - lastSeen)}"
        RobotPresence.Stale -> "Last report ${formatAgo(nowMs - lastSeen)}"
        RobotPresence.NeverSeen -> "Never reported"
    }
}

@Composable
private fun PresencePill(presence: RobotPresence, cloudConnected: Boolean) {
    when {
        presence == RobotPresence.Live && cloudConnected -> StatusPill("Live", GarbyColors.Ok)
        presence == RobotPresence.Live -> StatusPill("Cached", GarbyColors.Caution)
        presence == RobotPresence.Stale -> StatusPill("Offline", GarbyColors.Danger)
        else -> StatusPill("No data", GarbyColors.Neutral)
    }
}

@Composable
private fun ReturnRequestLine(progress: ReturnProgress, nowMs: Long) {
    val stage = returnStageUi(progress.stage)
    val locale = LocalConfiguration.current.locales[0]
    val time = formatTimeOfDay(progress.command.requestedAtMs, nowMs, locale)
    GlassPanel(
        tier = GlassTier.Clear,
        shape = GlassShapes.Inner,
        contentPadding = PaddingValues(horizontal = 14.dp, vertical = 12.dp),
        modifier = Modifier.fillMaxWidth()
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Icon(
                Icons.AutoMirrored.Rounded.KeyboardReturn,
                contentDescription = null,
                tint = GarbyColors.TextSecondary,
                modifier = Modifier.size(18.dp)
            )
            Spacer(Modifier.width(8.dp))
            Text(
                "Return request",
                style = MaterialTheme.typography.labelMedium,
                color = GarbyColors.TextPrimary,
                modifier = Modifier.weight(1f)
            )
            StatusPill(stage.label, stage.color)
        }
        Spacer(Modifier.height(6.dp))
        Text(
            text = "Sent $time" + if (progress.ownRequest) " from this phone" else "",
            style = MaterialTheme.typography.bodySmall,
            color = GarbyColors.TextMuted
        )
    }
}

@Composable
private fun ReturnDock(summary: ReturnSummary, robotState: RobotState?, onOpenReturn: () -> Unit) {
    val progress = summary.progress
    GlassDock {
        when {
            progress?.inFlight == true -> {
                val stage = returnStageUi(progress.stage)
                Row(verticalAlignment = Alignment.CenterVertically, modifier = Modifier.padding(horizontal = 6.dp)) {
                    StatusDot(stage.color)
                    Spacer(Modifier.width(8.dp))
                    Text(
                        text = if (progress.stage == ReturnStage.Returning) {
                            "GARBY accepted the return and is driving back."
                        } else {
                            "Return sent. Waiting for GARBY to accept it."
                        },
                        style = MaterialTheme.typography.bodySmall,
                        color = GarbyColors.TextSecondary
                    )
                }
                GlassButton(
                    text = "Track return",
                    onClick = onOpenReturn,
                    style = GlassButtonStyle.Secondary,
                    icon = Icons.AutoMirrored.Rounded.KeyboardReturn,
                    modifier = Modifier.fillMaxWidth()
                )
            }
            summary.canSend -> {
                Text(
                    text = "GARBY is waiting at the collection point.",
                    style = MaterialTheme.typography.bodySmall,
                    color = GarbyColors.TextSecondary,
                    modifier = Modifier.padding(horizontal = 6.dp)
                )
                GlassButton(
                    text = "Send back to base",
                    onClick = onOpenReturn,
                    icon = Icons.AutoMirrored.Rounded.KeyboardReturn,
                    modifier = Modifier.fillMaxWidth()
                )
            }
            else -> GlassButton(
                text = if (robotState == RobotState.AtDestination) "Return to base (checks pending)" else "Return to base",
                onClick = onOpenReturn,
                style = GlassButtonStyle.Secondary,
                icon = Icons.AutoMirrored.Rounded.KeyboardReturn,
                modifier = Modifier.fillMaxWidth()
            )
        }
    }
}

/** Shared frame of the half-width metric cards, so rows line up. */
@Composable
private fun MetricCard(
    icon: ImageVector,
    title: String,
    accent: Color,
    pill: @Composable () -> Unit,
    footnote: String,
    updated: String? = null,
    content: @Composable ColumnScope.() -> Unit
) {
    GlassPanel(
        modifier = Modifier
            .fillMaxWidth()
            .heightIn(min = METRIC_CARD_MIN_HEIGHT),
        verticalArrangement = Arrangement.SpaceBetween
    ) {
        Column {
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                IconOrb(icon, accent, size = 28.dp)
                Text(
                    text = title,
                    style = MaterialTheme.typography.labelMedium,
                    color = GarbyColors.TextSecondary,
                    maxLines = 2,
                    overflow = TextOverflow.Ellipsis,
                    modifier = Modifier.weight(1f)
                )
            }
            Spacer(Modifier.height(10.dp))
            content()
            Spacer(Modifier.height(12.dp))
            pill()
        }
        Column(Modifier.padding(top = 10.dp)) {
            Text(
                text = footnote,
                style = MaterialTheme.typography.bodySmall,
                color = GarbyColors.TextMuted,
                maxLines = 3,
                overflow = TextOverflow.Ellipsis
            )
            if (updated != null) {
                Text(
                    text = updated,
                    style = MaterialTheme.typography.labelSmall,
                    color = GarbyColors.TextSecondary,
                    modifier = Modifier.padding(top = 4.dp)
                )
            }
        }
    }
}

private val METRIC_CARD_MIN_HEIGHT = 264.dp

@Composable
private fun ValueLine(value: String, unit: String) {
    Row(verticalAlignment = Alignment.Bottom) {
        Text(value, style = MaterialTheme.typography.headlineMedium, color = GarbyColors.TextPrimary)
        Spacer(Modifier.width(4.dp))
        Text(
            unit,
            style = MaterialTheme.typography.labelLarge,
            color = GarbyColors.TextSecondary,
            modifier = Modifier.padding(bottom = 4.dp)
        )
    }
}

/** Loading, missing or sentinel readings, with the same footprint as a live card. */
@Composable
private fun UnavailableCard(icon: ImageVector, title: String, message: String) {
    MetricCard(
        icon = icon,
        title = title,
        accent = GarbyColors.Neutral,
        pill = { StatusPill("No data", GarbyColors.Neutral, icon = Icons.Rounded.ErrorOutline) },
        footnote = message
    ) {
        Text("—", style = MaterialTheme.typography.headlineMedium, color = GarbyColors.TextMuted)
    }
}

private fun updatedText(reading: SensorReading, nowMs: Long): String =
    if (reading.updatedAtMs > 0L) "Updated ${formatAgo(nowMs - reading.updatedAtMs)}" else "No timestamp"

@Composable
private fun WeightCard(state: SensorReadingUiState, robotState: RobotState?, nowMs: Long) {
    val reading = state.reading
    val title = "Bin weight"
    when {
        state.isLoading -> ShimmerPanel(label = "Loading bin weight", modifier = Modifier.heightIn(min = METRIC_CARD_MIN_HEIGHT))
        reading == null -> UnavailableCard(Icons.Rounded.Scale, title, state.error ?: "No weight reported yet.")
        reading.value < 0f -> UnavailableCard(
            Icons.Rounded.Scale,
            title,
            "The load cell is not ready (not tared yet, or its readings are invalid)."
        )
        else -> {
            val locale = LocalConfiguration.current.locales[0]
            val level = loadLevel(reading.value)
            val (label, color) = when (level) {
                LoadLevel.Light, null -> "Light" to GarbyColors.Ok
                LoadLevel.Filling -> "Filling up" to GarbyColors.Caution
                LoadLevel.AtTrigger -> "Full" to GarbyColors.Danger
                LoadLevel.OutOfRange -> "Out of range" to GarbyColors.Danger
            }
            val idle = robotState == null || robotState == RobotState.Idle
            MetricCard(
                icon = Icons.Rounded.Scale,
                title = title,
                accent = if (state.isStale) GarbyColors.Neutral else color,
                pill = { FreshnessPill(state.isStale, label, color) },
                footnote = if (idle || !state.isStale) {
                    "Starts a run at ${formatDecimal(ExecutorThresholds.LOAD_TRIGGER_KG, 1, locale)} kg."
                } else {
                    "Reported only while GARBY is idle at base."
                },
                updated = updatedText(reading, nowMs)
            ) {
                ValueLine(formatDecimal(reading.value, 2, locale), "kg")
                Spacer(Modifier.height(10.dp))
                BandBar(
                    value = reading.value,
                    maxValue = ExecutorThresholds.LOAD_TRIGGER_KG * 1.25f,
                    bands = listOf(
                        BarBand(ExecutorThresholds.LOAD_CAUTION_KG, GarbyColors.Ok),
                        BarBand(ExecutorThresholds.LOAD_TRIGGER_KG, GarbyColors.Caution),
                        BarBand(ExecutorThresholds.LOAD_TRIGGER_KG * 1.25f, GarbyColors.Danger)
                    )
                )
            }
        }
    }
}

@Composable
private fun BinDistanceCard(state: SensorReadingUiState, nowMs: Long) {
    val reading = state.reading
    val title = "Bin distance"
    when {
        state.isLoading -> ShimmerPanel(label = "Loading bin distance", modifier = Modifier.heightIn(min = METRIC_CARD_MIN_HEIGHT))
        reading == null -> UnavailableCard(Icons.Rounded.Straighten, title, state.error ?: "No bin reading yet.")
        reading.value >= 999f -> UnavailableCard(
            Icons.Rounded.Straighten,
            title,
            "The bin's ultrasonic sensor is offline."
        )
        else -> {
            val locale = LocalConfiguration.current.locales[0]
            MetricCard(
                icon = Icons.Rounded.Straighten,
                title = title,
                accent = if (state.isStale) GarbyColors.Neutral else GarbyColors.Info,
                pill = { FreshnessPill(state.isStale, "Live", GarbyColors.Info) },
                footnote = "Ultrasonic reading from the bin sensor board.",
                updated = updatedText(reading, nowMs)
            ) {
                ValueLine(formatDecimal(reading.value, 1, locale), reading.unit.ifBlank { "cm" })
                Spacer(Modifier.height(10.dp))
                BandBar(
                    value = reading.value,
                    maxValue = 300f,
                    bands = listOf(BarBand(300f, GarbyColors.Info))
                )
            }
        }
    }
}

private const val GAS_SCALE_PPM = 1_000f

@Composable
private fun GasCard(sensor: GasSensor, state: SensorReadingUiState, nowMs: Long) {
    val reading = state.reading
    val title = sensor.key.displayName
    val icon = if (sensor == GasSensor.Mq135) Icons.Rounded.Air else Icons.Rounded.Science
    when {
        state.isLoading -> ShimmerPanel(label = "Loading $title", modifier = Modifier.heightIn(min = METRIC_CARD_MIN_HEIGHT))
        reading == null -> UnavailableCard(icon, title, state.error ?: "No reading yet.")
        reading.value < 0f -> UnavailableCard(icon, title, "This gas sensor is unavailable.")
        else -> {
            val locale = LocalConfiguration.current.locales[0]
            val band = gasBand(sensor, reading.value)
            val color = band?.let { severityColor(it.severity) } ?: GarbyColors.Neutral
            MetricCard(
                icon = icon,
                title = title,
                accent = if (state.isStale) GarbyColors.Neutral else color,
                pill = { FreshnessPill(state.isStale, band?.label ?: "Unknown", color) },
                footnote = gasThresholdText(sensor),
                updated = updatedText(reading, nowMs)
            ) {
                ValueLine(formatDecimal(reading.value, 0, locale), reading.unit.ifBlank { "ppm" })
                Spacer(Modifier.height(10.dp))
                BandBar(
                    value = reading.value,
                    maxValue = GAS_SCALE_PPM,
                    bands = sensor.bands.map { gasBandSegment ->
                        BarBand(gasBandSegment.upperPpm ?: GAS_SCALE_PPM, severityColor(gasBandSegment.severity))
                    }
                )
            }
        }
    }
}

private fun gasThresholdText(sensor: GasSensor): String = sensor.bands
    .zipWithNext { lower, upper -> "${upper.label} ${lower.upperPpm?.toInt()}" }
    .joinToString(" · ", postfix = " ppm.")

/** Whether the current readings meet the executor's danger rule for one packet. */
@Composable
private fun GasRuleCard(state: DeviceUiState) {
    val readings = listOf(state.mq4, state.mq135, state.mq137)
    val allLive = readings.all { reading ->
        val value = reading.reading?.value
        value != null && value >= 0f && !reading.isStale
    }
    val met = allLive && gasMatchesDangerRule(
        mq4 = state.mq4.reading?.value,
        mq135 = state.mq135.reading?.value,
        mq137 = state.mq137.reading?.value
    )
    val (pill, color, headline) = when {
        !allLive -> Triple("No data", GarbyColors.Neutral, "Waiting for live readings")
        met -> Triple("Met", GarbyColors.Danger, "Danger rule met")
        else -> Triple("Not met", GarbyColors.Ok, "Below the danger rule")
    }
    MetricCard(
        icon = Icons.Rounded.Warning,
        title = "Gas trigger",
        accent = color,
        pill = { StatusPill(pill, color) },
        footnote = "${ExecutorThresholds.GAS_DANGER_PPM.toInt()} ppm on any sensor, or all three in " +
            "their second band.",
        updated = "${ExecutorThresholds.GAS_CONFIRMATIONS} in a row starts a run while idle"
    ) {
        Text(
            headline,
            style = MaterialTheme.typography.titleMedium,
            color = GarbyColors.TextPrimary,
            modifier = Modifier.semantics { stateDescription = pill }
        )
    }
}

@Composable
private fun FreshnessPill(stale: Boolean, label: String, color: Color) {
    if (stale) {
        StatusPill("Stale", GarbyColors.Neutral, icon = Icons.Rounded.Schedule)
    } else {
        StatusPill(label, color)
    }
}

private data class HealthTileUi(
    val icon: ImageVector,
    val title: String,
    val value: String,
    val level: HealthLevel
)

private fun healthTiles(state: DeviceUiState): List<HealthTileUi> {
    val status = state.device.status
    val presence = state.presence
    val live = presence == RobotPresence.Live
    // Pi-reported values describe the moment of the last report; when that is old, say so.
    fun piReported(value: String, level: HealthLevel): Pair<String, HealthLevel> =
        if (live) value to level else "Last known: $value" to HealthLevel.Unknown

    val heartbeat = when (presence) {
        RobotPresence.Live -> "Seen ${formatAgo(state.nowMs - (status?.lastSeenMs ?: 0L))}" to HealthLevel.Ok
        RobotPresence.Stale -> "Silent ${formatAgo(state.nowMs - (status?.lastSeenMs ?: 0L)).removeSuffix(" ago")}" to HealthLevel.Fault
        RobotPresence.NeverSeen -> "No reports" to HealthLevel.Unknown
    }
    val cloud = when {
        state.connection is ConnectionState.Connected -> "Connected" to HealthLevel.Ok
        state.connection is ConnectionState.Reconnecting -> "Connecting…" to HealthLevel.Caution
        else -> "Unavailable" to HealthLevel.Fault
    }
    val ble = piReported(
        when (status?.bleConnected) { true -> "Connected"; false -> "Down"; null -> "Not reported" },
        linkLevel(status?.bleConnected)
    )
    val lidar = piReported(
        when (status?.lidarHealthy) { true -> "Scanning"; false -> "No scans"; null -> "Not reported" },
        linkLevel(status?.lidarHealthy)
    )
    val sensors = piReported(
        when (status?.sensorSerialConnected) { true -> "Reporting"; false -> "Silent"; null -> "Not reported" },
        linkLevel(status?.sensorSerialConnected)
    )
    val temperature = piReported(
        status?.cpuTemperatureC?.let { "${formatDecimal(it, 0, Locale.ROOT)} °C" } ?: "Not reported",
        piTemperatureLevel(status?.cpuTemperatureC)
    )
    val flags = status?.throttledFlags
    val conditions = decodeThrottleFlags(flags)
    val power = piReported(
        when {
            flags == null -> "Not reported"
            flags == 0L -> "Normal"
            else -> (conditions.firstOrNull { it.activeNow } ?: conditions.firstOrNull())?.label
                ?: "Flags 0x${flags.toString(16)}"
        },
        piPowerLevel(flags)
    )

    return listOf(
        HealthTileUi(Icons.Rounded.MonitorHeart, "Pi heartbeat", heartbeat.first, heartbeat.second),
        HealthTileUi(Icons.Rounded.Cloud, "Cloud", cloud.first, cloud.second),
        HealthTileUi(
            if (state.internetAvailable) Icons.Rounded.Wifi else Icons.Rounded.WifiOff,
            "Phone internet",
            if (state.internetAvailable) "Online" else "Offline",
            if (state.internetAvailable) HealthLevel.Ok else HealthLevel.Fault
        ),
        HealthTileUi(Icons.Rounded.Bluetooth, "Robot link", ble.first, ble.second),
        HealthTileUi(Icons.Rounded.Radar, "LiDAR", lidar.first, lidar.second),
        HealthTileUi(Icons.Rounded.Sensors, "Sensor board", sensors.first, sensors.second),
        HealthTileUi(Icons.Rounded.Thermostat, "Pi temperature", temperature.first, temperature.second),
        HealthTileUi(Icons.Rounded.Bolt, "Pi power", power.first, power.second)
    )
}

@Composable
private fun HealthTile(tile: HealthTileUi) {
    val color = healthColor(tile.level)
    GlassPanel(
        modifier = Modifier
            .fillMaxWidth()
            .semantics { stateDescription = tile.value },
        contentPadding = PaddingValues(14.dp)
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            IconOrb(tile.icon, color, size = 32.dp)
            Spacer(Modifier.weight(1f))
            Icon(
                imageVector = when (tile.level) {
                    HealthLevel.Ok -> Icons.Rounded.CheckCircle
                    HealthLevel.Caution -> Icons.Rounded.Info
                    HealthLevel.Fault -> Icons.Rounded.ErrorOutline
                    HealthLevel.Unknown -> Icons.Rounded.Schedule
                },
                contentDescription = null,
                tint = color,
                modifier = Modifier.size(18.dp)
            )
        }
        Spacer(Modifier.height(10.dp))
        Text(tile.title, style = MaterialTheme.typography.labelMedium, color = GarbyColors.TextMuted, maxLines = 1)
        Text(
            tile.value,
            style = MaterialTheme.typography.titleSmall,
            color = GarbyColors.TextPrimary,
            minLines = 2,
            maxLines = 2,
            overflow = TextOverflow.Ellipsis
        )
    }
}

@Composable
private fun Footer() {
    Text(
        text = "This app watches GARBY and can ask it to return to base. It can never make GARBY move " +
            "unsafely: the robot's own sensors decide every movement, and the Pi monitor's emergency " +
            "stop always wins.",
        style = MaterialTheme.typography.bodySmall,
        color = GarbyColors.TextMuted,
        textAlign = TextAlign.Center,
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 12.dp, vertical = 8.dp)
    )
}

@Composable
private fun SignOutDialog(onConfirm: () -> Unit, onDismiss: () -> Unit) {
    GlassDialog(onDismissRequest = onDismiss) {
        Text("Sign out?", style = MaterialTheme.typography.headlineSmall, color = GarbyColors.TextPrimary)
        Spacer(Modifier.height(10.dp))
        Text(
            "You will need to connect again to watch GARBY. A return request that was already sent " +
                "is not affected.",
            style = MaterialTheme.typography.bodyMedium,
            color = GarbyColors.TextSecondary
        )
        Spacer(Modifier.height(22.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            GlassButton(
                text = "Cancel",
                onClick = onDismiss,
                style = GlassButtonStyle.Secondary,
                modifier = Modifier.weight(1f)
            )
            GlassButton(text = "Sign out", onClick = onConfirm, modifier = Modifier.weight(1f))
        }
    }
}

private const val PREVIEW_NOW = 1_763_000_000_000L

private fun previewReading(value: Float, unit: String, ageMs: Long = 6_000L) = SensorReadingUiState(
    reading = SensorReading(value, unit, "", PREVIEW_NOW - ageMs),
    isLoading = false,
    isStale = false
)

internal fun previewDeviceState(robotState: RobotState = RobotState.AtDestination) = DeviceUiState(
    connection = ConnectionState.Connected,
    internetAvailable = true,
    level = previewReading(48.2f, "cm"),
    weight = previewReading(2.35f, "kg", ageMs = 25_000L),
    mq135 = previewReading(412f, "ppm"),
    mq137 = previewReading(128f, "ppm"),
    mq4 = previewReading(236f, "ppm"),
    device = DeviceStatusUiState(
        status = DeviceStatus(
            lastSeenMs = PREVIEW_NOW - 12_000L,
            robotState = robotState,
            isReadyToReturn = robotState == RobotState.AtDestination,
            cpuTemperatureC = 54.3f,
            thermalWarning = false,
            throttledFlags = 0x50000L,
            bleConnected = true,
            lidarHealthy = true,
            sensorSerialConnected = true
        ),
        isLoading = false,
        isStale = false
    ),
    nowMs = PREVIEW_NOW
)

@Preview(name = "Dashboard", widthDp = 400, heightDp = 1500, showBackground = true, backgroundColor = 0xFF0B0305)
@Composable
private fun DashboardPreview() {
    GarbyTheme {
        DashboardContent(
            state = previewDeviceState(),
            summary = ReturnSummary(
                progress = ReturnProgress(
                    command = ReturnCommand(PREVIEW_NOW - 3_600_000L, "op", ReturnStatus.Done, doneAtMs = PREVIEW_NOW - 3_000_000L),
                    stage = ReturnStage.BackAtBase,
                    ownRequest = true,
                    ownRequestReplaced = false
                ),
                canSend = true
            ),
            onOpenReturn = {},
            onSignOut = {}
        )
    }
}

@Preview(name = "Dashboard, driving", widthDp = 400, heightDp = 900, showBackground = true, backgroundColor = 0xFF0B0305)
@Composable
private fun DashboardDrivingPreview() {
    GarbyTheme {
        DashboardContent(
            state = previewDeviceState(RobotState.Outbound),
            summary = ReturnSummary(),
            onOpenReturn = {},
            onSignOut = {}
        )
    }
}
