package com.thesis.garby

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.calculateEndPadding
import androidx.compose.foundation.layout.calculateStartPadding
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.rounded.ArrowBack
import androidx.compose.material.icons.automirrored.rounded.KeyboardReturn
import androidx.compose.material.icons.rounded.Cancel
import androidx.compose.material.icons.rounded.CheckCircle
import androidx.compose.material.icons.rounded.Close
import androidx.compose.material.icons.rounded.DeleteOutline
import androidx.compose.material.icons.rounded.ErrorOutline
import androidx.compose.material.icons.rounded.PanTool
import androidx.compose.material.icons.rounded.RadioButtonUnchecked
import androidx.compose.material.icons.rounded.Sensors
import androidx.compose.material.icons.rounded.Shield
import androidx.compose.material.icons.rounded.WarningAmber
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.platform.LocalLayoutDirection
import androidx.compose.ui.semantics.LiveRegionMode
import androidx.compose.ui.semantics.liveRegion
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.semantics.stateDescription
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.thesis.garby.realtime.DeviceStatus
import com.thesis.garby.realtime.ReturnCheck
import com.thesis.garby.realtime.ReturnCommand
import com.thesis.garby.realtime.ReturnProgress
import com.thesis.garby.realtime.ReturnStage
import com.thesis.garby.realtime.ReturnStatus
import com.thesis.garby.realtime.RobotPresence
import com.thesis.garby.realtime.RobotState
import com.thesis.garby.realtime.acceptanceTimeLeftMs
import com.thesis.garby.realtime.describeReturnFailure
import com.thesis.garby.realtime.evaluateReturnReadiness
import com.thesis.garby.realtime.robotPresence
import com.thesis.garby.ui.formatAgo
import com.thesis.garby.ui.formatCountdown
import com.thesis.garby.ui.formatTimeOfDay
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
import com.thesis.garby.ui.glass.StatusPill
import com.thesis.garby.ui.returnCheckBlocker
import com.thesis.garby.ui.returnCheckLabel
import com.thesis.garby.ui.returnStageUi
import com.thesis.garby.ui.robotStateUi
import com.thesis.garby.ui.theme.GarbyColors
import com.thesis.garby.ui.theme.GarbyTheme

@Composable
fun ReturnScreen(
    viewModel: ReturnViewModel,
    onBack: () -> Unit
) {
    val state by viewModel.uiState.collectAsStateWithLifecycle()
    var confirming by rememberSaveable { mutableStateOf(false) }

    ReturnContent(
        state = state,
        onBack = onBack,
        onSend = { confirming = true },
        onDismissNotice = viewModel::clearNotice
    )

    if (confirming) {
        ConfirmReturnDialog(
            canSend = state.readiness.canSend,
            onConfirm = {
                confirming = false
                viewModel.requestReturn()
            },
            onDismiss = { confirming = false }
        )
    }
}

@Composable
fun ReturnContent(
    state: ReturnUiState,
    onBack: () -> Unit,
    onSend: () -> Unit,
    onDismissNotice: () -> Unit
) {
    val robotUi = robotStateUi(state.status?.robotState, state.status?.isReadyToReturn)
    GlassScaffold(
        topBar = {
            GlassTopBar(
                title = "Return to base",
                subtitle = if (state.statusLoaded) robotUi.title else "Loading robot state…",
                subtitleColor = if (state.statusLoaded) robotUi.color else GarbyColors.TextMuted,
                navigationIcon = {
                    GlassIconButton(
                        icon = Icons.AutoMirrored.Rounded.ArrowBack,
                        contentDescription = "Back",
                        onClick = onBack
                    )
                }
            )
        },
        bottomBar = { SendDock(state = state, onSend = onSend) }
    ) { padding ->
        val layoutDirection = LocalLayoutDirection.current
        Box(Modifier.fillMaxSize(), contentAlignment = Alignment.TopCenter) {
            LazyColumn(
                modifier = Modifier
                    .fillMaxSize()
                    .widthIn(max = 640.dp),
                contentPadding = PaddingValues(
                    start = padding.calculateStartPadding(layoutDirection) + 16.dp,
                    end = padding.calculateEndPadding(layoutDirection) + 16.dp,
                    top = padding.calculateTopPadding() + 4.dp,
                    bottom = padding.calculateBottomPadding() + 20.dp
                ),
                verticalArrangement = Arrangement.spacedBy(12.dp)
            ) {
                item(key = "intro") { IntroCard() }
                state.notice?.let { notice ->
                    item(key = "notice") { NoticeCard(notice, onDismissNotice) }
                }
                state.progress?.let { progress ->
                    item(key = "progress") { ProgressCard(progress, state) }
                }
                item(key = "checks") { ReadinessCard(state) }
                item(key = "safety") { SafetyCard() }
            }
        }
    }
}

@Composable
private fun IntroCard() {
    GlassPanel(
        modifier = Modifier.fillMaxWidth(),
        shape = GlassShapes.Hero,
        glow = GarbyColors.Ember,
        contentPadding = PaddingValues(20.dp)
    ) {
        IconOrb(Icons.AutoMirrored.Rounded.KeyboardReturn, GarbyColors.EmberGlow, size = 48.dp)
        Spacer(Modifier.height(14.dp))
        Text(
            "Send GARBY back to base",
            style = MaterialTheme.typography.headlineSmall,
            color = GarbyColors.TextPrimary
        )
        Spacer(Modifier.height(6.dp))
        Text(
            "Once the bin has been emptied at the collection point, GARBY drives its calibrated " +
                "return route. It still stops for people and obstacles and carries on by itself " +
                "once the path is clear.",
            style = MaterialTheme.typography.bodyMedium,
            color = GarbyColors.TextSecondary
        )
    }
}

@Composable
private fun NoticeCard(notice: ReturnNotice, onDismiss: () -> Unit) {
    val (title, message, color) = when (notice) {
        is ReturnNotice.NotSent -> Triple("Not sent", notice.message, GarbyColors.Caution)
        is ReturnNotice.Unconfirmed -> Triple("Delivery not confirmed", notice.message, GarbyColors.Danger)
    }
    GlassPanel(
        modifier = Modifier
            .fillMaxWidth()
            .semantics { liveRegion = LiveRegionMode.Polite },
        glow = color,
        contentPadding = PaddingValues(start = 16.dp, top = 14.dp, bottom = 14.dp, end = 6.dp)
    ) {
        Row(verticalAlignment = Alignment.Top) {
            IconOrb(Icons.Rounded.WarningAmber, color, size = 36.dp)
            Spacer(Modifier.width(12.dp))
            Column(Modifier.weight(1f)) {
                Text(title, style = MaterialTheme.typography.titleSmall, color = GarbyColors.TextPrimary)
                Text(message, style = MaterialTheme.typography.bodySmall, color = GarbyColors.TextSecondary)
            }
            GlassIconButton(Icons.Rounded.Close, contentDescription = "Dismiss", onClick = onDismiss)
        }
    }
}

@Composable
private fun ProgressCard(progress: ReturnProgress, state: ReturnUiState) {
    val command = progress.command
    val stage = returnStageUi(progress.stage)
    val locale = LocalConfiguration.current.locales[0]
    val now = state.nowMs

    GlassPanel(
        modifier = Modifier
            .fillMaxWidth()
            .semantics {
                liveRegion = LiveRegionMode.Polite
                stateDescription = stage.label
            },
        glow = stage.color,
        contentPadding = PaddingValues(20.dp)
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Column(Modifier.weight(1f)) {
                Text("LATEST REQUEST", style = MaterialTheme.typography.labelSmall, color = GarbyColors.TextMuted)
                Text(
                    text = when {
                        progress.ownRequest -> "Sent from this phone"
                        progress.ownRequestReplaced -> "Your request was replaced by a newer one"
                        else -> "Sent by another operator"
                    },
                    style = MaterialTheme.typography.bodySmall,
                    color = GarbyColors.TextSecondary
                )
            }
            StatusPill(stage.label, stage.color)
        }
        Spacer(Modifier.height(16.dp))

        TimelineStep(
            title = "Requested",
            detail = formatTimeOfDay(command.requestedAtMs, now, locale),
            state = StepState.Done,
            showConnector = true
        )
        when (progress.stage) {
            ReturnStage.WaitingForAcceptance -> {
                val left = acceptanceTimeLeftMs(command, state.serverNowMs)
                TimelineStep(
                    title = "Accepted by GARBY",
                    detail = if (left > 0L) {
                        "Waiting · ${formatCountdown(left)} left to accept"
                    } else {
                        "Waiting · the Pi's 2-minute window is closing"
                    },
                    state = StepState.Active,
                    showConnector = true
                )
                TimelineStep("Back at base", null, StepState.Pending, showConnector = false)
            }
            ReturnStage.NotAccepted -> {
                TimelineStep(
                    title = "Not accepted",
                    detail = "GARBY did not accept this request within 2 minutes, so the Pi will " +
                        "ignore it. Check that GARBY is online before sending again.",
                    state = StepState.Failed,
                    showConnector = false
                )
            }
            ReturnStage.Returning, ReturnStage.Overdue -> {
                TimelineStep(
                    title = "Accepted by GARBY",
                    detail = command.ackAtMs?.let { formatTimeOfDay(it, now, locale) },
                    state = StepState.Done,
                    showConnector = true
                )
                TimelineStep(
                    title = "Back at base",
                    detail = if (progress.stage == ReturnStage.Returning) {
                        "Driving back" + (command.ackAtMs?.let { " · accepted ${formatAgo(now - it)}" } ?: "")
                    } else {
                        "No arrival report after 15 minutes. Check the robot in person."
                    },
                    state = if (progress.stage == ReturnStage.Returning) StepState.Active else StepState.Failed,
                    showConnector = false
                )
            }
            ReturnStage.BackAtBase -> {
                TimelineStep(
                    title = "Accepted by GARBY",
                    detail = command.ackAtMs?.let { formatTimeOfDay(it, now, locale) },
                    state = StepState.Done,
                    showConnector = true
                )
                TimelineStep(
                    title = "Back at base",
                    detail = command.doneAtMs?.let { formatTimeOfDay(it, now, locale) },
                    state = StepState.Done,
                    showConnector = false
                )
            }
            ReturnStage.Failed -> {
                command.ackAtMs?.let { ack ->
                    TimelineStep("Accepted by GARBY", formatTimeOfDay(ack, now, locale), StepState.Done, showConnector = true)
                }
                TimelineStep(
                    title = "Failed" + (command.failedAtMs?.let { " · ${formatTimeOfDay(it, now, locale)}" } ?: ""),
                    detail = describeReturnFailure(command.reason),
                    state = StepState.Failed,
                    showConnector = false
                )
            }
            ReturnStage.Unknown -> {
                TimelineStep(
                    title = "Unrecognised status",
                    detail = "The cloud holds a status this app does not know. Treat GARBY's state as " +
                        "unknown and check it in person.",
                    state = StepState.Failed,
                    showConnector = false
                )
            }
        }
    }
}

private enum class StepState { Done, Active, Pending, Failed }

@Composable
private fun TimelineStep(title: String, detail: String?, state: StepState, showConnector: Boolean) {
    val (icon, color) = when (state) {
        StepState.Done -> Icons.Rounded.CheckCircle to GarbyColors.Ok
        StepState.Active -> Icons.Rounded.RadioButtonUnchecked to GarbyColors.Info
        StepState.Pending -> Icons.Rounded.RadioButtonUnchecked to GarbyColors.TextMuted
        StepState.Failed -> Icons.Rounded.ErrorOutline to GarbyColors.Danger
    }
    Row {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Icon(icon, contentDescription = null, tint = color, modifier = Modifier.size(22.dp))
            if (showConnector) {
                Box(
                    Modifier
                        .padding(vertical = 2.dp)
                        .width(2.dp)
                        .height(26.dp)
                        .clip(CircleShape)
                        .background(color.copy(alpha = 0.45f))
                )
            }
        }
        Spacer(Modifier.width(12.dp))
        Column(Modifier.padding(bottom = if (showConnector) 4.dp else 0.dp)) {
            Text(
                title,
                style = MaterialTheme.typography.titleSmall,
                color = if (state == StepState.Pending) GarbyColors.TextMuted else GarbyColors.TextPrimary
            )
            detail?.let {
                Text(it, style = MaterialTheme.typography.bodySmall, color = GarbyColors.TextSecondary)
            }
        }
    }
}

@Composable
private fun ReadinessCard(state: ReturnUiState) {
    val status = state.status
    GlassPanel(modifier = Modifier.fillMaxWidth(), contentPadding = PaddingValues(20.dp)) {
        Text("BEFORE SENDING", style = MaterialTheme.typography.labelSmall, color = GarbyColors.TextMuted)
        Text(
            "Every check must pass. The app asks the cloud again just before it writes.",
            style = MaterialTheme.typography.bodySmall,
            color = GarbyColors.TextSecondary
        )
        Spacer(Modifier.height(12.dp))
        state.readiness.results.forEach { result ->
            CheckRow(
                label = returnCheckLabel(result.check),
                detail = checkDetail(result.check, state, status),
                passed = result.passed,
                known = state.statusLoaded || result.check in PHONE_SIDE_CHECKS
            )
        }
    }
}

/** Checks answered by the phone itself, known before the robot's status arrives. */
private val PHONE_SIDE_CHECKS = setOf(ReturnCheck.Internet, ReturnCheck.Cloud)

private fun checkDetail(check: ReturnCheck, state: ReturnUiState, status: DeviceStatus?): String = when (check) {
    ReturnCheck.AtDestination -> "Now: " + robotStateUi(status?.robotState, status?.isReadyToReturn).title
    ReturnCheck.RobotOnline -> when (robotPresence(status, state.nowMs)) {
        RobotPresence.Live -> "Seen ${formatAgo(state.nowMs - (status?.lastSeenMs ?: 0L))}"
        RobotPresence.Stale -> "Last report ${formatAgo(state.nowMs - (status?.lastSeenMs ?: 0L))}"
        RobotPresence.NeverSeen -> "No reports"
    }
    ReturnCheck.BleLink -> when (status?.bleConnected) { true -> "Connected"; false -> "Down"; null -> "Not reported" }
    ReturnCheck.Lidar -> when (status?.lidarHealthy) { true -> "Scanning"; false -> "No scans"; null -> "Not reported" }
    ReturnCheck.Cloud -> if (state.cloudConnected) "Connected" else "Not connected"
    ReturnCheck.Internet -> if (state.internetAvailable) "Online" else "Offline"
    ReturnCheck.NoReturnInFlight -> when {
        state.submitting -> "Sending now"
        state.progress?.inFlight == true -> "One is in progress"
        else -> "None running"
    }
}

@Composable
private fun CheckRow(label: String, detail: String, passed: Boolean, known: Boolean) {
    val (icon, color) = when {
        passed -> Icons.Rounded.CheckCircle to GarbyColors.Ok
        known -> Icons.Rounded.Cancel to GarbyColors.Danger
        else -> Icons.Rounded.RadioButtonUnchecked to GarbyColors.TextMuted
    }
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 7.dp)
            .semantics(mergeDescendants = true) {
                stateDescription = if (passed) "Passed" else if (known) "Not met" else "Checking"
            }
    ) {
        Icon(icon, contentDescription = null, tint = color, modifier = Modifier.size(22.dp))
        Spacer(Modifier.width(12.dp))
        Column(Modifier.weight(1f)) {
            Text(label, style = MaterialTheme.typography.titleSmall, color = GarbyColors.TextPrimary)
            Text(detail, style = MaterialTheme.typography.bodySmall, color = GarbyColors.TextMuted)
        }
    }
}

@Composable
private fun SafetyCard() {
    GlassPanel(
        modifier = Modifier.fillMaxWidth(),
        tier = GlassTier.Clear,
        contentPadding = PaddingValues(16.dp)
    ) {
        Row(verticalAlignment = Alignment.Top) {
            IconOrb(Icons.Rounded.Shield, GarbyColors.Info, size = 36.dp)
            Spacer(Modifier.width(12.dp))
            Text(
                "The app can only ask. GARBY's own sensors decide every movement, and the emergency " +
                    "stop on the Pi monitor always wins. If GARBY stops with a route fault, recover it " +
                    "in person; never resend from the app to move it.",
                style = MaterialTheme.typography.bodySmall,
                color = GarbyColors.TextSecondary
            )
        }
    }
}

@Composable
private fun SendDock(state: ReturnUiState, onSend: () -> Unit) {
    val readiness = state.readiness
    GlassDock {
        val caption = when {
            state.submitting -> "Sending the request…"
            readiness.canSend -> "All checks passed."
            state.progress?.inFlight == true -> "A return is already in progress."
            else -> readiness.firstUnmet?.let(::returnCheckBlocker) ?: "Checking…"
        }
        Text(
            caption,
            style = MaterialTheme.typography.bodySmall,
            color = if (readiness.canSend) GarbyColors.Ok else GarbyColors.TextSecondary,
            modifier = Modifier.padding(horizontal = 6.dp)
        )
        GlassButton(
            text = "Send back to base",
            onClick = onSend,
            enabled = readiness.canSend,
            loading = state.submitting,
            icon = Icons.AutoMirrored.Rounded.KeyboardReturn,
            modifier = Modifier.fillMaxWidth()
        )
    }
}

@Composable
private fun ConfirmReturnDialog(canSend: Boolean, onConfirm: () -> Unit, onDismiss: () -> Unit) {
    GlassDialog(onDismissRequest = onDismiss) {
        IconOrb(Icons.AutoMirrored.Rounded.KeyboardReturn, GarbyColors.EmberGlow, size = 48.dp)
        Spacer(Modifier.height(14.dp))
        Text(
            "Send GARBY back to base?",
            style = MaterialTheme.typography.headlineSmall,
            color = GarbyColors.TextPrimary
        )
        Spacer(Modifier.height(6.dp))
        Text(
            "It will drive the return route from the collection point.",
            style = MaterialTheme.typography.bodyMedium,
            color = GarbyColors.TextSecondary
        )
        Spacer(Modifier.height(14.dp))
        DialogPoint(Icons.Rounded.DeleteOutline, "The bin has been emptied and the hallway is clear.")
        DialogPoint(Icons.Rounded.Sensors, "It still stops for people and obstacles, and beeps while driving.")
        DialogPoint(Icons.Rounded.PanTool, "Keep the Pi monitor's emergency stop within reach.")
        if (!canSend) {
            Spacer(Modifier.height(8.dp))
            Text(
                "GARBY is no longer ready to return.",
                style = MaterialTheme.typography.labelLarge,
                color = GarbyColors.Danger,
                modifier = Modifier.semantics { liveRegion = LiveRegionMode.Assertive }
            )
        }
        Spacer(Modifier.height(20.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(12.dp)) {
            GlassButton(
                text = "Cancel",
                onClick = onDismiss,
                style = GlassButtonStyle.Secondary,
                modifier = Modifier.weight(1f)
            )
            GlassButton(
                text = "Send back",
                onClick = onConfirm,
                enabled = canSend,
                modifier = Modifier.weight(1f)
            )
        }
    }
}

@Composable
private fun DialogPoint(icon: ImageVector, text: String) {
    Row(verticalAlignment = Alignment.CenterVertically, modifier = Modifier.padding(vertical = 5.dp)) {
        Icon(icon, contentDescription = null, tint = GarbyColors.Rose, modifier = Modifier.size(20.dp))
        Spacer(Modifier.width(12.dp))
        Text(text, style = MaterialTheme.typography.bodyMedium, color = GarbyColors.TextPrimary)
    }
}

private const val PREVIEW_NOW = 1_763_000_000_000L

private fun previewReturnState(progress: ReturnProgress?): ReturnUiState {
    val status = DeviceStatus(
        lastSeenMs = PREVIEW_NOW - 9_000L,
        robotState = if (progress == null) RobotState.AtDestination else RobotState.Returning,
        isReadyToReturn = progress == null,
        bleConnected = true,
        lidarHealthy = true,
        sensorSerialConnected = true
    )
    return ReturnUiState(
        status = status,
        statusLoaded = true,
        cloudConnected = true,
        internetAvailable = true,
        latest = progress?.command,
        progress = progress,
        readiness = evaluateReturnReadiness(status, true, true, progress?.inFlight == true, PREVIEW_NOW),
        nowMs = PREVIEW_NOW,
        serverNowMs = PREVIEW_NOW
    )
}

@Preview(name = "Ready", widthDp = 400, heightDp = 1200, showBackground = true, backgroundColor = 0xFF0B0305)
@Composable
private fun ReturnReadyPreview() {
    GarbyTheme {
        ReturnContent(previewReturnState(null), onBack = {}, onSend = {}, onDismissNotice = {})
    }
}

@Preview(name = "Returning", widthDp = 400, heightDp = 1200, showBackground = true, backgroundColor = 0xFF0B0305)
@Composable
private fun ReturnInProgressPreview() {
    val command = ReturnCommand(
        requestedAtMs = PREVIEW_NOW - 95_000L,
        requestedBy = "op",
        status = ReturnStatus.Accepted,
        ackAtMs = PREVIEW_NOW - 91_000L
    )
    GarbyTheme {
        ReturnContent(
            previewReturnState(ReturnProgress(command, ReturnStage.Returning, ownRequest = true, ownRequestReplaced = false)),
            onBack = {},
            onSend = {},
            onDismissNotice = {}
        )
    }
}

@Preview(name = "Failed", widthDp = 400, heightDp = 1200, showBackground = true, backgroundColor = 0xFF0B0305)
@Composable
private fun ReturnFailedPreview() {
    val command = ReturnCommand(
        requestedAtMs = PREVIEW_NOW - 60_000L,
        requestedBy = "op",
        status = ReturnStatus.Failed,
        failedAtMs = PREVIEW_NOW - 55_000L,
        reason = "operator_emergency_stop"
    )
    GarbyTheme {
        ReturnContent(
            previewReturnState(ReturnProgress(command, ReturnStage.Failed, ownRequest = true, ownRequestReplaced = false))
                .copy(notice = ReturnNotice.NotSent("GARBY's Pi heartbeat is stale. Nothing was sent.")),
            onBack = {},
            onSend = {},
            onDismissNotice = {}
        )
    }
}
