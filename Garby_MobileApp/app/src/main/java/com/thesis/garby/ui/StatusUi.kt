package com.thesis.garby.ui

import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.rounded.HelpOutline
import androidx.compose.material.icons.automirrored.rounded.KeyboardReturn
import androidx.compose.material.icons.rounded.ErrorOutline
import androidx.compose.material.icons.rounded.Flag
import androidx.compose.material.icons.rounded.Home
import androidx.compose.material.icons.rounded.HourglassTop
import androidx.compose.material.icons.rounded.Route
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import com.thesis.garby.realtime.HealthLevel
import com.thesis.garby.realtime.ReturnCheck
import com.thesis.garby.realtime.ReturnStage
import com.thesis.garby.realtime.RobotState
import com.thesis.garby.realtime.Severity
import com.thesis.garby.ui.glass.RoutePosition
import com.thesis.garby.ui.theme.GarbyColors

/** How one robot state is presented. */
data class RobotStateUi(
    val title: String,
    val description: String,
    val icon: ImageVector,
    val color: Color,
    val route: RoutePosition,
    val moving: Boolean = false
)

/**
 * Wording follows the robot's actual behaviour (README "What it does" and "Use"). A null state
 * means the Pi does not publish one; its destination flag is then the only evidence.
 */
fun robotStateUi(state: RobotState?, readyFlag: Boolean?): RobotStateUi = when (state) {
    RobotState.Idle -> RobotStateUi(
        title = "Idle at base",
        description = "Starts a run on its own when the bin reaches 4 kg or the gas readings stay " +
            "dangerous, or when Start is pressed on the Pi monitor.",
        icon = Icons.Rounded.Home,
        color = GarbyColors.Ok,
        route = RoutePosition.AtBase
    )
    RobotState.Outbound -> RobotStateUi(
        title = "Driving to the collection point",
        description = "Following its calibrated route and beeping as it goes. It stops for anything " +
            "in its path and carries on by itself once the path is clear.",
        icon = Icons.Rounded.Route,
        color = GarbyColors.Info,
        route = RoutePosition.ToDestination,
        moving = true
    )
    RobotState.AtDestination -> RobotStateUi(
        title = "Waiting at the collection point",
        description = "Parked and stopped. Once the bin has been emptied, send GARBY back to base " +
            "from this app.",
        icon = Icons.Rounded.Flag,
        color = GarbyColors.EmberGlow,
        route = RoutePosition.AtDestination
    )
    RobotState.Returning -> RobotStateUi(
        title = "Returning to base",
        description = "Driving the return route. It still stops for people and obstacles and " +
            "carries on once the path is clear.",
        icon = Icons.AutoMirrored.Rounded.KeyboardReturn,
        color = GarbyColors.Info,
        route = RoutePosition.ToBase,
        moving = true
    )
    RobotState.Fault -> RobotStateUi(
        title = "Route fault",
        description = "GARBY stopped where it is (or refused a return). If this stays, move it back " +
            "to base by hand and clear the fault at the robot with the supervised reset. The app " +
            "cannot clear it.",
        icon = Icons.Rounded.ErrorOutline,
        color = GarbyColors.Danger,
        route = RoutePosition.Stopped
    )
    RobotState.Unknown -> RobotStateUi(
        title = "Waiting for the controllers",
        description = "The Pi has no state from the robot's controllers yet. They may be restarting, " +
            "or the Bluetooth link between the Pi and the robot is down.",
        icon = Icons.Rounded.HourglassTop,
        color = GarbyColors.Neutral,
        route = RoutePosition.Unknown
    )
    null -> if (readyFlag == true) {
        robotStateUi(RobotState.AtDestination, null)
    } else {
        RobotStateUi(
            title = "State not reported",
            description = "This Pi version does not publish the robot state. Update the Pi software " +
                "to see where GARBY is.",
            icon = Icons.AutoMirrored.Rounded.HelpOutline,
            color = GarbyColors.Neutral,
            route = RoutePosition.Unknown
        )
    }
}

fun severityColor(severity: Severity): Color = when (severity) {
    Severity.Normal -> GarbyColors.Ok
    Severity.Elevated -> GarbyColors.Caution
    Severity.High -> GarbyColors.Warning
    Severity.Danger -> GarbyColors.Danger
}

fun healthColor(level: HealthLevel): Color = when (level) {
    HealthLevel.Ok -> GarbyColors.Ok
    HealthLevel.Caution -> GarbyColors.Caution
    HealthLevel.Fault -> GarbyColors.Danger
    HealthLevel.Unknown -> GarbyColors.Neutral
}

fun returnCheckLabel(check: ReturnCheck): String = when (check) {
    ReturnCheck.AtDestination -> "Waiting at the collection point"
    ReturnCheck.RobotOnline -> "GARBY reporting"
    ReturnCheck.BleLink -> "Pi to robot Bluetooth link"
    ReturnCheck.Lidar -> "LiDAR scanning"
    ReturnCheck.Cloud -> "Cloud connected"
    ReturnCheck.Internet -> "Phone online"
    ReturnCheck.NoReturnInFlight -> "No other return running"
}

/** Why the send button is disabled, as one sentence for the first unmet check. */
fun returnCheckBlocker(check: ReturnCheck): String = when (check) {
    ReturnCheck.AtDestination -> "GARBY is not waiting at the collection point."
    ReturnCheck.RobotOnline -> "GARBY's Pi is not reporting."
    ReturnCheck.BleLink -> "The Pi has no Bluetooth link to the robot."
    ReturnCheck.Lidar -> "The LiDAR is not scanning."
    ReturnCheck.Cloud -> "The cloud is not connected."
    ReturnCheck.Internet -> "This phone is offline."
    ReturnCheck.NoReturnInFlight -> "A return is already in progress."
}

data class StageUi(val label: String, val color: Color)

fun returnStageUi(stage: ReturnStage): StageUi = when (stage) {
    ReturnStage.WaitingForAcceptance -> StageUi("Waiting for GARBY", GarbyColors.Caution)
    ReturnStage.NotAccepted -> StageUi("Not accepted", GarbyColors.Danger)
    ReturnStage.Returning -> StageUi("Returning", GarbyColors.Info)
    ReturnStage.Overdue -> StageUi("No arrival report", GarbyColors.Danger)
    ReturnStage.BackAtBase -> StageUi("Back at base", GarbyColors.Ok)
    ReturnStage.Failed -> StageUi("Failed", GarbyColors.Danger)
    ReturnStage.Unknown -> StageUi("Unknown status", GarbyColors.Neutral)
}
