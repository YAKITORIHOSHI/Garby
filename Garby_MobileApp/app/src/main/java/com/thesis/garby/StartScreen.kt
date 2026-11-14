package com.thesis.garby

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.rounded.KeyboardReturn
import androidx.compose.material.icons.automirrored.rounded.Login
import androidx.compose.material.icons.rounded.ErrorOutline
import androidx.compose.material.icons.rounded.Route
import androidx.compose.material.icons.rounded.Sensors
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.semantics.LiveRegionMode
import androidx.compose.ui.semantics.heading
import androidx.compose.ui.semantics.liveRegion
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.thesis.garby.ui.glass.Backdrop
import com.thesis.garby.ui.glass.GlassButton
import com.thesis.garby.ui.glass.GlassPanel
import com.thesis.garby.ui.glass.GlassScaffold
import com.thesis.garby.ui.glass.GlassShapes
import com.thesis.garby.ui.glass.IconOrb
import com.thesis.garby.ui.glass.LogoBadge
import com.thesis.garby.ui.glass.widthCap
import com.thesis.garby.ui.theme.GarbyColors
import com.thesis.garby.ui.theme.GarbyTheme

@Composable
fun StartScreen(
    isAuthenticating: Boolean,
    errorMessage: String?,
    onConnect: () -> Unit
) {
    GlassScaffold(backdrop = Backdrop.Deep) { padding ->
        BoxWithConstraints(
            Modifier
                .fillMaxSize()
                .padding(padding),
            contentAlignment = Alignment.TopCenter
        ) {
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.SpaceBetween,
                modifier = Modifier
                    .widthCap()
                    .fillMaxWidth()
                    .verticalScroll(rememberScrollState())
                    .heightIn(min = maxHeight)
                    .padding(horizontal = 24.dp, vertical = 16.dp)
            ) {
                Column(horizontalAlignment = Alignment.CenterHorizontally) {
                    Spacer(Modifier.height(24.dp))
                    LogoBadge(size = 148.dp, ring = 4.dp)
                    Spacer(Modifier.height(22.dp))
                    Text(
                        "GARBY",
                        style = MaterialTheme.typography.displaySmall,
                        color = GarbyColors.TextPrimary,
                        modifier = Modifier.semantics { heading() }
                    )
                    Text(
                        "Operator console",
                        style = MaterialTheme.typography.titleMedium,
                        color = GarbyColors.Rose
                    )
                    Spacer(Modifier.height(6.dp))
                    Text(
                        "Your IoT-powered mobile waste robot",
                        style = MaterialTheme.typography.bodyMedium,
                        color = GarbyColors.TextSecondary,
                        textAlign = TextAlign.Center
                    )
                }

                GlassPanel(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(vertical = 28.dp),
                    shape = GlassShapes.Hero,
                    contentPadding = PaddingValues(horizontal = 18.dp, vertical = 8.dp)
                ) {
                    Feature(
                        icon = Icons.Rounded.Sensors,
                        title = "Live bin and air readings",
                        text = "Weight, bin level and gas, marked stale when they get old."
                    )
                    Feature(
                        icon = Icons.Rounded.Route,
                        title = "Follow every run",
                        text = "Where GARBY is between base and the collection point."
                    )
                    Feature(
                        icon = Icons.AutoMirrored.Rounded.KeyboardReturn,
                        title = "Send it back to base",
                        text = "Only while it is waiting at the collection point."
                    )
                }

                Column(horizontalAlignment = Alignment.CenterHorizontally) {
                    if (!errorMessage.isNullOrBlank()) {
                        GlassPanel(
                            modifier = Modifier
                                .fillMaxWidth()
                                .padding(bottom = 14.dp)
                                .semantics { liveRegion = LiveRegionMode.Polite },
                            glow = GarbyColors.Danger,
                            contentPadding = PaddingValues(14.dp)
                        ) {
                            Row(verticalAlignment = Alignment.CenterVertically) {
                                IconOrb(Icons.Rounded.ErrorOutline, GarbyColors.Danger, size = 32.dp)
                                Spacer(Modifier.width(12.dp))
                                Text(
                                    errorMessage,
                                    style = MaterialTheme.typography.bodySmall,
                                    color = GarbyColors.TextPrimary
                                )
                            }
                        }
                    }
                    GlassButton(
                        text = if (isAuthenticating) "Connecting…" else "Connect to GARBY",
                        onClick = onConnect,
                        loading = isAuthenticating,
                        icon = Icons.AutoMirrored.Rounded.Login,
                        modifier = Modifier.fillMaxWidth()
                    )
                    Spacer(Modifier.height(10.dp))
                    Text(
                        "Signs in anonymously with Firebase. No password is stored on this phone.",
                        style = MaterialTheme.typography.bodySmall,
                        color = GarbyColors.TextMuted,
                        textAlign = TextAlign.Center
                    )
                }
            }
        }
    }
}

@Composable
private fun Feature(icon: ImageVector, title: String, text: String) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 10.dp)
    ) {
        IconOrb(icon, GarbyColors.EmberGlow, size = 40.dp)
        Spacer(Modifier.width(14.dp))
        Column(Modifier.weight(1f)) {
            Text(title, style = MaterialTheme.typography.titleSmall, color = GarbyColors.TextPrimary)
            Text(text, style = MaterialTheme.typography.bodySmall, color = GarbyColors.TextSecondary)
        }
    }
}

@Preview(name = "Start", widthDp = 400, heightDp = 860, showBackground = true, backgroundColor = 0xFF0B0305)
@Composable
private fun StartPreview() {
    GarbyTheme {
        StartScreen(isAuthenticating = false, errorMessage = null, onConnect = {})
    }
}

@Preview(name = "Start, error", widthDp = 400, heightDp = 860, showBackground = true, backgroundColor = 0xFF0B0305)
@Composable
private fun StartErrorPreview() {
    GarbyTheme {
        StartScreen(
            isAuthenticating = false,
            errorMessage = "Firebase connection timed out. Check internet and try again.",
            onConnect = {}
        )
    }
}
