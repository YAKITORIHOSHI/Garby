package com.thesis.garby

import androidx.activity.compose.BackHandler
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.outlined.LockReset
import androidx.compose.material.icons.outlined.WarningAmber
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.CenterAlignedTopAppBar
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Icon
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.viewmodel.compose.viewModel
import com.thesis.garby.ui.theme.Montserrat

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun ResetTrashbinScreen(
    onBack: () -> Unit = {},
    resetViewModel: ResetViewModel = viewModel()
) {
    val resetState by resetViewModel.uiState.collectAsState()
    val isReadyToReturn by resetViewModel.isReadyToReturn.collectAsState()
    var showConfirmation by rememberSaveable { mutableStateOf(false) }

    val busy = resetState is ResetUiState.Sending ||
        resetState is ResetUiState.AwaitingCompletion

    BackHandler(enabled = busy) { }

    if (showConfirmation) {
        AlertDialog(
            onDismissRequest = { if (!busy) showConfirmation = false },
            title = {
                Text(
                    "Confirm Reset Command",
                    fontFamily = Montserrat,
                    fontWeight = FontWeight.Bold
                )
            },
            text = {
                Text(
                    "Only continue after the trash bin is physically returned to GARBY. " +
                        "This command will signal the robot that the bin is reset and ready."
                )
            },
            confirmButton = {
                Button(
                    onClick = {
                        showConfirmation = false
                        resetViewModel.requestReset()
                    },
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFC62828))
                ) {
                    Text("SEND RESET", fontFamily = Montserrat, fontWeight = FontWeight.Bold)
                }
            },
            dismissButton = {
                TextButton(onClick = { showConfirmation = false }) {
                    Text("CANCEL", fontFamily = Montserrat)
                }
            }
        )
    }

    Scaffold(
        topBar = {
            CenterAlignedTopAppBar(
                title = {
                    Text(
                        "Reset Trash Bin",
                        color = Color.White,
                        fontFamily = Montserrat,
                        fontWeight = FontWeight.Black,
                        fontSize = 24.sp
                    )
                },
                colors = TopAppBarDefaults.topAppBarColors(containerColor = Color.Transparent)
            )
        },
        containerColor = Color.Transparent
    ) { innerPadding ->
        Box(modifier = Modifier.fillMaxSize()) {
            Image(
                painter = painterResource(id = R.drawable.app_background),
                contentDescription = null,
                contentScale = ContentScale.Crop,
                modifier = Modifier.fillMaxSize()
            )

            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.Center,
                modifier = Modifier
                    .fillMaxSize()
                    .padding(innerPadding)
                    .verticalScroll(rememberScrollState())
                    .padding(horizontal = 20.dp, vertical = 20.dp)
            ) {
                // Main Content Card
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .shadow(12.dp, RoundedCornerShape(28.dp))
                        .background(Color.White, RoundedCornerShape(28.dp))
                        .padding(24.dp)
                ) {
                    Column(
                        horizontalAlignment = Alignment.CenterHorizontally,
                        modifier = Modifier.fillMaxWidth()
                    ) {
                        // Icon Badge
                        Box(
                            modifier = Modifier
                                .size(80.dp)
                                .background(Color(0xFFFFE4EE), CircleShape),
                            contentAlignment = Alignment.Center
                        ) {
                            Icon(
                                imageVector = Icons.Outlined.LockReset,
                                contentDescription = null,
                                modifier = Modifier.size(44.dp),
                                tint = Color(0xFFC62828)
                            )
                        }

                        Spacer(modifier = Modifier.height(16.dp))

                        Text(
                            text = "BIN RESET CONTROL",
                            color = Color(0xFFC62828),
                            fontFamily = Montserrat,
                            fontWeight = FontWeight.Black,
                            fontSize = 22.sp,
                            textAlign = TextAlign.Center
                        )

                        Spacer(modifier = Modifier.height(12.dp))

                        Text(
                            text = "Ensure the trash bin is physically returned to GARBY before issuing a reset command.",
                            color = Color(0xFF424242),
                            fontFamily = Montserrat,
                            fontWeight = FontWeight.Medium,
                            fontSize = 14.sp,
                            lineHeight = 20.sp,
                            textAlign = TextAlign.Center
                        )

                        Spacer(modifier = Modifier.height(16.dp))

                        // Warning Box
                        Row(
                            verticalAlignment = Alignment.CenterVertically,
                            modifier = Modifier
                                .fillMaxWidth()
                                .background(
                                    if (!isReadyToReturn) Color(0xFFFFEBEE) else Color(0xFFFFF3CD),
                                    RoundedCornerShape(12.dp)
                                )
                                .padding(12.dp)
                        ) {
                            Icon(
                                imageVector = Icons.Outlined.WarningAmber,
                                contentDescription = "Warning",
                                tint = if (!isReadyToReturn) Color(0xFFC62828) else Color(0xFF856404),
                                modifier = Modifier.size(20.dp)
                            )
                            Spacer(modifier = Modifier.width(8.dp))
                            Text(
                                text = if (!isReadyToReturn) {
                                    "RESET DISABLED: Trash bin is not ready to return yet."
                                } else {
                                    "Do not use reset for network recovery."
                                },
                                color = if (!isReadyToReturn) Color(0xFFC62828) else Color(0xFF856404),
                                fontFamily = Montserrat,
                                fontWeight = FontWeight.Bold,
                                fontSize = 12.sp
                            )
                        }

                        Spacer(modifier = Modifier.height(20.dp))

                        // Status Display
                        ResetStatusPanel(resetState)

                        Spacer(modifier = Modifier.height(24.dp))

                        // Action Buttons
                        Column(
                            verticalArrangement = Arrangement.spacedBy(12.dp),
                            modifier = Modifier.fillMaxWidth()
                        ) {
                            val canClickReset = !busy && (isReadyToReturn || resetState is ResetUiState.Complete || resetState is ResetUiState.Failed)
                            // Primary Action: Reset / Clear
                            Button(
                                onClick = {
                                    when (resetState) {
                                        ResetUiState.Complete,
                                        is ResetUiState.Failed -> resetViewModel.clearResult()
                                        else -> showConfirmation = true
                                    }
                                },
                                enabled = canClickReset,
                                shape = RoundedCornerShape(16.dp),
                                colors = ButtonDefaults.buttonColors(
                                    containerColor = Color(0xFFC62828),
                                    contentColor = Color.White,
                                    disabledContainerColor = Color(0xFFE0E0E0),
                                    disabledContentColor = Color(0xFF9E9E9E)
                                ),
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .height(54.dp)
                            ) {
                                if (busy) {
                                    CircularProgressIndicator(
                                        color = Color.White,
                                        strokeWidth = 3.dp,
                                        modifier = Modifier.size(24.dp)
                                    )
                                } else {
                                    Text(
                                        text = when (resetState) {
                                            ResetUiState.Complete,
                                            is ResetUiState.Failed -> "CLEAR STATUS"
                                            else -> "RESET TRASH BIN"
                                        },
                                        fontFamily = Montserrat,
                                        fontWeight = FontWeight.Bold,
                                        fontSize = 16.sp
                                    )
                                }
                            }

                            // Secondary Action: Return to Dashboard
                            OutlinedButton(
                                onClick = onBack,
                                enabled = !busy,
                                shape = RoundedCornerShape(16.dp),
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .height(54.dp)
                            ) {
                                Text(
                                    text = "RETURN TO DASHBOARD",
                                    fontFamily = Montserrat,
                                    fontWeight = FontWeight.Bold,
                                    fontSize = 15.sp,
                                    color = Color(0xFF424242)
                                )
                            }
                        }
                    }
                }
            }
        }
    }
}

@Composable
private fun ResetStatusPanel(state: ResetUiState) {
    val (message, bgColor, textColor) = when (state) {
        ResetUiState.Idle -> Triple(
            "Ready to send reset command.",
            Color(0xFFF5F5F5),
            Color(0xFF616161)
        )
        ResetUiState.Sending -> Triple(
            "Sending reset command to Firebase...",
            Color(0xFFFFF9C4),
            Color(0xFFF57F17)
        )
        ResetUiState.AwaitingCompletion -> Triple(
            "Command sent. Waiting for robot confirmation...",
            Color(0xFFE3F2FD),
            Color(0xFF1565C0)
        )
        ResetUiState.Complete -> Triple(
            "Robot reported reset complete!",
            Color(0xFFE8F5E9),
            Color(0xFF2E7D32)
        )
        is ResetUiState.Failed -> {
            val prefix = if (state.deliveryUncertain) "STATUS UNKNOWN: " else "RESET FAILED: "
            Triple(
                prefix + state.message,
                Color(0xFFFFEBEE),
                Color(0xFFC62828)
            )
        }
    }

    Box(
        modifier = Modifier
            .fillMaxWidth()
            .background(bgColor, RoundedCornerShape(14.dp))
            .padding(14.dp),
        contentAlignment = Alignment.Center
    ) {
        Text(
            text = message,
            color = textColor,
            fontFamily = Montserrat,
            fontWeight = FontWeight.Bold,
            fontSize = 13.sp,
            textAlign = TextAlign.Center
        )
    }
}
