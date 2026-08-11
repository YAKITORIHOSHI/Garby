package com.thesis.garby

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.google.firebase.auth.FirebaseAuth
import com.thesis.garby.network.NetworkConnectivityManager
import com.thesis.garby.realtime.GarbyRealtimeDb
import com.thesis.garby.realtime.ResetStatus
import com.thesis.garby.realtime.RtdbConstants
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Job
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeout
import kotlin.time.Duration.Companion.milliseconds

sealed interface ResetUiState {
    object Idle : ResetUiState
    object Sending : ResetUiState
    object AwaitingCompletion : ResetUiState
    object Complete : ResetUiState
    data class Failed(
        val message: String,
        val deliveryUncertain: Boolean = false
    ) : ResetUiState
}

/**
 * Lifecycle-stable reset workflow. The UI only asks for a reset after explicit
 * confirmation; this ViewModel performs the authenticated/correlated exchange.
 */
class ResetViewModel : ViewModel() {

    private val firebaseAuth = FirebaseAuth.getInstance()
    private var resetJob: Job? = null

    private val _uiState = MutableStateFlow<ResetUiState>(ResetUiState.Idle)
    val uiState: StateFlow<ResetUiState> = _uiState.asStateFlow()

    private val _isReadyToReturn = MutableStateFlow(false)
    val isReadyToReturn: StateFlow<Boolean> = _isReadyToReturn.asStateFlow()

    init {
        viewModelScope.launch {
            GarbyRealtimeDb.isReadyToReturn().collect { ready ->
                _isReadyToReturn.value = ready
            }
        }
    }

    fun requestReset() {
        if (resetJob?.isActive == true) return

        val uid = firebaseAuth.currentUser?.uid?.takeIf { it.isNotBlank() }
        if (uid == null) {
            _uiState.value = ResetUiState.Failed("Reset requires an authenticated operator session.")
            return
        }
        if (!NetworkConnectivityManager.isNetworkConnected()) {
            _uiState.value = ResetUiState.Failed("Reset requires a validated internet connection.")
            return
        }

        resetJob = viewModelScope.launch {
            _uiState.value = ResetUiState.Sending

            val markerResult = GarbyRealtimeDb.requestReset(
                deviceId = RtdbConstants.DEFAULT_DEVICE_ID,
                requestedBy = uid
            )
            val marker = markerResult.getOrElse {
                _uiState.value = ResetUiState.Failed(
                    message = "Reset was not confirmed by Firebase. Verify the robot is stationary before retrying.",
                    deliveryUncertain = true
                )
                return@launch
            }

            _uiState.value = ResetUiState.AwaitingCompletion
            try {
                val outcome = withTimeout(RtdbConstants.RESET_ACK_TIMEOUT_MS.milliseconds) {
                    GarbyRealtimeDb.resetStatus(RtdbConstants.DEFAULT_DEVICE_ID)
                        .first { command ->
                            marker.matches(command) &&
                                (command.status == ResetStatus.Done || command.status == ResetStatus.Failed)
                        }
                }

                _uiState.value = when (outcome.status) {
                    ResetStatus.Done -> ResetUiState.Complete
                    ResetStatus.Failed -> ResetUiState.Failed("The robot reported that reset failed.")
                    else -> ResetUiState.Failed("Reset ended in an unexpected state.")
                }
            } catch (_: TimeoutCancellationException) {
                _uiState.value = ResetUiState.Failed(
                    message = "No completion acknowledgement arrived. Treat robot state as unknown and verify it physically.",
                    deliveryUncertain = true
                )
            } catch (e: CancellationException) {
                throw e
            } catch (_: Exception) {
                _uiState.value = ResetUiState.Failed(
                    message = "Reset status could not be verified. Treat robot state as unknown.",
                    deliveryUncertain = true
                )
            }
        }
    }

    fun clearResult() {
        if (resetJob?.isActive == true) return
        _uiState.value = ResetUiState.Idle
    }

    override fun onCleared() {
        resetJob?.cancel()
        super.onCleared()
    }
}
