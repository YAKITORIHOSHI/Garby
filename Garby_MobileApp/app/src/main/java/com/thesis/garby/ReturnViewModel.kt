package com.thesis.garby

import androidx.lifecycle.SavedStateHandle
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.google.firebase.auth.FirebaseAuth
import com.thesis.garby.network.NetworkConnectivityManager
import com.thesis.garby.performance.GarbyDispatchers
import com.thesis.garby.realtime.ConnectionState
import com.thesis.garby.realtime.DeviceStatus
import com.thesis.garby.realtime.GarbyRealtimeDb
import com.thesis.garby.realtime.ReturnCommand
import com.thesis.garby.realtime.ReturnProgress
import com.thesis.garby.realtime.ReturnReadiness
import com.thesis.garby.realtime.ReturnRequestRef
import com.thesis.garby.realtime.ReturnSubmission
import com.thesis.garby.realtime.RtdbConstants
import com.thesis.garby.realtime.evaluateReturnReadiness
import com.thesis.garby.realtime.resolveReturnProgress
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.flow.flowOn
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.onStart
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlin.time.Duration.Companion.milliseconds

/** Result of the last submission attempt, shown until dismissed or replaced. */
sealed interface ReturnNotice {
    /** Nothing was written; the message says why. */
    data class NotSent(val message: String) : ReturnNotice

    /** A write may or may not exist. The operator must treat the robot state as unknown. */
    data class Unconfirmed(val message: String) : ReturnNotice
}

data class ReturnUiState(
    val status: DeviceStatus? = null,
    val statusLoaded: Boolean = false,
    val cloudConnected: Boolean = false,
    val internetAvailable: Boolean = false,
    val latest: ReturnCommand? = null,
    val progress: ReturnProgress? = null,
    val readiness: ReturnReadiness = ReturnReadiness.Blocked,
    val submitting: Boolean = false,
    val notice: ReturnNotice? = null,
    val nowMs: Long = 0L,
    val serverNowMs: Long = 0L
)

/** What the dashboard needs; changes only when the request or the gate changes, not every tick. */
data class ReturnSummary(
    val progress: ReturnProgress? = null,
    val canSend: Boolean = false
)

/**
 * Activity-scoped return-to-base workflow.
 *
 * The request identity lives in [SavedStateHandle], and the outcome is read back from Firebase,
 * so the operator can leave the screen, background the app or rotate the phone without losing
 * the request. The ViewModel never infers an outcome: ACK, completion and failure come only from
 * the Pi's matching transitions of `devices/<id>/commands/reset`.
 */
class ReturnViewModel(private val savedState: SavedStateHandle) : ViewModel() {

    private companion object {
        const val KEY_TRACKED_AT = "return.trackedRequestedAt"
        const val KEY_TRACKED_BY = "return.trackedRequestedBy"
        const val STOP_TIMEOUT_MS = 5_000L
    }

    private val firebaseAuth = FirebaseAuth.getInstance()
    private val deviceId = RtdbConstants.DEFAULT_DEVICE_ID
    private val submitting = MutableStateFlow(false)
    private val notice = MutableStateFlow<ReturnNotice?>(null)

    private val tracked = combine(
        savedState.getStateFlow(KEY_TRACKED_AT, 0L),
        savedState.getStateFlow(KEY_TRACKED_BY, "")
    ) { at, by -> if (at > 0L && by.isNotBlank()) ReturnRequestRef(at, by) else null }

    private data class RemoteInputs(
        val status: DeviceStatus?,
        val statusLoaded: Boolean,
        val cloudConnected: Boolean,
        val internetAvailable: Boolean,
        val latest: ReturnCommand?,
        val serverOffsetMs: Long
    )

    private data class LocalInputs(
        val tracked: ReturnRequestRef?,
        val submitting: Boolean,
        val notice: ReturnNotice?
    )

    private val statusFlow = GarbyRealtimeDb.deviceStatus(deviceId)
        .map<DeviceStatus, Pair<DeviceStatus?, Boolean>> { it to true }
        .onStart { emit(null to false) }
        .catch { error ->
            if (error is CancellationException) throw error
            emit(null to true)
        }

    private val commandFlow = GarbyRealtimeDb.returnCommand(deviceId)
        .catch { error ->
            if (error is CancellationException) throw error
            emit(null)
        }

    private val remoteInputs = combine(
        statusFlow,
        GarbyRealtimeDb.connectionState.map { it is ConnectionState.Connected }.onStart { emit(false) },
        NetworkConnectivityManager.isInternetAvailable,
        commandFlow.onStart { emit(null) },
        GarbyRealtimeDb.serverTimeOffsetMs.onStart { emit(0L) }.catch { emit(0L) }
    ) { status, cloud, internet, latest, offset ->
        RemoteInputs(status.first, status.second, cloud, internet, latest, offset)
    }

    private val localInputs = combine(tracked, submitting, notice) { trackedRequest, busy, lastNotice ->
        LocalInputs(trackedRequest, busy, lastNotice)
    }

    private val clock = flow {
        while (true) {
            emit(System.currentTimeMillis())
            delay(RtdbConstants.RETURN_TICK_MS.milliseconds)
        }
    }

    val uiState: StateFlow<ReturnUiState> = combine(remoteInputs, localInputs, clock) { remote, local, now ->
        val serverNow = now + remote.serverOffsetMs
        val progress = resolveReturnProgress(remote.latest, local.tracked, serverNow)
        ReturnUiState(
            status = remote.status,
            statusLoaded = remote.statusLoaded,
            cloudConnected = remote.cloudConnected,
            internetAvailable = remote.internetAvailable,
            latest = remote.latest,
            progress = progress,
            readiness = evaluateReturnReadiness(
                status = remote.status,
                cloudConnected = remote.cloudConnected,
                internetAvailable = remote.internetAvailable,
                returnInFlight = local.submitting || progress?.inFlight == true,
                nowMs = now
            ),
            submitting = local.submitting,
            notice = local.notice,
            nowMs = now,
            serverNowMs = serverNow
        )
    }
        .flowOn(GarbyDispatchers.background)
        .stateIn(viewModelScope, SharingStarted.WhileSubscribed(STOP_TIMEOUT_MS), ReturnUiState())

    val summary: StateFlow<ReturnSummary> = uiState
        .map { ReturnSummary(it.progress, it.readiness.canSend) }
        .distinctUntilChanged()
        .stateIn(viewModelScope, SharingStarted.WhileSubscribed(STOP_TIMEOUT_MS), ReturnSummary())

    /**
     * Writes one return request after the operator's confirmation. The gate is evaluated again
     * here, and once more against the server copy inside [GarbyRealtimeDb.requestReturn].
     */
    fun requestReturn() {
        if (submitting.value) return
        val readiness = uiState.value.readiness
        if (!readiness.canSend) {
            notice.value = ReturnNotice.NotSent("GARBY is not ready to return. Nothing was sent.")
            return
        }
        val uid = firebaseAuth.currentUser?.uid?.takeIf { it.isNotBlank() }
        if (uid == null) {
            notice.value = ReturnNotice.NotSent("Your session has ended. Connect again, then retry.")
            return
        }

        submitting.value = true
        notice.value = null
        viewModelScope.launch {
            try {
                val result = withContext(GarbyDispatchers.background) {
                    GarbyRealtimeDb.requestReturn(deviceId = deviceId, requestedBy = uid)
                }
                // SavedStateHandle and visible submission state stay on the main dispatcher.
                when (result) {
                    is ReturnSubmission.Sent -> {
                        savedState[KEY_TRACKED_AT] = result.request.requestedAtMs
                        savedState[KEY_TRACKED_BY] = result.request.requestedBy
                    }
                    is ReturnSubmission.NotSent -> notice.value = ReturnNotice.NotSent(result.message)
                    is ReturnSubmission.Unconfirmed -> notice.value = ReturnNotice.Unconfirmed(
                        result.message + " Treat GARBY's state as unknown and check it before sending again."
                    )
                }
            } finally {
                submitting.value = false
            }
        }
    }

    fun clearNotice() {
        notice.value = null
    }

    /** A new anonymous session has a different identity, so its requests are tracked afresh. */
    fun onSignedOut() {
        savedState[KEY_TRACKED_AT] = 0L
        savedState[KEY_TRACKED_BY] = ""
        notice.value = null
    }
}
