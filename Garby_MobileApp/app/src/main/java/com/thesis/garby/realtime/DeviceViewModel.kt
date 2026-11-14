package com.thesis.garby.realtime

import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.google.firebase.auth.FirebaseAuth
import com.thesis.garby.network.NetworkConnectivityManager
import com.thesis.garby.performance.GarbyDispatchers
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.awaitCancellation
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlin.time.Duration.Companion.milliseconds

/** Dashboard state for one sensor, including freshness of the latest callback. */
data class SensorReadingUiState(
    val reading: SensorReading? = null,
    val isLoading: Boolean = true,
    val isStale: Boolean = true,
    val error: String? = null
)

data class DeviceStatusUiState(
    val status: DeviceStatus? = null,
    val isLoading: Boolean = true,
    val isStale: Boolean = true,
    val error: String? = null
)

data class DeviceUiState(
    val connection: ConnectionState = ConnectionState.Reconnecting,
    val internetAvailable: Boolean = true,
    val level: SensorReadingUiState = SensorReadingUiState(),
    val weight: SensorReadingUiState = SensorReadingUiState(),
    val mq135: SensorReadingUiState = SensorReadingUiState(),
    val mq137: SensorReadingUiState = SensorReadingUiState(),
    val mq4: SensorReadingUiState = SensorReadingUiState(),
    val device: DeviceStatusUiState = DeviceStatusUiState(),
    val disposal: DisposalUiState = DisposalUiState(),
    /** Clock used for every age shown on screen; ticks every [RtdbConstants.FRESHNESS_TICK_MS]. */
    val nowMs: Long = 0L
) {
    val cloudConnected: Boolean get() = connection is ConnectionState.Connected
    val presence: RobotPresence get() = robotPresence(device.status, nowMs)
}

/** Owns all live dashboard subscriptions and continuously re-evaluates freshness. */
class DeviceViewModel : ViewModel() {

    private companion object {
        const val TAG = "DeviceViewModel"
    }

    private val deviceId = RtdbConstants.DEFAULT_DEVICE_ID
    private var disposalJob: Job? = null
    private val _uiState = MutableStateFlow(DeviceUiState(nowMs = System.currentTimeMillis()))
    val uiState: StateFlow<DeviceUiState> = _uiState.asStateFlow()

    /** Called inside repeatOnLifecycle(STARTED); cancellation detaches all listeners. */
    suspend fun observeWhileVisible(): Unit = coroutineScope {
        // Never show cached pre-background data as ready before fresh callbacks.
        _uiState.value = DeviceUiState(
            internetAvailable = NetworkConnectivityManager.isInternetAvailable.value,
            disposal = DisposalUiState(
                submitting = _uiState.value.disposal.submitting,
                notice = _uiState.value.disposal.notice,
                noticeRequestId = _uiState.value.disposal.noticeRequestId
            ),
            nowMs = System.currentTimeMillis()
        )
        collectConnection(this)
        collectInternet(this)
        // Stagger subscription times so container hydration (and the first
        // compose burst) is spread out instead of hitting one frame at once.
        collectSensor(this, SensorKey.Level, startupDelayMs = 0L)
        collectSensor(this, SensorKey.Weight, startupDelayMs = 80L)
        collectSensor(this, SensorKey.Mq135, startupDelayMs = 160L)
        collectSensor(this, SensorKey.Mq137, startupDelayMs = 240L)
        collectSensor(this, SensorKey.Mq4, startupDelayMs = 320L)
        collectDeviceStatus(this)
        collectDisposal(this)
        startFreshnessTicker(this)
        awaitCancellation()
    }

    private fun collectDisposal(scope: CoroutineScope) {
        scope.launch(GarbyDispatchers.background) {
            GarbyRealtimeDb.disposalCommand(deviceId)
                .catch { error ->
                    if (error is CancellationException) throw error
                    _uiState.update { state -> state.copy(disposal = state.disposal.copy(
                        loaded = true, error = "Manual disposal status unavailable"
                    )) }
                }
                .collect { command ->
                    _uiState.update { state -> state.copy(disposal = state.disposal.withCommand(command)) }
                }
        }
    }

    /** Called only after explicit confirmation; recheck the gate before doing any cloud work. */
    fun requestDisposal() {
        if (disposalJob?.isActive == true) return
        val state = _uiState.value
        val blocked = disposalBlockReason(state.device.status, state.cloudConnected,
            state.internetAvailable, state.disposal, System.currentTimeMillis())
        val uid = FirebaseAuth.getInstance().currentUser?.uid?.takeIf { it.isNotBlank() }
        if (blocked != null || uid == null) {
            _uiState.update { it.copy(disposal = it.disposal.copy(
                notice = blocked ?: "Your session ended. Connect again.", noticeRequestId = null
            )) }
            return
        }
        _uiState.update { it.copy(disposal = it.disposal.copy(
            submitting = true, notice = null, noticeRequestId = null)) }
        disposalJob = viewModelScope.launch(GarbyDispatchers.background) {
            try {
                val result = GarbyRealtimeDb.requestDisposal(deviceId = deviceId, requestedBy = uid)
                _uiState.update { it.copy(disposal = it.disposal.withSubmission(result)) }
            } finally {
                _uiState.update { it.copy(disposal = it.disposal.copy(submitting = false)) }
            }
        }
    }

    fun clearDisposalNotice() {
        _uiState.update { it.copy(disposal = it.disposal.copy(notice = null, noticeRequestId = null)) }
    }

    private fun collectConnection(scope: CoroutineScope) {
        scope.launch(GarbyDispatchers.background) {
            GarbyRealtimeDb.connectionState
                .catch { error ->
                    if (error is CancellationException) throw error
                    Log.w(TAG, "Connection stream failed", error)
                    emit(ConnectionState.Disconnected)
                }
                .collect { connection ->
                    _uiState.update { state ->
                        state.copy(connection = connection).recheckAll(System.currentTimeMillis())
                    }
                }
        }
    }

    private fun collectInternet(scope: CoroutineScope) {
        scope.launch(GarbyDispatchers.background) {
            NetworkConnectivityManager.isInternetAvailable.collect { available ->
                _uiState.update { it.copy(internetAvailable = available) }
            }
        }
    }

    private fun collectSensor(scope: CoroutineScope, key: SensorKey, startupDelayMs: Long = 0L) {
        scope.launch(GarbyDispatchers.background) {
            if (startupDelayMs > 0L) delay(startupDelayMs.milliseconds)
            GarbyRealtimeDb.sensorValue(deviceId, key).collect { state ->
                val next = when (state) {
                    SensorUiState.Loading -> SensorReadingUiState(isLoading = true)
                    is SensorUiState.Value -> SensorReadingUiState(
                        reading = state.data,
                        isLoading = false,
                        isStale = true,
                        error = null
                    )
                    is SensorUiState.Error -> SensorReadingUiState(
                        reading = null,
                        isLoading = false,
                        isStale = true,
                        error = state.message
                    )
                }
                _uiState.update { current ->
                    val now = System.currentTimeMillis()
                    current.withSensor(key, next.recheckFreshness(now, current.cloudConnected))
                }
            }
        }
    }

    private fun collectDeviceStatus(scope: CoroutineScope) {
        scope.launch(GarbyDispatchers.background) {
            GarbyRealtimeDb.deviceStatus(deviceId)
                .catch { error ->
                    if (error is CancellationException) throw error
                    Log.w(TAG, "Device-status stream failed", error)
                    _uiState.update {
                        it.copy(
                            device = DeviceStatusUiState(
                                status = null,
                                isLoading = false,
                                isStale = true,
                                error = "Robot status unavailable"
                            )
                        )
                    }
                }
                .collect { status ->
                    _uiState.update { current ->
                        val now = System.currentTimeMillis()
                        current.copy(
                            device = DeviceStatusUiState(
                                status = status,
                                isLoading = false,
                                isStale = true,
                                error = null
                            ).recheckFreshness(now, current.cloudConnected)
                        )
                    }
                }
        }
    }

    private fun startFreshnessTicker(scope: CoroutineScope) {
        scope.launch(GarbyDispatchers.background) {
            while (isActive) {
                delay(RtdbConstants.FRESHNESS_TICK_MS.milliseconds)
                _uiState.update { it.recheckAll(System.currentTimeMillis()) }
            }
        }
    }

    private fun DeviceUiState.recheckAll(now: Long): DeviceUiState {
        val connected = cloudConnected
        return copy(
            level = level.recheckFreshness(now, connected),
            weight = weight.recheckFreshness(now, connected),
            mq135 = mq135.recheckFreshness(now, connected),
            mq137 = mq137.recheckFreshness(now, connected),
            mq4 = mq4.recheckFreshness(now, connected),
            device = device.recheckFreshness(now, connected),
            nowMs = now
        )
    }

    private fun SensorReadingUiState.recheckFreshness(
        now: Long,
        cloudConnected: Boolean
    ): SensorReadingUiState {
        val reading = reading ?: return copy(isStale = true)
        return copy(
            isStale = !cloudConnected || !isFreshTimestamp(reading.updatedAtMs, nowMs = now)
        )
    }

    private fun DeviceStatusUiState.recheckFreshness(
        now: Long,
        cloudConnected: Boolean
    ): DeviceStatusUiState {
        val status = status ?: return copy(isStale = true)
        return copy(
            isStale = !cloudConnected || !isFreshTimestamp(status.lastSeenMs, nowMs = now)
        )
    }

    private fun DeviceUiState.withSensor(key: SensorKey, value: SensorReadingUiState): DeviceUiState =
        when (key) {
            SensorKey.Level -> copy(level = value)
            SensorKey.Weight -> copy(weight = value)
            SensorKey.Mq135 -> copy(mq135 = value)
            SensorKey.Mq137 -> copy(mq137 = value)
            SensorKey.Mq4 -> copy(mq4 = value)
        }
}
