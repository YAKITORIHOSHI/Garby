package com.thesis.garby.realtime

import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
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
    val level: SensorReadingUiState = SensorReadingUiState(),
    val weight: SensorReadingUiState = SensorReadingUiState(),
    val mq135: SensorReadingUiState = SensorReadingUiState(),
    val mq137: SensorReadingUiState = SensorReadingUiState(),
    val mq4: SensorReadingUiState = SensorReadingUiState(),
    val device: DeviceStatusUiState = DeviceStatusUiState()
)

/** Owns all live dashboard subscriptions and continuously re-evaluates freshness. */
class DeviceViewModel : ViewModel() {

    private companion object {
        const val TAG = "DeviceViewModel"
    }

    private val deviceId = RtdbConstants.DEFAULT_DEVICE_ID
    private val _uiState = MutableStateFlow(DeviceUiState())
    val uiState: StateFlow<DeviceUiState> = _uiState.asStateFlow()

    init {
        collectConnection()
        // Stagger subscription times so container hydration (and the first
        // compose burst) is spread out instead of hitting one frame at once.
        collectSensor(SensorKey.Level, startupDelayMs = 0L)
        collectSensor(SensorKey.Weight, startupDelayMs = 80L)
        collectSensor(SensorKey.Mq135, startupDelayMs = 160L)
        collectSensor(SensorKey.Mq137, startupDelayMs = 240L)
        collectSensor(SensorKey.Mq4, startupDelayMs = 320L)
        collectDeviceStatus()
        startFreshnessTicker()
    }

    private fun collectConnection() {
        viewModelScope.launch(Dispatchers.Default) {
            GarbyRealtimeDb.connectionState
                .catch { error ->
                    if (error is CancellationException) throw error
                    Log.w(TAG, "Connection stream failed", error)
                    emit(ConnectionState.Disconnected)
                }
                .collect { connection ->
                    val now = System.currentTimeMillis()
                    _uiState.update { state ->
                        val connected = connection is ConnectionState.Connected
                        state.copy(
                            connection = connection,
                            level = state.level.recheckFreshness(now, connected),
                            weight = state.weight.recheckFreshness(now, connected),
                            mq135 = state.mq135.recheckFreshness(now, connected),
                            mq137 = state.mq137.recheckFreshness(now, connected),
                            mq4 = state.mq4.recheckFreshness(now, connected),
                            device = state.device.recheckFreshness(now, connected)
                        )
                    }
                }
        }
    }

    private fun collectSensor(key: SensorKey, startupDelayMs: Long = 0L) {
        viewModelScope.launch(Dispatchers.Default) {
            if (startupDelayMs > 0L) delay(startupDelayMs.milliseconds)
            GarbyRealtimeDb.sensorValue(deviceId, key).collect { state ->
                when (state) {
                    SensorUiState.Loading -> updateSensor(
                        key,
                        SensorReadingUiState(isLoading = true)
                    )
                    is SensorUiState.Value -> updateSensor(
                        key,
                        SensorReadingUiState(
                            reading = state.data,
                            isLoading = false,
                            isStale = _uiState.value.connection !is ConnectionState.Connected ||
                                !isFreshTimestamp(state.data.updatedAtMs),
                            error = null
                        )
                    )
                    is SensorUiState.Error -> updateSensor(
                        key,
                        SensorReadingUiState(
                            reading = null,
                            isLoading = false,
                            isStale = true,
                            error = state.message
                        )
                    )
                }
            }
        }
    }

    private fun collectDeviceStatus() {
        viewModelScope.launch(Dispatchers.Default) {
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
                                error = "Device status unavailable"
                            )
                        )
                    }
                }
                .collect { status ->
                    _uiState.update {
                        it.copy(
                            device = DeviceStatusUiState(
                                status = status,
                                isLoading = false,
                                isStale = _uiState.value.connection !is ConnectionState.Connected ||
                                    !isFreshTimestamp(status.lastSeenMs),
                                error = null
                            )
                        )
                    }
                }
        }
    }

    private fun startFreshnessTicker() {
        viewModelScope.launch(Dispatchers.Default) {
            while (isActive) {
                delay(RtdbConstants.FRESHNESS_TICK_MS.milliseconds)
                val now = System.currentTimeMillis()
                _uiState.update { state ->
                    val connected = state.connection is ConnectionState.Connected
                    state.copy(
                        level = state.level.recheckFreshness(now, connected),
                        weight = state.weight.recheckFreshness(now, connected),
                        mq135 = state.mq135.recheckFreshness(now, connected),
                        mq137 = state.mq137.recheckFreshness(now, connected),
                        mq4 = state.mq4.recheckFreshness(now, connected),
                        device = state.device.recheckFreshness(now, connected)
                    )
                }
            }
        }
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

    private fun updateSensor(key: SensorKey, value: SensorReadingUiState) {
        _uiState.update { current ->
            when (key) {
                SensorKey.Level -> current.copy(level = value)
                SensorKey.Weight -> current.copy(weight = value)
                SensorKey.Mq135 -> current.copy(mq135 = value)
                SensorKey.Mq137 -> current.copy(mq137 = value)
                SensorKey.Mq4 -> current.copy(mq4 = value)
            }
        }
    }
}
