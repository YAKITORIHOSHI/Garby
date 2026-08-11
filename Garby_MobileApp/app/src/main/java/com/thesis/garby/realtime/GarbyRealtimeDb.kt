package com.thesis.garby.realtime

import android.util.Log
import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseError
import com.google.firebase.database.FirebaseDatabase
import com.google.firebase.database.ServerValue
import com.google.firebase.database.ValueEventListener
import com.thesis.garby.network.NetworkConnectivityManager
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.conflate
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.flowOn
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.mapNotNull
import kotlinx.coroutines.flow.onStart
import kotlinx.coroutines.flow.retryWhen
import kotlinx.coroutines.tasks.await
import kotlinx.coroutines.withTimeout
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.TimeZone
import kotlin.math.min
import kotlin.time.Duration.Companion.milliseconds

/**
 * Safely extracts a Double from a DataSnapshot child, handling various numeric types
 * (Double, Long, Integer, Float, String representations) that Firebase may store.
 */
private fun DataSnapshot.getDoubleOrNull(): Double? {
    return when (val value = this.value) {
        is Number -> value.toDouble()
        is String -> value.toDoubleOrNull()
        else -> null
    }
}

private val uptimeFormatSpecs = listOf(
    "yyyy-MM-dd HH:mm:ss",
    "yyyy-MM-dd'T'HH:mm:ss",
    "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'",
    "yyyy/MM/dd HH:mm:ss"
)

/** SimpleDateFormat is not thread-safe, so each parsing thread gets its own set. */
private val uptimeFormatters = object : ThreadLocal<List<SimpleDateFormat>>() {
    override fun initialValue(): List<SimpleDateFormat> =
        uptimeFormatSpecs.flatMap { fmt ->
            listOf(TimeZone.getDefault(), TimeZone.getTimeZone("UTC"))
                .map { tz -> SimpleDateFormat(fmt, Locale.US).apply { timeZone = tz } }
        }
}

private fun parseUptimeToMillis(dateStr: String): Long? {
    val trimmed = dateStr.trim()
    if (trimmed.isBlank()) return null

    for (sdf in uptimeFormatters.get()!!) {
        try {
            sdf.parse(trimmed)?.let { return it.time }
        } catch (_: Exception) {}
    }
    return null
}

/**
 * Safely extracts a Long from a DataSnapshot child, handling various numeric types.
 */
private fun DataSnapshot.getLongOrNull(): Long? {
    return when (val value = this.value) {
        is Number -> value.toLong()
        is String -> value.toLongOrNull()
        else -> null
    }
}

/**
 * Firebase RTDB gateway for GARBY's Android client.
 *
 * Monitoring streams are latest-value flows. Control writes are gated by both
 * Android's validated-network signal and Firebase's own `.info/connected`
 * signal. Unknown/malformed data never becomes a synthetic safe value.
 */
object GarbyRealtimeDb {

    private const val TAG = "GarbyRTDB"
    private const val MAX_RETRY_ATTEMPTS = 8
    private val validDeviceId = Regex("^[A-Za-z0-9_-]{1,64}$")

    private val database by lazy {
        try {
            FirebaseDatabase.getInstance(RtdbConstants.DATABASE_URL)
        } catch (_: Exception) {
            FirebaseDatabase.getInstance()
        }
    }

    private fun extractReadingFromSnapshot(
        snapshot: DataSnapshot,
        sensorKey: SensorKey
    ): SensorReading? {
        if (!snapshot.exists()) return null

        var rawVal: Double? = snapshot.child("value").getDoubleOrNull()
        if (rawVal == null) {
            rawVal = snapshot.getDoubleOrNull()
        }
        if (rawVal == null) {
            rawVal = snapshot.child(sensorKey.legacyValueKey).getDoubleOrNull()
        }
        if (rawVal == null) {
            rawVal = when (sensorKey) {
                SensorKey.Level -> snapshot.child("CM_DISTANCE").getDoubleOrNull()
                    ?: snapshot.child("distance").getDoubleOrNull()
                    ?: snapshot.child("level").getDoubleOrNull()
                SensorKey.Weight -> snapshot.child("weight").getDoubleOrNull()
                    ?: snapshot.child("kg").getDoubleOrNull()
                    ?: snapshot.child("WEIGHT_IN_KG").getDoubleOrNull()
                    ?: snapshot.child("loadcell").getDoubleOrNull()
                    ?: snapshot.child("reading").getDoubleOrNull()
                    ?: snapshot.child("val").getDoubleOrNull()
                SensorKey.Mq135 -> snapshot.child("AIR_QUALITY").getDoubleOrNull()
                    ?: snapshot.child("value").getDoubleOrNull()
                SensorKey.Mq137 -> snapshot.child("AMMONIA").getDoubleOrNull()
                    ?: snapshot.child("value").getDoubleOrNull()
                SensorKey.Mq4 -> snapshot.child("METHANE").getDoubleOrNull()
                    ?: snapshot.child("value").getDoubleOrNull()
            }
        }

        if (rawVal == null || !rawVal.isFinite()) return null

        val floatVal = rawVal.toFloat()
        val unit = snapshot.child("unit").getValue(String::class.java)?.ifBlank { null } ?: sensorKey.unit
        val sensorType = snapshot.child("sensorType").getValue(String::class.java) ?: snapshot.key ?: ""
        val rawUpdatedAt = snapshot.child("updatedAt").getLongOrNull()
            ?: snapshot.child("at").getLongOrNull()
            ?: snapshot.child("lastSeen").getLongOrNull()
            ?: 0L
        val isOfflineSentinel = when (sensorKey) {
            SensorKey.Level -> rawVal >= 999.0
            SensorKey.Weight -> false
            SensorKey.Mq135, SensorKey.Mq137, SensorKey.Mq4 -> rawVal < 0.0
        }
        val updatedAt = if (isOfflineSentinel) 0L else rawUpdatedAt

        return SensorReading(floatVal, unit, sensorType, updatedAt)
    }

    /**
     * Cold flow: a listener exists only while there is a collector, preventing
     * background cleanup from silently detaching an app-wide singleton listener.
     */
    val connectionState: Flow<ConnectionState>
        get() = callbackFlow {
            val ref = database.getReference(".info/connected")
            val listener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    trySend(snapshot)
                }

                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }
            ref.addValueEventListener(listener)
            awaitClose { ref.removeEventListener(listener) }
        }
            .map { snapshot ->
                val connected = snapshot.getValue(Boolean::class.java) == true
                if (connected) ConnectionState.Connected else ConnectionState.Reconnecting
            }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .catch { error ->
                if (error is CancellationException) throw error
                Log.w(TAG, "Connection-state listener stopped", error)
                emit(ConnectionState.Disconnected)
            }
            .conflate()
            .flowOn(Dispatchers.Default)

    private sealed interface SensorEvent {
        data class Raw(
            val primarySnapshot: DataSnapshot?,
            val legacySnapshot: DataSnapshot?,
            val bothLoaded: Boolean
        ) : SensorEvent
        data object Loading : SensorEvent
    }

    private fun SensorEvent.toReadingState(
        sensorKey: SensorKey
    ): SensorUiState<SensorReading> = when (this) {
        SensorEvent.Loading -> SensorUiState.Loading
        is SensorEvent.Raw -> {
            val primary = primarySnapshot?.let {
                extractReadingFromSnapshot(it, sensorKey)
            }
            val legacy = legacySnapshot?.let {
                extractReadingFromSnapshot(it, sensorKey)
            }
            // During a rolling deployment both paths may exist. Prefer the
            // newest timestamp, with the device path winning an equal/missing
            // timestamp tie. This prevents an older compatibility copy from
            // overriding fresh device telemetry.
            val reading = when {
                primary == null -> legacy
                legacy == null -> primary
                legacy.updatedAtMs > primary.updatedAtMs -> legacy
                else -> primary
            }
            if (reading != null) {
                SensorUiState.Value(reading)
            } else if (bothLoaded) {
                SensorUiState.Error("${sensorKey.displayName} unavailable")
            } else {
                SensorUiState.Loading
            }
        }
    }

    fun sensorValue(
        deviceId: String,
        sensorKey: SensorKey
    ): Flow<SensorUiState<SensorReading>> {
        requireValidDeviceId(deviceId)

        return callbackFlow {
            trySend(SensorEvent.Loading)

            val primaryRef = database.getReference(
                "devices/$deviceId/sensors/${sensorKey.storageKey}"
            )
            val legacyRef = database.getReference(sensorKey.legacyPath)
            val snapshotLock = Any()
            var primarySnapshot: DataSnapshot? = null
            var legacySnapshot: DataSnapshot? = null
            var primaryLoaded = false
            var legacyLoaded = false

            fun publishSnapshots() {
                val event = synchronized(snapshotLock) {
                    SensorEvent.Raw(
                        primarySnapshot = primarySnapshot,
                        legacySnapshot = legacySnapshot,
                        bothLoaded = primaryLoaded && legacyLoaded
                    )
                }
                trySend(event)
            }

            val primaryListener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    synchronized(snapshotLock) {
                        primarySnapshot = snapshot
                        primaryLoaded = true
                    }
                    publishSnapshots()
                }

                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }

            val legacyListener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    synchronized(snapshotLock) {
                        legacySnapshot = snapshot
                        legacyLoaded = true
                    }
                    publishSnapshots()
                }

                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }

            primaryRef.addValueEventListener(primaryListener)
            legacyRef.addValueEventListener(legacyListener)
            awaitClose {
                primaryRef.removeEventListener(primaryListener)
                legacyRef.removeEventListener(legacyListener)
            }
        }
            .onStart { emit(SensorEvent.Loading) }
            .map { it.toReadingState(sensorKey) }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .catch { error ->
                if (error is CancellationException) throw error
                Log.w(TAG, "${sensorKey.storageKey} stream failed", error)
                emit(SensorUiState.Error("${sensorKey.displayName} unavailable"))
            }
            .conflate()
            .flowOn(Dispatchers.Default)
    }

    @Suppress("unused")
    fun recentHistory(
        deviceId: String,
        sensorKey: SensorKey,
        limit: Int = RtdbConstants.MAX_HISTORY
    ): Flow<List<HistoryPoint>> {
        requireValidDeviceId(deviceId)
        val boundedLimit = limit.coerceIn(1, RtdbConstants.MAX_HISTORY)

        return callbackFlow {
            val ref = database
                .getReference("devices/$deviceId/sensors/${sensorKey.storageKey}/history")
                .limitToLast(boundedLimit)
            val listener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    val points = snapshot.children.mapNotNull { child ->
                        val value = child.child("value").getDoubleOrNull()?.toFloat()
                            ?: child.getDoubleOrNull()?.toFloat()
                            ?: return@mapNotNull null
                        val at = child.child("at").getLongOrNull()
                            ?: System.currentTimeMillis()
                        if (!value.isFinite() || at <= 0L) return@mapNotNull null
                        HistoryPoint(value, at)
                    }
                    trySend(points)
                }

                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }
            ref.addValueEventListener(listener)
            awaitClose { ref.removeEventListener(listener) }
        }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .conflate()
            .flowOn(Dispatchers.Default)
    }

    private data class DeviceStatusSnapshots(
        val raspiStates: DataSnapshot?,
        val deviceStatus: DataSnapshot?
    )

    fun deviceStatus(deviceId: String): Flow<DeviceStatus> {
        requireValidDeviceId(deviceId)

        return callbackFlow {
            val raspiRef = database.getReference("RASPI/STATES")
            val deviceRef = database.getReference("devices/$deviceId/status")
            val snapshotLock = Any()
            var raspiSnapshot: DataSnapshot? = null
            var deviceSnapshot: DataSnapshot? = null

            fun publishSnapshots() {
                val event = synchronized(snapshotLock) {
                    DeviceStatusSnapshots(raspiSnapshot, deviceSnapshot)
                }
                trySend(event)
            }

            val raspiListener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    synchronized(snapshotLock) { raspiSnapshot = snapshot }
                    publishSnapshots()
                }

                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }
            val deviceListener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    synchronized(snapshotLock) { deviceSnapshot = snapshot }
                    publishSnapshots()
                }

                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }

            raspiRef.addValueEventListener(raspiListener)
            deviceRef.addValueEventListener(deviceListener)
            awaitClose {
                raspiRef.removeEventListener(raspiListener)
                deviceRef.removeEventListener(deviceListener)
            }
        }
            .mapNotNull { snapshots ->
                extractDeviceStatus(snapshots.raspiStates, snapshots.deviceStatus)
            }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .conflate()
            .flowOn(Dispatchers.Default)
    }

    private fun extractDeviceStatus(
        raspiStates: DataSnapshot?,
        deviceStatusSnap: DataSnapshot?
    ): DeviceStatus? {
        if (raspiStates?.exists() != true && deviceStatusSnap?.exists() != true) {
            // Wait for both initial callbacks, then represent a genuinely
            // absent robot record as offline instead of an endless spinner.
            if (raspiStates == null || deviceStatusSnap == null) return null
            return DeviceStatus(
                online = false,
                lastSeenMs = 0L,
                batteryPercent = null,
                wifiRssi = null
            )
        }

        val rawUptimeStr = raspiStates?.child("recent_uptime")?.getValue(String::class.java)
            ?: deviceStatusSnap?.child("recent_uptime")?.getValue(String::class.java)
            ?: raspiStates?.child("launchTime")?.getValue(String::class.java)

        val parsedMs = rawUptimeStr?.let { parseUptimeToMillis(it) }

        val lastSeenMs = raspiStates?.child("lastSeen")?.getLongOrNull()
            ?: deviceStatusSnap?.child("lastSeen")?.getLongOrNull()
            ?: deviceStatusSnap?.child("lastSeenMs")?.getLongOrNull()
            ?: parsedMs
            ?: 0L

        val now = System.currentTimeMillis()
        val ageMs = now - lastSeenMs
        val isOnline = ageMs in 0L..RtdbConstants.ROBOT_UPTIME_TIMEOUT_MS

        val battery = deviceStatusSnap?.child("batteryPercent")?.getLongOrNull()?.toInt()
            ?: deviceStatusSnap?.child("battery")?.getLongOrNull()?.toInt()

        val rssi = deviceStatusSnap?.child("wifiRssi")?.getLongOrNull()?.toInt()
            ?: deviceStatusSnap?.child("rssi")?.getLongOrNull()?.toInt()

        val cpuTemperatureC = deviceStatusSnap?.child("cpuTemperatureC")
            ?.getDoubleOrNull()
            ?.takeIf { it.isFinite() }
            ?.toFloat()
        val thermalWarning = deviceStatusSnap?.child("thermalWarning")
            ?.getValue(Boolean::class.java)
        val throttledFlags = deviceStatusSnap?.child("throttledFlags")?.getLongOrNull()
        val bleConnected = deviceStatusSnap?.child("bleConnected")
            ?.getValue(Boolean::class.java)
        val lidarHealthy = deviceStatusSnap?.child("lidarHealthy")
            ?.getValue(Boolean::class.java)
        val sensorSerialConnected = deviceStatusSnap?.child("sensorSerialConnected")
            ?.getValue(Boolean::class.java)

        return DeviceStatus(
            online = isOnline,
            lastSeenMs = lastSeenMs,
            batteryPercent = battery,
            wifiRssi = rssi,
            recentUptimeStr = rawUptimeStr,
            cpuTemperatureC = cpuTemperatureC,
            thermalWarning = thermalWarning,
            throttledFlags = throttledFlags,
            bleConnected = bleConnected,
            lidarHealthy = lidarHealthy,
            sensorSerialConnected = sensorSerialConnected
        )
    }

    /**
     * Sends a reset intent after proving that the client and Firebase are online.
     *
     * The returned marker lets the caller ignore an old `done`/`failed` value
     * that belonged to an earlier request. Outstanding writes are purged before
     * a new reset and whenever the write fails/times out to reduce the risk of a
     * delayed control command being replayed when connectivity returns.
     */
    suspend fun requestReset(
        deviceId: String,
        requestedBy: String
    ): Result<ResetRequestMarker> {
        requireValidDeviceId(deviceId)
        require(requestedBy.isNotBlank()) { "Authenticated requester is required" }

        try {
            check(NetworkConnectivityManager.isNetworkConnected()) {
                "A validated internet connection is required for reset"
            }

            waitForFirebaseConnection()

            // This Android client only writes reset commands. Clear any older
            // timed-out write before issuing a new safety-relevant request.
            database.purgeOutstandingWrites()

            val resetRef = database.getReference("devices/$deviceId/commands/reset")
            val previousRequestedAt = withTimeout(RtdbConstants.WRITE_TIMEOUT_MS.milliseconds) {
                resetRef.get().await()
                    .child("requestedAt")
                    .getLongOrNull()
                    ?: 0L
            }

            val marker = ResetRequestMarker(
                requestedBy = requestedBy,
                previousRequestedAtMs = previousRequestedAt
            )

            withTimeout(RtdbConstants.WRITE_TIMEOUT_MS.milliseconds) {
                // One atomic multi-location write prevents the structured
                // command and deployed APP compatibility flag from briefly
                // contradicting each other.
                database.reference.updateChildren(
                    mapOf(
                        "devices/$deviceId/commands/reset/requestedAt" to ServerValue.TIMESTAMP,
                        "devices/$deviceId/commands/reset/requestedBy" to requestedBy,
                        "devices/$deviceId/commands/reset/status" to "pending",
                        "APP/isReadyToReset" to true
                    )
                ).await()
            }
            return Result.success(marker)
        } catch (error: kotlinx.coroutines.TimeoutCancellationException) {
            database.purgeOutstandingWrites()
            Log.w(TAG, "Reset request timed out", error)
            return Result.failure(error)
        } catch (error: CancellationException) {
            database.purgeOutstandingWrites()
            throw error
        } catch (error: Exception) {
            database.purgeOutstandingWrites()
            Log.w(TAG, "Reset request failed", error)
            return Result.failure(error)
        }
    }

    private data class ResetSnapshots(
        val structured: DataSnapshot?,
        val appReady: Boolean?
    )

    fun resetStatus(deviceId: String): Flow<ResetCommand> {
        requireValidDeviceId(deviceId)

        return callbackFlow {
            val resetRef = database.getReference("devices/$deviceId/commands/reset")
            val appReadyRef = database.getReference("APP/isReadyToReset")
            val snapshotLock = Any()
            var resetSnapshot: DataSnapshot? = null
            var appReady: Boolean? = null

            fun publishSnapshots() {
                val event = synchronized(snapshotLock) {
                    ResetSnapshots(resetSnapshot, appReady)
                }
                trySend(event)
            }

            val resetListener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    synchronized(snapshotLock) { resetSnapshot = snapshot }
                    publishSnapshots()
                }

                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }
            val appReadyListener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    synchronized(snapshotLock) {
                        appReady = snapshot.getValue(Boolean::class.java)
                    }
                    publishSnapshots()
                }

                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }

            resetRef.addValueEventListener(resetListener)
            appReadyRef.addValueEventListener(appReadyListener)
            awaitClose {
                resetRef.removeEventListener(resetListener)
                appReadyRef.removeEventListener(appReadyListener)
            }
        }
            .map { snapshots ->
                extractResetCommand(snapshots.structured, snapshots.appReady)
            }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .conflate()
            .flowOn(Dispatchers.Default)
    }

    private fun extractResetCommand(
        resetSnap: DataSnapshot?,
        appReadyReset: Boolean?
    ): ResetCommand {
        var at = resetSnap?.child("requestedAt")?.getLongOrNull() ?: 0L
        val by = resetSnap?.child("requestedBy")?.getValue(String::class.java).orEmpty()
        val statusStr = resetSnap?.child("status")?.getValue(String::class.java)
        val structuredResetExists = resetSnap?.exists() == true
        if (appReadyReset == true && !structuredResetExists) {
            if (at <= 0L) at = System.currentTimeMillis()
        }

        val status = resolveResetStatus(statusStr, appReadyReset, structuredResetExists)
        return ResetCommand(at, by, status)
    }

    fun isReadyToReturn(deviceId: String = RtdbConstants.DEFAULT_DEVICE_ID): Flow<Boolean> {
        requireValidDeviceId(deviceId)
        return callbackFlow {
            val legacyRef = database.getReference("RASPI/STATES/isReadyToReturn")
            val deviceRef = database.getReference("devices/$deviceId/status/isReadyToReturn")
            val valueLock = Any()
            var legacyReady: Boolean? = null
            var deviceReady: Boolean? = null

            fun publishValue() {
                val ready = synchronized(valueLock) { deviceReady ?: legacyReady ?: false }
                trySend(ready)
            }

            val legacyListener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    synchronized(valueLock) {
                        legacyReady = snapshot.getValue(Boolean::class.java)
                    }
                    publishValue()
                }

                override fun onCancelled(error: DatabaseError) {
                    trySend(false)
                }
            }
            val deviceListener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    synchronized(valueLock) {
                        deviceReady = snapshot.getValue(Boolean::class.java)
                    }
                    publishValue()
                }

                override fun onCancelled(error: DatabaseError) {
                    trySend(false)
                }
            }

            legacyRef.addValueEventListener(legacyListener)
            deviceRef.addValueEventListener(deviceListener)
            awaitClose {
                legacyRef.removeEventListener(legacyListener)
                deviceRef.removeEventListener(deviceListener)
            }
        }.conflate()
    }

    fun saveFcmToken(token: String, deviceId: String = RtdbConstants.DEFAULT_DEVICE_ID) {
        if (token.isBlank()) return
        requireValidDeviceId(deviceId)
        try {
            database.reference.updateChildren(
                mapOf(
                    "APP/fcmToken" to token,
                    "devices/$deviceId/fcmToken" to token,
                    "RASPI/fcmToken" to token
                )
            ).addOnFailureListener { error ->
                Log.w(TAG, "Failed to save FCM token to RTDB", error)
            }
        } catch (e: Exception) {
            Log.w(TAG, "Failed to save FCM token to RTDB", e)
        }
    }

    /**
     * Warm-up hook called on a background thread before dashboard routing.
     *
     * Forces the lazy [FirebaseDatabase] initialization and waits for the actual
     * socket connection to be established. This ensures that when the dashboard
     * mounts, the sensors can start loading immediately without a transient
     * "unavailable" flicker or animation freeze during transition.
     */
    suspend fun prewarm() {
        try {
            // Wait specifically for a Connected state, not just any emission.
            withTimeout(RtdbConstants.FIREBASE_CONNECT_TIMEOUT_MS.milliseconds) {
                connectionState.first { it is ConnectionState.Connected }
            }
            // Allow a small window for the initial RTDB sync to populate the local cache.
            delay(300.milliseconds)
        } catch (error: CancellationException) {
            throw error
        } catch (error: Exception) {
            Log.w(TAG, "prewarm failed or timed out", error)
        }
    }

    private suspend fun waitForFirebaseConnection() {
        withTimeout(RtdbConstants.FIREBASE_CONNECT_TIMEOUT_MS.milliseconds) {
            connectionState.first { it is ConnectionState.Connected }
        }
    }

    private fun requireValidDeviceId(deviceId: String) {
        require(validDeviceId.matches(deviceId)) { "Invalid device id" }
    }

    private fun Throwable.isTransientRtdbFailure(): Boolean {
        val error = this as? RtdbCancelledException ?: return false
        return error.code == DatabaseError.DISCONNECTED ||
            error.code == DatabaseError.NETWORK_ERROR ||
            error.code == DatabaseError.UNAVAILABLE
    }

    private fun <T> Flow<T>.retryTransientWithJitter(maxAttempts: Int): Flow<T> =
        retryWhen { cause, attempt ->
            if (!cause.isTransientRtdbFailure() || attempt >= maxAttempts.toLong()) {
                false
            } else {
                val exponent = min(attempt.toInt(), 4)
                val baseMs = 750L * (1L shl exponent)
                val jitterMs = (baseMs * 0.25).toLong()
                val delayMs = baseMs + kotlin.random.Random.nextLong(-jitterMs, jitterMs + 1L)
                Log.d(TAG, "Transient RTDB retry ${attempt + 1}/$maxAttempts in ${delayMs}ms")
                delay(delayMs.coerceAtLeast(250L).milliseconds)
                true
            }
        }

    private class RtdbCancelledException(
        val code: Int,
        message: String
    ) : Exception(message)

}
