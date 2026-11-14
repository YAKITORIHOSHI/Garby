package com.thesis.garby.realtime

import android.util.Log
import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseError
import com.google.firebase.database.MutableData
import com.google.firebase.database.Transaction
import com.google.firebase.database.FirebaseDatabase
import com.google.firebase.database.ServerValue
import com.google.firebase.database.ValueEventListener
import com.thesis.garby.network.NetworkConnectivityManager
import com.thesis.garby.performance.GarbyDispatchers
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.conflate
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.flow.flowOn
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.mapNotNull
import kotlinx.coroutines.flow.onStart
import kotlinx.coroutines.flow.retryWhen
import kotlinx.coroutines.tasks.await
import kotlinx.coroutines.suspendCancellableCoroutine
import kotlinx.coroutines.withTimeout
import java.text.SimpleDateFormat
import java.util.Locale
import java.util.TimeZone
import java.util.UUID
import kotlin.coroutines.resume
import kotlin.coroutines.resumeWithException
import kotlin.math.min
import kotlin.time.Duration.Companion.milliseconds

/**
 * Typed reads that never throw on unexpected types. Firebase's `getValue(Boolean::class.java)`
 * throws when a field holds, say, a string, which used to turn one malformed field into a
 * failed stream.
 */
private fun DataSnapshot.getDoubleOrNull(): Double? = when (val value = this.value) {
    is Number -> value.toDouble()
    is String -> value.trim().toDoubleOrNull()
    else -> null
}

private fun DataSnapshot.getLongOrNull(): Long? = when (val value = this.value) {
    is Number -> value.toLong()
    is String -> value.trim().toLongOrNull() ?: value.trim().toDoubleOrNull()?.toLong()
    else -> null
}

private fun DataSnapshot.getBooleanOrNull(): Boolean? = when (val value = this.value) {
    is Boolean -> value
    is String -> value.trim().lowercase().toBooleanStrictOrNull()
    else -> null
}

private fun DataSnapshot.getStringOrNull(): String? = (value as? String)?.takeIf { it.isNotBlank() }

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
        } catch (_: Exception) {
        }
    }
    return null
}

/** Outcome of one return-to-base submission. */
sealed interface ReturnSubmission {
    /** Firebase confirmed the write and issued this request identity. */
    data class Sent(val request: ReturnRequestRef) : ReturnSubmission

    /** Nothing was written. */
    data class NotSent(val message: String) : ReturnSubmission

    /** A write was attempted but not confirmed; the request may or may not exist. */
    data class Unconfirmed(val message: String) : ReturnSubmission
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
    private const val MAX_SERVER_OFFSET_MS = 86_400_000L
    private val validDeviceId = Regex("^[A-Za-z0-9_-]{1,64}$")

    private val monitoringScope by lazy { CoroutineScope(SupervisorJob() + GarbyDispatchers.background) }
    private val sharedConnection by lazy { observeConnectionState().shareWhileObserved(monitoringScope) }
    private val sharedDeviceStatus by lazy {
        observeDeviceStatus(RtdbConstants.DEFAULT_DEVICE_ID).shareWhileObserved(monitoringScope)
    }

    /** Latest `.info/serverTimeOffset`, refreshed while [serverTimeOffsetMs] is collected. */
    @Volatile
    private var knownServerOffsetMs: Long = 0L

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

        val rawVal: Double = snapshot.child("value").getDoubleOrNull()
            ?: snapshot.getDoubleOrNull()
            ?: snapshot.child(sensorKey.legacyValueKey).getDoubleOrNull()
            ?: when (sensorKey) {
                SensorKey.Level -> snapshot.child("distance").getDoubleOrNull()
                    ?: snapshot.child("level").getDoubleOrNull()
                SensorKey.Weight -> snapshot.child("weight").getDoubleOrNull()
                    ?: snapshot.child("kg").getDoubleOrNull()
                SensorKey.Mq135, SensorKey.Mq137, SensorKey.Mq4 -> null
            }
            ?: return null

        if (!rawVal.isFinite()) return null

        val unit = snapshot.child("unit").getStringOrNull() ?: sensorKey.unit
        val sensorType = snapshot.child("sensorType").getStringOrNull() ?: snapshot.key.orEmpty()
        val rawUpdatedAt = snapshot.child("updatedAt").getLongOrNull()
            ?: snapshot.child("at").getLongOrNull()
            ?: snapshot.child("lastSeen").getLongOrNull()
            ?: 0L
        // The Pi publishes 999 (bin distance) and -1 (weight, gases) when a sensor is unavailable.
        val isOfflineSentinel = when (sensorKey) {
            SensorKey.Level -> rawVal >= 999.0
            SensorKey.Weight, SensorKey.Mq135, SensorKey.Mq137, SensorKey.Mq4 -> rawVal < 0.0
        }
        val updatedAt = if (isOfflineSentinel) 0L else rawUpdatedAt

        return SensorReading(rawVal.toFloat(), unit, sensorType, updatedAt)
    }

    /** One connection listener shared by the dashboard and return workflow while observed. */
    val connectionState: Flow<ConnectionState>
        get() = sharedConnection

    private fun observeConnectionState(): Flow<ConnectionState> = callbackFlow {
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
            .conflate()
            .map { snapshot ->
                if (snapshot.getBooleanOrNull() == true) ConnectionState.Connected else ConnectionState.Reconnecting
            }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .catch { error ->
                if (error is CancellationException) throw error
                Log.w(TAG, "Connection-state listener stopped", error)
                emit(ConnectionState.Disconnected)
            }
            .distinctUntilChanged()
            .conflate()
            .flowOn(GarbyDispatchers.background)

    /**
     * Firebase's estimate of (server clock - phone clock). Request timestamps are server time, so
     * the return screen adds this before measuring how long a request has been waiting.
     */
    val serverTimeOffsetMs: Flow<Long>
        get() = callbackFlow {
            val ref = database.getReference(".info/serverTimeOffset")
            val listener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    val offset = snapshot.getDoubleOrNull()
                        ?.takeIf { it.isFinite() }
                        ?.toLong()
                        ?.coerceIn(-MAX_SERVER_OFFSET_MS, MAX_SERVER_OFFSET_MS)
                        ?: 0L
                    knownServerOffsetMs = offset
                    trySend(offset)
                }

                override fun onCancelled(error: DatabaseError) {
                    trySend(knownServerOffsetMs)
                }
            }
            ref.addValueEventListener(listener)
            awaitClose { ref.removeEventListener(listener) }
        }
            .distinctUntilChanged()
            .conflate()
            .flowOn(GarbyDispatchers.background)

    /** The phone clock corrected by the last known Firebase server offset. */
    fun serverNowMs(): Long = System.currentTimeMillis() + knownServerOffsetMs

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
            // A present device record owns the reading, including unavailable
            // sentinels. Legacy snapshots are retained only while it is absent.
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

            // The device schema is authoritative when present (including invalid
            // or unavailable values). Only absent records subscribe to legacy.
            var legacyAttached = false
            var closed = false
            var legacyListener: ValueEventListener? = null

            fun detachLegacy() {
                legacyAttached = false
                legacyListener?.let { legacyRef.removeEventListener(it) }
                legacyListener = null
                legacySnapshot = null
                legacyLoaded = false
            }

            fun attachLegacy() {
                if (legacyAttached || closed) return
                legacyAttached = true
                legacyLoaded = false
                val listener = object : ValueEventListener {
                    override fun onDataChange(snapshot: DataSnapshot) {
                        synchronized(snapshotLock) {
                            if (closed || legacyListener !== this) return
                            legacySnapshot = snapshot
                            legacyLoaded = true
                            publishSnapshots()
                        }
                    }

                    override fun onCancelled(error: DatabaseError) {
                        synchronized(snapshotLock) {
                            if (!closed && legacyListener === this) {
                                close(RtdbCancelledException(error.code, error.message))
                            }
                        }
                    }
                }
                legacyListener = listener
                legacyRef.addValueEventListener(listener)
            }

            val primaryListener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) {
                    synchronized(snapshotLock) {
                        if (closed) return
                        primarySnapshot = snapshot
                        primaryLoaded = true
                        if (snapshot.exists()) {
                            detachLegacy()
                            legacyLoaded = true // no fallback is needed
                        } else {
                            attachLegacy()
                        }
                        publishSnapshots()
                    }
                }

                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }

            primaryRef.addValueEventListener(primaryListener)
            awaitClose {
                synchronized(snapshotLock) {
                    closed = true
                    primaryRef.removeEventListener(primaryListener)
                    detachLegacy()
                }
            }
        }
            // Fuse with callbackFlow's channel BEFORE decoding. A slow low-end CPU retains
            // the newest snapshot instead of a 64-element backlog of obsolete snapshots.
            .conflate()
            .onStart { emit(SensorEvent.Loading) }
            .map { it.toReadingState(sensorKey) }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .catch { error ->
                if (error is CancellationException) throw error
                Log.w(TAG, "${sensorKey.storageKey} stream failed", error)
                emit(SensorUiState.Error("${sensorKey.displayName} unavailable"))
            }
            .distinctUntilChanged()
            .conflate()
            .flowOn(GarbyDispatchers.background)
    }

    private data class DeviceStatusSnapshots(
        val raspiStates: DataSnapshot?,
        val deviceStatus: DataSnapshot?
    )

    /**
     * The Pi's status: `devices/<id>/status` (current schema) with `RASPI/STATES` as the legacy
     * fallback. Both are needed before an absent record is reported as never seen.
     */
    fun deviceStatus(deviceId: String = RtdbConstants.DEFAULT_DEVICE_ID): Flow<DeviceStatus> {
        requireValidDeviceId(deviceId)
        return if (deviceId == RtdbConstants.DEFAULT_DEVICE_ID) sharedDeviceStatus else observeDeviceStatus(deviceId)
    }

    private fun observeDeviceStatus(deviceId: String): Flow<DeviceStatus> {
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
            .conflate()
            .mapNotNull { snapshots ->
                extractDeviceStatus(snapshots.raspiStates, snapshots.deviceStatus)
            }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .distinctUntilChanged()
            .conflate()
            .flowOn(GarbyDispatchers.background)
    }

    private fun extractDeviceStatus(
        raspiStates: DataSnapshot?,
        deviceStatusSnap: DataSnapshot?
    ): DeviceStatus? {
        val device = deviceStatusSnap?.takeIf { it.exists() }
        val legacy = raspiStates?.takeIf { it.exists() }
        if (device == null && legacy == null) {
            // Wait for both initial callbacks, then represent a genuinely
            // absent robot record as never seen instead of an endless spinner.
            if (raspiStates == null || deviceStatusSnap == null) return null
            return DeviceStatus(lastSeenMs = 0L)
        }

        val uptime = device?.child("recent_uptime")?.getStringOrNull()
            ?: legacy?.child("recent_uptime")?.getStringOrNull()

        // Both heartbeats are written in one update; take the newer in case one is left over.
        val lastSeenMs = listOfNotNull(
            device?.child("lastSeen")?.getLongOrNull(),
            legacy?.child("lastSeen")?.getLongOrNull()
        ).maxOrNull()
            ?: uptime?.let { parseUptimeToMillis(it) }
            ?: 0L

        return DeviceStatus(
            lastSeenMs = lastSeenMs,
            robotState = RobotState.fromWire(device?.child("robotState")?.getStringOrNull()),
            isReadyToReturn = device?.child("isReadyToReturn")?.getBooleanOrNull()
                ?: legacy?.child("isReadyToReturn")?.getBooleanOrNull(),
            recentUptimeStr = uptime,
            cpuTemperatureC = device?.child("cpuTemperatureC")
                ?.getDoubleOrNull()
                ?.takeIf { it.isFinite() }
                ?.toFloat(),
            thermalWarning = device?.child("thermalWarning")?.getBooleanOrNull(),
            throttledFlags = device?.child("throttledFlags")?.getLongOrNull(),
            bleConnected = device?.child("bleConnected")?.getBooleanOrNull(),
            lidarHealthy = device?.child("lidarHealthy")?.getBooleanOrNull(),
            sensorSerialConnected = device?.child("sensorSerialConnected")?.getBooleanOrNull(),
            manualDisposalReady = device?.child("manualDisposalReady")?.getBooleanOrNull(),
            manualDisposalUpdatedAtMs = device?.child("manualDisposalUpdatedAt")?.getLongOrNull() ?: 0L
        )
    }

    /** The latest return request in `devices/<id>/commands/reset`, or null when there is none. */
    fun returnCommand(deviceId: String = RtdbConstants.DEFAULT_DEVICE_ID): Flow<ReturnCommand?> {
        requireValidDeviceId(deviceId)

        return callbackFlow {
            val ref = database.getReference("devices/$deviceId/commands/reset")
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
            .conflate()
            .map { extractReturnCommand(it) }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .distinctUntilChanged()
            .conflate()
            .flowOn(GarbyDispatchers.background)
    }

    private fun extractReturnCommand(snapshot: DataSnapshot): ReturnCommand? {
        if (!snapshot.exists()) return null
        return ReturnCommand(
            requestedAtMs = snapshot.child("requestedAt").getLongOrNull() ?: 0L,
            requestedBy = snapshot.child("requestedBy").getStringOrNull().orEmpty(),
            status = ReturnStatus.fromWire(snapshot.child("status").getStringOrNull()),
            ackAtMs = snapshot.child("ackAt").getLongOrNull()?.takeIf { it > 0L },
            doneAtMs = snapshot.child("doneAt").getLongOrNull()?.takeIf { it > 0L },
            failedAtMs = snapshot.child("failedAt").getLongOrNull()?.takeIf { it > 0L },
            reason = snapshot.child("reason").getStringOrNull()
        )
    }

    /** One conflated listener, owned by the visible dashboard. */
    fun disposalCommand(deviceId: String = RtdbConstants.DEFAULT_DEVICE_ID): Flow<DisposalCommand?> {
        requireValidDeviceId(deviceId)
        return callbackFlow {
            val ref = database.getReference("devices/$deviceId/commands/dispose")
            val listener = object : ValueEventListener {
                override fun onDataChange(snapshot: DataSnapshot) { trySend(snapshot) }
                override fun onCancelled(error: DatabaseError) {
                    close(RtdbCancelledException(error.code, error.message))
                }
            }
            ref.addValueEventListener(listener)
            awaitClose { ref.removeEventListener(listener) }
        }.conflate().map { extractDisposalCommand(it) }
            .retryTransientWithJitter(MAX_RETRY_ATTEMPTS)
            .distinctUntilChanged().conflate().flowOn(GarbyDispatchers.background)
    }

    private fun extractDisposalCommand(snapshot: DataSnapshot): DisposalCommand? {
        if (!snapshot.exists()) return null
        return DisposalCommand(
            requestId = snapshot.child("requestId").getStringOrNull().orEmpty(),
            requestedAtMs = snapshot.child("requestedAt").getLongOrNull() ?: 0L,
            requestedBy = snapshot.child("requestedBy").getStringOrNull().orEmpty(),
            status = DisposalStatus.fromWire(snapshot.child("status").getStringOrNull()),
            reason = snapshot.child("reason").getStringOrNull()
        )
    }

    /** Atomic submission prevents two phones from replacing an unresolved disposal request. */
    suspend fun requestDisposal(deviceId: String, requestedBy: String): DisposalSubmission {
        requireValidDeviceId(deviceId)
        require(requestedBy.isNotBlank())
        var writeAttempted = false
        val requestId = UUID.randomUUID().toString().replace("-", "")
        try {
            return withTimeout(RtdbConstants.WRITE_TIMEOUT_MS.milliseconds) {
                if (!NetworkConnectivityManager.isNetworkConnected()) {
                    return@withTimeout DisposalSubmission.NotSent("This phone is offline. Nothing was sent.")
                }
                waitForFirebaseConnection()
                database.purgeOutstandingWrites()
                val statusSnapshot = database.getReference("devices/$deviceId/status").get().await()
                val ref = database.getReference("devices/$deviceId/commands/dispose")
                val current = extractDisposalCommand(ref.get().await())
                val block = disposalBlockReason(
                    extractDeviceStatus(null, statusSnapshot), true,
                    NetworkConnectivityManager.isNetworkConnected(),
                    DisposalUiState(command = current, loaded = true), System.currentTimeMillis()
                )
                if (block != null) return@withTimeout DisposalSubmission.NotSent("$block Nothing was sent.")
                val expectedId = current?.requestId
                writeAttempted = true
                suspendCancellableCoroutine<DisposalSubmission> { continuation ->
                    ref.runTransaction(object : Transaction.Handler {
                        override fun doTransaction(data: MutableData): Transaction.Result {
                            if (!continuation.isActive || !NetworkConnectivityManager.isNetworkConnected() ||
                                !canReplaceDisposal(expectedId, data.child("requestId").value as? String,
                                    data.child("status").value as? String, data.value != null)) return Transaction.abort()
                            data.value = mapOf(
                                "version" to 1,
                                "requestId" to requestId,
                                "requestedAt" to ServerValue.TIMESTAMP,
                                "requestedBy" to requestedBy,
                                "status" to "pending"
                            )
                            return Transaction.success(data)
                        }

                        override fun onComplete(error: DatabaseError?, committed: Boolean, snapshot: DataSnapshot?) {
                            if (!continuation.isActive) return
                            if (error != null) continuation.resumeWithException(error.toException())
                            else {
                                val written = snapshot?.let { extractDisposalCommand(it) }
                                continuation.resume(when {
                                    !committed -> DisposalSubmission.NotSent(
                                        "Request status changed. Refresh and try again; nothing was sent.")
                                    written != null && written.requestId == requestId &&
                                        written.requestedBy == requestedBy && written.requestedAtMs > 0L ->
                                        DisposalSubmission.Sent(written)
                                    else -> DisposalSubmission.Unconfirmed(
                                        "Request identity was not confirmed. Check GARBY before trying again.", requestId)
                                })
                            }
                        }
                    }, false) // Never render a local speculative command as accepted by the server.
                }
            }
        } catch (error: CancellationException) {
            database.purgeOutstandingWrites()
            if (error !is TimeoutCancellationException) throw error
        } catch (error: Exception) {
            database.purgeOutstandingWrites()
            Log.w(TAG, "Manual disposal submission failed", error)
        }
        return if (writeAttempted) DisposalSubmission.Unconfirmed(
            "Delivery not confirmed. Check GARBY and the request status before trying again.", requestId
        ) else DisposalSubmission.NotSent("The request could not be prepared. Nothing was sent.")
    }

    /**
     * Sends a return-to-base request after proving that the client and Firebase are online and
     * re-reading the robot's readiness from the server.
     *
     * The Pi relays any fresh pending request to the executor, and a request that arrives while
     * the robot is still driving outbound latches a route fault, so readiness is checked again
     * here against the server copy rather than trusting the screen's last update.
     *
     * Outstanding writes are purged before a new request and whenever the write fails or times
     * out, so a delayed control write cannot be replayed when connectivity returns.
     */
    suspend fun requestReturn(
        deviceId: String,
        requestedBy: String
    ): ReturnSubmission {
        requireValidDeviceId(deviceId)
        require(requestedBy.isNotBlank()) { "Authenticated requester is required" }

        var writeAttempted = false
        try {
            if (!NetworkConnectivityManager.isNetworkConnected()) {
                return ReturnSubmission.NotSent("This phone has no validated internet connection.")
            }

            waitForFirebaseConnection()

            // This Android client only writes return commands (and its FCM token). Clear any
            // older timed-out write before issuing a new safety-relevant request.
            database.purgeOutstandingWrites()

            val resetRef = database.getReference("devices/$deviceId/commands/reset")
            val (status, current) = withTimeout(RtdbConstants.WRITE_TIMEOUT_MS.milliseconds) {
                val deviceSnapshot = database.getReference("devices/$deviceId/status").get().await()
                val legacySnapshot = database.getReference("RASPI/STATES").get().await()
                val commandSnapshot = resetRef.get().await()
                extractDeviceStatus(legacySnapshot, deviceSnapshot) to extractReturnCommand(commandSnapshot)
            }

            val serverNow = serverNowMs()
            val inFlight = resolveReturnProgress(current, null, serverNow)?.inFlight == true
            val readiness = evaluateReturnReadiness(
                status = status,
                cloudConnected = true,
                internetAvailable = true,
                returnInFlight = inFlight,
                nowMs = System.currentTimeMillis()
            )
            readiness.firstUnmet?.let { unmet ->
                return ReturnSubmission.NotSent(notReadyMessage(unmet, status))
            }

            val previousRequestedAt = current?.requestedAtMs ?: 0L
            val requestedAt = withTimeout(RtdbConstants.WRITE_TIMEOUT_MS.milliseconds) {
                // Structured commands are authoritative; prior outcome fields are cleared so an
                // old ACK or result can never describe this request.
                writeAttempted = true
                database.reference.updateChildren(
                    mapOf(
                        "devices/$deviceId/commands/reset/requestedAt" to ServerValue.TIMESTAMP,
                        "devices/$deviceId/commands/reset/requestedBy" to requestedBy,
                        "devices/$deviceId/commands/reset/status" to "pending",
                        "devices/$deviceId/commands/reset/ackAt" to null,
                        "devices/$deviceId/commands/reset/doneAt" to null,
                        "devices/$deviceId/commands/reset/failedAt" to null,
                        "devices/$deviceId/commands/reset/reason" to null
                    )
                ).await()
                val written = resetRef.get().await()
                check(written.child("requestedBy").getStringOrNull() == requestedBy) {
                    "Return request was replaced before it could be confirmed"
                }
                val resolvedAt = written.child("requestedAt").getLongOrNull() ?: 0L
                check(resolvedAt > previousRequestedAt) { "Return identity could not be confirmed" }
                resolvedAt
            }
            return ReturnSubmission.Sent(ReturnRequestRef(requestedAt, requestedBy))
        } catch (error: TimeoutCancellationException) {
            database.purgeOutstandingWrites()
            Log.w(TAG, "Return request timed out", error)
            return if (writeAttempted) {
                ReturnSubmission.Unconfirmed("Firebase did not confirm the request in time.")
            } else {
                ReturnSubmission.NotSent("Firebase did not respond in time. Nothing was sent.")
            }
        } catch (error: CancellationException) {
            database.purgeOutstandingWrites()
            throw error
        } catch (error: Exception) {
            database.purgeOutstandingWrites()
            Log.w(TAG, "Return request failed", error)
            return if (writeAttempted) {
                ReturnSubmission.Unconfirmed("Firebase did not confirm the request.")
            } else {
                ReturnSubmission.NotSent("The request could not be prepared. Nothing was sent.")
            }
        }
    }

    private fun notReadyMessage(unmet: ReturnCheck, status: DeviceStatus?): String = when (unmet) {
        ReturnCheck.AtDestination -> {
            val state = status?.robotState
            if (state != null && state != RobotState.AtDestination) {
                "GARBY is no longer waiting at the collection point (state: ${state.wireValue})."
            } else {
                "GARBY is not reporting a completed outbound route."
            }
        }
        ReturnCheck.RobotOnline -> "GARBY's Pi heartbeat is stale."
        ReturnCheck.BleLink -> "The Pi has lost its BLE link to the robot."
        ReturnCheck.Lidar -> "The Pi does not report healthy LiDAR scans."
        ReturnCheck.Cloud -> "Firebase is not connected."
        ReturnCheck.Internet -> "This phone has no validated internet connection."
        ReturnCheck.NoReturnInFlight -> "Another return request is still in progress."
    } + " Nothing was sent."

    /**
     * Publishes this phone's FCM registration identity. FCM now registers app instances by
     * Firebase Installation ID (FID); the legacy registration-token API is deprecated, so the
     * old `fcmToken` keys are no longer written. Nothing in this repository sends FCM messages
     * yet; a future sender targets this ID.
     */
    fun saveMessagingInstallationId(
        installationId: String,
        deviceId: String = RtdbConstants.DEFAULT_DEVICE_ID
    ) {
        if (installationId.isBlank()) return
        requireValidDeviceId(deviceId)
        try {
            database.reference.updateChildren(
                mapOf(
                    "devices/$deviceId/fcmInstallationId" to installationId,
                    "devices/$deviceId/fcmInstallationIdUpdatedAt" to ServerValue.TIMESTAMP
                )
            ).addOnFailureListener { error ->
                Log.w(TAG, "Failed to publish the FCM installation ID", error)
            }
        } catch (e: Exception) {
            Log.w(TAG, "Failed to publish the FCM installation ID", e)
        }
    }

    /**
     * Warm-up hook called on a background thread before dashboard routing.
     *
     * Forces the lazy [FirebaseDatabase] initialization and waits for the actual
     * socket connection to be established, so the dashboard can start loading at once.
     */
    suspend fun prewarm() {
        try {
            withTimeout(RtdbConstants.FIREBASE_CONNECT_TIMEOUT_MS.milliseconds) {
                connectionState.first { it is ConnectionState.Connected }
            }
            // Allow a small window for the initial RTDB sync to populate the local cache.
            delay(300.milliseconds)
        } catch (error: TimeoutCancellationException) {
            Log.w(TAG, "Firebase warm-up timed out", error)
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
