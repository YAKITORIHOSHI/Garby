package com.thesis.garby.realtime

import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.conflate
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.shareIn

/**
 * Multiple screens share one listener, which detaches as soon as the last observer leaves.
 * Reset replay on stop: resuming must await a new callback before showing live readiness.
 * Share failures as values internally so each observer's normal error handling still runs.
 */
internal fun <T> Flow<T>.shareWhileObserved(scope: CoroutineScope): Flow<T> {
    val shared = map<T, Result<T>> { Result.success(it) }
        .catch { error ->
            if (error is CancellationException) throw error
            emit(Result.failure(error))
        }
        // Override shareIn's default buffered queue too. A slow screen needs the newest
        // status, not dozens of previously decoded snapshots after it catches up.
        .conflate()
        .shareIn(
            scope = scope,
            started = SharingStarted.WhileSubscribed(stopTimeoutMillis = 0, replayExpirationMillis = 0),
            replay = 1
        )
    return shared.map { it.getOrThrow() }
}
