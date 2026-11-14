package com.thesis.garby.realtime

import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.channels.ProducerScope
import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.flow.callbackFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.flow
import kotlinx.coroutines.launch
import kotlinx.coroutines.test.UnconfinedTestDispatcher
import kotlinx.coroutines.test.runCurrent
import kotlinx.coroutines.test.runTest
import org.junit.Assert.assertEquals
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class MonitoringFlowTest {
    @Test
    fun slowObserverSkipsObsoleteTelemetryDuringABurst() = runTest {
        val startBurst = CompletableDeferred<Unit>()
        val resumeObserver = CompletableDeferred<Unit>()
        val shared = flow {
            emit(0)
            startBurst.await()
            for (value in 1..1000) emit(value)
        }.shareWhileObserved(backgroundScope)
        val values = mutableListOf<Int>()
        backgroundScope.launch(UnconfinedTestDispatcher(testScheduler)) {
            shared.collect {
                values.add(it)
                if (it == 0) {
                    startBurst.complete(Unit)
                    resumeObserver.await()
                }
            }
        }
        runCurrent()
        assertEquals(listOf(0), values)
        resumeObserver.complete(Unit)
        runCurrent()
        assertEquals(listOf(0, 1000), values)
    }

    @Test
    fun dashboardAndReturnShareListenerAndResumeWithFreshCallback() = runTest {
        var attachments = 0
        var active = 0
        val shared = callbackFlow {
            active++
            trySend(++attachments)
            awaitClose { active-- }
        }.shareWhileObserved(backgroundScope)

        val dashboardValues = mutableListOf<Int>()
        val returnValues = mutableListOf<Int>()
        val first = backgroundScope.launch(UnconfinedTestDispatcher(testScheduler)) {
            shared.collect { dashboardValues.add(it) }
        }
        val second = backgroundScope.launch(UnconfinedTestDispatcher(testScheduler)) {
            shared.collect { returnValues.add(it) }
        }
        runCurrent()
        assertEquals(1, attachments)
        assertEquals(1, active)
        assertEquals(listOf(1), dashboardValues)
        assertEquals(listOf(1), returnValues)

        first.cancel()
        runCurrent()
        assertEquals(1, active)
        second.cancel()
        runCurrent()
        assertEquals(0, active)

        val resumedValues = mutableListOf<Int>()
        backgroundScope.launch(UnconfinedTestDispatcher(testScheduler)) {
            shared.collect { resumedValues.add(it) }
        }
        runCurrent()
        assertEquals(2, attachments)
        assertEquals(listOf(2), resumedValues) // no cached "live" value from session 1
    }

    @Test
    fun sharedListenerFailureReachesBothObserversInsteadOfLeavingLiveCachedState() = runTest {
        var producer: ProducerScope<Int>? = null
        val shared = callbackFlow {
            producer = this
            awaitClose { }
        }.shareWhileObserved(backgroundScope)
        val errors = mutableListOf<Throwable>()
        repeat(2) {
            backgroundScope.launch(UnconfinedTestDispatcher(testScheduler)) {
                shared.catch { errors.add(it) }.collect { }
            }
        }
        runCurrent()
        val failure = IllegalStateException("permission denied")
        producer!!.close(failure)
        runCurrent()
        assertEquals(2, errors.size)
        // Coroutine stack-trace recovery can copy exceptions at suspension boundaries.
        errors.forEach {
            assertEquals(failure.javaClass, it.javaClass)
            assertEquals(failure.message, it.message)
        }
    }
}
