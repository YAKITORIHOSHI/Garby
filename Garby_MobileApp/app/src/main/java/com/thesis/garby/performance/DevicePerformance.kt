package com.thesis.garby.performance

import android.app.ActivityManager
import android.content.Context
import androidx.compose.runtime.staticCompositionLocalOf
import kotlinx.coroutines.CoroutineDispatcher
import kotlinx.coroutines.Dispatchers

object DevicePerformance {
    var profile: PerformanceProfile = PerformanceProfile.Conservative
        private set

    /** Called before any application coroutine work is started. */
    fun initialize(context: Context) {
        val manager = context.getSystemService(ActivityManager::class.java) ?: return
        val memory = ActivityManager.MemoryInfo()
        manager.getMemoryInfo(memory)
        profile = PerformanceProfile.forDevice(
            lowRam = manager.isLowRamDevice,
            totalMemoryBytes = memory.totalMem,
            memoryClassMb = manager.memoryClass,
            processorCount = Runtime.getRuntime().availableProcessors()
        )
    }
}

val LocalPerformanceProfile = staticCompositionLocalOf { PerformanceProfile.Conservative }

object GarbyDispatchers {
    // One shared view, not a new pool per sensor or screen. Suspended Firebase Tasks do not
    // occupy a worker. This limits app CPU work; Firebase still owns its network threads.
    val background: CoroutineDispatcher by lazy {
        Dispatchers.Default.limitedParallelism(
            DevicePerformance.profile.backgroundParallelism,
            "GarbyBackground"
        )
    }
}
