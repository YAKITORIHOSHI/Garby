package com.thesis.garby.performance

/** Stable for the process lifetime, so fluctuating free RAM never changes the screen design. */
data class PerformanceProfile(val reducedEffects: Boolean) {
    val backgroundParallelism: Int get() = if (reducedEffects) 1 else 2
    val backdropMaxDimensionPx: Int get() = if (reducedEffects) 1024 else 2048
    val logoMaxDimensionPx: Int get() = if (reducedEffects) 512 else 1024
    val bitmapCacheBytes: Int get() = (if (reducedEffects) 4 else 16) * 1024 * 1024

    companion object {
        val Conservative = PerformanceProfile(reducedEffects = true)

        fun forDevice(
            lowRam: Boolean,
            totalMemoryBytes: Long,
            memoryClassMb: Int,
            processorCount: Int
        ): PerformanceProfile {
            // Capacity, not manufacturer/model, determines the memory/concurrency budget.
            // Six-GB devices can still have slow cores, so core count alone is insufficient.
            val reduced = lowRam || totalMemoryBytes <= 6L * 1024 * 1024 * 1024 ||
                memoryClassMb <= 192 || processorCount <= 4
            return PerformanceProfile(reducedEffects = reduced)
        }
    }
}

/** Decode to a power-of-two sample without first allocating the full-size bitmap. */
internal fun bitmapSampleSize(width: Int, height: Int, maxDimensionPx: Int): Int {
    require(maxDimensionPx > 0)
    var sample = 1
    val largest = maxOf(width, height)
    while (largest / sample > maxDimensionPx && sample <= Int.MAX_VALUE / 2) sample *= 2
    return sample
}
