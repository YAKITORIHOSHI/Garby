package com.thesis.garby.ui

import android.content.res.Resources
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.LruCache
import androidx.annotation.DrawableRes
import androidx.compose.runtime.Composable
import androidx.compose.runtime.State
import androidx.compose.runtime.produceState
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.platform.LocalResources
import com.thesis.garby.performance.DevicePerformance
import com.thesis.garby.performance.GarbyDispatchers
import com.thesis.garby.performance.bitmapSampleSize
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext

/** Small, byte-bounded cache shared by screens; bundled artwork is decoded off the UI thread. */
internal object ResourceBitmaps {
    private data class Key(val resourceId: Int, val maxDimensionPx: Int)

    private val decodeMutex = Mutex()
    private val cache by lazy {
        object : LruCache<Key, Bitmap>(DevicePerformance.profile.bitmapCacheBytes) {
            override fun sizeOf(key: Key, value: Bitmap): Int = value.allocationByteCount
        }
    }

    suspend fun load(resources: Resources, @DrawableRes id: Int, maxDimensionPx: Int): ImageBitmap =
        withContext(GarbyDispatchers.background) {
            decodeMutex.withLock {
                val key = Key(id, maxDimensionPx)
                val bitmap = cache.get(key) ?: run {
                    val bounds = BitmapFactory.Options().apply { inJustDecodeBounds = true }
                    BitmapFactory.decodeResource(resources, id, bounds)
                    val options = BitmapFactory.Options().apply {
                        inScaled = false // This loader is for the bundled drawable-nodpi artwork.
                        inSampleSize = bitmapSampleSize(bounds.outWidth, bounds.outHeight, maxDimensionPx)
                    }
                    checkNotNull(BitmapFactory.decodeResource(resources, id, options)).also {
                        it.prepareToDraw()
                        cache.put(key, it)
                    }
                }
                bitmap.asImageBitmap()
            }
        }

    fun clear() = cache.evictAll()
}

@Composable
internal fun rememberResourceBitmap(@DrawableRes id: Int, maxDimensionPx: Int): State<ImageBitmap?> {
    val resources = LocalResources.current
    return produceState<ImageBitmap?>(null, resources, id, maxDimensionPx) {
        value = ResourceBitmaps.load(resources, id, maxDimensionPx)
    }
}
