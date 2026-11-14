package com.thesis.garby

import android.app.Application
import android.util.Log
import com.google.firebase.FirebaseApp
import com.thesis.garby.network.NetworkConnectivityManager
import com.thesis.garby.notifications.GarbyNotificationHelper
import com.thesis.garby.performance.DevicePerformance
import com.thesis.garby.ui.ResourceBitmaps

/** Process-wide initialization for validated networking and Firebase. */
class GarbyApplication : Application() {

    override fun onCreate() {
        super.onCreate()
        DevicePerformance.initialize(this)
        Log.i(TAG, "Reduced effects=${DevicePerformance.profile.reducedEffects}, " +
            "background parallelism=${DevicePerformance.profile.backgroundParallelism}")
        NetworkConnectivityManager.initialize(this)

        try {
            FirebaseApp.initializeApp(this)
            GarbyNotificationHelper.createNotificationChannel(this)
            // The FCM token is saved by AuthViewModel once a session exists: the database
            // rejects unauthenticated writes, so saving it here at launch only failed.
        } catch (e: Exception) {
            Log.e(TAG, "Firebase initialization failed", e)
        }
    }

    /** Android only invokes this for emulated processes, not normal production exit. */
    override fun onTerminate() {
        NetworkConnectivityManager.cleanup()
        super.onTerminate()
    }

    @Suppress("DEPRECATION") // Running-low callbacks still apply to supported Android 8–13 phones.
    override fun onTrimMemory(level: Int) {
        super.onTrimMemory(level)
        // Release cached, off-screen images; displayed bitmaps must never be recycled here.
        if (level >= TRIM_MEMORY_RUNNING_LOW) ResourceBitmaps.clear()
    }

    private companion object {
        const val TAG = "GarbyApplication"
    }
}
