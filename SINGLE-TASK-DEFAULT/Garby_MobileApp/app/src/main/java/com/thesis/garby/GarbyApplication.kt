package com.thesis.garby

import android.app.Application
import android.util.Log
import com.google.firebase.FirebaseApp
import com.thesis.garby.network.NetworkConnectivityManager

/** Process-wide initialization for validated networking and Firebase. */
class GarbyApplication : Application() {

    override fun onCreate() {
        super.onCreate()
        NetworkConnectivityManager.initialize(this)

        try {
            FirebaseApp.initializeApp(this)
            com.thesis.garby.notifications.GarbyNotificationHelper.createNotificationChannel(this)

            com.google.firebase.messaging.FirebaseMessaging.getInstance().token.addOnCompleteListener { task ->
                if (task.isSuccessful) {
                    val token = task.result
                    Log.d(TAG, "FCM registration token obtained on launch")
                    com.thesis.garby.realtime.GarbyRealtimeDb.saveFcmToken(token)
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Firebase initialization failed", e)
        }
    }

    /** Android only invokes this for emulated processes, not normal production exit. */
    override fun onTerminate() {
        NetworkConnectivityManager.cleanup()
        super.onTerminate()
    }

    private companion object {
        const val TAG = "GarbyApplication"
    }
}
