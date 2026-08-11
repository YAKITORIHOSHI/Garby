package com.thesis.garby.notifications

import android.util.Log
import com.google.firebase.messaging.FirebaseMessagingService
import com.google.firebase.messaging.RemoteMessage
import com.thesis.garby.realtime.GarbyRealtimeDb

class GarbyFirebaseMessagingService : FirebaseMessagingService() {

    override fun onNewToken(token: String) {
        super.onNewToken(token)
        Log.d(TAG, "New FCM registration token generated")
        GarbyRealtimeDb.saveFcmToken(token)
    }

    override fun onMessageReceived(remoteMessage: RemoteMessage) {
        super.onMessageReceived(remoteMessage)
        Log.d(TAG, "FCM Message received from: ${remoteMessage.from}")

        // 1. If notification payload exists (sent via Firebase Console or server notification)
        remoteMessage.notification?.let { notification ->
            val title = notification.title ?: "GARBY Navigation Alert"
            val body = notification.body ?: "GARBY is now running to Point B!"
            GarbyNotificationHelper.showNotification(applicationContext, title, body)
            return
        }

        // 2. If data payload exists (sent via FCM REST API / Cloud Functions)
        val data = remoteMessage.data
        if (data.isNotEmpty()) {
            val event = data["event"] ?: data["action"] ?: ""
            val title = data["title"] ?: "GARBY Navigation Alert"
            val body = data["body"] ?: "GARBY is now running to Point B!"

            if (event.contains("isRunningToPointB", ignoreCase = true) ||
                data["isRunningToPointB"] == "true"
            ) {
                GarbyNotificationHelper.showRunningToPointBNotification(applicationContext)
            } else if (event.contains("isReadyToReturn", ignoreCase = true) ||
                data["isReadyToReturn"] == "true"
            ) {
                GarbyNotificationHelper.showReadyToReturnNotification(applicationContext)
            } else {
                GarbyNotificationHelper.showNotification(applicationContext, title, body)
            }
        }
    }

    companion object {
        private const val TAG = "GarbyFcmService"
    }
}
