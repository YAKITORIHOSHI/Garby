package com.thesis.garby.notifications

import android.annotation.SuppressLint
import android.util.Log
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.messaging.FirebaseMessagingService
import com.google.firebase.messaging.RemoteMessage
import com.thesis.garby.realtime.GarbyRealtimeDb

/**
 * Receives FCM messages. This repository has no deployed sender; the event names below are the
 * ones the receiver has always accepted.
 *
 * The lint rule below still asks for onNewToken(), which Firebase deprecated with the move from
 * registration tokens to installation IDs; onRegistered() is its replacement.
 */
@SuppressLint("MissingFirebaseInstanceTokenRefresh")
class GarbyFirebaseMessagingService : FirebaseMessagingService() {

    /**
     * FCM registers this app instance by its Firebase Installation ID (enabled in the manifest
     * with `firebase_messaging_installation_id_enabled`). Called at startup syncs and whenever
     * the ID changes.
     */
    override fun onRegistered(installationId: String) {
        super.onRegistered(installationId)
        Log.d(TAG, "FCM registration updated")
        // Database writes need an authenticated session; AuthViewModel publishes it after sign-in.
        if (FirebaseAuth.getInstance().currentUser != null) {
            GarbyRealtimeDb.saveMessagingInstallationId(installationId)
        }
    }

    override fun onMessageReceived(remoteMessage: RemoteMessage) {
        super.onMessageReceived(remoteMessage)
        Log.d(TAG, "FCM message received")

        remoteMessage.notification?.let { notification ->
            GarbyNotificationHelper.showNotification(
                applicationContext,
                notification.title ?: DEFAULT_TITLE,
                notification.body ?: DEFAULT_BODY
            )
            return
        }

        val data = remoteMessage.data
        if (data.isEmpty()) return
        val event = data["event"] ?: data["action"] ?: ""
        when {
            event.contains("isRunningToPointB", ignoreCase = true) || data["isRunningToPointB"] == "true" ->
                GarbyNotificationHelper.showRunningToDestinationNotification(applicationContext)
            event.contains("isReadyToReturn", ignoreCase = true) || data["isReadyToReturn"] == "true" ->
                GarbyNotificationHelper.showReadyToReturnNotification(applicationContext)
            else -> GarbyNotificationHelper.showNotification(
                applicationContext,
                data["title"] ?: DEFAULT_TITLE,
                data["body"] ?: DEFAULT_BODY
            )
        }
    }

    companion object {
        private const val TAG = "GarbyFcmService"
        private const val DEFAULT_TITLE = "GARBY"
        private const val DEFAULT_BODY = "GARBY has a status update. Open the app for details."
    }
}
