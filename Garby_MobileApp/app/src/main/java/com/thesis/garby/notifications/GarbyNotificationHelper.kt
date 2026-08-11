package com.thesis.garby.notifications

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import androidx.core.app.NotificationCompat
import androidx.core.app.NotificationManagerCompat
import com.thesis.garby.MainActivity
import com.thesis.garby.R

object GarbyNotificationHelper {
    const val CHANNEL_ID = "garby_status_channel"
    private const val GROUP_KEY_GARBY = "com.thesis.garby.GARBY_ALERTS"
    private const val SUMMARY_ID = 1000
    private const val SINGLE_NOTIFICATION_ID = 1001

    fun createNotificationChannel(context: Context) {
        val name = "GARBY Navigation & Status Alerts"
        val descriptionText = "Notifications for GARBY navigation and robot status updates"
        val importance = NotificationManager.IMPORTANCE_HIGH
        val channel = NotificationChannel(CHANNEL_ID, name, importance).apply {
            description = descriptionText
            enableVibration(true)
        }
        val notificationManager =
            context.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        notificationManager.createNotificationChannel(channel)
    }

    fun showRunningToPointBNotification(context: Context) {
        showNotification(context, "GARBY Navigation Alert", "GARBY is now running to Point B!")
    }

    fun showReadyToReturnNotification(context: Context) {
        showNotification(context, "GARBY Status Alert", "GARBY is now ready to return!")
    }

    fun showNotification(context: Context, title: String, body: String) {
        createNotificationChannel(context)

        val intent = Intent(context, MainActivity::class.java).apply {
            flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
            putExtra("skip_start", true)
            putExtra("navigate_to", "main_dashboard")
        }
        val pendingIntent = PendingIntent.getActivity(
            context,
            0,
            intent,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )

        // Individual alert bundled into GROUP_KEY_GARBY
        val builder = NotificationCompat.Builder(context, CHANNEL_ID)
            .setSmallIcon(R.drawable.app_icon)
            .setContentTitle(title)
            .setContentText(body)
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setContentIntent(pendingIntent)
            .setGroup(GROUP_KEY_GARBY)
            .setAutoCancel(true)

        // Summary notification to bundle all alerts cleanly in status shade
        val summaryBuilder = NotificationCompat.Builder(context, CHANNEL_ID)
            .setSmallIcon(R.drawable.app_icon)
            .setStyle(NotificationCompat.InboxStyle().setSummaryText("GARBY Alerts"))
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setGroup(GROUP_KEY_GARBY)
            .setGroupSummary(true)
            .setContentIntent(pendingIntent)
            .setAutoCancel(true)

        try {
            with(NotificationManagerCompat.from(context)) {
                val notificationId = (System.currentTimeMillis() % 10000).toInt() + 1002
                notify(notificationId, builder.build())
                notify(SUMMARY_ID, summaryBuilder.build())
            }
        } catch (_: SecurityException) {
            // Permission missing/denied on API 33+
        }
    }
}
