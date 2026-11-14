package com.thesis.garby.notifications

import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import androidx.core.app.NotificationCompat
import androidx.core.app.NotificationManagerCompat
import androidx.core.content.ContextCompat
import com.thesis.garby.MainActivity
import com.thesis.garby.R

object GarbyNotificationHelper {
    const val CHANNEL_ID = "garby_status_channel"
    const val EXTRA_OPEN_DASHBOARD = "skip_start"
    const val EXTRA_NAVIGATE_TO = "navigate_to"
    const val DESTINATION_DASHBOARD = "main_dashboard"

    private const val GROUP_KEY_GARBY = "com.thesis.garby.GARBY_ALERTS"
    private const val SUMMARY_ID = 1000

    fun createNotificationChannel(context: Context) {
        val channel = NotificationChannel(
            CHANNEL_ID,
            "GARBY run and status alerts",
            NotificationManager.IMPORTANCE_HIGH
        ).apply {
            description = "Runs, arrival at the collection point and robot status changes"
            enableVibration(true)
        }
        val notificationManager =
            context.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        notificationManager.createNotificationChannel(channel)
    }

    fun showRunningToDestinationNotification(context: Context) {
        showNotification(context, "GARBY is on its way", "GARBY is driving to the collection point.")
    }

    fun showReadyToReturnNotification(context: Context) {
        showNotification(
            context,
            "GARBY is waiting",
            "GARBY is at the collection point. Empty the bin, then send it back to base."
        )
    }

    fun showNotification(context: Context, title: String, body: String) {
        createNotificationChannel(context)

        val intent = Intent(context, MainActivity::class.java).apply {
            flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP or
                Intent.FLAG_ACTIVITY_SINGLE_TOP
            putExtra(EXTRA_OPEN_DASHBOARD, true)
            putExtra(EXTRA_NAVIGATE_TO, DESTINATION_DASHBOARD)
        }
        val pendingIntent = PendingIntent.getActivity(
            context,
            0,
            intent,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE
        )
        val accent = ContextCompat.getColor(context, R.color.garby_crimson)

        // Small icons must be monochrome; the colour logo would render as a white disc.
        val builder = NotificationCompat.Builder(context, CHANNEL_ID)
            .setSmallIcon(R.drawable.ic_stat_garby)
            .setColor(accent)
            .setContentTitle(title)
            .setContentText(body)
            .setStyle(NotificationCompat.BigTextStyle().bigText(body))
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setContentIntent(pendingIntent)
            .setGroup(GROUP_KEY_GARBY)
            .setAutoCancel(true)

        val summaryBuilder = NotificationCompat.Builder(context, CHANNEL_ID)
            .setSmallIcon(R.drawable.ic_stat_garby)
            .setColor(accent)
            .setStyle(NotificationCompat.InboxStyle().setSummaryText("GARBY alerts"))
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
            // Permission missing/denied on API 33+.
        }
    }
}
