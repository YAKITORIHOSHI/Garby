package com.thesis.garby.ui

import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.size
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.dp
import com.thesis.garby.R

/**
 * Shown while the persisted Firebase Auth session is being loaded from disk.
 * Visible only for ~50–200 ms on a warm device; longer on a cold start.
 */
@Composable
fun SplashScreen() {
    Surface(
        modifier = Modifier.fillMaxSize(),
        color = MaterialTheme.colorScheme.background
    ) {
        Box(
            modifier = Modifier.fillMaxSize(),
            contentAlignment = Alignment.Center
        ) {
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.Center
            ) {
                Image(
                    painter = painterResource(id = R.drawable.app_icon),
                    contentDescription = null,
                    modifier = Modifier.size(160.dp)
                )
                Spacer(modifier = Modifier.height(24.dp))
                CircularProgressIndicator(
                    strokeWidth = 3.dp,
                    modifier = Modifier.size(32.dp)
                )
            }
        }
    }
}
