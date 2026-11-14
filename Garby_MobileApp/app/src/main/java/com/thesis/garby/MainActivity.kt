package com.thesis.garby

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.SystemBarStyle
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.animation.AnimatedContent
import androidx.compose.animation.EnterTransition
import androidx.compose.animation.ExitTransition
import androidx.compose.animation.core.tween
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.animation.scaleIn
import androidx.compose.animation.scaleOut
import androidx.compose.animation.togetherWith
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.ui.Modifier
import androidx.core.content.ContextCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import com.thesis.garby.firestore.AuthState
import com.thesis.garby.firestore.AuthViewModel
import com.thesis.garby.notifications.GarbyNotificationHelper
import com.thesis.garby.performance.LocalPerformanceProfile
import com.thesis.garby.ui.hideSystemBars
import com.thesis.garby.ui.theme.GarbyTheme

class MainActivity : ComponentActivity() {

    /** Bumped for every launch from a notification, so each tap can trigger sign-in once. */
    private val notificationOpens = mutableIntStateOf(0)

    private val notificationPermission =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) {
            // Notifications are optional; the dashboard works either way.
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // The glass UI is always dark, so the system bars use light icons whatever the theme.
        enableEdgeToEdge(
            statusBarStyle = SystemBarStyle.dark(Color.TRANSPARENT),
            navigationBarStyle = SystemBarStyle.dark(Color.TRANSPARENT)
        )
        window.hideSystemBars()
        if (savedInstanceState == null) {
            requestNotificationPermission()
            handleNotificationIntent(intent)
        }

        setContent {
            GarbyTheme {
                GarbyApp(notificationOpens = notificationOpens.intValue)
            }
        }
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        // Re-enter immersive mode after Home/resume, permission prompts and modal windows.
        if (hasFocus) window.hideSystemBars()
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        setIntent(intent)
        handleNotificationIntent(intent)
    }

    private fun handleNotificationIntent(intent: Intent?) {
        if (intent == null) return
        val fromNotification = intent.getBooleanExtra(GarbyNotificationHelper.EXTRA_OPEN_DASHBOARD, false) ||
            intent.getStringExtra(GarbyNotificationHelper.EXTRA_NAVIGATE_TO) == GarbyNotificationHelper.DESTINATION_DASHBOARD ||
            intent.hasExtra("google.message_id")
        if (fromNotification) notificationOpens.intValue += 1
    }

    private fun requestNotificationPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU &&
            ContextCompat.checkSelfPermission(this, Manifest.permission.POST_NOTIFICATIONS) !=
            PackageManager.PERMISSION_GRANTED
        ) {
            notificationPermission.launch(Manifest.permission.POST_NOTIFICATIONS)
        }
    }
}

private object Routes {
    const val DASHBOARD = "dashboard"
    const val RETURN = "return"
}

@Composable
fun GarbyApp(notificationOpens: Int) {
    val reducedEffects = LocalPerformanceProfile.current.reducedEffects
    val authViewModel: AuthViewModel = viewModel()
    // Activity-scoped: a return request stays tracked while the operator moves between screens.
    val returnViewModel: ReturnViewModel = viewModel()
    val authState by authViewModel.authState.collectAsStateWithLifecycle()
    val authError by authViewModel.signInError.collectAsStateWithLifecycle()
    val signingIn by authViewModel.isSigningIn.collectAsStateWithLifecycle()

    LaunchedEffect(notificationOpens) {
        if (notificationOpens > 0 && authViewModel.authState.value is AuthState.Unauthenticated) {
            authViewModel.signIn()
        }
    }

    AnimatedContent(
        targetState = authState is AuthState.Authenticated,
        transitionSpec = {
            if (reducedEffects) EnterTransition.None togetherWith ExitTransition.None
            else fadeIn(tween(420)) togetherWith fadeOut(tween(260))
        },
        modifier = Modifier.fillMaxSize(),
        label = "authenticated"
    ) { authenticated ->
        if (authenticated) {
            GarbyNavHost(
                returnViewModel = returnViewModel,
                onSignOut = {
                    returnViewModel.onSignedOut()
                    authViewModel.signOut()
                }
            )
        } else {
            StartScreen(
                isAuthenticating = signingIn,
                errorMessage = authError,
                onConnect = authViewModel::signIn
            )
        }
    }
}

@Composable
private fun GarbyNavHost(
    returnViewModel: ReturnViewModel,
    onSignOut: () -> Unit
) {
    val navController = rememberNavController()
    val reducedEffects = LocalPerformanceProfile.current.reducedEffects
    NavHost(
        navController = navController,
        startDestination = Routes.DASHBOARD,
        modifier = Modifier.fillMaxSize(),
        // Every screen draws the same backdrop, so a fade-through reads as the glass changing.
        enterTransition = {
            if (reducedEffects) EnterTransition.None
            else fadeIn(tween(280)) + scaleIn(tween(280), initialScale = 0.97f)
        },
        exitTransition = { if (reducedEffects) ExitTransition.None else fadeOut(tween(200)) },
        popEnterTransition = { if (reducedEffects) EnterTransition.None else fadeIn(tween(280)) },
        popExitTransition = {
            if (reducedEffects) ExitTransition.None
            else fadeOut(tween(200)) + scaleOut(tween(200), targetScale = 0.97f)
        }
    ) {
        composable(Routes.DASHBOARD) {
            MainDashboard(
                returnViewModel = returnViewModel,
                onOpenReturn = {
                    navController.navigate(Routes.RETURN) { launchSingleTop = true }
                },
                onSignOut = onSignOut
            )
        }
        composable(Routes.RETURN) { entry ->
            ReturnScreen(
                viewModel = returnViewModel,
                onBack = {
                    // Ignore a second tap while the exit animation runs.
                    if (entry.lifecycle.currentState == Lifecycle.State.RESUMED) {
                        navController.popBackStack()
                    }
                }
            )
        }
    }
}
