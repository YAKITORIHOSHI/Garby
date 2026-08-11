package com.thesis.garby

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.animation.core.tween
import androidx.compose.animation.slideInHorizontally
import androidx.compose.animation.slideOutHorizontally
import androidx.compose.foundation.Image
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsCompat
import androidx.core.view.WindowInsetsControllerCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import com.thesis.garby.firestore.AuthState
import com.thesis.garby.firestore.AuthViewModel
import com.thesis.garby.ui.theme.GarbyTheme
import com.thesis.garby.ui.theme.Montserrat

class MainActivity : ComponentActivity() {

    private val skipToDashboardState = mutableStateOf(false)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        hideSystemBars()
        requestNotificationPermission()
        handleNotificationIntent(intent)

        setContent {
            GarbyTheme {
                Surface(
                    modifier = Modifier.fillMaxSize(),
                    color = MaterialTheme.colorScheme.background
                ) {
                    AppScreen(skipToDashboard = skipToDashboardState.value)
                }
            }
        }
    }

    override fun onNewIntent(intent: android.content.Intent) {
        super.onNewIntent(intent)
        handleNotificationIntent(intent)
    }

    private fun handleNotificationIntent(intent: android.content.Intent?) {
        if (intent == null) return
        val skip = intent.getBooleanExtra("skip_start", false) ||
            intent.getStringExtra("navigate_to") == "main_dashboard" ||
            intent.hasExtra("google.message_id")
        if (skip) {
            skipToDashboardState.value = true
        }
    }

    private fun requestNotificationPermission() {
        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.TIRAMISU) {
            if (checkSelfPermission(android.Manifest.permission.POST_NOTIFICATIONS) != android.content.pm.PackageManager.PERMISSION_GRANTED) {
                requestPermissions(arrayOf(android.Manifest.permission.POST_NOTIFICATIONS), 101)
            }
        }
    }

    private fun hideSystemBars() {
        WindowCompat.setDecorFitsSystemWindows(window, false)
        WindowInsetsControllerCompat(window, window.decorView).apply {
            hide(WindowInsetsCompat.Type.navigationBars())
            systemBarsBehavior = WindowInsetsControllerCompat.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
        }
    }
}

@Composable
fun AppScreen(skipToDashboard: Boolean = false) {
    val authViewModel: AuthViewModel = viewModel()
    val authState by authViewModel.authState.collectAsState()
    val authError by authViewModel.signInError.collectAsState()
    val isSigningIn by authViewModel.isSigningIn.collectAsState()

    androidx.compose.runtime.LaunchedEffect(skipToDashboard) {
        if (skipToDashboard && authState is AuthState.Unauthenticated) {
            authViewModel.signIn()
        }
    }

    val shouldShowDashboard = authState is AuthState.Authenticated

    if (shouldShowDashboard) {
        AppNavHost(
            startDestination = "main_dashboard",
            onSignOut = authViewModel::signOut
        )
    } else {
        AppNavHost(
            startDestination = "start",
            onSignIn = authViewModel::signIn,
            isAuthenticating = isSigningIn,
            authError = authError
        )
    }
}

@Composable
private fun AppNavHost(
    startDestination: String,
    onSignIn: () -> Unit = {},
    onSignOut: () -> Unit = {},
    isAuthenticating: Boolean = false,
    authError: String? = null
) {
    val navController = rememberNavController()

    NavHost(
        navController = navController,
        startDestination = startDestination,
        modifier = Modifier.fillMaxSize()
    ) {
        val enter = slideInHorizontally(initialOffsetX = { it }, animationSpec = tween(300))
        val exit = slideOutHorizontally(targetOffsetX = { -it }, animationSpec = tween(300))
        val popEnter = slideInHorizontally(initialOffsetX = { -it }, animationSpec = tween(300))
        val popExit = slideOutHorizontally(targetOffsetX = { it }, animationSpec = tween(300))

        composable(
            route = "start",
            enterTransition = { enter },
            exitTransition = { exit },
            popEnterTransition = { popEnter },
            popExitTransition = { popExit }
        ) {
            StartScreen(
                isAuthenticating = isAuthenticating,
                errorMessage = authError,
                onStartClicked = onSignIn
            )
        }

        composable(
            route = "main_dashboard",
            enterTransition = { enter },
            exitTransition = { exit },
            popEnterTransition = { popEnter },
            popExitTransition = { popExit }
        ) {
            MainDashboard(
                onResetTrashbin = { navController.navigate("reset_trashbin") },
                onSignOut = onSignOut
            )
        }

        composable(
            route = "reset_trashbin",
            enterTransition = { enter },
            exitTransition = { exit },
            popEnterTransition = { popEnter },
            popExitTransition = { popExit }
        ) {
            ResetTrashbinScreen(onBack = { navController.popBackStack() })
        }
    }
}

@Composable
fun StartScreen(
    isAuthenticating: Boolean,
    onStartClicked: () -> Unit,
    errorMessage: String? = null
) {
    Surface(modifier = Modifier.fillMaxSize()) {
        Box(modifier = Modifier.fillMaxSize()) {
            Image(
                painter = painterResource(id = R.drawable.darker_app_background),
                contentDescription = null,
                contentScale = ContentScale.Crop,
                modifier = Modifier.fillMaxSize()
            )

            // Top Middle: Logo and Label
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier
                    .align(Alignment.TopCenter)
                    .padding(top = 70.dp, start = 28.dp, end = 28.dp)
            ) {
                Image(
                    painter = painterResource(id = R.drawable.app_icon),
                    contentDescription = "GARBY logo",
                    modifier = Modifier.size(190.dp)
                )

                Spacer(modifier = Modifier.height(16.dp))

                Text(
                    text = "GARBY OPERATOR",
                    color = Color.White,
                    fontFamily = Montserrat,
                    fontWeight = FontWeight.Black,
                    fontSize = 26.sp,
                    textAlign = TextAlign.Center
                )
            }

            // Bottom Middle: Error message and START button
            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier
                    .align(Alignment.BottomCenter)
                    .padding(bottom = 50.dp, start = 28.dp, end = 28.dp)
                    .fillMaxWidth()
            ) {
                if (!errorMessage.isNullOrBlank()) {
                    Text(
                        text = errorMessage,
                        color = Color(0xFFFFCDD2),
                        fontFamily = Montserrat,
                        fontWeight = FontWeight.Bold,
                        fontSize = 13.sp,
                        textAlign = TextAlign.Center,
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(bottom = 16.dp)
                    )
                }

                Button(
                    onClick = onStartClicked,
                    enabled = !isAuthenticating,
                    shape = RoundedCornerShape(20.dp),
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color.White,
                        contentColor = Color.Black,
                        disabledContainerColor = Color(0xFFE0E0E0),
                        disabledContentColor = Color.DarkGray
                    ),
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(60.dp)
                ) {
                    if (isAuthenticating) {
                        CircularProgressIndicator(
                            color = Color.Black,
                            strokeWidth = 3.dp,
                            modifier = Modifier.size(24.dp)
                        )
                    } else {
                        Text(
                            text = "START",
                            fontFamily = Montserrat,
                            fontWeight = FontWeight.Bold,
                            fontSize = 20.sp
                        )
                    }
                }
            }
        }
    }
}
