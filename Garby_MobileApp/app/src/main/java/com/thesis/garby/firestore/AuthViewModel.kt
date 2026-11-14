package com.thesis.garby.firestore

import android.util.Log
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.auth.FirebaseAuthException
import com.google.firebase.auth.FirebaseAuthInvalidCredentialsException
import com.google.firebase.auth.FirebaseAuthInvalidUserException
import com.google.firebase.installations.FirebaseInstallations
import com.thesis.garby.network.NetworkConnectivityManager
import com.thesis.garby.performance.GarbyDispatchers
import com.thesis.garby.realtime.GarbyRealtimeDb
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.tasks.await
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeout

/**
 * Single source of truth for Firebase authentication.
 *
 * The prototype uses Firebase anonymous auth. No password or service
 * credential is embedded in the APK. FirebaseAuth itself owns token refresh;
 * duplicating that lifecycle in application code creates conflicting states.
 */
class AuthViewModel : ViewModel() {

    private companion object {
        const val TAG = "AuthViewModel"
        const val SIGN_IN_TIMEOUT_MS = 15_000L
    }

    private val firebaseAuth = FirebaseAuth.getInstance()

    private val _authState = MutableStateFlow<AuthState>(
        if (firebaseAuth.currentUser != null) AuthState.Authenticated else AuthState.Unauthenticated
    )
    val authState: StateFlow<AuthState> = _authState.asStateFlow()

    private val _isSigningIn = MutableStateFlow(false)
    val isSigningIn: StateFlow<Boolean> = _isSigningIn.asStateFlow()

    private val _signInError = MutableStateFlow<String?>(null)
    val signInError: StateFlow<String?> = _signInError.asStateFlow()

    private var signInJob: Job? = null

    private val authStateListener = FirebaseAuth.AuthStateListener { auth ->
        val authenticated = auth.currentUser != null
        if (authenticated) {
            _signInError.value = null
            publishMessagingInstallationId()
            // While signIn() is in flight, the coroutine drives the state
            // change so background RTDB warm-up completes before routing.
            if (_isSigningIn.value) return@AuthStateListener
        }
        _authState.value = if (authenticated) {
            AuthState.Authenticated
        } else {
            AuthState.Unauthenticated
        }
        _isSigningIn.value = false
    }

    init {
        firebaseAuth.addAuthStateListener(authStateListener)
    }

    /** FCM registers this app instance by its Firebase Installation ID; see the messaging service. */
    private fun publishMessagingInstallationId() {
        FirebaseInstallations.getInstance().id.addOnCompleteListener { task ->
            val installationId = if (task.isSuccessful) task.result else null
            if (!installationId.isNullOrBlank()) {
                GarbyRealtimeDb.saveMessagingInstallationId(installationId)
            }
        }
    }

    fun signIn() {
        if (_isSigningIn.value) return
        if (!NetworkConnectivityManager.isNetworkConnected() && firebaseAuth.currentUser == null) {
            _signInError.value = "This phone is offline. Connect to the internet and try again."
            return
        }
        _signInError.value = null
        _isSigningIn.value = true

        signInJob?.cancel()
        signInJob = viewModelScope.launch(GarbyDispatchers.background) {
            var authenticated = firebaseAuth.currentUser != null
            try {
                if (!authenticated) {
                    withTimeout(SIGN_IN_TIMEOUT_MS) {
                        firebaseAuth.signInAnonymously().await()
                    }
                    authenticated = firebaseAuth.currentUser != null
                }
                // Warm up Firebase RTDB (lazy init + socket) before the
                // dashboard is routed to, so the transition stays smooth.
                GarbyRealtimeDb.prewarm()
                authenticated = firebaseAuth.currentUser != null
            } catch (_: TimeoutCancellationException) {
                _signInError.value = "Firebase connection timed out. Check internet and try again."
                authenticated = false
            } catch (e: CancellationException) {
                throw e
            } catch (_: FirebaseAuthInvalidUserException) {
                _signInError.value = "Firebase user is disabled or no longer exists."
                authenticated = false
            } catch (_: FirebaseAuthInvalidCredentialsException) {
                _signInError.value = "Firebase credentials were rejected."
                authenticated = false
            } catch (e: FirebaseAuthException) {
                _signInError.value = e.message ?: "Firebase authentication failed."
                authenticated = false
            } catch (e: Exception) {
                Log.w(TAG, "Sign-in warm-up error: ${e.message}")
                _signInError.value = "Sign-in failed. Check internet and Firebase access."
                authenticated = false
            } finally {
                withContext(Dispatchers.Main) {
                    _authState.value = if (authenticated) {
                        AuthState.Authenticated
                    } else {
                        AuthState.Unauthenticated
                    }
                    _isSigningIn.value = false
                }
            }
        }
    }

    fun signOut() {
        signInJob?.cancel()
        _signInError.value = null
        _isSigningIn.value = false
        firebaseAuth.signOut()
    }

    override fun onCleared() {
        firebaseAuth.removeAuthStateListener(authStateListener)
        signInJob?.cancel()
    }
}

sealed interface AuthState {
    data object Authenticated : AuthState
    data object Unauthenticated : AuthState
}
