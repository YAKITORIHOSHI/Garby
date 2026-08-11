package com.thesis.garby.network

import android.content.Context
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkCapabilities
import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * Process-wide monitor for validated internet access.
 *
 * A network being present is not enough for control operations: it must expose
 * NET_CAPABILITY_INTERNET and be validated by Android. StateFlow is thread-safe,
 * so callbacks update it directly without a leaked process coroutine scope.
 */
object NetworkConnectivityManager {

    private const val TAG = "NetworkConnectivity"
    private val lock = Any()

    @Volatile
    private var connectivityManager: ConnectivityManager? = null

    @Volatile
    private var networkCallback: ConnectivityManager.NetworkCallback? = null

    @Volatile
    private var isInitialized = false

    private val _isInternetAvailable = MutableStateFlow(false)
    val isInternetAvailable: StateFlow<Boolean> = _isInternetAvailable.asStateFlow()

    fun initialize(context: Context) = synchronized(lock) {
        if (isInitialized) return@synchronized

        val manager = context.applicationContext
            .getSystemService(Context.CONNECTIVITY_SERVICE) as? ConnectivityManager

        if (manager == null) {
            Log.e(TAG, "ConnectivityManager unavailable")
            _isInternetAvailable.value = false
            return@synchronized
        }

        connectivityManager = manager
        _isInternetAvailable.value = hasValidatedInternet(manager)

        val callback = object : ConnectivityManager.NetworkCallback() {
            override fun onAvailable(network: Network) = refreshState()
            override fun onLost(network: Network) = refreshState()

            override fun onCapabilitiesChanged(
                network: Network,
                networkCapabilities: NetworkCapabilities
            ) = refreshState()

            override fun onUnavailable() = refreshState()
        }

        try {
            manager.registerDefaultNetworkCallback(callback)
            networkCallback = callback
            isInitialized = true
        } catch (e: SecurityException) {
            Log.e(TAG, "Missing permission to monitor connectivity", e)
            networkCallback = null
            connectivityManager = null
            _isInternetAvailable.value = false
        } catch (e: RuntimeException) {
            Log.e(TAG, "Unable to register network callback", e)
            networkCallback = null
            connectivityManager = null
            _isInternetAvailable.value = false
        }
    }

    fun isNetworkConnected(): Boolean {
        val manager = connectivityManager ?: return false
        return hasValidatedInternet(manager)
    }

    private fun refreshState() {
        val manager = connectivityManager
        _isInternetAvailable.value = manager != null && hasValidatedInternet(manager)
    }

    private fun hasValidatedInternet(manager: ConnectivityManager): Boolean {
        return try {
            val network = manager.activeNetwork ?: return false
            val capabilities = manager.getNetworkCapabilities(network) ?: return false
            capabilities.hasCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET) &&
                capabilities.hasCapability(NetworkCapabilities.NET_CAPABILITY_VALIDATED)
        } catch (e: SecurityException) {
            Log.w(TAG, "Connectivity check denied", e)
            false
        }
    }

    fun cleanup() = synchronized(lock) {
        val manager = connectivityManager
        val callback = networkCallback
        if (manager != null && callback != null) {
            try {
                manager.unregisterNetworkCallback(callback)
            } catch (_: IllegalArgumentException) {
                // Callback was already unregistered.
            } catch (e: RuntimeException) {
                Log.w(TAG, "Unable to unregister network callback", e)
            }
        }
        networkCallback = null
        connectivityManager = null
        isInitialized = false
        _isInternetAvailable.value = false
    }
}
