plugins {
    alias(libs.plugins.androidApplication) apply false
    alias(libs.plugins.kotlinCompose) apply false
    id("com.google.gms.google-services") version "4.4.4" apply false
}
