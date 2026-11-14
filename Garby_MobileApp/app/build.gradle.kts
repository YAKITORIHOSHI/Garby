plugins {
    alias(libs.plugins.androidApplication)
    alias(libs.plugins.kotlinCompose)
    alias(libs.plugins.googleServices)
}

android {
    namespace = "com.thesis.garby"
    // Android 17 QPR2 SDK (37.2). Compose 1.12 and core 1.19 need at least API 37.
    compileSdk {
        version = release(37) {
            minorApiLevel = 2
        }
    }

    defaultConfig {
        applicationId = "com.thesis.garby"
        minSdk = 26
        // Android 17 target-only changes (local network permission, certificate transparency,
        // background audio, widgets, RFCOMM reads) do not touch this Firebase-only app.
        targetSdk = 37
        versionCode = 6
        versionName = "Most Latest · Manual Disposal"
    }

    buildTypes {
        release {
            isMinifyEnabled = true
            isShrinkResources = true
            // No app-specific keep rules are needed; each library ships its own consumer rules.
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"))
        }
        create("benchmark") {
            // Installable, release-equivalent local performance build. Production release
            // signing stays external; the development key is only for device verification.
            initWith(getByName("release"))
            signingConfig = signingConfigs.getByName("debug")
            isDebuggable = false
            matchingFallbacks += "release"
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_21
        targetCompatibility = JavaVersion.VERSION_21
    }

    buildFeatures {
        compose = true
    }

    packaging {
        resources {
            excludes += setOf(
                "META-INF/DEPENDENCIES",
                "META-INF/LICENSE",
                "META-INF/LICENSE.txt",
                "META-INF/NOTICE",
                "META-INF/NOTICE.txt"
            )
        }
    }
}

dependencies {
    implementation(libs.androidx.activity.compose)
    implementation(platform(libs.androidx.compose.bom))
    implementation(libs.androidx.ui)
    implementation(libs.androidx.ui.graphics)
    implementation(libs.androidx.ui.tooling.preview)
    implementation(libs.androidx.material3)
    implementation(libs.androidx.compose.foundation)
    implementation(libs.androidx.navigation.compose)
    implementation(libs.androidx.compose.material.icons.extended)
    implementation(libs.androidx.lifecycle.runtime.compose)
    implementation(libs.androidx.lifecycle.viewmodel.compose)

    implementation(platform(libs.firebase.bom))
    implementation(libs.firebase.database)
    implementation(libs.firebase.auth)
    implementation(libs.firebase.messaging)
    implementation(libs.firebase.installations)
    implementation(libs.kotlinx.coroutines.play.services)

    debugImplementation(libs.androidx.ui.tooling)

    constraints {
        // Play services brings an old Fragment; the Activity Result API needs 1.3.0 or later.
        implementation(libs.androidx.fragment)
    }

    testImplementation(libs.junit)
    testImplementation(libs.kotlinx.coroutines.test)
}
