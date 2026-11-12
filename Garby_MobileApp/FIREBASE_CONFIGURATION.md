# Android Firebase configuration — 2026-10-31

The Android app's client configuration is synced from
`../RasPi/Firebase/google-services.json` to
`app/google-services.json`. The two files were byte-identical during this
review. SHA-256:

```text
b7cb3fb4dead8e625288cb9dd068f54a8ca5e1cb52eeb8d8a647ee1589b4d582
```

The configured Android package is `com.thesis.garby`, matching the app's
`applicationId`. The configuration and
`app/src/main/java/com/thesis/garby/realtime/Constants.kt` point to project
`garby-thesis` and its Asia Southeast 1 Realtime Database. The Google Services
Gradle plugin reads the app-level JSON during the Android build.

## Credential boundaries

The mobile app uses Firebase anonymous authentication through
`firestore/AuthViewModel.kt`. It must not contain a Firebase Admin SDK key or a
human account password. The Admin SDK JSON under `RasPi/Firebase` contains a
private key and belongs only in a trusted server-side environment; it was not
copied to the app. `RasPi/Firebase/FireBase.txt` includes a plaintext account
password, which was also not copied or embedded. If that password is current,
rotate it and keep future copies in a private secret store.

`google-services.json` is Firebase client configuration, not an Admin
credential. App access still depends on Firebase Authentication settings,
Realtime Database rules, and any applicable API-key restrictions. Do not use a
client key or anonymous authentication as a substitute for server-side access
control.

## Verification

- Source and destination client JSON parsed successfully and have the same
  SHA-256.
- The registered Android package matches `applicationId`.
- The Realtime Database URL matches the configured project URL.
- Source inspection found no supplied account password or Admin private key in
  the Android app source.
- The app remains on its existing anonymous-authentication flow; no account
  password was added to the APK.
- `python3 -B tools/static_quality_check.py`: passed (22 Kotlin files).
- Gradle Google-services processing, Android unit tests, and APK assembly could
  not run here because this environment has no Java/JDK (`JAVA_HOME` is unset
  and no `java` executable is installed). The config/package/URL checks above
  were run directly against the source files.
