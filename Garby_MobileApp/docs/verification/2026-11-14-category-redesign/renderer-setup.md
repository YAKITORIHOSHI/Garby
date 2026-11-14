# Native Compose layout evidence

Eight previews were rendered and visually inspected using the Android layout engine:

| Image | Configuration |
| --- | --- |
| `categories-400.png` | 400 dp, default text; eight category/status examples |
| `categories-320.png` | 320 dp, default text |
| `categories-150-percent.png` | 360 dp, 150% text |
| `categories-200-percent.png` | 320 dp, 200% text; status moves below the title |
| `dashboard-400.png` | Actual dashboard, 400 dp, default text |
| `dashboard-320.png` | Actual dashboard, 320 dp, default text |
| `dashboard-400-large-text.png` | Actual dashboard, 400 dp, 150% text; one metric column |
| `dashboard-360-large-text.png` | Actual dashboard, 360 dp, 150% text |

All images are native renderer output at 160 dpi, without image resizing or retouching.
The fixture uses synthetic telemetry and makes no Firebase calls. The category gallery
loads the bundled background directly; the dashboard's asynchronous artwork loader does
not complete in this headless renderer. Dashboard logo/backdrop absence is a preview
limitation; production artwork loading was not changed. Lower dashboard sections outside
the preview viewport remain scrollable in the app.

## Reproduce

Copy the project sources and Gradle configuration into an isolated local directory,
excluding `.git`, caches and build outputs. Use the same SDK/JDK as the production build,
and set a separate absolute `garby.buildRoot` in that copy's `local.properties`. Copy
`CategoryPreviewTest.kt` to:

```text
app/src/screenshotTest/kotlin/com/thesis/garby/CategoryPreviewTest.kt
```

Add these properties to the copy's `gradle.properties`:

```properties
android.experimental.enableScreenshotTest=true
android.experimental.testSuiteSupport=true
```

Append this to the copy's `app/build.gradle.kts`:

```kotlin
android {
    testOptions {
        unitTests.isIncludeAndroidResources = true
        screenshotTests.create("screenshotTest") {
            engineVersion = "0.0.1-alpha16"
            targetVariants.add("debug")
            dependencies {
                implementation(libs.androidx.ui.tooling)
                implementation("com.android.tools.screenshot:screenshot-validation-api:0.0.1-alpha16")
            }
        }
    }
}

androidComponents {
    beforeVariants {
        if (it.buildType != "debug") it.enable = false
    }
}
```

Run the task exposed by the installed AGP 9.5.1:

```powershell
./gradlew.bat :app:updateDebugScreenshotTestDefaultTestSuite --console=plain
```

In this shell, the batch wrapper's redirected output failed before Gradle started, so the
equivalent wrapper entry point was used for both production and preview checks:

```powershell
& "$env:JAVA_HOME/bin/java.exe" -classpath gradle/wrapper/gradle-wrapper.jar `
  org.gradle.wrapper.GradleWrapperMain :app:updateDebugScreenshotTestDefaultTestSuite --console=plain
```

Reference images appear under `app/src/screenshotTestDefaultDebug/reference/`. The recorded
run generated images and inspected them; it did not compare them against an existing golden
baseline. See `preview-render.log` for the actual successful task output. These experimental
dependencies and flags are not part of the production project.

Setup follows the [official AGP screenshot suite documentation](https://developer.android.com/studio/preview/compose-screenshot-testing-with-testsuites).
