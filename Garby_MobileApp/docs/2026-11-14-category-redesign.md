# Most Latest — dashboard category redesign

2026-11-14. Android UI and display summaries only.

The dashboard's Bin, Air and System labels are now inset dividers with category icons,
quiet borders, an accent rule, shorter descriptions and a summary of the readings below.
Status capsules use tinted text and a symbol or static dot halo. Stale sensors use neutral
icon accents and a clock; system tiles distinguish healthy, caution, fault and unknown.

Metric cards place status below the reading, giving longer labels their own row. Their
264 dp minimum height aligns normal-size cards. The adaptive grid scales its minimum
column width with system font size, and explanatory update text can wrap. Dividers wrap
their status onto a separate line when needed and remain accessible headings.

The displayed app version and release title are **Most Latest**. Android retains numeric
build ordering internally. Existing dated release history is preserved.

## Display summary rules

- Bin: fresh, valid weight/distance counts; filling, dispatch-weight and out-of-range
  weight states take priority when current.
- Air: the worst current gas band takes priority, including when another sensor is missing.
  This describes a band, not the executor's consecutive-packet danger rule.
- System: faults first, then cautions, then healthy/total counts. Missing reports cannot
  produce an all-clear. The summary consumes the same levels as the displayed tiles.
- Positive sensor counts require phone/cloud connectivity, a fresh timestamp, a finite
  value and the sensor's validity checks. Loading, stale and unavailable have distinct
  symbols independent of their displayed wording.

These summaries do not participate in dispatch, return readiness or Firebase control.
No blur, continuous animation, new runtime dependency or artwork was added.

## Changed files

| File | Purpose |
| --- | --- |
| [MainDashboard.kt](../app/src/main/java/com/thesis/garby/MainDashboard.kt) | Category integration, metric layout, font-aware grid and status symbols |
| [GlassKit.kt](../app/src/main/java/com/thesis/garby/ui/glass/GlassKit.kt) | Reusable category divider and refined status capsule |
| [DashboardIndicators.kt](../app/src/main/java/com/thesis/garby/ui/DashboardIndicators.kt) | Display-only aggregation and explicit indicator kinds |
| [DashboardIndicatorsTest.kt](../app/src/test/java/com/thesis/garby/ui/DashboardIndicatorsTest.kt) | Ten tests for freshness, invalid/partial readings, bands and system priority |
| [app/build.gradle.kts](../app/build.gradle.kts) | Displayed version name and internal upgrade ordering |
| [PATCH_NOTES.md](../PATCH_NOTES.md) | Most Latest release notes |
| [SYSTEM_ARCHITECTURE.md](../SYSTEM_ARCHITECTURE.md) | Category summaries, font scaling and rendering behavior |
| This report and [verification artifacts](verification/2026-11-14-category-redesign/) | Commands, results, source/binary hashes and native preview images |

The exact task file list and hashes are in the verification summary. Pre-existing untracked
files are outside this change.

## Verification

| Check | Result |
| --- | --- |
| `:app:testDebugUnitTest` | 53 passed, no failures or skips |
| `:app:lintDebug` | No issues found |
| `tools/static_quality_check.py` | Passed; 30 Kotlin files, three fonts |
| `:app:assembleDebug` | Passed |
| `:app:assembleRelease` | Passed; R8-minified, unsigned |
| `:app:assembleBenchmark` | Passed; R8-minified and signed with the existing local development key |
| Headless Compose render | Eight preview configurations; normal/narrow layouts and text scales up to 200% inspected |
| APK metadata/signature | Most Latest display name; installable optimized APK signature verified |

Logs, fixtures, preview configuration and SHA-256 hashes are saved under
[verification/2026-11-14-category-redesign](verification/2026-11-14-category-redesign/).
Rendering uses the Android layout engine in an isolated copy of the final source. The
experimental screenshot setup is confined to that copy. Its fixtures and setup are saved
with the evidence; the production project gains no screenshot engine dependency.

The category gallery uses the bundled background directly. In the full-dashboard headless
preview, asynchronous artwork loading can leave the logo/backdrop absent; the actual
production loader is unchanged. Synthetic preview telemetry is used throughout. This is
layout verification, not physical-phone or live-robot acceptance.

## Deployment

- **MCU upload requirement: None.** No bridge/executor production sources or protocol
  consumers changed.
- **Pi deployment requirement: None.** No Pi runtime or Firebase paths changed.
- The installable APK uses the existing local development certificate. The unsigned release
  APK is available for external production signing. No signing key is published.
- No app installation, robot command or MCU upload was performed for this change. The
  installed firmware identity and physical acceptance remain unknown.
