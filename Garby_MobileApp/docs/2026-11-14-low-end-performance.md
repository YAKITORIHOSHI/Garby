# Device-wide rendering and memory performance — 2026-11-14

The source changes address measured UI stalls and reduce background contention and bitmap
memory across device brands, including low-end and mid-range phones. A successful build alone
is not evidence of a particular frame-rate or memory improvement.

## Read-only observations on the connected phone

| Item | Observed |
| --- | --- |
| Device | Samsung SM-A125F, Android 12, MT6765V/CB |
| RAM reported by the kernel | 5,899,428 KiB (about 5.63 GiB, nominal 6 GB device) |
| App heap growth limit | 256 MiB |
| Installed app | 2.0.0, version code 2, `DEBUGGABLE` and `TEST_ONLY` |
| Saved frame history | 372 frames; 371 janky (99.73%) |
| Frame latency | Median 250 ms; 90th percentile 650 ms; 99th percentile 2450 ms |
| GPU latency | Median 3 ms; 90th percentile 7 ms |
| Slow UI-thread frames | 365 |
| Saved process memory sample | Total PSS 294,421 KiB; graphics 132,130 KiB; Java heap 22,072 KiB |

These counters describe the existing app session, not a controlled benchmark. The first
inspection saw 178/178 janky frames; the saved snapshot includes further user activity. Memory
also varied between samples. The filtered app log records repeated skipped frames. The process
thread sample contains eight DefaultDispatcher workers, mostly sleeping; it does not establish
that eight CPU tasks were running continuously.

The much lower GPU times and slow UI-thread counters point to CPU-side UI/drawing overhead.
Source inspection identified always-allocated Haze backdrop/content layers on Android 12,
multiple per-surface blur/noise effects, decorative animations, and synchronous raster decoding
through `painterResource`. The existing debug build adds instrumentation overhead. RAM alone
does not explain the symptom: this A125F has roughly 6 GB and eight CPU cores, and about 3 GiB
was available at inspection. These observations support the selected fixes; a full trace would
be needed to apportion time among individual functions.

## Implemented behavior

- `PerformanceProfile` uses Android's low-RAM flag, total RAM at or below 6 GiB, heap class at or
  below 192 MiB, or at most four available CPUs. It has no manufacturer/model inputs or special
  cases. Unknown zero-valued capacities are conservative. The profile is stable for the process.
- Every device retains marble artwork, glass borders, readings, labels and controls without live
  backdrop blur, pulsing dots, animated route dashes or shimmer movement. The Haze dependencies
  are removed. Reduced devices also omit large decorative shadows/glows and use immediate gauge
  and navigation updates. Capable devices use short finite transitions; loading spinners remain
  for active operations. These changes also cover mid-range devices regardless of brand.
- App-owned background CPU work uses one shared view of the existing coroutine scheduler, with
  parallelism one on reduced devices and two otherwise. It does not create private per-screen
  executors or resize Firebase, Android, or the global coroutine worker pools. Firebase Tasks
  remain asynchronous and release the worker while awaiting a network response.
- Callback queues conflate **before** snapshot decoding. Output queues and shared monitoring
  streams also retain the latest value; duplicate decoded states are suppressed. There is no
  added telemetry sampling interval and no new delay on control writes.
- The dashboard declares composition content types for grid cards and headers, so compatible
  gas/health cards and section headers can reuse their compositions when scrolling.
- Dashboard/return observers share connection and default-device status subscriptions. The last
  observer stops the listener and clears replay. Listener errors still reach each observer's
  existing error handling. Existing screen/return-workflow lifecycle timeouts remain unchanged.
- Raster decoding runs on the bounded background dispatcher. Reduced backdrops are sampled to
  a maximum 1024 px dimension and the logo to 512 px (1024 px on capable devices). The reduced
  profile uses a sample size of two for the current assets,
  approximately a quarter of the original pixel memory. A shared 4 MiB cache (16 MiB on capable
  devices) avoids repeated screen decodes; a mutex prevents simultaneous duplicate decodes.
  Memory-trim callbacks clear cached references without recycling images still displayed.
- Version is 2.0.1/code 3. `assembleBenchmark` builds the same app ID with release optimization,
  R8/resource shrinking, debug instrumentation disabled, and the local development signing key.
  The production release remains unsigned for the external signing workflow.

No robot firmware, Pi production files, command fields, database paths, return gating rules,
freshness deadlines, acceptance windows or completion semantics were changed.

## Verification and evidence

Raw evidence is retained in the coordinated workspace's `artifacts/android-low-end-2026-11-14/`.
The standalone repository includes a portable
[verification summary](verification/2026-11-14/summary.json), and APKs/checksums are release assets.

- `source-before.json` and `readme-before.json`: hashes recorded before the corresponding edits.
- `device-details.txt`, `device-before-gfxinfo.txt`, `device-before-meminfo.txt`,
  `device-before-threads.txt`, `device-before-stalls.txt`: read-only phone observations.
- `gradle-release-final.log`: final Gradle verification command and output.
- `static-quality.log`: source-only project invariant check.
- `test-results/` and lint report: saved final verification results.
- `change-manifest.json`: exact edited/created project files and their final hashes, plus the
  available pre-change hashes. The dependency-catalog edit is also listed; no pre-change hash
  was captured for that file.
- `binary-hashes.json` and `apk-verification.txt`: built APK identities and manifest/signature checks.

The unit suite covers capacity selection (including generic 6 GB/eight-core devices), bitmap sampling,
listener sharing and detach, replay expiry, error delivery, and a 1,000-update burst behind a
slow observer. Existing freshness, thresholds, schema and return-command tests remain included.
The burst test was first observed failing against the initial shared stream's default buffer,
then passed with conflation before sharing (`burst-regression-before.log`/`.xml`). An earlier
test-only failure compared exception object identity, which coroutine stack-trace recovery may
copy; the final assertion checks the exception type and message.

The final Gradle run passed all 43 tests (zero failures/skips), debug/release/benchmark assembly,
and lint with no issues. The independent static quality check passed. Final check results,
APK identities and publication details are recorded in the local `verification-summary.json`.

## On-phone verification

The user approved installation and UI measurements. The exact final 2.0.1/code 3 optimized APK
was installed over the existing app with `adb install -r`; the session/data were retained and the
cloud reconnected. Package flags confirm it is non-debuggable and not test-only. The process
logged `Reduced effects=true, background parallelism=1`, selected from capacity alone.

Both scrolling sequences used three 400 ms upward swipes and three downward swipes at the same
coordinates, with 250 ms pauses. The final pass checked GARBY was foreground before each gesture.

| Observed metric | Previous installed 2.0.0 debug | Final optimized 2.0.1 |
| --- | ---: | ---: |
| Frames in sampled scrolling session | 39 | 286 |
| Janky frames | 39 (100%) | 73 (25.52%) |
| Median frame time | 300 ms | 26 ms |
| 90th percentile | 800 ms | 36 ms |
| 99th percentile | 850 ms | 65 ms |
| Process PSS after scrolling | 205,451 KiB (200.6 MiB) | 59,857 KiB (58.5 MiB) |
| Graphics PSS | 108,697 KiB | 23,576 KiB |

The final cold-process launch reported `TotalTime: 1835` ms; no startup improvement is calculated
because the old launch attempt was screen-locked. The comparison includes removal of debug
instrumentation as well as the source changes, and covers one phone/session, not every device.
Some scrolling jank remains. An early post-install sample contained initialization stalls and
only five frames; it is retained as evidence and excluded from the warm scrolling comparison.

Opening the return screen and backing out, then Home/resume, were checked. The stale robot
status still blocked sending; no return command was sent. One earlier navigation sample showed
Android Settings instead of GARBY, so it was discarded and repeated in the app. The temporary
stay-awake setting used during checks was restored to its original value. No fatal app exception
was seen in the captured final-process log. Fresh live robot transitions and behavior on other
physical phones were not exercised; their policy selection is covered by unit tests.

No MCU upload, Pi deployment or SMS test was performed. Normal app authentication/installation
registration remained active; no manual database writes or robot commands were issued.

## Phone acceptance

Install the development-signed benchmark APK using the
matching key and without clearing app data. Do not uninstall a differently signed app just to
force installation. Verify normal startup, dashboard scrolling, opening/backing out of Return,
background/resume and offline/reconnect. Avoid sending a return command during UI checks.

Capture fresh frame statistics and memory after the same warm-up and interaction sequence.
Keep debug-to-debug and optimized-to-optimized comparisons separate when attributing changes;
the recorded old session and a fresh optimized APK are not a controlled code-only comparison.
For further devices, repeat this procedure and check the selected capacity profile in the log.
Physical robot acceptance and externally signed distribution remain separate workflows.

**MCU upload requirement: None. Pi deployment requirement: None.** These Android-only source and
build changes create no additional firmware requirement; earlier installed firmware identity
was not inspected.

## Technical references

- [Android Compose performance](https://developer.android.com/develop/ui/compose/performance):
  release-mode verification and debug overhead.
- [Kotlin limitedParallelism](https://kotlinlang.org/api/kotlinx.coroutines/kotlinx-coroutines-core/kotlinx.coroutines/-coroutine-dispatcher/limited-parallelism.html):
  dispatcher views reuse scheduler threads and limit simultaneous execution.
- [Kotlin shareIn buffering](https://kotlinlang.org/api/kotlinx.coroutines/kotlinx-coroutines-core/kotlinx.coroutines.flow/share-in.html):
  override the default shared buffer with conflation and preserve observer error handling.
- [Android bitmap optimization](https://developer.android.com/develop/ui/compose/graphics/images/optimization):
  decode images at appropriate dimensions and keep expensive decode work off the UI thread.
