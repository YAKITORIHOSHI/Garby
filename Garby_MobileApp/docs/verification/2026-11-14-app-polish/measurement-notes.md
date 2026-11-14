# Runtime verification notes

- Captured text dumps trim trailing padding and final empty lines for Git; recorded values,
  ordering and timestamps are unchanged.
- User authorized optimized app installation and scrolling/fullscreen/Home-resume checks.
- Original observed installation: Most Latest, internal build code 4, DEBUGGABLE and TEST_ONLY.
- Controlled debug baseline: `debug-scroll-*`; source/binary build code 4.
- First optimized installation was replaced during the test by a 23,381,489-byte debug APK:
  `781edb9c7b278d072df1f3ff0f381adf382f6db176098c847d5e8ff832eeabea`.
  That deployment's source/process was not independently traced. Its flags match an IDE test
  deployment. Files beginning `invalidated-` describe that interrupted attempt and are not
  optimized-performance evidence.
- The verified second install: internal code 5, Most Latest, no DEBUGGABLE/TEST_ONLY flags.
  `install-verified.log`, `installed-package.txt`, `optimized-scroll-summary.json` and
  `final-installed-sha256.txt` identify the actual tested binary.
- The scroll script normalizes to the top, resets counters, then records only the fixed
  six-swipe sequence. Each gesture first checks GARBY is foreground. The optimized run also
  checks package flags and exact APK hash before/after. No taps or robot commands are in
  this measurement script.
- Phone GUI checks used the sign-out dialog's Cancel button, Home/resume, an edge swipe,
  and navigation to/from the return screen. The send button remained disabled. No actual
  sign-out or return submission was performed.
- Window dumps and screenshots confirm hidden status/navigation bars for activity and modal
  windows, including after returning from Home. Transient bars remain accessible by swipe.
- Default and optimized idle thread samples show no continuously busy app worker at those
  instants. This is not a server-load benchmark or proof that burst activity never occurs.
- Debug versus optimized compilation contributes to the observed improvement. Fullscreen
  changes viewport height, and the background-dispatch change was included too. Only one
  phone was measured; residual jank is visible in the raw counters.

Native launcher preview source is `LauncherIconPreviewTest.kt`. Use the isolated screenshot
setup described in the preceding category verification directory's `renderer-setup.md`,
copy the final resources and Fullscreen/MainActivity/ReturnViewModel/GlassKit sources into
that isolated copy, and run `:app:updateDebugScreenshotTestDefaultTestSuite`. The additional
before-icon resource is the foreground from the prior category commit, referenced by a
temporary copy of the adaptive icon resource. `icon-render.log` records successful native
generation. These screenshot dependencies are not part of the production project.

Icon geometry is calculated from the final Android vector's line-segment vertices.
`icon-geometry.json` records its maximum radius against the guaranteed adaptive safe area.
The vector's original artwork is unchanged and the generation script is retained under
`design/`.
