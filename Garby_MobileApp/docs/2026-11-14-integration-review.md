# GarbyMostLatest-Test2 integration review — 2026-11-14

This review covers the mobile working tree after `aa957b0`, plus the coordinated
`RasPi`, `BLE_Receiver-Final` and `MAIN_EXECUTOR_CODE` directories. Three sub-agents
reviewed those robot components at xhigh reasoning. Their inherited sandbox permissions
were stale after the owner enabled full access, so their shell work was stopped and all
remaining checks were executed by the primary session.

The pending feature is confirmed manual disposal: the app submits a uniquely identified
intent; the Pi claims it durably and sends one START; the executor's matching response
establishes acceptance. It does not establish arrival. Motion still needs fresh path,
heading and obstacle evidence, with emergency stop taking precedence. Review found the
Pi's claim/query-only recovery and executor request deduplication consistent with that plan.

## Repairs

1. A local Android timeout notice could hide a later matching acceptance or rejection.
   Submission results and listener reports now reconcile by request ID in either callback
   order. Unrelated or unknown reports retain uncertainty; existing outcomes and newer
   commands survive a delayed write receipt. Five new unit tests cover these races.
2. A BLE session reset cleared the bridge's `turnActive` flag, while the executor emits its
   initial TURN:START only once. Two clear front/back packets could consequently grant GO
   during an interrupted turn despite occupied, stale or missing spin-sweep evidence.
   A new host regression failed on the original production code. The bridge now preserves
   the turn scope across BLE reconnect, blocks GO until mode is known after bridge restart,
   and accepts idempotent turn-state replay. The executor reports its whole turn scope,
   including holds, before readiness in status/handshake replies. Controller regressions
   also failed before this repair. No new command grammar or route calibration is introduced.
3. One Tk integration test invoked the new confirmation dialog without mocking its answer.
   On a working Windows display this blocked unattended testing. Its confirmation is now
   explicitly answered by the fixture. Production Pi runtime behavior is unchanged.

## Verification and remaining limits

Logs, before/after hashes, changed-file manifest and the robot patch are under
[`verification/2026-11-14-mobile-integration/`](verification/2026-11-14-mobile-integration/).
The final verification summary there records exact commands and target-build outcomes.

- Android: **64 tests passed**, no failures/errors/skips; **lint reports no issues**;
  optimized development-signed benchmark APK and unsigned release APK assembled successfully.
- Firmware host suite: **all eight programs passed**, including production bridge/controller
  integration and reconnect/restart regressions. Existing numeric-conversion compiler warnings
  remain. Host peripherals do not establish electrical timing, physical movement or stopping.
- Both actual ESP32 targets compiled successfully from matching local source snapshots with
  one compiler worker. The bridge uses 590,168 bytes flash / 38,068 bytes static RAM; the
  executor uses 416,274 bytes flash / 29,448 bytes static RAM. CLI 1.5.1, ESP32 core 3.3.11
  and the verified saved board configurations were used. Binary hashes are recorded locally.
- The full Pi suite completed **464 tests with 3 failures and 6 errors on Windows**. The
  records include Linux adapter expectations, unavailable bash/POSIX pseudo-terminals,
  `/dev` path assumptions, and one orientation fixture receiving no frame. The latter is
  unresolved in this run. This is not an all-green Raspberry Pi target validation.
- Live Firebase rules, actual multi-phone operation, phone performance for this feature,
  deployed firmware identity, physical floor behavior and stopping distance remain unverified.
  No live control write, rule update, SMS, robot command, firmware upload, Pi restart or phone
  installation was performed in this task.

## Host crash report

After the build/test phase, the owner reported a PC crash and supplied a Studio freeze dump.
Windows System event 1001 records bugcheck `0x0000000A` and a minidump path; the Studio dump
at 11:15 records the UI waiting for a write-intent lock while a file read is in native
Windows CreateFile. The supplied Studio dump is after the recorded Windows reboot.
These records do **not** identify the cause of the kernel crash. The kernel dump is denied
to this process and no debugger was available. No driver, system service, Studio setting or
OS configuration was changed. Remaining native builds use a local source copy, one compiler
worker and below-normal process priority. Completed checks are not rerun.

Microsoft describes `0xA` as invalid memory access at raised IRQL; identifying the responsible
driver requires kernel-dump analysis. See the
[official bugcheck reference](https://learn.microsoft.com/en-us/windows-hardware/drivers/debugger/bug-check-0xa--irql-not-less-or-equal).

## Source and deployment boundaries

The mobile working directory is a Git repository; its sibling working directories and
parent have no Git metadata. Mobile source and evidence are committed normally to
[`Garby-Mobile-App`](https://github.com/YAKITORIHOSHI/Garby-Mobile-App). The owner also
requested synchronization of [`Garby`](https://github.com/YAKITORIHOSHI/Garby), so the
coordinated robot sources and current mobile subtree are published through a separate local
checkout of that repository. The exact eleven review repairs remain archived as
`robot-integration.patch` with source hashes for comparison with the starting workspace.

**MCU upload requirement for the turn repair: Both** complete `BLE_Receiver-Final` and
`MAIN_EXECUTOR_CODE` sketches, build ID `RECONNECT-20261114`. Actual uploads: none in this
task; installed revisions remain unknown. This follow-up changes only a Pi test, so it
does not add a Pi runtime deployment. The earlier manual-disposal feature still requires
deployment of the matching **entire RasPi directory** and reviewed Firebase disposal rules.

Test1 preserves mobile commit `aa957b03ab6c12befc6c25260fe4b38f694e224d` and the original
installable APK hash `25cd2d13511b1c6eaf74fd05441219e733525733ce60f21fc3a027e0498d63a2`.
Test2 contains the new mobile source and its matching optimized APK. Both ZIPs are explicit
release assets; GitHub's automatically generated source ZIP is separate. Neither test bundle
is a claim of physical acceptance. Android package identity is shared, so the APKs replace
each other rather than installing side by side.
