---
name: garby-change-handoff
description: Summarize completed GARBY changes, identify the actual edited files, and state the BLE/executor upload and Pi deployment requirements after project work or when asked for a handoff.
---

# GARBY change handoff

Give a concise handoff grounded in the current session's actual edits and checks.
Read the project `AGENTS.md`; use the existing maintenance skill for runtime or
firmware work. This skill formats and checks the handoff, without operating hardware.

## Summary and edited files

- List the resulting behavior changes in a few short bullets.
- List the files actually edited or created in this task, using clickable local
  file links and a short purpose. Group long test/documentation/artifact lists,
  with an exact change manifest linked when needed. Do not claim an inspected or
  historically changed file was edited now.
- Use the actual diff or recorded before/after hashes. This project can lack Git;
  never treat all existing files or old dated artifacts as new changes.
- Mention preserved GUI optimizations or workarounds only when relevant and checked.

## Identify the upload target

Trace which production source is compiled into each controller. Headers count,
even when their `.ino` file is unchanged. Use this mapping as a starting point:

| Actual change | MCU upload requirement |
| --- | --- |
| Production `.ino/.cpp/.h/.hpp/.c` compiled into `BLE_Receiver-Final/` only | **BLE bridge only** — upload the complete `BLE_Receiver-Final` sketch |
| Production source compiled into `MAIN_EXECUTOR_CODE/` only | **Executor only** — upload the complete `MAIN_EXECUTOR_CODE` sketch |
| Production source compiled into both, or coordinated firmware/protocol changes requiring both revisions | **Both BLE bridge and Executor** — name both matching sketches |
| Pi runtime/GUI only | **No MCU upload for this change** — deploy the entire `RasPi/` directory |
| Tests, documentation, skills, agent instructions or saved verification artifacts only | **No MCU upload** |
| Dependency/build-configuration changes or unclear include/protocol effects | Inspect affected builds; report **unverified** rather than guessing |

Check the complete consumer/include chain and protocol compatibility before finalizing
that mapping. Do not infer a target from a test filename, old binary, board package,
COM port, or the fact that a verification command compiled both targets.

State the upload requirement for **this change** separately from earlier firmware
that remains unuploaded. For example: a Pi-only follow-up needs no additional flash,
but an earlier forward-direction fix may still need the Executor image. If installed
firmware identity is unknown, say so; do not equate unchanged local source with an
up-to-date device. A matching image already confirmed installed need not be reflashed.

Give the verified build ID and source/binary artifact path when available. Describe
the sketch as the upload unit, rather than telling the user to upload a header alone.
Read current flasher state for a requested board/port; saved state does not establish
a current connection. Refer to the deployment guide for hardware acceptance.

## Checks and completion state

Report relevant checks actually run, their outcomes and skips. Distinguish source
applied, software tested, target compiled, firmware uploaded and physically accepted.
Do not turn a compiled binary or simulated wheel direction into hardware acceptance.

Keep the final response easy to scan, normally with these compact items:

- **Changes:** concrete results.
- **Files:** actual edits with links and purposes.
- **MCU upload:** BLE bridge / Executor / Both / None / Unverified, plus the reason and any
  earlier pending image. Include **Pi deployment** when applicable.
- **Checks:** outcomes and material skips; **Upload status:** what actually happened.

Keep upload requirement separate from upload status. If deployment, upload or
physical acceptance was not observed or recorded, label its status **unknown**;
do not silently turn missing evidence into either completed or not performed.

Preserve existing user authorization. Producing this summary does not itself
authorize an upload, robot command, service restart or SMS test; do not ask again
when the user has already authorized the specific action.
