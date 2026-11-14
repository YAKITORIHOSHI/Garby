---
name: garby-software-verification
description: Run scoped, repeatable GARBY software checks with saved evidence after code changes or when verification automation is requested, without flashing or operating the robot.
---

# GARBY software verification

Automate verification through the project's existing tools. Read root `AGENTS.md`
and the maintenance skill for the affected runtime/firmware scope. Preserve their
safety contracts and the user's existing authorization.

Choose checks from the actual change; do not run every expensive build for a
documentation/skill edit. For normal runtime/firmware integration, the project
commands are:

```bash
python3 -B -m unittest discover -s RasPi -p 'test_*.py'
python3 -B tools/run_firmware_tests.py
python3 -B tools/source_sanity.py .
```

`python3 -B tools/verify_project.py --host-only` provides the existing combined
robot host workflow, including syntax checks. Its default full mode builds ESP32
targets; `--mobile-repo <path>` opts in to a separate app checkout without embedding it: inspect target options and build paths
against the sketch guidance before using it. Prefer the scoped commands when
only one subsystem changed. For skill-only work, use the skill creator validator
and a proportionate behavior review; robot tests need not be repeated.

For affected firmware, compile each actual target with its verified configuration
and an external build/cache path. Read current or saved flasher state rather than
guessing targets from installed packages. Never add upload flags or a serial port
to verification. Unchanged source and dependency hashes can reference an existing
successful target build; record that distinction instead of claiming a new build.

Capture commands, exit codes and logs under a new dated artifact directory.
Record source hashes and any compiled binary hashes, without copying credentials,
operator stop/request-ID files or modifying historical evidence. Explicitly report
Tk display skips and missing toolchains. A substitute Canvas or host peripheral
does not prove physical screen geometry, motor travel or stopping distance.

Run independent read-only checks concurrently when appropriate, keeping edits,
dependent builds and waits sequential. After a failure, fix the evidenced cause
before rerunning the affected check. Once checks pass, do not repeat or broaden
them without a new change or unresolved concern.

Finish with the GARBY change-handoff skill's summary, edited files, MCU target,
Pi deployment and verification/upload status. Verification automation neither
schedules recurring tasks nor grants permission to operate hardware or services.
