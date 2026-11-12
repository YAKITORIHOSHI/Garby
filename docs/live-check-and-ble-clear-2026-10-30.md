# Live monitor check and BLE log clearing — 2026-10-30

The updated Pi runtime was launched on the user's request and its live GUI was
inspected. BLE notifications confirmed `STRAIGHT-20261030` and `POWERON`; LiDAR
streaming and cloud connectivity were visible. The bin/gas sensor UART remains
unavailable at `/dev/ttyAMA0`. No substitute port was guessed.

A read-only ROS subscriber captured ten advancing scans at approximately
5.75 Hz. Their acquisition ages were 173–176 ms, inside the existing 0.8 s
source-age budget. The live front reading was about 37 cm. Independent nearest
group estimates were CF 37.1–37.3 cm, CL 41.1–41.3 cm and CR 34.8–36.1 cm;
these would suppress forward nudging and veto a right correction. The independent
subscriber did not inspect actual transmitted packets or internal MPU state.

The user clarified that Garby was at home with an uncertain position relative
to the calibrated school route. Normal Start performs calibration moves and a
left turn before reaching its nudge-enabled segment. No Start or release was
issued by the agent. The agent invoked the visible global emergency stop at
13:06:43 local; controller motion messages confirmed STOP:OPERATOR and EMERGENCY
STOP, and the persisted Pi stop file was present. Operator sonar-disable and
release events before that were user actions. The GUI also showed directional
LiDAR checks in read-only mode. This setup cannot establish obstacle-aware
motion or straightness; no physical route trial was run.

Evidence is in `artifacts/live-nudging-2026-10-30/`, including GUI screenshots,
the raw-scan observation JSON/plot, runtime logs and launch records. Passive
`btmon` access was denied by the environment; permissions were not overridden.
Available `/dev/video*` entries were processing nodes, not proof of a camera
view of the robot. A screen view and LiDAR plot are not a physical trajectory
measurement.

## Clear logs

The BLE/messages tab now has a keyboard-accessible **Clear logs** button beside
the message heading. It removes displayed history and restores the Text widget's
read-only state. Pending/new messages continue to arrive, connection cards are
unaffected, and saved runtime logs remain available for diagnosis.

Displayed history retains at most 250 logical text lines. New lines evict the
oldest, including when a single received message contains multiple lines. Manual
scrollback and the bounded 40-message poll budget remain. Focus-only outlines
are removed from custom buttons, tabs and the log field; decorative/status
borders retain their colors. Keyboard activation remains available.

The focused Tk suite passed 32 tests without skips on `localhost:10.0`, including
clear-then-receive behavior and small-display visibility. Exact commands/hashes
are in `artifacts/ble-log-clear-2026-10-30/`. The owned Pi runtime was gracefully
reloaded to show the button while preserving the emergency-stop file. No MCU
firmware change/upload is part of this GUI revision.

The button was also clicked in the live monitor: prior messages disappeared,
subsequent status messages appeared, and the saved log size did not decrease.
Screenshots and the check record are in the BLE-clear artifact directory.
Four final focused Tk checks then passed without skips for focus styling,
keyboard activation, exact 250/251-line retention, multiline input, clearing and
small-display visibility. See `artifacts/monitor-focus-2026-10-30/`. The monitor
was reloaded again with the stop latch preserved to apply that final revision.

## Next safety concern

The user requires obstacle/person stopping to be reviewed after these recent
items. Current LiDAR supplies generic obstacle returns and does not classify
people. Software checks cannot establish measured braking distance or timely
physical stopping; those require a secured, supervised acceptance procedure.
The current inspection leaves the robot stopped.
