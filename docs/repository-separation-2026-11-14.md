# Repository separation correction — 2026-11-14

Garby contains robot source and robot evidence. Garby-Mobile-App contains Android source,
Android evidence and APKs. The current repositories and corrected Test2 ZIPs respect this
boundary. The embedded mobile tree/APK and mobile evidence were removed from robot source;
robot patches, logs and hashes were moved out of the app repository into robot evidence.
Both repositories link to the other for the public Firebase contract and compatibility.

The robot verifier now works without an embedded app. Optional cross-repository checks
require `--mobile-repo <external-checkout>` or GARBY_MOBILE_APP_ROOT for the specific test.
App static checks reject robot folders/evidence. Production app and robot runtime hashes
and APK bytes remain unchanged; completed native/Android builds were not repeated.

Test1 archives and their hashes remain unchanged. Named Test2 packages and the current
release source aliases are updated to the separated source revisions.
MCU upload requirement for this packaging correction: None. Prior turn-recovery firmware
still needs both complete sketches if not already installed; actual installed state is unknown.
