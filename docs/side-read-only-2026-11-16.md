# Sides read-only — 2026-11-16

## Request

> "add read only for sides too, so that the motor can be tested and run."

Until now the side checks could never be switched off. Front and back read-only existed, but
something right beside GARBY always held every motion. That is what kept Manual disposal still
on the bench this morning (`docs/gui-controls-2026-11-16.md`). The owner asked for the sides
to get the same read-only switch.

## What it does

- **Controls → LiDAR obstacle checks** now has three toggles: **Front**, **Back**, **Sides**.
  Each shows "checks on" or "read only". Headless key `l` toggles Sides, next to `f` and `b`.
- **Sides read only** stops these from holding the motors:
  - the side sector limits (22 cm),
  - the side bands, i.e. a return within about 2 cm beside the body, beside the front half
    and, while backing into a parking spot, beside the rear half.

  This applies to forward driving (`F`), backing up (`B`) and parking travel (`R`).
- **Unchanged:**
  - The readings and zones stay on the map and in the log; side readings say "Reading only".
  - The steering never turns *toward* a close side: those vetoes only steer, never stop.
  - Missing or stale scans still stop GARBY.
  - The emergency stop and the gyro checks still apply.
- **Turns.** A spin sweeps a circle around GARBY, so something right beside it is still inside
  that circle. Spins stay with the Front and Back groups, as before: to let a route spin with
  something beside it, set Front (and Back, for the rear half) read only too.
- **Defaults.** Front and Back read-only still never bypass a side. All three groups are
  "checks on" every time the monitor starts.
- **Phone.** It cannot see or change Sides. Its status document is checked by the database
  rules in the app repository, so no field was added.

## Use

On a cluttered bench, set **Sides** (plus Front and Back if needed) to read only, run the test,
then set them back. The hold above the tabs and the footer ("Threshold bypass: sides") show
what is switched off. On the floor, leave them on.

## Checks

- `tests/pi/test_side_read_only.py` (new), all with the real Pi runtime:
  - a return 1 cm beside the body, or inside the 22 cm side-sector limit, holds
    `F=O|B=O|T=O|R=O`,
  - with Sides read only, beside the body it is `F=C|B=C|T=O|R=C` (the spin would sweep
    through it), and with Front as well `F=C|B=C|T=C|R=C`,
  - Front+Back read only still hold it,
  - Sides read only does not clear the front box,
  - a missing scan stays `F=S|B=S`,
  - the rear-half band while backing clears only with Sides,
  - the zone stays listed.
- `tests/pi/test_monitor_ui.py`: the toggle, the "Reading only" side readings, the footer, and
  all Controls buttons visible at 800×480 (`artifacts/side-read-only-2026-11-16/controls-sides.png`).
- Full Pi suite: see `artifacts/side-read-only-2026-11-16/VERIFICATION.md`.

Correction (same day): an earlier draft of this test said the 22 cm side-sector limit could
never trigger. That was wrong, an artifact of a failing sub-case that left Sides read-only
switched on. The sector limit does hold (the bench log at 09:01 shows
`Side clearance blocked (RIGHT)`), and Sides read-only bypasses it as well.

**MCU upload requirement for this change: None** (Pi only, `SIDES-20261116`). Still pending,
unchanged: both boards at `REMOTE-20261116`.
