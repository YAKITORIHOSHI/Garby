# Phone control, the map on the phone, the headless launcher and heat — 2026-11-15

The owner's requests 5, 6 and 7 of 2026-11-14 (the full list is in
[`work-plan-2026-11-14.md`](work-plan-2026-11-14.md); requests 1–4 and 8 are in
[`objects-station-reverse-2026-11-14.md`](objects-station-reverse-2026-11-14.md)):

> (5) Add the remote controls on the mobile app ... implement the lidar map to the Mobile app? I
> was planning to have a headless run of this project and it's overall control was on the mobile.
> (6) Overheating; could you think of a approach on how can we utilize this by optimizing memory
> and processor?
> (7) Create a headless launcher; this should only rely on the mobile for its full monitoring and
> control.

Pi software revision `PHONE-20261114`. **Source implemented and software tested here. Not run
against the live database, the robot or a phone.** No controller firmware changes beyond
`REVERSE-20261114`. The app side is in https://github.com/YAKITORIHOSHI/Garby-Mobile-App.

Contents: [headless](#running-without-a-screen) · [what the phone can do](#what-the-phone-can-do) ·
[cloud contract](#cloud-contract) · [setting it up](#setting-it-up) · [heat](#heat) ·
[tests](#tests-run-here) · [first use, step by step](#first-use-step-by-step) · [limits](#limits)

## Running without a screen

```bash
bash RasPi/run_headless.sh start    # background, log in ~/garby-logs/bridge.log
bash RasPi/run_headless.sh status   # running? link, LiDAR, cloud, stop, hold reason, route step
bash RasPi/run_headless.sh logs     # follow the log
bash RasPi/run_headless.sh stop     # SIGTERM; the bridge shuts down within 5 s
bash RasPi/run_headless.sh doctor   # runtime check, BLE scan (nothing connected), cloud key
```

- `start` runs the runtime check first and starts nothing when it fails. It never starts a
  route: the bridge comes up with every check enabled and waits for a deliberate start.
- The log is one file per day, 14 days kept. Until now a floor run left no log unless the
  console was captured. `--log-dir` (or `GARBY_LOG_DIR`) gives the monitor the same log.
- At boot: the service file (`garby-bridge.service.example`, installed by
  `sudo python3 RasPi/install_startup.py`) now runs `run_headless.sh run`.
- The monitor window is unchanged and still the tool for bench work.

## What the phone can do

Everything on the monitor's *Controls* tab, through the same gate:

| On the phone | Pi action | Offered when |
| --- | --- | --- |
| **EMERGENCY STOP**, under every screen | `ESTOP` | always (even with no report from the robot) |
| Release stop | `RELEASE` | stop latched and confirmed, executor at IDLE |
| Continue run | `CONTINUE` | as Release, and the executor reports a route to continue |
| Reverse to start | `REVERSE` | as Release, and the executor reports a route to reverse |
| Obstacle sonar on/off | `SONAR_ON` / `SONAR_OFF` | executor at IDLE |
| Front / rear LiDAR read-only | `FRONT_READONLY_ON/OFF`, `BACK_READONLY_ON/OFF` | the Pi permits read-only |
| Restart Pi + bridge + executor | `REBOOT` | stop latched and confirmed |

Manual disposal and the return to base were already in the app and are unchanged.

The **Map** tab shows the LiDAR scan as the monitor does: GARBY in the middle, front up, with
the red, amber and orange markers, the zones and the hold reason. The Pi publishes frames only
while a phone has the map open.

## Cloud contract

All under `devices/<id>/` in the Realtime Database. Logic: `RasPi/cloud_control.py`; the loop
is `firebase_control_loop` in `final_w_serial.py`.

| Path | Writer | Content |
| --- | --- | --- |
| `commands/operator` | app, then Pi | `{version: 1, requestId (32 hex), requestedAt (server time), requestedBy (uid), action, status}`; the Pi moves `status` from `pending` to `claimed` and then `accepted`, `failed` or `unconfirmed`, with `reason` |
| `operator` | Pi | the mirror of the Controls tab: stop latched/confirmed, what is allowed (`canRelease`, `canContinue`, `canReverse`, `canReset`, `canConfigure`), sonar, read-only, robot state, hold reason, zones, route step, people and objects in view, `launcher {mode, startedAt, software}`, `updatedAt` |
| `commands/map` | app | `{leaseAt (server time), by}`, renewed every 20 s while the map is open |
| `map/frame` | Pi | `{v, seq, at, ageMs, d, m, z, p, hold, wl, wr, h}`: `d` = 360 distances in cm (16 bit, base64, index 0 ahead, counter-clockwise), `m` = markers `[forward cm, left cm, kind, vx, vy]` (kind 1 likely person, 2 moving, 3 object) |

Rules the Pi keeps:

- **At most once.** A request is claimed in a transaction before it is acted on. A request left
  `claimed` by a Pi process that has since restarted ends `unconfirmed`; it is never sent again.
- **Age.** An emergency stop is applied up to 120 s old (it only ever stops), and applied even
  when it is then reported expired. Anything else older than 20 s is refused, so a release or a
  continue cannot fire late after a reconnect. The age is checked again after the claim, with
  the time the claim's transaction took, read-only toggles included (fixed 2026-11-15: the
  second check reused the poll's own time and could never refuse).
- **The same gate.** The relay calls the function behind the monitor's buttons
  (`operator_control.request`). Release, Continue and Reverse still need the confirmed stop, and
  the route still needs fresh GO and every motion gate on the robot.
- **Outcome.** `accepted` means the executor acknowledged that request ID; `failed` carries the
  executor's or the Pi's reason; no answer within 8 s is `unconfirmed`.
- **Read-only** from the phone needs the Pi's `ALLOW_RUNTIME_SAFETY_DISABLE` and is off again
  after every restart of the Pi software.
- **Map.** A frame is published only under a lease younger than 45 s, at most twice a second
  (once a second when the Pi is at 75 °C or warned), and never from a scan older than 0.8 s.
  About 1.2 kB per frame: roughly 9 MB per hour per open map.
- The mirror is rewritten when it changes and every 10 s; the app calls it stale after 30 s.
- Commands are polled once a second. Nothing here is path evidence or grants GO.

## Setting it up

1. **Database rules.** Merge `firebase/manual-disposal.rules.example.json` from the app
   repository into the project's rules. It adds `commands/operator`, `commands/map`, read
   access to `operator` and `map/frame`, and an allow-list: only accounts under
   `operators/<uid> = true` may send operator requests or hold a map lease.
2. **Operators.** In the Firebase console add `operators/<uid>: true` for each phone account
   that may control the robot (the uid is on the Authentication page).
3. **Pi.** Deploy the entire `RasPi/` directory, then `bash RasPi/run_headless.sh doctor` and
   `start`.
4. **Phone.** Install the app release from the app repository.

Changing the live rules and the operator list are the owner's steps: nothing here touched the
live database.

## Heat

Measured first, as planned. `tools/thermal_profile.py` samples temperature, clock, fan and the
CPU use of the bridge, the LiDAR driver, the display server and ssh.

Isolated on this Pi 5 (no LiDAR port, no robot link, no cloud; `artifacts/work-2026-11-14/thermal/`):

| Run | Temperature | All cores busy | Bridge | Display server |
| --- | --- | --- | --- | --- |
| Headless | 38.0 °C | 3.7 % | 0.5 % of one core | 0.2 % |
| Monitor window (virtual display) | 38.2 °C | 3.7 % | 1.0 % of one core | 0.6 % |

Perception on recorded real scans costs about 6 ms per scan, about 3.5 % of one core at the
LiDAR's 5.75 scans per second.

**What this says:** the Pi software by itself is a small load on a Pi 5, and the monitor window
costs about as much again. That is not enough to overheat a Pi 5 with a working fan. These runs
had no LiDAR driver and no `ssh -Y` window, which are the two loads still unmeasured, so before
anything else is changed, measure a real run:

```bash
python3 tools/thermal_profile.py --seconds 600 --out ~/garby-logs/thermal-run.csv
```

Run it once during a real headless run and once with the monitor over `ssh -Y`. If the Pi stays
under about 70 °C in both, the heat is elsewhere: the Pi's enclosure and airflow, the buck
converter, the motor drivers or an ESP32. If the `ssh -Y` run is hot and the headless one is
not, running headless is the fix.

Changes made for load, all in place:

- One conversion of each scan instead of one per consumer (`RasPi/scan_frame.py`).
- The phone map costs nothing while nobody has it open, and halves its rate on a hot Pi.
- Headless runs draw nothing at all.
- No change was made to the monitor's drawing: its measured cost does not justify one.

Memory is not the problem: about 1.2 GB of 4 GB was in use.

## Tests run here

| Check | Result |
| --- | --- |
| `test_cloud_control.py` (relay, age limits incl. a slow claim, mirror, map frames, golden frame) | 21 pass |
| `test_runtime.py` phone tests through the production functions | pass |
| `test_launcher.py` (`run_headless.sh` against a stand-in bridge) | 7 pass |
| Isolated real bridge, headless: starts, writes its status file, stops on SIGTERM | "Shutdown complete" |
| Full Pi suite, firmware host tests, source sanity | see `artifacts/work-2026-11-14/` |

## First use, step by step

Wheels lifted for 3 to 5.

1. `bash RasPi/run_headless.sh doctor`, then `start`, then `status`. Send back the output.
2. Open the app. Controls shows "Pi and controllers connected" and the state. If it says the Pi
   is not reporting, send back `status` and the last 30 lines of `logs`.
3. Press **EMERGENCY STOP** on the phone. The button turns to "STOPPED / Confirmed by the
   controllers" and the Pi log shows `Operator action requested: ESTOP (from the phone)`.
   Note the delay in seconds.
4. Start a route from the monitor or by manual disposal, stop it from the phone, then try
   **Continue run** and, on a second attempt, **Reverse to start**. Send back the log between
   `Operator action requested` and the next `Executor state:` line.
5. Open the Map tab. Wave a hand board or walk in front of the LiDAR: points move within about
   a second, and the title says "Live". Close the tab; `status` and the log show no more frames.
6. Switch the phone to aeroplane mode while a route runs (supervised, hand on the robot's own
   stop): the robot keeps to its own rules and the app says the phone is offline.

## Limits

- **The phone's stop is not a safety device.** It needs the phone's internet, Firebase and the
  Pi's Wi-Fi, and the Pi looks for requests once a second. Expect one to three seconds, and
  nothing at all without coverage. Keep a way to stop the robot at the robot.
- **Wi-Fi along the route** is unknown. Where the Pi has none, the app shows the Pi as not
  reporting; the robot keeps running on its own rules and SMS notices continue.
- The Pi's clock must be right (NTP) for the age limits; a wrong clock makes requests look
  expired, which refuses them, never the opposite.
- Nothing here was exercised against the live database: rules, the operator list and a real
  phone are the first use above.
