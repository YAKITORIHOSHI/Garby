# Android Firebase configuration

Updated 2026-11-14.

## Client configuration

`app/google-services.json` is synced from `../RasPi/Firebase/google-services.json`; the two files
are byte-identical (SHA-256 `b7cb3fb4dead8e625288cb9dd068f54a8ca5e1cb52eeb8d8a647ee1589b4d582`,
checked 2026-11-14). It registers package `com.thesis.garby` in project `garby-thesis`. The
Realtime Database URL in that file, in `app/src/main/java/com/thesis/garby/realtime/Constants.kt`
and in the Pi runtime must stay the same; `RasPi/test_firebase_contract.py` checks this.

## Credential boundaries

- The app signs in with Firebase **anonymous authentication** (`firestore/AuthViewModel.kt`). It
  contains no password and no service-account key.
- The Admin SDK key under `RasPi/Firebase/` belongs only on the Pi; never copy it into the app.
  `RasPi/Firebase/FireBase.txt` holds a plaintext account password: rotate it if it is current
  and keep future copies in a private secret store.
- `google-services.json` is client configuration, not an Admin credential. Access control comes
  from Authentication settings, database rules and any API-key restrictions.

## What the app reads

| Path | Use |
| --- | --- |
| `.info/connected`, `.info/serverTimeOffset` | Cloud connection; server clock for return ages |
| `devices/garby-bin-01/status` | Heartbeat `lastSeen`, `robotState`, `isReadyToReturn`, BLE/LiDAR/sensor links, CPU temperature, throttle flags |
| `RASPI/STATES` | Legacy fallback for the heartbeat and ready flag |
| `devices/garby-bin-01/sensors/{level,weight,mq135,mq137,mq4}` | Readings with `updatedAt` |
| `RASPI/VALUES/...` | Legacy sensor paths, read only while a device record is absent |
| `devices/garby-bin-01/commands/reset` | The latest return request and the Pi's outcome |
| `devices/garby-bin-01/commands/dispose` | Latest manual-disposal request and matching executor acceptance/rejection |

## What the app writes

| Path | When |
| --- | --- |
| `devices/garby-bin-01/commands/reset/{requestedAt, requestedBy, status, ackAt, doneAt, failedAt, reason}` | One operator-confirmed return: server-timestamp `requestedAt`, the operator's anonymous UID, `pending`, prior outcome fields cleared |
| `devices/garby-bin-01/commands/dispose` | One confirmed transaction: `version: 1`, random 32-hex `requestId`, server `requestedAt`, authenticated UID `requestedBy`, `status: pending`; refuses replacing unresolved requests |
| `devices/garby-bin-01/fcmInstallationId`, `fcmInstallationIdUpdatedAt` | After sign-in: the Firebase Installation ID used by FCM |

The app no longer writes `APP/fcmToken`, `devices/<id>/fcmToken` or `RASPI/fcmToken` (FCM
registration tokens are deprecated), and no longer reads `APP/isReadyToReset`. The Pi writes
everything else with the Admin SDK, which bypasses database rules.

## Database rules

The deployed rules are not in this repository and have not been verified. They should allow
authenticated reads and only the writes above, and should make the return command a fresh,
self-attributed pending request. A starting point, **not tested against the live project**:

```json
{
  "rules": {
    ".read": "auth != null",
    "devices": {
      "$device": {
        "commands": {
          "reset": {
            ".write": "auth != null && newData.child('requestedBy').val() === auth.uid && newData.child('status').val() === 'pending' && newData.child('requestedAt').val() === now"
          }
        },
        "fcmInstallationId": { ".write": "auth != null && newData.isString()" },
        "fcmInstallationIdUpdatedAt": { ".write": "auth != null" }
      }
    }
  }
}
```

Test any rule change with the Firebase emulator or the Rules Playground before deploying: a rule
that rejects the return write shows in the app as "Delivery not confirmed" or "Nothing was sent".

## Manual disposal rules preparation — 2026-11-13

The new path needs an explicit rule when your existing rules only allow `commands/reset`.
[`firebase/manual-disposal.rules.example.json`](firebase/manual-disposal.rules.example.json)
is a **disposal-only merge example**, not a replacement for the project's full rules. It retains
the app's existing authenticated-operator model, requires a new random identity and a fresh
server timestamp, blocks replacement of pending/dispatching requests, and only lets clients
write the five request fields with status `pending`. The Pi Admin SDK alone records
`dispatching`, controller identity, claim time and the terminal result. Parent rules must not
already grant broader writes, because a child rule cannot revoke such permission.

The example has been JSON-checked only; its Firebase rule semantics, deployed permissions and
multi-client transactions require Emulator/Rules Playground acceptance before deployment.
Test: authenticated valid request allowed; unauthenticated, reused identity, old timestamp,
extra claim/result fields, invented accepted status, and replacing unresolved requests denied.
Test a normal request after each terminal result (accepted/failed/unconfirmed) as well.
No live rules or database records were changed by this task.

Status now includes `manualDisposalReady` and `manualDisposalUpdatedAt`, written by the Pi from
fresh executor eligibility. The app requires explicit IDLE and expires eligibility after
15 seconds. Bin/gas/load readings are not prerequisites. The Pi rejects pending requests
over 30 seconds old or more than 5 seconds in the future and claims each one before queuing
one START. A restart only queries the controller ID recorded in the claim. A matching MCU
ACK means `accepted`, not arrival; after 15 seconds without confirmation the claim becomes
`unconfirmed`. Return requests retain their separate path and behavior.
