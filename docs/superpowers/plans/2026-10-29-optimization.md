# Approved optimization implementation

User selected 4 kg and delegated the remaining choices from OPTIMIZATION_REVIEW.md.
Implement inline in the existing workspace (no git repository available).

1. Test empty/below/at-4 kg, negative load and stale confirmations; set 4 kg on
   corrected, filtered weight. Reject negative readings outside the tare deadband.
2. Test failed/in-flight writer shutdown; enforce a flush deadline, report whether
   data flushed, prevent new retries after expiry. Add explicit SDK timeout and
   connection health based on successful operations plus pending work.
3. Test idle/active command polling: 10 seconds idle, 2 seconds at destination or
   during a pending/active return; preserve all request/ACK/completion checks.
4. Test headless scan decisions with no display queue; omit display construction
   there. Remove unused Pi executor classes. Keep scan geometry unchanged.
5. Reduce Android background work by collecting dashboard jobs within STARTED
   lifecycle; prefer current sensor paths, attach legacy fallback only when the
   primary record is absent. Preserve sentinel/freshness and command behavior.
6. Bound executor modem/debug passthrough per iteration. Leave buzzer allocation
   and geometry caching deferred until hardware profiling warrants them.
7. Run whole-project host validation and repeat offline profiling. Record limits:
   no target builds/flashing, no Android execution or live authenticated Firebase.
