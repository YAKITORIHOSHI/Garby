#include "stubs/Arduino.h"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"

static unsigned failures = 0;
static void check(bool ok, const char* label) {
  if (!ok) { std::fprintf(stderr, "FAIL %s\n", label); ++failures; }
  else std::printf("PASS %s\n", label);
}

static void resetNotices() {
  tickHook = nullptr; fakeNow = 1000;
  smsQueueState = SmsQueueState{};
  for (auto& monitor : healthMonitors) monitor = NoticeMonitor{};
  linkMonitor = NoticeMonitor{};
  healthPacketSeen = false; healthPacketAtMs = 0;
  bridgeLinkObserved = false; bridgeLinkUp = false;
  blockedNoticeActive = false; blockedNoticeAtMs = 0;
  modemReadyFlag = true; modemServiceBusy = smsWorkerBusy = false;
  operatorEmergencyLatched = false; routeFaultNoticeRaised = false;
  garbyState = GarbyState::IDLE;
  Air780.rx.clear(); Air780.tx.clear(); ESP_Serial.tx.clear();
}

// The substitute is only the UART peripheral. The real AT parser, bounded
// notice queue, retry policy, and production service step all execute below.
static void modemScript(int mode = 0) {
  Air780.tx.clear(); Air780.rx.clear();
  tickHook = [stage = 0, mode]() mutable {
    if (stage == 0 && Air780.tx.find("AT+CMGF=1") != std::string::npos) {
      Air780.rx += mode == 1 ? "\r\nERROR\r\n" : "\r\nOK\r\n"; ++stage;
    } else if (stage == 1 && Air780.tx.find("AT+CMGS=") != std::string::npos) {
      Air780.rx += "\r\n> "; ++stage;
    } else if (stage == 2 && Air780.tx.find(char(26)) != std::string::npos) {
      if (mode != 2) Air780.rx += "\r\n+CMGS: 7\r\n";
      if (mode == 0) Air780.rx += "\r\nOK\r\n";
      ++stage;
    }
  };
}

static bool attempt(int mode = 0) {
  modemScript(mode);
  const bool ran = serviceSmsQueueOnce(fakeNow);
  tickHook = nullptr;
  return ran;
}

static void advanceHealth(const char* packet, uint32_t duration) {
  while (duration) {
    const uint32_t step = std::min(duration, uint32_t(5000));
    fakeNow += step; duration -= step;
    processESPMessage(packet); serviceNoticeMonitoring(fakeNow);
  }
}

int main() {
  fakeNow = 1000;
  Air780.tx.clear(); Air780.rx.clear();
  bool injected = false;
  tickHook = [&]() {
    if (!injected) { Air780.rx = "\r\nBOOK\r\n"; injected = true; }
  };
  sendSMS("+123456789", "Controller notice");
  tickHook = nullptr;
  check(Air780.tx.find("AT+CMGS") == std::string::npos,
        "an unrelated line containing OK cannot authorize SMS submission");
  modemReadyFlag = true; modemServiceBusy = smsWorkerBusy = false;
  fullReset();
  check(!smsWorkerBusy, "a manual state reset cannot claim a completed return by SMS");

  resetNotices(); modemReadyFlag = false;
  const auto queuedAt = fakeNow;
  check(queueSMSNotice(SmsNoticeKind::EMERGENCY, "Emergency stop active.", 'D') && fakeNow == queuedAt,
        "critical notice queues without waiting for an unregistered modem");
  modemReadyFlag = true;
  check(attempt() && Air780.tx.find("Emergency stop active.") != std::string::npos,
        "a notice queued before registration is later submitted by the owner");
  check(!serviceSmsQueueOnce(fakeNow), "a completed notice is removed from the queue");

  resetNotices();
  queueSMSNotice(SmsNoticeKind::BLOCKED, "Old hold", 'D');
  queueSMSNotice(SmsNoticeKind::BLOCKED, "Latest hold", 'D');
  check(attempt() && Air780.tx.find("Latest hold") != std::string::npos &&
        Air780.tx.find("Old hold") == std::string::npos,
        "pending notices coalesce to the latest state of their category");
  queueSMSNotice(SmsNoticeKind::ROUTE, "At destination.");
  check(!attempt(), "ordinary notices cannot bypass the minimum SMS spacing");
  fakeNow += 20000;
  check(attempt(), "another category becomes eligible after SMS spacing");

  resetNotices();
  for (int kind = 0; kind < (int)SmsNoticeKind::COUNT; ++kind) {
    if ((SmsNoticeKind)kind != SmsNoticeKind::EMERGENCY &&
        (SmsNoticeKind)kind != SmsNoticeKind::ROUTE_FAULT)
      queueSMSNotice((SmsNoticeKind)kind, "Routine status");
  }
  check(queueSMSNotice(SmsNoticeKind::EMERGENCY, "Emergency priority", 'D') &&
        attempt() && Air780.tx.find("Emergency priority") != std::string::npos,
        "a full queue preserves and prioritizes an emergency notice");

  resetNotices();
  queueSMSNotice(SmsNoticeKind::LEGACY, "Retry before submission");
  check(attempt(1), "an explicit pre-submission failure consumes one bounded attempt");
  fakeNow += 29000;
  check(!attempt(1), "a failed notice waits through retry backoff");
  fakeNow += 1000;
  check(attempt(1), "a pre-submission failure receives one delayed retry");
  fakeNow += 120000;
  check(!attempt(1), "a failed notice cannot exceed two submission attempts");

  resetNotices();
  queueSMSNotice(SmsNoticeKind::LEGACY, "Uncertain result");
  check(attempt(2), "post-Ctrl-Z timeout finishes with an indeterminate result");
  fakeNow += 60000;
  check(!attempt(), "an uncertain submission is never automatically resent");
  resetNotices(); modemScript(3);
  check(sendSMS("+123456789", "Reference without final response") == SmsSubmitResult::INDETERMINATE,
        "CMGS without final OK cannot be reported as submitted");
  tickHook = nullptr;

  resetNotices();
  bool capped = true;
  for (int i = 0; i < 6; ++i) {
    queueSMSNotice(SmsNoticeKind::LEGACY, (String("Notice ") + String(i)).c_str());
    capped = capped && attempt(); fakeNow += 20000;
  }
  queueSMSNotice(SmsNoticeKind::EMERGENCY, "Held by global attempt cap", 'D');
  check(capped && !attempt(), "all categories share a maximum of six attempts in ten minutes");
  fakeNow = 601100;
  check(attempt() && Air780.tx.find("Held by global attempt cap") != std::string::npos,
        "the latest pending transition survives the global attempt cap");
  resetNotices(); queueSMSNotice(SmsNoticeKind::LEGACY, "Obsolete notice");
  fakeNow += 600001;
  check(!attempt(), "obsolete queued notices expire after ten minutes");

  resetNotices();
  shouldStop = true; pathCommandSeen = false; clearPathCommandCount = 0;
  lastBridgeRxMs = 17; lastPathCommandMs = 23; manualStartLinkReady = false;
  processESPMessage("[HEALTH:1:UUUUU]");
  check(healthPacketSeen && lastBridgeRxMs == 17 && lastPathCommandMs == 23 &&
        !pathCommandSeen && clearPathCommandCount == 0 && shouldStop && !manualStartLinkReady,
        "valid health telemetry changes no handshake, path freshness, or motion permission");
  const auto healthBeforeBad = healthPacketAtMs;
  for (const char* invalid : {"[HEALTH:2:UUUUU]", "[HEALTH:1:UUUU]", "[HEALTH:1:UUUUUU]",
                              "[HEALTH:1:UUUQU]", "[HEALTH:1:uuuuu]"}) {
    fakeNow += 100; processESPMessage(invalid);
  }
  check(healthPacketAtMs == healthBeforeBad, "malformed health packets cannot refresh telemetry evidence");

  resetNotices(); processESPMessage("[HEALTH:1:DXXXX]");
  for (int i = 0; i < 5; ++i) { fakeNow += 5000; processESPMessage("[HEALTH:1:DXXXX]"); serviceNoticeMonitoring(fakeNow); }
  check(!attempt(), "a brief internet outage produces no SMS");
  fakeNow += 5000; processESPMessage("[HEALTH:1:DXXXX]"); serviceNoticeMonitoring(fakeNow);
  check(attempt() && Air780.tx.find("Internet") != std::string::npos,
        "a sustained initially-down internet state produces one outage notice");
  fakeNow += 20000; processESPMessage("[HEALTH:1:DXXXX]"); serviceNoticeMonitoring(fakeNow);
  check(!attempt(), "a continuing confirmed outage cannot repeat its first notice");
  processESPMessage("[HEALTH:1:UXXXX]"); serviceNoticeMonitoring(fakeNow);
  fakeNow += 10000; processESPMessage("[HEALTH:1:UXXXX]"); serviceNoticeMonitoring(fakeNow);
  check(attempt() && Air780.tx.find("recovered") != std::string::npos,
        "healthy evidence sustained after an announced fault produces one recovery");

  resetNotices(); processESPMessage("[HEALTH:1:XXXXD]");
  fakeNow += 10000; processESPMessage("[HEALTH:1:XXXXD]"); serviceNoticeMonitoring(fakeNow);
  attempt(2);
  advanceHealth("[HEALTH:1:XXXXD]", 40000);
  check(!attempt(), "continued fault monitoring cannot recreate an uncertain submission");
  resetNotices(); processESPMessage("[HEALTH:1:XXXXD]");
  fakeNow += 10000; processESPMessage("[HEALTH:1:XXXXD]"); serviceNoticeMonitoring(fakeNow);
  attempt(1); advanceHealth("[HEALTH:1:XXXXD]", 30000); attempt(1);
  advanceHealth("[HEALTH:1:XXXXD]", 30000);
  check(!attempt(1), "continued fault monitoring cannot reset an exhausted retry budget");
  resetNotices(); processESPMessage("[HEALTH:1:XXXXD]");
  fakeNow += 10000; processESPMessage("[HEALTH:1:XXXXD]"); serviceNoticeMonitoring(fakeNow); attempt();
  processESPMessage("[HEALTH:1:XXXXU]");
  for (int i = 0; i < 3; ++i) { fakeNow += 10000; processESPMessage("[HEALTH:1:XXXXU]"); serviceNoticeMonitoring(fakeNow); }
  attempt(2); advanceHealth("[HEALTH:1:XXXXU]", 40000);
  check(!attempt(), "continued recovery monitoring cannot recreate an uncertain submission");

  resetNotices(); processESPMessage("[HEALTH:1:XXXXD]");
  advanceHealth("[HEALTH:1:XXXXD]", 10000); attempt(2);
  fakeNow += 20000; serviceNoticeMonitoring(fakeNow);
  advanceHealth("[HEALTH:1:XXXXD]", 20000);
  check(!attempt(), "unknown health gaps cannot rearm an uncertain fault submission");
  processESPMessage("[HEALTH:1:XXXXU]");
  advanceHealth("[HEALTH:1:XXXXU]", 30000);
  processESPMessage("[HEALTH:1:XXXXD]");
  advanceHealth("[HEALTH:1:XXXXD]", 10000);
  check(attempt(), "a confirmed healthy episode permits a genuinely new fault notice");

  resetNotices();
  queueSMSNotice(SmsNoticeKind::STARTUP, "Startup routine");
  queueSMSNotice(SmsNoticeKind::ROUTE, "Route routine");
  queueSMSNotice(SmsNoticeKind::INTERNET, "Important internet fault", 'D');
  check(attempt() && Air780.tx.find("Important internet fault") != std::string::npos,
        "a confirmed fault is prioritized ahead of older routine notices");
  resetNotices();
  for (SmsNoticeKind kind : {SmsNoticeKind::STARTUP, SmsNoticeKind::TRIP, SmsNoticeKind::ROUTE,
       SmsNoticeKind::LINK, SmsNoticeKind::CLOUD, SmsNoticeKind::LIDAR, SmsNoticeKind::SENSORS,
       SmsNoticeKind::THERMAL, SmsNoticeKind::BLOCKED, SmsNoticeKind::LEGACY})
    queueSMSNotice(kind, "Older routine or recovery", kind == SmsNoticeKind::LINK ? 'U' : 'E');
  check(queueSMSNotice(SmsNoticeKind::INTERNET, "Admitted internet fault", 'D') &&
        attempt() && Air780.tx.find("Admitted internet fault") != std::string::npos,
        "a new confirmed fault displaces routine or recovery traffic from a full queue");

  resetNotices();
  queueSMSNotice(SmsNoticeKind::INTERNET, "Internet down", 'D'); attempt();
  queueSMSNotice(SmsNoticeKind::INTERNET, "Internet recovered", 'U');
  queueSMSNotice(SmsNoticeKind::INTERNET, "Internet down", 'D'); fakeNow += 20000;
  check(!attempt(), "a fault returning before queued recovery cancels the obsolete recovery");
  queueSMSNotice(SmsNoticeKind::INTERNET, "Internet recovered", 'U');
  modemScript();
  const auto originalHook = tickHook;
  bool renewedFault = false;
  tickHook = [&]() {
    originalHook();
    if (!renewedFault && Air780.tx.find(char(26)) != std::string::npos) {
      renewedFault = true;
      queueSMSNotice(SmsNoticeKind::INTERNET, "Internet failed again", 'D');
    }
  };
  serviceSmsQueueOnce(fakeNow); tickHook = nullptr; fakeNow += 20000;
  check(attempt() && Air780.tx.find("Internet failed again") != std::string::npos,
        "a newer fault survives an older recovery already in flight");

  resetNotices(); processESPMessage("[HEALTH:1:XXXXD]");
  fakeNow += 10000; processESPMessage("[HEALTH:1:XXXXD]"); serviceNoticeMonitoring(fakeNow);
  check(attempt() && Air780.tx.find("Thermal") != std::string::npos,
        "thermal fault uses its shorter sustained-fault threshold");
  fakeNow += 20000; serviceNoticeMonitoring(fakeNow);
  check(healthMonitors[4].value == 'X' && !attempt(),
        "missing health frames invalidate evidence without inventing a recovery");
  processESPMessage("[HEALTH:1:XXXXU]"); serviceNoticeMonitoring(fakeNow);
  fakeNow += 10000; processESPMessage("[HEALTH:1:XXXXU]"); serviceNoticeMonitoring(fakeNow);
  check(!attempt(), "thermal recovery requires a longer stable healthy interval");
  fakeNow += 10000; processESPMessage("[HEALTH:1:XXXXU]"); serviceNoticeMonitoring(fakeNow);
  fakeNow += 10000; processESPMessage("[HEALTH:1:XXXXU]"); serviceNoticeMonitoring(fakeNow);
  check(attempt() && Air780.tx.find("recovered") != std::string::npos,
        "thermal recovery submits only after thirty healthy seconds");

  resetNotices(); processESPMessage("[HEALTH:1:DXXXX]");
  fakeNow += 10000; processESPMessage("[HEALTH:1:DXXXX]");
  fakeNow += 20000; serviceNoticeMonitoring(fakeNow);
  check(!attempt(), "a stale outage packet cannot mature into a fault notice");
  resetNotices(); processESPMessage("[HEALTH:1:UUUUU]");
  fakeNow += 12000; processESPMessage("[HEALTH:1:UUUUU]"); serviceNoticeMonitoring(fakeNow);
  check(!attempt(), "initial health cannot produce an unpaired recovery notice");

  resetNotices(); processESPMessage("[BLE CONNECTION ESTABLISHED]");
  processESPMessage("STOP:LINK");
  fakeNow += 10000; serviceNoticeMonitoring(fakeNow);
  check(attempt() && Air780.tx.find("link") != std::string::npos,
        "a sustained observed Pi/controller link failure produces a distinct link notice");
  processESPMessage("[BLE CONNECTION ESTABLISHED]"); serviceNoticeMonitoring(fakeNow);
  for (int i = 0; i < 10; ++i) { fakeNow += 1000; processESPMessage("GO"); serviceNoticeMonitoring(fakeNow); }
  fakeNow += 10000;
  check(attempt() && Air780.tx.find("recovered") != std::string::npos,
        "link recovery needs sustained fresh controller traffic after the handshake");
  resetNotices(); processESPMessage("[BLE CONNECTION ESTABLISHED]");
  processESPMessage("STOP:LINK"); fakeNow += 10000; serviceNoticeMonitoring(fakeNow); attempt();
  processESPMessage("[BLE CONNECTION ESTABLISHED]");
  for (int i = 0; i < 3; ++i) { fakeNow += 4000; processESPMessage("GO"); serviceNoticeMonitoring(fakeNow); }
  fakeNow += 20000;
  check(!attempt(), "late controller frames cannot bridge stale gaps into a false link recovery");

  resetNotices();
  FastAccelStepper first, second; stepper1 = &first; stepper2 = &second;
  first.runForward(); second.runForward();
  processESPMessage("[OP:100:ESTOP]");
  check(!first.isRunning() && !second.isRunning() && attempt() &&
        Air780.tx.find("Emergency stop") != std::string::npos,
        "an operator emergency stops both motors before queuing its critical notice");
  processESPMessage("[OP:100:ESTOP]"); fakeNow += 20000;
  check(!attempt(), "replayed emergency requests cannot generate duplicate stop notices");

  resetNotices(); routeFaultLatched = true; reportRouteFault(); fullReset();
  check(!attempt(), "a reset before fault submission cannot emit an unpaired route recovery");
  resetNotices(); routeFaultLatched = true; reportRouteFault(); attempt();
  fullReset(); fakeNow += 20000;
  check(attempt() && Air780.tx.find("cleared by reset") != std::string::npos,
        "an announced route fault reports explicit reset recovery");
  routeFaultLatched = true; reportRouteFault(); fakeNow += 20000;
  check(attempt() && Air780.tx.find("Route fault") != std::string::npos,
        "a new route fault after reset receives its own transition budget");
  resetNotices(); routeFaultLatched = true; reportRouteFault();
  modemScript();
  const auto routeScript = tickHook;
  bool resetDuringSubmission = false;
  tickHook = [&]() {
    routeScript();
    if (!resetDuringSubmission && Air780.tx.find(char(26)) != std::string::npos) {
      resetDuringSubmission = true; fullReset();
    }
  };
  serviceSmsQueueOnce(fakeNow); tickHook = nullptr; fakeNow += 20000;
  check(attempt() && Air780.tx.find("cleared by reset") != std::string::npos,
        "a reset during in-flight fault submission retains a conditional paired recovery");
  resetNotices(); routeFaultLatched = true; reportRouteFault();
  modemScript(2);
  const auto uncertainRouteScript = tickHook;
  resetDuringSubmission = false;
  tickHook = [&]() {
    uncertainRouteScript();
    if (!resetDuringSubmission && Air780.tx.find(char(26)) != std::string::npos) {
      resetDuringSubmission = true; fullReset();
    }
  };
  serviceSmsQueueOnce(fakeNow); tickHook = nullptr; fakeNow += 20000;
  check(!attempt(), "a conditional local recovery is discarded if in-flight fault submission remains uncertain");

  resetNotices();
  Air780.tx.clear(); Air780.rx.clear();
  tickHook = [stage = 0]() mutable {
    if (stage == 0 && Air780.tx.find("AT+CMGF") != std::string::npos) {
      Air780.rx = "\r\nOK\r\n"; ++stage;
    } else if (stage == 1 && Air780.tx.find("AT+CMGS") != std::string::npos) {
      Air780.rx = "\r\n+NOTICE: value>threshold\r\n"; ++stage;
    }
  };
  check(sendSMS("+123456789", "Do not send on an unsolicited angle bracket") == SmsSubmitResult::RETRYABLE_FAILURE &&
        Air780.tx.find(char(26)) == std::string::npos,
        "an angle bracket inside unsolicited text cannot authorize the SMS body");
  tickHook = nullptr;

  resetNotices(); fakeNow = 0xfffff000U;
  queueSMSNotice(SmsNoticeKind::LEGACY, "Before rollover"); attempt();
  queueSMSNotice(SmsNoticeKind::ROUTE, "After rollover"); fakeNow += 20000;
  check(attempt() && Air780.tx.find("After rollover") != std::string::npos,
        "SMS spacing and queue deadlines survive millis rollover");

  resetNotices();
  check(!queueSMSNotice(SmsNoticeKind::LEGACY, "Injected\ntext") &&
        !queueSMSNotice(SmsNoticeKind::LEGACY, std::string(141, 'A').c_str()) &&
        !queueSMSNotice(SmsNoticeKind::LEGACY, "Non-default alphabet [extension]"),
        "notice validation rejects control bytes, oversized bodies, and multi-septet ASCII extensions");
  check(registeredResponse("\r\n+CREG: 0,1\r\nOK\r\n") &&
        registeredResponse("\r\n+CREG: 0,5\r\nOK\r\n") &&
        !registeredResponse("\r\n+CREG: 0,10\r\nOK\r\n") &&
        !registeredResponse("\r\n+CREG: 0,1\r\nERROR\r\n"),
        "network readiness requires an exact registered status and a successful query");
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
