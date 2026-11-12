#include "MAIN_EXECUTOR_CODE.h"

// ============================================================
// SETUP
// ============================================================
void setup() {
  initOperatorControls();

  setCpuFrequencyMhz(EXECUTOR_CPU_MHZ);

  Serial.begin(115200);
  Serial.printf("[EXECUTOR BUILD:%s DIR1_UP=%s DIR2_UP=%s TURNS=MPU]\n", EXECUTOR_BUILD_ID,
                MOTOR1_DIR_HIGH_COUNTS_UP ? "HIGH" : "LOW",
                MOTOR2_DIR_HIGH_COUNTS_UP ? "HIGH" : "LOW");
  Air780.begin(115200, SERIAL_8N1, AIR_RX, AIR_TX);
  // The bridge streams 50 Hz MPU lines (~1.8 kB/s) beside its commands. The
  // buffer absorbs a bounded burst while a CPU-throttled loop delays pollESP().
  ESP_Serial.setRxBufferSize(1024);
  ESP_Serial.begin(115200, SERIAL_8N1, ESP_RX, ESP_TX);
  ESP_Serial.setTimeout(50);
  delay(250);
  flushESPSerial();

  // The cellular modem is operational telemetry, not a motion prerequisite.
  // Never block the safety/control handshake waiting for a network registration.

  // ── Stepper Motor Setup ────────────────────────────────────
  engine.init();
  stepper1 = engine.stepperConnectToPin(STEP_PIN1);
  stepper2 = engine.stepperConnectToPin(STEP_PIN2);
  if (stepper1) {
    stepper1->setDirectionPin(DIR_PIN1, MOTOR1_DIR_HIGH_COUNTS_UP);
    stepper1->setSpeedInHz(MAX_SPEED);
    stepper1->setAcceleration(ACCELERATION);
    Serial.println("[Motor 1] Initialized (Step: 21, Dir: 22)");
  } else {
    Serial.println("[Motor 1] FAILED to connect!");
  }
  if (stepper2) {
    stepper2->setDirectionPin(DIR_PIN2, MOTOR2_DIR_HIGH_COUNTS_UP);
    stepper2->setSpeedInHz(MAX_SPEED);
    stepper2->setAcceleration(ACCELERATION);
    Serial.println("[Motor 2] Initialized (Step: 13, Dir: 12)");
  } else {
    Serial.println("[Motor 2] FAILED to connect!");
  }
  heading.attach(stepper1, stepper2);
  initPowerControl();
  delay(100);

  // ── Servo Setup ───────────────────────────────────────────
  // The sonar sweep (SonarSweep.h) moves it from loop(); this only centres it.
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2500);
  servo.write(DEFAULT_VIEW);
  Serial.printf("[SERVO] Ready: forward %d, sweep %d..%d (servo commands)\n",
                DEFAULT_VIEW, SCAN_RIGHT, SCAN_LEFT);

  // ── Sensor / Buzzer Pin Setup ─────────────────────────────
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  startBuzzer();
  initUltrasonicMonitor();
  delay(100);

  // ── Load Cell Setup ───────────────────────────────────────
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  startLoadCellMonitor();

  // ── Handshake: Signal BLE bridge that MCU hardware setup is complete ───────────
  Serial.println("[BOOT] Setup finished. Sending [MCU READY] to BLE bridge...");
  flushESPSerial();
  ESP_Serial.println("[MCU READY]");
  startModemInitialization();

  // loop() already services UART, sonar and the bounded handshake retry.
  // Enter it immediately; waiting here added up to eight seconds to startup.
  // The motors stay fail-closed until fresh path confirmations arrive.
  Serial.println("[BOOT] Hardware ready; BLE handshake continues in main loop.");

  // The modem background task sends the restart SMS only after it is ready.
}

// Parked at the destination or a route fault, possibly for hours. The Pi's own 4 Hz
// stream keeps path data fresh, so polling at the moving rate only costs BLE airtime
// and Pi CPU. Same rate as IDLE.
static void requestParkedStatus() {
  static unsigned long lastParkedRequestMs = 0;
  const unsigned long now = millis();
  if (lastParkedRequestMs != 0 && now - lastParkedRequestMs < PARKED_STATUS_INTERVAL_MS) return;
  lastParkedRequestMs = now;
  requestStatus();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  static unsigned long lastIdleStatusRequestMs = 0;
  static unsigned long lastIdleSensorLogMs = 0;
  static uint32_t evaluatedLoadSampleSequence = 0;
  static unsigned long evaluatedSensorPacketMs = 0;
  static float filteredLoadKg = 0.0f;
  static bool loadFilterReady = false;
  static int loadConfirmCount = 0;
  static int gasConfirmCount  = 0;
  static GarbyState previousState = GarbyState::IDLE;
  const bool enteredIdle = garbyState == GarbyState::IDLE &&
                           previousState != GarbyState::IDLE;
  previousState = garbyState;
  if (enteredIdle) {
    routeFaultLatched = false;
    loadConfirmCount = 0;
    gasConfirmCount = 0;
    loadFilterReady = false;
    filteredLoadKg = 0.0f;
    evaluatedSensorPacketMs = 0;
    evaluatedLoadSampleSequence = 0;
  }
  // The persistent modem owner handles all cellular UART traffic. Its task
  // creation retry is bounded; this call never waits for a modem or network.
  startModemInitialization();
  pollESP();
  serviceUltrasonic();
  serviceBridgeRecovery();
  if (lastSensorPacketMs != 0 &&
      millis() - lastSensorPacketMs > SENSOR_DATA_TIMEOUT_MS) {
    sensorTripped = false;
  }

  // Only the top-level loop can retire an interrupted route stack. RELEASE
  // is rejected until this point is reached and the controller reports IDLE.
  if (serviceOperatorStop()) {
    loadConfirmCount = gasConfirmCount = 0;
    loadFilterReady = false;
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
  }

  // ── IDLE ──────────────────────────────────────────────────
  if (garbyState == GarbyState::IDLE) {
    printIdleUptime();
    const unsigned long idleNow = millis();
    if (idleNow - lastIdleStatusRequestMs >= 500UL) {
      lastIdleStatusRequestMs = idleNow;
      requestStatus();
    }
    const bool logIdleSensor =
      idleNow - lastIdleSensorLogMs >= 1000UL;
    if (logIdleSensor) lastIdleSensorLogMs = idleNow;

    bool newLoadSample = false;
    float absW;
    uint32_t sampleSequence;
    const bool loadValid = getLoadCellSample(absW, sampleSequence);
    if (!loadValid) {
      loadFilterReady = false;
      filteredLoadKg = 0.0f;
      loadConfirmCount = 0;
      evaluatedLoadSampleSequence = sampleSequence;
    } else if (sampleSequence != evaluatedLoadSampleSequence) {
      evaluatedLoadSampleSequence = sampleSequence;
      if (!loadFilterReady) {
        filteredLoadKg = absW;
        loadFilterReady = true;
      } else {
        filteredLoadKg = filteredLoadKg * 0.65f + absW * 0.35f;
      }
      newLoadSample = true;

      if (filteredLoadKg >= LOAD_TRIGGER_KG && filteredLoadKg <= LOAD_VALID_MAX_KG) {
        loadConfirmCount++;
        if (logIdleSensor) {
          Serial.printf("[IDLE] Load threshold met! Confirm count: %d/2\n",
                        loadConfirmCount);
        }
      } else {
        loadConfirmCount = 0;
      }
    }

    if (logIdleSensor) {
      ESP_Serial.printf("LOAD_CELL:%.2f\n", loadFilterReady ? filteredLoadKg : -1.0f);
    }

    if (lastSensorPacketMs != 0 &&
        lastSensorPacketMs != evaluatedSensorPacketMs) {
      evaluatedSensorPacketMs = lastSensorPacketMs;
      if (sensorTripped) {
        gasConfirmCount++;
      } else {
        gasConfirmCount = 0;
      }
    } else if (lastSensorPacketMs == 0 ||
               idleNow - lastSensorPacketMs > SENSOR_DATA_TIMEOUT_MS) {
      gasConfirmCount = 0;
    }

    bool loadReady = (loadConfirmCount >= 2);
    bool gasReady  = (gasConfirmCount >= 3);

    if (operatorAutomaticStartAllowed() && (loadReady || gasReady)) {
      if (gasReady && !loadcellSMSSent) {
        loadcellSMSSent = queueSMSNotice(SmsNoticeKind::TRIP,
                "GARBY: Dangerous gas detected. Outbound run requested; motion awaits safety clearance.");
      } else if (loadReady && !loadcellSMSSent) {
        loadcellSMSSent = queueSMSNotice(SmsNoticeKind::TRIP,
                "GARBY: Load threshold reached. Outbound run requested; motion awaits safety clearance.");
      }
      loadConfirmCount = 0;
      gasConfirmCount  = 0;
      outboundComplete = false;
      garbyState = GarbyState::RUNNING;
      ESP_Serial.println("[RUNNING]");
      Serial.println("[STATE] IDLE -> RUNNING");
    } else {
      vTaskDelay(pdMS_TO_TICKS(newLoadSample ? 5 : 10));
    }

  // ── RUNNING ────────────────────────────────────────────────
  } else if (garbyState == GarbyState::RUNNING) {
    if (!outboundComplete && !routeFaultLatched) {
      Serial.println("[RUN] Starting outbound run");
      const bool outboundSucceeded = runStart();
      if (outboundSucceeded && !resetQueued) {
        outboundComplete = true;
        smoothDecelStopMotors();
        ESP_Serial.println("[OUTBOUND COMPLETE]");
        queueSMSNotice(SmsNoticeKind::ROUTE,
            "GARBY: Destination reached. Robot is stopped and awaiting an explicit return request.");
        Serial.println("[RUN] Outbound run complete; remaining stationary");
      } else if (!resetQueued) {
        // A missing motor command or aborted segment is not a completed route
        // and must not be replayed from an unknown physical position. Hold
        // the controller fail-closed until an explicit reset/recovery cycle.
        routeFaultLatched = true;
        shouldStop = true;
        emergencyStopMotors();
        Serial.println("[RUN] Outbound segment incomplete; route fault latched");
        reportRouteFault();
        vTaskDelay(pdMS_TO_TICKS(20));
      }
    }

    if (resetQueued) {
      resetQueued = false;
      if (outboundComplete && !routeFaultLatched) {
        outboundComplete = false;
        garbyState = GarbyState::RETURNING;
        ESP_Serial.println("[RETURNING]");
        queueSMSNotice(SmsNoticeKind::ROUTE, "GARBY: Return to base requested. Motion remains subject to safety checks.");
        Serial.println("[STATE] RUNNING -> RETURNING (explicit reset)");
      } else {
        routeFaultLatched = true;
        shouldStop = true;
        emergencyStopMotors();
        reportRouteFault();
      }
    } else {
      // A completed route stays stopped at the destination. A route fault
      // also stays stopped until an explicit reset/recovery cycle.
      requestParkedStatus();
      vTaskDelay(pdMS_TO_TICKS(20));
    }

  // ── RETURNING ─────────────────────────────────────────────
  } else if (garbyState == GarbyState::RETURNING) {
    if (!routeFaultLatched) {
      Serial.println("[RETURN] Starting returnToPointB()");
      if (returnToPointB()) {
        Serial.println("[RETURN] Done.");
        outboundComplete = false;
        fullReset();
        queueSMSNotice(SmsNoticeKind::ROUTE, "GARBY: Return route completed. Robot is stopped at base for the next cycle.");
      } else if (!resetQueued) {
        // A failed return segment must not be replayed from the robot's
        // now-unknown physical position. Latch a stationary route fault;
        // only supervised physical recovery plus an explicit [RESET] clears it.
        routeFaultLatched = true;
        shouldStop = true;
        emergencyStopMotors();
        Serial.println("[RETURN] Route incomplete; route fault latched");
        reportRouteFault();
        vTaskDelay(pdMS_TO_TICKS(20));
      }
    } else {
      // Stationary route fault: hold fail-closed until a supervised [RESET]
      // clears it via fullReset(). The defensive resetQueued clear cannot
      // replay the route because the fault guard above blocks it, and
      // pollESP() never queues a reset while a route fault is latched.
      resetQueued = false;
      requestParkedStatus();
      vTaskDelay(pdMS_TO_TICKS(20));
    }

  }
}
