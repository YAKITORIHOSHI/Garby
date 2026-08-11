#include "NAPHTALI_CODE_V2.h"

// ============================================================
// SETUP
// ============================================================
void setup() {

  setCpuFrequencyMhz(EXECUTOR_CPU_MHZ);

  Serial.begin(115200);
  Air780.begin(115200, SERIAL_8N1, AIR_RX, AIR_TX);
  ESP_Serial.begin(115200, SERIAL_8N1, ESP_RX, ESP_TX);
  ESP_Serial.setTimeout(50);
  delay(1000);
  flushESPSerial();

  powerOnAir780();
  if (!waitForModule(10)) {
    Serial.println("[Air780E] WARNING: Module not responding — SMS alerts disabled.");
  } else {
    sendAT("AT+CGSN");
    sendAT("AT+CSQ");
    sendAT("AT+CIMI");
    sendAT("AT+COPS?");
    sendAT("AT+CFUN=1", 5000);
    delay(1000);
    if (!waitForNetwork(10)) {
      Serial.println("[Air780E] Proceeding anyway — SMS may fail.");
    }
    sendAT("AT+CSQ");
    delay(100);
  }

  // ── Stepper Motor Setup ────────────────────────────────────
  engine.init();
  stepper1 = engine.stepperConnectToPin(STEP_PIN1);
  stepper2 = engine.stepperConnectToPin(STEP_PIN2);
  if (stepper1) {
    stepper1->setDirectionPin(DIR_PIN1, false);
    stepper1->setSpeedInHz(MAX_SPEED);
    stepper1->setAcceleration(ACCELERATION);
  }
  if (stepper2) {
    stepper2->setDirectionPin(DIR_PIN2);
    stepper2->setSpeedInHz(MAX_SPEED);
    stepper2->setAcceleration(ACCELERATION);
  }
  Serial.println("[Motor] Setup Done.");
  delay(100);

  // ── Servo Setup ───────────────────────────────────────────
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2500);

  // ── Sensor / Buzzer Pin Setup ─────────────────────────────
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  initUltrasonicMonitor();
  delay(100);

  // ── Load Cell Setup ───────────────────────────────────────
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(500);

  if (scale.wait_ready_timeout(2000)) {
    scale.set_scale(CALIBRATION_FACTOR);
    scale.tare(5);
    long ZERO_FACTOR = scale.get_offset();
    Serial.printf("[DEBUG] LoadCell ready and tared. Zero factor: %ld\n", ZERO_FACTOR);
  } else {
    Serial.println("[LoadCell] WARNING: HX711 not responding after 2s — setting scale factor anyway.");
    scale.set_scale(CALIBRATION_FACTOR);
  }
  delay(100);

  // ── Handshake: Signal BLE bridge that MCU hardware setup is complete ───────────
  Serial.println("[BOOT] Setup finished. Sending [MCU READY] to BLE bridge...");
  flushESPSerial();
  ESP_Serial.println("[MCU READY]");

  bool bleReady = false;
  const unsigned long handshakeStartedMs = millis();
  unsigned long lastPingMs = millis();
  while (!bleReady && millis() - handshakeStartedMs < 8000UL) {
    while (Air780.available()) {
      char c = (char)Air780.read();
      if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '\t') {
        Serial.write(c);
      }
    }


    // Send periodic [MCU READY] handshake signal to BLE bridge every 1000ms
    if (millis() - lastPingMs >= 1000) {
      lastPingMs = millis();
      ESP_Serial.println("[MCU READY]");
    }

    pollESP();
    serviceUltrasonic();
    bleReady = lastConnected;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (bleReady) {
    Serial.println("[BOOT] BLE link confirmed — entering main loop.");
    xTaskCreate(buzzerTask, "garby-ready", 1536, nullptr, 1, nullptr);
  } else {
    // Do not hang forever after a bridge/RasPi restart. The main loop remains
    // fail-closed and keeps advertising [MCU READY] until the link recovers.
    Serial.println("[BOOT] BLE handshake timed out — entering fail-closed recovery mode.");
  }

  queueSMSAlert("[GARBY] Restarted and ready.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  static bool outboundComplete = false;
  static unsigned long lastIdleStatusRequestMs = 0;
  static unsigned long lastIdleSensorLogMs = 0;
  static unsigned long lastLoadSampleMs = 0;
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
    loadConfirmCount = 0;
    gasConfirmCount = 0;
    loadFilterReady = false;
    filteredLoadKg = 0.0f;
    evaluatedSensorPacketMs = 0;
  }
  // Always drain ESP and Air780 pass-through
  pollESP();
  serviceUltrasonic();
  serviceBridgeRecovery();
  updateNudge();
  if (lastSensorPacketMs != 0 &&
      millis() - lastSensorPacketMs > SENSOR_DATA_TIMEOUT_MS) {
    sensorTripped = false;
  }
  if (!smsAlertBusy()) {
    while (Air780.available()) {
      char c = (char)Air780.read();
      if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '\t') {
        Serial.write(c);
      }
    }
    while (Serial.available()) Air780.write(Serial.read());
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
    if (idleNow - lastLoadSampleMs >= 100UL && scale.is_ready()) {
      lastLoadSampleMs = idleNow;
      float w = scale.get_units(1) - 0.010f;
      w = (w > -0.05f && w < 0.05f) ? 0.0f : w;
      const float absW = fabsf(w);
      if (!loadFilterReady) {
        filteredLoadKg = absW;
        loadFilterReady = true;
      } else {
        filteredLoadKg = filteredLoadKg * 0.65f + absW * 0.35f;
      }
      newLoadSample = true;

      if (filteredLoadKg >= MAX_LOAD_APPRX && filteredLoadKg <= 20.0f) {
        loadConfirmCount++;
        if (logIdleSensor) {
          Serial.printf("[IDLE] Load threshold met! Confirm count: %d/2\n",
                        loadConfirmCount);
        }
      } else if (filteredLoadKg < MAX_LOAD_APPRX) {
        loadConfirmCount = 0;
      }
    }

    if (logIdleSensor) {
      if (loadFilterReady) {
        Serial.printf("[IDLE] Filtered load: %.2f kg\n", filteredLoadKg);
        ESP_Serial.printf("LOAD_CELL:%.2f\n", filteredLoadKg);
      } else {
        Serial.println("[IDLE] LoadCell unavailable; loop remains responsive");
      }
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

    if (loadReady || gasReady) {
      if (loadReady && !loadcellSMSSent) {
        queueSMSAlert(
                "[GARBY] Load threshold reached. Moving to Area B.");
        loadcellSMSSent = true;
      }
      if (gasReady && !loadcellSMSSent) {
        queueSMSAlert(
                "[GARBY] Dangerous gas detected. Moving to Area B.");
        loadcellSMSSent = true;
      }
      loadConfirmCount = 0;
      gasConfirmCount  = 0;
      outboundComplete = false;
      garbyState = GarbyState::RUNNING;
      Serial.println("[STATE] IDLE -> RUNNING");
    } else {
      vTaskDelay(pdMS_TO_TICKS(newLoadSample ? 5 : 10));
    }

    // ── RUNNING ────────────────────────────────────────────────
  } else if (garbyState == GarbyState::RUNNING) {
    if (!outboundComplete) {
      Serial.println("[RUN] Starting outbound run");
      runStart();
      if (!resetQueued) {
        outboundComplete = true;
        smoothDecelStopMotors();
        ESP_Serial.println("[OUTBOUND COMPLETE]");
        Serial.println("[RUN] Outbound run complete; remaining stationary");
      }
    }

    if (resetQueued) {
      resetQueued = false;
      outboundComplete = false;
      garbyState = GarbyState::RETURNING;
      Serial.println("[STATE] RUNNING -> RETURNING (explicit reset)");
    } else {
      // Do not execute runStart() again. Stay stopped at the destination and
      // continue servicing communication until an explicit return command.
      requestStatus();
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    // ── RETURNING ─────────────────────────────────────────────
  } else if (garbyState == GarbyState::RETURNING) {
    Serial.println("[RETURN] Starting returnToPointB()");
    returnToPointB();
    Serial.println("[RETURN] Done.");
    outboundComplete = false;
    fullReset();

  }
}
