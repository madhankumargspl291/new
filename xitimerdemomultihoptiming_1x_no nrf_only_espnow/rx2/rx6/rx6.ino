#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define TRIG_PIN D1
#define ECHO_PIN D2
#define LED_PIN LED_BUILTIN
enum StartError {
  START_OK = 0,
  UNKNOWN_ERROR
};

StartError lastStartError = UNKNOWN_ERROR;


unsigned long rx2Segment = 0;
bool objectDetected = false;
bool rx1CrossedFlag = false;
bool rx2GoalCompleted = false;
bool rx1_1CrossedFlag = false;

float lap1Sec = 0, lap2Sec = 0, lap3Sec = 0;
float split1Sec = 0, split2Sec = 0, split3Sec = 0;
float totalSec = 0;

// Webserver
AsyncWebServer server(80);
float distanceThreshold = 20.0;
const char* ssid = "XITIMER";
const char* password = "12345678";

// -- ESP-NOW MACs (replace with your actual MACs)
uint8_t rx1_1MAC[] = { 0xEC, 0x64, 0xC9, 0xCE, 0x0F, 0x20 };
enum PacketType : uint8_t {
  CMD_RESET = 0x01,
  CMD_START = 0x02,
  CMD_THRESHOLD = 0x03,
  CMD_TXDATA = 0x04,
  CMD_ACK = 0x05

};
struct BasePacket {
  uint8_t type;  // matches PacketType enum
};
// --- Data Struct for RX1/RX1.1 → RX2 communication
struct Rx1_1ToRx2Data {
  uint8_t type;
  unsigned long txSegmentDurationMicros;  // TX → RX1
  unsigned long txDetectionMicros;
  unsigned long rx1SegmentDurationMicros;  // RX1 → RX1.1
  unsigned long rx1DetectionMicros;
  unsigned long rx1_1SegmentDurationMicros;  // RX1.1 → RX2
  unsigned long rx1_1DetectionMicros;
};

Rx1_1ToRx2Data dataFromRx1_1;  // incoming timing data (RX1 + RX1.1 segments)
struct AckPacket {
  uint8_t type;   // CMD_ACK
  uint8_t stage;  // 0=TX, 1=RX1, 2=RX1.1, 3=RX2
  unsigned long detectMicros;
};

bool dataFromRx1_1Received = false;
struct StartResetPacket {
  uint8_t type;
  int command;
};
// Control packet (threshold updates)
struct ControlPacket {
  uint8_t type;
  float newDistance;
};
// Example usage in your sketch global area:
unsigned long minTravelTimeMicros = 0;
unsigned long rx2_data_reception_micros = 0;
unsigned long minTravelTime = 200;
bool dataFromRx1Received = false;  // Becomes true when RX1 sends data
bool timerStopped = false;
unsigned long lastDetectionMicrosRX2 = 0;

volatile unsigned long echoStartMicrosRX2 = 0;
volatile unsigned long echoEndMicrosRX2 = 0;
volatile bool echoCapturedMicrosRX2 = false;

// -- Global State --
bool timingSequenceStarted = false;
bool rxDataReceived = false;
unsigned long finalRx2Segment = 0;
unsigned long ultrasonicTimeout = 20000;
unsigned long lastDetectionMillis = 0;
const unsigned long debounceInterval = 200;  // ms debounce for RX2 detection

// --- ISR for ultrasonic echo
void IRAM_ATTR echoISR_RX2() {
  if (digitalRead(ECHO_PIN) == HIGH) echoStartMicrosRX2 = micros();
  else {
    echoEndMicrosRX2 = micros();
    echoCapturedMicrosRX2 = true;
  }
}

// --- ESP-NOW send callback (optional)
void OnDataSent(uint8_t* mac_addr, uint8_t sendStatus) {
  Serial.print("ESP-NOW send to: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" → status: ");
  Serial.println(sendStatus == 0 ? "Success" : "Fail");
}

// --- Helper: send int command (0=RESET,1=START) to peer
bool sendIntCommand(uint8_t* peerMAC, int cmd) {
  int value = cmd;
  int res = esp_now_send(peerMAC, (uint8_t*)&value, sizeof(value));
  // esp_now_send returns 0 on success on ESP8266 Arduino core; OnDataSent will also be invoked.
  return (res == 0);
}

// --- Web handlers forward commands via ESP-NOW ---
void handleReset(AsyncWebServerRequest* request) {
  // Local state clear
  timingSequenceStarted = false;
  rxDataReceived = false;
  timerStopped = false;
  objectDetected = false;
  dataFromRx1Received = false;
  finalRx2Segment = 0;
  rx2Segment = 0;
  rx1CrossedFlag = false;
  rx1_1CrossedFlag = false;
  rx2GoalCompleted = false;
  lastDetectionMillis = 0;
  rx2_data_reception_micros = 0;
  Serial.println("\n🔄 RX2 Reset complete (local)");

  StartResetPacket packet;
  packet.type = CMD_RESET;
  packet.command = 0;  // RESET

  // Send RESET (0) to RX1 and TX so entire chain resets

  esp_now_send(rx1_1MAC, (uint8_t*)&packet, sizeof(packet));
  bool sent1 = sendIntCommand(rx1_1MAC, 0);
  Serial.printf("📤 RESET forwarded: RX1.1=%s\n", sent1 ? "ok" : "fail");

  request->send(200, "text/plain", "✅ Reset Done");
}

void handleStart(AsyncWebServerRequest* request) {
  Serial.println("handleStart() called (RX2)");


  // Clear all state for fresh start
  timingSequenceStarted = false;
  timerStopped = false;
  rxDataReceived = false;
  finalRx2Segment = 0;
  objectDetected = false;
  dataFromRx1Received = false;
  rx1CrossedFlag = false;
  rx1_1CrossedFlag = false;  // <-- add this
  rx2GoalCompleted = false;
  lastDetectionMillis = 0;
  rx2_data_reception_micros = 0;
  memset(&dataFromRx1_1, 0, sizeof(dataFromRx1_1));  // <-- clear old data

  StartResetPacket packet;
  packet.type = CMD_START;
  packet.command = 1;  // START

  bool sent = (esp_now_send(rx1_1MAC, (uint8_t*)&packet, sizeof(packet)) == 0);


  if (sent) {
    Serial.println("📤 START forwarded to RX1 (ESP-NOW)");
    request->send(200, "text/plain", "✅ Start sent (via ESP-NOW)");
  } else {
    Serial.println("❌ Failed to send START to RX1 (ESP-NOW)");
    request->send(503, "text/plain", "⚠️ Failed to send START (ESP-NOW)");
  }
}

void handleStatus(AsyncWebServerRequest* request) {
  StaticJsonDocument<2048> doc;

  doc["txCrossed"] = timingSequenceStarted ? 1 : 0;
  doc["timerStarted"] = timingSequenceStarted ? 1 : 0;
  doc["timerStopped"] = timerStopped ? 1 : 0;
  doc["rx1Crossed"] = rx1CrossedFlag ? 1 : 0;
  doc["rx1_1Crossed"] = rx1_1CrossedFlag ? 1 : 0;
  doc["rx2GoalCompleted"] = rx2GoalCompleted ? 1 : 0;
  doc["distanceThreshold"] = distanceThreshold;

  // --- status message ---
  String statusMessage;
  if (!timingSequenceStarted) statusMessage = "🏁 Ready – waiting for start signal";
  else if (timingSequenceStarted && !rx1CrossedFlag) statusMessage = "⏱ Race started!";
  else if (rx1CrossedFlag && !rx1_1CrossedFlag) statusMessage = "✅ First checkpoint passed";
  else if (rx1_1CrossedFlag && !rx2GoalCompleted) statusMessage = "✅ Almost there – final stretch!";
  else if (rx2GoalCompleted) statusMessage = "🎉 Finished! Great job!";

  doc["statusMessage"] = statusMessage;

  // --- partial or full splits ---
  JsonArray segmentArray = doc.createNestedArray("segmentSplits");
  JsonArray cumulativeArray = doc.createNestedArray("cumulativeSplits");
  JsonArray lapLabels = doc.createNestedArray("lapLabels");

  auto round2 = [](float v) {
    return roundf(v * 100) / 100.0;
  };

  // TX → RX1
  segmentArray.add(round2(lap1Sec));
  cumulativeArray.add(round2(split1Sec));
  lapLabels.add("TX->RX1");

  // RX1 → RX1.1
  if (rx1_1CrossedFlag || rx2GoalCompleted) {
    segmentArray.add(round2(lap2Sec));
    cumulativeArray.add(round2(split2Sec));
    lapLabels.add("RX1->RX1.1");
  }

  // RX1.1 → RX2
  if (rx2GoalCompleted) {
    segmentArray.add(round2(lap3Sec));
    cumulativeArray.add(round2(split3Sec));
    lapLabels.add("RX1.1->RX2");
  }

  doc["lapsCount"] = lapLabels.size();
  doc["totalDurationSec"] = round2(split1Sec + split2Sec + split3Sec);

  String jsonOutput;
  serializeJson(doc, jsonOutput);
  request->send(200, "application/json", jsonOutput);
}

void onEspNowDataRecv(uint8_t* mac, uint8_t* data, uint8_t len) {
  if (len < 1) return;  // at least 1 byte for type
  uint8_t type = data[0];

  switch (type) {
    case CMD_RESET:
      {
        StartResetPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));
        Serial.println("🔄 RX2: RESET received");
        // clear state …
        timingSequenceStarted = false;
        rx1CrossedFlag = false;
        rx1_1CrossedFlag = false;
        rx2GoalCompleted = false;
        timerStopped = false;
        dataFromRx1_1Received = false;
        break;
      }
    case CMD_START:
      {
        StartResetPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));
        Serial.println("🚦 RX2: START received");
        timingSequenceStarted = true;
        break;
      }
    case CMD_THRESHOLD:
      {
        ControlPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));
        distanceThreshold = pkt.newDistance;
        Serial.printf("📥 RX2: New threshold = %.2f cm\n", distanceThreshold);
        // forward to TX + RX1 if needed
        esp_now_send(rx1_1MAC, (uint8_t*)&pkt, sizeof(pkt));
        break;
      }
    case CMD_ACK:
      {
        AckPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));

        if (pkt.stage == 0 && !timingSequenceStarted) {
          timingSequenceStarted = true;
          Serial.println("⏱ Timer STARTED (TX crossed)");
        } else if (pkt.stage == 1 && !rx1CrossedFlag) {
          rx1CrossedFlag = true;
          Serial.println("✅ RX1 crossed");
        } else if (pkt.stage == 2 && !rx1_1CrossedFlag) {
          rx1_1CrossedFlag = true;
          Serial.println("✅ RX1.1 crossed");
        }

        break;
      }
    case CMD_TXDATA:
      {
        // TX crossed → timer really starts
        memcpy(&dataFromRx1_1, data, sizeof(dataFromRx1_1));
        rx2_data_reception_micros = micros();
        dataFromRx1_1Received = true;

        Serial.println("RX2: Timing data received from RX1.1");
        break;
      }
    default:
      Serial.printf("RX2: Unknown packet type %d\n", type);
      break;
  }
}
float rx1ToRx2Meters = 25.0f;      // distance between RX1 and RX2 (meters)
float maxExpectedSpeedMps = 9.0f;  // e.g. elite sprint ~10 m/s; set to 6-10 for runners
float guardFactor = 0.3f;          // 0.2-0.4 recommended; 0.3 is a good starting point
float minGuardSec = 0.3f;          // never less than 300 ms

float maxGuardSec = 4.0f;  // clamp to max (optional)

// computed once at startup (or when config changes)
unsigned long computeMinTravelTimeMicros(float distanceMeters, float maxSpeedMps) {
  if (maxSpeedMps <= 0.01f) maxSpeedMps = 1.0f;        // avoid divide-by-zero
  float travelTimeSec = distanceMeters / maxSpeedMps;  // seconds
  float guardSec = travelTimeSec * guardFactor;
  if (guardSec < minGuardSec) guardSec = minGuardSec;
  if (guardSec > maxGuardSec) guardSec = maxGuardSec;
  unsigned long guardMicros = (unsigned long)(guardSec * 1e6f + 0.5f);
  return guardMicros;
}



void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR_RX2, CHANGE);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Setup WiFi AP for app
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Setup web endpoints
  server.on("/start", HTTP_GET, [](AsyncWebServerRequest* request) {
    handleStart(request);
  });
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest* request) {
    handleReset(request);
  });
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    handleStatus(request);
  });
  server.on("/threshold", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("value")) {
      String valueStr = request->getParam("value")->value();
      distanceThreshold = valueStr.toFloat();
      Serial.printf("✅ Distance threshold updated: %.2f cm\n", distanceThreshold);

      // Build control packet and send to RX1 and TX
      ControlPacket packet;
      packet.type = CMD_THRESHOLD;
      packet.newDistance = distanceThreshold;
      esp_now_send(rx1_1MAC, (uint8_t*)&packet, sizeof(packet));


      String jsonResponse = "{\"updatedDistance\":" + String(distanceThreshold, 2) + "}";
      request->send(200, "application/json", jsonResponse);
    } else {
      request->send(400, "application/json", "{\"error\":\"Missing value parameter\"}");
    }
  });
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send(404, "text/plain", "404 Not Found");
  });
  server.begin();

  // --- ESP-NOW Setup ---
  if (esp_now_init() != 0) {
    Serial.println("ESP-NOW INIT FAIL");
    while (1) delay(1000);
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onEspNowDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // Add peers (TX + RX1)
  esp_now_add_peer(rx1_1MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  minTravelTimeMicros = computeMinTravelTimeMicros(rx1ToRx2Meters, maxExpectedSpeedMps);
  Serial.printf("Guard time: %.3f s (%lu us)\n", minTravelTimeMicros / 1e6f, minTravelTimeMicros);
  Serial.println("✅ RX2 Ready (Web + ESP-NOW only)");
}
void loop() {
  //minTravelTime = minTravelTimeMicros;

  // if (timingSequenceStarted && dataFromRx1_1Received && !objectDetected) {
  if (timingSequenceStarted && dataFromRx1_1Received) {
    if (micros() - rx2_data_reception_micros < minTravelTime * 1000UL) {
      return;  // wait until min travel time passed
    }


    float minRange = 0.0;
    //float maxRange = 320.0;
    int consecutive = 0;
    for (int i = 0; i < 3; i++) {
      float d = measureDistance();

      Serial.print("RX4 Distance: ");
      Serial.println(d);
      //Serial.printf("🔹 Threshold: %.1f cm\n", distanceThreshold);
      if ((d > minRange) && (d < distanceThreshold)) {
        Serial.print("d - ");
        Serial.println(d);
        Serial.println();
        consecutive++;
      }
      delay(10);
    }
    Serial.println(consecutive);

    if (consecutive >= 1 && (micros() - lastDetectionMillis) > debounceInterval * 1000UL) {

      lastDetectionMillis = micros();
      objectDetected = true;

      unsigned long rx2DetectionMicros = micros();
      finalRx2Segment = rx2DetectionMicros - rx2_data_reception_micros;

      rx2Segment = finalRx2Segment;

      // ✅ FIX: correct mapping from struct
      // unsigned long lap1 = dataFromRx1_1.txSegmentDurationMicros;           // TX → RX1
      unsigned long lap1 = dataFromRx1_1.rx1SegmentDurationMicros;          // RX1 → RX1.1
      unsigned long lap2 = dataFromRx1_1.rx1_1SegmentDurationMicros;        // RX1 → RX1.1
      unsigned long lap3 = rx2DetectionMicros - rx2_data_reception_micros;  // RX1.1 → RX2

      unsigned long split1 = lap1;
      unsigned long split2 = lap1 + lap2;
      unsigned long split3 = split2 + lap3;
      unsigned long total = split3;

      // Convert to seconds
      lap1Sec = lap1 / 1e6;
      lap2Sec = lap2 / 1e6;
      lap3Sec = lap3 / 1e6;
      split1Sec = split1 / 1e6;
      split2Sec = split2 / 1e6;
      split3Sec = split3 / 1e6;
      totalSec = total / 1e6;

      Serial.println("\n🏁 RX2: OBJECT DETECTED");
      Serial.printf("🔹 Lap1 (TX→RX1): %.2f s\n", lap1Sec);
      Serial.printf("🔹 Lap2 (RX1→RX1.1): %.2f s\n", lap2Sec);
      Serial.printf("🔹 Lap3 (RX1.1→RX2): %.2f s\n", lap3Sec);
      Serial.printf("🔹 Split1 (TX→RX1): %.2f s\n", split1Sec);
      Serial.printf("🔹 Split2 (TX→RX1.1): %.2f s\n", split2Sec);
      Serial.printf("🔹 Split3 (TX→RX2): %.2f s\n", split3Sec);
      Serial.printf("🔸 Total Duration: %.2f s\n", totalSec);

      timerStopped = true;
      dataFromRx1_1Received = false;  // ✅ fix reset
      rx2GoalCompleted = true;
    }
  }
}

float measureDistance() {
  echoCapturedMicrosRX2 = false;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long startWait = micros();
  while (!echoCapturedMicrosRX2 && micros() - startWait < 20000) {}
  if (echoCapturedMicrosRX2) return (echoEndMicrosRX2 - echoStartMicrosRX2) * 0.0343 / 2.0;
  else return -1;
}

// // above is working code but issue in outdoor
// #include <ESP8266WiFi.h>
// #include <espnow.h>
// #include <SPI.h>
// #include <ArduinoJson.h>
// #include <ESPAsyncTCP.h>
// #include <ESPAsyncWebServer.h>

// #define TRIG_PIN D1
// #define ECHO_PIN D2
// #define LED_PIN LED_BUILTIN
// enum StartError {
//   START_OK = 0,
//   UNKNOWN_ERROR
// };

// StartError lastStartError = UNKNOWN_ERROR;


// unsigned long rx2Segment = 0;
// bool objectDetected = false;
// bool rx1CrossedFlag = false;
// bool rx2GoalCompleted = false;
// bool rx1_1CrossedFlag = false;

// float lap1Sec = 0, lap2Sec = 0, lap3Sec = 0;
// float split1Sec = 0, split2Sec = 0, split3Sec = 0;
// float totalSec = 0;

// // Webserver
// AsyncWebServer server(80);
// float distanceThreshold = 20.0;
// const char* ssid = "XITIMER";
// const char* password = "12345678";

// // -- ESP-NOW MACs (replace with your actual MACs)
// uint8_t rx1_1MAC[] = { 0xEC, 0x64, 0xC9, 0xCE, 0x0F, 0x20 };
// enum PacketType : uint8_t {
//   CMD_RESET = 0x01,
//   CMD_START = 0x02,
//   CMD_THRESHOLD = 0x03,
//   CMD_TXDATA = 0x04,
//   CMD_ACK = 0x05

// };
// struct BasePacket {
//   uint8_t type;  // matches PacketType enum
// };
// // --- Data Struct for RX1/RX1.1 → RX2 communication
// struct Rx1_1ToRx2Data {
//   uint8_t type;
//   unsigned long txSegmentDurationMicros;  // TX → RX1
//   unsigned long txDetectionMicros;
//   unsigned long rx1SegmentDurationMicros;  // RX1 → RX1.1
//   unsigned long rx1DetectionMicros;
//   unsigned long rx1_1SegmentDurationMicros;  // RX1.1 → RX2
//   unsigned long rx1_1DetectionMicros;
// };

// Rx1_1ToRx2Data dataFromRx1_1;  // incoming timing data (RX1 + RX1.1 segments)
// struct AckPacket {
//   uint8_t type;   // CMD_ACK
//   uint8_t stage;  // 0=TX, 1=RX1, 2=RX1.1, 3=RX2
//   unsigned long detectMicros;
// };

// bool dataFromRx1_1Received = false;
// struct StartResetPacket {
//   uint8_t type;
//   int command;
// };
// // Control packet (threshold updates)
// struct ControlPacket {
//   uint8_t type;
//   float newDistance;
// };
// // Example usage in your sketch global area:
// unsigned long minTravelTimeMicros = 0;
// unsigned long rx2_data_reception_micros = 0;
// unsigned long minTravelTime = 200;
// bool dataFromRx1Received = false;  // Becomes true when RX1 sends data
// bool timerStopped = false;
// unsigned long lastDetectionMicrosRX2 = 0;

// volatile unsigned long echoStartMicrosRX2 = 0;
// volatile unsigned long echoEndMicrosRX2 = 0;
// volatile bool echoCapturedMicrosRX2 = false;

// // -- Global State --
// bool timingSequenceStarted = false;
// bool rxDataReceived = false;
// unsigned long finalRx2Segment = 0;
// unsigned long ultrasonicTimeout = 20000;
// unsigned long lastDetectionMillis = 0;
// const unsigned long debounceInterval = 200;  // ms debounce for RX2 detection

// // --- ISR for ultrasonic echo
// void IRAM_ATTR echoISR_RX2() {
//   if (digitalRead(ECHO_PIN) == HIGH) echoStartMicrosRX2 = micros();
//   else {
//     echoEndMicrosRX2 = micros();
//     echoCapturedMicrosRX2 = true;
//   }
// }

// // --- ESP-NOW send callback (optional)
// void OnDataSent(uint8_t* mac_addr, uint8_t sendStatus) {
//   Serial.print("ESP-NOW send to: ");
//   for (int i = 0; i < 6; i++) {
//     Serial.printf("%02X", mac_addr[i]);
//     if (i < 5) Serial.print(":");
//   }
//   Serial.print(" → status: ");
//   Serial.println(sendStatus == 0 ? "Success" : "Fail");
// }

// // --- Helper: send int command (0=RESET,1=START) to peer
// bool sendIntCommand(uint8_t* peerMAC, int cmd) {
//   int value = cmd;
//   int res = esp_now_send(peerMAC, (uint8_t*)&value, sizeof(value));
//   // esp_now_send returns 0 on success on ESP8266 Arduino core; OnDataSent will also be invoked.
//   return (res == 0);
// }

// // --- Web handlers forward commands via ESP-NOW ---
// void handleReset(AsyncWebServerRequest* request) {
//   // Local state clear
//   timingSequenceStarted = false;
//   rxDataReceived = false;
//   timerStopped = false;
//   objectDetected = false;
//   dataFromRx1Received = false;
//   finalRx2Segment = 0;
//   rx2Segment = 0;
//   rx1CrossedFlag = false;
//   rx1_1CrossedFlag = false;
//   rx2GoalCompleted = false;
//   lastDetectionMillis = 0;
//   rx2_data_reception_micros = 0;
//   Serial.println("\n🔄 RX2 Reset complete (local)");

//   StartResetPacket packet;
//   packet.type = CMD_RESET;
//   packet.command = 0;  // RESET

//   // Send RESET (0) to RX1 and TX so entire chain resets

//   esp_now_send(rx1_1MAC, (uint8_t*)&packet, sizeof(packet));
//   bool sent1 = sendIntCommand(rx1_1MAC, 0);
//   Serial.printf("📤 RESET forwarded: RX1.1=%s\n", sent1 ? "ok" : "fail");

//   request->send(200, "text/plain", "✅ Reset Done");
// }

// void handleStart(AsyncWebServerRequest* request) {
//   Serial.println("handleStart() called (RX2)");


//   // Clear all state for fresh start
//   timingSequenceStarted = false;
//   timerStopped = false;
//   rxDataReceived = false;
//   finalRx2Segment = 0;
//   objectDetected = false;
//   dataFromRx1Received = false;
//   rx1CrossedFlag = false;
//   rx1_1CrossedFlag = false;  // <-- add this
//   rx2GoalCompleted = false;
//   lastDetectionMillis = 0;
//   rx2_data_reception_micros = 0;
//   memset(&dataFromRx1_1, 0, sizeof(dataFromRx1_1));  // <-- clear old data

//   StartResetPacket packet;
//   packet.type = CMD_START;
//   packet.command = 1;  // START

//   bool sent = (esp_now_send(rx1_1MAC, (uint8_t*)&packet, sizeof(packet)) == 0);


//   if (sent) {
//     Serial.println("📤 START forwarded to RX1 (ESP-NOW)");
//     request->send(200, "text/plain", "✅ Start sent (via ESP-NOW)");
//   } else {
//     Serial.println("❌ Failed to send START to RX1 (ESP-NOW)");
//     request->send(503, "text/plain", "⚠️ Failed to send START (ESP-NOW)");
//   }
// }

// void handleStatus(AsyncWebServerRequest* request) {
//   StaticJsonDocument<2048> doc;

//   doc["txCrossed"] = timingSequenceStarted ? 1 : 0;
//   doc["timerStarted"] = timingSequenceStarted ? 1 : 0;
//   doc["timerStopped"] = timerStopped ? 1 : 0;
//   doc["rx1Crossed"] = rx1CrossedFlag ? 1 : 0;
//   doc["rx1_1Crossed"] = rx1_1CrossedFlag ? 1 : 0;
//   doc["rx2GoalCompleted"] = rx2GoalCompleted ? 1 : 0;
//   doc["distanceThreshold"] = distanceThreshold;

//   // --- status message ---
//   String statusMessage;
//   if (!timingSequenceStarted) statusMessage = "🏁 Ready – waiting for start signal";
//   else if (timingSequenceStarted && !rx1CrossedFlag) statusMessage = "⏱ Race started!";
//   else if (rx1CrossedFlag && !rx1_1CrossedFlag) statusMessage = "✅ First checkpoint passed";
//   else if (rx1_1CrossedFlag && !rx2GoalCompleted) statusMessage = "✅ Almost there – final stretch!";
//   else if (rx2GoalCompleted) statusMessage = "🎉 Finished! Great job!";

//   doc["statusMessage"] = statusMessage;

//   // --- partial or full splits ---
//   JsonArray segmentArray = doc.createNestedArray("segmentSplits");
//   JsonArray cumulativeArray = doc.createNestedArray("cumulativeSplits");
//   JsonArray lapLabels = doc.createNestedArray("lapLabels");

//   auto round2 = [](float v) {
//     return roundf(v * 100) / 100.0;
//   };

//   // TX → RX1
//   segmentArray.add(round2(lap1Sec));
//   cumulativeArray.add(round2(split1Sec));
//   lapLabels.add("TX->RX1");

//   // RX1 → RX1.1
//   if (rx1_1CrossedFlag || rx2GoalCompleted) {
//     segmentArray.add(round2(lap2Sec));
//     cumulativeArray.add(round2(split2Sec));
//     lapLabels.add("RX1->RX1.1");
//   }

//   // RX1.1 → RX2
//   if (rx2GoalCompleted) {
//     segmentArray.add(round2(lap3Sec));
//     cumulativeArray.add(round2(split3Sec));
//     lapLabels.add("RX1.1->RX2");
//   }

//   doc["lapsCount"] = lapLabels.size();
//   doc["totalDurationSec"] = round2(split1Sec + split2Sec + split3Sec);

//   String jsonOutput;
//   serializeJson(doc, jsonOutput);
//   request->send(200, "application/json", jsonOutput);
// }

// void onEspNowDataRecv(uint8_t* mac, uint8_t* data, uint8_t len) {
//   if (len < 1) return;  // at least 1 byte for type
//   uint8_t type = data[0];

//   switch (type) {
//     case CMD_RESET:
//       {
//         StartResetPacket pkt;
//         memcpy(&pkt, data, sizeof(pkt));
//         Serial.println("🔄 RX2: RESET received");
//         // clear state …
//         timingSequenceStarted = false;
//         rx1CrossedFlag = false;
//         rx1_1CrossedFlag = false;
//         rx2GoalCompleted = false;
//         timerStopped = false;
//         dataFromRx1_1Received = false;
//         break;
//       }
//     case CMD_START:
//       {
//         StartResetPacket pkt;
//         memcpy(&pkt, data, sizeof(pkt));
//         Serial.println("🚦 RX2: START received");
//         timingSequenceStarted = true;
//         break;
//       }
//     case CMD_THRESHOLD:
//       {
//         ControlPacket pkt;
//         memcpy(&pkt, data, sizeof(pkt));
//         distanceThreshold = pkt.newDistance;
//         Serial.printf("📥 RX2: New threshold = %.2f cm\n", distanceThreshold);
//         // forward to TX + RX1 if needed
//         esp_now_send(rx1_1MAC, (uint8_t*)&pkt, sizeof(pkt));
//         break;
//       }
//     case CMD_ACK:
//       {
//         AckPacket pkt;
//         memcpy(&pkt, data, sizeof(pkt));

//         if (pkt.stage == 0 && !timingSequenceStarted) {
//           timingSequenceStarted = true;
//           Serial.println("⏱ Timer STARTED (TX crossed)");
//         } else if (pkt.stage == 1 && !rx1CrossedFlag) {
//           rx1CrossedFlag = true;
//           Serial.println("✅ RX1 crossed");
//         } else if (pkt.stage == 2 && !rx1_1CrossedFlag) {
//           rx1_1CrossedFlag = true;
//           Serial.println("✅ RX1.1 crossed");
//         }

//         break;
//       }
//     case CMD_TXDATA:
//       {
//         // TX crossed → timer really starts
//         memcpy(&dataFromRx1_1, data, sizeof(dataFromRx1_1));
//         rx2_data_reception_micros = micros();
//         dataFromRx1_1Received = true;

//         Serial.println("RX2: Timing data received from RX1.1");
//         break;
//       }
//     default:
//       Serial.printf("RX2: Unknown packet type %d\n", type);
//       break;
//   }
// }
// float rx1ToRx2Meters = 25.0f;      // distance between RX1 and RX2 (meters)
// float maxExpectedSpeedMps = 9.0f;  // e.g. elite sprint ~10 m/s; set to 6-10 for runners
// float guardFactor = 0.3f;          // 0.2-0.4 recommended; 0.3 is a good starting point
// float minGuardSec = 0.3f;          // never less than 300 ms

// float maxGuardSec = 4.0f;  // clamp to max (optional)

// // computed once at startup (or when config changes)
// unsigned long computeMinTravelTimeMicros(float distanceMeters, float maxSpeedMps) {
//   if (maxSpeedMps <= 0.01f) maxSpeedMps = 1.0f;        // avoid divide-by-zero
//   float travelTimeSec = distanceMeters / maxSpeedMps;  // seconds
//   float guardSec = travelTimeSec * guardFactor;
//   if (guardSec < minGuardSec) guardSec = minGuardSec;
//   if (guardSec > maxGuardSec) guardSec = maxGuardSec;
//   unsigned long guardMicros = (unsigned long)(guardSec * 1e6f + 0.5f);
//   return guardMicros;
// }



// void setup() {
//   Serial.begin(115200);
//   pinMode(TRIG_PIN, OUTPUT);
//   pinMode(ECHO_PIN, INPUT);
//   attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR_RX2, CHANGE);
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);
//   // Setup WiFi AP for app
//   WiFi.mode(WIFI_AP_STA);
//   WiFi.softAP(ssid, password);
//   Serial.print("AP IP: ");
//   Serial.println(WiFi.softAPIP());

//   // Setup web endpoints
//   server.on("/start", HTTP_GET, [](AsyncWebServerRequest* request) {
//     handleStart(request);
//   });
//   server.on("/reset", HTTP_GET, [](AsyncWebServerRequest* request) {
//     handleReset(request);
//   });
//   server.on("/status", HTTP_GET, [](AsyncWebServerRequest* request) {
//     handleStatus(request);
//   });
//   server.on("/threshold", HTTP_GET, [](AsyncWebServerRequest* request) {
//     if (request->hasParam("value")) {
//       String valueStr = request->getParam("value")->value();
//       distanceThreshold = valueStr.toFloat();
//       Serial.printf("✅ Distance threshold updated: %.2f cm\n", distanceThreshold);

//       // Build control packet and send to RX1 and TX
//       ControlPacket packet;
//       packet.type = CMD_THRESHOLD;
//       packet.newDistance = distanceThreshold;
//       esp_now_send(rx1_1MAC, (uint8_t*)&packet, sizeof(packet));


//       String jsonResponse = "{\"updatedDistance\":" + String(distanceThreshold, 2) + "}";
//       request->send(200, "application/json", jsonResponse);
//     } else {
//       request->send(400, "application/json", "{\"error\":\"Missing value parameter\"}");
//     }
//   });
//   server.onNotFound([](AsyncWebServerRequest* request) {
//     request->send(404, "text/plain", "404 Not Found");
//   });
//   server.begin();

//   // --- ESP-NOW Setup ---
//   if (esp_now_init() != 0) {
//     Serial.println("ESP-NOW INIT FAIL");
//     while (1) delay(1000);
//   }
//   esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
//   esp_now_register_recv_cb(onEspNowDataRecv);
//   esp_now_register_send_cb(OnDataSent);

//   // Add peers (TX + RX1)
//   esp_now_add_peer(rx1_1MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

//   minTravelTimeMicros = computeMinTravelTimeMicros(rx1ToRx2Meters, maxExpectedSpeedMps);
//   Serial.printf("Guard time: %.3f s (%lu us)\n", minTravelTimeMicros / 1e6f, minTravelTimeMicros);
//   Serial.println("✅ RX2 Ready (Web + ESP-NOW only)");
// }
// void loop() {
//   minTravelTime = minTravelTimeMicros / 1000UL;  // convert µs → ms for existing logic


//   if (timingSequenceStarted && dataFromRx1_1Received && !objectDetected) {

//     if (micros() - rx2_data_reception_micros < minTravelTime * 1000UL) {
//       return;  // wait until min travel time passed
//     }


//     float minRange = 0.0;

//     // 🔧 Outdoor-safe effective detection window
//     float minRangeEffective = 15.0;  // ignore anything closer than ~40 cm (ground)
//     float maxRangeEffective = distanceThreshold;
//     if (maxRangeEffective > 150.0) {
//       maxRangeEffective = 150.0;  // clamp maximum detection range
//     }

//     int consecutive = 0;
//     for (int i = 0; i < 3; i++) {
//       float d = measureDistance();

//       Serial.print("RX4 Distance: ");
//       Serial.println(d);

//       if ((d > minRangeEffective) && (d < maxRangeEffective)) {
//         Serial.print("VALID d: ");
//         Serial.println(d);
//         Serial.println();
//         consecutive++;
//       }
//       delay(10);
//     }
//     Serial.println(consecutive);


//     if (consecutive >= 3 && (micros() - lastDetectionMillis) > debounceInterval * 1000UL) {

//       lastDetectionMillis = micros();
//       objectDetected = true;

//       unsigned long rx2DetectionMicros = micros();
//       finalRx2Segment = rx2DetectionMicros - rx2_data_reception_micros;

//       rx2Segment = finalRx2Segment;

//       // ✅ FIX: correct mapping from struct
//       // unsigned long lap1 = dataFromRx1_1.txSegmentDurationMicros;           // TX → RX1
//       unsigned long lap1 = dataFromRx1_1.rx1SegmentDurationMicros;          // RX1 → RX1.1
//       unsigned long lap2 = dataFromRx1_1.rx1_1SegmentDurationMicros;        // RX1 → RX1.1
//       unsigned long lap3 = rx2DetectionMicros - rx2_data_reception_micros;  // RX1.1 → RX2

//       unsigned long split1 = lap1;
//       unsigned long split2 = lap1 + lap2;
//       unsigned long split3 = split2 + lap3;
//       unsigned long total = split3;

//       // Convert to seconds
//       lap1Sec = lap1 / 1e6;
//       lap2Sec = lap2 / 1e6;
//       lap3Sec = lap3 / 1e6;
//       split1Sec = split1 / 1e6;
//       split2Sec = split2 / 1e6;
//       split3Sec = split3 / 1e6;
//       totalSec = total / 1e6;

//       Serial.println("\n🏁 RX2: OBJECT DETECTED");
//       Serial.printf("🔹 Lap1 (TX→RX1): %.2f s\n", lap1Sec);
//       Serial.printf("🔹 Lap2 (RX1→RX1.1): %.2f s\n", lap2Sec);
//       Serial.printf("🔹 Lap3 (RX1.1→RX2): %.2f s\n", lap3Sec);
//       Serial.printf("🔹 Split1 (TX→RX1): %.2f s\n", split1Sec);
//       Serial.printf("🔹 Split2 (TX→RX1.1): %.2f s\n", split2Sec);
//       Serial.printf("🔹 Split3 (TX→RX2): %.2f s\n", split3Sec);
//       Serial.printf("🔸 Total Duration: %.2f s\n", totalSec);

//       timerStopped = true;
//       dataFromRx1_1Received = false;  // ✅ fix reset
//       rx2GoalCompleted = true;
//     }
//   }
// }

// // ***************************************
// // void loop() {
// //   unsigned long minTravelTime = minTravelTimeMicros;

// //   if (timingSequenceStarted && dataFromRx1_1Received && !objectDetected) {
// //     // Wait for minimum travel time before enabling detection
// //     if (micros() - rx2_data_reception_micros > minTravelTime) {

// //       // --- New: single-hit detection with confirmation filter ---
// //       float d = measureDistance();
// //       float background = distanceThreshold + 10;  // assume no object
// //       if (d > 5 && d < distanceThreshold) {
// //         unsigned long now = micros();
// //         if (now - rx2_data_reception_micros > minTravelTime && now - lastDetectionMillis > debounceInterval * 1000UL) {
// //           // Confirm it’s not a false echo: check next few samples quickly
// //           int confirm = 0;
// //           for (int j = 0; j < 3; j++) {
// //             float d2 = measureDistance();
// //             if (d2 > 5 && d2 < distanceThreshold + 5) confirm++;
// //             delay(5);
// //           }
// //           if (confirm >= 1) {  // only 1 stable confirmation needed
// //             // ✅ Confirmed detection
// //             lastDetectionMillis = micros();
// //             objectDetected = true;

// //             unsigned long rx2DetectionMicros = micros();
// //             finalRx2Segment = rx2DetectionMicros - rx2_data_reception_micros;
// //             rx2Segment = finalRx2Segment;

// //             // --- Compute splits ---
// //             unsigned long lap1 = dataFromRx1_1.rx1SegmentDurationMicros;          // RX1 → RX1.1
// //             unsigned long lap2 = dataFromRx1_1.rx1_1SegmentDurationMicros;        // RX1.1 → RX2
// //             unsigned long lap3 = rx2DetectionMicros - rx2_data_reception_micros;  // Final segment

// //             unsigned long split1 = lap1;
// //             unsigned long split2 = lap1 + lap2;
// //             unsigned long split3 = split2 + lap3;
// //             unsigned long total = split3;

// //             // --- Convert to seconds ---
// //             lap1Sec = lap1 / 1e6;
// //             lap2Sec = lap2 / 1e6;
// //             lap3Sec = lap3 / 1e6;
// //             split1Sec = split1 / 1e6;
// //             split2Sec = split2 / 1e6;
// //             split3Sec = split3 / 1e6;
// //             totalSec = total / 1e6;

// //             Serial.println("\n🏁 RX2: OBJECT DETECTED");
// //             Serial.printf("🔹 Lap1 (TX→RX1): %.2f s\n", lap1Sec);
// //             Serial.printf("🔹 Lap2 (RX1→RX1.1): %.2f s\n", lap2Sec);
// //             Serial.printf("🔹 Lap3 (RX1.1→RX2): %.2f s\n", lap3Sec);
// //             Serial.printf("🔹 Split1 (TX→RX1): %.2f s\n", split1Sec);
// //             Serial.printf("🔹 Split2 (TX→RX1.1): %.2f s\n", split2Sec);
// //             Serial.printf("🔹 Split3 (TX→RX2): %.2f s\n", split3Sec);
// //             Serial.printf("🔸 Total Duration: %.2f s\n", totalSec);

// //             timerStopped = true;
// //             dataFromRx1_1Received = false;
// //             rx2GoalCompleted = true;
// //           }
// //         }
// //       }
// //     }
// //   }
// // }
// // ***************************************

// float measureDistance() {
//   echoCapturedMicrosRX2 = false;
//   digitalWrite(TRIG_PIN, LOW);
//   delayMicroseconds(2);
//   digitalWrite(TRIG_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(TRIG_PIN, LOW);

//   unsigned long startWait = micros();
//   while (!echoCapturedMicrosRX2 && micros() - startWait < 20000) {}
//   if (echoCapturedMicrosRX2) return (echoEndMicrosRX2 - echoStartMicrosRX2) * 0.0343 / 2.0;
//   else return -1;
// }