#include <ESP8266WiFi.h>
#include <espnow.h>

// ---------------- PIN DEFINITIONS ----------------
#define TRIG_PIN D1   // GPIO16
#define ECHO_PIN D2   // GPIO15

// ---------------- RX2 MAC ADDRESS ----------------
// 🔴 CHANGE THIS TO YOUR RX2 MAC
uint8_t rx2MAC[] = { 0xC8, 0xc9, 0xA3, 0xA3, 0xDF, 0xEC };

// ---------------- PACKET TYPES ----------------
enum PacketType : uint8_t {
  CMD_RESET     = 0x01,
  CMD_START     = 0x02,
  CMD_THRESHOLD = 0x03,
  CMD_STARTDATA = 0x04,
  CMD_ACK       = 0x05
};

// ---------------- PACKET STRUCTS ----------------
struct ControlPacket {
  uint8_t type;
  float newDistance;
};

struct StartResetPacket {
  uint8_t type;
  uint8_t command;   // 1 = START, 0 = RESET
};

struct StartDataPacket {
  uint8_t type;      // CMD_STARTDATA
  unsigned long startDetectionMicros;
};

struct AckPacket {
  uint8_t type;      // CMD_ACK
  uint8_t stage;     // 1 = RX1 (start gate)
  unsigned long detectMicros;
};

// ---------------- ULTRASONIC ISR DATA ----------------
volatile unsigned long echoStartMicros = 0;
volatile unsigned long echoEndMicros = 0;
volatile bool echoCaptured = false;

// ---------------- STATE VARIABLES ----------------
bool raceStarted = false;
bool detectionSent = false;

unsigned long lastDetectionMicros = 0;
float distanceThreshold = 20.0;

const unsigned long debounceInterval = 200;      // ms
const unsigned long ultrasonicTimeout = 20000;   // µs

// ---------------- ISR ----------------
void IRAM_ATTR echoISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoStartMicros = micros();
  } else {
    echoEndMicros = micros();
    echoCaptured = true;
  }
}

// ---------------- MEASURE DISTANCE ----------------
float measureDistance() {
  echoCaptured = false;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long startWait = micros();
  while (!echoCaptured && micros() - startWait < ultrasonicTimeout) {}

  if (echoCaptured) {
    unsigned long duration = echoEndMicros - echoStartMicros;
    return duration * 0.0343f / 2.0f;
  }
  return -1;
}

// ---------------- ESP-NOW SEND CALLBACK ----------------
void onDataSent(uint8_t *mac, uint8_t status) {
  Serial.println(status == 0 ? "📤 Send OK" : "❌ Send FAIL");
}

// ---------------- ESP-NOW RECEIVE ----------------
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  uint8_t type = incomingData[0];

  switch (type) {

    case CMD_RESET: {
      Serial.println("🔄 RESET received");
      raceStarted = false;
      detectionSent = false;
      break;
    }

    case CMD_START: {
      Serial.println("🚦 START received");
      raceStarted = true;
      detectionSent = false;
      break;
    }

    case CMD_THRESHOLD: {
      ControlPacket pkt;
      memcpy(&pkt, incomingData, sizeof(pkt));
      distanceThreshold = pkt.newDistance;
      Serial.printf("📏 New threshold: %.1f cm\n", distanceThreshold);
      break;
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != 0) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(onDataSent);

  if (esp_now_add_peer(rx2MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) == 0) {
    Serial.println("✅ RX2 paired");
  } else {
    Serial.println("❌ RX2 pairing failed");
  }

  Serial.println("🏁 RX1 START GATE READY");
}

// ---------------- LOOP ----------------
void loop() {
  if (!raceStarted || detectionSent) return;

  float d = measureDistance();

  if (d > 0 && d < distanceThreshold) {

    if (millis() - (lastDetectionMicros / 1000) < debounceInterval) return;
    lastDetectionMicros = micros();

    unsigned long detectMicros = micros();

    Serial.printf("🏁 START DETECTED @ %.6f s\n", detectMicros / 1e6);

    // ---- Send start timing to RX2 ----
    StartDataPacket startPkt;
    startPkt.type = CMD_STARTDATA;
    startPkt.startDetectionMicros = detectMicros;
    esp_now_send(rx2MAC, (uint8_t *)&startPkt, sizeof(startPkt));

    // ---- Send ACK/status ----
    AckPacket ack;
    ack.type = CMD_ACK;
    ack.stage = 1; // RX1
    ack.detectMicros = detectMicros;
    esp_now_send(rx2MAC, (uint8_t *)&ack, sizeof(ack));

    detectionSent = true;
  }
}
