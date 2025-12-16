#include <ESP8266WiFi.h>
#include <espnow.h>

// ---------------- PIN DEFINITIONS ----------------
#define TRIG_PIN D0
#define ECHO_PIN D8

// ---------------- MAC ADDRESSES ----------------
uint8_t rx1MAC[] = { 0xEC, 0xFA, 0xBC, 0x4C, 0x18, 0xA1 }; // RX1
uint8_t rx2MAC[] = { 0x8C, 0xAA, 0xB5, 0x4F, 0xE5, 0x36 }; // RX2

// ---------------- PACKET TYPES ----------------
enum PacketType : uint8_t {
  CMD_RESET     = 0x01,
  CMD_START     = 0x02,
  CMD_THRESHOLD = 0x03,
  CMD_STARTDATA = 0x04,
  CMD_ACK       = 0x05
};

// ---------------- PACKETS ----------------
struct StartDataPacket {
  uint8_t type;
  unsigned long startDetectionMicros;
};

struct Rx1_1ToRx2Data {
  uint8_t type;
  unsigned long startDetectionMicros;
  unsigned long rx1_1SegmentMicros;
  unsigned long rx1_1DetectionMicros;
};

struct AckPacket {
  uint8_t type;
  uint8_t stage; // 2 = RX1.1
  unsigned long detectMicros;
};

struct ControlPacket {
  uint8_t type;
  float newDistance;
};

// ---------------- ULTRASONIC ----------------
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool echoDone = false;

// ---------------- STATE ----------------
bool startDataReceived = false;
bool dataSent = false;

unsigned long startDetectionMicros = 0;
unsigned long rx1_1DetectionMicros = 0;
unsigned long rx1_1DataRxMicros = 0;

float distanceThreshold = 20.0;
unsigned long lastDetectMicros = 0;

const unsigned long debounceMs = 200;
const unsigned long minTravelMs = 200;
const unsigned long ultrasonicTimeout = 20000;

// ---------------- ISR ----------------
void IRAM_ATTR echoISR() {
  if (digitalRead(ECHO_PIN)) echoStart = micros();
  else {
    echoEnd = micros();
    echoDone = true;
  }
}

// ---------------- DISTANCE ----------------
float measureDistance() {
  echoDone = false;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long t0 = micros();
  while (!echoDone && micros() - t0 < ultrasonicTimeout) {}

  if (!echoDone) return -1;
  return (echoEnd - echoStart) * 0.0343f / 2.0f;
}

// ---------------- RECEIVE ----------------
void onDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  uint8_t type = data[0];

  if (type == CMD_STARTDATA) {
    StartDataPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    startDetectionMicros = pkt.startDetectionMicros;
    rx1_1DataRxMicros = micros();
    startDataReceived = true;
    dataSent = false;

    Serial.println("📥 RX1.1 got START timing");
  }

  else if (type == CMD_RESET) {
    startDataReceived = false;
    dataSent = false;
    Serial.println("🔄 RX1.1 RESET");
  }

  else if (type == CMD_THRESHOLD) {
    ControlPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));
    distanceThreshold = pkt.newDistance;
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_add_peer(rx1MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(rx2MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  Serial.println("✅ RX1.1 READY");
}

// ---------------- LOOP ----------------
void loop() {
  if (!startDataReceived || dataSent) return;

  if (micros() - rx1_1DataRxMicros < minTravelMs * 1000UL) return;

  int hits = 0;
  for (int i = 0; i < 5; i++) {
    float d = measureDistance();
    if (d > 0 && d < distanceThreshold) hits++;
    delay(10);
  }

  if (hits >= 3 && millis() - lastDetectMicros / 1000 > debounceMs) {
    lastDetectMicros = micros();
    rx1_1DetectionMicros = micros();

    Rx1_1ToRx2Data pkt;
    pkt.type = CMD_STARTDATA;
    pkt.startDetectionMicros = startDetectionMicros;
    pkt.rx1_1DetectionMicros = rx1_1DetectionMicros;
    pkt.rx1_1SegmentMicros = rx1_1DetectionMicros - startDetectionMicros;

    esp_now_send(rx2MAC, (uint8_t *)&pkt, sizeof(pkt));

    AckPacket ack{ CMD_ACK, 2, rx1_1DetectionMicros };
    esp_now_send(rx2MAC, (uint8_t *)&ack, sizeof(ack));

    dataSent = true;
  }
}
