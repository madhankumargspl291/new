#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SPI.h>

// --- Pin Definitions ---
#define TRIG_PIN D1
#define ECHO_PIN D2

// --- ESP-NOW MAC Addresses ---
uint8_t rx1_1MAC[] = { 0xC8, 0xC9, 0xA3, 0xA3, 0xDF, 0xEC };  // RX1.1
uint8_t rx2MAC[] = { 0x8C, 0xAA, 0xB5, 0x4F, 0xE5, 0x36 };    // RX2 (optional for status)

// --- Data Structures ---
struct ControlPacket {
  int type;
  float newDistance;
};

struct StartResetPacket {
  uint8_t type;
  int command;  // 1 = START, 0 = RESET
};

// RX1 → RX1.1
struct Rx1ToRx1_1Data {
  uint8_t type;
  unsigned long rx1SegmentDurationMicros;
  unsigned long rx1DetectionMicros;
};

struct AckPacket {
  uint8_t type;   
  uint8_t stage;  // 1=RX1, 2=RX1.1, 3=RX2
  unsigned long detectMicros;
};

// --- Ultrasonic variables ---
volatile unsigned long echoStartMicrosRX1 = 0;
volatile unsigned long echoEndMicrosRX1 = 0;
volatile bool echoCapturedMicrosRX1 = false;

unsigned long rx1DetectionMicros = 0;
unsigned long rx1_data_reception_micros = 0;
float distanceThreshold = 20.0;

// --- State variables ---
Rx1ToRx1_1Data dataToRx1_1;
bool detectionArmed = false;
bool detectionDone = false;
bool dataSentToRx1_1 = false;
unsigned long lastDetectionMicros = 0;

const unsigned long debounceInterval = 200;  // ms
const unsigned long minTravelTime = 200;     // ms
unsigned long ultrasonicTimeout = 20000;     // µs

// --- Packet Types ---
enum PacketType : uint8_t {
  CMD_RESET     = 0x01,
  CMD_START     = 0x02,
  CMD_THRESHOLD = 0x03,
  CMD_TXDATA    = 0x04,
  CMD_ACK       = 0x05
};

// --- ISR ---
void IRAM_ATTR echoISR_RX1() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoStartMicrosRX1 = micros();
  } else {
    echoEndMicrosRX1 = micros();
    echoCapturedMicrosRX1 = true;
  }
}

// --- Send callback ---
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("📤 RX1 → RX1.1 status: ");
  Serial.println(sendStatus == 0 ? "Success" : "Fail");
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR_RX1, CHANGE);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("❌ RX1 ESP-NOW init failed!");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  if (esp_now_add_peer(rx1_1MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) == 0)
    Serial.println("✅ RX1.1 added as peer");
  else
    Serial.println("❌ Failed to add RX1.1");

  Serial.println("✅ RX1 Ready (TX removed)");
}

// --- Main Loop ---
void loop() {
  if (detectionArmed && !dataSentToRx1_1) {

    if (micros() - rx1_data_reception_micros < minTravelTime * 1000UL) return;

    int consecutive = 0;
    for (int i = 0; i < 5; i++) {
      float d = measureDistance();
      if (d > 0 && d < distanceThreshold) consecutive++;
      delay(10);
    }

    if (consecutive >= 3 &&
        micros() - lastDetectionMicros > debounceInterval * 1000UL) {

      lastDetectionMicros = micros();
      rx1DetectionMicros = micros();
      unsigned long rx1Segment =
          rx1DetectionMicros - rx1_data_reception_micros;

      Rx1ToRx1_1Data pkt;
      pkt.type = CMD_TXDATA;
      pkt.rx1SegmentDurationMicros = rx1Segment;
      pkt.rx1DetectionMicros = rx1DetectionMicros;

      esp_now_send(rx1_1MAC, (uint8_t *)&pkt, sizeof(pkt));
      Serial.println("📤 RX1 → RX1.1 detection data sent");

      AckPacket ack;
      ack.type = CMD_ACK;
      ack.stage = 1;
      ack.detectMicros = rx1DetectionMicros;
      esp_now_send(rx1_1MAC, (uint8_t *)&ack, sizeof(ack));

      detectionArmed = false;
      detectionDone = true;
      dataSentToRx1_1 = true;
    }
  }
}

// --- Receive handler ---
void onDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  if (len < 1) return;
  uint8_t type = data[0];

  switch (type) {

    case CMD_RESET:
    case CMD_START:
    case CMD_THRESHOLD:
      esp_now_send(rx1_1MAC, data, len);
      detectionArmed = false;
      dataSentToRx1_1 = false;
      Serial.printf("📥 RX1 received command %d, forwarded to RX1.1\n", type);
      break;

    default:
      Serial.printf("⚠️ RX1 Unknown packet type %d\n", type);
      break;
  }
}

// --- Ultrasonic ---
float measureDistance() {
  echoCapturedMicrosRX1 = false;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long startWait = micros();
  while (!echoCapturedMicrosRX1 &&
         micros() - startWait < ultrasonicTimeout) {}

  if (!echoCapturedMicrosRX1) return -1;

  unsigned long duration = echoEndMicrosRX1 - echoStartMicrosRX1;
  return duration * 0.0343 / 2.0;
}
