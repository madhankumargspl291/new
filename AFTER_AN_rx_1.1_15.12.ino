#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SPI.h>

// --- Pin Definitions ---
#define TRIG_PIN D1
#define ECHO_PIN D2

// --- ESP-NOW MAC Addresses ---
uint8_t rx2MAC[] = { 0x8C, 0xFA, 0xBC, 0x4C, 0x18, 0xEC };  // RX2
uint8_t rx1MAC[] = { 0xEC, 0xFA, 0xBC, 0x4C, 0x18, 0xa1 };  // RX1

// --- Data Structures ---
struct ControlPacket {
  int type;
  float newDistance;
};

struct StartResetPacket {
  uint8_t type;
  int command;
};

// RX1 → RX1.1 (TX REMOVED)
struct Rx1ToRx1_1Data {
  uint8_t type;
  unsigned long rx1SegmentDurationMicros;
  unsigned long rx1DetectionMicros;
};

// RX1.1 → RX2 (TX REMOVED)
struct Rx1_1ToRx2Data {
  uint8_t type;
  unsigned long rx1SegmentDurationMicros;
  unsigned long rx1DetectionMicros;
  unsigned long rx1_1SegmentDurationMicros;
  unsigned long rx1_1DetectionMicros;
};

struct AckPacket {
  uint8_t type;
  uint8_t stage;  // 1=RX1, 2=RX1.1, 3=RX2
  unsigned long detectMicros;
};

// --- Ultrasonic variables ---
volatile unsigned long echoStartMicrosRX11 = 0;
volatile unsigned long echoEndMicrosRX11 = 0;
volatile bool echoCapturedMicrosRX11 = false;

unsigned long rx1_1DetectionMicros = 0;
unsigned long rx1_1data_reception_micros = 0;
float distanceThreshold = 20.0;

// --- State variables ---
Rx1ToRx1_1Data dataFromRx1;
bool rxDataReceived = false;
bool dataSentToRX2 = false;

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
void IRAM_ATTR echoISR_RX11() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoStartMicrosRX11 = micros();
  } else {
    echoEndMicrosRX11 = micros();
    echoCapturedMicrosRX11 = true;
  }
}

// --- Send callback ---
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("📤 RX1.1 → RX2 status: ");
  Serial.println(sendStatus == 0 ? "Success" : "Fail");
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR_RX11, CHANGE);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("❌ RX1.1 ESP-NOW init failed!");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_add_peer(rx2MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
  esp_now_add_peer(rx1MAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  Serial.println("✅ RX1.1 Ready (TX removed)");
}

// --- Main Loop ---
void loop() {
  if (rxDataReceived && !dataSentToRX2) {

    if (micros() - rx1_1data_reception_micros < minTravelTime * 1000UL) return;

    int consecutive = 0;
    for (int i = 0; i < 5; i++) {
      float d = measureDistance();
      if (d > 0 && d < distanceThreshold) consecutive++;
      delay(10);
    }

    if (consecutive >= 3 &&
        micros() - lastDetectionMicros > debounceInterval * 1000UL) {

      lastDetectionMicros = micros();
      rx1_1DetectionMicros = micros();
      unsigned long rx1_1Segment =
          rx1_1DetectionMicros - rx1_1data_reception_micros;

      Rx1_1ToRx2Data pkt;
      pkt.type = CMD_TXDATA;
      pkt.rx1SegmentDurationMicros = dataFromRx1.rx1SegmentDurationMicros;
      pkt.rx1DetectionMicros = dataFromRx1.rx1DetectionMicros;
      pkt.rx1_1SegmentDurationMicros = rx1_1Segment;
      pkt.rx1_1DetectionMicros = rx1_1DetectionMicros;

      esp_now_send(rx2MAC, (uint8_t *)&pkt, sizeof(pkt));
      Serial.println("📤 RX1.1 → RX2 forwarded timing data");

      AckPacket ack;
      ack.type = CMD_ACK;
      ack.stage = 2;
      ack.detectMicros = rx1_1DetectionMicros;
      esp_now_send(rx2MAC, (uint8_t *)&ack, sizeof(ack));

      dataSentToRX2 = true;
      rxDataReceived = false;
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
      esp_now_send(rx1MAC, data, len);
      rxDataReceived = false;
      dataSentToRX2 = false;
      break;

    case CMD_TXDATA:
      memcpy(&dataFromRx1, data, sizeof(dataFromRx1));
      rx1_1data_reception_micros = micros();
      rxDataReceived = true;
      dataSentToRX2 = false;
      Serial.println("📥 RX1.1 received RX1 data");
      break;

    case CMD_ACK:
      esp_now_send(rx2MAC, data, len);
      break;

    default:
      Serial.printf("⚠️ RX1.1 Unknown packet %d\n", type);
      break;
  }
}

// --- Ultrasonic ---
float measureDistance() {
  echoCapturedMicrosRX11 = false;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long startWait = micros();
  while (!echoCapturedMicrosRX11 &&
         micros() - startWait < ultrasonicTimeout) {}

  if (!echoCapturedMicrosRX11) return -1;

  unsigned long duration = echoEndMicrosRX11 - echoStartMicrosRX11;
  return duration * 0.0343 / 2.0;
}