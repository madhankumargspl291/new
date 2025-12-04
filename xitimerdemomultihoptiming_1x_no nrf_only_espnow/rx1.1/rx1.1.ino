#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SPI.h>

// --- Pin Definitions ---
#define TRIG_PIN D0
#define ECHO_PIN D8

// --- ESP-NOW MAC Addresses ---
uint8_t rx2MAC[] = { 0xEC, 0x64, 0xC9, 0xCD, 0xFB, 0x9D };  // RX2
uint8_t rx1MAC[] = { 0xEC, 0x64, 0xC9, 0xCE, 0x0F, 0xDE };  // RX1

// --- Data Structures ---
struct ControlPacket {
  int type;
  float newDistance;
};

struct StartResetPacket {
  uint8_t type;
  int command;  // 1 = START, 0 = RESET
};

struct Rx1ToRx1_1Data {
  uint8_t type;
  unsigned long txSegmentDurationMicros;
  unsigned long txDetectionMicros;
  unsigned long rx1SegmentDurationMicros;
  unsigned long rx1DetectionMicros;
};

struct Rx1_1ToRx2Data {
  uint8_t type;
  unsigned long txSegmentDurationMicros;
  unsigned long txDetectionMicros;
  unsigned long rx1SegmentDurationMicros;
  unsigned long rx1DetectionMicros;
  unsigned long rx1_1SegmentDurationMicros;
  unsigned long rx1_1DetectionMicros;
};
struct AckPacket {
  uint8_t type;   // CMD_ACK
  uint8_t stage;  // 0=TX, 1=RX1, 2=RX1.1, 3=RX2
  unsigned long detectMicros;
};

// --- Ultrasonic variables ---
volatile unsigned long echoStartMicrosRX11 = 0;
volatile unsigned long echoEndMicrosRX11 = 0;
volatile bool echoCapturedMicrosRX11 = false;

unsigned long rx1_1DetectionMicros = 0;
unsigned long rx1_1data_reception_micros = 0;
float distanceThreshold = 20.0;
bool detectionArmed = false;
bool detectionDone = false;

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
  CMD_RESET = 0x01,
  CMD_START = 0x02,
  CMD_THRESHOLD = 0x03,
  CMD_TXDATA = 0x04,
  CMD_ACK = 0x05

};

// --- ISRs ---
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

  if (esp_now_add_peer(rx2MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) == 0)
    Serial.println("✅ RX2 added as peer");
  else
    Serial.println("❌ Failed to add RX2");

  if (esp_now_add_peer(rx1MAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0) == 0)
    Serial.println("✅ RX1 added as peer");
  else
    Serial.println("❌ Failed to add RX1");

  Serial.println("✅ RX1.1 Ready");
}

// --- Main Loop ---
void loop() {
  if (rxDataReceived && !dataSentToRX2) {
    if (micros() - rx1_1data_reception_micros < minTravelTime * 1000UL) {
      return;  // too soon after receiving previous packet
    }
    float minRange = 0.0;
    //float maxRange = 320.0;
    int consecutive = 0;

    for (int i = 0; i < 5; i++) {
      float d = measureDistance();
      //Serial.printf("🔹 Threshold: %.1f cm\n", distanceThreshold);
      if (d > minRange && d < distanceThreshold) consecutive++;
      delay(10);
    }

    if (consecutive >= 3 && micros() - lastDetectionMicros > debounceInterval * 1000UL) {
      lastDetectionMicros = micros();
      rx1_1DetectionMicros = micros();
      unsigned long rx1_1Segment = rx1_1DetectionMicros - rx1_1data_reception_micros;

      Rx1_1ToRx2Data pkt;
      pkt.type = CMD_TXDATA;
      pkt.txSegmentDurationMicros = dataFromRx1.txSegmentDurationMicros;
      pkt.txDetectionMicros = dataFromRx1.txDetectionMicros;
      pkt.rx1SegmentDurationMicros = dataFromRx1.rx1SegmentDurationMicros;
      pkt.rx1DetectionMicros = dataFromRx1.rx1DetectionMicros;
      pkt.rx1_1SegmentDurationMicros = rx1_1Segment;
      pkt.rx1_1DetectionMicros = rx1_1DetectionMicros;

      esp_now_send(rx2MAC, (uint8_t *)&pkt, sizeof(pkt));
      Serial.println("📤 RX1.1 → RX2 forwarded timing data");
      AckPacket ack;
      ack.type = CMD_ACK;
      ack.stage = 2;  // RX1.1 stage
      ack.detectMicros = rx1_1DetectionMicros;
      esp_now_send(rx2MAC, (uint8_t *)&ack, sizeof(ack));
      Serial.println("📡 RX1.1 ACK sent (Lap 2 crossed)");

      detectionArmed = false;
      detectionDone = true;
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
      {
        StartResetPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));
        rxDataReceived = false;
        dataSentToRX2 = false;
        Serial.println("🔄 RX1.1 RESET received");

        // Forward RESET to RX1
        esp_now_send(rx1MAC, (uint8_t *)&pkt, sizeof(pkt));
        break;
      }

    case CMD_START:
      {
        StartResetPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));
        rxDataReceived = false;
        dataSentToRX2 = false;
        Serial.println("🚦 RX1.1 START received");

        // Forward START to RX1
        esp_now_send(rx1MAC, (uint8_t *)&pkt, sizeof(pkt));
        break;
      }

    case CMD_THRESHOLD:
      {
        ControlPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));
        distanceThreshold = pkt.newDistance;
        Serial.printf("📥 RX1.1 new distance threshold: %.2f cm\n", distanceThreshold);

        // Forward to RX1
        esp_now_send(rx1MAC, (uint8_t *)&pkt, sizeof(pkt));
        break;
      }

    case CMD_TXDATA:
      {
        memcpy(&dataFromRx1, data, sizeof(dataFromRx1));
        rx1_1data_reception_micros = micros();
        rxDataReceived = true;
        dataSentToRX2 = false;


        detectionArmed = true;
        detectionDone = false;
        Serial.println("📥 RX1.1 received data from RX1");
        break;
      }
    case CMD_ACK:
      {
        AckPacket pkt;
        memcpy(&pkt, data, min(len, (uint8_t)sizeof(pkt)));
        Serial.printf("📥 RX1_1 received ACK stage=%d, forward if needed\n", pkt.stage);

        // Forward ACK down the chain (to RX1.1)
        esp_now_send(rx2MAC, (uint8_t *)&pkt, sizeof(pkt));
        break;
      }

    default:
      Serial.printf("⚠️ RX1.1 Unknown packet type %d\n", type);
      break;
  }
}

// --- Ultrasonic Measurement ---
float measureDistance() {
  echoCapturedMicrosRX11 = false;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long startWait = micros();
  while (!echoCapturedMicrosRX11 && micros() - startWait < ultrasonicTimeout) {}

  if (echoCapturedMicrosRX11) {
    unsigned long duration = echoEndMicrosRX11 - echoStartMicrosRX11;
    return duration * 0.0343 / 2.0;
  } else {
    return -1;  // timeout
  }
}
