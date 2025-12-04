#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SPI.h>  // Common library, not specifically for NRF here

// --- Pin Definitions ---
#define TRIG_PIN D0  // GPIO16 (D0 on NodeMCU)
#define ECHO_PIN D8  // GPIO15 (D8 on NodeMCU, requires 10k pull-down if used as output during boot)

// --- ESP-NOW MAC Addresses ---
// Replace these with your actual MAC addresses
uint8_t txMAC[] = { 0xEC, 0x64, 0xC9, 0xCD, 0xFA, 0x6E };     // TX
uint8_t rx1_1MAC[] = { 0xEC, 0x64, 0xC9, 0xCE, 0x0F, 0x20 };  // RX1.1 (next)

// --- Packet structs ---
struct ControlPacket {
  int type;
  float newDistance;
};

struct StartResetPacket {
  uint8_t type;
  int command;  // 1 = START, 0 = RESET
};

struct TxToRx1Data {
  uint8_t type;  // CMD_TXDATA
  unsigned long txSegmentDurationMicros;
  unsigned long txDetectionMicros;
};

// This is the packet RX1 sends to RX1.1 (contains TX->RX1 and RX1 detection)
struct Rx1ToRx1_1Data {
  uint8_t type;
  unsigned long txSegmentDurationMicros;  // TX->RX1
  unsigned long txDetectionMicros;
  unsigned long rx1SegmentDurationMicros;  // RX1 detection segment
  unsigned long rx1DetectionMicros;        // RX1 detection timestamp
};
struct AckPacket {
  uint8_t type;   // CMD_ACK
  uint8_t stage;  // 0=TX, 1=RX1, 2=RX1.1, 3=RX2
  unsigned long detectMicros;
};

// Small status packet to notify RX2 (and app) about a detection event
struct StatusPacket {
  uint8_t type;    // reuse same CMD_TXDATA or a new code if desired
  uint8_t nodeId;  // 1 = TX, 2 = RX1, 3 = RX1.1, 4 = RX2 (for example)
  unsigned long detectMicros;
  float distanceCm;
};

volatile unsigned long echoStartMicrosRX1 = 0;
volatile unsigned long echoEndMicrosRX1 = 0;
volatile bool echoCapturedMicrosRX1 = false;

// --- Global State Variables ---
bool txDataReceived = false;
bool rx1DataSent = false;

unsigned long storedTxSegmentDurationMicros = 0;
unsigned long storedTxDetectionMicros = 0;  // TX detection time forwarded from TX
unsigned long rx1_tx_data_reception_micros = 0;
unsigned long lastDetectionMicros = 0;  // Tracks last ESP-NOW or sensor activity

const unsigned long debounceInterval = 200;  // ms — adjust if needed
const unsigned long minTravelTime = 200;     // ms — ignore detections too soon
unsigned long ultrasonicTimeout = 20000;     // microseconds
float distanceThreshold = 20.0;

// --- Packet type values (match other nodes) ---
enum PacketType : uint8_t {
  CMD_RESET = 0x01,
  CMD_START = 0x02,
  CMD_THRESHOLD = 0x03,
  CMD_TXDATA = 0x04,
  CMD_ACK = 0x05

};

// Function prototypes
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len);
float measureDistance();
void IRAM_ATTR echoISR_RX1();
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus);

// ISR for echo capture
void IRAM_ATTR echoISR_RX1() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoStartMicrosRX1 = micros();
  } else {
    echoEndMicrosRX1 = micros();
    echoCapturedMicrosRX1 = true;
  }
}

// Optional send callback (useful for debug)
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("ESP-NOW send callback, status: ");
  Serial.println(sendStatus == 0 ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR_RX1, CHANGE);

  // --- ESP-NOW Setup ---
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("❌ RX1 ESP-NOW Init Failed!");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Add peers: RX1.1 and RX2 and TX
  if (esp_now_add_peer(rx1_1MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0) {
    Serial.println("❌ Failed to add RX1.1 as peer");
  } else {
    Serial.println("✅ RX1.1 added as peer");
  }

  if (esp_now_add_peer(txMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0) {
    Serial.println("❌ Failed to add TX as peer");
  } else {
    Serial.println("✅ TX added as peer");
  }

  Serial.println("✅ RX1 Ready. Waiting for ESP-NOW data from TX or commands from RX2.");
}

void loop() {
  // If we have TX timing data and haven't yet sent RX1 timing to next node
  if (txDataReceived && !rx1DataSent) {

    // small guard to avoid false fast triggers (object cannot teleport)
    if (micros() - rx1_tx_data_reception_micros < minTravelTime * 1000UL) {
      return;
    }

    // quick multi-sample check
    float minRange = 0.0;  // ~0.2 m (adjust per your positioning)
    // float maxRange = 320.0;  // ~3.2 m
    int consecutive = 0;
    for (int i = 0; i < 5; i++) {
      float d = measureDistance();
      //Serial.printf("🔹 Threshold: %.1f cm\n", distanceThreshold);
      if (d > 0 && d > minRange && d < distanceThreshold) consecutive++;
      delay(10);
    }

    if (consecutive >= 3 && (millis() - (lastDetectionMicros / 1000)) > debounceInterval) {
      lastDetectionMicros = micros();

      // RX1 detection timestamp and segment duration
      unsigned long rx1DetectionMicros = micros();
      unsigned long rx1SegmentDurationMicros = rx1DetectionMicros - rx1_tx_data_reception_micros;

      Serial.printf("📏 RX1 detected object. RX1 segment: %.6f s\n", rx1SegmentDurationMicros / 1e6);

      // Prepare packet for RX1.1 (next in chain)
      Rx1ToRx1_1Data pkt;
      pkt.type = CMD_TXDATA;
      pkt.txSegmentDurationMicros = storedTxSegmentDurationMicros;
      pkt.txDetectionMicros = storedTxDetectionMicros;
      pkt.rx1SegmentDurationMicros = rx1SegmentDurationMicros;
      pkt.rx1DetectionMicros = rx1DetectionMicros;

      // Send timing packet to RX1.1
      esp_now_send(rx1_1MAC, (uint8_t *)&pkt, sizeof(pkt));
      Serial.println("📤 RX1 -> RX1.1: forwarded timing packet");
      // Send ACK to RX1.1 (so it can forward to RX2) for live status
      AckPacket ack;
      ack.type = CMD_ACK;
      ack.stage = 1;  // RX1 stage
      ack.detectMicros = rx1DetectionMicros;
      esp_now_send(rx1_1MAC, (uint8_t *)&ack, sizeof(ack));
      Serial.println("📡 RX1 ACK sent (Lap 1 crossed)");

      // Also send a lightweight status update to RX2 for live UI
      StatusPacket s;
      s.type = CMD_TXDATA;  // reuse CMD code, you can define a new one if you prefer
      s.nodeId = 2;         // 2 = RX1 (convention)
      s.detectMicros = rx1DetectionMicros;
      // last measured distance (take one measurement)
      float lastDist = measureDistance();
      s.distanceCm = (lastDist > 0) ? lastDist : -1.0f;
      Serial.println("📤 RX1 -> RX1_1: status update sent");

      rx1DataSent = true;
      txDataReceived = false;  // ready for next sequence (TX will send a new TXDATA)
      Serial.println("✅ RX1 sequence complete.");
    }
  }
}

// Incoming ESP-NOW handler
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (len < 1) return;  // safety
  uint8_t type = incomingData[0];

  switch (type) {
    case CMD_RESET:
      {
        StartResetPacket pkt;
        // Ensure we don't overflow memcpy — incoming len may vary
        memcpy(&pkt, incomingData, min(len, (uint8_t)sizeof(pkt)));
        Serial.println("🔄 RX1: RESET received");

        // Clear state
        txDataReceived = false;
        rx1DataSent = false;
        storedTxSegmentDurationMicros = 0;
        storedTxDetectionMicros = 0;
        rx1_tx_data_reception_micros = 0;

        // Forward RESET to TX so it resets as well
        esp_now_send(txMAC, (uint8_t *)&pkt, sizeof(pkt));
        Serial.println("📤 RX1: Forwarded RESET to TX ✅");
        break;
      }
    case CMD_START:
      {
        StartResetPacket pkt;
        memcpy(&pkt, incomingData, min(len, (uint8_t)sizeof(pkt)));
        Serial.println("🚦 RX1: START received");

        // Clear state
        txDataReceived = false;
        rx1DataSent = false;
        storedTxSegmentDurationMicros = 0;
        storedTxDetectionMicros = 0;
        rx1_tx_data_reception_micros = 0;

        // Forward START to TX
        esp_now_send(txMAC, (uint8_t *)&pkt, sizeof(pkt));
        Serial.println("📤 RX1: Forwarded START to TX ✅");
        break;
      }
    case CMD_THRESHOLD:
      {
        ControlPacket pkt;
        memcpy(&pkt, incomingData, min(len, (uint8_t)sizeof(pkt)));
        distanceThreshold = pkt.newDistance;
        Serial.printf("📥 RX1: New distance threshold = %.2f cm\n", pkt.newDistance);

        // Forward threshold to TX as well
        esp_now_send(txMAC, (uint8_t *)&pkt, sizeof(pkt));
        Serial.println("📤 RX1: Forwarded threshold to TX ✅");
        break;
      }
    case CMD_TXDATA:
      {
        // TX sent timing data to RX1
        if (len >= (int)sizeof(TxToRx1Data)) {
          TxToRx1Data pkt;
          memcpy(&pkt, incomingData, sizeof(pkt));

          storedTxSegmentDurationMicros = pkt.txSegmentDurationMicros;
          storedTxDetectionMicros = pkt.txDetectionMicros;
          rx1_tx_data_reception_micros = micros();

          txDataReceived = true;
          rx1DataSent = false;

          Serial.printf("✅ RX1: Got TX segment %.6f s, detection @ %.6f s\n",
                        storedTxSegmentDurationMicros / 1e6,
                        storedTxDetectionMicros / 1e6);

        } else {
          Serial.println("⚠️ RX1: CMD_TXDATA received but length mismatch");
        }
        break;
      }
    case CMD_ACK:
      {
        AckPacket pkt;
        memcpy(&pkt, incomingData, min(len, (uint8_t)sizeof(pkt)));
        Serial.printf("📥 RX1 received ACK stage=%d, forward if needed\n", pkt.stage);

        // Forward ACK down the chain (to RX1.1)
        esp_now_send(rx1_1MAC, (uint8_t *)&pkt, sizeof(pkt));
        break;
      }

    default:
      Serial.printf("⚠️ RX1: Unknown packet type %d (len=%d)\n", type, len);
      break;
  }
}

// measureDistance uses ISR captured times
float measureDistance() {
  echoCapturedMicrosRX1 = false;

  // Send 10 µs trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for ISR to capture echo (timeout ~20ms)
  unsigned long startWait = micros();
  while (!echoCapturedMicrosRX1 && micros() - startWait < ultrasonicTimeout) {
    // busy wait (short)
  }

  if (echoCapturedMicrosRX1) {
    unsigned long duration = echoEndMicrosRX1 - echoStartMicrosRX1;
    float distance = duration * 0.0343f / 2.0f;  // cm
    return distance;
  } else {
    return -1;  // timeout → no object
  }
}
