#include <ESP8266WiFi.h>
#include <espnow.h>

// --- Pin Definitions ---
#define TRIG_PIN D1  // GPIO16 (D0 on NodeMCU)
#define ECHO_PIN D2  // GPIO15 (D8 on NodeMCU, requires 10k pull-down if used as output during boot)

// --- Ultrasonic timing variables ---
volatile unsigned long echoStart = 0;
volatile unsigned long echoEnd = 0;
volatile bool echoCaptured = false;
unsigned long lastDetectionMicros = 0;  // for precise timing in microseconds

unsigned long detectionEnableTime = 0;  // Time after which detection starts
const unsigned long detectionDelayMs = 200;
bool detectionEnabled = false;
unsigned long lastActivityMillis = 0;
unsigned long lastDetectionMillis = 0;

bool startSignalReceived = false;
unsigned long tx_start_signal_reception_micros;
bool detectionArmed = false;
float distanceThreshold = 20.0;
const unsigned long debounceInterval = 200;

// --- ESP-NOW MAC Addresses ---
// Replace with your actual MACs
uint8_t rx1MAC[] = { 0xEC, 0x64, 0xC9, 0xCE, 0x0F, 0xDE };

enum PacketType : uint8_t {
  CMD_RESET = 0x01,
  CMD_START = 0x02,
  CMD_THRESHOLD = 0x03,
  CMD_TXDATA = 0x04,
  CMD_ACK = 0x05

};

// --- Data Structures ---
struct ControlPacket {
  int type;
  float newDistance;  // used only if type == 2
};
struct StartResetPacket {
  int command;  // 1 = START, 0 = RESET
};
struct CommandPacket {
  uint8_t type;
  float value;  // used only if needed (threshold, etc.)
};

struct TxToRx1Data {
  uint8_t type;
  unsigned long txSegmentDurationMicros;
  unsigned long txDetectionMicros;
};

// --- ACK Packet ---
struct AckPacket {
  uint8_t type;
  uint8_t stage;  // 0 = TX, 1 = RX1, 2 = RX1.1, 3 = RX2
};

// --- ISR for ultrasonic echo ---
void IRAM_ATTR echoISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoStart = micros();
  } else {
    echoEnd = micros();
    echoCaptured = true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  // --- ESP-NOW Setup ---
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    Serial.println("❌ TX ESP-NOW Init Failed!");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(OnDataRecv);
  if (esp_now_add_peer(rx1MAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0) {
    Serial.println("❌ Failed to add RX1 peer");
  }

  Serial.println("✅ TX Ready (ESP-NOW only)");
}

void loop() {
  if (startSignalReceived) {
    if (!detectionEnabled && micros() >= detectionEnableTime) {
      detectionEnabled = true;
      Serial.println("🔵 Detection ENABLED");
    }

    if (detectionArmed) {

      // if (d > 0 && d < distanceThreshold && micros() - (lastDetectionMillis * 1000UL) > (debounceInterval * 1000UL)) {
      float minRange = 0.0;  // ~2.5 m
      int consecutive = 0;
      for (int i = 0; i < 5; i++) {
        float d = measureDistance();
        //Serial.printf("🔹 Threshold: %.1f cm\n", distanceThreshold);
        if (d >= minRange && d <= distanceThreshold) consecutive++;
        delay(10);
      }

      if (consecutive >= 3 && micros() - lastDetectionMicros > debounceInterval * 1000UL) {
        lastDetectionMicros = micros();
        lastActivityMillis = millis();

        unsigned long tx_detection_micros = micros();
        unsigned long txSegmentDuration = tx_detection_micros - tx_start_signal_reception_micros;
        //unsigned long txSegmentDuration = 0;
        Serial.printf("✅ TX detected object. Segment: %.3f s\n", txSegmentDuration / 1e6);

        TxToRx1Data dataToRx1;
        dataToRx1.type = CMD_TXDATA;
        dataToRx1.txSegmentDurationMicros = txSegmentDuration;
        dataToRx1.txDetectionMicros = tx_detection_micros;

        esp_now_send(rx1MAC, (uint8_t *)&dataToRx1, sizeof(dataToRx1));
        Serial.println("📤 TX → RX1: Segment sent");
        // Send ACK for live status
        AckPacket ack;
        ack.type = CMD_ACK;
        ack.stage = 0;  // TX stage
        esp_now_send(rx1MAC, (uint8_t *)&ack, sizeof(ack));
        Serial.println("📡 TX ACK sent (Lap started)");


        detectionArmed = false;
        startSignalReceived = false;
        detectionEnabled = false;
      }
    }
  }
}
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (len < 1) return;  // at least 1 byte needed

  uint8_t type = incomingData[0];

  switch (type) {
    case CMD_RESET:
      {
        Serial.println("🔄 TX: RESET received. State cleared.");
        startSignalReceived = false;
        detectionEnabled = false;
        detectionArmed = false;
        break;
      }
    case CMD_START:
      {
        Serial.println("🚦 TX: START received. Timer begins now.");
        startSignalReceived = true;
        detectionArmed = true;
        tx_start_signal_reception_micros = micros();
        detectionEnableTime = micros() + (detectionDelayMs * 1000UL);
        detectionEnabled = false;
        break;
      }
    case CMD_THRESHOLD:
      {
        if (len >= sizeof(CommandPacket)) {
          CommandPacket pkt;
          memcpy(&pkt, incomingData, sizeof(pkt));
          distanceThreshold = pkt.value;
          Serial.printf("📥 TX: Distance threshold updated → %.2f cm\n", distanceThreshold);
        }
        break;
      }
    case CMD_ACK:
      {
        AckPacket pkt;
        memcpy(&pkt, incomingData, sizeof(pkt));
        Serial.printf("📥 TX received ACK stage=%d (forward if needed)\n", pkt.stage);
        // Optionally forward ACK to RX1 (or ignore if only RX2 needs it)
        break;
      }


    default:
      Serial.printf("⚠️ TX: Unknown packet type %d\n", type);
      break;
  }
}



float measureDistance() {
  echoCaptured = false;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long startWait = micros();
  while (!echoCaptured && micros() - startWait < 20000) {
    // wait max 20 ms
  }

  if (echoCaptured) {
    unsigned long duration = echoEnd - echoStart;
    float distance = duration * 0.0343 / 2.0;
    return distance;
  }
  return -1;
}
