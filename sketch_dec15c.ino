#include <ESP8266WiFi.h>
#include <espnow.h>

// ---------------- PACKET TYPES ----------------
enum PacketType : uint8_t {
  CMD_STARTDATA = 0x04,
  CMD_ACK = 0x05
};

struct Rx1_1ToRx2Data {
  uint8_t type;
  unsigned long startDetectionMicros;
  unsigned long rx1_1SegmentMicros;
  unsigned long rx1_1DetectionMicros;
};

struct AckPacket {
  uint8_t type;
  uint8_t stage;
  unsigned long detectMicros;
};

// ---------------- RECEIVE ----------------
void onDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  uint8_t type = data[0];

  if (type == CMD_STARTDATA) {
    Rx1_1ToRx2Data pkt;
    memcpy(&pkt, data, sizeof(pkt));

    float startToRX11 = pkt.rx1_1SegmentMicros / 1e6;

    Serial.println("🏁 RACE DATA RECEIVED");
    Serial.printf("Start → RX1.1: %.6f s\n", startToRX11);
  }

  else if (type == CMD_ACK) {
    AckPacket ack;
    memcpy(&ack, data, sizeof(ack));
    Serial.printf("📡 ACK from stage %d @ %.6f s\n",
                  ack.stage, ack.detectMicros / 1e6);
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("✅ RX2 READY");
}

void loop() {}
