#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <espnow.h>
#include <ArduinoJson.h>

/* ---------------- CONFIG ---------------- */
#define LED_PIN LED_BUILTIN

ESP8266WebServer server(80);

/* -------- MACs of RX1.1 and RX2 -------- */
uint8_t rx11MAC[] = {0xEC,0x64,0xC9,0xCE,0x0F,0x11}; // RX1.1
uint8_t rx2MAC[]  = {0xEC,0x64,0xC9,0xCE,0x0F,0x12}; // RX2

/* ---------------- TIMING ---------------- */
unsigned long raceStartMicros = 0;
unsigned long split1Micros = 0;
unsigned long split2Micros = 0;

bool split1Received = false;
bool split2Received = false;

/* ---------------- PACKET ---------------- */
enum PacketType : uint8_t {
  CMD_SEGMENT = 3
};

struct SegmentPacket {
  uint8_t type;
  uint8_t from;                 // 1 = RX1.1, 2 = RX2
  unsigned long segmentMicros;
};
 
/* ---------------- ESP-NOW RX ---------------- */
void onDataRecv(uint8_t* mac, uint8_t* data, uint8_t len) {
  if (len != sizeof(SegmentPacket)) return;

  SegmentPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.type != CMD_SEGMENT) return;

  if (pkt.from == 1 && !split1Received) {
    split1Micros = pkt.segmentMicros;
    split1Received = true;
    Serial.println("✅ Split 1 received");
  }

  if (pkt.from == 2 && !split2Received) {
    split2Micros = pkt.segmentMicros;
    split2Received = true;
    Serial.println("🏁 Finish received");
  }
}

/* ---------------- WEB JSON ---------------- */
void handleResult() {
  StaticJsonDocument<256> doc;

  doc["split1_ms"] = split1Received ? split1Micros / 1000.0 : 0;
  doc["split2_ms"] = split2Received ? split2Micros / 1000.0 : 0;
  doc["total_ms"]  = (split1Received && split2Received)
                     ? (split1Micros + split2Micros) / 1000.0
                     : 0;

  String json;
  serializeJson(doc, json);

  server.send(200, "application/json", json);
}

/* ---------------- START RACE ---------------- */
void startRace() {
  raceStartMicros = micros();
  split1Received = false;
  split2Received = false;

  esp_now_send(rx11MAC, NULL, 0); // trigger RX1.1
  esp_now_send(rx2MAC, NULL, 0);  // trigger RX2

  server.send(200, "text/plain", "Race Started");
  Serial.println("🚦 Race Started");
}

/* ---------------- SETUP ---------------- */
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("RX1_TIMER", "12345678");

  server.on("/start", startRace);
  server.on("/result", handleResult);
  server.begin();

  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);


  Serial.println("🟢 RX1 READY");
}

/* ---------------- LOOP ---------------- */
void loop() {
  server.handleClient();
}
