Short description

ESP8266-based RX1 split-timing node for a wireless race timing system using ESP-NOW, ultrasonic detection, and a web interface for control and monitoring.

Table of Contents

Features

Requirements

Installation

Usage

Configuration

Examples

Running Tests

Contributing

License

Author

Features

Wireless split-1 timing using ESP-NOW

Ultrasonic-based athlete/object detection

Microsecond-precision timing (micros())

Sends split-1 data to RX1.1 node

Built-in Wi-Fi Access Point for control

REST API endpoints for start, reset, status, and threshold

Debounce and guard-time logic for reliable outdoor detection

Real-time JSON status reporting

Requirements
Hardware

ESP8266 (NodeMCU / ESP-12E)

Ultrasonic sensor (HC-SR04 or equivalent)

Power supply (USB / battery)

Software

Arduino IDE

ESP8266 Board Package v3.x

Libraries:

ESP8266WiFi

espnow

ESPAsyncTCP

ESPAsyncWebServer

ArduinoJson

Installation

Clone the repository:

git clone https://github.com/madhankumargspl291/new.git
cd new/02_01_2025


Open the project in Arduino IDE:

Select Board → NodeMCU 1.0 (ESP-12E)

Select correct COM port

Install required libraries from Library Manager

Upload the sketch to the ESP8266.

Usage

After flashing:

Power the ESP8266

Connect to Wi-Fi AP:

SSID: XITIMER
Password: 12345678


Access control endpoints using a browser or REST client

Configuration
GPIO Pins
#define TRIG_PIN D1
#define ECHO_PIN D2
#define LED_PIN  LED_BUILTIN

ESP-NOW Peer (RX1.1)
uint8_t rx1_1MAC[] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11};


Replace with the actual MAC address of RX1.1.

Distance Threshold

Default: 20.0 cm

Adjustable via API

Guard & Debounce Logic

Minimum travel time calculation based on distance & speed

2-out-of-3 ultrasonic confirmations required

Debounce interval: 200 ms

Examples
Start Timing
curl http://192.168.4.1/start

Reset Timer
curl http://192.168.4.1/reset

Get Status
curl http://192.168.4.1/status

Update Detection Threshold
curl "http://192.168.4.1/threshold?value=25"

Example JSON Response
{
  "timerStarted": 1,
  "rx1Completed": 1,
  "distanceThreshold": 20,
  "statusMessage": "Split 1 Finished!",
  "split1Sec": 3.42
}

Running Tests

This project runs on embedded hardware.

Recommended validation:

Serial Monitor @ 115200 baud

Physical pass-through tests

ESP-NOW packet confirmation logs

Outdoor false-trigger testing

Contributing

Fork the repository

Create a branch:

git checkout -b feature/improvement-name


Commit changes:

git commit -m "Improve RX1 timing accuracy"


Push and open a Pull Request

Please maintain code style and timing accuracy.

License

This project is licensed under the MIT License.
See the LICENSE file for details.

Author

madhankumargspl291
Embedded Systems | ESP8266 | IoT Timing Systems
