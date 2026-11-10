// Ground node: listens for payload telemetry and prints CSV on USB serial.
// Capture it with any terminal that logs to file; the PC is the recorder, so
// the payload needs no SD card.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "telemetry.h"

static const uint8_t ESPNOW_CHANNEL = 1;

// The ground station's own measurements. Without these a log cannot identify
// anything: the aerodynamic balance is written in terms of tension, line
// length, elevation and wind, none of which the payload can see. Logging only
// what arrives over the radio produces a file that looks complete and fits
// nothing.
static const int PIN_TENSION_SENSE = 34;   // BTS7960 IS pin
static const int PIN_ELEVATION_SENSE = 35; // ground angle sensor, if fitted
static const int PIN_ANEMOMETER = 32;      // pulse output

// Replace with the bench calibration; the identification is more sensitive to
// the anemometer's gain than to anything else measured here.
static const float TENSION_PER_COUNT = 0.012f;   // N per ADC count
static const float ELEVATION_PER_COUNT = 0.030f; // deg per ADC count
static const float ELEVATION_OFFSET = -15.0f;    // deg
static const float WIND_PER_HZ = 0.34f;          // m/s per pulse per second

static volatile uint32_t anemometer_pulses = 0;
static float wind_ms = 0.0f;
static float line_length_m = 0.0f;   // fed by the winch controller

static void IRAM_ATTR on_anemometer_pulse() { anemometer_pulses++; }

static float read_tension() {
  return analogRead(PIN_TENSION_SENSE) * TENSION_PER_COUNT;
}

static float read_elevation() {
  return analogRead(PIN_ELEVATION_SENSE) * ELEVATION_PER_COUNT
         + ELEVATION_OFFSET;
}

static volatile uint32_t received = 0;
static volatile uint32_t last_seq = 0;
static volatile uint32_t dropped = 0;

// The receive callback signature changed in ESP32 Arduino core 3.x.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
static void on_recv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (len != (int)sizeof(Telemetry)) {
    Serial.printf("# bad length %d, expected %u\n", len, (unsigned)sizeof(Telemetry));
    return;
  }

  Telemetry pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.version != TELEMETRY_VERSION) {
    Serial.printf("# version mismatch %u\n", pkt.version);
    return;
  }

  if (received > 0 && pkt.seq > last_seq + 1) {
    dropped += pkt.seq - last_seq - 1;
  }
  last_seq = pkt.seq;
  received++;

  // rx_ms is the ground clock; uptime_ms is the payload clock. Keep both --
  // the offset between them is what lets logs from the two nodes be aligned.
  Serial.printf(
      "%lu,%.3f,%.3f,%.2f,%.2f,"
      "%lu,%lu,%lu,%.2f,%.2f,%.2f,%u,%u,%.7f,%.7f,%.1f,%lu,%u,%u,%lu,"
      "%lu,%.4f,%.3f,%.4f,%.3f,%.4f,%u,%u\n",
      (unsigned long)millis(), line_length_m, read_tension(), read_elevation(),
      wind_ms,
      (unsigned long)millis(), (unsigned long)pkt.seq,
      (unsigned long)pkt.uptime_ms, pkt.roll, pkt.pitch, pkt.yaw,
      pkt.fix, pkt.sats, pkt.lat, pkt.lon, pkt.alt_gps,
      (unsigned long)pkt.gps_time, pkt.imu_age_ms, pkt.imu_fields,
      (unsigned long)dropped,
      (unsigned long)pkt.pv_sweep_ms, pkt.pv_pmax, pkt.pv_vmp, pkt.pv_imp,
      pkt.pv_voc, pkt.pv_isc, pkt.pv_points, pkt.pv_valid);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("# esp_now_init failed");
    while (true) {
      delay(1000);
    }
  }
  esp_now_register_recv_cb(on_recv);

  pinMode(PIN_ANEMOMETER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ANEMOMETER), on_anemometer_pulse,
                  FALLING);

  Serial.println("# ground station up");
  Serial.println("t_ms,line_length_m,tension_n,elevation_deg,wind_ms,"
                 "rx_ms,seq,uptime_ms,roll,pitch,yaw,fix,sats,lat,lon,"
                 "alt_gps,gps_time,imu_age_ms,imu_fields,dropped,"
                 "pv_sweep_ms,pv_pmax,pv_vmp,pv_imp,pv_voc,pv_isc,"
                 "pv_points,pv_valid");
}

void loop() {
  // Wind is a pulse rate, so it needs its own fixed window rather than being
  // sampled whenever a packet happens to arrive.
  static uint32_t next_wind = 0;
  if ((int32_t)(millis() - next_wind) >= 0) {
    next_wind = millis() + 1000;
    noInterrupts();
    uint32_t pulses = anemometer_pulses;
    anemometer_pulses = 0;
    interrupts();
    wind_ms = pulses * WIND_PER_HZ;
  }

  static uint32_t next_report = 0;
  if (millis() > next_report) {
    next_report = millis() + 5000;
    Serial.printf("# rx=%lu dropped=%lu\n", (unsigned long)received,
                  (unsigned long)dropped);
  }
  delay(50);
}
