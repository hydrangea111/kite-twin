// Airborne node: EBIMU-9DOFV5 attitude + GPS position, broadcast to the
// ground station over ESP-NOW.
//
// The IMU must be configured once with E2BOX's own utility (ASCII output,
// Euler angles, 50-100 Hz). Its command codes differ between board revisions,
// so nothing is sent to it from here -- this only reads.
//
// Attitude is what makes the power-generation data interpretable: the payload
// swings up to ~68 deg in gusts, which swings the panel's incident angle with
// it. Log attitude and generation on the same clock or the power numbers mean
// nothing.

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TinyGPSPlus.h>

#include "ebimu_parse.h"
#include "ina226.h"
#include "iv_sweep.h"
#include "telemetry.h"

static const int PIN_IMU_RX = 18;
static const int PIN_IMU_TX = 17;
static const int PIN_GPS_RX = 16;
static const int PIN_GPS_TX = 15;

static const uint32_t IMU_BAUD = 115200;
static const uint32_t GPS_BAUD = 9600;
static const uint32_t SEND_HZ = 50;
static const uint8_t ESPNOW_CHANNEL = 1;

static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// PV sweep. The capacitor is shorted through PIN_PV_DISCHARGE, then released;
// the panel charges it from short-circuit up towards open-circuit while we
// sample. 2200 uF gives roughly a 20 ms sweep at this panel's current, which
// is about 1.5% of a payload swing period -- quasi-static, so every sample in
// one curve shares an incident angle.
static const int PIN_PV_DISCHARGE = 5;
static const int PIN_I2C_SDA = 8;
static const int PIN_I2C_SCL = 9;
static const float PV_SHUNT_OHMS = 0.05f;
static const uint32_t PV_PERIOD_MS = 1000;
static const uint32_t PV_DISCHARGE_MS = 8;
static const uint32_t PV_SWEEP_TIMEOUT_MS = 60;
static const int PV_MAX_SAMPLES = 96;

static bool pv_ready = false;
static IvResult pv_last = {0};
static uint32_t pv_stamp = 0;

HardwareSerial ImuSerial(1);
HardwareSerial GpsSerial(2);
TinyGPSPlus gps;

static char imu_line[128];
static size_t imu_len = 0;
static bool imu_capturing = false;

static float attitude[3] = {0.0f, 0.0f, 0.0f};
static uint8_t attitude_fields = 0;
static uint32_t attitude_stamp = 0;
static uint32_t gps_stamp = 0;
static uint32_t seq = 0;

static void pump_imu() {
  while (ImuSerial.available()) {
    char c = (char)ImuSerial.read();

    if (c == '*') {
      imu_capturing = true;
      imu_len = 0;
      continue;
    }
    if (!imu_capturing) {
      continue;
    }
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      imu_line[imu_len] = '\0';
      EbimuFrame frame;
      if (ebimu_parse(imu_line, &frame)) {
        attitude[0] = frame.values[0];
        attitude[1] = frame.values[1];
        attitude[2] = frame.values[2];
        attitude_fields = frame.count;
        attitude_stamp = millis();
      }
      imu_capturing = false;
      imu_len = 0;
      continue;
    }

    if (imu_len + 1 < sizeof(imu_line)) {
      imu_line[imu_len++] = c;
    } else {
      imu_capturing = false;  // overlong line, resync on the next '*'
      imu_len = 0;
    }
  }
}

static void pump_gps() {
  while (GpsSerial.available()) {
    if (gps.encode((char)GpsSerial.read()) && gps.location.isValid()) {
      gps_stamp = millis();
    }
  }
}

// Blocks for the sweep duration (tens of ms). That is deliberate: splitting it
// across loop iterations would let telemetry sends interleave and stretch the
// sweep past the quasi-static window.
static void run_pv_sweep() {
  static float volts[PV_MAX_SAMPLES];
  static float amps[PV_MAX_SAMPLES];

  digitalWrite(PIN_PV_DISCHARGE, HIGH);
  delay(PV_DISCHARGE_MS);
  digitalWrite(PIN_PV_DISCHARGE, LOW);

  int n = 0;
  uint32_t started = millis();
  float previous = -1.0f;
  while (n < PV_MAX_SAMPLES && millis() - started < PV_SWEEP_TIMEOUT_MS) {
    float v, a;
    if (!ina226_sample(PV_SHUNT_OHMS, &v, &a)) {
      break;
    }
    volts[n] = v;
    amps[n] = a;
    n++;
    // Open circuit reached: the capacitor has stopped charging.
    if (previous >= 0.0f && v - previous < 1e-3f && a < 0.02f) {
      break;
    }
    previous = v;
  }

  pv_last = iv_analyse(volts, amps, n);
  pv_stamp = millis();
}

static uint16_t age_ms(uint32_t stamp, uint32_t now) {
  if (stamp == 0) {
    return 0xFFFF;
  }
  uint32_t age = now - stamp;
  return age > 0xFFFE ? 0xFFFE : (uint16_t)age;
}

void setup() {
  Serial.begin(115200);
  ImuSerial.begin(IMU_BAUD, SERIAL_8N1, PIN_IMU_RX, PIN_IMU_TX);
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init failed");
    while (true) {
      delay(1000);
    }
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("esp_now_add_peer failed");
  }

  pinMode(PIN_PV_DISCHARGE, OUTPUT);
  digitalWrite(PIN_PV_DISCHARGE, LOW);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  pv_ready = ina226_present() && ina226_begin();
  if (!pv_ready) {
    Serial.println("INA226 not found; flying without power measurement");
  }

  Serial.printf("payload up, packet %u bytes\n", (unsigned)sizeof(Telemetry));
}

void loop() {
  pump_imu();
  pump_gps();

  static uint32_t next_sweep = 0;
  if (pv_ready && (int32_t)(millis() - next_sweep) >= 0) {
    next_sweep = millis() + PV_PERIOD_MS;
    run_pv_sweep();
  }

  static uint32_t next_send = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next_send) < 0) {
    return;
  }
  next_send = now + 1000 / SEND_HZ;

  Telemetry pkt = {};
  pkt.version = TELEMETRY_VERSION;
  pkt.seq = seq++;
  pkt.uptime_ms = now;

  pkt.roll = attitude[0];
  pkt.pitch = attitude[1];
  pkt.yaw = attitude[2];
  pkt.imu_fields = attitude_fields;
  pkt.imu_age_ms = age_ms(attitude_stamp, now);

  pkt.fix = gps.location.isValid() ? 1 : 0;
  pkt.sats = gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0;
  pkt.lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  pkt.lon = gps.location.isValid() ? gps.location.lng() : 0.0;
  pkt.alt_gps = gps.altitude.isValid() ? (float)gps.altitude.meters() : 0.0f;
  pkt.gps_time = gps.time.isValid() ? gps.time.value() : 0;
  pkt.gps_age_ms = age_ms(gps_stamp, now);

  pkt.pv_sweep_ms = pv_stamp;
  pkt.pv_pmax = pv_last.pmax;
  pkt.pv_vmp = pv_last.vmp;
  pkt.pv_imp = pv_last.imp;
  pkt.pv_voc = pv_last.voc;
  pkt.pv_isc = pv_last.isc;
  pkt.pv_points = pv_last.count;
  pkt.pv_valid = pv_last.valid ? 1 : 0;

  esp_now_send(BROADCAST, (const uint8_t *)&pkt, sizeof(pkt));
}
