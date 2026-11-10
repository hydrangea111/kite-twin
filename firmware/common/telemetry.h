// Shared packet definition. Both nodes must build against the same copy --
// ESP-NOW does no schema checking, so a mismatch shows up as silent garbage.

#pragma once
#include <stdint.h>

static const uint8_t TELEMETRY_VERSION = 2;   // 2 added the IV sweep fields

struct __attribute__((packed)) Telemetry {
  uint8_t  version;
  uint8_t  fix;          // GPS fix: 0 none, 1 valid
  uint8_t  sats;
  uint8_t  imu_fields;   // how many floats the IMU line carried, for diagnostics

  uint32_t seq;
  uint32_t uptime_ms;

  float    roll;         // deg
  float    pitch;        // deg
  float    yaw;          // deg

  double   lat;
  double   lon;
  float    alt_gps;      // m above MSL
  uint32_t gps_time;     // HHMMSSCC, 0 when no fix

  uint16_t imu_age_ms;   // staleness of the attitude in this packet
  uint16_t gps_age_ms;

  // Last completed IV sweep. Attitude travels in the same packet on purpose:
  // incident angle moves with the payload's swing, so a power reading without
  // the attitude it was taken at cannot be normalised afterwards.
  uint32_t pv_sweep_ms;  // uptime when the sweep ran, 0 if none yet
  float    pv_pmax;      // W
  float    pv_vmp;       // V
  float    pv_imp;       // A
  float    pv_voc;       // V
  float    pv_isc;       // A
  uint16_t pv_points;
  uint8_t  pv_valid;
  uint8_t  pv_reserved;
};

static_assert(sizeof(Telemetry) <= 250, "ESP-NOW payload limit is 250 bytes");
