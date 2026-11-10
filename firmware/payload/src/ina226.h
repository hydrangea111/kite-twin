// Minimal INA226 access for fast IV sweeps.
//
// A library is avoided deliberately: the point of this sensor here is the
// 140 us conversion setting and tight control of when each register is read,
// which the convenience wrappers hide. Only raw shunt and bus voltage are
// used -- the chip's calibrated current register adds a register write and no
// speed, and the shunt value is known.
//
// UNTESTED against hardware. First bring-up should check `ina226_present()`
// and confirm a known load reads correctly before trusting any sweep.

#pragma once

#include <Arduino.h>
#include <Wire.h>

static const uint8_t INA226_ADDR = 0x40;

static const uint8_t INA226_REG_CONFIG = 0x00;
static const uint8_t INA226_REG_SHUNT = 0x01;
static const uint8_t INA226_REG_BUS = 0x02;
static const uint8_t INA226_REG_MASK = 0x06;
static const uint8_t INA226_REG_DIE = 0xFF;
static const uint16_t INA226_DIE_ID = 0x2260;

// Bit 14 reads back set; AVG=1 sample, both conversions 140 us, continuous.
static const uint16_t INA226_CONFIG_FAST = 0x4007;

static const float INA226_SHUNT_LSB = 2.5e-6f;   // V
static const float INA226_BUS_LSB = 1.25e-3f;    // V

static bool ina226_write(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool ina226_read(uint8_t reg, uint16_t *value) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((int)INA226_ADDR, 2) != 2) {
    return false;
  }
  *value = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

static bool ina226_present() {
  uint16_t die = 0;
  return ina226_read(INA226_REG_DIE, &die) && die == INA226_DIE_ID;
}

static bool ina226_begin() {
  return ina226_write(INA226_REG_CONFIG, INA226_CONFIG_FAST);
}

// Shunt voltage is signed; current flows one way here but a reversed sense
// resistor would otherwise read as a huge positive value.
static bool ina226_sample(float shunt_ohms, float *volts, float *amps) {
  uint16_t raw_bus = 0, raw_shunt = 0;
  if (!ina226_read(INA226_REG_BUS, &raw_bus)) {
    return false;
  }
  if (!ina226_read(INA226_REG_SHUNT, &raw_shunt)) {
    return false;
  }
  *volts = raw_bus * INA226_BUS_LSB;
  *amps = ((int16_t)raw_shunt) * INA226_SHUNT_LSB / shunt_ohms;
  return true;
}
