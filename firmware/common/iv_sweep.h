// Reducing a capacitor-charge IV sweep to the numbers worth transmitting.
//
// Kept free of Arduino and I2C so it can be compiled and tested on a host.
// The sweep itself is: short the capacitor, release it, and sample voltage and
// current while the panel charges it from 0 up to Voc. Samples therefore run
// from short-circuit towards open-circuit.
//
// Isc and Voc are extrapolated rather than read off the ends: the first sample
// is never exactly at V=0 and the last never exactly at I=0.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define IV_EDGE_POINTS 5        // samples averaged into each end fit
#define IV_MIN_POINTS 8

typedef struct {
  float pmax;     // W
  float vmp;      // V at the maximum power point
  float imp;      // A at the maximum power point
  float voc;      // V, extrapolated to zero current
  float isc;      // A, extrapolated to zero voltage
  float fill;     // pmax / (voc * isc)
  uint16_t count; // samples used
  bool valid;
} IvResult;

// Least-squares slope and intercept of y against x over n points.
static inline bool iv_fit(const float *x, const float *y, int n,
                          float *slope, float *intercept) {
  if (n < 2) {
    return false;
  }
  float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
  for (int k = 0; k < n; k++) {
    sx += x[k];
    sy += y[k];
    sxx += x[k] * x[k];
    sxy += x[k] * y[k];
  }
  float denom = n * sxx - sx * sx;
  if (denom > -1e-12f && denom < 1e-12f) {
    return false;   // all samples at one x; no slope to find
  }
  *slope = (n * sxy - sx * sy) / denom;
  *intercept = (sy - *slope * sx) / n;
  return true;
}

// `volts` and `amps` run from short-circuit towards open-circuit.
static inline IvResult iv_analyse(const float *volts, const float *amps,
                                  int n) {
  IvResult out = {0};
  out.count = (uint16_t)(n < 0 ? 0 : n);
  if (volts == 0 || amps == 0 || n < IV_MIN_POINTS) {
    return out;
  }

  int best = 0;
  float best_power = -1.0f;
  for (int k = 0; k < n; k++) {
    float p = volts[k] * amps[k];
    if (p > best_power) {
      best_power = p;
      best = k;
    }
  }
  if (best_power <= 0.0f) {
    return out;    // dark panel, or the sweep never ran
  }

  int edge = n < IV_EDGE_POINTS * 2 ? n / 2 : IV_EDGE_POINTS;

  // Isc: current extrapolated back to V = 0, from the low-voltage end.
  float slope, intercept;
  out.isc = iv_fit(volts, amps, edge, &slope, &intercept) ? intercept : amps[0];

  // Voc: voltage extrapolated to I = 0, fitting V against I at the far end.
  const float *v_tail = volts + (n - edge);
  const float *i_tail = amps + (n - edge);
  out.voc = iv_fit(i_tail, v_tail, edge, &slope, &intercept) ? intercept
                                                             : volts[n - 1];

  out.pmax = best_power;
  out.vmp = volts[best];
  out.imp = amps[best];

  // A fit through noise can land the intercept below the data it came from.
  if (out.isc < out.imp) {
    out.isc = out.imp;
  }
  if (out.voc < out.vmp) {
    out.voc = out.vmp;
  }

  out.fill = (out.voc > 0.0f && out.isc > 0.0f)
                 ? out.pmax / (out.voc * out.isc)
                 : 0.0f;
  out.valid = out.fill > 0.1f && out.fill <= 1.0f;
  return out;
}
