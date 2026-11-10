// Host-side tests for the IV sweep reduction.
//
// Build and run (Windows Application Control blocks new executables under the
// project tree, so build into the temp scratchpad):
//   python -m ziglang cc -std=c11 -I../common test_iv_sweep.c -o %TEMP%\ivtest.exe
//   %TEMP%\ivtest.exe

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "iv_sweep.h"

static int failures = 0;
static int checks = 0;

static void check(bool ok, const char *what) {
  checks++;
  if (!ok) {
    failures++;
    printf("  FAIL: %s\n", what);
  }
}

static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

// Single-diode panel, same model the design analysis used.
static const float ISC = 0.75f;
static const float VOC = 7.9f;
static const float SCALE = 10.0f * 1.5f * 0.02585f;   // n * Vt * Ns

static float panel_current(float v) {
  float i0 = ISC / (expf(VOC / SCALE) - 1.0f);
  float i = ISC - i0 * (expf(v / SCALE) - 1.0f);
  return i < 0.0f ? 0.0f : i;
}

// Sweep from short-circuit to just past open-circuit, as the capacitor does.
static int make_sweep(float *v, float *a, int n, float noise) {
  for (int k = 0; k < n; k++) {
    float frac = (float)k / (float)(n - 1);
    v[k] = frac * VOC * 1.02f;
    a[k] = panel_current(v[k]);
    if (noise > 0.0f) {
      float jitter = ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f * noise;
      a[k] += jitter * ISC;
      v[k] += jitter * VOC * 0.2f;
      if (a[k] < 0.0f) a[k] = 0.0f;
    }
  }
  return n;
}

int main(void) {
  float v[128], a[128];

  // Clean sweep: the extrapolations should land on the panel's own Isc and Voc.
  int n = make_sweep(v, a, 60, 0.0f);
  IvResult r = iv_analyse(v, a, n);
  check(r.valid, "clean sweep valid");
  check(near(r.isc, ISC, 0.02f), "clean Isc");
  check(near(r.voc, VOC, 0.15f), "clean Voc");
  check(r.pmax > 0.0f && r.pmax < ISC * VOC, "pmax below the Isc*Voc box");
  check(r.fill > 0.6f && r.fill < 0.95f, "fill factor plausible");
  check(r.vmp > 0.0f && r.vmp < r.voc, "Vmp inside the curve");
  check(r.imp > 0.0f && r.imp <= r.isc, "Imp inside the curve");
  check(r.count == 60, "sample count reported");

  // Pmax must beat every individual sample, since it is the best sample.
  float best = 0.0f;
  for (int k = 0; k < n; k++) {
    if (v[k] * a[k] > best) best = v[k] * a[k];
  }
  check(near(r.pmax, best, 1e-6f), "pmax equals the best sample");

  // Noisy sweep: still close, and still self-consistent.
  srand(7);
  n = make_sweep(v, a, 60, 0.03f);
  r = iv_analyse(v, a, n);
  check(r.valid, "noisy sweep valid");
  check(near(r.isc, ISC, 0.08f), "noisy Isc within tolerance");
  check(near(r.voc, VOC, 0.5f), "noisy Voc within tolerance");
  check(r.isc >= r.imp, "Isc not below Imp");
  check(r.voc >= r.vmp, "Voc not below Vmp");

  // A panel in the dark produces nothing; that must not read as a measurement.
  for (int k = 0; k < 40; k++) {
    v[k] = 0.0f;
    a[k] = 0.0f;
  }
  r = iv_analyse(v, a, 40);
  check(!r.valid, "dark panel rejected");
  check(r.pmax == 0.0f, "dark panel reports no power");

  // Too few samples to fit anything.
  n = make_sweep(v, a, 5, 0.0f);
  r = iv_analyse(v, a, n);
  check(!r.valid, "short sweep rejected");

  // Null inputs must not crash or claim validity.
  r = iv_analyse(NULL, a, 60);
  check(!r.valid, "null voltage rejected");
  r = iv_analyse(v, NULL, 60);
  check(!r.valid, "null current rejected");

  // A stuck sensor gives one voltage for every sample; the fit has no slope.
  n = make_sweep(v, a, 60, 0.0f);
  for (int k = 0; k < n; k++) v[k] = 3.0f;
  r = iv_analyse(v, a, n);
  check(r.voc >= r.vmp, "stuck voltage still self-consistent");

  // Fill factor above one would mean Pmax outside the Isc*Voc box.
  n = make_sweep(v, a, 60, 0.0f);
  r = iv_analyse(v, a, n);
  check(r.fill <= 1.0f, "fill factor never exceeds one");

  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
