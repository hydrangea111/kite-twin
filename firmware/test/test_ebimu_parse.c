// Host-side tests for the EBIMU frame parser.
// Build and run:  python -m ziglang cc -I../common test_ebimu_parse.c -o t.exe && ./t.exe

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ebimu_parse.h"

static int failures = 0;
static int checks = 0;

static void check(bool ok, const char *what) {
  checks++;
  if (!ok) {
    failures++;
    printf("  FAIL: %s\n", what);
  }
}

static bool close_to(float a, float b) { return fabsf(a - b) < 1e-4f; }

int main(void) {
  EbimuFrame f;

  // Euler output, the mode we ask the module to be configured in.
  check(ebimu_parse("1.25,-2.50,180.00", &f), "euler accepted");
  check(f.count == 3, "euler field count");
  check(close_to(f.values[0], 1.25f) && close_to(f.values[1], -2.50f)
            && close_to(f.values[2], 180.0f), "euler values");

  // Module left in a richer mode: still usable, count tells the ground station.
  check(ebimu_parse("1,2,3,4,5,6,7", &f), "seven fields accepted");
  check(f.count == 7, "seven field count");
  check(close_to(f.values[0], 1.0f) && close_to(f.values[2], 3.0f),
        "first three of seven");

  // Quaternion-only mode would give four.
  check(ebimu_parse("0.707,0.0,0.0,0.707", &f), "quaternion accepted");
  check(f.count == 4, "quaternion field count");

  // Too few to be an attitude.
  check(!ebimu_parse("1.0,2.0", &f), "two fields rejected");
  check(!ebimu_parse("5.0", &f), "one field rejected");
  check(!ebimu_parse("", &f), "empty rejected");
  check(!ebimu_parse(NULL, &f), "null rejected");

  // Garbage, and garbage that starts out looking fine.
  check(!ebimu_parse("abc,def,ghi", &f), "non-numeric rejected");
  check(!ebimu_parse("1.0,xyz", &f), "truncated garbage rejected");

  // Three good floats followed by corruption. The prefix looks usable, but the
  // frame is evidently damaged, so it must be dropped rather than half-trusted.
  check(!ebimu_parse("1.0,2.0,3.0,abc", &f), "valid prefix then garbage rejected");
  check(!ebimu_parse("1.0,2.0,3.0,,4.0", &f), "empty field rejected");

  // Whitespace and a trailing comma are both things real serial output does.
  check(ebimu_parse(" 1.0 , 2.0 , 3.0", &f), "whitespace tolerated");
  check(f.count == 3, "whitespace field count");
  check(ebimu_parse("1.0,2.0,3.0,", &f), "trailing comma tolerated");
  check(f.count == 3, "trailing comma field count");

  // More fields than we can hold must not overflow the buffer.
  check(ebimu_parse("1,2,3,4,5,6,7,8,9,10,11,12", &f), "overlong accepted");
  check(f.count == EBIMU_MAX_FIELDS, "overlong capped at EBIMU_MAX_FIELDS");

  // Scientific notation: strtof takes it, so make sure it is not mangled.
  check(ebimu_parse("1e2,-2.5e-1,3", &f), "exponent form accepted");
  check(close_to(f.values[0], 100.0f) && close_to(f.values[1], -0.25f),
        "exponent values");

  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
