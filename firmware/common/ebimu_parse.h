// EBIMU ASCII frame parsing, kept free of Arduino headers so it can be
// compiled and tested on a host machine.
//
// A frame is '*' then comma-separated floats then CRLF. The field count
// depends on how the module was configured and differs between board
// revisions, so the parser takes whatever floats it finds and reports how
// many -- the caller decides whether that count is the mode it expected.

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define EBIMU_MAX_FIELDS 8
#define EBIMU_MIN_FIELDS 3

typedef struct {
  float values[EBIMU_MAX_FIELDS];
  uint8_t count;
} EbimuFrame;

// `line` is the frame body, with the leading '*' and trailing CRLF removed.
// Rejects the frame outright if it finds anything that is not a number where
// one belongs: a line carrying evidence of corruption should be dropped rather
// than half-trusted, and at 50 Hz losing a frame costs nothing. Also rejects
// frames with fewer than three floats, which carry no usable attitude.
static inline bool ebimu_parse(const char *line, EbimuFrame *out) {
  out->count = 0;
  if (line == NULL) {
    return false;
  }

  const char *p = line;
  while (*p != '\0' && out->count < EBIMU_MAX_FIELDS) {
    char *end = NULL;
    float v = strtof(p, &end);
    if (end == p) {
      return false;  // garbage where a number belongs -- drop the frame
    }
    out->values[out->count++] = v;

    p = end;
    while (*p == ' ' || *p == '\t') {
      p++;
    }
    if (*p == ',') {
      p++;
    } else {
      break;
    }
  }

  return out->count >= EBIMU_MIN_FIELDS;
}
