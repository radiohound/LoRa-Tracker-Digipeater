#pragma once
// Flight phase from the GPS: ASCENT (also on the bench and at power-up), DESCENT, LANDED.
// Plain C with no Arduino calls, so it is tested on a PC (test/host/test_flight_phase.c).
//
// - DESCENT: the balloon has flown (climbed FP_FLOWN_M above the first fix) and then sinks faster than
//   FP_DESCENT_MS for FP_DESCENT_HOLD_MS in a row. One bad GPS fix cannot trigger it.
// - LANDED: it has flown, is below its highest point by FP_BELOW_MAX_M, and has stood still
//   (vertical and horizontal speed under the limits) for FP_LANDED_HOLD_MS.
//   Because it needs a flight first, a board on a desk stays in ASCENT.
// - Back to ASCENT: a sustained climb of FP_CLIMB_MS for FP_CLIMB_HOLD_MS (a new launch).

#include <stdint.h>

#define FP_FLOWN_M          500.0f     // climb needed before descent/landing can be detected
#define FP_DESCENT_MS       (-5.0f)    // vertical speed (m/s) that counts as descending
#define FP_DESCENT_HOLD_MS  5000u
#define FP_STILL_VERT_MS    0.7f       // |vertical speed| under this (m/s) ...
#define FP_STILL_KMH        3.0f       // ... and ground speed under this (km/h) is "standing still"
#define FP_LANDED_HOLD_MS   60000u
#define FP_BELOW_MAX_M      300.0f     // landed only this far below the highest point reached
#define FP_CLIMB_MS         2.0f       // climbing this fast (m/s) ...
#define FP_CLIMB_HOLD_MS    10000u     // ... for this long means a new ascent

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { FP_ASCENT = 0, FP_DESCENT = 1, FP_LANDED = 2 } fp_phase_t;

typedef struct {
    fp_phase_t phase;
    int        haveRef;
    float      refAlt, maxAlt;
    int        flown;
    uint32_t   descMs, stillMs, climbMs;
} fp_state_t;

void fp_init(fp_state_t *s);

/* Call once per new GPS fix. dtMs = time since the previous fix. Returns the (possibly new) phase. */
fp_phase_t fp_update(fp_state_t *s, float altM, float velUpMs, float groundKmh, uint32_t dtMs);

const char *fp_name(fp_phase_t p);

#ifdef __cplusplus
}
#endif
