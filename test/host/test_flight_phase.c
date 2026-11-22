/* Simulated flights against the phase detector. */
#include <stdio.h>
#include <stdlib.h>
#include "flight_phase.h"

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

/* run `secs` seconds of fixes (1 Hz) at constant vertical speed; returns the phase at the end */
static float alt;
static fp_phase_t run(fp_state_t *s, int secs, float vup, float kmh)
{
    fp_phase_t p = s->phase;
    for (int i = 0; i < secs; i++) { alt += vup; p = fp_update(s, alt, vup, kmh, 1000); }
    return p;
}

int main(void)
{
    fp_state_t s;

    /* full flight: climb 30 km, burst, fall, land, sit */
    fp_init(&s); alt = 100;
    CHECK(run(&s, 600, 0, 0) == FP_ASCENT);            /* on the ground before launch */
    CHECK(run(&s, 6000, 5, 30) == FP_ASCENT);          /* ascent */
    CHECK(s.flown);
    CHECK(run(&s, 4, -40, 20) == FP_ASCENT);           /* 4 s of fall: not yet */
    CHECK(run(&s, 2, -40, 20) == FP_DESCENT);          /* 6 s: descent */
    CHECK(run(&s, 600, -20, 20) == FP_DESCENT);
    CHECK(run(&s, 400, -5, 10) == FP_DESCENT);
    alt = 120;                                         /* on the ground */
    CHECK(run(&s, 50, 0, 1) == FP_DESCENT);            /* 50 s still: not yet */
    CHECK(run(&s, 15, 0, 1) == FP_LANDED);
    CHECK(run(&s, 3600, 0.1f, 1) == FP_LANDED);        /* stays landed */
    CHECK(run(&s, 300, 0, 40) == FP_LANDED);           /* carried in a car: stays landed */

    /* a desk test never leaves ascent */
    fp_init(&s); alt = 100;
    CHECK(run(&s, 3600, 0, 0) == FP_ASCENT);
    CHECK(run(&s, 20, 0.3f, 0) == FP_ASCENT);

    /* one wild fix does not trigger descent */
    fp_init(&s); alt = 100; run(&s, 600, 5, 10);
    fp_update(&s, alt, -60, 0, 1000);
    CHECK(run(&s, 20, 5, 10) == FP_ASCENT);

    /* a long hover at the top is not "landed" */
    CHECK(run(&s, 600, 0, 0) == FP_ASCENT);

    /* landed, then a new launch */
    fp_init(&s); alt = 0; run(&s, 600, 5, 10); run(&s, 10, -30, 10); run(&s, 100, -10, 10); alt = 10; run(&s, 80, 0, 0);
    CHECK(s.phase == FP_LANDED);
    CHECK(run(&s, 9, 4, 0) == FP_LANDED);
    CHECK(run(&s, 3, 4, 0) == FP_ASCENT);

    /* the first fix is bad (altitude 0 for 5 s, then the real altitude of 1500 m) */
    fp_init(&s); alt = 0; run(&s, 5, 0, 0); alt = 1500; CHECK(run(&s, 600, 0, 0) == FP_ASCENT);
    /* known limit: a bad first fix makes the later real altitude look like a 1500 m climb, so the detector is armed early */

    printf(fails ? "%d FAILED\n" : "flight phase: all passed\n", fails);
    return fails != 0;
}
