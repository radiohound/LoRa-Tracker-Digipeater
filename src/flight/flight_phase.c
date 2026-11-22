#include "flight_phase.h"
#include <string.h>
#include <math.h>

void fp_init(fp_state_t *s) { memset(s, 0, sizeof *s); s->phase = FP_ASCENT; }

const char *fp_name(fp_phase_t p)
{
    return p == FP_DESCENT ? "descent" : p == FP_LANDED ? "landed" : "ascent";
}

static uint32_t acc(uint32_t v, int cond, uint32_t dt)
{
    if (!cond) return 0;
    return (v > 0xFFFFFFFFu - dt) ? v : v + dt;
}

fp_phase_t fp_update(fp_state_t *s, float altM, float velUpMs, float groundKmh, uint32_t dtMs)
{
    if (!s->haveRef) { s->haveRef = 1; s->refAlt = s->maxAlt = altM; }
    if (altM > s->maxAlt) s->maxAlt = altM;
    if (!s->flown && s->maxAlt - s->refAlt >= FP_FLOWN_M) s->flown = 1;

    s->climbMs = acc(s->climbMs, velUpMs > FP_CLIMB_MS, dtMs);
    s->descMs  = acc(s->descMs,  velUpMs < FP_DESCENT_MS, dtMs);
    s->stillMs = acc(s->stillMs, fabsf(velUpMs) < FP_STILL_VERT_MS && groundKmh < FP_STILL_KMH
                                 && altM < s->maxAlt - FP_BELOW_MAX_M, dtMs);

    if (s->phase != FP_ASCENT && s->climbMs >= FP_CLIMB_HOLD_MS) {   // launched again
        s->phase = FP_ASCENT; s->flown = 0; s->refAlt = s->maxAlt = altM;
        s->descMs = s->stillMs = 0;
        return s->phase;
    }
    if (s->flown) {
        if (s->phase != FP_LANDED && s->stillMs >= FP_LANDED_HOLD_MS) s->phase = FP_LANDED;
        else if (s->phase == FP_ASCENT && s->descMs >= FP_DESCENT_HOLD_MS) s->phase = FP_DESCENT;
    }
    return s->phase;
}
