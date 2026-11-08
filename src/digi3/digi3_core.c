/* LACF 3.0 digipeater core. See digi3_core.h. */
#include "digi3_core.h"
#include "hb3.h"
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#ifdef D3_WITH_MODEL
#include "lacf3_destinations.h"
static char last_decoded[D3_MAX_TEXT + 1];
int d3_init_model(void) { return hb3_set_list(LACF3_DESTINATIONS, (int)(sizeof LACF3_DESTINATIONS / sizeof LACF3_DESTINATIONS[0])); }
const char *d3_last_decoded(void) { return last_decoded; }
#endif

static const char *const REASONS[D3_REASONS] = {
    "ok", "not an APRS frame", "LACF 2.0 frame (needs the tables)", "damaged or invalid frame", "our own packet",
    "path breaks the WIDEn-N rule", "already repeated by this station", "no hops left", "not a hop this digipeater serves", "too long"
};

const char *d3_reason(int r) { return (r >= 0 && r < D3_REASONS) ? REASONS[r] : "?"; }

uint32_t d3_fnv1a(uint32_t h, const uint8_t *p, size_t n)
{
    size_t i;
    for (i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

static int ieq(const char *a, size_t an, const char *b)
{
    size_t i;
    if (an != strlen(b)) return 0;
    for (i = 0; i < an; i++) if (toupper((unsigned char)a[i]) != toupper((unsigned char)b[i])) return 0;
    return 1;
}

/* WIDEn-N must have 1 <= N <= n <= 7; other hops pass. */
static int wide_ok(const char *h, size_t l)
{
    int n, N = 0; size_t i;
    if (l && h[l - 1] == '*') l--;
    if (l < 7 || toupper((unsigned char)h[0]) != 'W' || toupper((unsigned char)h[1]) != 'I' || toupper((unsigned char)h[2]) != 'D' ||
        toupper((unsigned char)h[3]) != 'E' || !isdigit((unsigned char)h[4]) || h[5] != '-') return 1;
    for (i = 6; i < l; i++) { if (!isdigit((unsigned char)h[i])) return 1; N = N * 10 + (h[i] - '0'); if (N > 99) return 0; }
    n = h[4] - '0';
    return N >= 1 && N <= n && n <= 7;
}

int d3_new_path(const char *path, size_t plen, const char *mycall, int mode, char *out, size_t cap)
{
    const char *hop[D3_MAX_HOPS + 1]; size_t hl[D3_MAX_HOPS + 1]; int nh = 0, used = -1, k, i;
    size_t s = 0, e, w = 0;
    char mystar[24];
    if (mode != D3_MODE_FILLIN && mode != D3_MODE_WIDE) return D3_NOT_SERVED;
    while (s < plen) {                                                   /* split on commas, empty hops skipped */
        e = s; while (e < plen && path[e] != ',') e++;
        if (e > s) { if (nh == D3_MAX_HOPS) return D3_TOO_LONG; hop[nh] = path + s; hl[nh] = e - s; nh++; }
        s = e + 1;
    }
    for (i = 0; i < nh; i++) if (!wide_ok(hop[i], hl[i])) return D3_PATH_RULE;
    for (i = 0; i < nh; i++) if (hl[i] && hop[i][hl[i] - 1] == '*') used = i;
    for (i = 0; i <= used; i++) { size_t l = hl[i]; if (l && hop[i][l - 1] == '*') l--; if (ieq(hop[i], l, mycall)) return D3_ALREADY; }
    k = used + 1;
    if (k >= nh) return D3_NO_HOPS;
    snprintf(mystar, sizeof mystar, "%s*", mycall);
    {
        const char *repl[2]; int nr;
        if (ieq(hop[k], hl[k], mycall) || ieq(hop[k], hl[k], "WIDE1-1")) { repl[0] = mystar; nr = 1; }
        else if (mode == D3_MODE_WIDE && ieq(hop[k], hl[k], "WIDE2-1")) { repl[0] = mystar; nr = 1; }
        else if (mode == D3_MODE_WIDE && ieq(hop[k], hl[k], "WIDE2-2")) { repl[0] = mystar; repl[1] = "WIDE2-1"; nr = 2; }
        else return D3_NOT_SERVED;
        if (nh - 1 + nr > D3_MAX_HOPS) return D3_TOO_LONG;
        for (i = 0; i < nh; i++) {
            const char *p; size_t l; int r, reps = (i == k) ? nr : 1;
            for (r = 0; r < reps; r++) {
                if (i == k) { p = repl[r]; l = strlen(p); } else { p = hop[i]; l = hl[i]; }
                if (w + l + 2 > cap) return D3_TOO_LONG;
                if (w) out[w++] = ',';
                memcpy(out + w, p, l); w += l;
            }
        }
    }
    out[w] = 0;
    return D3_OK;
}

/* "src>dest,path:" -> its parts. info may be empty. */
static int split_tnc2(const char *t, size_t n, size_t *gt, size_t *comma_or_colon, size_t *colon)
{
    size_t i; *gt = *colon = (size_t)-1; *comma_or_colon = (size_t)-1;
    for (i = 0; i < n; i++) if (t[i] == ':') { *colon = i; break; }
    if (*colon == (size_t)-1) return 0;
    for (i = 0; i < *colon; i++) if (t[i] == '>') { *gt = i; break; }
    if (*gt == (size_t)-1 || *gt == 0) return 0;
    for (i = *gt + 1; i < *colon; i++) if (t[i] == ',') break;
    *comma_or_colon = i;                                                 /* end of the destination: a comma before the path, or the colon */
    return *comma_or_colon > *gt + 1;
}

static int decide_text(const char *t, size_t n, const char *mycall, int mode, char *out, size_t cap, size_t *outlen, uint32_t *key, int as_plain)
{
    size_t gt, de, colon, w; char np[200]; int r;
    if (!split_tnc2(t, n, &gt, &de, &colon)) return D3_NOT_APRS;
    if (ieq(t, gt, mycall)) return D3_OWN;
    if (de == colon) return D3_NO_HOPS;                                  /* no path at all */
    r = d3_new_path(t + de + 1, colon - de - 1, mycall, mode, np, sizeof np);
    if (r != D3_OK) return r;
    w = de + 1 + strlen(np) + (n - colon);
    if (w + 1 > cap || (as_plain && w + 3 > D3_MAX_FRAME)) return D3_TOO_LONG;   /* +3: the 3C FF 01 prefix on the air (a LACF relay checks its own length) */
    memcpy(out, t, de); out[de] = ','; memcpy(out + de + 1, np, strlen(np)); memcpy(out + de + 1 + strlen(np), t + colon, n - colon); out[w] = 0;
    *outlen = w;
    if (key) { uint32_t h = d3_fnv1a(2166136261u, (const uint8_t *)t, de); *key = d3_fnv1a(h, (const uint8_t *)t + colon, n - colon); }
    return D3_OK;
}

int d3_decide(const uint8_t *fr, size_t n, const char *mycall, int mode, d3_job_t *job)
{
    if (n < 4 || fr[0] != 0x3C || fr[1] != 0xFF) return D3_NOT_APRS;
    if (fr[2] == 0x02) return D3_LACF2;
    if (fr[2] == 0x01) {                                                  /* plain LoRa APRS: TNC2 text */
        size_t outlen, i;
        for (i = 3; i < n; i++) if (fr[i] == 0 || fr[i] == '\r' || fr[i] == '\n') { if (fr[i] == 0) return D3_BAD_FRAME; n = i; break; }
        {
            int r = decide_text((const char *)fr + 3, n - 3, mycall, mode, job->text, sizeof job->text, &outlen, &job->key, 1);
            if (r != D3_OK) return r;
        }
        job->fmt = D3_FMT_PLAIN; job->text_len = (uint16_t)outlen; job->rx_len = 0; job->decoded = 0; job->convert = 0;
        return D3_OK;
    }
    if (fr[2] == 0x03) {
        char front[300], np[200]; hb3_info_t in; int len, r; size_t gt, de, colon, w; uint32_t h;
#ifdef D3_WITH_MODEL
        {   /* with the model: the full receiver check first (spec 12, Receiver), then the relay copies the content byte for byte (Relay) */
            char full[D3_MAX_TEXT + 1]; size_t outlen; int dl = hb3_decode(fr, n, full, sizeof full, &in);
            if (dl > 0) {
                r = decide_text(full, (size_t)dl, mycall, mode, job->text, sizeof job->text, &outlen, NULL, 0);
                if (r != D3_OK) return r;
                memcpy(last_decoded, full, (size_t)dl + 1);
                job->text_len = (uint16_t)outlen;
                memcpy(job->rx, fr, n); job->rx_len = (uint16_t)n; job->fmt = D3_FMT_LACF3; job->decoded = 1; job->convert = 0;
                h = d3_fnv1a(2166136261u, fr + 4, in.path_start - 4);
                job->key = d3_fnv1a(h, fr + in.front_len, in.content_start + in.content_len - in.front_len);
                /* the frame is now checked; building the relay needs no second PPM decode (the content is copied either way) */
                { uint8_t tmp[D3_MAX_FRAME]; static const uint8_t probe[4] = { 0, 0, 0, 0 };
                  if (!hb3_relay_to_passthrough(fr, n, job->text, job->text_len, mycall, probe, tmp, sizeof tmp)) return D3_TOO_LONG; }
                return D3_OK;
            }
            /* not decoded: relay without the model only when what is missing is the model or the destination code; anything else is damage */
            if (dl != HB3_ERR_VERSION && dl != HB3_ERR_FRONT) return D3_BAD_FRAME;
        }
#endif
        /* without the model: structure and CRC only */
        len = hb3_decode_front(fr, n, front, sizeof front, &in);
        if (len <= 0) return D3_BAD_FRAME;
        if (!split_tnc2(front, (size_t)len, &gt, &de, &colon)) return D3_BAD_FRAME;
        if (ieq(front, gt, mycall)) return D3_OWN;
        if (de == colon) return D3_NO_HOPS;
        r = d3_new_path(front + de + 1, colon - de - 1, mycall, mode, np, sizeof np);
        if (r != D3_OK) return r;
        w = de + 1 + strlen(np) + 1;
        if (w + 1 > sizeof job->text) return D3_TOO_LONG;
        memcpy(job->text, front, de); job->text[de] = ','; memcpy(job->text + de + 1, np, strlen(np)); job->text[w - 1] = ':'; job->text[w] = 0;
        job->text_len = (uint16_t)w;
        memcpy(job->rx, fr, n); job->rx_len = (uint16_t)n; job->fmt = D3_FMT_LACF3; job->decoded = 0; job->convert = 0;
        /* duplicate key: the received bytes of the source, the destination, the position block and the content field (spec 12, Relay step 8), not the path or RXT */
        h = d3_fnv1a(2166136261u, fr + 4, in.path_start - 4);
        job->key = d3_fnv1a(h, fr + in.front_len, in.content_start + in.content_len - in.front_len);
        /* check now that the relay can be built (a frame it would refuse is not queued) */
        { uint8_t tmp[D3_MAX_FRAME]; static const uint8_t probe[4] = { 0, 0, 0, 0 };
          if (!hb3_relay_to_passthrough(fr, n, job->text, job->text_len, mycall, probe, tmp, sizeof tmp)) return D3_TOO_LONG; }
        return D3_OK;
    }
    return D3_NOT_APRS;
}

size_t d3_build(const d3_job_t *job, const char *mycall, const uint8_t *tuple, uint8_t *out, size_t cap)
{
    if (job->fmt == D3_FMT_PLAIN) {
#ifdef D3_WITH_MODEL
        if (job->convert) {                                               /* re-encoded as LACF 3.0 (LACF or plain content, whichever is shorter), no RXT block */
            hb3_info_t in; size_t n = hb3_encode(job->text, job->text_len, NULL, out, cap, &in);
            if (n) return n;                                              /* 0: it cannot be encoded: sent as plain text below */
        }
#endif
        if (job->text_len + 3u > cap || job->text_len + 3u > D3_MAX_FRAME) return 0;
        out[0] = 0x3C; out[1] = 0xFF; out[2] = 0x01; memcpy(out + 3, job->text, job->text_len);
        return job->text_len + 3u;
    }
    if (job->fmt == D3_FMT_LACF3) {
        static const uint8_t none[4] = { 93, 93, 93, 93 };               /* 93 = not measured (3.0.0-dev.1) */
        /* a decoded frame was checked with the model in d3_decide; hb3_relay_to would decode it again only to build the same bytes */
        return hb3_relay_to_passthrough(job->rx, job->rx_len, job->text, job->text_len, mycall, tuple ? tuple : none, out, cap);
    }
    return 0;
}

/* ---- RXT scales ---- */
static uint8_t clamp89(float v)
{
    long r = lroundf(v);
    return (uint8_t)(r < 0 ? 0 : r > 89 ? 89 : r);
}

uint8_t d3_rxt_rssi(float dbm) { return clamp89(dbm + 130.0f); }

uint8_t d3_rxt_snr(float db)
{
    if (db < -24.0f) db = -24.0f;
    if (db > 20.0f) db = 20.0f;
    return clamp89((db + 24.0f) / 0.5f);
}

uint8_t d3_rxt_freq_offset(float hz)
{
    float n = hz < -2500.0f ? -1.0f : hz > 2500.0f ? 1.0f : hz / 2500.0f;
    float s = sqrtf(fabsf(n));
    return clamp89((n < 0 ? -s : s) * 45.0f + 45.0f);
}

uint8_t d3_rxt_time_to_hop(float ms, int sf, float bw_khz)
{
    float unit = (float)(1L << sf) / bw_khz * 10.0f;                    /* ten symbol times, in ms */
    float x = ms / unit + 1.0f;
    if (x < 1.0f) x = 1.0f;
    return clamp89(logf(x) / logf(1.08f));
}

void d3_rxt_tuple(float rssi_dbm, float snr_db, float fo_hz, float tth_ms, int sf, float bw_khz, uint8_t out[4])
{
    out[0] = d3_rxt_rssi(rssi_dbm);
    out[1] = d3_rxt_snr(snr_db);
    out[2] = d3_rxt_freq_offset(fo_hz);
    out[3] = d3_rxt_time_to_hop(tth_ms, sf, bw_khz);
}
