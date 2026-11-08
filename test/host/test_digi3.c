/* Host test of the LACF 3.0 digipeater core against the reference vectors of lora-aprs-ideas (proposal 3.0.0-dev.1).
 *   make -C test/host VECTORS=/path/to/lora-aprs-ideas/test
 * The build uses the same hb3.c and the same model-less stand-ins as the firmware, so it checks the code that goes on the board. */
#include "digi3_core.h"
#include "hb3.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static size_t unhex(const char *s, uint8_t *out, size_t cap)
{
    size_t n = 0;
    while (s[0] && s[1] && n < cap) { unsigned v; if (sscanf(s, "%2x", &v) != 1) break; out[n++] = (uint8_t)v; s += 2; }
    return n;
}

static void tohex(const uint8_t *p, size_t n, char *out) { size_t i; for (i = 0; i < n; i++) sprintf(out + 2 * i, "%02x", p[i]); out[2 * n] = 0; }

/* One vector line: received frame, expected relayed frame or "-". Relay N0RLY-(k+1) with tuple 50+k 20+k 45+k 10+k, k = tuples already in the frame. */
static void run_file(const char *dir, const char *name, int *n_ok, int *n_lines)
{
    char path[512], line[4096]; FILE *f;
    snprintf(path, sizeof path, "%s/%s", dir, name);
    f = fopen(path, "r");
    if (!f) { printf("FAIL cannot open %s\n", path); fails++; return; }
    while (fgets(line, sizeof line, f)) {
        char *a, *b, *c; uint8_t rx[300], want[300], got[300]; size_t rn, wn, gn = 0; d3_job_t job; hb3_info_t in; char front[300], call[16], gh[700], wh[700];
        int k = 0, r; uint8_t tup[4];
        if (line[0] == '#' || line[0] == '\n') continue;
        a = strtok(line, " \n"); b = strtok(NULL, " \n"); c = strtok(NULL, " \n");
        if (!a || !b) continue;
        (*n_lines)++;
        rn = unhex(a, rx, sizeof rx);
        if (hb3_decode_front(rx, rn, front, sizeof front, &in) > 0 && in.has_rxt) k = in.rxt.n;
        snprintf(call, sizeof call, "N0RLY-%d", k + 1);
        tup[0] = (uint8_t)(50 + k); tup[1] = (uint8_t)(20 + k); tup[2] = (uint8_t)(45 + k); tup[3] = (uint8_t)(10 + k);
        r = d3_decide(rx, rn, call, D3_MODE_WIDE, &job);
        if (r == D3_OK) gn = d3_build(&job, call, tup, got, sizeof got);
        if (!strcmp(b, "-")) {
            CHECK(gn == 0, "%s: relayed a frame that must not be relayed: %s", name, a);
            if (gn == 0) (*n_ok)++;
        } else {
            wn = unhex(b, want, sizeof want);
            tohex(got, gn, gh); tohex(want, wn, wh);
            CHECK(gn == wn && !memcmp(got, want, wn), "%s: %s\n  want %s\n  got  %s (%s)", name, a, wh, gn ? gh : "nothing", d3_reason(r));
#ifdef D3_WITH_MODEL
            /* with the model, a frame the decoding relay accepts must have gone through the full decode (not the fallback without the model) */
            { int decoding_relay_accepts = strstr(name, "passthrough") ? (c && strcmp(c, "-")) : 1;
              CHECK(!decoding_relay_accepts || job.decoded, "%s: %s was relayed without decoding it", name, a);
              if (gn == wn && !memcmp(got, want, wn) && (!decoding_relay_accepts || job.decoded)) (*n_ok)++; }
#else
            if (gn == wn && !memcmp(got, want, wn)) (*n_ok)++;
#endif
        }
    }
    fclose(f);
}

static void plain_cases(void)
{
    struct { const char *in, *want; int mode; } T[] = {
        { "K6ATV-3>APZLPI,WIDE1-1:!3700.16N/12134.97W>204/013", "K6ATV-3>APZLPI,K6ATV-11*:!3700.16N/12134.97W>204/013", D3_MODE_FILLIN },
        { "K6ATV-3>APZLPI,WIDE1-1,WIDE2-1:>hi", "K6ATV-3>APZLPI,K6ATV-11*,WIDE2-1:>hi", D3_MODE_WIDE },
        { "K6ATV-3>APZLPI,K6ATV-10*,WIDE2-1:>hi", "K6ATV-3>APZLPI,K6ATV-10*,K6ATV-11*:>hi", D3_MODE_WIDE },
        { "K6ATV-3>APZLPI,K6ATV-10*,WIDE2-1:>hi", NULL, D3_MODE_FILLIN },                     /* fill-in does not serve WIDE2 */
        { "K6ATV-3>APZLPI,WIDE2-2:>hi", "K6ATV-3>APZLPI,K6ATV-11*,WIDE2-1:>hi", D3_MODE_WIDE },
        { "K6ATV-3>APZLPI,K6ATV-11:>hi", "K6ATV-3>APZLPI,K6ATV-11*:>hi", D3_MODE_FILLIN },          /* addressed to this station */
        { "K6ATV-3>APZLPI,K6ATV-11*,WIDE2-1:>hi", NULL, D3_MODE_WIDE },                         /* already repeated here */
        { "K6ATV-11>APZLD3,WIDE1-1:>me", NULL, D3_MODE_WIDE },                                   /* our own packet */
        { "K6ATV-3>APZLPI,WIDE7-8:>hi", NULL, D3_MODE_WIDE },                                    /* breaks WIDEn-N */
        { "K6ATV-3>APZLPI:>no path", NULL, D3_MODE_WIDE },
        { "K6ATV-3>APZLPI,WIDE3-3:>hi", NULL, D3_MODE_WIDE },                                    /* not served */
        { "garbage without colon", NULL, D3_MODE_WIDE },
    };
    size_t i;
    for (i = 0; i < sizeof T / sizeof T[0]; i++) {
        uint8_t fr[300], out[300]; size_t n = strlen(T[i].in), on = 0; d3_job_t job; int r;
        fr[0] = 0x3C; fr[1] = 0xFF; fr[2] = 0x01; memcpy(fr + 3, T[i].in, n);
        r = d3_decide(fr, n + 3, "K6ATV-11", T[i].mode, &job);
        if (r == D3_OK) on = d3_build(&job, "K6ATV-11", NULL, out, sizeof out);
        if (!T[i].want) CHECK(on == 0, "plain: should not repeat %s", T[i].in);
        else CHECK(on == strlen(T[i].want) + 3 && !memcmp(out + 3, T[i].want, on - 3) && out[2] == 0x01, "plain: %s -> %.*s (%s)", T[i].in, (int)(on > 3 ? on - 3 : 0), out + 3, d3_reason(r));
    }
    /* the same packet over two paths gets the same duplicate key */
    {
        uint8_t a[100] = { 0x3C, 0xFF, 0x01 }, b[100] = { 0x3C, 0xFF, 0x01 }; d3_job_t ja, jb;
        const char *pa = "K6ATV-3>APZLPI,WIDE1-1,WIDE2-1:>hi", *pb = "K6ATV-3>APZLPI,K6ATV-10*,WIDE2-1:>hi";
        memcpy(a + 3, pa, strlen(pa)); memcpy(b + 3, pb, strlen(pb));
        CHECK(d3_decide(a, strlen(pa) + 3, "K6ATV-11", D3_MODE_WIDE, &ja) == D3_OK && d3_decide(b, strlen(pb) + 3, "K6ATV-11", D3_MODE_WIDE, &jb) == D3_OK && ja.key == jb.key,
              "duplicate key should not depend on the path");
    }
    /* LACF 2.0 frames are not relayed by this build */
    { uint8_t f2[] = { 0x3C, 0xFF, 0x02, 0x10, 0x00 }; d3_job_t j; CHECK(d3_decide(f2, sizeof f2, "K6ATV-11", D3_MODE_WIDE, &j) == D3_LACF2, "LACF 2.0 frame"); }
}

static void rxt_cases(void)
{
    uint8_t t[4];
    CHECK(d3_rxt_rssi(-130) == 0 && d3_rxt_rssi(-41) == 89 && d3_rxt_rssi(-80) == 50 && d3_rxt_rssi(-150) == 0 && d3_rxt_rssi(0) == 89, "rssi scale");
    CHECK(d3_rxt_snr(-24) == 0 && d3_rxt_snr(20) == 88 && d3_rxt_snr(-14) == 20 && d3_rxt_snr(30) == 88, "snr scale");
    CHECK(d3_rxt_freq_offset(0) == 45 && d3_rxt_freq_offset(-2500) == 0 && d3_rxt_freq_offset(2500) == 89 && d3_rxt_freq_offset(-9000) == 0, "fo scale");
    CHECK(d3_rxt_time_to_hop(0, 12, 125) == 0 && d3_rxt_time_to_hop(1e9f, 12, 125) == 89, "tth ends");
    /* the LoRa32 relay of 2 November 2026: TTH 3,338 ms at SF12 -> ln(3338/327.68 + 1)/ln 1.08 = 31.4 */
    CHECK(d3_rxt_time_to_hop(3338, 12, 125) == 31, "tth 3338 ms = %d", d3_rxt_time_to_hop(3338, 12, 125));
    d3_rxt_tuple(-81, 12.75f, -3324, 3338, 12, 125, t);
    CHECK(t[0] == 49 && t[1] == 74 && t[2] == 0 && t[3] == 31, "tuple %d %d %d %d", t[0], t[1], t[2], t[3]);
}

#ifdef D3_WITH_MODEL
/* Encode vectors (TNC2 -> frame, '-' = sent as plain text) through the digipeater's own path: a plain job with convert set, and decode back. */
static void run_encode(const char *dir, int *n_ok, int *n_lines)
{
    char path[512], line[4096]; FILE *f;
    snprintf(path, sizeof path, "%s/vectors_hb3_v1.txt", dir);
    f = fopen(path, "r");
    if (!f) { printf("FAIL cannot open %s\n", path); fails++; return; }
    while (fgets(line, sizeof line, f)) {
        char *a, *b, back[400]; uint8_t fr[300], tx[300], out[300]; size_t fn, tn, on; d3_job_t job; hb3_info_t in; int dl, ok = 1;
        if (line[0] == '#' || line[0] == '\n') continue;
        a = strtok(line, " \n"); b = strtok(NULL, " \n");
        if (!a || !b) continue;
        (*n_lines)++;
        tn = unhex(b, tx, sizeof tx - 1); tx[tn] = 0;
        if (tn >= sizeof job.text) {                                      /* longer than any packet the radio can carry: only the codec itself */
            on = hb3_encode((const char *)tx, tn, NULL, out, sizeof out, &in);
            if (!strcmp(a, "-")) { CHECK(on == 0, "encode: a %zu-byte packet should not encode", tn); if (!on) (*n_ok)++; continue; }
        } else {
            memset(&job, 0, sizeof job); job.fmt = D3_FMT_PLAIN; job.convert = 1; memcpy(job.text, tx, tn); job.text_len = (uint16_t)tn;
            on = d3_build(&job, "N0CALL", NULL, out, sizeof out);
        }
        if (!strcmp(a, "-")) {                                            /* not encodable: the digipeater sends it as plain text, if it fits */
            ok = tn + 3 <= D3_MAX_FRAME ? (on == tn + 3 && out[2] == 0x01) : on == 0;
            CHECK(ok, "encode: %s should go out as plain text", (char *)tx); }
        else {
            fn = unhex(a, fr, sizeof fr);
            ok = on == fn && !memcmp(out, fr, fn);
            CHECK(ok, "encode: %s differs from the reference", (char *)tx);
            dl = hb3_decode(fr, fn, back, sizeof back, &in);
            CHECK(dl == (int)tn && !memcmp(back, tx, tn), "decode: %s", a); ok = ok && dl == (int)tn;
        }
        if (ok) (*n_ok)++;
    }
    fclose(f);
}
#endif

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : "../lora-aprs-ideas/test";
    const char *files[] = { "vectors_hb3_passthrough_v1.txt", "vectors_hb3_relay_v1.txt", "vectors_hb3_relay_chain_v1.txt", "vectors_hb3_relay_edge_v1.txt" };
    size_t i;
#ifdef D3_WITH_MODEL
    CHECK(d3_init_model() == 62, "destination list");
#endif
    plain_cases();
    rxt_cases();
    for (i = 0; i < sizeof files / sizeof files[0]; i++) {
        int ok = 0, lines = 0;
        run_file(dir, files[i], &ok, &lines);
        printf("%-34s %d of %d as the reference\n", files[i], ok, lines);
    }
#ifdef D3_WITH_MODEL
    { int ok = 0, lines = 0; run_encode(dir, &ok, &lines); printf("%-34s %d of %d as the reference (encode and decode)\n", "vectors_hb3_v1.txt", ok, lines); }
#endif
    printf("%s: %d checks, %d failed\n", fails ? "FAILED" : "OK", checks, fails);
    return fails != 0;
}
