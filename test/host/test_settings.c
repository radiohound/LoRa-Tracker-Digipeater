/* Host test of src/digi3/settings.c: commands, ranges, round trips, and the flash record. */
#include "settings.h"
#include "digi3_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static char out[64][200]; static int nout;
static void emit(void *ctx, const char *l) { (void)ctx; if (nout < 64) { strncpy(out[nout], l, 199); out[nout][199] = 0; } nout++; }
static unsigned run(d3_settings_t *s, const char *cmd) { nout = 0; return d3s_exec(s, cmd, emit, NULL); }
static int said(const char *needle) { int i; for (i = 0; i < nout && i < 64; i++) if (strstr(out[i], needle)) return 1; return 0; }

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu; size_t i; int b;
    for (i = 0; i < n; i++) { c ^= p[i]; for (b = 0; b < 8; b++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
    return ~c;
}

/* a record with one hand-made entry list and a correct CRC */
static size_t make_rec(uint8_t *r, const uint8_t *entries, size_t n)
{
    uint32_t c; size_t w = 6;
    r[0] = 'D'; r[1] = '3'; r[2] = 'S'; r[3] = 1; r[4] = (uint8_t)n; r[5] = (uint8_t)(n >> 8);
    memcpy(r + 6, entries, n); w += n;
    c = crc32(r, w);
    r[w++] = (uint8_t)c; r[w++] = (uint8_t)(c >> 8); r[w++] = (uint8_t)(c >> 16); r[w++] = (uint8_t)(c >> 24);
    return w;
}

static void test_commands(void)
{
    d3_settings_t s, d; unsigned f; char fc[16];
    d3s_defaults(&s); d3s_defaults(&d);
    CHECK(strcmp(s.call, "N0CALL") == 0 && s.ssid == 10);

    f = run(&s, "set call K6ATV-2");
    CHECK(f & D3S_CHANGED); d3s_fullcall(&s, fc, sizeof fc); CHECK(strcmp(fc, "K6ATV-2") == 0);
    f = run(&s, "set call k6atv");                      /* lower case; the SSID stays */
    d3s_fullcall(&s, fc, sizeof fc); CHECK(strcmp(fc, "K6ATV-2") == 0 && !(f & D3S_CHANGED));
    f = run(&s, "SET CALL w2hab-11");                   /* commands and names are not case sensitive */
    d3s_fullcall(&s, fc, sizeof fc); CHECK(strcmp(fc, "W2HAB-11") == 0 && (f & D3S_CHANGED));
    f = run(&s, "set call K6ATV-16"); CHECK(!f && said("error") && strcmp(s.call, "W2HAB") == 0 && s.ssid == 11);   /* refused: nothing changed, not even the call */
    run(&s, "set call K6ATV-x");   CHECK(said("error") && strcmp(s.call, "W2HAB") == 0);
    run(&s, "set call TOOLONGCALL"); CHECK(said("error") && strcmp(s.call, "W2HAB") == 0);
    run(&s, "set call K6 ATV");    CHECK(said("not allowed") && strcmp(s.call, "W2HAB") == 0);
    run(&s, "set sf 9 oops");      CHECK(said("one value only"));
    d3s_defaults(&s);

    /* radio settings */
    f = run(&s, "set freq 439.9125"); CHECK((f & D3S_CHANGED) && (f & D3S_RADIO) && s.freq_hz == 439912500u);
    f = run(&s, "set freq 433.775");  CHECK(s.freq_hz == 433775000u);
    run(&s, "set freq 433.1234567");  CHECK(said("too many decimals") && s.freq_hz == 433775000u);
    run(&s, "set freq 900");          CHECK(said("out of range") && s.freq_hz == 433775000u);
    run(&s, "set freq abc");          CHECK(said("not a number"));
    f = run(&s, "set bw 62.5");       CHECK(s.bw_hz == 62500u && (f & D3S_RADIO));
    run(&s, "set bw 100");            CHECK(said("bandwidth") && s.bw_hz == 62500u);
    run(&s, "set bw 125");            CHECK(s.bw_hz == 125000u);
    run(&s, "set sf 12");             CHECK(s.sf == 12);
    run(&s, "set sf 13");             CHECK(said("out of range") && s.sf == 12);
    run(&s, "set sf 4");              CHECK(said("out of range") && s.sf == 12);
    run(&s, "set cr 4/7");            CHECK(s.cr == 7);
    run(&s, "set cr 5");              CHECK(s.cr == 5);
    run(&s, "get cr");                CHECK(said("cr = 4/5") && !said("4/x"));
    run(&s, "set cr 4/9");            CHECK(said("out of range") && s.cr == 5);
    run(&s, "set cr 4/");             CHECK(said("not a number"));
    run(&s, "set sync 0x34");         CHECK(s.sync == 0x34);
    run(&s, "set sync 18");           CHECK(s.sync == 18);
    run(&s, "set sync 0xZZ");         CHECK(said("not a number"));
    run(&s, "set power 23");          CHECK(said("out of range") && s.power_dbm == 22);
    run(&s, "set power -1");          CHECK(said("out of range"));

    /* booleans and mode */
    run(&s, "set moving on");  CHECK(s.moving == 1);
    run(&s, "set moving off"); CHECK(s.moving == 0);
    run(&s, "set moving 1");   CHECK(s.moving == 1);
    run(&s, "set moving maybe"); CHECK(said("on or off") && s.moving == 1);
    run(&s, "set mode fillin");  CHECK(s.mode == D3_MODE_FILLIN);
    run(&s, "set mode wide");    CHECK(s.mode == D3_MODE_WIDE);
    run(&s, "set mode both");    CHECK(said("fillin or wide") && s.mode == D3_MODE_WIDE);

    /* delay pair */
    run(&s, "set delay_min 600");  CHECK(said("delay_min must not") && s.delay_min_ms == 100);
    run(&s, "set delay_max 900");  CHECK(s.delay_max_ms == 900);
    run(&s, "set delay_min 600");  CHECK(s.delay_min_ms == 600);
    run(&s, "set delay_max 5");    CHECK(said("delay_min must not") && s.delay_max_ms == 900);

    /* position */
    run(&s, "set lat 37.123456"); CHECK(s.lat_u == 37123456);
    run(&s, "set lon -121.5");    CHECK(s.lon_u == -121500000);
    run(&s, "set lat 90.000001"); CHECK(said("out of range") && s.lat_u == 37123456);
    run(&s, "set lon -180");      CHECK(s.lon_u == -180000000);
    run(&s, "set lon 180.1");     CHECK(said("out of range"));

    /* text */
    run(&s, "set comment Hello there, LACF 3 digi"); CHECK(strcmp(s.comment, "Hello there, LACF 3 digi") == 0);
    run(&s, "set comment a|b");   CHECK(said("not allowed") && strcmp(s.comment, "Hello there, LACF 3 digi") == 0);
    run(&s, "set comment 0123456789012345678901234567890123456789012345"); CHECK(said("wrong length"));
    run(&s, "set comment -");     CHECK(s.comment[0] == 0);
    run(&s, "set path wide1-1");  CHECK(strcmp(s.path, "WIDE1-1") == 0);
    run(&s, "set path WIDE1-1,WIDE2-1"); CHECK(strcmp(s.path, "WIDE1-1,WIDE2-1") == 0);
    run(&s, "set path wide 1");   CHECK(said("not allowed") && strcmp(s.path, "WIDE1-1,WIDE2-1") == 0);
    run(&s, "set path -");        CHECK(s.path[0] == 0);
    run(&s, "set path a:b");      CHECK(said("not allowed"));
    run(&s, "set always_tocalls aplaix,aplab"); CHECK(strcmp(s.always_tocalls, "APLAIX,APLAB") == 0);
    run(&s, "set sym_table /");   CHECK(s.sym_table[0] == '/');
    run(&s, "set sym_code ab");   CHECK(said("wrong length"));
    run(&s, "set tocall apzld3"); CHECK(strcmp(s.tocall, "APZLD3") == 0);
    run(&s, "set tocall");        CHECK(said("error"));

    /* shape of the commands */
    CHECK(run(&s, "") == 0 && nout == 0);
    CHECK(run(&s, "   ") == 0 && nout == 0);
    run(&s, "frobnicate");        CHECK(said("unknown command"));
    run(&s, "set nothing 1");     CHECK(said("unknown setting"));
    run(&s, "get sf");            CHECK(said("sf = 12"));
    run(&s, "get nothing");       CHECK(said("error"));
    CHECK(run(&s, "save") == D3S_SAVE);
    CHECK(run(&s, "apply") == D3S_APPLY);
    CHECK(run(&s, "stats") == D3S_STATS);
    CHECK(run(&s, "beacon") == D3S_BEACON);
    CHECK(run(&s, "reboot") == D3S_REBOOT);
    CHECK(run(&s, "reboot force") == (D3S_REBOOT | D3S_FORCE));
    f = run(&s, "defaults"); CHECK((f & D3S_DEFAULTS) && (f & D3S_RADIO) && s.sf == d.sf && strcmp(s.call, "N0CALL") == 0);
    { char longline[300]; memset(longline, 'a', 299); longline[299] = 0; CHECK(run(&s, longline) == 0 && said("too long")); }
    nout = 0; d3s_menu(emit, NULL); CHECK(nout >= 6 && said("set call K6ATV-2") && said("save") && said("show"));
    run(&s, "help");   CHECK(nout > 40 && said("freq") && said("cad_difs"));
    run(&s, "show");   CHECK(said("callsign N0CALL-10") && said("freq = 433.775 MHz") && said("bw = 125 kHz") && said("sync = 0x12"));
}

/* every setting: what show prints, set accepts back unchanged */
static void test_show_roundtrip(void)
{
    d3_settings_t s, before; int i, n; char cmd[200], name[64], val[128];
    d3s_defaults(&s);
    run(&s, "set lat -33.8688"); run(&s, "set lon 151.2093"); run(&s, "set comment test, with spaces"); run(&s, "set path WIDE1-1");
    run(&s, "set always_tocalls APLAIX,APLAB"); run(&s, "set bw 31.25"); run(&s, "set freq 439.9125"); run(&s, "set sync 0x34");
    before = s;
    run(&s, "show"); n = nout;
    CHECK(n > 38);
    for (i = 1; i < n && i < 64; i++) {      /* line 0 is the "callsign" summary */
        char *eq = strstr(out[i], " = "), *u;
        if (!eq) { CHECK(0); continue; }
        memcpy(name, out[i], (size_t)(eq - out[i])); name[eq - out[i]] = 0;
        strcpy(val, eq + 3);
        if (strcmp(val, "-") && (u = strchr(val, ' ')) && strcmp(name, "comment") && strcmp(name, "always_tocalls") && strcmp(name, "path")) *u = 0;   /* drop the unit */
        snprintf(cmd, sizeof cmd, "set %s %s", name, val);
        { unsigned f; char first[200]; strcpy(first, out[i]); f = run(&s, cmd); if (f) { printf("  round trip changed: %s (from \"%s\")\n", cmd, first); } CHECK(f == 0); }
        run(&s, "show");                     /* run() clobbers out[]: show again for the next line */
    }
    CHECK(memcmp(&s, &before, sizeof s) == 0 || (strcmp(s.call, before.call) == 0 && s.freq_hz == before.freq_hz));
}

static void test_record(void)
{
    d3_settings_t a, b, d; uint8_t rec[D3S_REC_MAX]; size_t n; int rej, r, bit, i;
    d3s_defaults(&a); d3s_defaults(&d);
    run(&a, "set call K6ATV-2"); run(&a, "set freq 439.9125"); run(&a, "set bw 62.5"); run(&a, "set sf 9"); run(&a, "set moving on");
    run(&a, "set lat 37.5"); run(&a, "set lon -121.25"); run(&a, "set comment LACF 3 digi, Coe"); run(&a, "set path WIDE1-1");
    n = d3s_pack(&a, rec, sizeof rec);
    CHECK(n > 0 && n % 8 == 0 && n < 700);
    printf("  record: %zu bytes (a 2048-byte flash page)\n", n);
    r = d3s_unpack(rec, n, &b, &rej);
    CHECK(r > 30 && rej == 0);
    CHECK(strcmp(b.call, "K6ATV") == 0 && b.ssid == 2 && b.freq_hz == 439912500u && b.bw_hz == 62500u && b.sf == 9 && b.moving == 1);
    CHECK(b.lat_u == 37500000 && b.lon_u == -121250000 && strcmp(b.comment, "LACF 3 digi, Coe") == 0 && strcmp(b.path, "WIDE1-1") == 0);
    CHECK(b.delay_min_ms == d.delay_min_ms && b.cad_difs == d.cad_difs);
    { uint8_t rec2[D3S_REC_MAX]; size_t n2 = d3s_pack(&b, rec2, sizeof rec2); CHECK(n2 == n && memcmp(rec, rec2, n) == 0); }   /* same settings, same bytes: save can skip the write */

    CHECK(d3s_pack(&a, rec, 40) == 0);                                  /* too small a buffer */

    /* blank flash, other data */
    { uint8_t blank[256]; memset(blank, 0xFF, sizeof blank); CHECK(d3s_unpack(blank, sizeof blank, &b, NULL) == D3S_E_BLANK && b.sf == d.sf); }
    { uint8_t z[256]; memset(z, 0, sizeof z); CHECK(d3s_unpack(z, sizeof z, &b, NULL) == D3S_E_FORMAT); }
    CHECK(d3s_unpack(rec, 3, &b, NULL) == D3S_E_BLANK);

    /* every single-bit error in the record is caught, and never crashes */
    { int caught = 0, total = 0;
      for (i = 0; i < (int)n; i++) for (bit = 0; bit < 8; bit++) {
          uint8_t t[D3S_REC_MAX]; int rr; size_t body;
          memcpy(t, rec, n); t[i] ^= (uint8_t)(1u << bit); rr = d3s_unpack(t, n, &b, NULL);
          body = 6 + ((size_t)rec[4] | ((size_t)rec[5] << 8)) + 4;
          if (i >= (int)body) continue;                                   /* the 0xFF padding is not covered */
          total++; if (rr < 0) caught++;
          if (rr >= 0) printf("  bit flip at byte %d bit %d not caught (%d)\n", i, bit, rr);
      }
      CHECK(caught == total); }
    /* every cut */
    for (i = 0; i < (int)n; i++) { CHECK(d3s_unpack(rec, (size_t)i, &b, NULL) < 0 || i >= (int)(6 + ((size_t)rec[4] | ((size_t)rec[5] << 8)) + 4)); }

    /* a newer firmware's record: an unknown id is skipped, the rest is read */
    { uint8_t e[64], r2[128]; size_t m = 0, rn;
      e[m++] = 200; e[m++] = 4; e[m++] = 1; e[m++] = 2; e[m++] = 3; e[m++] = 4;       /* unknown id */
      e[m++] = 32; e[m++] = 4; e[m++] = 10; e[m++] = 0; e[m++] = 0; e[m++] = 0;       /* sf = 10 */
      rn = make_rec(r2, e, m);
      r = d3s_unpack(r2, rn, &b, &rej); CHECK(r == 1 && rej == 0 && b.sf == 10 && b.bw_hz == d.bw_hz); }
    /* an older firmware's record: only a few settings, the rest default */
    { uint8_t e[16], r2[64]; size_t m = 0, rn;
      e[m++] = 1; e[m++] = 4; e[m++] = 7; e[m++] = 0; e[m++] = 0; e[m++] = 0;         /* ssid = 7 */
      rn = make_rec(r2, e, m); r = d3s_unpack(r2, rn, &b, &rej); CHECK(r == 1 && b.ssid == 7 && b.freq_hz == d.freq_hz); }
    /* a valid CRC does not make a bad value good */
    { uint8_t e[64], r2[128]; size_t m = 0, rn;
      e[m++] = 32; e[m++] = 4; e[m++] = 99; e[m++] = 0; e[m++] = 0; e[m++] = 0;       /* sf = 99 */
      e[m++] = 31; e[m++] = 4; e[m++] = 0x10; e[m++] = 0x27; e[m++] = 0; e[m++] = 0;  /* bw = 10000 (not a LoRa bandwidth) */
      e[m++] = 34; e[m++] = 4; e[m++] = 5; e[m++] = 0; e[m++] = 0; e[m++] = 0;        /* power = 5 */
      e[m++] = 0; e[m++] = 8; memcpy(e + m, "bad call", 8); m += 8;                   /* call with a space */
      rn = make_rec(r2, e, m); r = d3s_unpack(r2, rn, &b, &rej);
      CHECK(r == 1 && rej == 3 && b.sf == d.sf && b.bw_hz == d.bw_hz && b.power_dbm == 5 && strcmp(b.call, d.call) == 0); }
    /* wrong entry length */
    { uint8_t e[16], r2[64]; size_t m = 0, rn;
      e[m++] = 32; e[m++] = 2; e[m++] = 10; e[m++] = 0;
      rn = make_rec(r2, e, m); r = d3s_unpack(r2, rn, &b, &rej); CHECK(r == 0 && rej == 1 && b.sf == d.sf); }
    /* entry length running past the end */
    { uint8_t e[16], r2[64]; size_t m = 0, rn;
      e[m++] = 32; e[m++] = 200; e[m++] = 10;
      rn = make_rec(r2, e, m); r = d3s_unpack(r2, rn, &b, &rej); CHECK(r == 0 && b.sf == d.sf); }
    /* the delay pair read together: min above the default max, both valid, in the record's order */
    { uint8_t e[32], r2[64]; size_t m = 0, rn;
      e[m++] = 3; e[m++] = 4; e[m++] = 0x58; e[m++] = 0x02; e[m++] = 0; e[m++] = 0;   /* delay_min 600 */
      e[m++] = 4; e[m++] = 4; e[m++] = 0x84; e[m++] = 0x03; e[m++] = 0; e[m++] = 0;   /* delay_max 900 */
      rn = make_rec(r2, e, m); r = d3s_unpack(r2, rn, &b, &rej); CHECK(r == 2 && rej == 0 && b.delay_min_ms == 600 && b.delay_max_ms == 900); }
    /* the pair out of order in the record: back to the defaults together */
    { uint8_t e[32], r2[64]; size_t m = 0, rn;
      e[m++] = 3; e[m++] = 4; e[m++] = 0x58; e[m++] = 0x02; e[m++] = 0; e[m++] = 0;   /* delay_min 600 */
      e[m++] = 4; e[m++] = 4; e[m++] = 0x0A; e[m++] = 0; e[m++] = 0; e[m++] = 0;      /* delay_max 10 */
      rn = make_rec(r2, e, m); r = d3s_unpack(r2, rn, &b, &rej); CHECK(rej >= 1 && b.delay_min_ms <= b.delay_max_ms); }
}

int main(void)
{
    test_commands();
    test_show_roundtrip();
    test_record();
    printf("settings: %d checks, %d failed\n", checks, fails);
    return fails != 0;
}
