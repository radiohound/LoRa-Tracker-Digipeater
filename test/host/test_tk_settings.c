/* PC test of src/settings/tk_settings.c: commands, ranges, round trips, and the flash record. */
#include "tk_settings.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0, checks = 0;
#define CHECK(c) do { checks++; if (!(c)) { fails++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static char out[96][200]; static int nout;
static void emit(void *ctx, const char *l) { (void)ctx; if (nout < 96) { strncpy(out[nout], l, 199); out[nout][199] = 0; } nout++; }
static unsigned run(tk_settings_t *s, const char *cmd) { nout = 0; return tks_exec(s, cmd, emit, NULL); }
static int said(const char *needle) { int i; for (i = 0; i < nout && i < 96; i++) if (strstr(out[i], needle)) return 1; return 0; }

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
    r[0] = 'T'; r[1] = 'K'; r[2] = 'S'; r[3] = 1; r[4] = (uint8_t)n; r[5] = (uint8_t)(n >> 8);
    memcpy(r + 6, entries, n); w += n;
    c = crc32(r, w);
    r[w++] = (uint8_t)c; r[w++] = (uint8_t)(c >> 8); r[w++] = (uint8_t)(c >> 16); r[w++] = (uint8_t)(c >> 24);
    return w;
}

static void test_defaults_and_callsign(void)
{
    tk_settings_t s; unsigned f; char fc[16];
    tks_defaults(&s);
    CHECK(strcmp(s.call, "N0CALL") == 0 && s.ssid == 0);
    CHECK(!tks_call_is_set(&s));                          /* nothing may be sent until a callsign is set */
    f = run(&s, "set call W2HAB-11");
    CHECK((f & TKS_CHANGED) && tks_call_is_set(&s));
    tks_fullcall(&s, fc, sizeof fc); CHECK(strcmp(fc, "W2HAB-11") == 0);
    f = run(&s, "set call k6atv");                        /* lower case in; the SSID stays */
    tks_fullcall(&s, fc, sizeof fc); CHECK(strcmp(fc, "K6ATV-11") == 0 && (f & TKS_CHANGED));
    run(&s, "set call K6ATV-100"); CHECK(said("error") && strcmp(s.call, "K6ATV") == 0 && s.ssid == 11);   /* refused: nothing changed */
    run(&s, "set call K6ATV-x");   CHECK(said("error") && s.ssid == 11);
    run(&s, "set call TOOLONGCALL"); CHECK(said("wrong length"));
    run(&s, "set call K6 ATV");    CHECK(said("not allowed"));
    run(&s, "set call N0CALL-5");  CHECK(!tks_call_is_set(&s));   /* setting N0CALL again switches sending off */
    run(&s, "SET Call w2hab-11");  CHECK(tks_call_is_set(&s));    /* commands and names are not case sensitive */
}

static void test_numbers_and_radio(void)
{
    tk_settings_t s; unsigned f;
    tks_defaults(&s);
    f = run(&s, "set beacon_s 60");   CHECK((f & TKS_CHANGED) && !(f & TKS_RADIO) && s.beacon_s == 60);
    run(&s, "set beacon_s 4");        CHECK(said("out of range") && s.beacon_s == 60);
    run(&s, "set beacon_s 86401");    CHECK(said("out of range") && s.beacon_s == 60);
    run(&s, "set beacon_s 5 oops");   CHECK(said("one value only"));
    run(&s, "set beacon_s abc");      CHECK(said("not a number"));
    f = run(&s, "set freq 433.775");  CHECK(!(f & TKS_CHANGED) && s.freq_hz == 433775000u);   /* same as the default */
    f = run(&s, "set freq 439.9125"); CHECK((f & TKS_CHANGED) && (f & TKS_RADIO) && s.freq_hz == 439912500u && said("type apply"));
    run(&s, "set freq 300");          CHECK(said("out of range"));
    run(&s, "set bw 100");            CHECK(said("not a LoRa bandwidth"));
    run(&s, "set bw 250");            CHECK(s.bw_hz == 250000u);
    run(&s, "set sf 13");             CHECK(said("out of range"));
    run(&s, "set cr 4/8");            CHECK(s.cr == 8);
    run(&s, "set cr 6");              CHECK(s.cr == 6);
    run(&s, "set sync 0x34");         CHECK(s.sync == 0x34);
    run(&s, "set power 22");          CHECK(s.power_dbm == 22);
    run(&s, "set power 23");          CHECK(said("out of range"));
    CHECK((run(&s, "apply") & TKS_APPLY) && (run(&s, "save") & TKS_SAVE) && (run(&s, "beacon") & TKS_BEACON) && (run(&s, "horus") & TKS_HORUS));
    CHECK((run(&s, "reboot") & TKS_REBOOT) && !(run(&s, "reboot") & TKS_FORCE) && (run(&s, "reboot force") & TKS_FORCE));
    run(&s, "nonsense"); CHECK(said("unknown command"));
    run(&s, "set nonsense 1"); CHECK(said("unknown setting"));
    run(&s, "set sf"); CHECK(said("set sf VALUE"));
}

static void test_text(void)
{
    tk_settings_t s;
    tks_defaults(&s);
    run(&s, "set comment Balloon payload, W2HAB");   CHECK(strcmp(s.comment, "Balloon payload, W2HAB") == 0);   /* comments keep their case */
    run(&s, "set comment a|b");                       CHECK(said("not allowed"));
    run(&s, "set path wide1-1");                      CHECK(strcmp(s.path, "WIDE1-1") == 0);                   /* paths are upper-cased */
    run(&s, "set path -");                            CHECK(s.path[0] == 0);
    run(&s, "show");                                  CHECK(said("path = -"));
    run(&s, "set sym_code OO");                       CHECK(said("wrong length"));
    run(&s, "set sym_code O");                        CHECK(s.sym_code[0] == 'O');
}

static void test_horus(void)
{
    tk_settings_t s; unsigned f; char hc[20];
    tks_defaults(&s);
    CHECK(s.horus_ver == 3 && s.horus_call[0] == 0);
    run(&s, "set call W2HAB-11");
    tks_horus_call(&s, hc, sizeof hc); CHECK(strcmp(hc, "W2HAB-11") == 0);      /* empty horus_call = the callsign */
    run(&s, "set horus_call K6ATV/Balloon-1");                                  /* case sensitive, / and - allowed */
    tks_horus_call(&s, hc, sizeof hc); CHECK(strcmp(hc, "K6ATV/Balloon-1") == 0);
    run(&s, "set horus_call SIXTEENCHARSXXXX");  CHECK(said("wrong length"));    /* 16 characters */
    run(&s, "set horus_call FIFTEENCHARSXXX");   CHECK(!said("error") && strlen(s.horus_call) == 15);   /* 15 is the longest allowed */
    run(&s, "set horus_call bad_call");         CHECK(said("not allowed"));
    run(&s, "set horus_call -");                tks_horus_call(&s, hc, sizeof hc); CHECK(strcmp(hc, "W2HAB-11") == 0);
    run(&s, "set horus_ver 1");   CHECK(said("out of range"));
    run(&s, "set horus_ver 2");   CHECK(s.horus_ver == 2);
    run(&s, "set horus_id 255");  CHECK(said("out of range"));
    run(&s, "set horus_id 930");  CHECK(s.horus_id == 930);
    f = run(&s, "set horus_freq 434.201"); CHECK((f & TKS_CHANGED) && !(f & TKS_RADIO) && s.horus_freq_hz == 434201000u);
    f = run(&s, "set horus_freq 431.051"); CHECK((f & TKS_CHANGED) && s.horus_freq_hz == 431051000u);
    f = run(&s, "set horus_freq 431.051"); CHECK(!(f & TKS_CHANGED) && said("(no change)"));      /* the same value again */
    run(&s, "set horus_power -9"); CHECK(s.horus_power_dbm == -9 && said("= -9 dBm"));      /* a negative value in an int8 */
    run(&s, "set horus_power -10"); CHECK(said("out of range") && s.horus_power_dbm == -9);
    run(&s, "set horus_power 22"); CHECK(s.horus_power_dbm == 22);
    /* time slots: the offset must be inside the cycle */
    run(&s, "set horus_off 5");    CHECK(s.horus_off_s == 5 && !said("error"));          /* sync is 0 (off): any offset below 3600 */
    run(&s, "set horus_sync 10");  CHECK(s.horus_sync_s == 10 && s.horus_off_s == 5);
    run(&s, "set horus_off 10");   CHECK(said("below horus_sync") && s.horus_off_s == 5);
    run(&s, "set horus_sync 5");   CHECK(said("below horus_sync") && s.horus_sync_s == 10);
    run(&s, "set horus_div 0");    CHECK(said("out of range"));
    run(&s, "set horus on");       CHECK(s.horus == 1);
}

static void test_show_help(void)
{
    tk_settings_t s;
    tks_defaults(&s);
    run(&s, "show");   CHECK(said("callsign N0CALL") && said("call = N0CALL") && said("freq = 433.775 MHz *") && said("beacon_s = 30 s") && !said("beacon_s = 30 s *") && said("horus_ver = 3") && said("* = LoRa radio setting"));
    run(&s, "get freq"); CHECK(said("freq = 433.775 MHz") && !said("*"));
    run(&s, "get nonsense"); CHECK(said("error"));
    run(&s, "help");   CHECK(said("* freq") && said("  horus_ver") && said("Nothing is sent while the callsign is N0CALL"));
    run(&s, "?");      CHECK(said("Commands"));
    CHECK(run(&s, "") == 0 && run(&s, "   ") == 0);
}

static void test_record(void)
{
    tk_settings_t s, r, d; uint8_t rec[TKS_REC_MAX]; size_t n; int rej, got; uint8_t e[64]; size_t k;
    tks_defaults(&d);

    /* a record round trip with changes in every kind of field, including a negative one and a text one */
    tks_defaults(&s);
    run(&s, "set call W2HAB-11"); run(&s, "set comment Hello balloon"); run(&s, "set beacon_s 45"); run(&s, "set aprs off");
    run(&s, "set freq 439.9125"); run(&s, "set bw 250"); run(&s, "set cr 6"); run(&s, "set sync 0x34");
    run(&s, "set horus on"); run(&s, "set horus_ver 2"); run(&s, "set horus_id 930"); run(&s, "set horus_call K6ATV/B1");
    run(&s, "set horus_freq 431.051"); run(&s, "set horus_power -9"); run(&s, "set horus_sync 10"); run(&s, "set horus_off 5");
    n = tks_pack(&s, rec, sizeof rec);
    CHECK(n > 0 && n % 8 == 0 && n <= TKS_REC_MAX);
    got = tks_unpack(rec, n, &r, &rej);
    CHECK(got > 0 && rej == 0 && memcmp(&s, &r, sizeof s) == 0);       /* every field survives */
    CHECK(r.horus_power_dbm == -9 && strcmp(r.horus_call, "K6ATV/B1") == 0 && r.horus_off_s == 5);
    CHECK(tks_pack(&s, rec, 10) == 0);                                 /* too small */

    /* a blank page, a foreign record, a damaged record */
    memset(rec, 0xFF, sizeof rec);                      CHECK(tks_unpack(rec, 64, &r, NULL) == TKS_E_BLANK && memcmp(&r, &d, sizeof r) == 0);
    n = tks_pack(&s, rec, sizeof rec); rec[0] = 'D';    CHECK(tks_unpack(rec, n, &r, NULL) == TKS_E_FORMAT);
    n = tks_pack(&s, rec, sizeof rec); rec[10] ^= 0x40; CHECK(tks_unpack(rec, n, &r, NULL) == TKS_E_CRC && memcmp(&r, &d, sizeof r) == 0);

    /* an out-of-range value is skipped and counted, the others still load; an id from a newer firmware is skipped silently */
    k = 0;
    e[k++] = 7;  e[k++] = 4; e[k++] = 60; e[k++] = 0; e[k++] = 0; e[k++] = 0;            /* beacon_s = 60: fine */
    e[k++] = 12; e[k++] = 4; e[k++] = 99; e[k++] = 0; e[k++] = 0; e[k++] = 0;            /* sf = 99: out of range */
    e[k++] = 200; e[k++] = 2; e[k++] = 1; e[k++] = 2;                                      /* id 200: unknown, from a newer firmware */
    e[k++] = 2;  e[k++] = 3; e[k++] = 'h'; e[k++] = 'i'; e[k++] = '!';                     /* comment = hi! */
    n = make_rec(rec, e, k);
    got = tks_unpack(rec, n, &r, &rej);
    CHECK(got == 2 && rej == 1 && r.beacon_s == 60 && r.sf == d.sf && strcmp(r.comment, "hi!") == 0);

    /* a bad value for a bool, and a text entry that is too long or has a character that is not allowed */
    k = 0;
    e[k++] = 8;  e[k++] = 4; e[k++] = 7; e[k++] = 0; e[k++] = 0; e[k++] = 0;             /* aprs = 7: not a bool */
    e[k++] = 19; e[k++] = 4; e[k++] = 'a'; e[k++] = '_'; e[k++] = 'b'; e[k++] = 'c';     /* horus_call with '_' */
    n = make_rec(rec, e, k);
    got = tks_unpack(rec, n, &r, &rej);
    CHECK(got == 0 && rej == 2 && r.aprs == d.aprs && r.horus_call[0] == 0);

    /* slots that do not fit together fall back to the defaults */
    k = 0;
    e[k++] = 23; e[k++] = 4; e[k++] = 10; e[k++] = 0; e[k++] = 0; e[k++] = 0;            /* horus_sync = 10 */
    e[k++] = 24; e[k++] = 4; e[k++] = 20; e[k++] = 0; e[k++] = 0; e[k++] = 0;            /* horus_off = 20: not below it */
    n = make_rec(rec, e, k);
    got = tks_unpack(rec, n, &r, &rej);
    CHECK(rej == 1 && r.horus_sync_s == d.horus_sync_s && r.horus_off_s == d.horus_off_s);

    /* defaults loaded over changes */
    tks_defaults(&s); run(&s, "set call W2HAB-11");
    CHECK((run(&s, "defaults") & TKS_DEFAULTS) && !tks_call_is_set(&s));
}

int main(void)
{
    test_defaults_and_callsign(); test_numbers_and_radio(); test_text(); test_horus(); test_show_help(); test_record();
    printf("%d checks, %d failures\n", checks, fails);
    return fails ? 1 : 0;
}
