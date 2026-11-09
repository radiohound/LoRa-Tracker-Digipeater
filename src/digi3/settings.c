/* Run-time settings, serial command line and flash record. See settings.h. */
#include "settings.h"
#include "digi3_core.h"
#include "config_digi3.h"
#include <ctype.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

enum { K_NUM, K_BOOL, K_MODE, K_STR };
enum { CS_CALL, CS_PATH, CS_TOCALLS, CS_TEXT, CS_SYM };
#define F_RADIO 1u      /* needs the radio to be re-initialised */
#define F_HEX   2u      /* shown and accepted as 0xNN */
#define F_BW    4u      /* only the LoRa bandwidths */
#define F_CALL  8u      /* a callsign: also accepts -SSID */
#define F_CR    16u     /* LoRa coding rate: stored as 5 to 8, shown and accepted as 4/5 to 4/8 */

typedef struct {
    const char *name;
    uint8_t  kind, size, scale, flags, cs;
    uint16_t off;
    int32_t  min, max;      /* numbers: the range; text: the length range */
    const char *unit;
} field_t;

#define OFS(m) ((uint16_t)offsetof(d3_settings_t, m))
#define SZ(m)  ((uint8_t)sizeof(((d3_settings_t *)0)->m))
#define NUMF(n, m, sc, fl, lo, hi, u) { n, K_NUM, SZ(m), sc, fl, 0, OFS(m), lo, hi, u }
#define BOOLF(n, m)                   { n, K_BOOL, SZ(m), 0, 0, 0, OFS(m), 0, 1, "" }
#define STRF(n, m, cs, fl, lo, hi)    { n, K_STR, SZ(m), 0, fl, cs, OFS(m), lo, hi, "chars" }

/* The id of a setting in the flash record is its position here. ONLY APPEND. */
static const field_t FIELDS[] = {
    /*  0 */ STRF ("call", call, CS_CALL, F_CALL, 1, 6),
    /*  1 */ NUMF ("ssid", ssid, 0, 0, 0, 15, ""),
    /*  2 */ { "mode", K_MODE, SZ(mode), 0, 0, 0, OFS(mode), D3_MODE_FILLIN, D3_MODE_WIDE, "fillin|wide" },
    /*  3 */ NUMF ("delay_min", delay_min_ms, 0, 0, 0, 10000, "ms"),
    /*  4 */ NUMF ("delay_max", delay_max_ms, 0, 0, 0, 10000, "ms"),
    /*  5 */ NUMF ("cad_difs", cad_difs, 0, 0, 1, 10, "checks"),
    /*  6 */ NUMF ("cad_backoff", cad_backoff, 0, 0, 1, 20, "slots"),
    /*  7 */ NUMF ("cad_wait", cad_wait_s, 0, 0, 1, 120, "s"),
    /*  8 */ BOOLF("cancel", cancel_on_hear),
    /*  9 */ NUMF ("dedupe", dedupe_s, 0, 0, 1, 3600, "s"),
    /* 10 */ BOOLF("moving", moving),
    /* 11 */ NUMF ("move_m", move_m, 0, 0, 1, 100000, "m"),
    /* 12 */ NUMF ("stationary_min", stationary_min, 0, 0, 0, 10080, "min"),
    /* 13 */ BOOLF("mv_messages", mv_messages),
    /* 14 */ BOOLF("mv_other", mv_other),
    /* 15 */ BOOLF("mv_unreadable", mv_unreadable),
    /* 16 */ STRF ("always_tocalls", always_tocalls, CS_TOCALLS, 0, 0, 39),
    /* 17 */ BOOLF("tx_lacf3", tx_lacf3),
    /* 18 */ BOOLF("log", log),
    /* 19 */ BOOLF("log_decoded", log_decoded),
    /* 20 */ NUMF ("lat", lat_u, 6, 0, -90000000, 90000000, "deg"),
    /* 21 */ NUMF ("lon", lon_u, 6, 0, -180000000, 180000000, "deg"),
    /* 22 */ STRF ("sym_table", sym_table, CS_SYM, 0, 1, 1),
    /* 23 */ STRF ("sym_code", sym_code, CS_SYM, 0, 1, 1),
    /* 24 */ STRF ("comment", comment, CS_TEXT, 0, 0, 43),
    /* 25 */ STRF ("path", path, CS_PATH, 0, 0, 23),
    /* 26 */ STRF ("tocall", tocall, CS_CALL, 0, 1, 6),
    /* 27 */ NUMF ("beacon_min", beacon_min, 0, 0, 1, 1440, "min"),
    /* 28 */ NUMF ("beacon_first", beacon_first_s, 0, 0, 0, 3600, "s"),
    /* 29 */ NUMF ("stats_min", stats_min, 0, 0, 0, 1440, "min"),
    /* 30 */ NUMF ("freq", freq_hz, 6, F_RADIO, 410000000, 510000000, "MHz"),
    /* 31 */ NUMF ("bw", bw_hz, 3, F_RADIO | F_BW, 7800, 500000, "kHz"),
    /* 32 */ NUMF ("sf", sf, 0, F_RADIO, 5, 12, ""),
    /* 33 */ NUMF ("cr", cr, 0, F_RADIO | F_CR, 5, 8, ""),
    /* 34 */ NUMF ("power", power_dbm, 0, F_RADIO, 0, 22, "dBm"),
    /* 35 */ NUMF ("sync", sync, 0, F_RADIO | F_HEX, 0, 255, ""),
    /* 36 */ NUMF ("preamble", preamble, 0, F_RADIO, 6, 255, "symbols"),
    /* 37 */ BOOLF("boosted", boosted),
};
#define NFIELDS ((int)(sizeof FIELDS / sizeof FIELDS[0]))

static const int32_t BW_HZ[] = { 7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000 };

/* ---- defaults ---- */
static void put(char *dst, size_t cap, const char *src)
{
    size_t i;
    memset(dst, 0, cap);
    for (i = 0; i + 1 < cap && src[i]; i++) dst[i] = (char)toupper((unsigned char)src[i]);
}

void d3s_defaults(d3_settings_t *s)
{
    memset(s, 0, sizeof *s);
    put(s->call, sizeof s->call, MY_CALLSIGN);
    s->ssid = MY_SSID;
    s->mode = DIGI_MODE;
    s->delay_min_ms = DIGI_DELAY_MIN_MS; s->delay_max_ms = DIGI_DELAY_MAX_MS;
    s->cad_difs = CAD_DIFS_SLOTS; s->cad_backoff = CAD_BACKOFF_MAX; s->cad_wait_s = CAD_MAX_WAIT_S;
    s->cancel_on_hear = CANCEL_ON_HEAR;
    s->dedupe_s = DEDUPE_SECONDS;
    s->moving = DIGI_ONLY_MOVING; s->move_m = MOVE_THRESHOLD_M; s->stationary_min = STATIONARY_REPEAT_MIN;
    s->mv_messages = MOVING_REPEAT_MESSAGES; s->mv_other = MOVING_REPEAT_OTHER; s->mv_unreadable = MOVING_REPEAT_UNREADABLE;
    strncpy(s->always_tocalls, ALWAYS_REPEAT_TOCALLS, sizeof s->always_tocalls - 1);
    s->tx_lacf3 = TX_FORMAT_LACF3; s->log = 1; s->log_decoded = LOG_DECODED;
    s->stats_min = STATS_EVERY_MIN;
    s->lat_u = (int32_t)(BEACON_LAT * 1e6 + (BEACON_LAT < 0 ? -0.5 : 0.5));
    s->lon_u = (int32_t)(BEACON_LON * 1e6 + (BEACON_LON < 0 ? -0.5 : 0.5));
    s->sym_table[0] = BEACON_SYMBOL_TABLE; s->sym_code[0] = BEACON_SYMBOL_CODE;
    strncpy(s->comment, BEACON_COMMENT, sizeof s->comment - 1);
    strncpy(s->path, BEACON_PATH, sizeof s->path - 1);
    put(s->tocall, sizeof s->tocall, APRS_DESTINATION);
    s->beacon_min = BEACON_INTERVAL_MIN; s->beacon_first_s = BEACON_FIRST_S;
    s->freq_hz = (uint32_t)(LORA_FREQ * 1e6 + 0.5); s->bw_hz = (uint32_t)(LORA_BW * 1000.0 + 0.5);
    s->sf = LORA_SF; s->cr = LORA_CR; s->power_dbm = LORA_TX_POWER; s->sync = LORA_SYNC_WORD; s->preamble = LORA_PREAMBLE;
    s->boosted = RX_BOOSTED_GAIN;
}

void d3s_fullcall(const d3_settings_t *s, char *out, size_t cap)
{
    if (s->ssid) snprintf(out, cap, "%s-%d", s->call, s->ssid); else snprintf(out, cap, "%s", s->call);
}

/* ---- field access ---- */
static int32_t get_i(const d3_settings_t *s, const field_t *f)
{
    uint32_t u = 0;
    memcpy(&u, (const uint8_t *)s + f->off, f->size);        /* little endian on both the STM32 and the PC */
    return (int32_t)u;
}

static void set_i(d3_settings_t *s, const field_t *f, int32_t v)
{
    uint32_t u = (uint32_t)v;
    memcpy((uint8_t *)s + f->off, &u, f->size);
}

static char *str_at(d3_settings_t *s, const field_t *f) { return (char *)s + f->off; }
static const char *cstr_at(const d3_settings_t *s, const field_t *f) { return (const char *)s + f->off; }

static int same(const d3_settings_t *a, const d3_settings_t *b, const field_t *f)
{
    return memcmp((const uint8_t *)a + f->off, (const uint8_t *)b + f->off, f->size) == 0;
}

/* ---- checks ---- */
static int cs_ok(int cs, unsigned char c)
{
    switch (cs) {
    case CS_CALL:    return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
    case CS_PATH:    return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ',' || c == '-' || c == '*';
    case CS_TOCALLS: return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == ',';
    case CS_TEXT:    return c >= 0x20 && c <= 0x7D && c != '|';           /* APRS comments may not hold | or ~ */
    case CS_SYM:     return c >= 0x21 && c <= 0x7E;
    }
    return 0;
}

static const char *check_num(const field_t *f, int32_t v)
{
    int i;
    if (v < f->min || v > f->max) return "out of range";
    if (f->flags & F_BW) {
        for (i = 0; i < (int)(sizeof BW_HZ / sizeof BW_HZ[0]); i++) if (BW_HZ[i] == v) return NULL;
        return "not a LoRa bandwidth (7.8 10.4 15.6 20.8 31.25 41.7 62.5 125 250 500 kHz)";
    }
    return NULL;
}

/* text: upper-cases the call/path kinds in place, then checks length and characters */
static const char *check_str(const field_t *f, char *v)
{
    size_t n = strlen(v), i;
    if (f->cs != CS_TEXT && f->cs != CS_SYM) for (i = 0; i < n; i++) v[i] = (char)toupper((unsigned char)v[i]);
    if ((int32_t)n < f->min || (int32_t)n > f->max) return "wrong length";
    for (i = 0; i < n; i++) if (!cs_ok(f->cs, (unsigned char)v[i])) return "character not allowed";
    return NULL;
}

/* ---- number text ---- */
static const uint32_t POW10[] = { 1u, 10u, 100u, 1000u, 10000u, 100000u, 1000000u };

static const char *parse_num(const char *s, uint8_t scale, int hex_ok, int32_t *out)
{
    int neg = 0, seen = 0, frac = 0, dot = 0;
    int64_t v = 0;
    if (hex_ok && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        const char *p = s + 2;
        if (!*p) return "not a number";
        for (; *p; p++) {
            int d = isdigit((unsigned char)*p) ? *p - '0' : (tolower((unsigned char)*p) >= 'a' && tolower((unsigned char)*p) <= 'f') ? tolower((unsigned char)*p) - 'a' + 10 : -1;
            if (d < 0) return "not a number";
            v = v * 16 + d;
            if (v > 0x7FFFFFFF) return "out of range";
        }
        *out = (int32_t)v;
        return NULL;
    }
    if (*s == '-') { neg = 1; s++; } else if (*s == '+') s++;
    for (; *s; s++) {
        if (*s == '.' && !dot) { dot = 1; continue; }
        if (!isdigit((unsigned char)*s)) return "not a number";
        seen = 1;
        if (dot) { if (frac == scale) return "too many decimals"; frac++; }
        v = v * 10 + (*s - '0');
        if (v > 4000000000000LL) return "out of range";
    }
    if (!seen) return "not a number";
    v *= POW10[scale - frac];
    if (v > 0x7FFFFFFF) return "out of range";
    *out = (int32_t)(neg ? -v : v);
    return NULL;
}

static void fmt_num(char *o, size_t cap, int32_t v, uint8_t scale, int hex)
{
    uint32_t a, ip, fp; char frac[8]; int n, i;
    if (hex) { snprintf(o, cap, "0x%02lX", (unsigned long)(uint32_t)v); return; }
    a = v < 0 ? 0u - (uint32_t)v : (uint32_t)v;
    ip = a / POW10[scale]; fp = a % POW10[scale];
    n = snprintf(o, cap, "%s%lu", v < 0 ? "-" : "", (unsigned long)ip);
    if (scale && fp && n > 0 && (size_t)n < cap) {
        for (i = scale; i > 0; i--) { frac[i - 1] = (char)('0' + fp % 10); fp /= 10; }
        frac[scale] = 0;
        for (i = scale; i > 0 && frac[i - 1] == '0'; i--) frac[i - 1] = 0;
        snprintf(o + n, cap - (size_t)n, ".%s", frac);
    }
}

static void fmt_value(const d3_settings_t *s, const field_t *f, char *o, size_t cap)
{
    switch (f->kind) {
    case K_BOOL: snprintf(o, cap, "%s", get_i(s, f) ? "on" : "off"); break;
    case K_MODE: snprintf(o, cap, "%s", get_i(s, f) == D3_MODE_WIDE ? "wide" : "fillin"); break;
    case K_STR:  snprintf(o, cap, "%s", cstr_at(s, f)[0] ? cstr_at(s, f) : "-"); break;
    default:
        if (f->flags & F_CR) snprintf(o, cap, "4/%ld", (long)get_i(s, f));
        else fmt_num(o, cap, get_i(s, f), f->scale, (f->flags & F_HEX) != 0);
        break;
    }
}

/* ---- reply lines ---- */
static void say(d3s_emit_t emit, void *ctx, const char *fmt, ...)
{
    char b[160]; va_list ap;
    va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    if (emit) emit(ctx, b);
}

static int ieq(const char *a, const char *b)
{
    for (; *a && *b; a++, b++) if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
    return *a == *b;
}

static const field_t *find(const char *name)
{
    int i;
    for (i = 0; i < NFIELDS; i++) if (ieq(name, FIELDS[i].name)) return &FIELDS[i];
    return NULL;
}

static void show_one(const d3_settings_t *s, const field_t *f, const char *prefix, d3s_emit_t emit, void *ctx)
{
    char v[64];
    fmt_value(s, f, v, sizeof v);
    say(emit, ctx, "%s%s = %s%s%s", prefix, f->name, v, f->kind == K_NUM && f->unit[0] ? " " : "", f->kind == K_NUM ? f->unit : "");
}

/* A value typed by the user -> the setting, or an error text. Works on a copy so a refused value changes nothing. */
static const char *set_field(d3_settings_t *t, const field_t *f, char *val)
{
    const char *err;
    if (f->kind == K_STR) {
        char *dst = str_at(t, f);
        if (strcmp(val, "-") == 0) val[0] = 0;                       /* "-" = empty */
        if (f->flags & F_CALL && f == &FIELDS[0]) {                    /* the callsign may carry -SSID */
            char *h = strchr(val, '-');
            if (h) {
                int32_t ss; const char *e;
                *h = 0;
                e = parse_num(h + 1, 0, 0, &ss);
                if (e) return "bad SSID";
                if (ss < 0 || ss > 15) return "SSID must be 0 to 15";
                t->ssid = (uint8_t)ss;
            }
        }
        err = check_str(f, val);
        if (err) return err;
        memset(dst, 0, f->size);
        memcpy(dst, val, strlen(val));
        return NULL;
    }
    if (f->kind == K_BOOL) {
        if (ieq(val, "on") || ieq(val, "1") || ieq(val, "yes") || ieq(val, "true")) set_i(t, f, 1);
        else if (ieq(val, "off") || ieq(val, "0") || ieq(val, "no") || ieq(val, "false")) set_i(t, f, 0);
        else return "use on or off";
        return NULL;
    }
    if (f->kind == K_MODE) {
        if (ieq(val, "wide")) set_i(t, f, D3_MODE_WIDE);
        else if (ieq(val, "fillin") || ieq(val, "fill-in") || ieq(val, "fill")) set_i(t, f, D3_MODE_FILLIN);
        else return "use fillin or wide";
        return NULL;
    }
    {
        int32_t v;
        if ((f->flags & F_CR) && val[0] == '4' && val[1] == '/') val += 2;          /* 4/5 means 5 */
        err = parse_num(val, f->scale, (f->flags & F_HEX) != 0, &v);
        if (err) return err;
        err = check_num(f, v);
        if (err) return err;
        set_i(t, f, v);
    }
    return NULL;
}

static const char *cross_check(const d3_settings_t *t)
{
    if (t->delay_min_ms > t->delay_max_ms) return "delay_min must not be above delay_max";
    return NULL;
}

/* ---- the command line ---- */
void d3s_menu(d3s_emit_t emit, void *ctx)
{
    say(emit, ctx, "Commands (type one and press Enter):");
    say(emit, ctx, "  show                 every setting and its value        get NAME      one setting");
    say(emit, ctx, "  set NAME VALUE       change a setting, e.g.  set call K6ATV-2   (a text value of - means empty)");
    say(emit, ctx, "  save                 keep the changes after a power-off  defaults      load the compiled-in values");
    say(emit, ctx, "  apply                use changed radio settings (freq, bw, sf, cr, power, sync, preamble, boosted)");
    say(emit, ctx, "  stats  beacon  reboot [force]  help (lists every setting and its range)");
    say(emit, ctx, "Changes work at once, except the radio settings, which need apply.");
}

unsigned d3s_exec(d3_settings_t *s, const char *line, d3s_emit_t emit, void *ctx)
{
    char buf[D3S_LINE_MAX + 1], *cmd, *arg, *rest, *e;
    unsigned fl = 0;
    int i;
    size_t n = strlen(line);

    if (n > D3S_LINE_MAX) { say(emit, ctx, "error: line too long"); return 0; }
    memcpy(buf, line, n + 1);
    for (cmd = buf; *cmd == ' ' || *cmd == '\t'; cmd++) {}
    for (e = cmd + strlen(cmd); e > cmd && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r' || e[-1] == '\n'); e--) {}
    *e = 0;
    if (!*cmd) return 0;
    for (arg = cmd; *arg && *arg != ' ' && *arg != '\t'; arg++) {}
    if (*arg) { *arg++ = 0; while (*arg == ' ' || *arg == '\t') arg++; }

    if (ieq(cmd, "help") || ieq(cmd, "?")) {
        d3s_menu(emit, ctx);
        for (i = 0; i < NFIELDS; i++) {
            const field_t *f = &FIELDS[i];
            char lo[16], hi[16];
            if (f->flags & F_CR) say(emit, ctx, "  %-15s 4/5 to 4/8 (a plain 5 to 8 also works) (radio)", f->name);
            else if (f->kind == K_NUM) { fmt_num(lo, sizeof lo, f->min, f->scale, 0); fmt_num(hi, sizeof hi, f->max, f->scale, 0); say(emit, ctx, "  %-15s %s to %s %s%s", f->name, lo, hi, f->unit, f->flags & F_RADIO ? " (radio)" : ""); }
            else if (f->kind == K_STR) say(emit, ctx, "  %-15s text, %d to %d characters", f->name, (int)f->min, (int)f->max);
            else if (f->kind == K_MODE) say(emit, ctx, "  %-15s fillin or wide", f->name);
            else say(emit, ctx, "  %-15s on or off", f->name);
        }
    } else if (ieq(cmd, "show")) {
        char fc[16]; d3s_fullcall(s, fc, sizeof fc);
        say(emit, ctx, "callsign %s", fc);
        for (i = 0; i < NFIELDS; i++) show_one(s, &FIELDS[i], "", emit, ctx);
    } else if (ieq(cmd, "get")) {
        const field_t *f = *arg ? find(arg) : NULL;
        if (!f) say(emit, ctx, "error: get NAME (see help)"); else show_one(s, f, "", emit, ctx);
    } else if (ieq(cmd, "set")) {
        const field_t *f; d3_settings_t t = *s; const char *err;
        for (rest = arg; *rest && *rest != ' ' && *rest != '\t'; rest++) {}
        if (*rest) { *rest++ = 0; while (*rest == ' ' || *rest == '\t') rest++; }
        f = *arg ? find(arg) : NULL;
        if (!f) { say(emit, ctx, "error: unknown setting \"%s\" (see help)", arg); return 0; }
        if (!*rest) { say(emit, ctx, "error: set %s VALUE", f->name); return 0; }
        if (f->kind != K_STR && strpbrk(rest, " \t")) { say(emit, ctx, "error: %s: one value only", f->name); return 0; }
        err = set_field(&t, f, rest);
        if (!err) err = cross_check(&t);
        if (err) { say(emit, ctx, "error: %s: %s", f->name, err); return 0; }
        /* what really changed (the callsign can change two settings) */
        for (i = 0; i < NFIELDS; i++) {
            if (same(s, &t, &FIELDS[i])) continue;
            fl |= D3S_CHANGED;
            if (FIELDS[i].flags & F_RADIO) fl |= D3S_RADIO;
        }
        *s = t;
        if (f == &FIELDS[0]) { char fc[16]; d3s_fullcall(s, fc, sizeof fc); say(emit, ctx, "ok: callsign = %s", fc); }
        else show_one(s, f, "ok: ", emit, ctx);
        if (!(fl & D3S_CHANGED)) say(emit, ctx, "(no change)");
        else if (fl & D3S_RADIO) say(emit, ctx, "(radio setting: type apply to use it)");
    } else if (ieq(cmd, "defaults")) {
        d3s_defaults(s);
        fl = D3S_CHANGED | D3S_RADIO | D3S_DEFAULTS;
        say(emit, ctx, "ok: compiled-in defaults loaded (type apply, then save to keep them)");
    } else if (ieq(cmd, "save"))   fl = D3S_SAVE;
    else if (ieq(cmd, "apply"))    fl = D3S_APPLY;
    else if (ieq(cmd, "stats"))    fl = D3S_STATS;
    else if (ieq(cmd, "beacon"))   fl = D3S_BEACON;
    else if (ieq(cmd, "reboot") || ieq(cmd, "reset")) { fl = D3S_REBOOT; if (ieq(arg, "force")) fl |= D3S_FORCE; }
    else say(emit, ctx, "error: unknown command \"%s\" (type help)", cmd);
    return fl;
}

/* ---- flash record ---- */
static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu; size_t i; int b;
    for (i = 0; i < n; i++) { c ^= p[i]; for (b = 0; b < 8; b++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
    return ~c;
}

#define REC_MAGIC0 'D'
#define REC_MAGIC1 '3'
#define REC_MAGIC2 'S'
#define REC_VER    1
#define REC_HDR    6      /* magic(3) version(1) payload length(2) */

size_t d3s_pack(const d3_settings_t *s, uint8_t *out, size_t cap)
{
    size_t w = REC_HDR, total; int i; uint32_t c;
    if (cap < REC_HDR + 4 + 8) return 0;
    for (i = 0; i < NFIELDS; i++) {
        const field_t *f = &FIELDS[i];
        size_t len = f->kind == K_STR ? strlen(cstr_at(s, f)) : 4;
        if (w + 2 + len + 4 + 8 > cap) return 0;
        out[w++] = (uint8_t)i; out[w++] = (uint8_t)len;
        if (f->kind == K_STR) { memcpy(out + w, cstr_at(s, f), len); w += len; }
        else { uint32_t u = (uint32_t)get_i(s, f); out[w++] = (uint8_t)u; out[w++] = (uint8_t)(u >> 8); out[w++] = (uint8_t)(u >> 16); out[w++] = (uint8_t)(u >> 24); }
    }
    out[0] = REC_MAGIC0; out[1] = REC_MAGIC1; out[2] = REC_MAGIC2; out[3] = REC_VER;
    out[4] = (uint8_t)(w - REC_HDR); out[5] = (uint8_t)((w - REC_HDR) >> 8);
    c = crc32(out, w);
    out[w++] = (uint8_t)c; out[w++] = (uint8_t)(c >> 8); out[w++] = (uint8_t)(c >> 16); out[w++] = (uint8_t)(c >> 24);
    total = (w + 7u) & ~(size_t)7u;
    if (total > cap) return 0;
    memset(out + w, 0xFF, total - w);
    return total;
}

int d3s_unpack(const uint8_t *in, size_t n, d3_settings_t *s, int *rejected)
{
    size_t len, p, end; int applied = 0, rej = 0; uint32_t c;
    d3s_defaults(s);
    if (rejected) *rejected = 0;
    if (n < REC_HDR + 4) return D3S_E_BLANK;
    if (in[0] == 0xFF && in[1] == 0xFF && in[2] == 0xFF && in[3] == 0xFF) return D3S_E_BLANK;
    if (in[0] != REC_MAGIC0 || in[1] != REC_MAGIC1 || in[2] != REC_MAGIC2 || in[3] != REC_VER) return D3S_E_FORMAT;
    len = (size_t)in[4] | ((size_t)in[5] << 8);
    if (REC_HDR + len + 4 > n) return D3S_E_FORMAT;
    end = REC_HDR + len;
    c = (uint32_t)in[end] | ((uint32_t)in[end + 1] << 8) | ((uint32_t)in[end + 2] << 16) | ((uint32_t)in[end + 3] << 24);
    if (crc32(in, end) != c) return D3S_E_CRC;
    for (p = REC_HDR; p + 2 <= end;) {
        unsigned id = in[p], l = in[p + 1];
        const uint8_t *v = in + p + 2;
        p += 2u + l;
        if (p > end) break;
        if ((int)id >= NFIELDS) continue;                              /* a setting a newer firmware added */
        {
            const field_t *f = &FIELDS[id];
            d3_settings_t t = *s; const char *err = NULL;
            if (f->kind == K_STR) {
                char tmp[48];
                if (l >= sizeof tmp || l >= f->size) { rej++; continue; }
                memcpy(tmp, v, l); tmp[l] = 0;
                err = check_str(f, tmp);
                if (!err) { memset(str_at(&t, f), 0, f->size); memcpy(str_at(&t, f), tmp, l); }
            } else {
                int32_t x;
                if (l != 4) { rej++; continue; }
                x = (int32_t)((uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24));
                if (f->kind == K_NUM) err = check_num(f, x);
                else if (f->kind == K_BOOL) err = (x == 0 || x == 1) ? NULL : "bad";
                else err = (x == D3_MODE_FILLIN || x == D3_MODE_WIDE) ? NULL : "bad";
                if (!err) set_i(&t, f, x);
            }
            if (err) { rej++; continue; }
            *s = t; applied++;
        }
    }
    if (cross_check(s)) { s->delay_min_ms = DIGI_DELAY_MIN_MS; s->delay_max_ms = DIGI_DELAY_MAX_MS; rej++; }   /* the pair together, after both are read */
    if (rejected) *rejected = rej;
    return applied;
}
