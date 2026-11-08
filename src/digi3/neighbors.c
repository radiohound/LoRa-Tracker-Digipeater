/* Stations heard and "moving stations only". See neighbors.h. */
#include "neighbors.h"
#include <ctype.h>
#include <math.h>
#include <string.h>

/* ---- TNC2 parts ---- */
typedef struct { const char *src, *dest, *path, *info; size_t slen, dlen, plen, ilen; } parts_t;

static int split(const char *t, size_t n, parts_t *p)
{
    size_t i, colon = n, gt = n, de;
    for (i = 0; i < n; i++) if (t[i] == ':') { colon = i; break; }
    if (colon == n) return 0;
    for (i = 0; i < colon; i++) if (t[i] == '>') { gt = i; break; }
    if (gt == n || gt == 0) return 0;
    for (de = gt + 1; de < colon && t[de] != ','; de++) ;
    p->src = t; p->slen = gt;
    p->dest = t + gt + 1; p->dlen = de - gt - 1;
    p->path = t + de + (de < colon); p->plen = de < colon ? colon - de - 1 : 0;
    p->info = t + colon + 1; p->ilen = n - colon - 1;
    return p->dlen > 0;
}

static int ieqn(const char *a, size_t an, const char *b, size_t bn)
{
    size_t i;
    if (an != bn) return 0;
    for (i = 0; i < an; i++) if (toupper((unsigned char)a[i]) != toupper((unsigned char)b[i])) return 0;
    return 1;
}

/* ---- positions ---- */
static int digits(const char *s, int n, int *v)     /* digits, a space (position ambiguity) counts as 0 */
{
    int i, x = 0;
    for (i = 0; i < n; i++) {
        if (isdigit((unsigned char)s[i])) x = x * 10 + (s[i] - '0');
        else if (s[i] == ' ') x = x * 10;
        else return 0;
    }
    *v = x;
    return 1;
}

static int32_t udeg(int deg, int min, int hund) { return (int32_t)(((int64_t)deg * 6000 + min * 100 + hund) * 1000000 / 6000); }

static int uncompressed(const char *s, size_t n, int32_t *lat, int32_t *lon)
{
    int ld, lm, lh, od, om, oh;
    if (n < 19) return 0;
    if (!digits(s, 2, &ld) || !digits(s + 2, 2, &lm) || s[4] != '.' || !digits(s + 5, 2, &lh) || (s[7] != 'N' && s[7] != 'S')) return 0;
    if (!digits(s + 9, 3, &od) || !digits(s + 12, 2, &om) || s[14] != '.' || !digits(s + 15, 2, &oh) || (s[17] != 'E' && s[17] != 'W')) return 0;
    if (ld > 90 || lm > 59 || od > 180 || om > 59) return 0;
    *lat = udeg(ld, lm, lh); if (s[7] == 'S') *lat = -*lat;
    *lon = udeg(od, om, oh); if (s[17] == 'W') *lon = -*lon;
    return 1;
}

static int compressed(const char *s, size_t n, int32_t *lat, int32_t *lon)
{
    uint32_t a = 0, o = 0; int i; char t = s[0];
    if (n < 10) return 0;
    if (!(t == '/' || t == '\\' || (t >= 'A' && t <= 'Z') || (t >= 'a' && t <= 'j'))) return 0;
    for (i = 1; i <= 8; i++) if (s[i] < 33 || s[i] > 124) return 0;
    for (i = 1; i <= 4; i++) a = a * 91 + (uint32_t)(s[i] - 33);
    for (i = 5; i <= 8; i++) o = o * 91 + (uint32_t)(s[i] - 33);
    *lat = (int32_t)(90000000LL - (int64_t)a * 1000000 / 380926);
    *lon = (int32_t)(-180000000LL + (int64_t)o * 1000000 / 190463);
    return *lat >= -90000000 && *lat <= 90000000 && *lon >= -180000000 && *lon <= 180000000;
}

static int mic_e(const char *dest, size_t dlen, const char *info, size_t ilen, int32_t *lat, int32_t *lon)
{
    int d[6], i, deg, min, hund;
    if (dlen < 6 || ilen < 4) return 0;
    for (i = 0; i < 6; i++) {
        char c = dest[i];
        if (c >= '0' && c <= '9') d[i] = c - '0';
        else if (c >= 'A' && c <= 'J') d[i] = c - 'A';
        else if (c >= 'P' && c <= 'Y') d[i] = c - 'P';
        else if (c == 'K' || c == 'L' || c == 'Z') d[i] = 0;
        else return 0;
    }
    deg = d[0] * 10 + d[1]; min = d[2] * 10 + d[3]; hund = d[4] * 10 + d[5];
    if (deg > 90 || min > 59) return 0;
    *lat = udeg(deg, min, hund);
    if (!(dest[3] >= 'P' && dest[3] <= 'Z')) *lat = -*lat;
    deg = (unsigned char)info[1] - 28; if (dest[4] >= 'P' && dest[4] <= 'Z') deg += 100;
    if (deg >= 180 && deg <= 189) deg -= 80; else if (deg >= 190 && deg <= 199) deg -= 190;
    min = (unsigned char)info[2] - 28; if (min >= 60) min -= 60;
    hund = (unsigned char)info[3] - 28;
    if (deg < 0 || deg > 180 || min < 0 || min > 59 || hund < 0 || hund > 99) return 0;
    *lon = udeg(deg, min, hund);
    if (dest[5] >= 'P' && dest[5] <= 'Z') *lon = -*lon;
    return 1;
}

int d3_parse_position(const char *dest, size_t dlen, const char *info, size_t ilen, int32_t *lat, int32_t *lon)
{
    size_t p; char c;
    if (!ilen) return 0;
    c = info[0];
    if (c == '`' || c == '\'' || c == 0x1c || c == 0x1d) {
        size_t k; for (k = 0; k < dlen && dest[k] != '-'; k++) ;          /* the destination without its SSID */
        return mic_e(dest, k, info, ilen, lat, lon);
    }
    if (c == '!' || c == '=') p = 1;
    else if (c == '/' || c == '@') p = 8;                                  /* a 7-character time stamp first */
    else return 0;
    if (ilen <= p) return 0;
    return isdigit((unsigned char)info[p]) ? uncompressed(info + p, ilen - p, lat, lon) : compressed(info + p, ilen - p, lat, lon);
}

uint32_t d3_distance_m(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
    float mid = (float)(lat1 + lat2) * 0.5e-6f * 0.01745329f;
    float dy = (float)(lat2 - lat1) * 0.11132f;                           /* microdegrees to metres: 111,320 m per degree */
    float dx = (float)(lon2 - lon1) * 0.11132f * cosf(mid);
    return (uint32_t)sqrtf(dx * dx + dy * dy);
}

/* ---- the table ---- */
void d3_neighbors_init(d3_neighbors_t *nb, d3_station_t *slots, int n)
{
    nb->s = slots; nb->n = n;
    memset(slots, 0, sizeof *slots * (size_t)n);
}

d3_station_t *d3_neighbor_find(d3_neighbors_t *nb, const char *call, size_t len)
{
    int i;
    for (i = 0; i < nb->n; i++) if (nb->s[i].call[0] && ieqn(nb->s[i].call, strlen(nb->s[i].call), call, len)) return &nb->s[i];
    return 0;
}

static d3_station_t *get(d3_neighbors_t *nb, const char *call, size_t len, uint32_t now)
{
    d3_station_t *s = d3_neighbor_find(nb, call, len); int i, oldest = 0; uint32_t age = 0;
    if (s) return s;
    if (!len || len >= sizeof s->call) return 0;
    for (i = 0; i < nb->n; i++) {                                         /* a free slot, or the station heard longest ago */
        uint32_t last = nb->s[i].heard_ms > nb->s[i].rf_ms ? nb->s[i].heard_ms : nb->s[i].rf_ms;
        if (!nb->s[i].call[0]) { oldest = i; break; }
        if (now - last >= age) { age = now - last; oldest = i; }
    }
    s = &nb->s[oldest];
    memset(s, 0, sizeof *s);
    for (i = 0; i < (int)len; i++) s->call[i] = (char)toupper((unsigned char)call[i]);
    return s;
}

static int is_alias(const char *h, size_t l)                              /* a used alias says nothing about which station it was */
{
    return (l >= 4 && ieqn(h, 4, "WIDE", 4)) || (l >= 5 && ieqn(h, 5, "RELAY", 5)) || (l >= 5 && ieqn(h, 5, "TRACE", 5)) || (l >= 5 && ieqn(h, 5, "TCPIP", 5));
}

void d3_observe(d3_neighbors_t *nb, const char *tnc2, size_t len, float rssi, float snr, uint32_t now)
{
    parts_t p; d3_station_t *s; const char *from = 0; size_t flen = 0, i = 0; int32_t lat, lon;
    if (!split(tnc2, len, &p)) return;
    s = get(nb, p.src, p.slen, now);
    if (s) {
        s->heard_ms = now ? now : 1; s->packets++;
        if (d3_parse_position(p.dest, p.dlen, p.info, p.ilen, &lat, &lon)) { s->lat = lat; s->lon = lon; s->pos_ms = s->heard_ms; s->has_pos = 1; }
    }
    while (i < p.plen) {                                                  /* the last used hop is the station this digi heard */
        size_t e = i; while (e < p.plen && p.path[e] != ',') e++;
        if (e > i && p.path[e - 1] == '*') { from = p.path + i; flen = e - i - 1; }
        i = e + 1;
    }
    if (!from) { from = p.src; flen = p.slen; if (s) s->direct_src = 1; }
    else if (is_alias(from, flen)) return;
    s = get(nb, from, flen, now);
    if (!s) return;
    s->rf_ms = now ? now : 1; s->rf_frames++;
    s->rssi10 = (int16_t)lroundf(rssi * 10); s->snr10 = (int16_t)lroundf(snr * 10);
}

/* ---- moving stations only ---- */
static int tocall_listed(const char *list, const char *dest, size_t dlen)
{
    while (list && *list) {
        const char *e = strchr(list, ','); size_t l = e ? (size_t)(e - list) : strlen(list);
        while (l && *list == ' ') { list++; l--; }
        while (l && list[l - 1] == ' ') l--;
        if (l && l <= dlen && ieqn(list, l, dest, l)) return 1;
        list = e ? e + 1 : 0;
    }
    return 0;
}

int d3_moving_decision(d3_neighbors_t *nb, const d3_moving_cfg_t *cfg, const char *tnc2, size_t len, int named, uint32_t now)
{
    parts_t p; int32_t lat, lon; d3_station_t *s;
    if (named) return D3_MOVE_REPEAT_NAMED;
    if (!tnc2 || !split(tnc2, len, &p)) return cfg->repeat_unreadable ? D3_MOVE_REPEAT_UNREADABLE : D3_MOVE_SKIP_UNREADABLE;
    if (tocall_listed(cfg->always_tocalls, p.dest, p.dlen)) return D3_MOVE_REPEAT_TOCALL;
    if (!d3_parse_position(p.dest, p.dlen, p.info, p.ilen, &lat, &lon)) {
        if (p.ilen && p.info[0] == ':') return cfg->repeat_messages ? D3_MOVE_REPEAT_MESSAGE : D3_MOVE_SKIP_MESSAGE;
        return cfg->repeat_other ? D3_MOVE_REPEAT_OTHER : D3_MOVE_SKIP_OTHER;
    }
    s = get(nb, p.src, p.slen, now);
    if (!s) return D3_MOVE_REPEAT_NEW;
    {
        int d;
        if (!s->has_rep) d = D3_MOVE_REPEAT_NEW;
        else if (d3_distance_m(s->rep_lat, s->rep_lon, lat, lon) >= cfg->move_threshold_m) d = D3_MOVE_REPEAT_MOVED;
        else if (cfg->stationary_repeat_min && now - s->rep_ms >= cfg->stationary_repeat_min * 60000UL) d = D3_MOVE_REPEAT_REFRESH;
        else return D3_MOVE_SKIP_STATIONARY;
        s->rep_lat = lat; s->rep_lon = lon; s->rep_ms = now ? now : 1; s->has_rep = 1;
        return d;
    }
}

int d3_move_repeats(int d) { return d >= D3_MOVE_REPEAT_NAMED && d <= D3_MOVE_REPEAT_UNREADABLE; }

const char *d3_move_reason(int d)
{
    static const char *const R[] = { "?", "asks for this digi by name", "tocall on the always list", "new station", "moved", "fixed station, refresh",
        "message", "no position", "cannot be read", "fixed station, not moved", "message (not repeated)", "no position (not repeated)", "cannot be read (not repeated)" };
    return (d >= 0 && d < (int)(sizeof R / sizeof R[0])) ? R[d] : "?";
}
