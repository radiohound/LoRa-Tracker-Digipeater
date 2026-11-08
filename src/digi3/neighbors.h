/* Stations heard: where each one last was, and how well this digipeater hears it. Used by the "moving stations only" mode, and printed as a
 * neighbour list on the serial port. Plain C, no Arduino code, so it is tested on a PC with the rest of the core.
 *
 * Every packet the digipeater can read (plain text, or a LACF 3.0 frame decoded with the model) updates the table:
 *   - its source: last position (if the packet has one), when it was last heard
 *   - the station it was heard from (the source when heard directly, otherwise the last digipeater in the path): signal strength and SNR,
 *     so the table also says which stations are this digipeater's RF neighbours.
 *
 * "Moving stations only" (d3_moving_decision) repeats:
 *   - a packet whose next hop names this digipeater (MYCALL in the path): always
 *   - a packet whose destination (tocall) starts with one of the always-repeat prefixes
 *   - a position from a station not repeated before, or that has moved MOVE_THRESHOLD_M or more since the position this digi last repeated
 *   - optionally: a fixed station again after STATIONARY_REPEAT_MIN minutes; messages; other packets without a position; frames it cannot read
 * and skips a fixed station heard again in the same place. */
#ifndef NEIGHBORS_H
#define NEIGHBORS_H
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char     call[10];            /* empty = free slot */
    uint32_t heard_ms;            /* last time a packet from it was heard (any path) */
    int32_t  lat, lon;            /* last position, microdegrees */
    uint32_t pos_ms;
    int32_t  rep_lat, rep_lon;    /* the position this digi last repeated for it */
    uint32_t rep_ms;
    uint32_t rf_ms;               /* last time this digi heard it on the air itself (direct, or as the digipeater of someone else's packet) */
    int16_t  rssi10, snr10;       /* that reception, in tenths */
    uint16_t packets, rf_frames;
    uint8_t  has_pos, has_rep, direct_src;   /* direct_src: its own packets were heard directly at least once */
} d3_station_t;

typedef struct {
    d3_station_t *s;
    int n;
} d3_neighbors_t;

typedef struct {
    uint32_t    move_threshold_m;     /* a position this far from the last one repeated counts as moved */
    uint32_t    stationary_repeat_min;/* also repeat a fixed station after this long (0 = never) */
    uint8_t     repeat_messages;      /* messages and acks (no position) */
    uint8_t     repeat_other;         /* status, telemetry, weather without a position, objects that cannot be read... */
    uint8_t     repeat_unreadable;    /* frames this digi cannot decode (a LACF model it does not hold) */
    const char *always_tocalls;       /* comma-separated destination prefixes always repeated, e.g. "APLAIX,APLAB"; "" = none */
} d3_moving_cfg_t;

/* Position of an APRS information field: uncompressed, compressed (base 91) and Mic-E (dest is then needed). Returns 1 and microdegrees. */
int d3_parse_position(const char *dest, size_t dlen, const char *info, size_t ilen, int32_t *lat, int32_t *lon);
uint32_t d3_distance_m(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);

void d3_neighbors_init(d3_neighbors_t *nb, d3_station_t *slots, int n);
d3_station_t *d3_neighbor_find(d3_neighbors_t *nb, const char *call, size_t len);

/* Update the table from one packet heard (TNC2 text, the path as received), with this reception's signal. */
void d3_observe(d3_neighbors_t *nb, const char *tnc2, size_t len, float rssi, float snr, uint32_t now_ms);

/* For "moving stations only": should this packet (TNC2 text as received; NULL if unreadable) be repeated? named = its next hop is MYCALL.
 * Returns D3_MOVE_* and, on a repeat of a position, records it as the last position repeated. */
enum { D3_MOVE_REPEAT_NAMED = 1, D3_MOVE_REPEAT_TOCALL, D3_MOVE_REPEAT_NEW, D3_MOVE_REPEAT_MOVED, D3_MOVE_REPEAT_REFRESH, D3_MOVE_REPEAT_MESSAGE,
       D3_MOVE_REPEAT_OTHER, D3_MOVE_REPEAT_UNREADABLE, D3_MOVE_SKIP_STATIONARY, D3_MOVE_SKIP_MESSAGE, D3_MOVE_SKIP_OTHER, D3_MOVE_SKIP_UNREADABLE };
int d3_moving_decision(d3_neighbors_t *nb, const d3_moving_cfg_t *cfg, const char *tnc2, size_t len, int named, uint32_t now_ms);
int d3_move_repeats(int decision);
const char *d3_move_reason(int decision);

#ifdef __cplusplus
}
#endif
#endif
