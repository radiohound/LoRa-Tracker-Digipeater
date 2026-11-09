/* Run-time settings for the LACF 3.0 digipeater: the values, the serial command line that changes them, and the record that is kept in flash.
 * Plain C, no Arduino or radio code, so it is tested on a PC with the rest of the core (test/host/test_settings.c).
 *
 * The compiled-in values in config_digi3.h are the DEFAULTS. A saved record in flash overrides them. Each setting is checked against its range
 * when it is typed and again when the record is read back, so a damaged or hand-edited record can never load a value the radio would refuse.
 *
 * The flash record is a list of (id, length, bytes) entries with a CRC, so a newer firmware that adds settings still reads an older record
 * (the new settings take their defaults) and an older firmware skips entries it does not know. Setting ids are the position in the table in
 * settings.c: only ever append to it. */
#ifndef D3_SETTINGS_H
#define D3_SETTINGS_H
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* identity */
    char     call[8];             /* base callsign, 1 to 6 characters; the SSID is separate */
    uint8_t  ssid;                /* 0 to 15; 0 = none */
    /* digipeater */
    uint8_t  mode;                /* D3_MODE_FILLIN or D3_MODE_WIDE */
    uint16_t delay_min_ms, delay_max_ms;
    uint8_t  cad_difs, cad_backoff, cad_wait_s;
    uint8_t  cancel_on_hear;
    uint16_t dedupe_s;
    /* moving stations only */
    uint8_t  moving, mv_messages, mv_other, mv_unreadable;
    uint32_t move_m;
    uint16_t stationary_min;
    char     always_tocalls[40];
    /* output */
    uint8_t  tx_lacf3, log, log_decoded;
    uint16_t stats_min;
    /* beacon */
    int32_t  lat_u, lon_u;        /* microdegrees; both 0 = no beacon */
    char     sym_table[2], sym_code[2];
    char     comment[44];
    char     path[24];
    char     tocall[8];
    uint16_t beacon_min, beacon_first_s;
    /* radio */
    uint32_t freq_hz, bw_hz;
    uint8_t  sf, cr, power_dbm, sync, boosted;
    uint8_t  preamble;
} d3_settings_t;

/* What a command asks the caller to do (bit mask). */
#define D3S_CHANGED   0x001u      /* a setting changed: rebuild what depends on it, remember there is something to save */
#define D3S_RADIO     0x002u      /* a radio setting changed: it takes effect with D3S_APPLY or after a restart */
#define D3S_SAVE      0x004u
#define D3S_APPLY     0x008u      /* re-initialise the radio with the current settings */
#define D3S_REBOOT    0x010u
#define D3S_FORCE     0x020u      /* with D3S_REBOOT: also when there are unsaved changes */
#define D3S_STATS     0x040u
#define D3S_BEACON    0x080u      /* send the beacon now */
#define D3S_DEFAULTS  0x100u      /* the compiled-in defaults were loaded (not saved yet) */

#define D3S_LINE_MAX  120
#define D3S_REC_MAX   1024        /* most bytes a record can take (the flash page is 2048) */

typedef void (*d3s_emit_t)(void *ctx, const char *line);

void d3s_defaults(d3_settings_t *s);

/* Run one command line ("set call K6ATV-2", "show", "save"...). Replies go to emit, one line at a time. Returns D3S_* flags. */
unsigned d3s_exec(d3_settings_t *s, const char *line, d3s_emit_t emit, void *ctx);

/* The list of commands (what "help" starts with); the firmware also prints it at start-up. */
void d3s_menu(d3s_emit_t emit, void *ctx);

/* The full callsign with SSID, "K6ATV-2". */
void d3s_fullcall(const d3_settings_t *s, char *out, size_t cap);

/* The flash record. pack returns its length (a multiple of 8, padded with 0xFF) or 0 if cap is too small.
 * unpack starts from the defaults and applies every valid entry; returns the number applied, or a negative D3S_E_* when the record is not usable.
 * *rejected (may be NULL) counts entries that were skipped because their value was out of range. */
#define D3S_E_BLANK  (-1)         /* no record (erased flash) */
#define D3S_E_FORMAT (-2)         /* not a record this firmware understands */
#define D3S_E_CRC    (-3)         /* damaged */
size_t d3s_pack(const d3_settings_t *s, uint8_t *out, size_t cap);
int d3s_unpack(const uint8_t *in, size_t n, d3_settings_t *s, int *rejected);

#ifdef __cplusplus
}
#endif
#endif
