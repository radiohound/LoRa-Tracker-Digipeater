/* Run-time settings for the tracker: the values, the serial command line that changes them, and the record that is kept in flash.
 * Plain C, no Arduino or radio code, so it is tested on a PC (test/host/test_tk_settings.c).
 * The engine follows radiohound/lora-aprs-digi (src/digi3/settings.c): the same commands, the same checks, the same flash record layout,
 * so the same setup page style works. The values in config.h are the DEFAULTS; a saved record in flash overrides them.
 *
 * Each setting is checked against its range when it is typed and again when the record is read back, so a damaged or hand-edited record
 * can never load a value the radio would refuse. The flash record is a list of (id, length, bytes) entries with a CRC; ids are the position
 * in the table in tk_settings.c: only ever append to it. */
#ifndef TK_SETTINGS_H
#define TK_SETTINGS_H
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* identity and APRS */
    char     call[8];             /* base callsign, 1 to 6 characters; the SSID is separate. N0CALL = not set: nothing is sent */
    uint8_t  ssid;                /* 0 to 99; 0 = none */
    char     comment[44];
    char     sym_table[2], sym_code[2];
    char     path[24];
    char     tocall[8];
    uint16_t beacon_s;            /* seconds between beacons */
    uint8_t  aprs;                /* send the LoRa APRS beacon */
    uint16_t gps_timeout_s;       /* longest wait for a GPS fix */
    /* LoRa radio (take effect with "apply") */
    uint32_t freq_hz, bw_hz;
    uint8_t  sf, cr, power_dbm, sync;
    uint8_t  preamble;
    /* Horus Binary (only used when the firmware was built with HORUS_ENABLED) */
    uint8_t  horus;               /* send Horus packets */
    uint8_t  horus_ver;           /* 2 or 3 */
    char     horus_call[16];      /* v3 payload callsign; empty = the callsign above with its SSID */
    uint16_t horus_id;            /* v2 payload ID */
    uint32_t horus_freq_hz;       /* lowest of the four tones */
    int8_t   horus_power_dbm;
    uint16_t horus_sync_s, horus_off_s;   /* time slots: transmit when (t - off) mod sync == 0; sync 0 = off */
    uint8_t  horus_div;           /* tracker-only mode: Horus on every Nth beacon; ignored when the tracker + digipeater mode sends by phase (horus_asc etc.) */
    /* send rates per flight phase, seconds. beacon_s above is the LoRa APRS rate while ascending. */
    uint16_t aprs_desc_s, aprs_gnd_s;
    uint16_t horus_asc_s, horus_desc_s, horus_gnd_s;
    char     horus_note[32];      /* v3: free text sent in every Horus packet; empty = none */
} tk_settings_t;

/* What a command asks the caller to do (bit mask). */
#define TKS_CHANGED   0x001u      /* a setting changed: remember there is something to save */
#define TKS_RADIO     0x002u      /* a LoRa radio setting changed: it takes effect with TKS_APPLY or after a restart */
#define TKS_SAVE      0x004u
#define TKS_APPLY     0x008u      /* re-initialise the radio with the current settings */
#define TKS_REBOOT    0x010u
#define TKS_FORCE     0x020u      /* with TKS_REBOOT: also when there are unsaved changes */
#define TKS_BEACON    0x040u      /* send the APRS beacon now */
#define TKS_HORUS     0x080u      /* send a Horus packet now */
#define TKS_DEFAULTS  0x100u      /* the compiled-in defaults were loaded (not saved yet) */

#define TKS_LINE_MAX  120
#define TKS_REC_MAX   1024        /* most bytes a record can take (the flash page is 2048) */

typedef void (*tks_emit_t)(void *ctx, const char *line);

void tks_defaults(tk_settings_t *s);

/* Run one command line ("set call K6ATV-2", "show", "save"...). Replies go to emit, one line at a time. Returns TKS_* flags. */
unsigned tks_exec(tk_settings_t *s, const char *line, tks_emit_t emit, void *ctx);

/* The list of commands (what "help" starts with); the firmware also prints it at start-up. */
void tks_menu(tks_emit_t emit, void *ctx);

/* The full callsign with SSID, "W2HAB-11". */
void tks_fullcall(const tk_settings_t *s, char *out, size_t cap);

/* The callsign for Horus v3: horus_call, or the full callsign when horus_call is empty. */
void tks_horus_call(const tk_settings_t *s, char *out, size_t cap);

/* 1 when a real callsign is set for LoRa APRS. With N0CALL (the default) the firmware must not send APRS. */
int tks_call_is_set(const tk_settings_t *s);

/* 1 when the callsign Horus would use is a real one: horus_call if it is set (its part before any - or / must not be N0CALL),
 * otherwise the APRS callsign. LoRa APRS and Horus can use different callsigns, and each is checked on its own. */
int tks_horus_call_is_set(const tk_settings_t *s);

/* The flash record. pack returns its length (a multiple of 8, padded with 0xFF) or 0 if cap is too small.
 * unpack starts from the defaults and applies every valid entry; returns the number applied, or a negative TKS_E_* when the record is not
 * usable. *rejected (may be NULL) counts entries that were skipped because their value was out of range. */
#define TKS_E_BLANK  (-1)         /* no record (erased flash) */
#define TKS_E_FORMAT (-2)         /* not a record this firmware understands */
#define TKS_E_CRC    (-3)         /* damaged */
size_t tks_pack(const tk_settings_t *s, uint8_t *out, size_t cap);
int tks_unpack(const uint8_t *in, size_t n, tk_settings_t *s, int *rejected);

#ifdef __cplusplus
}
#endif
#endif
