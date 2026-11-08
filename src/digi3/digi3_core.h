/* LACF 3.0 digipeater core: the decision and the frame to send, with no Arduino or radio code, so it runs and is tested on a PC too.
 *
 * Frames handled:
 *   3C FF 01 + TNC2 text   plain LoRa APRS: repeated as plain text with the new path
 *   3C FF 03 ...           LACF 3.0 (proposal 3.0.0-dev.1): relayed "without the model" (spec 12, Relay step 8): structure and CRC checked,
 *                          path rewritten, position block and content copied byte for byte, this relay's RXT tuple appended
 *   3C FF 02 ...           LACF 2.0: needs the PPM tables to check the content, which this build does not carry: not relayed
 *
 * The path rule is the one the LoRa APRS Pi uses (and spec 11 for LACF relays): the first hop not yet used decides.
 *   fill-in: WIDE1-1 -> MYCALL*        MYCALL -> MYCALL*
 *   wide:    also WIDE2-1 -> MYCALL*   WIDE2-2 -> MYCALL*,WIDE2-1
 * A path that breaks the WIDEn-N rule (1 <= N <= n <= 7) is never repeated, nor a packet this station already repeated or sent. */
#ifndef DIGI3_CORE_H
#define DIGI3_CORE_H
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define D3_MODE_FILLIN 1
#define D3_MODE_WIDE   2

#define D3_FMT_PLAIN 1
#define D3_FMT_LACF3 3

#define D3_MAX_FRAME 255
#define D3_MAX_HOPS  8

/* What to send for one received packet. The frame is built at transmit time (d3_build), so the RXT time to hop can include the wait. */
typedef struct {
    uint8_t  fmt;                    /* D3_FMT_PLAIN or D3_FMT_LACF3 */
    uint16_t rx_len;
    uint8_t  rx[D3_MAX_FRAME];       /* LACF3: the received frame */
    uint16_t text_len;
    char     text[D3_MAX_FRAME + 1]; /* plain: the packet to send (TNC2); LACF3: "src>dest,newpath:" */
    uint32_t key;                    /* duplicate key: the same for every copy of the packet, whatever its path */
} d3_job_t;

/* Why a frame was not repeated (for the debug log and the counters). */
enum { D3_OK = 0, D3_NOT_APRS, D3_LACF2, D3_BAD_FRAME, D3_OWN, D3_PATH_RULE, D3_ALREADY, D3_NO_HOPS, D3_NOT_SERVED, D3_TOO_LONG, D3_REASONS };
const char *d3_reason(int r);

/* The path to repeat with. path is the TNC2 path ("WIDE1-1,WIDE2-1"), mycall upper case ("K6ATV-11"). Returns D3_OK and fills out, or a reason. */
int d3_new_path(const char *path, size_t plen, const char *mycall, int mode, char *out, size_t cap);

/* Decide for one received frame; on D3_OK the job holds what is needed to build the repeat. */
int d3_decide(const uint8_t *fr, size_t n, const char *mycall, int mode, d3_job_t *job);

/* The frame to transmit: plain text as is; LACF3 relayed with this station's RXT tuple (tuple may be NULL: no tuple). Returns the length, or 0 if it must not be sent. */
size_t d3_build(const d3_job_t *job, const char *mycall, const uint8_t *tuple, uint8_t *out, size_t cap);

/* RXT tuple (F4MLV's scales, the same as the LoRa APRS Pi and the iGate's rxt_scale.h). */
uint8_t d3_rxt_rssi(float dbm);
uint8_t d3_rxt_snr(float db);
uint8_t d3_rxt_freq_offset(float hz);
uint8_t d3_rxt_time_to_hop(float ms, int sf, float bw_khz);
void    d3_rxt_tuple(float rssi_dbm, float snr_db, float fo_hz, float tth_ms, int sf, float bw_khz, uint8_t out[4]);

uint32_t d3_fnv1a(uint32_t h, const uint8_t *p, size_t n);

#ifdef __cplusplus
}
#endif
#endif
