/* LACF 3.0 digipeater core: the decision and the frame to send, with no Arduino or radio code, so it runs and is tested on a PC too.
 *
 * Frames handled:
 *   3C FF 01 + TNC2 text   plain LoRa APRS: repeated as plain text with the new path
 *   3C FF 03 ...           LACF 3.0 (proposal 3.0.0-dev.1): relayed "without the model" (spec 12, Relay step 8): structure and CRC checked,
 *                          path rewritten, position block and content copied byte for byte, this relay's RXT tuple appended
 *   3C FF 02 ...           LACF 2.0: not relayed
 *
 * Built with D3_WITH_MODEL (the PPM tables, branch lacf3-digi-full), a LACF 3.0 frame is first decoded in full, with every receiver check
 * (the PPM content too), then relayed with its content copied byte for byte; a frame whose model or destination code this build does not hold is
 * still relayed without the model. A plain frame can be re-encoded as LACF 3.0 before it is repeated (d3_job_t.convert).
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
#define D3_MAX_TEXT  400    /* a decoded LACF frame of 255 bytes can be a longer TNC2 packet (its position block and destination expand) */

/* What to send for one received packet. The frame is built at transmit time (d3_build), so the RXT time to hop can include the wait. */
typedef struct {
    uint8_t  fmt;                    /* D3_FMT_PLAIN or D3_FMT_LACF3 */
    uint16_t rx_len;
    uint8_t  rx[D3_MAX_FRAME];       /* LACF3: the received frame */
    uint16_t text_len;
    char     text[D3_MAX_TEXT + 1];  /* plain: the packet to send (TNC2); LACF3: "src>dest,newpath:", or the whole new packet when decoded */
    uint32_t key;                    /* duplicate key: the same for every copy of the packet, whatever its path */
    uint8_t  decoded;                /* LACF3: decoded in full with the model (text is then the whole new packet) */
    uint8_t  named;                  /* the hop used was this station's own callsign (the sender asked for this digi by name) */
    uint8_t  used_hops;              /* hops already used in the path as received (for cancel-on-hear) */
    uint8_t  convert;                /* plain: send it as a LACF 3.0 frame (needs D3_WITH_MODEL) */
} d3_job_t;

/* Why a frame was not repeated (for the debug log and the counters). */
enum { D3_OK = 0, D3_NOT_APRS, D3_LACF2, D3_BAD_FRAME, D3_OWN, D3_PATH_RULE, D3_ALREADY, D3_NO_HOPS, D3_NOT_SERVED, D3_TOO_LONG, D3_REASONS };
const char *d3_reason(int r);

/* The path to repeat with. path is the TNC2 path ("WIDE1-1,WIDE2-1"), mycall upper case ("K6ATV-11"). Returns D3_OK and fills out, or a reason.
 * named (may be NULL) is set to 1 when the hop used was mycall itself. */
int d3_new_path(const char *path, size_t plen, const char *mycall, int mode, char *out, size_t cap, int *named);

/* For cancel-on-hear: the duplicate key of a frame (the same as d3_decide gives) and how many hops of its path are used. Returns 1 if the frame
 * could be read that far (plain text, or a LACF 3.0 frame whose structure and CRC are good). */
int d3_identify(const uint8_t *fr, size_t n, uint32_t *key, int *used);

/* The packet (TNC2, path as received) of the last frame d3_decide could read: plain text, or LACF 3.0 decoded with the model. Empty when it could
 * not (a LACF frame relayed without the model, a damaged frame). For the neighbour list and "moving stations only". */
const char *d3_last_text(void);

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

#ifdef D3_WITH_MODEL
/* Load the destination list (needed to decode and encode with the model). Returns the number of names. */
int d3_init_model(void);
#endif

#ifdef __cplusplus
}
#endif
#endif
