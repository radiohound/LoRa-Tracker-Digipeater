/* This digipeater relays LACF 3.0 frames "without the model" (spec 12, Relay step 8): it never decodes or encodes PPM content, so the two
 * PPM tables (about 152 KB) are left out. hb3.c still names their entry points; these stand-ins refuse every payload, so a call that
 * needed the model can only fail, never accept a frame it did not check. hb3_decode_front and hb3_relay_to_passthrough never call them. */
#include <stddef.h>
#include <stdint.h>

size_t hbi_payload_encode(const uint8_t *text, size_t tl, uint8_t *out, size_t cap) { (void)text; (void)tl; (void)out; (void)cap; return 0; }
int    hbi_payload_check(const uint8_t *pay, size_t n, size_t L, uint8_t *text) { (void)pay; (void)n; (void)L; (void)text; return 0; }
size_t hbc_payload_encode(const uint8_t *text, size_t tl, uint8_t *out, size_t cap) { (void)text; (void)tl; (void)out; (void)cap; return 0; }
int    hbc_payload_check(const uint8_t *pay, size_t n, size_t L, uint8_t *text) { (void)pay; (void)n; (void)L; (void)text; return 0; }
