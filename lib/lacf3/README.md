# LACF 3.0 frame code (vendored)

`hb3.c` and `hb3.h` are the C reference implementation of the LACF 3.0 frame (`3C FF 03`, spec 12),
copied unchanged from [radiohound/lora-aprs-ideas](https://github.com/radiohound/lora-aprs-ideas),
branch `proposal-3.0.0-dev.1`, commit `ac1a4f4` (`tools/hb3/`). MIT licence.

3.0.0-dev.1 is a proposal that F4MLV has not yet agreed to. Its "relay without the model" (spec 12,
Relay step 8) is what lets this digipeater work without the PPM tables: the CRC is over the bytes as
sent, so a relay checks the frame and forwards the compressed content byte for byte.

The digipeater uses only `hb3_decode_front` and `hb3_relay_to_passthrough`. The PPM entry points that
`hb3.c` names (`hbi_*`, `hbc_*`) are stand-ins in `src/digi3/hb3_nomodel.c` that refuse every payload,
so the about 152 KB of tables are not linked.

To update: copy the two files again from a newer commit, note it here, and run the host test
(`make -C test/host VECTORS=<lora-aprs-ideas>/test`), which checks the relay against the reference vectors.
