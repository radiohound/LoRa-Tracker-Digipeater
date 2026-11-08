# LACF 3.0 frame code (vendored)

`hb3.c` and `hb3.h` are the C reference implementation of the LACF 3.0 frame (`3C FF 03`, spec 12),
copied unchanged from [radiohound/lora-aprs-ideas](https://github.com/radiohound/lora-aprs-ideas),
branch `proposal-3.0.0-dev.1`, commit `ac1a4f4` (`tools/hb3/`). MIT licence.

3.0.0-dev.1 is a proposal that F4MLV has not yet agreed to. Its "relay without the model" (spec 12,
Relay step 8) is what lets this digipeater work without the PPM tables: the CRC is over the bytes as
sent, so a relay checks the frame and forwards the compressed content byte for byte.

On the `lacf3-digi` branch the digipeater uses only `hb3_decode_front` and `hb3_relay_to_passthrough`, and the
PPM entry points that `hb3.c` names (`hbi_*`, `hbc_*`) are stand-ins that refuse every payload, so the tables
are not linked. On this branch (`lacf3-digi-full`) the real tables below take their place.

To update: copy the two files again from a newer commit, note it here, and run the host test
(`make -C test/host VECTORS=<lora-aprs-ideas>/test`), which checks the relay against the reference vectors.

## The PPM tables (branch `lacf3-digi-full`)

Also from the same commit, unchanged except for file names and include paths: `reference/aprs_ppm.c`
(as `aprs_ppm.inc`), `reference/aprs_ppm.h`, `reference/aprs_ppm_model.h`, `tools/hb/hbi.c`, `tools/hb/hbc.c`,
`tools/hb/hb_payload.h`, `tools/integrity/fast_freq.h` and the model 0 tables `tools/hybrid/tables/model_hybridB_info_native.c`
and `model_hybridB_comment.c` (as `.inc`). The `.inc` files are compiled through `hbi.c` and `hbc.c`, which rename
the codec's names so both tables live in one program. `lacf3_destinations.h` is generated from `tools/hybrid/tables/tocall_set1.txt`.
