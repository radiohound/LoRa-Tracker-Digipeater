# Third-party code in this folder

`horus_l2.c`, `horus_l2.h`, `golay23.c`, `golay23.h`, `golayenctable.h`, `golaydectable.h`,
`mpdecode_core.h`, `comp.h`
are copied unchanged from **horusdemodlib** (https://github.com/projecthorus/horusdemodlib,
`src/`, commit 8cc43de, 2026-07-30). They implement the Horus Binary layer-2 coding
(Golay (23,12) forward error correction, interleaver, scrambler, unique word).

They are licensed under the **GNU Lesser General Public License v2.1**, not the MIT license
of the rest of this repository. See the headers in each file and
https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html. Anyone distributing firmware built
with Horus support enabled (`HORUS_ENABLED 1`) must comply with the LGPL for these files:
keep these notices, and make it possible to replace this code and re-link.

Horus support is off by default and these files are not compiled into the default build.
`horus.cpp` and `horus.h` (this repository's own code) only call the public functions
`horus_l2_init()`, `horus_l2_encode_tx_packet()` and `horus_l2_get_num_tx_data_bytes()`.
