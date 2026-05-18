// Vendored from libCATS (https://github.com/CamK06/libCATS), BSD-2-Clause.
// TX-only trim: upstream cats_ldpc_decode() and ldpc_pick_code() depend on the
// external labrador-ldpc library. This firmware only transmits, so the decode
// path is replaced with a stub and the labrador dependency is dropped. The
// encode path below is unmodified and self-contained (uses bundled matrices).
#include "cats/ldpc.h"
#include "cats/util.h"
#include "cats/error.h"
#include "cats/ldpc_matrices.h"

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <assert.h>

// Temporary: Will replace pick_code once decoding is implemented
static cats_ldpc_code* ldpc_pick_code_2(const size_t len)
{
    if(len >= 512) {
        return &tm8192;
    }
    else if(len >= 128) {
        return &tm2048;
    }
    else if(len >= 32) {
        return &tc512;
    }
    else if(len >= 16) {
        return &tc256;
    }
    else {
        return &tc128;
    }
}

static size_t ldpc_encode_chunk(const uint8_t* data, const cats_ldpc_code* code, uint8_t* parity_out)
{
    // Code parameters
    const int data_length_bits = code->data_length_bits;
    const int parity_length_bits = code->code_length_bits - code->data_length_bits;
    const int circ_size = code->circulant_size;
    const uint64_t* gc = code->matrix;
    const int row_len = parity_length_bits / 64;

    memset(parity_out, 0x00, parity_length_bits / 8);

    for (int offset = 0; offset < circ_size; offset++) {
        for (int crow = 0; crow < data_length_bits / circ_size; crow++) {
            const int bit = crow * circ_size + offset;
            if(GET_BIT(data[bit / 8], bit % 8)) {
                for (int idx = 0; idx < row_len; idx++) {
                    uint64_t circ = gc[(crow * row_len) + idx];
                    for(int j = 0; j < 8; j++) {
                        parity_out[idx*8 + j] ^= (uint8_t)(circ >> ((7 - j) * 8));
                    }
                }
            }
        }

        for (int block = 0; block < parity_length_bits / circ_size; block++) {
            uint8_t* parity_block = &parity_out[block * circ_size / 8];
            uint8_t carry = parity_block[0] >> 7;
            for (int x = (circ_size / 8) - 1; x >= 0; x--) {
                uint8_t c = parity_block[x] >> 7;
                parity_block[x] = (parity_block[x] << 1) | carry;
                carry = c;
            }
        }
    }

    return parity_length_bits / 8;
}

size_t cats_ldpc_encode(uint8_t* data, size_t len)
{
    assert(len < 4096); // 8192??

    // Enough temp space to support the largest LDPC code; TM8192
    uint8_t parity[512];

    // Split data into blocks and encode each block
    int i = 0;
    while(i < len) {
        const int remaining_bytes = len - i;
        const cats_ldpc_code* code = ldpc_pick_code_2(remaining_bytes);
        const int data_length = code->data_length_bits / 8;

        uint8_t chunk[data_length];
        memset(chunk, 0xAA, data_length);
        memcpy(chunk, data + i, (remaining_bytes < data_length) ? remaining_bytes : data_length);

        const size_t parity_len = ldpc_encode_chunk(chunk, code, parity);
        memcpy(data + len + i, parity, parity_len); // Parity

        i += data_length;
    }

    const int new_len = (len*2) + (i-len) + 2; // (Data + Parity) + (Padded parity) + (Length)
    data[new_len - 2] = len;
    data[new_len - 1] = len >> 8;

    return new_len;
}

// Decode path intentionally not vendored (TX-only firmware; would otherwise
// require the external labrador-ldpc dependency). Kept as a stub so the libCATS
// API/ABI stays intact and any unused decode call sites still link.
size_t cats_ldpc_decode(uint8_t* buf, size_t buf_len)
{
    (void)buf;
    (void)buf_len;
    throw(LDPC_DECODE_FAIL);
}
