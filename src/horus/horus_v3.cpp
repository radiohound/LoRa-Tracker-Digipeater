// Horus Binary v3 encoder: hand-written UPER for the HorusBinaryV3.asn1 "Telemetry" type.
// Only the fields listed in horus_v3.h are supported; every other OPTIONAL is sent as absent.
// Checked bit-for-bit against the asn1tools encoder (see tools/test_horus_v3.py).
#include "horus_v3.h"
#include <string.h>

namespace {

struct BitWriter {
    uint8_t* buf; size_t cap; size_t nbits; bool overflow;
    void put(uint32_t value, int bits) {          // MSB first
        for (int i = bits - 1; i >= 0; i--) {
            if (nbits >= cap * 8) { overflow = true; return; }
            if ((value >> i) & 1) buf[nbits >> 3] |= (uint8_t)(0x80 >> (nbits & 7));
            nbits++;
        }
    }
};

// Constrained whole number: (value - lo) in `bits` bits. Returns false if out of range.
bool putRange(BitWriter& w, int32_t value, int32_t lo, int32_t hi, int bits) {
    if (value < lo || value > hi) return false;
    w.put((uint32_t)(value - lo), bits);
    return true;
}

// Index of a character in the permitted alphabet "-/0-9A-Za-z" (64 characters, 6 bits).
int charIndex(char c) {
    if (c == '-') return 0;
    if (c == '/') return 1;
    if (c >= '0' && c <= '9') return 2 + (c - '0');
    if (c >= 'A' && c <= 'Z') return 12 + (c - 'A');
    if (c >= 'a' && c <= 'z') return 38 + (c - 'a');
    return -1;
}

uint16_t crc16(const uint8_t* d, size_t n) {        // CCITT, init 0xFFFF (binascii.crc_hqx)
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

} // namespace

int horusV3Encode(const HorusV3Packet& p, uint8_t* out, size_t frameSize) {
    if (!(frameSize == 32 || frameSize == 48 || frameSize == 64 || frameSize == 96 || frameSize == 128))
        return -1;
    if (!p.callsign) return -2;
    size_t len = strlen(p.callsign);
    if (len < 1 || len > 15) return -2;
    for (size_t i = 0; i < len; i++) if (charIndex(p.callsign[i]) < 0) return -2;

    memset(out, 0, frameSize);
    BitWriter w = { out + 2, frameSize - 2, 0, false };       // UPER bytes start after the CRC

    // Preamble: extension bit, then one presence bit per root OPTIONAL, in schema order:
    // extraSensors, velocity, sats, ascent, pressure, temperature, humidity, milliVolts,
    // counts, gnssPowerSaveState, customData.
    w.put(0, 1);                                  // not using the extension (via)
    w.put(0, 1);                                  // extraSensors
    w.put(p.hasSpeed,   1);
    w.put(p.hasSats,    1);
    w.put(p.hasAscent,  1);
    w.put(0, 1);                                  // pressure
    w.put(p.hasTemp,    1);
    w.put(0, 1);                                  // humidity
    w.put(p.hasBattery, 1);
    w.put(0, 1);                                  // counts
    w.put(0, 1);                                  // gnssPowerSaveState
    w.put(0, 1);                                  // customData

    // payloadCallsign: length 1..15 in 4 bits (len-1), then 6 bits per character
    w.put((uint32_t)(len - 1), 4);
    for (size_t i = 0; i < len; i++) w.put((uint32_t)charIndex(p.callsign[i]), 6);

    if (!putRange(w, p.sequence,         0,        65535,    16)) return -3;
    if (!putRange(w, p.timeOfDaySeconds, -1,       86400,    17)) return -3;
    if (!putRange(w, p.lat1e5,           -9000000, 9000000,  25)) return -3;
    if (!putRange(w, p.lon1e5,           -18000000,18000000, 26)) return -3;
    if (!putRange(w, p.altitudeM,        -1000,    50000,    16)) return -3;

    if (p.hasSpeed  && !putRange(w, p.speedKmh,     0,      512,   10)) return -3;
    if (p.hasSats   && !putRange(w, p.sats,         0,      31,    5))  return -3;
    if (p.hasAscent && !putRange(w, p.ascentCmPerS, -32767, 32767, 16)) return -3;
    if (p.hasTemp) {                              // TemperatureSensors: 4 presence bits, then values
        w.put(1, 1); w.put(0, 1); w.put(0, 1); w.put(0, 1);        // only "internal"
        if (!putRange(w, p.tempInternalX10, -1023, 1023, 11)) return -3;
    }
    if (p.hasBattery) {                           // MilliVoltSensors: 4 presence bits, then values
        w.put(1, 1); w.put(0, 1); w.put(0, 1); w.put(0, 1);        // only "battery"
        if (!putRange(w, p.batteryMv, 0, 16383, 14)) return -3;
    }

    if (w.overflow) return -4;
    uint16_t crc = crc16(out + 2, frameSize - 2);
    out[0] = (uint8_t)crc; out[1] = (uint8_t)(crc >> 8);
    return (int)frameSize;
}
