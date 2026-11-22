#pragma once
// Horus Binary v3 packet encoder (ASN.1 UPER, subset). Plain C++, no Arduino dependencies,
// so it can be tested on a PC against the horusdemodlib ASN.1 definition (HorusBinaryV3.asn1).
//
// Frame layout (as horusdemodlib's decoder expects): [CRC16 LE of the rest][UPER bytes][zero pad]
// padded to a valid size (32, 48, 64, 96 or 128 bytes). Layer-2 coding is done by the caller.

#include <stdint.h>
#include <stddef.h>

struct HorusV3Packet {
    const char* callsign;      // 1..15 chars of A-Z a-z 0-9 - /
    uint16_t    sequence;
    int32_t     timeOfDaySeconds;   // 0..86399, or -1 if unknown
    int32_t     lat1e5;             // degrees * 100000
    int32_t     lon1e5;
    int32_t     altitudeM;          // -1000 = invalid

    // Optional fields: only sent when the matching has* flag is set.
    bool     hasSpeed;   uint16_t speedKmh;        // 0..512
    bool     hasSats;    uint8_t  sats;            // 0..31
    bool     hasAscent;  int16_t  ascentCmPerS;    // -32767..32767
    bool     hasTemp;    int16_t  tempInternalX10; // -1023..1023 (tenths of a degree C)
    bool     hasBattery; uint16_t batteryMv;       // 0..16383
};

// Encodes into out[0..frameSize-1]. Returns frameSize on success, or a negative number if
// the callsign is invalid, a value is out of range, or the packet does not fit.
int horusV3Encode(const HorusV3Packet& p, uint8_t* out, size_t frameSize);
