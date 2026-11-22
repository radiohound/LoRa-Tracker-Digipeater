#pragma once
// Horus Binary v2 (4FSK, 100 baud) transmitter for the E77 / STM32WLE5.
// Enabled with HORUS_ENABLED 1 in config.h. See README for the clock requirement.

#include <stdint.h>

struct HorusFix {
    bool    valid;        // false: nothing is sent
    float   lat, lon;     // degrees
    float   altM;         // metres
    float   speedKmh;
    uint8_t sats;
    uint8_t hour, minute, second;   // UTC from the GPS
    int8_t  tempC;
    float   batteryV;     // 0 if unknown
    uint32_t fixMs;       // millis() when the GPS time above was read
};

// Builds one Horus v2 packet and sends it on HORUS_FREQ_MHZ. Leaves the radio in FSK
// standby: the caller must re-initialise it for LoRa afterwards.
// Returns true if a packet was sent.
bool horusSend(const HorusFix& fix);

// Counts calls and sends only every HORUS_BEACON_DIVIDER-th one.
bool horusSendDivided(const HorusFix& fix);

// Packet counter (wraps at 65535), exposed for debug output.
uint16_t horusSequence();
