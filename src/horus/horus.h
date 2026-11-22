#pragma once
// Horus Binary (4FSK, 100 baud) transmitter for the E77 / STM32WLE5: v2 (payload ID) or v3 (callsign).
// Enabled with HORUS_ENABLED 1 in config.h.

#include <stdint.h>

struct HorusFix {
    bool     valid;                 // false: nothing is sent
    float    lat, lon;              // degrees
    float    altM;                  // metres
    float    speedKmh;
    uint8_t  sats;
    uint8_t  hour, minute, second;  // UTC from the GPS
    uint32_t fixMs;                 // millis() when the GPS time above was read
    bool     hasTemp;  float tempC; // optional
    float    batteryV;              // 0 = unknown
};

// Runtime settings. Initialised from config.h, so a serial or web settings page can change them.
struct HorusSettings {
    uint8_t  version;          // 2 or 3
    char     callsign[16];     // v3 payload callsign, 1..15 chars of A-Z a-z 0-9 - /
    uint16_t payloadId;        // v2 payload ID (256..65535)
    float    freqMhz;          // lowest of the four tones
    int8_t   powerDbm;         // above 14 uses the HP PA
    uint16_t syncSeconds;      // time slots: 0 = off
    uint16_t offsetSeconds;
    uint8_t  beaconDivider;    // tracker-only mode: send on every Nth beacon
};

HorusSettings& horusSettings();                    // defaults are filled in on first use
bool horusSetCallsign(const char* callsign);       // validates; false if not a legal v3 callsign

// Builds one packet and sends it, after waiting for the next time slot if slots are on.
// Leaves the radio in FSK standby: the caller must re-initialise it for LoRa afterwards.
bool horusSend(const HorusFix& fix);

// Counts calls and sends only every beaconDivider-th one.
bool horusSendDivided(const HorusFix& fix);

// millis() when the last packet's transmission began (after the wait for the time slot); 0 before the first one.
uint32_t horusLastTxStartMs();

uint16_t horusSequence();                          // packet counter, for debug output
