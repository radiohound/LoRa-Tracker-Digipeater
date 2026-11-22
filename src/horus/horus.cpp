// Horus Binary transmitter. Layer-2 coding is in horus_l2.c (LGPL, see NOTICE.md).
#include <Arduino.h>
#include <RadioLib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "config.h"
#include "horus.h"
#include "horus_v3.h"
#include "../settings/tk_glue.h"

extern "C" {
#include "horus_l2.h"
}

extern STM32WLx radio;      // defined in main.cpp

#define HORUS_PREAMBLE_LEN  16            // 0x1B bytes: tones 0,1,2,3 repeating

static HorusSettings settings;
static bool     settingsReady = false;
static uint16_t seq = 0;
static uint8_t  divCount = 0;
static uint32_t lastTxStartMs = 0;
uint32_t horusLastTxStartMs() { return lastTxStartMs; }
static float    lastAltM = 0;
static uint32_t lastAltMs = 0;
static bool     haveLastAlt = false;

HorusSettings& horusSettings() {
    if (!settingsReady) {
        settings.version       = HORUS_VERSION;
        snprintf(settings.callsign, sizeof(settings.callsign), "%s", DEFAULT_CALLSIGN);   // replaced by tkPushHorus() at boot
        settings.payloadId     = HORUS_PAYLOAD_ID;
        settings.freqMhz       = HORUS_FREQ_MHZ;
        settings.powerDbm      = HORUS_TX_POWER_DBM;
        settings.syncSeconds   = HORUS_TIME_SYNC_SECONDS;
        settings.offsetSeconds = HORUS_TIME_SYNC_OFFSET_SECONDS;
        settings.beaconDivider = HORUS_BEACON_DIVIDER;
        settingsReady = true;
    }
    return settings;
}

bool horusSetCallsign(const char* c) {
    size_t n = c ? strlen(c) : 0;
    if (n < 1 || n > 15) return false;
    for (size_t i = 0; i < n; i++) {
        char ch = c[i];
        if (!((ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
              ch == '-' || ch == '/')) return false;
    }
    snprintf(horusSettings().callsign, sizeof(settings.callsign), "%s", c);
    return true;
}

uint16_t horusSequence() { return seq; }

// CRC16-CCITT, init 0xFFFF, poly 0x1021 (same as Python binascii.crc_hqx)
static uint16_t crc16(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }

// Packs the 32-byte Horus v2 payload (field order as in horusdemodlib encoder.py).
static void buildV2(uint8_t* p, const HorusFix& f) {
    memset(p, 0, 32);
    put16(p + 0, settings.payloadId);
    put16(p + 2, seq);
    p[4] = f.hour; p[5] = f.minute; p[6] = f.second;
    memcpy(p + 7,  &f.lat, 4);                       // little-endian float32
    memcpy(p + 11, &f.lon, 4);
    float a = f.altM; if (a < 0) a = 0; if (a > 65535) a = 65535;
    put16(p + 15, (uint16_t)a);
    float s = f.speedKmh; if (s < 0) s = 0; if (s > 255) s = 255;
    p[17] = (uint8_t)s;
    p[18] = f.sats;
    p[19] = f.hasTemp ? (uint8_t)(int8_t)f.tempC : 0;
    float b = f.batteryV; if (b < 0) b = 0; if (b > 5) b = 5;
    p[20] = (uint8_t)(255.0f * b / 5.0f);
    // p[21..29]: custom data (ascent rate, external temp/humidity/pressure) left at zero
    put16(p + 30, crc16(p, 30));
}

// Builds a v3 frame (CRC + UPER + padding). Tries a 32-byte frame, then 48. Returns its size.
static int buildV3(uint8_t* frame, const HorusFix& f, uint32_t t) {
    HorusV3Packet p = {};
    p.callsign = settings.callsign;
    p.sequence = seq;
    p.timeOfDaySeconds = (int32_t)(t / 1000UL);
    p.lat1e5 = (int32_t)lroundf(f.lat * 100000.0f);      // round, do not truncate
    p.lon1e5 = (int32_t)lroundf(f.lon * 100000.0f);
    p.altitudeM = (int32_t)lroundf(f.altM);
    if (p.altitudeM < -999) p.altitudeM = -999;
    if (p.altitudeM > 50000) p.altitudeM = 50000;
    p.hasSpeed = true; p.speedKmh = (uint16_t)(f.speedKmh < 0 ? 0 : (f.speedKmh > 512 ? 512 : f.speedKmh));
    p.hasSats  = true; p.sats = f.sats > 31 ? 31 : f.sats;
    if (haveLastAlt && f.fixMs != lastAltMs) {      // ascent rate from the last two packets
        float dt = (float)(int32_t)(f.fixMs - lastAltMs) / 1000.0f;
        if (dt >= 1.0f && dt <= 900.0f) {
            float cms = (f.altM - lastAltM) * 100.0f / dt;
            if (cms > 32767) cms = 32767; if (cms < -32767) cms = -32767;
            p.hasAscent = true; p.ascentCmPerS = (int16_t)cms;
        }
    }
    if (f.hasTemp) {
        float x = f.tempC * 10.0f; if (x > 1023) x = 1023; if (x < -1023) x = -1023;
        p.hasTemp = true; p.tempInternalX10 = (int16_t)x;
    }
    if (f.batteryV > 0) {
        float mv = f.batteryV * 1000.0f; if (mv > 16383) mv = 16383;
        p.hasBattery = true; p.batteryMv = (uint16_t)mv;
    }
    int n = horusV3Encode(p, frame, 32);
    if (n < 0) n = horusV3Encode(p, frame, 48);
    return n;
}

// Moves the packet time forward to the moment of transmission and, if time slots are on,
// waits for the next slot start. GPS time -> seconds since midnight UTC, slots counted from the hour.
// Returns ms since midnight UTC at transmit start.
static uint32_t alignToSlot(HorusFix& f) {
    uint32_t t = ((uint32_t)f.hour * 3600UL + (uint32_t)f.minute * 60UL + f.second) * 1000UL
               + (millis() - f.fixMs);
    if (settings.syncSeconds > 0) {
        const uint32_t period = (uint32_t)settings.syncSeconds * 1000UL;
        const uint32_t off    = ((uint32_t)settings.offsetSeconds * 1000UL) % period;
        uint32_t phase = ((t % 3600000UL) + period - off) % period;
        if (phase) {
            uint32_t wait = period - phase, t0 = millis();
            while (millis() - t0 < wait) { tkSerialPoll(); delay(5); }   // keep the command line alive while waiting
            t += wait;
        }
    }
    t %= 86400000UL;
    f.hour   = t / 3600000UL;
    f.minute = (t / 60000UL) % 60;
    f.second = (t / 1000UL) % 60;
    return t;
}

bool horusSend(const HorusFix& fixIn) {
    if (!fixIn.valid) return false;
    horusSettings();
    HorusFix fix = fixIn;
    uint32_t t = alignToSlot(fix);
    lastTxStartMs = millis();
    ++seq;

    uint8_t frame[64];
    int frameLen;
    if (settings.version == 3) {
        frameLen = buildV3(frame, fix, t);
        if (frameLen < 0) return false;
    } else {
        buildV2(frame, fix);
        frameLen = 32;
    }
    lastAltM = fix.altM; lastAltMs = fix.fixMs; haveLastAlt = true;

    uint8_t coded[160];
    horus_l2_init();
    int n = horus_l2_encode_tx_packet(coded, frame, frameLen);
    if (n != horus_l2_get_num_tx_data_bytes(frameLen) || n > 120) return false;

    static uint8_t tx[HORUS_PREAMBLE_LEN + 120];
    memset(tx, 0x1B, HORUS_PREAMBLE_LEN);
    memcpy(tx + HORUS_PREAMBLE_LEN, coded, n);

    // Same bring-up order as radioInit() in main.cpp, in FSK mode.
    int st = radio.beginFSK(settings.freqMhz);
    if (st == RADIOLIB_ERR_NONE) st = radio.setTCXO(TCXO_VOLTAGE);
    // The HP PA is used above 14 dBm; the E77-400M22S antenna is fed from the HP PA.
    if (st == RADIOLIB_ERR_NONE) st = radio.setOutputPower(settings.powerDbm);
    if (st == RADIOLIB_ERR_NONE) st = radio.setCurrentLimit(LORA_OCP_MA);
    if (st != RADIOLIB_ERR_NONE) return false;

    FSK4Client fsk4(&radio);
    st = fsk4.begin(settings.freqMhz, (uint32_t)(HORUS_TONE_SPACING_HZ + 0.5f), HORUS_BAUD);
    if (st != RADIOLIB_ERR_NONE) return false;

    fsk4.write(tx, HORUS_PREAMBLE_LEN + n);      // one call: no gap between preamble and packet
    radio.standby();
    return true;
}

bool horusSendDivided(const HorusFix& fix) {
    horusSettings();
    if (++divCount < settings.beaconDivider) return false;
    divCount = 0;
    return horusSend(fix);
}
