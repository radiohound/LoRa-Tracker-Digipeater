// Horus Binary v2 transmitter. Layer-2 coding is in horus_l2.c (LGPL, see NOTICE.md).
#include <Arduino.h>
#include <RadioLib.h>
#include <string.h>
#include "config.h"
#include "horus.h"

extern "C" {
#include "horus_l2.h"
}

extern STM32WLx radio;      // defined in main.cpp

#define HORUS_PAYLOAD_LEN   32            // Horus v2 payload: 30 bytes + CRC16
#define HORUS_PREAMBLE_LEN  16            // 0x1B bytes: tones 0,1,2,3 repeating

static uint16_t seq = 0;
static uint8_t  divCount = 0;

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
static void buildPayload(uint8_t* p, const HorusFix& f) {
    memset(p, 0, HORUS_PAYLOAD_LEN);
    put16(p + 0, HORUS_PAYLOAD_ID);
    put16(p + 2, ++seq);
    p[4] = f.hour; p[5] = f.minute; p[6] = f.second;
    memcpy(p + 7,  &f.lat, 4);                       // little-endian float32
    memcpy(p + 11, &f.lon, 4);
    float a = f.altM; if (a < 0) a = 0; if (a > 65535) a = 65535;
    put16(p + 15, (uint16_t)a);
    float s = f.speedKmh; if (s < 0) s = 0; if (s > 255) s = 255;
    p[17] = (uint8_t)s;
    p[18] = f.sats;
    p[19] = (uint8_t)f.tempC;
    float b = f.batteryV; if (b < 0) b = 0; if (b > 5) b = 5;
    p[20] = (uint8_t)(255.0f * b / 5.0f);
    // p[21..29]: custom data (ascent rate, external temp/humidity/pressure) left at zero
    put16(p + 30, crc16(p, 30));
}

// Moves the packet time forward to the moment of transmission and, if time slots are on,
// waits for the next slot start. GPS time -> seconds since midnight UTC, slots counted from the hour.
static void alignToSlot(HorusFix& f) {
    uint32_t t = ((uint32_t)f.hour * 3600UL + (uint32_t)f.minute * 60UL + f.second) * 1000UL
               + (millis() - f.fixMs);                      // ms since midnight at this moment
#if HORUS_TIME_SYNC_SECONDS > 0
    const uint32_t period = (uint32_t)HORUS_TIME_SYNC_SECONDS * 1000UL;
    const uint32_t off    = ((uint32_t)HORUS_TIME_SYNC_OFFSET_SECONDS * 1000UL) % period;
    uint32_t phase = ((t % 3600000UL) + period - off) % period;
    if (phase) {
        uint32_t wait = period - phase;
        delay(wait);
        t += wait;
    }
#endif
    t %= 86400000UL;
    f.hour   = t / 3600000UL;
    f.minute = (t / 60000UL) % 60;
    f.second = (t / 1000UL) % 60;
}

bool horusSend(const HorusFix& fixIn) {
    if (!fixIn.valid) return false;
    HorusFix fix = fixIn;
    alignToSlot(fix);

    uint8_t payload[HORUS_PAYLOAD_LEN];
    buildPayload(payload, fix);

    uint8_t coded[128];
    horus_l2_init();
    int n = horus_l2_encode_tx_packet(coded, payload, HORUS_PAYLOAD_LEN);   // 65 bytes
    if (n != horus_l2_get_num_tx_data_bytes(HORUS_PAYLOAD_LEN) || n > 100) return false;

    static uint8_t frame[HORUS_PREAMBLE_LEN + 100];
    memset(frame, 0x1B, HORUS_PREAMBLE_LEN);
    memcpy(frame + HORUS_PREAMBLE_LEN, coded, n);

    // Same bring-up order as radioInit() in main.cpp, in FSK mode.
    int st = radio.beginFSK(HORUS_FREQ_MHZ);
    if (st == RADIOLIB_ERR_NONE) st = radio.setTCXO(TCXO_VOLTAGE);
    // The HP PA is used above 14 dBm; the E77-400M22S antenna is fed from the HP PA.
    if (st == RADIOLIB_ERR_NONE) st = radio.setOutputPower(HORUS_TX_POWER_DBM);
    if (st == RADIOLIB_ERR_NONE) st = radio.setCurrentLimit(LORA_OCP_MA);
    if (st != RADIOLIB_ERR_NONE) return false;

    FSK4Client fsk4(&radio);
    st = fsk4.begin(HORUS_FREQ_MHZ, (uint32_t)(HORUS_TONE_SPACING_HZ + 0.5f), HORUS_BAUD);
    if (st != RADIOLIB_ERR_NONE) return false;

    fsk4.write(frame, HORUS_PREAMBLE_LEN + n);   // one call: no gap between preamble and packet
    radio.standby();
    return true;
}

bool horusSendDivided(const HorusFix& fix) {
    if (++divCount < HORUS_BEACON_DIVIDER) return false;
    divCount = 0;
    return horusSend(fix);
}
