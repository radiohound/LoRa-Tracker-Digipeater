// ============================================================
// LACF 3.0 digipeater for the STM32WLE5 (Ebyte E77-400MBL-01), with the PPM tables
//
// Digipeat only: listens all the time. LACF 3.0 frames (spec 12 of radiohound/lora-aprs-ideas,
// proposal 3.0.0-dev.1) are decoded in full and checked with the model, then relayed with the path
// rewritten, the compressed content copied byte for byte and this relay's RXT tuple added; a frame
// with a model this build does not hold is relayed without the model. Plain LoRa APRS frames are
// repeated as text, or re-encoded as LACF 3.0 when TX_FORMAT_LACF3 is 1. No GPS, no iGate.
// LACF 2.0 frames are not repeated.
//
// Radio set-up as in PicoTrack and LoRa-Tracker-Digipeater (K6ATV): RF switch on PA6/PA7/PB3, TCXO 1.7 V.
// Settings: src/digi3/config_digi3.h.  Core logic (tested on a PC): src/digi3/digi3_core.c.
// ============================================================

#include <Arduino.h>
#include <RadioLib.h>
#include <IWatchdog.h>
#include "digi3_core.h"
#include "config_digi3.h"
#include "hb3.h"
#ifndef D3_WITH_MODEL
#error "this branch needs the PPM tables: build the e77_lacf3_digi_full environment (it defines D3_WITH_MODEL)"
#endif

#if DEBUG_SERIAL
  #define LOG(...)   Serial.print(__VA_ARGS__)
  #define LOGLN(...) Serial.println(__VA_ARGS__)
#else
  #define LOG(...)
  #define LOGLN(...)
#endif

STM32WLx radio = new STM32WLx_Module();
static const uint32_t               rfswitch_pins[]  = RFSWITCH_PINS;
static const Module::RfSwitchMode_t rfswitch_table[] = { RFSWITCH_TABLE };

static char myCall[16];
static bool callsignSet = false;

// ---- received-packet flag (set in the radio interrupt) ----
static volatile bool     rxFlag = false;
static volatile uint32_t rxEndMs = 0;
static void onRadioRx() { rxFlag = true; rxEndMs = millis(); }

// ---- transmit queue ----
struct TxSlot {
    bool     used;
    uint8_t  tries;
    uint32_t dueMs;      // earliest time to transmit
    uint32_t rxEndMs;    // end of the reception this repeats (for the RXT time to hop); 0 for our own beacon
    float    rssi, snr, fo;
    d3_job_t job;
};
static TxSlot txq[DIGI_QUEUE_SIZE];

// ---- duplicates ----
struct Seen { uint32_t key, ms; };
static Seen seen[DEDUPE_SLOTS];
static uint8_t seenNext = 0;

// ---- counters ----
static uint32_t nDecoded = 0, nConverted = 0;
static uint32_t nHeard = 0, nCrc = 0, nRelayPlain = 0, nRelayLacf3 = 0, nDup = 0, nBusyDrop = 0, nQueueFull = 0, nBeacons = 0;
static uint32_t nWhy[D3_REASONS];

static uint32_t nextBeaconMs = 0, nextStatsMs = 0;

// ============================================================
static bool seenRecently(uint32_t key, uint32_t now)
{
    for (int i = 0; i < DEDUPE_SLOTS; i++)
        if (seen[i].ms && seen[i].key == key && now - seen[i].ms < DEDUPE_SECONDS * 1000UL) return true;
    return false;
}

static void remember(uint32_t key, uint32_t now)
{
    seen[seenNext].key = key;
    seen[seenNext].ms  = now ? now : 1;
    seenNext = (seenNext + 1) % DEDUPE_SLOTS;
}

static TxSlot *freeSlot()
{
    for (int i = 0; i < DIGI_QUEUE_SIZE; i++) if (!txq[i].used) return &txq[i];
    return nullptr;
}

static uint32_t randomBetween(uint32_t lo, uint32_t hi) { return lo + (uint32_t)random((long)(hi - lo + 1)); }

// ============================================================
// RADIO
// ============================================================
static void radioInit()
{
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    LOG(F("[Radio] init... "));
    int16_t st = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
    if (st == RADIOLIB_ERR_NONE) st = radio.setTCXO(TCXO_VOLTAGE);
    if (st == RADIOLIB_ERR_NONE) st = radio.setOutputPower(LORA_TX_POWER);
    if (st == RADIOLIB_ERR_NONE) st = radio.setCurrentLimit(LORA_OCP_MA);
    if (st == RADIOLIB_ERR_NONE) st = radio.setSyncWord(LORA_SYNC_WORD);
    if (st == RADIOLIB_ERR_NONE) st = radio.setPreambleLength(LORA_PREAMBLE);
    if (st == RADIOLIB_ERR_NONE) st = radio.setCRC(true);
    if (st == RADIOLIB_ERR_NONE) st = radio.setRxBoostedGainMode(RX_BOOSTED_GAIN ? true : false);
    if (st != RADIOLIB_ERR_NONE) {
        LOG(F("failed, code ")); LOGLN(st);
        delay(2000);
        NVIC_SystemReset();                     // a remote site: try again from the start instead of hanging
    }
    radio.setPacketReceivedAction(onRadioRx);
    LOGLN(F("OK"));
}

static void startRx()
{
    rxFlag = false;
    int16_t st = radio.startReceive();
    if (st != RADIOLIB_ERR_NONE) { LOG(F("[Radio] startReceive failed ")); LOGLN(st); }
}

// ============================================================
// RECEIVE
// ============================================================
static void handleRx()
{
    uint8_t  fr[256];
    uint32_t endMs = rxEndMs;
    size_t   n = radio.getPacketLength();
    if (n > sizeof fr) n = sizeof fr;
    int16_t st = radio.readData(fr, n);
    float rssi = radio.getRSSI(), snr = radio.getSNR(), fo = radio.getFrequencyError();
    startRx();                                  // listen again at once; the work below needs no radio
    if (st == RADIOLIB_ERR_CRC_MISMATCH) { nCrc++; return; }
    if (st != RADIOLIB_ERR_NONE || n < 4) return;
    nHeard++;

    TxSlot *slot = freeSlot();
    static d3_job_t scratch;                    // decided here first, so a full queue still counts the reason
    d3_job_t *job = slot ? &slot->job : &scratch;
    int r = callsignSet ? d3_decide(fr, n, myCall, DIGI_MODE, job) : D3_NOT_SERVED;
    LOG(F("[RX] ")); LOG(n); LOG(F(" B ")); LOG(rssi, 0); LOG(F(" dBm ")); LOG(snr, 1); LOG(F(" dB ")); LOG((long)fo); LOG(F(" Hz: "));
    if (r != D3_OK) { nWhy[r]++; LOGLN(d3_reason(r)); return; }
    if (job->decoded) {
        nDecoded++;
#if LOG_DECODED
        LOG(F("decoded ")); LOGLN(d3_last_decoded()); LOG(F("      "));
#endif
    }
    job->convert = (job->fmt == D3_FMT_PLAIN && TX_FORMAT_LACF3) ? 1 : 0;
    uint32_t now = millis();
    if (seenRecently(job->key, now)) { nDup++; LOGLN(F("duplicate")); return; }
    remember(job->key, now);
    if (!slot) { nQueueFull++; LOGLN(F("queue full")); return; }
    slot->used = true; slot->tries = 0;
    slot->dueMs = now + randomBetween(DIGI_DELAY_MIN_MS, DIGI_DELAY_MAX_MS);
    slot->rxEndMs = endMs ? endMs : now;
    slot->rssi = rssi; slot->snr = snr; slot->fo = fo;
    LOG(job->fmt == D3_FMT_LACF3 ? (job->decoded ? F("LACF3 relay ") : F("LACF3 relay without the model ")) : (job->convert ? F("plain, sent as LACF3 ") : F("plain repeat "))); LOGLN(job->text);
}

// ============================================================
// TRANSMIT
// ============================================================
static void transmitDue()
{
    uint32_t now = millis();
    for (int i = 0; i < DIGI_QUEUE_SIZE; i++) {
        TxSlot &s = txq[i];
        if (!s.used || (int32_t)(now - s.dueMs) < 0) continue;

        int16_t cad = radio.scanChannel();      // listen before talking: LoRa channel activity detection
        if (cad == RADIOLIB_LORA_DETECTED) {
            if (++s.tries > DIGI_BUSY_TRIES) { s.used = false; nBusyDrop++; LOGLN(F("[TX] channel busy, dropped")); }
            else s.dueMs = millis() + randomBetween(DIGI_BUSY_MIN_MS, DIGI_BUSY_MAX_MS);
            startRx();
            return;
        }

        uint8_t out[D3_MAX_FRAME]; size_t len;
        if (s.job.fmt == D3_FMT_LACF3) {
            // Time to hop: from the end of reception to the end of our transmission. The length is the relayed length, so build once to learn it.
            uint8_t tup[4] = { 0, 0, 0, 0 };
            len = d3_build(&s.job, myCall, tup, out, sizeof out);
            float toaMs = len ? radio.getTimeOnAir(len) / 1000.0f : 0;
            d3_rxt_tuple(s.rssi, s.snr, s.fo, (float)(millis() - s.rxEndMs) + toaMs, LORA_SF, LORA_BW, tup);
            len = d3_build(&s.job, myCall, tup, out, sizeof out);
        } else {
            len = d3_build(&s.job, myCall, nullptr, out, sizeof out);
        }
        s.used = false;
        if (!len) { startRx(); return; }

        IWatchdog.reload();
        int16_t st = radio.transmit(out, len);
        IWatchdog.reload();
        if (st == RADIOLIB_ERR_NONE) {
            if (s.job.fmt == D3_FMT_PLAIN && s.job.convert && out[2] == 0x03) nConverted++;
            if (s.rxEndMs == 0) nBeacons++;
            else if (s.job.fmt == D3_FMT_LACF3) nRelayLacf3++;
            else nRelayPlain++;
            LOG(F("[TX] ")); LOG(len); LOGLN(F(" B sent"));
        } else {
            LOG(F("[TX] failed, code ")); LOGLN(st);
        }
        startRx();
        return;                                 // one transmission per pass, then listen again
    }
}

// ============================================================
// BEACON: fixed position, plain text
// ============================================================
static void latlon(char *out, size_t cap, double lat, double lon)
{
    long a = lround(fabs(lat) * 6000.0), o = lround(fabs(lon) * 6000.0);     // hundredths of a minute
    snprintf(out, cap, "%02ld%02ld.%02ld%c%c%03ld%02ld.%02ld%c%c",
             a / 6000, (a % 6000) / 100, a % 100, lat >= 0 ? 'N' : 'S', BEACON_SYMBOL_TABLE,
             o / 6000, (o % 6000) / 100, o % 100, lon >= 0 ? 'E' : 'W', BEACON_SYMBOL_CODE);
}

static void queueBeacon()
{
    TxSlot *slot = freeSlot();
    if (!slot) return;
    char pos[32];
    latlon(pos, sizeof pos, BEACON_LAT, BEACON_LON);
    int w = snprintf(slot->job.text, sizeof slot->job.text, "%s>%s%s%s:!%s%s", myCall, APRS_DESTINATION,
                     sizeof(BEACON_PATH) > 1 ? "," : "", BEACON_PATH, pos, BEACON_COMMENT);
    if (w <= 0 || w >= (int)sizeof slot->job.text) return;
    slot->job.fmt = D3_FMT_PLAIN; slot->job.text_len = (uint16_t)w; slot->job.rx_len = 0; slot->job.decoded = 0; slot->job.convert = TX_FORMAT_LACF3;
    slot->used = true; slot->tries = 0; slot->dueMs = millis(); slot->rxEndMs = 0;
    LOG(F("[Beacon] ")); LOGLN(slot->job.text);
}

static void printStats()
{
    LOG(F("[Stats] heard ")); LOG(nHeard); LOG(F(", CRC errors ")); LOG(nCrc);
    LOG(F(", repeated plain ")); LOG(nRelayPlain); LOG(F(", relayed LACF3 ")); LOG(nRelayLacf3);
    LOG(F(", duplicates ")); LOG(nDup); LOG(F(", LACF2 skipped ")); LOG(nWhy[D3_LACF2]);
    LOG(F(", damaged ")); LOG(nWhy[D3_BAD_FRAME]); LOG(F(", busy drops ")); LOG(nBusyDrop);
    LOG(F(", queue full ")); LOG(nQueueFull); LOG(F(", beacons ")); LOG(nBeacons);
    LOG(F(", LACF3 decoded ")); LOG(nDecoded); LOG(F(", sent as LACF3 ")); LOGLN(nConverted);
}

// ============================================================
void setup()
{
#if DEBUG_SERIAL
    Serial.begin(DEBUG_BAUD);
    delay(300);
#endif
    LOGLN(F("=== LACF 3 digipeater with the tables (STM32WLE5), spec " HB3_SPEC_VERSION " ==="));
    if (IWatchdog.isReset(true)) LOGLN(F("[Boot] restarted by the watchdog"));
    IWatchdog.begin((uint32_t)WATCHDOG_S * 1000000UL);

    if (MY_SSID) snprintf(myCall, sizeof myCall, "%s-%d", MY_CALLSIGN, MY_SSID);
    else snprintf(myCall, sizeof myCall, "%s", MY_CALLSIGN);
    for (char *p = myCall; *p; p++) *p = (char)toupper((unsigned char)*p);
    callsignSet = strncmp(myCall, "N0CALL", 6) != 0;
    if (!callsignSet) LOGLN(F("[Boot] set MY_CALLSIGN in config_digi3.h: not relaying or beaconing until then"));

    memset(txq, 0, sizeof txq);
    memset(seen, 0, sizeof seen);
    LOG(F("[Boot] LACF destination list: ")); LOG(d3_init_model()); LOGLN(F(" names"));
    radioInit();
    randomSeed(radio.random(0x7FFFFFFF));
    startRx();

    nextBeaconMs = millis() + BEACON_FIRST_S * 1000UL;
    nextStatsMs  = millis() + STATS_EVERY_MIN * 60000UL;
}

void loop()
{
    IWatchdog.reload();
    if (rxFlag) handleRx();
    transmitDue();

    uint32_t now = millis();
    if (callsignSet && (BEACON_LAT != 0.0 || BEACON_LON != 0.0) && (int32_t)(now - nextBeaconMs) >= 0) {
        queueBeacon();
        nextBeaconMs = now + BEACON_INTERVAL_MIN * 60000UL;
    }
    if ((int32_t)(now - nextStatsMs) >= 0) { printStats(); nextStatsMs = now + STATS_EVERY_MIN * 60000UL; }

    // Sleep until the next interrupt (the radio, or the 1 ms system tick). The radio keeps listening.
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}
