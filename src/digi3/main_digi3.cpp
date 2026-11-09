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
// Settings: typed over the serial port (help, show, set, save), kept in the last flash page; src/digi3/config_digi3.h holds the defaults.
// Core logic (tested on a PC): src/digi3/digi3_core.c, settings.c.
// ============================================================

#include <Arduino.h>
#include <RadioLib.h>
#include <IWatchdog.h>
#include "digi3_core.h"
#include "neighbors.h"
#include "config_digi3.h"
#include "settings.h"
#include "hb3.h"
#ifndef D3_WITH_MODEL
#error "this branch needs the PPM tables: build the e77_lacf3_digi_full environment (it defines D3_WITH_MODEL)"
#endif

// LOG: events (every packet heard, every transmission). Compiled out when DEBUG_SERIAL is 0, and silenced at run time by "set log off".
// OUT: always printed (command replies, boot messages, the counters on request).
#if DEBUG_SERIAL
  #define LOG(...)   do { if (cfg.log) Serial.print(__VA_ARGS__); } while (0)
  #define LOGLN(...) do { if (cfg.log) Serial.println(__VA_ARGS__); } while (0)
#else
  #define LOG(...)
  #define LOGLN(...)
#endif
#define OUT(...)   Serial.print(__VA_ARGS__)
#define OUTLN(...) Serial.println(__VA_ARGS__)

STM32WLx radio = new STM32WLx_Module();
static const uint32_t               rfswitch_pins[]  = RFSWITCH_PINS;
static const Module::RfSwitchMode_t rfswitch_table[] = { RFSWITCH_TABLE };

static d3_settings_t cfg;            // the settings in use (defaults, then the saved record, then whatever was typed)
static d3_settings_t radioApplied;   // the radio settings the chip is really using
static bool          dirty = false;  // typed but not saved
static char myCall[16];
static bool callsignSet = false;

// ---- received-packet flag (set in the radio interrupt) ----
static volatile bool     rxFlag = false;
static volatile uint32_t rxEndMs = 0;
static void onRadioRx() { rxFlag = true; rxEndMs = millis(); }

// ---- transmit queue ----
struct TxSlot {
    bool     used;
    uint8_t  freeChecks; // channel found free this many times in a row
    uint8_t  usedHops;   // hops used in the path when we heard it (cancel-on-hear)
    uint32_t waitStartMs;// first channel check (0 = not yet)
    uint32_t dueMs;      // earliest time to transmit or check the channel again
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
static uint32_t nDecoded = 0, nConverted = 0, nBusy = 0, nCancelled = 0, nSkipStationary = 0, nSkipOther = 0;
static uint32_t nHeard = 0, nCrc = 0, nRelayPlain = 0, nRelayLacf3 = 0, nDup = 0, nBusyDrop = 0, nQueueFull = 0, nBeacons = 0;

// ---- stations heard ----
static d3_station_t   stationSlots[NEIGHBOR_SLOTS];
static d3_neighbors_t neighbors;
static d3_moving_cfg_t movingCfg;
static uint32_t cadSlotMs;           // one channel check: about 4 symbols
static uint32_t nWhy[D3_REASONS];

static uint32_t nextBeaconMs = 0, nextStatsMs = 0;

// ============================================================
// Everything derived from the settings; run after any change.
static void applySettings()
{
    d3s_fullcall(&cfg, myCall, sizeof myCall);
    callsignSet = strcmp(cfg.call, "N0CALL") != 0;
    movingCfg.move_threshold_m = cfg.move_m; movingCfg.stationary_repeat_min = cfg.stationary_min;
    movingCfg.repeat_messages = cfg.mv_messages; movingCfg.repeat_other = cfg.mv_other; movingCfg.repeat_unreadable = cfg.mv_unreadable;
    movingCfg.always_tocalls = cfg.always_tocalls;
    cadSlotMs = (4000UL * (1UL << cfg.sf) + cfg.bw_hz / 2) / cfg.bw_hz;
}

static float mhz(uint32_t hz) { return (float)(hz / 1000000u) + (float)(hz % 1000000u) / 1e6f; }

static bool seenRecently(uint32_t key, uint32_t now)
{
    for (int i = 0; i < DEDUPE_SLOTS; i++)
        if (seen[i].ms && seen[i].key == key && now - seen[i].ms < cfg.dedupe_s * 1000UL) return true;
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
// The radio settings that can change while running (frequency, bandwidth, SF, CR, power, sync word, preamble, gain).
static int16_t radioSetParams(const d3_settings_t &c)
{
    int16_t st = radio.setFrequency(mhz(c.freq_hz));
    if (st == RADIOLIB_ERR_NONE) st = radio.setBandwidth(c.bw_hz / 1000.0f);
    if (st == RADIOLIB_ERR_NONE) st = radio.setSpreadingFactor(c.sf);
    if (st == RADIOLIB_ERR_NONE) st = radio.setCodingRate(c.cr);
    if (st == RADIOLIB_ERR_NONE) st = radio.setOutputPower(c.power_dbm);
    if (st == RADIOLIB_ERR_NONE) st = radio.setSyncWord(c.sync);
    if (st == RADIOLIB_ERR_NONE) st = radio.setPreambleLength(c.preamble);
    if (st == RADIOLIB_ERR_NONE) st = radio.setRxBoostedGainMode(c.boosted ? true : false);
    return st;
}

static bool radioDiffers()
{
    return cfg.freq_hz != radioApplied.freq_hz || cfg.bw_hz != radioApplied.bw_hz || cfg.sf != radioApplied.sf || cfg.cr != radioApplied.cr ||
           cfg.power_dbm != radioApplied.power_dbm || cfg.sync != radioApplied.sync || cfg.preamble != radioApplied.preamble ||
           cfg.boosted != radioApplied.boosted;
}

// Start the radio from cfg. Returns the RadioLib status; does not reset (the caller decides what to do about a failure).
static int16_t radioInit()
{
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    LOG(F("[Radio] init... "));
    int16_t st = radio.begin(mhz(cfg.freq_hz), cfg.bw_hz / 1000.0f, cfg.sf, cfg.cr);
    if (st == RADIOLIB_ERR_NONE) st = radio.setTCXO(TCXO_VOLTAGE);
    if (st == RADIOLIB_ERR_NONE) st = radio.setCurrentLimit(LORA_OCP_MA);
    if (st == RADIOLIB_ERR_NONE) st = radioSetParams(cfg);
    if (st == RADIOLIB_ERR_NONE) st = radio.setCRC(true);
    if (st == RADIOLIB_ERR_NONE) {
        radio.setPacketReceivedAction(onRadioRx);
        radioApplied = cfg;
        LOGLN(F("OK"));
    } else {
        OUT(F("[Radio] init failed, code ")); OUTLN(st);
    }
    return st;
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
    uint32_t now = millis();

    if (cfg.cancel_on_hear) {   // another digipeater repeated a packet we are still waiting to repeat: drop ours
        uint32_t key; int usedNow;
        if (d3_identify(fr, n, &key, &usedNow))
            for (int i = 0; i < DIGI_QUEUE_SIZE; i++)
                if (txq[i].used && txq[i].rxEndMs && txq[i].job.key == key && usedNow > txq[i].usedHops) {
                    txq[i].used = false; nCancelled++;
                    LOG(F("[Digi] cancelled, already repeated by another digi: ")); LOGLN(txq[i].job.text);
                }
    }

    TxSlot *slot = freeSlot();
    static d3_job_t scratch;                    // decided here first, so a full queue still counts the reason
    d3_job_t *job = slot ? &slot->job : &scratch;
    int r = callsignSet ? d3_decide(fr, n, myCall, cfg.mode, job) : D3_NOT_SERVED;
    const char *text = d3_last_text();          // the packet as received, if this digi could read it
    if (text[0]) d3_observe(&neighbors, text, strlen(text), rssi, snr, now);
    LOG(F("[RX] ")); LOG(n); LOG(F(" B ")); LOG(rssi, 0); LOG(F(" dBm ")); LOG(snr, 1); LOG(F(" dB ")); LOG((long)fo); LOG(F(" Hz: "));
    if (cfg.log_decoded && text[0] && n > 2 && fr[2] == 0x03) { LOG(F("decoded ")); LOGLN(text); LOG(F("      ")); }
    if (r != D3_OK) { nWhy[r]++; LOGLN(d3_reason(r)); return; }
    if (job->decoded) nDecoded++;
    job->convert = (job->fmt == D3_FMT_PLAIN && cfg.tx_lacf3) ? 1 : 0;
    if (seenRecently(job->key, now)) { nDup++; LOGLN(F("duplicate")); return; }
    remember(job->key, now);
    if (cfg.moving) {
        int md = d3_moving_decision(&neighbors, &movingCfg, text[0] ? text : nullptr, strlen(text), job->named, now);
        if (!d3_move_repeats(md)) {
            if (md == D3_MOVE_SKIP_STATIONARY) nSkipStationary++; else nSkipOther++;
            LOGLN(d3_move_reason(md));
            return;
        }
        LOG(d3_move_reason(md)); LOG(F(": "));
    }
    if (!slot) { nQueueFull++; LOGLN(F("queue full")); return; }
    slot->used = true; slot->freeChecks = 0; slot->waitStartMs = 0; slot->usedHops = job->used_hops;
    slot->dueMs = now + randomBetween(cfg.delay_min_ms, cfg.delay_max_ms);
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
        if (rxFlag) return;                     // a packet waiting to be read: the channel is busy, and it may cancel this repeat

        // Listen before talk: CAD_DIFS_SLOTS free checks in a row; a busy check backs off 1 to CAD_BACKOFF_MAX slots.
        if (!s.waitStartMs) s.waitStartMs = now ? now : 1;
        if (now - s.waitStartMs > cfg.cad_wait_s * 1000UL) {
            s.used = false; nBusyDrop++;
            LOGLN(F("[TX] channel busy too long, given up"));
            if (s.rxEndMs == 0) nextBeaconMs = now + 30000UL;      // our beacon: try again in 30 s
            return;
        }
        int16_t cad = radio.scanChannel();
        startRx();                              // keep receiving between checks
        if (cad != RADIOLIB_CHANNEL_FREE) {
            nBusy++; s.freeChecks = 0;
            s.dueMs = millis() + cadSlotMs * randomBetween(1, cfg.cad_backoff);
            return;
        }
        if (++s.freeChecks < cfg.cad_difs) { s.dueMs = millis(); return; }

        uint8_t out[D3_MAX_FRAME]; size_t len;
        if (s.job.fmt == D3_FMT_LACF3) {
            // Time to hop: from the end of reception to the end of our transmission. The length is the relayed length, so build once to learn it.
            uint8_t tup[4] = { 0, 0, 0, 0 };
            len = d3_build(&s.job, myCall, tup, out, sizeof out);
            float toaMs = len ? radio.getTimeOnAir(len) / 1000.0f : 0;
            d3_rxt_tuple(s.rssi, s.snr, s.fo, (float)(millis() - s.rxEndMs) + toaMs, cfg.sf, cfg.bw_hz / 1000.0f, tup);
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
static void latlon(char *out, size_t cap, int32_t lat_u, int32_t lon_u)
{
    uint32_t a = ((uint32_t)(lat_u < 0 ? -lat_u : lat_u) * 6u + 500u) / 1000u;       // hundredths of a minute
    uint32_t o = ((uint32_t)(lon_u < 0 ? -lon_u : lon_u) * 6u + 500u) / 1000u;
    snprintf(out, cap, "%02lu%02lu.%02lu%c%c%03lu%02lu.%02lu%c%c",
             (unsigned long)(a / 6000), (unsigned long)((a % 6000) / 100), (unsigned long)(a % 100), lat_u >= 0 ? 'N' : 'S', cfg.sym_table[0],
             (unsigned long)(o / 6000), (unsigned long)((o % 6000) / 100), (unsigned long)(o % 100), lon_u >= 0 ? 'E' : 'W', cfg.sym_code[0]);
}

static void queueBeacon()
{
    TxSlot *slot = freeSlot();
    if (!slot) return;
    char pos[32];
    latlon(pos, sizeof pos, cfg.lat_u, cfg.lon_u);
    int w = snprintf(slot->job.text, sizeof slot->job.text, "%s>%s%s%s:!%s%s", myCall, cfg.tocall,
                     cfg.path[0] ? "," : "", cfg.path, pos, cfg.comment);
    if (w <= 0 || w >= (int)sizeof slot->job.text) return;
    slot->job.fmt = D3_FMT_PLAIN; slot->job.text_len = (uint16_t)w; slot->job.rx_len = 0; slot->job.decoded = 0; slot->job.convert = cfg.tx_lacf3;
    slot->used = true; slot->freeChecks = 0; slot->waitStartMs = 0; slot->usedHops = 0; slot->dueMs = millis(); slot->rxEndMs = 0;
    LOG(F("[Beacon] ")); LOGLN(slot->job.text);
}

static void printStats()
{
    OUT(F("[Stats] heard ")); OUT(nHeard); OUT(F(", CRC errors ")); OUT(nCrc);
    OUT(F(", repeated plain ")); OUT(nRelayPlain); OUT(F(", relayed LACF3 ")); OUT(nRelayLacf3);
    OUT(F(", duplicates ")); OUT(nDup); OUT(F(", LACF2 skipped ")); OUT(nWhy[D3_LACF2]);
    OUT(F(", damaged ")); OUT(nWhy[D3_BAD_FRAME]); OUT(F(", busy drops ")); OUT(nBusyDrop);
    OUT(F(", queue full ")); OUT(nQueueFull); OUT(F(", beacons ")); OUT(nBeacons);
    OUT(F(", LACF3 decoded ")); OUT(nDecoded); OUT(F(", sent as LACF3 ")); OUT(nConverted);
    OUT(F(", channel busy ")); OUT(nBusy); OUT(F(", cancelled ")); OUT(nCancelled);
    OUT(F(", fixed stations skipped ")); OUT(nSkipStationary); OUT(F(", other skipped ")); OUTLN(nSkipOther);
    // The station list: heard on the air by this digi (signal), and where each station last reported itself.
    uint32_t now = millis();
    for (int i = 0; i < neighbors.n; i++) {
        const d3_station_t &st = neighbors.s[i];
        if (!st.call[0]) continue;
        OUT(F("  ")); OUT(st.call);
        if (st.rf_ms) { OUT(F("  heard ")); OUT((now - st.rf_ms) / 60000UL); OUT(F(" min ago ")); OUT(st.rssi10 / 10); OUT(F(" dBm ")); OUT(st.snr10 / 10.0f, 1); OUT(F(" dB")); OUT(st.direct_src ? F(" direct") : F("")); }
        if (st.has_pos) { OUT(F("  at ")); OUT(st.lat / 1e6, 5); OUT(F(",")); OUT(st.lon / 1e6, 5); OUT(F(" ")); OUT((now - st.pos_ms) / 60000UL); OUT(F(" min ago")); }
        if (st.has_rep) { OUT(F("  last repeated ")); OUT((now - st.rep_ms) / 60000UL); OUT(F(" min ago")); }
        OUTLN();
    }
}

// ============================================================
// SETTINGS IN FLASH: the last 2 KB page of the chip. platformio.ini lowers the linker's flash size by one page so the program
// can never grow into it. One record, with a CRC; an erased or damaged page just means "use the compiled-in defaults".
// ============================================================
#define FLASH_PAGE_BYTES 2048u

static uint32_t settingsAddr()
{
    uint32_t kb = *(const uint16_t *)FLASHSIZE_BASE;
    return kb == 256 ? FLASH_BASE + kb * 1024u - FLASH_PAGE_BYTES : 0;   // the linker reservation in platformio.ini assumes 256 KB
}

static void loadSettings()
{
    d3s_defaults(&cfg);
    uint32_t addr = settingsAddr();
    if (!addr) { OUTLN(F("[Settings] unexpected flash size: saving is off, using the compiled-in defaults")); return; }
    int rej = 0;
    int r = d3s_unpack((const uint8_t *)addr, D3S_REC_MAX, &cfg, &rej);
    if (r == D3S_E_BLANK)       OUTLN(F("[Settings] none saved yet, using the compiled-in defaults"));
    else if (r < 0)             OUTLN(F("[Settings] the saved record is damaged: using the compiled-in defaults"));
    else { OUT(F("[Settings] loaded ")); OUT(r); OUT(F(" from flash")); if (rej) { OUT(F(", ")); OUT(rej); OUT(F(" refused (out of range)")); } OUTLN(); }
}

static void saveSettings()
{
    uint32_t addr = settingsAddr();
    if (!addr) { OUTLN(F("error: this chip's flash size is not the 256 KB the settings page assumes")); return; }
    if (radioDiffers()) { OUTLN(F("error: radio settings changed but not applied: type apply first, so a setting the radio refuses is never saved")); return; }
    uint8_t rec[D3S_REC_MAX];
    size_t n = d3s_pack(&cfg, rec, sizeof rec);
    if (!n) { OUTLN(F("error: the record does not fit")); return; }
    if (memcmp((const void *)addr, rec, n) == 0) { dirty = false; OUTLN(F("already saved, nothing written")); return; }

    radio.standby();                            // no radio activity while the flash is erased and written
    IWatchdog.reload();
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef er = {};
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.Page      = (addr - FLASH_BASE) / FLASH_PAGE_BYTES;
    er.NbPages   = 1;
    uint32_t pageErr = 0;
    bool ok = HAL_FLASHEx_Erase(&er, &pageErr) == HAL_OK;
    for (size_t i = 0; ok && i < n; i += 8) {
        uint64_t dw; memcpy(&dw, rec + i, 8);
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dw) == HAL_OK;
    }
    HAL_FLASH_Lock();
    IWatchdog.reload();
    startRx();
    ok = ok && memcmp((const void *)addr, rec, n) == 0;          // read it back
    if (ok) { dirty = false; OUT(F("saved ")); OUT((unsigned long)n); OUTLN(F(" bytes to flash")); }
    else    { OUT(F("error: flash write failed, code ")); OUTLN((unsigned long)HAL_FLASH_GetError()); }
}

// ============================================================
// SERIAL COMMANDS
// ============================================================
static void emitLine(void *, const char *l) { OUTLN(l); }

static void applyRadio()
{
    radio.standby();
    int16_t st = radioSetParams(cfg);
    if (st == RADIOLIB_ERR_NONE) { radioApplied = cfg; OUTLN(F("ok: radio settings applied")); }
    else {
        OUT(F("error: the radio refused those settings (code ")); OUT(st); OUTLN(F("); going back to the ones in use"));
        cfg.freq_hz = radioApplied.freq_hz; cfg.bw_hz = radioApplied.bw_hz; cfg.sf = radioApplied.sf; cfg.cr = radioApplied.cr;
        cfg.power_dbm = radioApplied.power_dbm; cfg.sync = radioApplied.sync; cfg.preamble = radioApplied.preamble; cfg.boosted = radioApplied.boosted;
        radioSetParams(cfg);
    }
    applySettings();
    startRx();
}

static void handleCommand(const char *line)
{
    char before[16]; strcpy(before, myCall);
    unsigned fl = d3s_exec(&cfg, line, emitLine, nullptr);
    if (fl & D3S_CHANGED) {
        applySettings();
        if (strcmp(before, myCall) != 0) memset(txq, 0, sizeof txq);      // packets queued under the old callsign are dropped
        if (!dirty) OUTLN(F("(not saved yet: type save to keep it after a power-off)"));
        dirty = true;
        if (!callsignSet) OUTLN(F("(callsign is still N0CALL: nothing is relayed or beaconed)"));
    }
    if (fl & D3S_APPLY)  applyRadio();
    if (fl & D3S_SAVE)   saveSettings();
    if (fl & D3S_STATS)  printStats();
    if (fl & D3S_BEACON) {
        if (!callsignSet) OUTLN(F("error: set a callsign first"));
        else if (cfg.lat_u == 0 && cfg.lon_u == 0) OUTLN(F("error: no position set (set lat, set lon)"));
        else { queueBeacon(); OUTLN(F("ok: beacon queued")); }
    }
    if (fl & D3S_REBOOT) {
        if (dirty && !(fl & D3S_FORCE)) OUTLN(F("there are unsaved changes: type save first, or reboot force to drop them"));
        else { OUTLN(F("restarting...")); Serial.flush(); delay(50); NVIC_SystemReset(); }
    }
}

static char    lineBuf[D3S_LINE_MAX + 1];
static uint8_t lineLen = 0;
static bool    lineOver = false;

static void serialPoll()
{
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (lineLen || lineOver) {
                OUTLN();
                lineBuf[lineLen] = 0;
                if (lineOver) OUTLN(F("error: line too long")); else handleCommand(lineBuf);
                lineLen = 0; lineOver = false;
            }
        } else if (c == 8 || c == 127) {
            if (lineLen) { lineLen--; OUT(F("\b \b")); }
        } else if (c >= 32 && c < 127) {
            if (lineLen < D3S_LINE_MAX) { lineBuf[lineLen++] = (char)c; Serial.write((uint8_t)c); } else lineOver = true;
        }
    }
}

// ============================================================
void setup()
{
    Serial.begin(DEBUG_BAUD);                   // always on: the settings commands use it
    delay(300);
    OUTLN(F("=== LACF 3 digipeater with the tables (STM32WLE5), spec " HB3_SPEC_VERSION " ==="));
    if (IWatchdog.isReset(true)) OUTLN(F("[Boot] restarted by the watchdog"));
    IWatchdog.begin((uint32_t)WATCHDOG_S * 1000000UL);

    loadSettings();
    applySettings();
    OUT(F("[Boot] callsign ")); OUTLN(myCall);
    if (!callsignSet) OUTLN(F("[Boot] callsign is N0CALL: nothing is relayed or beaconed. Type  set call YOURCALL-SSID  then  save  (help lists everything)"));
    OUTLN();
    d3s_menu(emitLine, nullptr);
    OUTLN();

    memset(txq, 0, sizeof txq);
    d3_neighbors_init(&neighbors, stationSlots, NEIGHBOR_SLOTS);
    memset(seen, 0, sizeof seen);
    OUT(F("[Boot] LACF destination list: ")); OUT(d3_init_model()); OUTLN(F(" names"));
    if (radioInit() != RADIOLIB_ERR_NONE) {
        // a saved radio setting the chip refuses must not leave a remote site dead: go back to the compiled-in radio values
        d3_settings_t def; d3s_defaults(&def);
        cfg.freq_hz = def.freq_hz; cfg.bw_hz = def.bw_hz; cfg.sf = def.sf; cfg.cr = def.cr; cfg.power_dbm = def.power_dbm;
        cfg.sync = def.sync; cfg.preamble = def.preamble; cfg.boosted = def.boosted;
        applySettings();
        OUTLN(F("[Radio] trying the compiled-in radio settings"));
        if (radioInit() != RADIOLIB_ERR_NONE) { delay(2000); NVIC_SystemReset(); }   // try again from the start instead of hanging
        OUTLN(F("[Radio] running on the compiled-in radio settings; the saved ones were refused"));
    }
    randomSeed(radio.random(0x7FFFFFFF));
    startRx();

    nextBeaconMs = millis() + cfg.beacon_first_s * 1000UL;
    nextStatsMs  = millis() + (cfg.stats_min ? cfg.stats_min : 10) * 60000UL;
}

void loop()
{
    IWatchdog.reload();
    serialPoll();
    if (rxFlag) handleRx();
    transmitDue();

    uint32_t now = millis();
    if (callsignSet && (cfg.lat_u != 0 || cfg.lon_u != 0) && (int32_t)(now - nextBeaconMs) >= 0) {
        queueBeacon();
        nextBeaconMs = now + cfg.beacon_min * 60000UL;
    }
    if ((int32_t)(now - nextStatsMs) >= 0) {
        if (cfg.stats_min && cfg.log) printStats();
        nextStatsMs = now + (cfg.stats_min ? cfg.stats_min : 10) * 60000UL;
    }

    // Sleep until the next interrupt (the radio, the serial port, or the 1 ms system tick). The radio keeps listening.
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}
