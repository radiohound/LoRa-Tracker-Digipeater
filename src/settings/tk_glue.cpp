// Flash record, serial command line and boot lines for the tracker settings. See tk_glue.h.
#include <Arduino.h>
#include <RadioLib.h>
#include <string.h>
#include "config.h"
#include "tk_glue.h"
#if HORUS_ENABLED
#include "../horus/horus.h"
#endif

extern STM32WLx radio;                  // defined in main.cpp

tk_settings_t tkCfg;
volatile bool tkBeaconNow = false;
volatile bool tkHorusNow  = false;

static tk_settings_t radioApplied;      // the LoRa settings the radio is running with
static bool dirty     = false;          // changed since the last save
static bool setupMode = false;          // in the boot window: stay in setup until "exit"

#define OUT(...)   Serial.print(__VA_ARGS__)
#define OUTLN(...) Serial.println(__VA_ARGS__)

// ---------------------------------------------------------------- flash: the last 2 KB page of the chip
// platformio.ini lowers the allowed program size by one page so the program can never grow into it. One record with a CRC;
// an erased or damaged page just means "use the defaults from config.h".
#define FLASH_PAGE_BYTES 2048u

static uint32_t settingsAddr() {
    uint32_t kb = *(const uint16_t *)FLASHSIZE_BASE;
    return kb == 256 ? FLASH_BASE + kb * 1024u - FLASH_PAGE_BYTES : 0;      // the reservation in platformio.ini assumes 256 KB
}

static bool writeRecord(const uint8_t *rec, size_t n) {
    uint32_t addr = settingsAddr();
    if (!addr) return false;
    if (memcmp((const void *)addr, rec, n) == 0) return true;
    radio.standby();                                // no radio activity while the flash is erased and written
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
    return ok && memcmp((const void *)addr, rec, n) == 0;
}

static bool radioDiffers() {
    return tkCfg.freq_hz != radioApplied.freq_hz || tkCfg.bw_hz != radioApplied.bw_hz || tkCfg.sf != radioApplied.sf ||
           tkCfg.cr != radioApplied.cr || tkCfg.power_dbm != radioApplied.power_dbm || tkCfg.sync != radioApplied.sync ||
           tkCfg.preamble != radioApplied.preamble;
}

static void saveSettings() {
    uint32_t addr = settingsAddr();
    if (!addr) { OUTLN(F("error: this chip's flash size is not the 256 KB the settings page assumes")); return; }
    if (radioDiffers()) { OUTLN(F("error: radio settings changed but not applied: type apply first, so a setting the radio refuses is never saved")); return; }
    uint8_t rec[TKS_REC_MAX];
    size_t n = tks_pack(&tkCfg, rec, sizeof rec);
    if (!n) { OUTLN(F("error: the record does not fit")); return; }
    if (memcmp((const void *)addr, rec, n) == 0) { dirty = false; OUTLN(F("already saved, nothing written")); return; }
    if (writeRecord(rec, n)) { dirty = false; OUT(F("saved ")); OUT((unsigned long)n); OUTLN(F(" bytes to flash")); }
    else { OUT(F("error: flash write failed, code ")); OUTLN((unsigned long)HAL_FLASH_GetError()); }
}

// ---------------------------------------------------------------- Horus
void tkPushHorus() {
#if HORUS_ENABLED
    HorusSettings& h = horusSettings();
    h.version       = tkCfg.horus_ver;
    tks_horus_call(&tkCfg, h.callsign, sizeof h.callsign);
    h.payloadId     = tkCfg.horus_id;
    h.freqMhz       = tkCfg.horus_freq_hz / 1e6f;
    h.powerDbm      = tkCfg.horus_power_dbm;
    h.syncSeconds   = tkCfg.horus_sync_s;
    h.offsetSeconds = tkCfg.horus_off_s;
    h.beaconDivider = tkCfg.horus_div;
#endif
}

// ---------------------------------------------------------------- boot
void tkSettingsBegin() {
#if !DEBUG_SERIAL
    Serial.begin(DEBUG_BAUD);                       // the settings commands use the serial port even without debug output
#endif
    tks_defaults(&tkCfg);
    uint32_t addr = settingsAddr();
    if (!addr) {
        OUTLN(F("[Settings] unexpected flash size: saving is off, using the defaults from config.h"));
    } else {
        int rej = 0;
        int r = tks_unpack((const uint8_t *)addr, TKS_REC_MAX, &tkCfg, &rej);
        if (r == TKS_E_BLANK)      OUTLN(F("[Settings] none saved yet, using the defaults from config.h"));
        else if (r < 0)            OUTLN(F("[Settings] the saved record is damaged: using the defaults from config.h"));
        else { OUT(F("[Settings] loaded ")); OUT(r); OUT(F(" from flash")); if (rej) { OUT(F(", ")); OUT(rej); OUT(F(" refused (out of range)")); } OUTLN(); }
    }
    radioApplied = tkCfg;
    tkOnSettingsChanged();
    tkPushHorus();
    char fc[16]; tks_fullcall(&tkCfg, fc, sizeof fc);
    OUT(F("[Boot] callsign ")); OUTLN(fc);
    if (!tks_call_is_set(&tkCfg)) OUTLN(F("[Boot] callsign is N0CALL: nothing is transmitted. Type  set call YOURCALL-SSID  then  save"));
#if !HORUS_ENABLED
    if (tkCfg.horus) OUTLN(F("[Boot] horus is on, but this firmware was built without Horus"));
#endif
}

// ---------------------------------------------------------------- the command line
static void emitLine(void *, const char *l) { OUTLN(l); }

static void handleCommand(const char *line) {
    if (strcasecmp(line, "exit") == 0 || strcasecmp(line, "run") == 0) {
        if (setupMode) { setupMode = false; OUTLN(F("ok: running")); } else OUTLN(F("(already running)"));
        return;
    }
    unsigned fl = tks_exec(&tkCfg, line, emitLine, nullptr);
    if (fl & TKS_CHANGED) {
        tkOnSettingsChanged();
        tkPushHorus();
        if (!dirty) OUTLN(F("(not saved yet: type save to keep it after a power-off)"));
        dirty = true;
        if (!tks_call_is_set(&tkCfg)) OUTLN(F("(callsign is still N0CALL: nothing is transmitted)"));
    }
    if (fl & TKS_APPLY) {
        tkApplyRadio();
        radioApplied = tkCfg;
        OUTLN(F("ok: radio settings applied"));
    }
    if (fl & TKS_SAVE) saveSettings();
    if (fl & TKS_BEACON) {
        if (!tks_call_is_set(&tkCfg)) OUTLN(F("error: set a callsign first"));
        else { tkBeaconNow = true; OUTLN(F("ok: beacon cycle starts now")); }
    }
    if (fl & TKS_HORUS) {
        if (!tks_call_is_set(&tkCfg)) OUTLN(F("error: set a callsign first"));
#if HORUS_ENABLED
        else if (!tkCfg.horus) OUTLN(F("error: horus is off (set horus on)"));
        else { tkBeaconNow = tkHorusNow = true; OUTLN(F("ok: a Horus packet follows the next beacon cycle, which starts now")); }
#else
        else OUTLN(F("error: this firmware was built without Horus"));
#endif
    }
    if (fl & TKS_REBOOT) {
        if (dirty && !(fl & TKS_FORCE)) OUTLN(F("there are unsaved changes: type save first, or reboot force to drop them"));
        else { OUTLN(F("restarting...")); Serial.flush(); delay(50); NVIC_SystemReset(); }
    }
}

static char    lineBuf[TKS_LINE_MAX + 1];
static uint8_t lineLen = 0;
static bool    lineOver = false;

void tkSerialPoll() {
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
            if (lineLen < TKS_LINE_MAX) { lineBuf[lineLen++] = (char)c; Serial.write((uint8_t)c); } else lineOver = true;
        }
    }
}

void tkBootWindow(uint32_t ms) {
    OUT(F("[Boot] press a key within ")); OUT(ms / 1000); OUTLN(F(" s to change settings (help lists the commands)"));
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (Serial.available() > 0) { setupMode = true; break; }
        delay(10);
    }
    if (!setupMode) return;
    OUTLN(); tks_menu(emitLine, nullptr);
    OUTLN(F("Setup mode: type exit to start running, or reboot."));
    while (setupMode) { tkSerialPoll(); delay(5); }
}
