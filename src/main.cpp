// ============================================================
//  STM32WLE5 Low-Power LoRa APRS Tracker + Digipeater
//  Target: Ebyte E77-400MBL-01 (STM32WLE5CC)
//
//  Radio init sequence verified against PicoTrack by K6ATV:
//    https://github.com/radiohound/PicoTrack
//
//  Key differences from PicoTrack (intentional):
//    - Uses APRSPacketLib for packet encoding/decoding (enables
//      digipeater path manipulation on received packets).
//    - Adds digipeater and digi-only operating modes.
//    - Adds GPS power-gating option.
//    - Re-uses PicoTrack's proven radio init sequence exactly:
//        setRfSwitchTable → begin() → aprs/radio setup →
//        setTCXO → setOutputPower → setCurrentLimit
//
//  GPS wiring (E77 dev board, per PicoTrack):
//    PA9  = I2C SCL
//    PA10 = I2C SDA
//    Uses Wire (I2C), not UART.
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <RadioLib.h>
#include <APRSPacketLib.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>

#include "config.h"

// ============================================================
//  DEBUG HELPERS
// ============================================================
#if DEBUG_SERIAL
    #define DBG(...)   Serial.print(__VA_ARGS__)
    #define DBGLN(...) Serial.println(__VA_ARGS__)
#else
    #define DBG(...)
    #define DBGLN(...)
#endif

// ============================================================
//  RADIO
//  STM32WLx: no SPI pins needed — radio is internal to chip.
// ============================================================
STM32WLx radio = new STM32WLx_Module();

static const uint32_t rfswitch_pins[] = RFSWITCH_PINS;
static const Module::RfSwitchMode_t rfswitch_table[] = { RFSWITCH_TABLE };

// ============================================================
//  GPS  (I2C, matching PicoTrack wiring PA9=SCL, PA10=SDA)
// ============================================================
SFE_UBLOX_GNSS myGPS;

// ============================================================
//  RTC
// ============================================================
STM32RTC& rtc = STM32RTC::getInstance();

// ============================================================
//  STATE
// ============================================================
static char myCallsignFull[12];   // "N0CALL-9"

struct Position {
    float    lat    = 0.0f;
    float    lon    = 0.0f;
    float    altM   = 0.0f;   // metres MSL
    int      speed  = 0;      // knots
    int      course = 0;      // degrees
    uint8_t  sats   = 0;
    bool     valid  = false;
};
static Position lastPos;

// Digipeater RX flag — set from ISR
volatile bool radioRxFlag = false;

// Duplicate suppression ring buffer (CRC32)
static uint32_t dedupe[8];
static uint8_t  dedupeIdx = 0;

// Pending retransmit queue
struct DigiQueueEntry {
    String   packet;
    uint32_t txAfterMs;
    bool     used;
};
static DigiQueueEntry digiQueue[DIGI_QUEUE_SIZE];

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void radioAndGpsInit();
void radioSleep();
void radioStartRx();
bool radioTransmit(const String& packet);
void IRAM_ATTR onRadioRx();

bool gpsBegin();
bool waitForFix(uint32_t timeoutMs);
void gpsPowerOn();
void gpsPowerOff();

void     sendBeacon();
String   buildAPRSPacket();

void     processRxPacket();
bool     needsDigipeat(const String& raw, String& modified);
bool     isDuplicate(const String& raw);
void     markDuplicate(const String& raw);
uint32_t crc32(const String& s);
void     enqueueForDigipeat(const String& pkt);
void     flushDigiQueue();

void     enterDeepSleep(uint32_t ms);

// ============================================================
//  RADIO + GPS INITIALISATION
//
//  IMPORTANT: This sequence matches PicoTrack exactly:
//    1. setRfSwitchTable (must be before begin())
//    2. radio.begin(freq, bw, sf, cr)
//    3. radio.setTCXO(voltage)       ← AFTER begin(), per PicoTrack
//    4. radio.setOutputPower(dBm)    ← AFTER setTCXO()
//    5. radio.setCurrentLimit(mA)    ← AFTER setOutputPower()
//
//  PicoTrack calls setup() again from the top of loop() after
//  waking from deepSleep because STM32 peripherals need
//  re-initialisation post-Stop2. We do the same.
// ============================================================
void radioAndGpsInit() {
    Serial.begin(DEBUG_BAUD);

    snprintf(myCallsignFull, sizeof(myCallsignFull), "%s-%d",
             MY_CALLSIGN, MY_SSID);

    // --- GPS power pin ---
#if GPS_POWER_PIN != RADIOLIB_NC
    pinMode(GPS_POWER_PIN, OUTPUT);
    gpsPowerOn();
#endif

    // --- GPS over I2C (PA9=SCL, PA10=SDA on E77 dev board) ---
    Wire.begin();
    Wire.setClock(GPS_I2C_CLOCK);

    // --- Radio: RF switch must be set before begin() ---
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

    DBG(F("[STM32WL] Initializing ... "));
    // Parameters matching PicoTrack: freq, BW, SF, CR.
    // Power, preamble, sync word are NOT passed here —
    // they are set individually below, as PicoTrack does.
    int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("failed, code ")); DBGLN(state);
        while (true) { delay(10); }
    }
    DBGLN(F("success!"));

    // --- TCXO: called AFTER begin(), matching PicoTrack ---
    // PicoTrack comment: "set appropriate TCXO voltage for
    // Nucleo WL55JC1, WL55JC2, or E77 boards"
    DBG(F("Set TCXO voltage ... "));
    state = radio.setTCXO(TCXO_VOLTAGE);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("failed, code ")); DBGLN(state);
        while (true) { delay(10); }
    }
    DBGLN(F("success!"));

    // --- Output power: set after TCXO, matching PicoTrack ---
    if (radio.setOutputPower(LORA_TX_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        DBGLN(F("Invalid output power!"));
        while (true) { delay(10); }
    }

    // --- OCP: required for full TX power (confirmed by PicoTrack author) ---
    if (radio.setCurrentLimit(LORA_OCP_MA) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        DBGLN(F("Invalid current limit!"));
        while (true) { delay(10); }
    }

    // --- Sync word and preamble (not in PicoTrack but harmless to set) ---
    radio.setSyncWord(LORA_SYNC_WORD);
    radio.setPreambleLength(LORA_PREAMBLE);

    // --- RX interrupt callback ---
    radio.setPacketReceivedAction(onRadioRx);

    DBGLN(F("[Radio] Init complete."));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    LowPower.begin();
    rtc.begin();

    memset(digiQueue, 0, sizeof(digiQueue));
    memset(dedupe,    0, sizeof(dedupe));

    radioAndGpsInit();

#if OPERATING_MODE == MODE_DIGI_ONLY
    DBGLN(F("Mode: DIGI_ONLY"));
    radioStartRx();
#endif
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
    // PicoTrack calls setup() again at the top of every loop
    // iteration after waking from deepSleep, because peripherals
    // (Serial, Wire, radio) need re-initialisation after Stop2.
    // We do the same for TRACKER_ONLY; other modes stay awake.

#if OPERATING_MODE == MODE_TRACKER_ONLY
    // Re-init after wakeup (matches PicoTrack pattern)
    radioAndGpsInit();

    sendBeacon();

    radioSleep();
    enterDeepSleep((uint32_t)BEACON_INTERVAL_S * 1000UL);

#elif OPERATING_MODE == MODE_TRACKER_DIGI
    // Re-init after wakeup
    radioAndGpsInit();

    sendBeacon();

    // After TX, stay in RX for the rest of the interval.
    radioStartRx();
    uint32_t intervalEnd = millis() + (uint32_t)BEACON_INTERVAL_S * 1000UL;
    while (millis() < intervalEnd) {
        if (radioRxFlag) {
            radioRxFlag = false;
            processRxPacket();
        }
        flushDigiQueue();
        LowPower.idle(10);
    }

#elif OPERATING_MODE == MODE_DIGI_ONLY
    if (radioRxFlag) {
        radioRxFlag = false;
        processRxPacket();
    }
    flushDigiQueue();
    LowPower.idle(10);
#endif
}

// ============================================================
//  RADIO HELPERS
// ============================================================
void radioSleep() {
    radio.sleep();
}

void radioStartRx() {
    int state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("[RX] startReceive failed, code ")); DBGLN(state);
    }
}

// Transmit a LoRa-APRS packet.
// LoRa-APRS wire format: 3-byte header "<\xff\x01" + TNC2 string.
bool radioTransmit(const String& tnc2) {
    String pkt = "<\xff\x01" + tnc2;
    DBG(F("[TX] ")); DBGLN(pkt);
    int state = radio.transmit(pkt);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("[TX] failed, code ")); DBGLN(state);
        return false;
    }
    return true;
}

void IRAM_ATTR onRadioRx() {
    radioRxFlag = true;
}

// ============================================================
//  GPS HELPERS
// ============================================================
void gpsPowerOn() {
#if GPS_POWER_PIN != RADIOLIB_NC
    digitalWrite(GPS_POWER_PIN, HIGH);
    delay(100);
#endif
}

void gpsPowerOff() {
#if GPS_POWER_PIN != RADIOLIB_NC
    digitalWrite(GPS_POWER_PIN, LOW);
#endif
}

bool gpsBegin() {
    if (!myGPS.begin()) {
        DBGLN(F("[GPS] Not detected on I2C. Check wiring (PA9=SCL, PA10=SDA)."));
        return false;
    }
    // Match PicoTrack GPS setup exactly
    myGPS.setUART1Output(0);
    myGPS.setUART2Output(0);
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(1);
    myGPS.setDynamicModel(DYN_MODEL_AIRBORNE1g);  // 50 km altitude limit
    myGPS.saveConfiguration();
    DBGLN(F("[GPS] OK"));
    return true;
}

bool waitForFix(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        myGPS.checkUblox();
        if (myGPS.getPVT() && myGPS.getFixType() != 0 && myGPS.getSIV() > 3) {
            lastPos.lat    = myGPS.getLatitude()  / 1e7f;
            lastPos.lon    = myGPS.getLongitude() / 1e7f;
            lastPos.altM   = myGPS.getAltitudeMSL() / 1000.0f;
            lastPos.speed  = (int)(myGPS.getGroundSpeed() / 514.44f);
            lastPos.course = myGPS.getHeading() / 100000;
            lastPos.sats   = myGPS.getSIV();
            lastPos.valid  = true;
            return true;
        }
        delay(100);
    }
    DBGLN(F("[GPS] Timeout — no fix."));
    return false;
}

// ============================================================
//  APRS BEACON
// ============================================================
String buildAPRSPacket() {
    // Build a TNC2-format APRS position packet using APRSPacketLib.
    // APRSPacketLib is used here instead of RadioLib's APRSClient
    // because it also handles decoding received packets for digipeating.
    APRSPacket pkt;
    pkt.source    = String(myCallsignFull);
    pkt.path      = String(BEACON_PATH);
    pkt.symbol    = APRS_SYMBOL_CODE;
    pkt.overlay   = APRS_SYMBOL_TABLE;
    pkt.latitude  = lastPos.lat;
    pkt.longitude = lastPos.lon;
    pkt.altitude  = (int)(lastPos.altM * 3.28084f);  // metres → feet
    pkt.speed     = lastPos.speed;
    pkt.course    = lastPos.course;
    pkt.comment   = String(MY_COMMENT);
    return APRSPacketLib::generatePositionPacket(pkt);
}

void sendBeacon() {
    DBGLN(F("[Beacon] Getting GPS fix..."));
    gpsPowerOn();

    bool gotFix = false;
    if (gpsBegin()) {
        gotFix = waitForFix(GPS_FIX_TIMEOUT_MS);
    }

    if (!gotFix) {
        DBGLN(F("[Beacon] No fix — using last known position."));
        if (!lastPos.valid) {
            DBGLN(F("[Beacon] No position available — skipping TX."));
            // Don't power off if GPS is always-on (GPS_POWER_PIN == NC)
#if GPS_POWER_PIN != RADIOLIB_NC
            gpsPowerOff();
#endif
            return;
        }
    }

#if GPS_POWER_PIN != RADIOLIB_NC
    gpsPowerOff();
    DBGLN(F("[GPS] Powered off."));
#endif

    String packet = buildAPRSPacket();
    radioTransmit(packet);
}

// ============================================================
//  DIGIPEATER
// ============================================================
void processRxPacket() {
    String raw;
    int state = radio.readData(raw);

    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("[Digi] readData failed, code ")); DBGLN(state);
        radioStartRx();
        return;
    }

    // Strip LoRa-APRS 3-byte header "<\xff\x01"
    if (raw.length() > 3 && raw[0] == '<') {
        raw = raw.substring(3);
    }

    DBG(F("[RX] ")); DBGLN(raw);
    DBG(F("     RSSI=")); DBG(radio.getRSSI());
    DBG(F(" SNR=")); DBGLN(radio.getSNR());

    String modified;
    if (needsDigipeat(raw, modified)) {
        if (!isDuplicate(raw)) {
            markDuplicate(raw);
            enqueueForDigipeat(modified);
        } else {
            DBGLN(F("[Digi] Duplicate — dropped."));
        }
    }

    radioStartRx();
}

// Parse TNC2 path and build a modified packet for retransmission.
// TNC2 format: SOURCE>DEST,PATH:PAYLOAD
bool needsDigipeat(const String& raw, String& modified) {
    int colonIdx = raw.indexOf(':');
    if (colonIdx < 0) return false;

    String header  = raw.substring(0, colonIdx);
    String payload = raw.substring(colonIdx);  // includes ':'

    int arrowIdx = header.indexOf('>');
    if (arrowIdx < 0) return false;

    String source      = header.substring(0, arrowIdx);
    String destAndPath = header.substring(arrowIdx + 1);

    // Don't digipeat our own packets
    if (source.startsWith(MY_CALLSIGN)) return false;

    int commaIdx = destAndPath.indexOf(',');
    if (commaIdx < 0) return false;  // no path at all

    String dest = destAndPath.substring(0, commaIdx);
    String path = destAndPath.substring(commaIdx + 1);

    // Split path into fields
    String fields[8];
    int    nFields = 0;
    {
        String rem = path;
        while (rem.length() > 0 && nFields < 8) {
            int sep = rem.indexOf(',');
            if (sep < 0) { fields[nFields++] = rem; break; }
            fields[nFields++] = rem.substring(0, sep);
            rem = rem.substring(sep + 1);
        }
    }

    bool digipeated = false;
    for (int i = 0; i < nFields && !digipeated; i++) {
        String& f = fields[i];
        if (f.endsWith("*")) continue;  // already used

        if (f.equalsIgnoreCase(DIGI_PATH_1)) {
            fields[i]  = String(myCallsignFull) + "*";
            digipeated = true;
        } else if (strlen(DIGI_PATH_2) > 0 &&
                   (f.startsWith("WIDE2-") || f.startsWith("wide2-"))) {
            int n = f.substring(6).toInt();
            if (n >= 1) {
                if (n == 1) {
                    fields[i] = String(myCallsignFull) + "*";
                } else {
                    // Insert our callsign and decrement hop count
                    if (nFields < 8) {
                        for (int j = nFields; j > i; j--) fields[j] = fields[j-1];
                        fields[i] = String(myCallsignFull) + "*";
                        fields[i+1] = "WIDE2-" + String(n - 1);
                        nFields++;
                        i++;
                    }
                }
                digipeated = true;
            }
        }
    }

    if (!digipeated) return false;

    String newPath;
    for (int i = 0; i < nFields; i++) {
        if (i > 0) newPath += ',';
        newPath += fields[i];
    }

    modified = source + ">" + dest + "," + newPath + payload;
    return true;
}

uint32_t crc32(const String& s) {
    uint32_t c = 0xFFFFFFFF;
    for (size_t i = 0; i < s.length(); i++) {
        c ^= (uint8_t)s[i];
        for (int b = 0; b < 8; b++)
            c = (c & 1) ? (c >> 1) ^ 0xEDB88320 : (c >> 1);
    }
    return ~c;
}

bool isDuplicate(const String& raw) {
    uint32_t h = crc32(raw);
    for (int i = 0; i < 8; i++) if (dedupe[i] == h) return true;
    return false;
}

void markDuplicate(const String& raw) {
    dedupe[dedupeIdx] = crc32(raw);
    dedupeIdx = (dedupeIdx + 1) & 7;
}

void enqueueForDigipeat(const String& pkt) {
    for (int i = 0; i < DIGI_QUEUE_SIZE; i++) {
        if (!digiQueue[i].used) {
            uint32_t d = (uint32_t)random(DIGI_DELAY_MIN_MS, DIGI_DELAY_MAX_MS + 1);
            digiQueue[i].packet    = pkt;
            digiQueue[i].txAfterMs = millis() + d;
            digiQueue[i].used      = true;
            DBG(F("[Digi] Queued, delay=")); DBG(d); DBGLN(F("ms"));
            return;
        }
    }
    DBGLN(F("[Digi] Queue full — dropped."));
}

void flushDigiQueue() {
    for (int i = 0; i < DIGI_QUEUE_SIZE; i++) {
        if (digiQueue[i].used && millis() >= digiQueue[i].txAfterMs) {
            radio.standby();
            radioTransmit(digiQueue[i].packet);
            digiQueue[i].used = false;
            radioStartRx();
        }
    }
}

// ============================================================
//  POWER MANAGEMENT
// ============================================================
void enterDeepSleep(uint32_t ms) {
    DBG(F("[Sleep] deepSleep ")); DBG(ms / 1000); DBGLN(F("s"));
#if DEBUG_SERIAL
    Serial.flush();
    delay(10);
#endif
    LowPower.deepSleep(ms);
    // After wakeup: loop() calls radioAndGpsInit() to re-init peripherals.
}
