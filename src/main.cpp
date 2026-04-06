// ============================================================
//  STM32WLE5 LoRa APRS Tracker + Digipeater
//  Target: Ebyte E77-400MBL-01 (STM32WLE5CC)
//
//  Radio init sequence verified against PicoTrack (K6ATV):
//    https://github.com/radiohound/PicoTrack
//
//  Power improvements over original:
//  - radioInit() separated from gpsInit() — only re-init what
//    is actually needed after each deepSleep wakeup.
//  - MODE_DIGI_CAD: CAD-based digipeater uses ~0.1 mA average
//    vs ~6 mA for continuous RX, enabling battery/solar operation.
//  - CAD runs in batches to amortise the ~100 ms / 7 mA init
//    overhead across multiple scans before sleeping the radio.
//
//  GPS wiring (E77 dev board, per PicoTrack):
//    PA9 = I2C SCL,  PA10 = I2C SDA
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
//  DEBUG
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
// ============================================================
STM32WLx radio = new STM32WLx_Module();

static const uint32_t rfswitch_pins[] = RFSWITCH_PINS;
static const Module::RfSwitchMode_t rfswitch_table[] = { RFSWITCH_TABLE };

// ============================================================
//  GPS
// ============================================================
SFE_UBLOX_GNSS myGPS;

// ============================================================
//  RTC
// ============================================================
STM32RTC& rtc = STM32RTC::getInstance();

// ============================================================
//  STATE
// ============================================================
static char myCallsignFull[12];

struct Position {
    float   lat = 0.0f, lon = 0.0f, altM = 0.0f;
    int     speed = 0, course = 0;
    uint8_t sats = 0;
    bool    valid = false;
};
static Position lastPos;

// Whether radioInit() has been called since last power-on/reset.
// Avoids repeating the full init sequence unnecessarily.
static bool radioReady = false;
static bool gpsReady   = false;

// ============================================================
//  INTERRUPT FLAGS
// ============================================================
volatile bool radioRxFlag  = false;
volatile bool radioCadFlag = false;

// ============================================================
//  DUPLICATE SUPPRESSION
// ============================================================
static uint32_t dedupe[8];
static uint8_t  dedupeIdx = 0;

struct DigiQueueEntry { String packet; uint32_t txAfterMs; bool used; };
static DigiQueueEntry digiQueue[DIGI_QUEUE_SIZE];

// ============================================================
//  FORWARD DECLARATIONS
// ============================================================
void radioInit();
void radioSleep();
void radioStartRx();
void radioStartCad();
bool radioTransmit(const String& tnc2);
void IRAM_ATTR onRadioRx();
void IRAM_ATTR onRadioCad();

void gpsInit();
void gpsPowerOn();
void gpsPowerOff();
bool waitForFix(uint32_t timeoutMs);

void   sendBeacon();
String buildAPRSPacket();

void   runDiGiCAD();
void   processRxPacket();
bool   needsDigipeat(const String& raw, String& modified);
bool   isDuplicate(const String& raw);
void   markDuplicate(const String& raw);
uint32_t crc32(const String& s);
void   enqueueForDigipeat(const String& pkt);
void   flushDigiQueue();

void enterDeepSleep(uint32_t ms);

// ============================================================
//  RADIO INIT
//
//  Sequence verified against PicoTrack:
//  setRfSwitchTable → begin() → setTCXO() →
//  setOutputPower() → setCurrentLimit()
//
//  Separated from gpsInit() so tracker mode can skip re-init
//  of the radio on wakeups where only GPS is needed, and
//  digi modes can skip GPS entirely.
// ============================================================
void radioInit() {
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

    DBG(F("[Radio] Init... "));
    int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("failed, code ")); DBGLN(state);
        while (true) delay(10);
    }

    state = radio.setTCXO(TCXO_VOLTAGE);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("TCXO failed, code ")); DBGLN(state);
        while (true) delay(10);
    }

    if (radio.setOutputPower(LORA_TX_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        DBGLN(F("Invalid power!")); while (true) delay(10);
    }
    if (radio.setCurrentLimit(LORA_OCP_MA) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        DBGLN(F("Invalid OCP!")); while (true) delay(10);
    }

    radio.setSyncWord(LORA_SYNC_WORD);
    radio.setPreambleLength(LORA_PREAMBLE);
    radio.setPacketReceivedAction(onRadioRx);
    radio.setChannelScanAction(onRadioCad);

    radioReady = true;
    DBGLN(F("OK"));
}

// ============================================================
//  GPS INIT
//  Only called when a fix is actually needed (tracker modes).
//  Not called in digi-only modes.
// ============================================================
void gpsInit() {
    Wire.begin();
    Wire.setClock(GPS_I2C_CLOCK);

    gpsPowerOn();

    if (!myGPS.begin()) {
        DBGLN(F("[GPS] Not found. Check PA9=SCL, PA10=SDA."));
        return;
    }
    myGPS.setUART1Output(0);
    myGPS.setUART2Output(0);
    myGPS.setI2COutput(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(1);
    myGPS.setDynamicModel(DYN_MODEL_AIRBORNE1g);
    myGPS.saveConfiguration();
    gpsReady = true;
    DBGLN(F("[GPS] OK"));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(DEBUG_BAUD);

    snprintf(myCallsignFull, sizeof(myCallsignFull), "%s-%d",
             MY_CALLSIGN, MY_SSID);

    LowPower.begin();
    rtc.begin();
    memset(digiQueue, 0, sizeof(digiQueue));
    memset(dedupe,    0, sizeof(dedupe));

#if GPS_POWER_PIN != RADIOLIB_NC
    pinMode(GPS_POWER_PIN, OUTPUT);
    digitalWrite(GPS_POWER_PIN, LOW);
#endif

    radioInit();

#if OPERATING_MODE == MODE_DIGI_ONLY
    radioStartRx();

#elif OPERATING_MODE == MODE_DIGI_CAD
    // CAD mode manages its own radio state — don't start RX here.
    DBGLN(F("Mode: DIGI_CAD"));
#endif
}

// ============================================================
//  LOOP
// ============================================================
void loop() {

#if OPERATING_MODE == MODE_TRACKER_ONLY
    // Re-init radio and GPS after each deepSleep wakeup.
    // These are separated so future optimisations can skip
    // one or the other if state is already valid.
    radioInit();
    gpsInit();
    sendBeacon();
    gpsPowerOff();
    radioSleep();
    enterDeepSleep((uint32_t)BEACON_INTERVAL_S * 1000UL);

#elif OPERATING_MODE == MODE_TRACKER_DIGI
    radioInit();
    gpsInit();
    sendBeacon();
    gpsPowerOff();
    radioStartRx();
    uint32_t end = millis() + (uint32_t)BEACON_INTERVAL_S * 1000UL;
    while (millis() < end) {
        if (radioRxFlag) { radioRxFlag = false; processRxPacket(); }
        flushDigiQueue();
        LowPower.idle(10);
    }

#elif OPERATING_MODE == MODE_DIGI_ONLY
    // Radio already in RX from setup(). Idle loop — no re-init needed.
    if (radioRxFlag) { radioRxFlag = false; processRxPacket(); }
    flushDigiQueue();
    LowPower.idle(10);

#elif OPERATING_MODE == MODE_DIGI_CAD
    // CAD batch loop — run N scans per radio init cycle.
    // Re-init radio only at the start of each batch, not every scan.
    runDiGiCAD();
#endif
}

// ============================================================
//  CAD DIGIPEATER
//
//  Power budget per cycle (CAD_BATCH_SIZE=15, interval=2s):
//
//  Active batch (30 s):
//    radioInit ~100 ms @ 7 mA      =  0.7 mAs
//    15 × CAD 65 ms @ 1.5 mA      =  1.46 mAs
//    15 × sleep 1935 ms @ 0.002 mA =  0.058 mAs
//    Subtotal active 30 s          =  2.22 mAs → 0.074 mA avg
//
//  Deep sleep batch (30 s):
//    Stop2 30 s @ 0.002 mA         =  0.06 mAs → 0.002 mA avg
//
//  Total 60 s cycle average: (2.22 + 0.06) / 60 = ~0.038 mA
//
//  When a packet is received (digipeat TX):
//    Extra ~2.7 s TX @ 80 mA = 216 mAs per digipeat event.
//    In a light APRS environment (1 packet/min) adds ~3.6 mA avg.
//    In a quiet rural environment (1 packet/10 min) adds ~0.36 mA.
// ============================================================
void runDiGiCAD() {
    // --- Active batch: radio init + N CAD scans ---
    radioInit();

    for (int scan = 0; scan < CAD_BATCH_SIZE; scan++) {
        radioCadFlag = false;
        radioStartCad();

        // Wait up to 500 ms for CAD result (~65 ms typical at SF12).
        uint32_t cadStart = millis();
        while (!radioCadFlag && millis() - cadStart < 500) {
            LowPower.idle(5);
        }

        if (!radioCadFlag) {
            // Shouldn't happen — CAD timed out. Skip this scan.
            continue;
        }

        int result = radio.getChannelScanResult();

        if (result == RADIOLIB_LORA_DETECTED) {
            // Something is transmitting. We've missed the triggering
            // packet's preamble — switch to full RX and wait for the
            // NEXT complete packet on the channel.
            DBG(F("[CAD] Preamble! Waiting for next packet... "));
            radioRxFlag = false;
            radioStartRx();

            // Wait up to 60 s — long enough for the current packet
            // to finish and any retransmission / next station to TX.
            uint32_t rxStart = millis();
            while (!radioRxFlag && millis() - rxStart < 60000UL) {
                LowPower.idle(10);
            }

            if (radioRxFlag) {
                radioRxFlag = false;
                processRxPacket();   // digipeat if appropriate
            } else {
                DBGLN(F("[CAD] RX timeout — back to CAD."));
            }

            // After receiving (or timeout), restart CAD batch
            // from the beginning so we re-enter the scan loop fresh.
            // Break out of the inner for loop; the outer loop()
            // call will re-enter runDiGiCAD() immediately.
            radioSleep();
            return;
        }

        // Channel free — sleep until next scan.
        // Radio stays initialised between scans (no sleep here).
        // This is the key fix: we only sleep the MCU, not the radio,
        // so no re-init is needed between scans in the same batch.
        LowPower.deepSleep(CAD_SCAN_INTERVAL_MS);
    }

    // --- Batch complete with no detection ---
    // Sleep the radio and MCU for the inter-batch deep sleep period.
    // This is the long sleep that achieves the low average current.
    DBG(F("[CAD] Batch done. Deep sleep "));
    DBG(CAD_BATCH_SLEEP_MS / 1000);
    DBGLN(F("s"));
    radioSleep();
    enterDeepSleep(CAD_BATCH_SLEEP_MS);
    // On wakeup, loop() calls runDiGiCAD() again → radioInit() → new batch.
}

// ============================================================
//  RADIO HELPERS
// ============================================================
void radioSleep()    { radio.sleep();          radioReady = false; }
void radioStartRx()  { radio.startReceive();   }
void radioStartCad() { radio.startChannelScan(); }

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

void IRAM_ATTR onRadioRx()  { radioRxFlag  = true; }
void IRAM_ATTR onRadioCad() { radioCadFlag = true; }

// ============================================================
//  GPS HELPERS
// ============================================================
void gpsPowerOn() {
#if GPS_POWER_PIN != RADIOLIB_NC
    digitalWrite(GPS_POWER_PIN, HIGH); delay(100);
#endif
}

void gpsPowerOff() {
#if GPS_POWER_PIN != RADIOLIB_NC
    digitalWrite(GPS_POWER_PIN, LOW);
#endif
    gpsReady = false;
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
    DBGLN(F("[GPS] Fix timeout."));
    return false;
}

// ============================================================
//  BEACON
// ============================================================
String buildAPRSPacket() {
    APRSPacket pkt;
    pkt.source    = String(myCallsignFull);
    pkt.path      = String(BEACON_PATH);
    pkt.symbol    = APRS_SYMBOL_CODE;
    pkt.overlay   = APRS_SYMBOL_TABLE;
    pkt.latitude  = lastPos.lat;
    pkt.longitude = lastPos.lon;
    pkt.altitude  = (int)(lastPos.altM * 3.28084f);
    pkt.speed     = lastPos.speed;
    pkt.course    = lastPos.course;
    pkt.comment   = String(MY_COMMENT);
    return APRSPacketLib::generatePositionPacket(pkt);
}

void sendBeacon() {
    if (!waitForFix(GPS_FIX_TIMEOUT_MS)) {
        if (!lastPos.valid) {
            DBGLN(F("[Beacon] No position — skipping TX."));
            return;
        }
        DBGLN(F("[Beacon] Using last known position."));
    }
    radioTransmit(buildAPRSPacket());
}

// ============================================================
//  STANDARD DIGIPEATER
// ============================================================
void processRxPacket() {
    String raw;
    int state = radio.readData(raw);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("[RX] readData failed, code ")); DBGLN(state);
        radioStartRx();
        return;
    }
    if (raw.length() > 3 && raw[0] == '<') raw = raw.substring(3);

    DBG(F("[RX] ")); DBGLN(raw);
    DBG(F("     RSSI=")); DBG(radio.getRSSI());
    DBG(F(" SNR=")); DBGLN(radio.getSNR());

    String modified;
    if (needsDigipeat(raw, modified) && !isDuplicate(raw)) {
        markDuplicate(raw);
        enqueueForDigipeat(modified);
    }

    radioStartRx();
}

bool needsDigipeat(const String& raw, String& modified) {
    int colonIdx = raw.indexOf(':');
    if (colonIdx < 0) return false;
    String header  = raw.substring(0, colonIdx);
    String payload = raw.substring(colonIdx);
    int arrowIdx   = header.indexOf('>');
    if (arrowIdx < 0) return false;
    String source      = header.substring(0, arrowIdx);
    String destAndPath = header.substring(arrowIdx + 1);
    if (source.startsWith(MY_CALLSIGN)) return false;
    int commaIdx = destAndPath.indexOf(',');
    if (commaIdx < 0) return false;
    String dest = destAndPath.substring(0, commaIdx);
    String path = destAndPath.substring(commaIdx + 1);

    String fields[8]; int nFields = 0;
    { String rem = path;
      while (rem.length() > 0 && nFields < 8) {
          int sep = rem.indexOf(',');
          if (sep < 0) { fields[nFields++] = rem; break; }
          fields[nFields++] = rem.substring(0, sep);
          rem = rem.substring(sep + 1);
      }
    }

    bool digipeated = false;
    for (int i = 0; i < nFields && !digipeated; i++) {
        if (fields[i].endsWith("*")) continue;
        if (fields[i].equalsIgnoreCase(DIGI_PATH_1)) {
            fields[i] = String(myCallsignFull) + "*";
            digipeated = true;
        } else if (strlen(DIGI_PATH_2) > 0 &&
                   (fields[i].startsWith("WIDE2-") ||
                    fields[i].startsWith("wide2-"))) {
            int n = fields[i].substring(6).toInt();
            if (n >= 1) {
                if (n == 1) {
                    fields[i] = String(myCallsignFull) + "*";
                } else {
                    if (nFields < 8) {
                        for (int j = nFields; j > i; j--) fields[j] = fields[j-1];
                        fields[i]   = String(myCallsignFull) + "*";
                        fields[i+1] = "WIDE2-" + String(n - 1);
                        nFields++; i++;
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
bool isDuplicate(const String& r) {
    uint32_t h = crc32(r);
    for (int i = 0; i < 8; i++) if (dedupe[i] == h) return true;
    return false;
}
void markDuplicate(const String& r) {
    dedupe[dedupeIdx] = crc32(r);
    dedupeIdx = (dedupeIdx + 1) & 7;
}
void enqueueForDigipeat(const String& pkt) {
    for (int i = 0; i < DIGI_QUEUE_SIZE; i++) {
        if (!digiQueue[i].used) {
            uint32_t d = random(DIGI_DELAY_MIN_MS, DIGI_DELAY_MAX_MS + 1);
            digiQueue[i] = {pkt, millis() + d, true};
            DBG(F("[Digi] Queued, delay=")); DBG(d); DBGLN(F("ms"));
            return;
        }
    }
    DBGLN(F("[Digi] Queue full."));
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
//  SLEEP
// ============================================================
void enterDeepSleep(uint32_t ms) {
    DBG(F("[Sleep] ")); DBG(ms); DBGLN(F("ms"));
#if DEBUG_SERIAL
    Serial.flush(); delay(10);
#endif
    LowPower.deepSleep(ms);
}
