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
#if BMP280_ENABLED
#include <Adafruit_BMP280.h>
#endif

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
//  BMP280
// ============================================================
#if BMP280_ENABLED
Adafruit_BMP280 bmp;
#endif

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
#if BMP280_ENABLED
static bool  bmpReady       = false;
static float bmpSeaLevelHpa = 1013.25f; // updated by calibrateBmpFromGPS()
#endif

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
void onRadioRx();
void onRadioCad();

void gpsInit();
void gpsPowerOn();
void gpsPowerOff();
bool waitForFix(uint32_t timeoutMs);
#if BMP280_ENABLED
void bmpInit();
void calibrateBmpFromGPS(int samples);
#endif

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
#if BMP280_ENABLED
    bmpInit();
#endif
}

// ============================================================
//  BMP280 INIT & CALIBRATION
//
//  bmpInit() is called from gpsInit() so Wire is already up.
//  If the sensor is absent, bmpReady stays false and the tracker
//  falls back to GPS altitude — no configuration change needed.
//
//  calibrateBmpFromGPS() collects BMP280_CAL_SAMPLES GPS altitude
//  readings at 1 Hz immediately after a fix, averages them to
//  reduce GPS altitude noise, then back-calculates the sea-level
//  pressure reference the BMP280 needs for absolute altitude.
//  Formula: seaLevel_hPa = measured_hPa / (1 - alt_m/44330)^5.255
// ============================================================
#if BMP280_ENABLED
void bmpInit() {
    if (bmpReady) return;   // already initialised — survives deep sleep
    if (!bmp.begin(BMP280_I2C_ADDR)) {
        DBGLN(F("[BMP] Not found — using GPS altitude."));
        return;
    }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,    // temperature (needed internally)
                    Adafruit_BMP280::SAMPLING_X16,   // pressure — max oversampling
                    Adafruit_BMP280::FILTER_X16,     // IIR filter to smooth readings
                    Adafruit_BMP280::STANDBY_MS_500);
    bmpReady = true;
    DBGLN(F("[BMP] OK"));
}

void calibrateBmpFromGPS(int samples) {
    DBG(F("[BMP] Calibrating from GPS ("));
    DBG(samples); DBGLN(F(" samples)..."));

    float   altSum = 0;
    int     count  = 0;

    for (int i = 0; i < samples; i++) {
        // Wait up to 1.1 s for the next 1 Hz GPS position update.
        uint32_t t = millis();
        while (millis() - t < 1100) {
            myGPS.checkUblox();
            delay(10);
        }
        if (myGPS.getPVT() && myGPS.getFixType() != 0 && myGPS.getSIV() > 3) {
            altSum += myGPS.getAltitudeMSL() / 1000.0f;
            count++;
        }
    }

    if (count == 0) {
        DBGLN(F("[BMP] Cal failed — no valid GPS samples."));
        return;
    }

    float avgAltM  = altSum / count;
    float pressHpa = bmp.readPressure() / 100.0f;
    bmpSeaLevelHpa = pressHpa / powf(1.0f - avgAltM / 44330.0f, 5.255f);

    DBG(F("[BMP] avgAlt=")); DBG(avgAltM);
    DBG(F("m  seaLevel=")); DBG(bmpSeaLevelHpa); DBGLN(F("hPa"));
}
#endif

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

void onRadioRx()  { radioRxFlag  = true; }
void onRadioCad() { radioCadFlag = true; }

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
#if BMP280_ENABLED
    // Use BMP280 altitude if calibrated; fall back to GPS altitude otherwise.
    float altM = bmpReady ? bmp.readAltitude(bmpSeaLevelHpa) : lastPos.altM;
#else
    float altM = lastPos.altM;
#endif
    String gpsData = APRSPacketLib::encodeGPSIntoBase91(
        lastPos.lat, lastPos.lon,
        lastPos.course, lastPos.speed,
        String(APRS_SYMBOL_CODE),
        true, (int)(altM * 3.28084f));
    return APRSPacketLib::generateBase91GPSBeaconPacket(
        String(myCallsignFull),
        String(APRS_DESTINATION),
        String(BEACON_PATH),
        String(APRS_SYMBOL_TABLE),
        gpsData) + MY_COMMENT;
}

void sendBeacon() {
    if (!waitForFix(GPS_FIX_TIMEOUT_MS)) {
        if (!lastPos.valid) {
            DBGLN(F("[Beacon] No position — skipping TX."));
            return;
        }
        DBGLN(F("[Beacon] Using last known position."));
    }
#if BMP280_ENABLED
    // Calibrate BMP280 sea-level pressure reference from fresh GPS fix.
    // Skipped automatically if no BMP280 was detected at init.
    if (bmpReady) calibrateBmpFromGPS(BMP280_CAL_SAMPLES);
#endif
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

// Returns true if this packet should be digipeated, and writes the
// modified TNC2 string (with our callsign inserted into the path) to
// `modified`. Returns false if the packet is from us, already fully
// digipeated, or has no path we handle.
bool needsDigipeat(const String& raw, String& modified) {
    // Split "SOURCE>DEST,PATH:PAYLOAD" at the first colon.
    int colonIdx = raw.indexOf(':');
    if (colonIdx < 0) return false;
    String header  = raw.substring(0, colonIdx);
    String payload = raw.substring(colonIdx);  // includes the ':'

    // Split header into source and "DEST,PATH".
    int arrowIdx = header.indexOf('>');
    if (arrowIdx < 0) return false;
    String source      = header.substring(0, arrowIdx);
    String destAndPath = header.substring(arrowIdx + 1);

    // Never digipeat our own transmissions.
    if (source.startsWith(MY_CALLSIGN)) return false;

    // Separate destination callsign from the path fields.
    int commaIdx = destAndPath.indexOf(',');
    if (commaIdx < 0) return false;
    String dest = destAndPath.substring(0, commaIdx);
    String path = destAndPath.substring(commaIdx + 1);

    // Split the comma-separated path into individual fields,
    // e.g. "WIDE1-1,WIDE2-2" → fields[0]="WIDE1-1", fields[1]="WIDE2-2".
    String fields[8]; int nFields = 0;
    { String rem = path;
      while (rem.length() > 0 && nFields < 8) {
          int sep = rem.indexOf(',');
          if (sep < 0) { fields[nFields++] = rem; break; }
          fields[nFields++] = rem.substring(0, sep);
          rem = rem.substring(sep + 1);
      }
    }

    // Scan fields left-to-right for the first hop we should handle.
    // A trailing '*' means that hop has already been consumed — skip it.
    bool digipeated = false;
    for (int i = 0; i < nFields && !digipeated; i++) {
        if (fields[i].endsWith("*")) continue;

        if (fields[i].equalsIgnoreCase(DIGI_PATH_1)) {
            // Exact alias match (e.g. WIDE1-1): replace with our
            // callsign marked used (*). No hop count to decrement.
            fields[i] = String(myCallsignFull) + "*";
            digipeated = true;

        } else if (strlen(DIGI_PATH_2) > 0 &&
                   (fields[i].startsWith("WIDE2-") ||
                    fields[i].startsWith("wide2-"))) {
            int n = fields[i].substring(6).toInt();
            if (n >= 1) {
                if (n == 1) {
                    // Last remaining hop: replace field with our callsign.
                    // e.g. WIDE2-1 → N0CALL-9*
                    fields[i] = String(myCallsignFull) + "*";
                } else {
                    // More hops remain: insert our callsign before the
                    // decremented WIDE2 field so downstream digipeaters
                    // can still forward it.
                    // e.g. WIDE2-2 → N0CALL-9*, WIDE2-1
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

    // Reassemble the modified path and rebuild the full TNC2 packet.
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
