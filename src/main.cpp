// ============================================================
// STM32WLE5 LoRa APRS Tracker + Digipeater
// Target: Ebyte E77-400MBL-01 / E77-900MBL-01 (STM32WLE5CC)
//
// Base code from radiohound LoRa-Tracker-Digipeater:
//   https://github.com/radiohound/LoRa-Tracker-Digipeater
// Radio init sequence verified against PicoTrack (K6ATV):
//   https://github.com/radiohound/PicoTrack
//
// Required libraries (Arduino Library Manager):
//   RadioLib, APRSPacketLib, SparkFun u-blox GNSS,
//   STM32LowPower, STM32RTC, Adafruit BMP280 (if BMP280_ENABLED)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <RadioLib.h>
#include <APRSPacketLib.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>
#include "config.h"
#include "settings/tk_glue.h"
#if HORUS_ENABLED
#include "horus/horus.h"
#endif

#if BMP280_ENABLED
#include <Adafruit_BMP280.h>
#endif

// ============================================================
// DEBUG
// ============================================================
#if DEBUG_SERIAL
  #define DBG(...)   Serial.print(__VA_ARGS__)
  #define DBGLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG(...)
  #define DBGLN(...)
#endif

// ============================================================
// RADIO
// ============================================================
STM32WLx    radio = new STM32WLx_Module();

static const uint32_t              rfswitch_pins[]  = RFSWITCH_PINS;
static const Module::RfSwitchMode_t rfswitch_table[] = { RFSWITCH_TABLE };

// ============================================================
// GPS
// ============================================================
SFE_UBLOX_GNSS myGPS;

// ============================================================
// BMP280
// ============================================================
#if BMP280_ENABLED
Adafruit_BMP280 bmp;
#endif

// ============================================================
// RTC
// ============================================================
STM32RTC& rtc = STM32RTC::getInstance();

// ============================================================
// STATE
// ============================================================
static char myCallsignFull[12];

struct Position {
    float   lat    = 0.0f;
    float   lon    = 0.0f;
    float   altM   = 0.0f;
    int     speed  = 0;
    int     course = 0;
    uint8_t sats   = 0;
    uint8_t hour   = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    bool    valid  = false;
    uint32_t fixMs = 0;     // millis() when this fix was taken
};

static Position lastPos;
static bool     radioReady = false;
static bool     gpsReady   = false;

#if BMP280_ENABLED
static bool  bmpReady       = false;
static float bmpSeaLevelHpa = 1013.25f;
#endif

#if CUTDOWN_ENABLED
enum CutdownState { CUTDOWN_IDLE, CUTDOWN_ARMED, CUTDOWN_TRIGGERED };
static CutdownState cutdownState    = CUTDOWN_IDLE;
static float        cutdownLaunchAlt = 0.0f;
static int          cutdownConfirm  = 0;
#endif

// ============================================================
// INTERRUPT FLAGS
// ============================================================
volatile bool radioRxFlag  = false;
volatile bool radioCadFlag = false;

// ============================================================
// DUPLICATE SUPPRESSION
// ============================================================
static uint32_t      dedupe[8];
static uint8_t       dedupeIdx = 0;
struct DigiQueueEntry { String packet; uint32_t txAfterMs; bool used; };
static DigiQueueEntry digiQueue[DIGI_QUEUE_SIZE];

// ============================================================
// STATION TRACKING (MODE_DIGI_CAD_BATTERY)
// ============================================================
struct StationRecord {
    char  callsign[12];
    float lat;
    float lon;
    bool  valid;
};
static StationRecord stationTable[STATION_TRACK_COUNT];

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void    radioInit();
void    radioSleep();
void    radioStartRx();
void    radioStartCad();
bool    radioTransmit(const String& tnc2);
void    onRadioRx();
void    onRadioCad();
void    gpsInit();
void    gpsPowerOn();
void    gpsPowerOff();
bool    waitForFix(uint32_t timeoutMs);
void    sendBeacon();
String  buildAPRSPacket();
void    processRxPacket();
bool    processRxPacketBattery();
void    runDiGiCAD();
void    runDiGiCAD_Battery();
bool    needsDigipeat(const String& raw, String& modified);
bool    isDuplicate(const String& raw);
void    markDuplicate(const String& raw);
void    enqueueForDigipeat(const String& pkt);
void    flushDigiQueue();
uint32_t crc32(const String& s);
void    enterDeepSleep(uint32_t ms);
float   distanceMeters(float lat1, float lon1, float lat2, float lon2);
int     findStation(const String& call);
void    updateStation(const String& call, float lat, float lon);
int     classifyAndUpdate(const String& call, float lat, float lon);

#if BMP280_ENABLED
void    bmpInit();
void    calibrateBmpFromGPS(int samples);
#endif

#if CUTDOWN_ENABLED
void    checkCutdown(float altM);
#endif

// ============================================================
// RADIO INIT — LoRa mode (APRS)
//
// Sequence verified against PicoTrack:
//   setRfSwitchTable → begin() → setTCXO() →
//   setOutputPower() → setCurrentLimit()
// ============================================================
void radioInit() {
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

    DBG(F("[Radio] Init LoRa... "));
    int state = radio.begin(tkCfg.freq_hz / 1e6f, tkCfg.bw_hz / 1000.0f, tkCfg.sf, tkCfg.cr);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("failed, code ")); DBGLN(state);
        while (true) delay(10);
    }

    state = radio.setTCXO(TCXO_VOLTAGE);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("TCXO failed, code ")); DBGLN(state);
        while (true) delay(10);
    }

    if (radio.setOutputPower(tkCfg.power_dbm) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        DBGLN(F("[Radio] Invalid power!")); while (true) delay(10);
    }
    if (radio.setCurrentLimit(LORA_OCP_MA) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        DBGLN(F("[Radio] Invalid OCP!")); while (true) delay(10);
    }

    radio.setSyncWord(tkCfg.sync);
    radio.setPreambleLength(tkCfg.preamble);
    radio.setPacketReceivedAction(onRadioRx);
    radio.setChannelScanAction(onRadioCad);

    radioReady = true;
    DBGLN(F("OK"));
}

void tkApplyRadio()          { radioInit(); }
void tkOnSettingsChanged()   { tks_fullcall(&tkCfg, myCallsignFull, sizeof myCallsignFull); }

void radioSleep()    { radio.sleep(); radioReady = false; }
void radioStartRx()  { radio.setPacketReceivedAction(onRadioRx); radio.startReceive(); }
void radioStartCad() { radio.startChannelScan(); }

bool radioTransmit(const String& tnc2) {
    if (!tks_call_is_set(&tkCfg)) { DBGLN(F("[TX] callsign is N0CALL: nothing is transmitted")); return false; }
    String pkt = "<\xff\x01" + tnc2;
    DBG(F("[TX APRS] ")); DBGLN(pkt);
    int state = radio.transmit(pkt);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("[TX APRS] failed, code ")); DBGLN(state);
        return false;
    }
    return true;
}

void onRadioRx()  { radioRxFlag  = true; }
void onRadioCad() { radioCadFlag = true; }

// ============================================================
// GPS INIT
// Only called when a fix is needed (tracker modes).
// Matches radiohound gpsInit() exactly.
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
    myGPS.setAutoPVT(true);                        // a position every second without polling
    myGPS.setDynamicModel(DYN_MODEL_AIRBORNE1g);   // essential for HAB > 12 km
    myGPS.saveConfiguration();

    gpsReady = true;
    DBGLN(F("[GPS] OK"));

#if BMP280_ENABLED
    bmpInit();
#endif
}

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

static float lastVelUpMs = 0.0f, lastGroundKmh = 0.0f;

// Reads the newest GPS solution into lastPos when it has a usable fix.
static bool gpsReadPvt() {
    // The GPS sends a position by itself every second (setAutoPVT in gpsInit), so this never waits.
    // Read each value once: asking for the same one twice makes the library poll the GPS again.
    myGPS.checkUblox();
    if (myGPS.getPVT() && myGPS.getFixType() != 0 && myGPS.getSIV() > 3) {
        int32_t gs = myGPS.getGroundSpeed();                        // mm/s
        lastPos.lat    = myGPS.getLatitude()    / 1e7f;
        lastPos.lon    = myGPS.getLongitude()   / 1e7f;
        lastPos.altM   = myGPS.getAltitudeMSL() / 1000.0f;
        lastPos.speed  = (int)(gs / 514.44f);                       // mm/s → knots
        lastPos.course = myGPS.getHeading() / 100000;
        lastPos.sats   = myGPS.getSIV();
        lastPos.hour   = myGPS.getHour();
        lastPos.minute = myGPS.getMinute();
        lastPos.second = myGPS.getSecond();
        lastPos.fixMs  = millis();
        lastPos.valid  = true;
        lastVelUpMs    = -(float)myGPS.getNedDownVel() / 1000.0f;   // mm/s, down is positive
        lastGroundKmh  = gs / 277.78f;                              // mm/s -> km/h
        return true;
    }
    return false;
}

bool waitForFix(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        if (gpsReadPvt()) return true;
        tkSerialPoll();
        delay(100);
    }
    DBGLN(F("[GPS] Fix timeout."));
    return false;
}

// ============================================================
// BMP280 — matches radiohound bmpInit() / calibrateBmpFromGPS()
// ============================================================
#if BMP280_ENABLED
void bmpInit() {
    if (bmpReady) return;

    if (!bmp.begin(BMP280_I2C_ADDR)) {
        DBGLN(F("[BMP] Not found — using GPS altitude."));
        return;
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    bmpReady = true;
    DBGLN(F("[BMP] OK"));
}

void calibrateBmpFromGPS(int samples) {
    DBG(F("[BMP] Calibrating (")); DBG(samples); DBGLN(F(" samples)..."));
    float altSum = 0; int count = 0;
    for (int i = 0; i < samples; i++) {
        uint32_t t = millis();
        while (millis() - t < 1100) { myGPS.checkUblox(); delay(10); }
        if (myGPS.getPVT() && myGPS.getFixType() != 0 && myGPS.getSIV() > 3) {
            altSum += myGPS.getAltitudeMSL() / 1000.0f;
            count++;
        }
    }
    if (count == 0) { DBGLN(F("[BMP] Cal failed.")); return; }
    float avgAltM  = altSum / count;
    float pressHpa = bmp.readPressure() / 100.0f;
    bmpSeaLevelHpa = pressHpa / powf(1.0f - avgAltM / 44330.0f, 5.255f);
    DBG(F("[BMP] avgAlt=")); DBG(avgAltM);
    DBG(F("m  seaLevel=")); DBG(bmpSeaLevelHpa); DBGLN(F(" hPa"));
}
#endif

// ============================================================
// CUTDOWN — unchanged from radiohound
// ============================================================
#if CUTDOWN_ENABLED
void checkCutdown(float altM) {
    if (cutdownState == CUTDOWN_TRIGGERED) return;
    if (cutdownState == CUTDOWN_IDLE) {
        if (cutdownLaunchAlt == 0.0f) { cutdownLaunchAlt = altM; return; }
        if (altM >= cutdownLaunchAlt + CUTDOWN_ARM_ASCENT_M) {
            cutdownState = CUTDOWN_ARMED;
            DBGLN(F("[Cutdown] ARMED"));
        }
        return;
    }
    if (altM >= CUTDOWN_ALTITUDE_M) {
        if (++cutdownConfirm >= CUTDOWN_CONFIRM_COUNT) {
            DBGLN(F("[Cutdown] FIRING"));
            digitalWrite(CUTDOWN_PIN, HIGH);
            delay(CUTDOWN_PULSE_MS);
            digitalWrite(CUTDOWN_PIN, LOW);
            cutdownState = CUTDOWN_TRIGGERED;
            DBGLN(F("[Cutdown] Pulse complete"));
        }
    } else {
        cutdownConfirm = 0;
    }
}
#endif

// ============================================================
// APRS PACKET
// ============================================================
String buildAPRSPacket() {
#if BMP280_ENABLED
    float altM = bmpReady ? bmp.readAltitude(bmpSeaLevelHpa) : lastPos.altM;
#else
    float altM = lastPos.altM;
#endif

    String gpsData = APRSPacketLib::encodeGPSIntoBase91(
        lastPos.lat, lastPos.lon,
        lastPos.course, lastPos.speed,
        String(tkCfg.sym_code),
        true, (int)(altM * 3.28084f));

    // comment2 is added after the normal comment on every comment2_every-th beacon (0 = never)
    static uint32_t beaconCount = 0;
    beaconCount++;
    bool extra = tkCfg.comment2_every > 0 && tkCfg.comment2[0] && beaconCount % tkCfg.comment2_every == 0;
    String comment = tkCfg.comment[0] ? String(" ") + tkCfg.comment : String();
    if (extra) comment += String(" ") + tkCfg.comment2;

    return APRSPacketLib::generateBase91GPSBeaconPacket(
        String(myCallsignFull),
        String(tkCfg.tocall),
        String(tkCfg.path),
        String(tkCfg.sym_table),
        gpsData) + comment;
}

// ============================================================
// HORUS BINARY v2 — after the APRS beacon (optional)
// Uses the fix sendBeacon() just took. HORUS_FAKE_FIX is for bench tests without a GPS.
// ============================================================
#if HORUS_ENABLED
static HorusFix horusFixFromLastPos() {
    HorusFix f = {};
#ifdef HORUS_FAKE_FIX
    f.valid = true;  f.lat = 37.0058f; f.lon = -121.5683f; f.altM = 100; f.sats = 8;
    f.hour = 12; f.minute = 34; f.second = 56;               // fixed test time, not real UTC
    f.fixMs = millis();
#else
    f.valid = lastPos.valid;
    f.lat = lastPos.lat; f.lon = lastPos.lon;
#if BMP280_ENABLED
    f.altM = bmpReady ? bmp.readAltitude(bmpSeaLevelHpa) : lastPos.altM;
#else
    f.altM = lastPos.altM;
#endif
    f.sats = lastPos.sats;
    f.hour = lastPos.hour; f.minute = lastPos.minute; f.second = lastPos.second;
    f.fixMs = lastPos.fixMs;
#endif
    f.speedKmh = lastPos.speed * 1.852f;                      // knots -> km/h
#if BMP280_ENABLED
    if (bmpReady) { f.hasTemp = true; f.tempC = bmp.readTemperature(); }
#endif
    return f;
}

static bool horusEnabledNow() { return tkCfg.horus && tks_horus_call_is_set(&tkCfg); }   // Horus has its own callsign check

// Tracker-only mode: one Horus packet after each beacon (every Nth one, see horus_div).
static void horusAfterBeacon() {
    HorusFix f = horusFixFromLastPos();
    if (!horusEnabledNow()) return;
    DBGLN(F("[Horus] TX"));
    bool force = tkHorusNow; tkHorusNow = false;
    if (force ? horusSend(f) : horusSendDivided(f)) { DBG(F("[Horus] sent #")); DBGLN(horusSequence()); }
    radioReady = false;                                       // radio is in FSK mode now
}
#endif

// ============================================================
// SEND BEACON
// ============================================================
void sendBeacon() {
    bool haveFix = waitForFix((uint32_t)tkCfg.gps_timeout_s * 1000UL);
    bool havePos = haveFix || lastPos.valid;

    if (!havePos) {
        DBGLN(F("[Beacon] No position — skipping APRS."));
    } else if (!haveFix) {
        DBGLN(F("[Beacon] Using last known position."));
    }

#if BMP280_ENABLED
    if (bmpReady && havePos) calibrateBmpFromGPS(BMP280_CAL_SAMPLES);
#endif

#if CUTDOWN_ENABLED
    if (havePos) {
#if BMP280_ENABLED
        float altM = bmpReady ? bmp.readAltitude(bmpSeaLevelHpa) : lastPos.altM;
#else
        float altM = lastPos.altM;
#endif
        checkCutdown(altM);
    }
#endif

    if (havePos && tkCfg.aprs) {
        radioTransmit(buildAPRSPacket());
    }
}

// ============================================================
// DIGIPEATER
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
    DBG(F("  RSSI=")); DBG(radio.getRSSI());
    DBG(F("  SNR="));  DBGLN(radio.getSNR());

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

    int arrowIdx = header.indexOf('>');
    if (arrowIdx < 0) return false;
    String source      = header.substring(0, arrowIdx);
    String destAndPath = header.substring(arrowIdx + 1);
    if (source.equalsIgnoreCase(myCallsignFull)) return false;

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
// STATION TRACKING HELPERS
// ============================================================
float distanceMeters(float lat1, float lon1, float lat2, float lon2) {
    float dlat = (lat2 - lat1) * 111320.0f;
    float dlon = (lon2 - lon1) * 111320.0f * cosf(lat1 * 0.01745329f);
    return sqrtf(dlat * dlat + dlon * dlon);
}

int findStation(const String& call) {
    for (int i = 0; i < STATION_TRACK_COUNT; i++)
        if (stationTable[i].valid && String(stationTable[i].callsign) == call)
            return i;
    return -1;
}

void updateStation(const String& call, float lat, float lon) {
    int idx = findStation(call);
    if (idx < 0) {
        // Find an empty slot
        for (int i = 0; i < STATION_TRACK_COUNT; i++) {
            if (!stationTable[i].valid) { idx = i; break; }
        }
        // Table full — overwrite oldest entry (slot 0, simple rotation)
        if (idx < 0) {
            idx = 0;
            for (int i = 1; i < STATION_TRACK_COUNT; i++) stationTable[i-1] = stationTable[i];
            idx = STATION_TRACK_COUNT - 1;
        }
    }
    call.toCharArray(stationTable[idx].callsign, sizeof(stationTable[idx].callsign));
    stationTable[idx].lat   = lat;
    stationTable[idx].lon   = lon;
    stationTable[idx].valid = true;
}

// Classify a received station and update its stored position.
// Returns STATION_NEW, STATION_MOVED, or STATION_STATIONARY.
int classifyAndUpdate(const String& call, float lat, float lon) {
    int idx = findStation(call);
    int result;
    if (idx < 0) {
        result = STATION_NEW;
    } else {
        float dist = distanceMeters(stationTable[idx].lat, stationTable[idx].lon, lat, lon);
        result = (dist >= STATION_MOVE_THRESH_M) ? STATION_MOVED : STATION_STATIONARY;
    }
    updateStation(call, lat, lon);
    return result;
}

// ============================================================
// BATTERY-MODE RX PACKET HANDLER
// Parses the received packet, applies movement filter, and
// digipeats only if the station is new or has moved.
// Returns true if the packet warrants keeping the RX window open.
// Does NOT call radioStartRx() — caller is responsible.
// ============================================================
bool processRxPacketBattery() {
    String raw;
    int state = radio.readData(raw);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("[RX-BAT] readData failed, code ")); DBGLN(state);
        return false;
    }
    if (raw.length() > 3 && raw[0] == '<') raw = raw.substring(3);
    DBG(F("[RX-BAT] ")); DBGLN(raw);

    APRSPacket aprs = APRSPacketLib::processReceivedPacket(raw, (int)radio.getRSSI(),
                                                           radio.getSNR(), 0);
    bool hasPosition = !(aprs.latitude == 0.0f && aprs.longitude == 0.0f);

    bool shouldDigipeat = false;
    bool openWindow     = false;

    if (!hasPosition) {
        // Non-position packet (message, status, object) — treat as interesting
        DBG(F("[RX-BAT] No position — digipeating: ")); DBGLN(aprs.sender);
        shouldDigipeat = true;
        openWindow     = true;
    } else {
        int status = classifyAndUpdate(aprs.sender, aprs.latitude, aprs.longitude);
        if (status == STATION_STATIONARY) {
            DBG(F("[RX-BAT] Stationary — skipping: ")); DBGLN(aprs.sender);
        } else {
            const __FlashStringHelper* reason = (status == STATION_NEW) ? F("New") : F("Moved");
            DBG(F("[RX-BAT] ")); DBG(reason);
            DBG(F(" station — digipeating: ")); DBGLN(aprs.sender);
            shouldDigipeat = true;
            openWindow     = true;
        }
    }

    if (shouldDigipeat) {
        String modified;
        if (needsDigipeat(raw, modified) && !isDuplicate(raw)) {
            markDuplicate(raw);
            enqueueForDigipeat(modified);
            uint32_t flushDeadline = millis() + DIGI_DELAY_MAX_MS + 100UL;
            while (millis() < flushDeadline) { flushDigiQueue(); delay(10); }
        }
    }
    return openWindow;
}

// ============================================================
// CAD DIGIPEATER — two-stage design
//
// Stage 1: continuous low-power CAD sentinel.
//   Scans every CAD_SCAN_INTERVAL_MS. Guaranteed to detect any
//   transmission >= CAD_SCAN_INTERVAL_MS long. Average current ~100 uA.
//
// Stage 2: active RX window entered on CAD trigger.
//   Listens continuously for up to CAD_RX_WINDOW_SOLAR_MS (6 min).
//   Timer resets on each received packet so an active station
//   keeps the window open. Returns to Stage 1 after silence.
// ============================================================
void runDiGiCAD() {
    if (!radioReady) radioInit();

    // --- Stage 1: CAD sentinel ---
    radioCadFlag = false;
    radioStartCad();
    uint32_t cadStart = millis();
    while (!radioCadFlag && millis() - cadStart < 500) LowPower.idle(5);

    if (!radioCadFlag) {
        // No activity — sleep until next scan
        LowPower.deepSleep(CAD_SCAN_INTERVAL_MS);
        return;
    }

    int result = radio.getChannelScanResult();
    if (result != RADIOLIB_LORA_DETECTED) {
        LowPower.deepSleep(CAD_SCAN_INTERVAL_MS);
        return;
    }

    // --- Stage 2: active RX window ---
    DBGLN(F("[CAD] Signal detected — entering active RX window"));
    uint32_t windowDeadline = millis() + CAD_RX_WINDOW_SOLAR_MS;

    radioRxFlag = false;
    radioStartRx();

    while (millis() < windowDeadline) {
        if (radioRxFlag) {
            radioRxFlag = false;
            processRxPacket();
            // Flush digi queue with randomised delay
            uint32_t flushDeadline = millis() + DIGI_DELAY_MAX_MS + 100UL;
            while (millis() < flushDeadline) { flushDigiQueue(); delay(10); }
            // Reset window — active station keeps RX open
            windowDeadline = millis() + CAD_RX_WINDOW_SOLAR_MS;
            radioStartRx();
        }
        LowPower.idle(10);
    }

    DBGLN(F("[CAD] RX window expired — returning to CAD sentinel"));
}

// ============================================================
// CAD DIGIPEATER — battery-optimised two-stage design
//
// Same Stage 1 CAD sentinel as solar mode.
// Stage 2 is only entered if the triggering packet is from a
// station that is new or has moved since last heard.
// Stationary ground stations are received but not digipeated
// and do not open the RX window, saving power.
// Within an open RX window, only moving/new packets reset the
// timer — a burst of stationary packets lets the window expire.
// ============================================================
void runDiGiCAD_Battery() {
    if (!radioReady) radioInit();

    // --- Stage 1: CAD sentinel ---
    radioCadFlag = false;
    radioStartCad();
    uint32_t cadStart = millis();
    while (!radioCadFlag && millis() - cadStart < 500) LowPower.idle(5);

    if (!radioCadFlag) {
        LowPower.deepSleep(CAD_SCAN_INTERVAL_MS);
        return;
    }

    int result = radio.getChannelScanResult();
    if (result != RADIOLIB_LORA_DETECTED) {
        LowPower.deepSleep(CAD_SCAN_INTERVAL_MS);
        return;
    }

    // Receive the triggering packet and apply movement filter
    DBGLN(F("[CAD-BAT] Signal detected — receiving packet"));
    radioRxFlag = false;
    radioStartRx();
    uint32_t rxStart = millis();
    while (!radioRxFlag && millis() - rxStart < 5000UL) LowPower.idle(10);

    if (!radioRxFlag) {
        DBGLN(F("[CAD-BAT] No packet — returning to sentinel"));
        return;
    }
    radioRxFlag = false;

    bool openWindow = processRxPacketBattery();
    if (!openWindow) {
        // Stationary station — back to CAD immediately
        return;
    }

    // --- Stage 2: active RX window ---
    DBGLN(F("[CAD-BAT] Moving station — entering active RX window"));
    uint32_t windowDeadline = millis() + CAD_RX_WINDOW_BATTERY_MS;
    radioStartRx();

    while (millis() < windowDeadline) {
        if (radioRxFlag) {
            radioRxFlag = false;
            bool moved = processRxPacketBattery();
            // Only reset the timer if a moving/new station was heard
            if (moved) windowDeadline = millis() + CAD_RX_WINDOW_BATTERY_MS;
            radioStartRx();
        }
        LowPower.idle(10);
    }

    DBGLN(F("[CAD-BAT] RX window expired — returning to CAD sentinel"));
}

// ============================================================
// SLEEP
// ============================================================
void enterDeepSleep(uint32_t ms) {
    DBG(F("[Sleep] ")); DBG(ms); DBGLN(F("ms"));
#if DEBUG_SERIAL
    Serial.flush(); delay(10);
#endif
    LowPower.deepSleep(ms);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    tkKeysAtBoot();                                     // Key 1 held = bootloader, user key = defaults
#if DEBUG_SERIAL
    Serial.begin(DEBUG_BAUD);
    delay(500);
#endif
    DBGLN(F("=== LoRa APRS Tracker ==="));

    tkSettingsBegin();                                  // settings from flash (or the defaults in config.h), callsign

    LowPower.begin();
    rtc.begin();
    memset(digiQueue, 0, sizeof(digiQueue));
    memset(dedupe,    0, sizeof(dedupe));

#if GPS_POWER_PIN != RADIOLIB_NC
    pinMode(GPS_POWER_PIN, OUTPUT);
    digitalWrite(GPS_POWER_PIN, LOW);
#endif

#if CUTDOWN_ENABLED
    pinMode(CUTDOWN_PIN, OUTPUT);
    digitalWrite(CUTDOWN_PIN, LOW);
#endif

    radioInit();

    tkBootWindow(3000);                                 // a key within 3 s opens the setup command line

#if OPERATING_MODE == MODE_DIGI_ONLY
    radioStartRx();
#endif
}

// ============================================================
// TRACKER + DIGI SCHEDULER
// LoRa APRS and Horus each have their own rate for each flight phase (ascent, descent, landed).
// The GPS stays on, is read once a second, and drives the phase (src/flight/flight_phase.h).
// While nothing is due the radio listens, so the board still digipeats.
// ============================================================
#if OPERATING_MODE == MODE_TRACKER_DIGI
#include "flight/flight_phase.h"

static fp_state_t flight;
static uint32_t   nextAprsMs = 0, lastAprsMs = 0, nextHorusMs = 0;
static uint32_t   lastGpsPollMs = 0, lastFixAtMs = 0, lastWaitMsgMs = 0;
static bool       schedInit = false, firstFixDone = false;

static uint32_t aprsIntervalMs() {
    uint16_t s = flight.phase == FP_DESCENT ? tkCfg.aprs_desc_s : flight.phase == FP_LANDED ? tkCfg.aprs_gnd_s : tkCfg.beacon_s;
    return (uint32_t)s * 1000UL;
}
#if HORUS_ENABLED
static uint32_t horusIntervalMs() {
    uint16_t s = flight.phase == FP_DESCENT ? tkCfg.horus_desc_s : flight.phase == FP_LANDED ? tkCfg.horus_gnd_s : tkCfg.horus_asc_s;
    return (uint32_t)s * 1000UL;
}
#endif
static bool due(uint32_t now, uint32_t at) { return (int32_t)(now - at) >= 0; }

// Reads the GPS (at most once a second, unless force) and updates the flight phase.
static void gpsPollAndPhase(bool force) {
    uint32_t now = millis();
    if (!force && now - lastGpsPollMs < 1000) return;
    lastGpsPollMs = now;
    if (!gpsReadPvt()) return;
    uint32_t dt = lastFixAtMs ? now - lastFixAtMs : 1000;
    lastFixAtMs = now;
    fp_phase_t before = flight.phase;
    fp_update(&flight, lastPos.altM, lastVelUpMs, lastGroundKmh, dt);
    if (flight.phase != before) {
        DBG(F("[Flight] ")); DBGLN(fp_name(flight.phase));
        // the new phase may have a shorter interval: pull the next sends forward (never push them back)
        uint32_t a = lastAprsMs + aprsIntervalMs();
        if (due(nextAprsMs, a)) nextAprsMs = a;
#if HORUS_ENABLED
        uint32_t h = horusLastTxStartMs() + horusIntervalMs();
        if (due(nextHorusMs, h)) nextHorusMs = h;
#endif
    }
}

static void backToLoraRx() {
    if (!radioReady) radioInit();
    radioRxFlag = false;
    radioStartRx();
}

static void runTrackerScheduler() {
    if (!schedInit) {
        schedInit = true;
        fp_init(&flight);
        gpsInit();
        lastAprsMs = millis();
        nextAprsMs = nextHorusMs = millis();
        backToLoraRx();
    }
    if (!gpsReady) { static uint32_t retry = 0; if (millis() - retry > 10000) { retry = millis(); gpsInit(); } }
    if (gpsReady) gpsPollAndPhase(false);

    uint32_t now = millis();
#if HORUS_ENABLED
    const uint32_t LEAD_MS = 1500;              // start early so the wait for the time slot is short and the fix is fresh
    bool horusOn  = horusEnabledNow();
    bool horusDue = tkHorusNow || (horusOn && due(now + LEAD_MS, nextHorusMs));
#else
    bool horusDue = false;
#endif
    bool aprsDue = tkBeaconNow || due(now, nextAprsMs);
#if HORUS_ENABLED
    // a Horus slot is coming: let it go first, LoRa APRS follows right after
    if (aprsDue && !tkBeaconNow && horusOn && !horusDue && (int32_t)(nextHorusMs - now) < 6000) aprsDue = false;
#endif

    if (!lastPos.valid && (horusDue || aprsDue)) {
        if (now - lastWaitMsgMs > 10000) { lastWaitMsgMs = now; DBGLN(F("[GPS] waiting for a first fix")); }
        horusDue = aprsDue = false;
    }

#if HORUS_ENABLED
    if (horusDue) {
        gpsPollAndPhase(true);                  // freshest position before the slot
        HorusFix f = horusFixFromLastPos();
        DBGLN(F("[Horus] TX"));
        tkHorusNow = false;
        bool sent = horusSend(f);
        if (sent) { DBG(F("[Horus] sent #")); DBGLN(horusSequence()); }
        radioReady = false;                     // the radio is in FSK mode now
        nextHorusMs = (sent ? horusLastTxStartMs() : millis()) + horusIntervalMs();
        backToLoraRx();
        return;
    }
#endif
    if (aprsDue) {
        uint32_t startMs = millis();            // the interval counts from the start of the beacon, not its end
        gpsPollAndPhase(true);
        tkBeaconNow = false;
        if (!firstFixDone) {
            firstFixDone = true;
#if BMP280_ENABLED
            if (bmpReady) calibrateBmpFromGPS(BMP280_CAL_SAMPLES);
#endif
        }
#if CUTDOWN_ENABLED
        if (lastPos.valid) {
#if BMP280_ENABLED
            float altM = bmpReady ? bmp.readAltitude(bmpSeaLevelHpa) : lastPos.altM;
#else
            float altM = lastPos.altM;
#endif
            checkCutdown(altM);
        }
#endif
        if (tkCfg.aprs && lastPos.valid) { if (!radioReady) radioInit(); radio.standby(); radioTransmit(buildAPRSPacket()); }
        lastAprsMs = startMs;
        nextAprsMs = startMs + aprsIntervalMs();
        backToLoraRx();
        return;
    }

    tkSerialPoll();
    if (radioRxFlag) { radioRxFlag = false; processRxPacket(); }
    flushDigiQueue();
    delay(10);
}
#endif

// ============================================================
// LOOP
// ============================================================
void loop() {

#if OPERATING_MODE == MODE_TRACKER_ONLY
    if (!radioReady) radioInit();
    if (!gpsReady)   gpsInit();
    sendBeacon();
#if HORUS_ENABLED
    horusAfterBeacon();
#endif
    gpsPowerOff();
    radioSleep();
    enterDeepSleep((uint32_t)tkCfg.beacon_s * 1000UL);

#elif OPERATING_MODE == MODE_TRACKER_DIGI
    runTrackerScheduler();

#elif OPERATING_MODE == MODE_DIGI_ONLY
    tkSerialPoll();
    if (radioRxFlag) { radioRxFlag = false; processRxPacket(); }
    flushDigiQueue();
    LowPower.idle(10);

#elif OPERATING_MODE == MODE_DIGI_CAD_SOLAR
    runDiGiCAD();

#elif OPERATING_MODE == MODE_DIGI_CAD_BATTERY
    runDiGiCAD_Battery();
#endif
}
