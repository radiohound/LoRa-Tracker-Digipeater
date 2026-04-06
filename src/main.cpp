// ============================================================
// STM32WLE5 LoRa APRS Tracker + Digipeater + Horus Binary v2
// Target: Ebyte E77-400MBL-01 / E77-900MBL-01 (STM32WLE5CC)
//
// Base code from radiohound LoRa-Tracker-Digipeater:
//   https://github.com/radiohound/LoRa-Tracker-Digipeater
// Radio init sequence verified against PicoTrack (K6ATV):
//   https://github.com/radiohound/PicoTrack
//
// Horus Binary v2 encoding from projecthorus/horusbinary_radiolib:
//   https://github.com/projecthorus/horusbinary_radiolib
//   Copy horus_l2.h and horus_l2.cpp into this src/ folder.
//
// When HORUS_ENABLED is set in config.h, sendBeacon() transmits
// the APRS packet first (LoRa), then re-inits the radio to FSK
// mode and transmits a Horus Binary v2 packet, then re-inits
// back to LoRa ready for whatever the operating mode needs next.
// All other behaviour is unchanged from the radiohound original.
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

#if BMP280_ENABLED
#include <Adafruit_BMP280.h>
#endif

#if HORUS_ENABLED
#include "horus_l2.h"
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
STM32WLx radio = new STM32WLx_Module();

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
};

static Position lastPos;
static bool     radioReady = false;
static bool     gpsReady   = false;

#if BMP280_ENABLED
static bool  bmpReady       = false;
static float bmpSeaLevelHpa = 1013.25f;
#endif

#if HORUS_ENABLED
static uint16_t horusCounter = 0;

// Horus Binary v2 packet — 16 bytes, last 4 user-customisable
// https://github.com/projecthorus/horusdemodlib/wiki/5-Customising-a-Horus-Binary-v2-Packet
struct __attribute__((packed)) HorusBinaryV2 {
    uint16_t PayloadID;
    uint16_t Counter;
    uint8_t  Hours;
    uint8_t  Minutes;
    uint8_t  Seconds;
    float    Latitude;
    float    Longitude;
    uint16_t Altitude;      // metres MSL
    uint8_t  Speed;         // km/h
    uint8_t  Sats;
    uint8_t  BattVoltage;   // optional: 0-255 mapped from supply voltage
    uint8_t  UserData;      // optional: temperature, flags, etc.
    uint16_t Checksum;      // CRC16-CCITT over all preceding bytes
};

static uint8_t horusRawBuf[128];
static uint8_t horusCodedBuf[128];
static uint32_t horusToneDelay_us = 0;
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
void    runDiGiCAD();
bool    needsDigipeat(const String& raw, String& modified);
bool    isDuplicate(const String& raw);
void    markDuplicate(const String& raw);
void    enqueueForDigipeat(const String& pkt);
void    flushDigiQueue();
uint32_t crc32(const String& s);
void    enterDeepSleep(uint32_t ms);

#if BMP280_ENABLED
void    bmpInit();
void    calibrateBmpFromGPS(int samples);
#endif

#if CUTDOWN_ENABLED
void    checkCutdown(float altM);
#endif

#if HORUS_ENABLED
void    radioInitFSK();
void    fsk4_setup();
void    fsk4_tone(int tone);
void    fsk4_idle();
void    fsk4_preamble();
void    fsk4_write(uint8_t* data, int len);
uint16_t crc16_ccitt(uint8_t* data, uint16_t len);
void    buildHorusPacket(uint8_t* buf, int* len);
void    transmitHorusV2();
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
        DBGLN(F("[Radio] Invalid power!")); while (true) delay(10);
    }
    if (radio.setCurrentLimit(LORA_OCP_MA) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        DBGLN(F("[Radio] Invalid OCP!")); while (true) delay(10);
    }

    radio.setSyncWord(LORA_SYNC_WORD);
    radio.setPreambleLength(LORA_PREAMBLE);
    radio.setPacketReceivedAction(onRadioRx);
    radio.setChannelScanAction(onRadioCad);

    radioReady = true;
    DBGLN(F("OK"));
}

void radioSleep()    { radio.sleep(); radioReady = false; }
void radioStartRx()  { radio.startReceive(); }
void radioStartCad() { radio.startChannelScan(); }

bool radioTransmit(const String& tnc2) {
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
// RADIO INIT — FSK mode (Horus Binary)
//
// Called immediately before a Horus Binary transmission.
// Uses the same init sequence as radioInit() but calls
// beginFSK() instead of begin(), and configures the radio
// for direct/continuous FSK rather than packet mode.
// After Horus TX, radioInit() is called to return to LoRa.
// ============================================================
#if HORUS_ENABLED
void radioInitFSK() {
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);

    DBG(F("[Radio] Init FSK... "));
    int state = radio.beginFSK();
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("failed, code ")); DBGLN(state);
        return;                 // non-fatal — APRS still works
    }

    state = radio.setTCXO(TCXO_VOLTAGE);
    if (state != RADIOLIB_ERR_NONE) {
        DBG(F("TCXO failed, code ")); DBGLN(state); return;
    }

    if (radio.setOutputPower(LORA_TX_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        DBGLN(F("[Radio] Invalid power!")); return;
    }
    if (radio.setCurrentLimit(LORA_OCP_MA) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT) {
        DBGLN(F("[Radio] Invalid OCP!")); return;
    }

    radio.setFrequency(HORUS_FREQ);
    radio.setBitRate(HORUS_FSK4_BAUD * 2.0);           // 2 bits per symbol
    radio.setFrequencyDeviation(HORUS_FSK4_SPACING / 2.0);

    DBGLN(F("OK"));
    fsk4_setup();
}
#endif

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

bool waitForFix(uint32_t timeoutMs) {
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        myGPS.checkUblox();
        if (myGPS.getPVT() && myGPS.getFixType() != 0 && myGPS.getSIV() > 3) {
            lastPos.lat    = myGPS.getLatitude()    / 1e7f;
            lastPos.lon    = myGPS.getLongitude()   / 1e7f;
            lastPos.altM   = myGPS.getAltitudeMSL() / 1000.0f;
            lastPos.speed  = (int)(myGPS.getGroundSpeed() / 514.44f);  // mm/s → knots
            lastPos.course = myGPS.getHeading() / 100000;
            lastPos.sats   = myGPS.getSIV();
            lastPos.hour   = myGPS.getHour();
            lastPos.minute = myGPS.getMinute();
            lastPos.second = myGPS.getSecond();
            lastPos.valid  = true;
            return true;
        }
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
// HORUS BINARY 4FSK
//
// The radio is put into continuous/direct mode via transmitDirect().
// Each call sets the PLL to a new frequency, producing a carrier
// at that frequency for exactly one symbol period.
//
// Tone mapping (LSB-first, 2 bits per symbol):
//   0 → base freq + 0 * spacing
//   1 → base freq + 1 * spacing
//   2 → base freq + 2 * spacing
//   3 → base freq + 3 * spacing
// ============================================================
#if HORUS_ENABLED
void fsk4_setup() {
    horusToneDelay_us = (uint32_t)(1000000UL / HORUS_FSK4_BAUD);
}

void fsk4_tone(int tone) {
    uint32_t freq_hz = (uint32_t)(HORUS_FREQ * 1000000.0)
                     + (uint32_t)(tone * HORUS_FSK4_SPACING);
    radio.transmitDirect(freq_hz);
    delayMicroseconds(horusToneDelay_us);
}

void fsk4_idle() {
    radio.transmitDirect((uint32_t)(HORUS_FREQ * 1000000.0));
}

void fsk4_preamble() {
    for (int i = 0; i < 8; i++) {
        fsk4_tone(0); fsk4_tone(1); fsk4_tone(2); fsk4_tone(3);
    }
}

void fsk4_write(uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        fsk4_tone((data[i] >> 0) & 0x03);
        fsk4_tone((data[i] >> 2) & 0x03);
        fsk4_tone((data[i] >> 4) & 0x03);
        fsk4_tone((data[i] >> 6) & 0x03);
    }
}

uint16_t crc16_ccitt(uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void buildHorusPacket(uint8_t* buf, int* len) {
    HorusBinaryV2 pkt;
    memset(&pkt, 0, sizeof(pkt));

    // Use BMP280 altitude if calibrated, otherwise GPS altitude.
    // Matches the same selection logic used in buildAPRSPacket().
#if BMP280_ENABLED
    float altM = bmpReady ? bmp.readAltitude(bmpSeaLevelHpa) : lastPos.altM;
#else
    float altM = lastPos.altM;
#endif

    pkt.PayloadID  = HORUS_PAYLOAD_ID;
    pkt.Counter    = horusCounter++;
    pkt.Hours      = lastPos.hour;
    pkt.Minutes    = lastPos.minute;
    pkt.Seconds    = lastPos.second;
    pkt.Latitude   = lastPos.lat;
    pkt.Longitude  = lastPos.lon;
    pkt.Altitude   = (uint16_t)constrain((int)altM, 0, 65535);
    pkt.Speed      = (uint8_t)constrain(lastPos.speed, 0, 255);
    pkt.Sats       = lastPos.sats;
    pkt.BattVoltage = 0;    // optional: map ADC reading to 0-255
    pkt.UserData    = 0;    // optional: temperature, status flags, etc.
    pkt.Checksum   = crc16_ccitt((uint8_t*)&pkt, sizeof(pkt) - 2);

    memcpy(buf, &pkt, sizeof(pkt));
    *len = sizeof(pkt);
}

void transmitHorusV2() {
    int raw_len = 0;
    buildHorusPacket(horusRawBuf, &raw_len);

    // Apply Golay(23,12) FEC, interleaving and scrambling
    int coded_len = horus_l2_encode_tx_packet(horusCodedBuf, horusRawBuf, raw_len);

    DBG(F("[TX Horus] pkt #")); DBG(horusCounter - 1);
    DBG(F("  alt=")); DBG((int)horusRawBuf[11] | ((int)horusRawBuf[12] << 8));
    DBGLN(F("m"));

    fsk4_idle();    // brief carrier for AGC settle
    delay(100);
    fsk4_preamble();
    fsk4_write(horusCodedBuf, coded_len);
    radio.standby();

    DBGLN(F("[TX Horus] Done."));
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
        String(APRS_SYMBOL_CODE),
        true, (int)(altM * 3.28084f));

    return APRSPacketLib::generateBase91GPSBeaconPacket(
        String(myCallsignFull),
        String(APRS_DESTINATION),
        String(BEACON_PATH),
        String(APRS_SYMBOL_TABLE),
        gpsData) + MY_COMMENT;
}

// ============================================================
// SEND BEACON
//
// Transmission order when HORUS_ENABLED:
//   1. APRS packet  (LoRa, radio already in LoRa mode)
//   2. Horus packet (FSK, radio re-inited to FSK then back to LoRa)
//
// The radio is returned to LoRa mode before sendBeacon() returns
// so that TRACKER_DIGI can call radioStartRx() immediately after,
// and TRACKER_ONLY can call radioSleep() without a mode mismatch.
// ============================================================
void sendBeacon() {
    if (!waitForFix(GPS_FIX_TIMEOUT_MS)) {
        if (!lastPos.valid) {
            DBGLN(F("[Beacon] No position — skipping TX."));
            return;
        }
        DBGLN(F("[Beacon] Using last known position."));
    }

#if BMP280_ENABLED
    if (bmpReady) calibrateBmpFromGPS(BMP280_CAL_SAMPLES);
#endif

#if CUTDOWN_ENABLED
    {
#if BMP280_ENABLED
        float altM = bmpReady ? bmp.readAltitude(bmpSeaLevelHpa) : lastPos.altM;
#else
        float altM = lastPos.altM;
#endif
        checkCutdown(altM);
    }
#endif

    // --- APRS (LoRa) ---
    // Radio is already in LoRa mode from radioInit() in the main loop.
    radioTransmit(buildAPRSPacket());

#if HORUS_ENABLED
    // --- Horus Binary v2 (4FSK) ---
    // Switch to FSK mode, transmit, then return to LoRa so the
    // rest of the operating mode loop works without re-init.
    radioInitFSK();
    transmitHorusV2();
    radioInit();    // back to LoRa for RX / sleep
#endif
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
// CAD DIGIPEATER — unchanged from radiohound
// ============================================================
void runDiGiCAD() {
    radioInit();
    for (int scan = 0; scan < CAD_BATCH_SIZE; scan++) {
        radioCadFlag = false;
        radioStartCad();
        uint32_t cadStart = millis();
        while (!radioCadFlag && millis() - cadStart < 500) LowPower.idle(5);
        if (!radioCadFlag) continue;

        int result = radio.getChannelScanResult();
        if (result == RADIOLIB_LORA_DETECTED) {
            DBGLN(F("[CAD] Preamble! Waiting for packet..."));
            radioRxFlag = false;
            radioStartRx();
            uint32_t rxStart = millis();
            while (!radioRxFlag && millis() - rxStart < 60000UL) LowPower.idle(10);
            if (radioRxFlag) { radioRxFlag = false; processRxPacket(); }
            else             { DBGLN(F("[CAD] RX timeout.")); }
            radioSleep();
            return;
        }
        LowPower.deepSleep(CAD_SCAN_INTERVAL_MS);
    }
    DBG(F("[CAD] Batch done. Deep sleep "));
    DBG(CAD_BATCH_SLEEP_MS / 1000); DBGLN(F("s"));
    radioSleep();
    enterDeepSleep(CAD_BATCH_SLEEP_MS);
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
#if DEBUG_SERIAL
    Serial.begin(DEBUG_BAUD);
    delay(500);
#endif
    DBGLN(F("=== LoRa APRS"
#if HORUS_ENABLED
            " + Horus Binary v2"
#endif
            " Tracker ==="));

    snprintf(myCallsignFull, sizeof(myCallsignFull), "%s-%d", MY_CALLSIGN, MY_SSID);

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

#if OPERATING_MODE == MODE_DIGI_ONLY
    radioStartRx();
#endif
}

// ============================================================
// LOOP
// ============================================================
void loop() {

#if OPERATING_MODE == MODE_TRACKER_ONLY
    radioInit();
    gpsInit();
    sendBeacon();       // APRS then Horus (if enabled), ends in LoRa mode
    gpsPowerOff();
    radioSleep();
    enterDeepSleep((uint32_t)BEACON_INTERVAL_S * 1000UL);

#elif OPERATING_MODE == MODE_TRACKER_DIGI
    radioInit();
    gpsInit();
    sendBeacon();       // APRS then Horus (if enabled), ends in LoRa mode
    gpsPowerOff();
    radioStartRx();     // radio already in LoRa mode after sendBeacon()
    uint32_t end = millis() + (uint32_t)BEACON_INTERVAL_S * 1000UL;
    while (millis() < end) {
        if (radioRxFlag) { radioRxFlag = false; processRxPacket(); }
        flushDigiQueue();
        LowPower.idle(10);
    }

#elif OPERATING_MODE == MODE_DIGI_ONLY
    if (radioRxFlag) { radioRxFlag = false; processRxPacket(); }
    flushDigiQueue();
    LowPower.idle(10);

#elif OPERATING_MODE == MODE_DIGI_CAD
    runDiGiCAD();
#endif
}
