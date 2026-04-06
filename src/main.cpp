// ============================================================
//  STM32WLE5 LoRa APRS — Tracker + Digipeater + Balloon Tracker
//  Branch: balloon-digi
//
//  Adds MODE_BALLOON_DIGI and MODE_DIGI_CAD.
//  All power improvements from main branch are present here.
//
//  KEY FIX: radioInit() is amortised across multiple CAD scans
//  rather than called on every deepSleep wakeup. This reduces
//  acquisition phase current from ~0.35 mA to ~0.04 mA —
//  enabling multi-year battery life on two AA lithium cells.
//
//  Battery life estimate (2× Energizer L91, balloon monthly):
//    Acquisition phase: ~0.04 mA → negligible
//    Tracking phase:    ~0.7 mA → ~22 mAh/month
//    Total/month:       ~25 mAh
//    Battery life:      3000 mAh / 25 mAh ≈ 10 years
//    (Practical limit: ~5 years due to battery shelf life)
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
//  RADIO / GPS
// ============================================================
STM32WLx radio = new STM32WLx_Module();
static const uint32_t rfswitch_pins[] = RFSWITCH_PINS;
static const Module::RfSwitchMode_t rfswitch_table[] = { RFSWITCH_TABLE };

SFE_UBLOX_GNSS myGPS;
STM32RTC& rtc = STM32RTC::getInstance();

// ============================================================
//  STATE
// ============================================================
static char myCallsignFull[12];

struct Position {
    float lat=0,lon=0,altM=0; int speed=0,course=0; uint8_t sats=0; bool valid=false;
};
static Position lastPos;

static bool radioReady = false;
static bool gpsReady   = false;

// ============================================================
//  BALLOON DIGI STATE
// ============================================================
enum BalloonState { BALLOON_ACQUIRING, BALLOON_TRACKING };
static BalloonState  balloonState     = BALLOON_ACQUIRING;
static uint32_t      lastPacketTime   = 0;
static uint32_t      smoothedInterval = BALLOON_TX_INTERVAL_MS;
static uint8_t       missCount        = 0;
static uint16_t      packetCount      = 0;
static uint32_t      listenWindow     = LISTEN_WINDOW_INITIAL_MS;

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
void sendBeacon();
String buildAPRSPacket();

void runDiGiCAD();
void runAcquisition();
void runTracking();
bool receivePacket(String& out, uint32_t timeoutMs);
bool isOurBalloon(const String& raw);
void digipeatPacket(const String& raw);
void updateTiming(uint32_t rxTimeMs);
uint32_t computeSleepMs();
uint32_t computeListenWindow();

void processRxPacket();
bool needsDigipeat(const String& raw, String& modified);
bool isDuplicate(const String& raw);
void markDuplicate(const String& raw);
uint32_t crc32(const String& s);
void enqueueForDigipeat(const String& pkt);
void flushDigiQueue();
void enterDeepSleep(uint32_t ms);

// ============================================================
//  RADIO INIT  (same sequence as PicoTrack)
// ============================================================
void radioInit() {
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    DBG(F("[Radio] Init... "));
    int s = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
    if (s != RADIOLIB_ERR_NONE) { DBG(F("fail ")); DBGLN(s); while(1) delay(10); }
    s = radio.setTCXO(TCXO_VOLTAGE);
    if (s != RADIOLIB_ERR_NONE) { DBG(F("TCXO fail ")); DBGLN(s); while(1) delay(10); }
    if (radio.setOutputPower(LORA_TX_POWER) == RADIOLIB_ERR_INVALID_OUTPUT_POWER)
        { DBGLN(F("Bad power")); while(1) delay(10); }
    if (radio.setCurrentLimit(LORA_OCP_MA) == RADIOLIB_ERR_INVALID_CURRENT_LIMIT)
        { DBGLN(F("Bad OCP")); while(1) delay(10); }
    radio.setSyncWord(LORA_SYNC_WORD);
    radio.setPreambleLength(LORA_PREAMBLE);
    radio.setPacketReceivedAction(onRadioRx);
    radio.setChannelScanAction(onRadioCad);
    radioReady = true;
    DBGLN(F("OK"));
}

// ============================================================
//  GPS INIT  (only when needed — tracker modes)
// ============================================================
void gpsInit() {
    Wire.begin();
    Wire.setClock(GPS_I2C_CLOCK);
    gpsPowerOn();
    if (!myGPS.begin()) { DBGLN(F("[GPS] Not found.")); return; }
    myGPS.setUART1Output(0); myGPS.setUART2Output(0);
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
    snprintf(myCallsignFull, sizeof(myCallsignFull), "%s-%d", MY_CALLSIGN, MY_SSID);
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
#elif OPERATING_MODE == MODE_BALLOON_DIGI
    DBGLN(F("Mode: BALLOON_DIGI — acquiring..."));
    balloonState = BALLOON_ACQUIRING;
#elif OPERATING_MODE == MODE_DIGI_CAD
    DBGLN(F("Mode: DIGI_CAD"));
#endif
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
#if OPERATING_MODE == MODE_TRACKER_ONLY
    radioInit(); gpsInit();
    sendBeacon(); gpsPowerOff(); radioSleep();
    enterDeepSleep((uint32_t)BEACON_INTERVAL_S * 1000UL);

#elif OPERATING_MODE == MODE_TRACKER_DIGI
    radioInit(); gpsInit();
    sendBeacon(); gpsPowerOff();
    radioStartRx();
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

#elif OPERATING_MODE == MODE_BALLOON_DIGI
    // Re-init radio after each deepSleep wakeup.
    // GPS is never initialised in this mode.
    radioInit();
    switch (balloonState) {
        case BALLOON_ACQUIRING: runAcquisition(); break;
        case BALLOON_TRACKING:  runTracking();    break;
    }
#endif
}

// ============================================================
//  CAD DIGIPEATER  (MODE_DIGI_CAD)
//  Shared with balloon mode for the standard digi use case.
// ============================================================
void runDiGiCAD() {
    radioInit();
    for (int scan = 0; scan < CAD_BATCH_SIZE; scan++) {
        radioCadFlag = false;
        radioStartCad();
        uint32_t t = millis();
        while (!radioCadFlag && millis() - t < 500) LowPower.idle(5);
        if (!radioCadFlag) continue;

        if (radio.getChannelScanResult() == RADIOLIB_LORA_DETECTED) {
            DBG(F("[CAD] Preamble — waiting for next packet... "));
            radioRxFlag = false;
            radioStartRx();
            uint32_t rxStart = millis();
            while (!radioRxFlag && millis() - rxStart < 60000UL)
                LowPower.idle(10);
            if (radioRxFlag) { radioRxFlag = false; processRxPacket(); }
            else DBGLN(F("timeout."));
            radioSleep();
            return;
        }
        LowPower.deepSleep(CAD_SCAN_INTERVAL_MS);
    }
    DBG(F("[CAD] Batch done. Sleep "));
    DBG(CAD_BATCH_SLEEP_MS / 1000); DBGLN(F("s"));
    radioSleep();
    enterDeepSleep(CAD_BATCH_SLEEP_MS);
}

// ============================================================
//  BALLOON ACQUISITION
//
//  KEY FIX: CAD runs in batches of CAD_ACQ_BATCH_SIZE scans.
//  radioInit() is called once per batch, not once per scan.
//  This amortises the ~100 ms / 7 mA init overhead across
//  CAD_ACQ_BATCH_SIZE × CAD_ACQ_INTERVAL_MS of scan time.
//
//  Power per 60 s cycle (batch=15, scan=2s, sleep=30s):
//    Active 30 s: init(100ms@7mA) + 15×CAD(65ms@1.5mA)
//                 + 15×sleep(1935ms@0.002mA) → avg ~0.074 mA
//    Sleep  30 s: Stop2 → avg 0.002 mA
//    60 s average: ~0.038 mA
// ============================================================
void runAcquisition() {
    // Run a batch of CAD scans with one radioInit() at the start.
    for (int scan = 0; scan < CAD_ACQ_BATCH_SIZE; scan++) {
        radioCadFlag = false;
        radioStartCad();

        uint32_t t = millis();
        while (!radioCadFlag && millis() - t < 500) LowPower.idle(5);
        if (!radioCadFlag) { LowPower.deepSleep(CAD_ACQ_INTERVAL_MS); continue; }

        if (radio.getChannelScanResult() == RADIOLIB_LORA_DETECTED) {
            // Preamble detected — receive the full packet.
            DBG(F("[Acq] Preamble! Receiving... "));
            radioRxFlag = false;
            radioStartRx();

            String packet;
            bool got = receivePacket(packet, 5000);

            if (got && isOurBalloon(packet)) {
                // Synced!
                lastPacketTime   = millis();
                smoothedInterval = BALLOON_TX_INTERVAL_MS;
                missCount        = 0;
                packetCount      = 1;
                listenWindow     = LISTEN_WINDOW_INITIAL_MS;
                balloonState     = BALLOON_TRACKING;
                DBG(F("[Acq] SYNCED to ")); DBGLN(String(BALLOON_CALLSIGN));
                digipeatPacket(packet);
                radioSleep();
                enterDeepSleep(computeSleepMs());
                return;
            } else {
                DBG(got ? F("[Acq] Wrong callsign.\n") : F("[Acq] No decode.\n"));
                // Continue scanning in this batch.
            }
        }
        // Channel free or wrong callsign — sleep until next scan.
        // Radio stays initialised — no re-init between scans.
        LowPower.deepSleep(CAD_ACQ_INTERVAL_MS);
    }

    // Batch complete with no balloon detected.
    // Sleep radio and MCU for the inter-batch period.
    DBG(F("[Acq] Batch done. Sleep "));
    DBG(CAD_ACQ_BATCH_SLEEP_MS / 1000); DBGLN(F("s"));
    radioSleep();
    enterDeepSleep(CAD_ACQ_BATCH_SLEEP_MS);
    // On wakeup: loop() calls radioInit() → runAcquisition() → new batch.
}

// ============================================================
//  BALLOON TRACKING
//  Arrive here after deepSleep — already woken early relative
//  to expected TX. Open RX window, digipeat, update timing,
//  sleep until next expected packet.
// ============================================================
void runTracking() {
    DBG(F("[Track] Window=")); DBG(listenWindow); DBGLN(F("ms"));
    radioRxFlag = false;
    radioStartRx();

    String packet;
    bool got = receivePacket(packet, listenWindow);

    if (got && isOurBalloon(packet)) {
        updateTiming(millis());
        digipeatPacket(packet);
        missCount = 0;
        DBG(F("[Track] OK interval=")); DBG(smoothedInterval);
        DBG(F("ms window=")); DBG(listenWindow);
        DBG(F("ms miss=")); DBGLN(missCount);
    } else {
        missCount++;
        DBG(F("[Track] MISS (")); DBG(missCount);
        DBG(F("/")); DBGLN(MAX_MISS_COUNT);
        if (missCount >= MAX_MISS_COUNT) {
            DBGLN(F("[Track] Lost — back to ACQUIRING."));
            balloonState     = BALLOON_ACQUIRING;
            smoothedInterval = BALLOON_TX_INTERVAL_MS;
            listenWindow     = LISTEN_WINDOW_INITIAL_MS;
            packetCount      = 0;
            radioSleep();
            enterDeepSleep(CAD_ACQ_INTERVAL_MS);
            return;
        }
    }
    radioSleep();
    enterDeepSleep(computeSleepMs());
}

// ============================================================
//  BALLOON HELPERS
// ============================================================
bool receivePacket(String& out, uint32_t timeoutMs) {
    uint32_t start = millis();
    while (!radioRxFlag && millis() - start < timeoutMs) LowPower.idle(5);
    if (!radioRxFlag) return false;
    radioRxFlag = false;
    if (radio.readData(out) != RADIOLIB_ERR_NONE) return false;
    if (out.length() > 3 && out[0] == '<') out = out.substring(3);
    DBG(F("[RX] ")); DBGLN(out);
    return true;
}

bool isOurBalloon(const String& raw) {
    if (strlen(BALLOON_CALLSIGN) == 0) return true;
    int arrow = raw.indexOf('>');
    if (arrow < 0) return false;
    String src = raw.substring(0, arrow);
    int clen = strlen(BALLOON_CALLSIGN);
    return src.startsWith(BALLOON_CALLSIGN) &&
           ((int)src.length() == clen || src[clen] == '-');
}

void digipeatPacket(const String& raw) {
    String modified;
    if (needsDigipeat(raw, modified) && !isDuplicate(raw)) {
        markDuplicate(raw);
        delay(100);
        radio.standby();
        radioTransmit(modified);
        radioStartRx();
        DBG(F("[Digi] ")); DBGLN(modified);
    }
}

void updateTiming(uint32_t rxTimeMs) {
    if (lastPacketTime > 0 && packetCount > 1) {
        uint32_t measured = rxTimeMs - lastPacketTime;
        if (measured > smoothedInterval / 2 && measured < smoothedInterval * 2) {
            smoothedInterval = (smoothedInterval * IIR_WEIGHT + measured)
                               / (IIR_WEIGHT + 1);
        }
    }
    lastPacketTime = rxTimeMs;
    packetCount++;
    listenWindow = computeListenWindow();
}

uint32_t computeSleepMs() {
    uint32_t half = listenWindow / 2;
    return (smoothedInterval > half) ? smoothedInterval - half : 100;
}

uint32_t computeListenWindow() {
    if (packetCount >= TIMING_CONVERGE_COUNT) return LISTEN_WINDOW_MIN_MS;
    uint32_t range = LISTEN_WINDOW_INITIAL_MS - LISTEN_WINDOW_MIN_MS;
    return LISTEN_WINDOW_INITIAL_MS - (range * packetCount / TIMING_CONVERGE_COUNT);
}

// ============================================================
//  RADIO HELPERS
// ============================================================
void radioSleep()    { radio.sleep(); radioReady = false; }
void radioStartRx()  { radio.startReceive(); }
void radioStartCad() { radio.startChannelScan(); }

bool radioTransmit(const String& tnc2) {
    String pkt = "<\xff\x01" + tnc2;
    DBG(F("[TX] ")); DBGLN(pkt);
    return radio.transmit(pkt) == RADIOLIB_ERR_NONE;
}

void IRAM_ATTR onRadioRx()  { radioRxFlag  = true; }
void IRAM_ATTR onRadioCad() { radioCadFlag = true; }

// ============================================================
//  GPS HELPERS
// ============================================================
void gpsPowerOn()  {
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
bool waitForFix(uint32_t tms) {
    uint32_t s = millis();
    while (millis()-s < tms) {
        myGPS.checkUblox();
        if (myGPS.getPVT() && myGPS.getFixType()!=0 && myGPS.getSIV()>3) {
            lastPos.lat=myGPS.getLatitude()/1e7f;
            lastPos.lon=myGPS.getLongitude()/1e7f;
            lastPos.altM=myGPS.getAltitudeMSL()/1000.0f;
            lastPos.speed=(int)(myGPS.getGroundSpeed()/514.44f);
            lastPos.course=myGPS.getHeading()/100000;
            lastPos.sats=myGPS.getSIV(); lastPos.valid=true;
            return true;
        }
        delay(100);
    }
    return false;
}

// ============================================================
//  BEACON
// ============================================================
String buildAPRSPacket() {
    APRSPacket pkt;
    pkt.source=String(myCallsignFull); pkt.path=String(BEACON_PATH);
    pkt.symbol=APRS_SYMBOL_CODE; pkt.overlay=APRS_SYMBOL_TABLE;
    pkt.latitude=lastPos.lat; pkt.longitude=lastPos.lon;
    pkt.altitude=(int)(lastPos.altM*3.28084f);
    pkt.speed=lastPos.speed; pkt.course=lastPos.course;
    pkt.comment=String(MY_COMMENT);
    return APRSPacketLib::generatePositionPacket(pkt);
}
void sendBeacon() {
    if (!waitForFix(GPS_FIX_TIMEOUT_MS) && !lastPos.valid) {
        DBGLN(F("[Beacon] No position.")); return;
    }
    radioTransmit(buildAPRSPacket());
}

// ============================================================
//  STANDARD DIGIPEATER
// ============================================================
void processRxPacket() {
    String raw;
    if (radio.readData(raw) != RADIOLIB_ERR_NONE) { radioStartRx(); return; }
    if (raw.length()>3 && raw[0]=='<') raw=raw.substring(3);
    DBG(F("[RX] ")); DBGLN(raw);
    String modified;
    if (needsDigipeat(raw,modified) && !isDuplicate(raw)) {
        markDuplicate(raw); enqueueForDigipeat(modified);
    }
    radioStartRx();
}

bool needsDigipeat(const String& raw, String& modified) {
    int ci=raw.indexOf(':'); if(ci<0) return false;
    String hdr=raw.substring(0,ci), pay=raw.substring(ci);
    int ai=hdr.indexOf('>'); if(ai<0) return false;
    String src=hdr.substring(0,ai), dap=hdr.substring(ai+1);
    if(src.startsWith(MY_CALLSIGN)) return false;
    int cmi=dap.indexOf(','); if(cmi<0) return false;
    String dest=dap.substring(0,cmi), path=dap.substring(cmi+1);
    String f[8]; int nf=0;
    { String r=path;
      while(r.length()>0&&nf<8){int s=r.indexOf(',');if(s<0){f[nf++]=r;break;}f[nf++]=r.substring(0,s);r=r.substring(s+1);}
    }
    bool digi=false;
    for(int i=0;i<nf&&!digi;i++){
        if(f[i].endsWith("*")) continue;
        if(f[i].equalsIgnoreCase(DIGI_PATH_1)){f[i]=String(myCallsignFull)+"*";digi=true;}
        else if(strlen(DIGI_PATH_2)>0&&(f[i].startsWith("WIDE2-")||f[i].startsWith("wide2-"))){
            int n=f[i].substring(6).toInt();
            if(n>=1){
                if(n==1){f[i]=String(myCallsignFull)+"*";}
                else{if(nf<8){for(int j=nf;j>i;j--)f[j]=f[j-1];f[i]=String(myCallsignFull)+"*";f[i+1]="WIDE2-"+String(n-1);nf++;i++;}}
                digi=true;
            }
        }
    }
    if(!digi) return false;
    String np; for(int i=0;i<nf;i++){if(i>0)np+=',';np+=f[i];}
    modified=src+">"+dest+","+np+pay; return true;
}

uint32_t crc32(const String& s){
    uint32_t c=0xFFFFFFFF;
    for(size_t i=0;i<s.length();i++){c^=(uint8_t)s[i];for(int b=0;b<8;b++)c=(c&1)?(c>>1)^0xEDB88320:(c>>1);}
    return ~c;
}
bool isDuplicate(const String& r){uint32_t h=crc32(r);for(int i=0;i<8;i++)if(dedupe[i]==h)return true;return false;}
void markDuplicate(const String& r){dedupe[dedupeIdx]=crc32(r);dedupeIdx=(dedupeIdx+1)&7;}
void enqueueForDigipeat(const String& p){
    for(int i=0;i<DIGI_QUEUE_SIZE;i++)if(!digiQueue[i].used){
        uint32_t d=random(DIGI_DELAY_MIN_MS,DIGI_DELAY_MAX_MS+1);
        digiQueue[i]={p,millis()+d,true};return;
    }
    DBGLN(F("[Digi] Queue full."));
}
void flushDigiQueue(){
    for(int i=0;i<DIGI_QUEUE_SIZE;i++)
        if(digiQueue[i].used&&millis()>=digiQueue[i].txAfterMs){
            radio.standby();radioTransmit(digiQueue[i].packet);
            digiQueue[i].used=false;radioStartRx();
        }
}

// ============================================================
//  SLEEP
// ============================================================
void enterDeepSleep(uint32_t ms){
    DBG(F("[Sleep] ")); DBG(ms); DBGLN(F("ms"));
#if DEBUG_SERIAL
    Serial.flush(); delay(10);
#endif
    LowPower.deepSleep(ms);
}
