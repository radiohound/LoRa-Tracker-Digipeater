#pragma once

// ============================================================
//  USER CONFIGURATION
//
//  RF switch, TCXO, GPS wiring, and radio init order verified
//  against PicoTrack by K6ATV:
//  https://github.com/radiohound/PicoTrack
// ============================================================

// ------------------------------------------------------------
//  OPERATING MODE
//   TRACKER_ONLY  – beacon GPS, deep sleep between beacons
//   TRACKER_DIGI  – beacon GPS + digipeat while awake
//   DIGI_ONLY     – continuous RX digipeater (~5-6 mA)
//   DIGI_CAD      – Channel Activity Detection (CAD)-based digipeater (~0.1 mA, misses first
//                   packet of each burst but catches all others)
// ------------------------------------------------------------
#define MODE_TRACKER_ONLY  0
#define MODE_TRACKER_DIGI  1
#define MODE_DIGI_ONLY     2
#define MODE_DIGI_CAD      3

#define OPERATING_MODE     MODE_DIGI_CAD   // <-- change this

// ------------------------------------------------------------
//  STATION IDENTIFICATION
// ------------------------------------------------------------
#define MY_CALLSIGN        "N0CALL"
#define MY_SSID            9
#define MY_COMMENT         "STM32WLE5 LoRa APRS"

#define APRS_SYMBOL_TABLE  '/'
#define APRS_SYMBOL_CODE   '>'
#define BEACON_PATH        "WIDE1-1"
#define APRS_DESTINATION   "APZP01"

// ------------------------------------------------------------
//  TRACKER SETTINGS
// ------------------------------------------------------------
#define BEACON_INTERVAL_S   120
#define GPS_FIX_TIMEOUT_MS  90000UL
#define GPS_I2C_CLOCK       400000      // Hz; PA9=SCL, PA10=SDA
#define GPS_POWER_PIN       RADIOLIB_NC // GPIO to cut GPS power,
                                        // RADIOLIB_NC = always on

// ------------------------------------------------------------
//  STANDARD DIGIPEATER SETTINGS
// ------------------------------------------------------------
#define DIGI_PATH_1        "WIDE1-1"
#define DIGI_PATH_2        "WIDE2-1"   // "" = first-hop only
#define DIGI_DELAY_MIN_MS  80
#define DIGI_DELAY_MAX_MS  450
#define DIGI_QUEUE_SIZE    4

// ------------------------------------------------------------
//  CAD DIGIPEATER SETTINGS  (MODE_DIGI_CAD only)
//
//  The digipeater sleeps between CAD scans. When a preamble
//  is detected it switches to full RX for the NEXT packet.
//  It misses the triggering packet but catches all subsequent
//  ones. In a typical APRS environment with retries and
//  multi-hop paths this is fully acceptable behaviour.
//
//  CAD_SCAN_INTERVAL_MS must be < packet air time at your SF.
//  At SF12/BW125 a full packet is ~3 s, so 2000 ms is safe.
//
//  CAD_BATCH_SIZE: number of CAD scans per radio init cycle.
//  Higher = lower average power (amortises init overhead).
//  Lower = radio sleeps more deeply between batches.
//  Recommended: 10-30 (20-60 seconds per init cycle).
// ------------------------------------------------------------
#define CAD_SCAN_INTERVAL_MS   2000UL
#define CAD_BATCH_SIZE         15      // scans before radio sleeps
                                       // 15 × 2s = 30s per cycle

// Deep sleep between CAD batches. After CAD_BATCH_SIZE scans
// with no detection, the radio is fully slept and the MCU
// enters Stop2 for this duration before re-initialising.
// Longer = lower power. Must be reasonable for your traffic.
// At 30s you check every ~60s total. At 60s you check every ~90s.
#define CAD_BATCH_SLEEP_MS     30000UL

// ------------------------------------------------------------
//  RADIO SETTINGS
// ------------------------------------------------------------
#define LORA_FREQ          433.775
#define LORA_BW            125.0
#define LORA_SF            12
#define LORA_CR            5
#define LORA_PREAMBLE      8
#define LORA_SYNC_WORD     0x12
#define LORA_TX_POWER      22
#define LORA_OCP_MA        140
#define TCXO_VOLTAGE       1.7f

// RF switch — Ebyte E77-400MBL-01 (verified against PicoTrack)
// PA6=RF_TXEN, PA7=RF_RXEN, PB3=TX LED
#define RFSWITCH_PINS   {PA6, PA7, PB3, RADIOLIB_NC, RADIOLIB_NC}
#define RFSWITCH_TABLE  \
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},  \
    {STM32WLx::MODE_RX,    {LOW,  HIGH, LOW}},  \
    {STM32WLx::MODE_TX_LP, {HIGH, LOW,  HIGH}}, \
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}}, \
    END_OF_MODE_TABLE

// --- Seeed LoRa-E5 / LoRa-E5 mini ---
// #define RFSWITCH_PINS   {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC}
// #define RFSWITCH_TABLE  \
//     {STM32WLx::MODE_IDLE,  {LOW,  LOW}},  \
//     {STM32WLx::MODE_RX,    {HIGH, LOW}},  \
//     {STM32WLx::MODE_TX_HP, {LOW,  HIGH}}, \
//     END_OF_MODE_TABLE

// --- ST Nucleo-WL55JC ---
// #define RFSWITCH_PINS   {PC3, PC4, PC5, RADIOLIB_NC, RADIOLIB_NC}
// #define RFSWITCH_TABLE  \
//     {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},  \
//     {STM32WLx::MODE_RX,    {HIGH, HIGH, LOW}},   \
//     {STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH}},  \
//     {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}},  \
//     END_OF_MODE_TABLE

// --- RAK3172-E ---
// #define RFSWITCH_PINS   {PB8, PC13, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC}
// #define RFSWITCH_TABLE  \
//     {STM32WLx::MODE_IDLE,  {LOW,  LOW}},  \
//     {STM32WLx::MODE_RX,    {HIGH, LOW}},  \
//     {STM32WLx::MODE_TX_LP, {LOW,  HIGH}}, \
//     END_OF_MODE_TABLE

// ------------------------------------------------------------
//  BMP280 BAROMETRIC ALTIMETER  (optional — on the same I2C bus as GPS)
//
//  When enabled and a BMP280 is detected, altitude is taken from
//  the barometer instead of GPS. GPS altitude is used immediately
//  after each fix to calibrate the BMP280's sea-level pressure
//  reference (BMP280_CAL_SAMPLES readings averaged at 1 Hz).
//  If no BMP280 is found at runtime the tracker falls back to
//  GPS altitude automatically — no code change needed.
//
//  Set BMP280_ENABLED to 0 to exclude all BMP280 code entirely.
// ------------------------------------------------------------
#define BMP280_ENABLED     1       // 1 = use if present, 0 = disable
#define BMP280_I2C_ADDR    0x76    // 0x76 (SDO→GND) or 0x77 (SDO→VCC)
#define BMP280_CAL_SAMPLES 8       // GPS altitude samples to average
                                   // for sea-level pressure calibration

// ------------------------------------------------------------
//  DEBUG
// ------------------------------------------------------------
#define DEBUG_SERIAL       1
#define DEBUG_BAUD         115200
