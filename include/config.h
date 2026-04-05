#pragma once

// ============================================================
//  USER CONFIGURATION — edit everything in this file
//
//  RF switch, TCXO, GPS wiring, and radio init order are all
//  verified against the working PicoTrack source by K6ATV:
//  https://github.com/radiohound/PicoTrack
// ============================================================

// ------------------------------------------------------------
//  OPERATING MODE
//   TRACKER_ONLY  – beacon GPS position, no digipeating
//   TRACKER_DIGI  – beacon GPS position AND digipeat others
//   DIGI_ONLY     – low-power digipeater, no GPS needed
// ------------------------------------------------------------
#define MODE_TRACKER_ONLY  0
#define MODE_TRACKER_DIGI  1
#define MODE_DIGI_ONLY     2

#define OPERATING_MODE     MODE_TRACKER_DIGI   // <-- change this

// ------------------------------------------------------------
//  STATION IDENTIFICATION
// ------------------------------------------------------------
#define MY_CALLSIGN        "N0CALL"
#define MY_SSID            9          // 0-15
#define MY_COMMENT         "STM32WLE5 LoRa APRS"

// APRS symbol: table '/' = primary, '\\' = alternate.
// Code: '>' = car, '-' = house, 'O' = balloon, '^' = aircraft
#define APRS_SYMBOL_TABLE  '/'
#define APRS_SYMBOL_CODE   '>'

// Path for YOUR own beacon packets.
#define BEACON_PATH        "WIDE1-1"

// APRS destination field. APZxxx = experimental (APRS101 p.13)
#define APRS_DESTINATION   "APZP01"

// ------------------------------------------------------------
//  TRACKER SETTINGS  (ignored in DIGI_ONLY mode)
// ------------------------------------------------------------

// Beacon interval in seconds.
#define BEACON_INTERVAL_S  120

// How long to wait for a GPS fix before giving up (ms).
#define GPS_FIX_TIMEOUT_MS  90000UL

// GPS wiring: PicoTrack connects u-blox to the E77 dev board
// via I2C on PA9 (SCL) and PA10 (SDA), exactly as labelled
// on the board headers. The SparkFun library uses Wire (I2C).
// Wire.begin() / Wire.setClock() are called in radioAndGpsInit().
#define GPS_I2C_CLOCK      400000     // Hz; 400 kHz fast mode

// Optional GPIO to cut power to the GPS between fixes.
// PicoTrack leaves GPS always powered — set to RADIOLIB_NC.
// Wire up a P-FET or load switch and change this pin if needed.
#define GPS_POWER_PIN      RADIOLIB_NC

// ------------------------------------------------------------
//  DIGIPEATER SETTINGS  (ignored in TRACKER_ONLY mode)
// ------------------------------------------------------------

#define DIGI_PATH_1        "WIDE1-1"
#define DIGI_PATH_2        "WIDE2-1"  // set "" to handle first-hop only

// Random pre-TX delay (ms) to reduce collision probability.
#define DIGI_DELAY_MIN_MS  80
#define DIGI_DELAY_MAX_MS  450

// Pending retransmit queue depth.
#define DIGI_QUEUE_SIZE    4

// ------------------------------------------------------------
//  RADIO SETTINGS
// ------------------------------------------------------------

// Frequency confirmed working in PicoTrack on E77 dev board.
#define LORA_FREQ          433.775   // MHz

// Modem parameters matching PicoTrack: radio.begin(433.775, 125, 12, 5)
#define LORA_BW            125.0    // kHz
#define LORA_SF            12
#define LORA_CR            5        // coding rate 4/5
// PicoTrack uses RadioLib defaults for preamble and sync word,
// so these are never passed to begin() — they are used if you
// need to call setSpreadingFactor() or setSyncWord() separately.
#define LORA_PREAMBLE      8
#define LORA_SYNC_WORD     0x12    // LoRa-APRS / private network

// Output power — set via setOutputPower() after begin().
// PicoTrack sequence: begin() → aprs.begin() → setTCXO() →
//                     setOutputPower() → setCurrentLimit()
#define LORA_TX_POWER      22      // dBm; RadioLib caps at hardware max

// OCP confirmed by PicoTrack author as required for full TX power.
#define LORA_OCP_MA        140     // mA

// ------------------------------------------------------------
//  TCXO VOLTAGE
//
//  PicoTrack calls radio.setTCXO(1.7) AFTER radio.begin() and
//  AFTER aprs.begin(), not as a parameter to begin().
//  Comment from PicoTrack: "set appropriate TCXO voltage for
//  Nucleo WL55JC1, WL55JC2, or E77 boards" → 1.7 V.
//  Set to 0.0f only if you have a passive crystal module.
// ------------------------------------------------------------
#define TCXO_VOLTAGE       1.7f    // volts

// ------------------------------------------------------------
//  RF SWITCH — copied directly from PicoTrack (proven working)
//
//  PA6 = RF_TXEN  HIGH = TX active
//  PA7 = RF_RXEN  HIGH = RX active
//  PB3 = TX LED   HIGH during TX (via the switch table)
//
//  PicoTrack includes both TX_LP and TX_HP pointing to the
//  same pin state. The E77 has only the HP path physically
//  wired, but defining LP lets RadioLib's power range logic
//  work without errors across all power levels.
// ------------------------------------------------------------
#define RFSWITCH_PINS   {PA6, PA7, PB3, RADIOLIB_NC, RADIOLIB_NC}
#define RFSWITCH_TABLE  \
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},  \
    {STM32WLx::MODE_RX,    {LOW,  HIGH, LOW}},  \
    {STM32WLx::MODE_TX_LP, {HIGH, LOW,  HIGH}}, \
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}}, \
    END_OF_MODE_TABLE

// --- Seeed LoRa-E5 / LoRa-E5 mini (HP only, PA4/PA5) ---
// #define RFSWITCH_PINS   {PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC}
// #define RFSWITCH_TABLE  \
//     {STM32WLx::MODE_IDLE,  {LOW,  LOW}},  \
//     {STM32WLx::MODE_RX,    {HIGH, LOW}},  \
//     {STM32WLx::MODE_TX_HP, {LOW,  HIGH}}, \
//     END_OF_MODE_TABLE

// --- ST Nucleo-WL55JC (LP + HP, PC3/PC4/PC5) ---
// #define RFSWITCH_PINS   {PC3, PC4, PC5, RADIOLIB_NC, RADIOLIB_NC}
// #define RFSWITCH_TABLE  \
//     {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},  \
//     {STM32WLx::MODE_RX,    {HIGH, HIGH, LOW}},   \
//     {STM32WLx::MODE_TX_LP, {HIGH, HIGH, HIGH}},  \
//     {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}},  \
//     END_OF_MODE_TABLE

// --- RAK3172-E (LP only, PB8/PC13) ---
// #define RFSWITCH_PINS   {PB8, PC13, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC}
// #define RFSWITCH_TABLE  \
//     {STM32WLx::MODE_IDLE,  {LOW,  LOW}},  \
//     {STM32WLx::MODE_RX,    {HIGH, LOW}},  \
//     {STM32WLx::MODE_TX_LP, {LOW,  HIGH}}, \
//     END_OF_MODE_TABLE

// ------------------------------------------------------------
//  DEBUG / SERIAL OUTPUT
// ------------------------------------------------------------
#define DEBUG_SERIAL       1
#define DEBUG_BAUD         115200
