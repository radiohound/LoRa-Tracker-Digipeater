#pragma once

// ============================================================
//  USER CONFIGURATION — balloon-digi branch
//
//  Adds MODE_BALLOON_DIGI to the standard modes.
//  All improvements from main branch (amortised re-init,
//  MODE_DIGI_CAD) are also present here.
// ============================================================

// ------------------------------------------------------------
//  OPERATING MODE
// ------------------------------------------------------------
#define MODE_TRACKER_ONLY  0
#define MODE_TRACKER_DIGI  1
#define MODE_DIGI_ONLY     2
#define MODE_DIGI_CAD      3
#define MODE_BALLOON_DIGI  4

#define OPERATING_MODE     MODE_BALLOON_DIGI

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
#define GPS_I2C_CLOCK       400000
#define GPS_POWER_PIN       RADIOLIB_NC

// ------------------------------------------------------------
//  STANDARD DIGIPEATER SETTINGS
// ------------------------------------------------------------
#define DIGI_PATH_1        "WIDE1-1"
#define DIGI_PATH_2        "WIDE2-1"
#define DIGI_DELAY_MIN_MS  80
#define DIGI_DELAY_MAX_MS  450
#define DIGI_QUEUE_SIZE    4

// ------------------------------------------------------------
//  CAD DIGIPEATER SETTINGS  (MODE_DIGI_CAD)
// ------------------------------------------------------------
#define CAD_SCAN_INTERVAL_MS   2000UL
#define CAD_BATCH_SIZE         15
#define CAD_BATCH_SLEEP_MS     30000UL

// ------------------------------------------------------------
//  BALLOON DIGIPEATER SETTINGS  (MODE_BALLOON_DIGI)
// ------------------------------------------------------------

// Callsign to track (no SSID). "" = accept any balloon.
#define BALLOON_CALLSIGN        "W6ABC"

// Expected TX interval. Starting value — IIR refines it.
#define BALLOON_TX_INTERVAL_MS  30000UL

// Acquisition: CAD scan interval. Must be < packet air time.
// At SF12/BW125 full packet = ~3 s, so 2000 ms is safe.
#define CAD_ACQ_INTERVAL_MS     2000UL

// Acquisition: scans per radio init cycle (amortises init cost).
// 15 scans × 2 s = 30 s active, then radio sleeps.
#define CAD_ACQ_BATCH_SIZE      15

// Acquisition: deep sleep between CAD batches.
#define CAD_ACQ_BATCH_SLEEP_MS  30000UL

// Tracking: listen window width.
// Starts wide, narrows as IIR timing confidence grows.
#define LISTEN_WINDOW_INITIAL_MS  10000UL
#define LISTEN_WINDOW_MIN_MS       4000UL

// Tracking: packets before window narrows to minimum.
#define TIMING_CONVERGE_COUNT   20

// Tracking: misses before returning to acquisition.
#define MAX_MISS_COUNT          3

// IIR smoothing weight (higher = slower adaptation).
#define IIR_WEIGHT              7

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
#define RFSWITCH_PINS   {PA6, PA7, PB3, RADIOLIB_NC, RADIOLIB_NC}
#define RFSWITCH_TABLE  \
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},  \
    {STM32WLx::MODE_RX,    {LOW,  HIGH, LOW}},  \
    {STM32WLx::MODE_TX_LP, {HIGH, LOW,  HIGH}}, \
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}}, \
    END_OF_MODE_TABLE

// --- Seeed LoRa-E5 ---
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

// ------------------------------------------------------------
//  DEBUG
// ------------------------------------------------------------
#define DEBUG_SERIAL       1
#define DEBUG_BAUD         115200
