#pragma once
// ============================================================
// LACF 3.0 digipeater (digipeat only: no GPS, no iGate, no tracking)
// Target: Ebyte E77-400MBL-01 (STM32WLE5CC)
// Build:  PlatformIO environment "e77_lacf3_digi"
// ============================================================

// ---- IDENTITY ------------------------------------------------
// The digipeater neither relays nor beacons until a real callsign is set here.
#define MY_CALLSIGN         "N0CALL"
#define MY_SSID             10              // 0 = no SSID

// ---- DIGIPEATER ----------------------------------------------
//   D3_MODE_FILLIN  WIDE1-1 and packets addressed to this callsign (a low, local fill-in digi)
//   D3_MODE_WIDE    also WIDE2-1 and WIDE2-2 (a high site that covers a wide area)
#define DIGI_MODE           D3_MODE_WIDE
#define DIGI_DELAY_MIN_MS   100             // random wait before repeating, so two digis rarely key at once
#define DIGI_DELAY_MAX_MS   500
#define DIGI_BUSY_TRIES     6               // channel busy (CAD): wait and look again this many times, then drop
#define DIGI_BUSY_MIN_MS    500
#define DIGI_BUSY_MAX_MS    2000
#define DIGI_QUEUE_SIZE     4
#define DEDUPE_SECONDS      30              // the same packet heard again within this time is not repeated again
#define DEDUPE_SLOTS        32

// ---- BEACON (fixed position, plain text so every receiver can read it) ----
// Leave LAT/LON at 0 to send no beacon.
#define BEACON_LAT          0.0             // degrees, north positive
#define BEACON_LON          0.0             // degrees, east positive
#define BEACON_SYMBOL_TABLE 'L'             // 'L' overlay on the digipeater symbol: a LoRa digi
#define BEACON_SYMBOL_CODE  '#'
#define BEACON_COMMENT      "LACF 3 digi, STM32WLE5"
#define BEACON_PATH         ""              // e.g. "WIDE1-1"; a high digi usually beacons with no path
#define BEACON_INTERVAL_MIN 60
#define BEACON_FIRST_S      30              // the first beacon after power-up
// Destination (tocall): experimental APZ until APLAB? is assigned for the STM32WLE5 projects.
#define APRS_DESTINATION    "APZLD3"

// ---- LORA (LoRa APRS standard: 433.775 MHz, 125 kHz, SF12, CR 4/5, sync word 0x12) ----
#define LORA_FREQ           433.775
#define LORA_BW             125.0
#define LORA_SF             12
#define LORA_CR             5
#define LORA_TX_POWER       22              // dBm, the STM32WL's maximum
#define LORA_OCP_MA         140
#define LORA_SYNC_WORD      0x12
#define LORA_PREAMBLE       8
// Boosted receive gain: about 2 to 3 dB more sensitivity for about 0.7 mA. At a noisy site it gains nothing.
#define RX_BOOSTED_GAIN     1

// ---- TCXO -----------------------------------------------------
// E77 modules with SN >= 3202995 have a TCXO. Same value as PicoTrack and LoRa-Tracker-Digipeater.
#define TCXO_VOLTAGE        1.7f

// ---- RF SWITCH: E77 MBL board (verified on PicoTrack) ------------
#define RFSWITCH_PINS   {PA6, PA7, PB3, RADIOLIB_NC, RADIOLIB_NC}
#define RFSWITCH_TABLE  \
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},  \
    {STM32WLx::MODE_RX,    {LOW,  HIGH, LOW}},  \
    {STM32WLx::MODE_TX_LP, {HIGH, LOW,  HIGH}}, \
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}}, \
    END_OF_MODE_TABLE

// ---- HOUSEKEEPING ---------------------------------------------
#define DEBUG_SERIAL        1
#define DEBUG_BAUD          115200
#define STATS_EVERY_MIN     10              // a counters line on the serial port
#define WATCHDOG_S          20              // restart if the loop stops for this long (a remote site has nobody to press reset)
