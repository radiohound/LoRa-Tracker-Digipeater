#pragma once
// ============================================================
// LACF 3.0 digipeater with the PPM tables (digipeat only: no GPS, no iGate, no tracking)
// Target: Ebyte E77-400MBL-01 (STM32WLE5CC)
// Build:  PlatformIO environment "e77_lacf3_digi_full"
//
// These values are the DEFAULTS. Over the serial port (115200 baud) "help" lists commands that change nearly all of them
// ("set call K6ATV-2", "set mode wide", ..., then "save" to keep them in flash); a saved record overrides the defaults here.
// Fixed at compile time: TCXO, RF switch, current limit, queue and table sizes, watchdog, serial speed.
// ============================================================

// ---- IDENTITY ------------------------------------------------
// The digipeater neither relays nor beacons until a real callsign is set (here, or over the serial port: set call K6ATV-2).
#define MY_CALLSIGN         "N0CALL"
#define MY_SSID             10              // 0 = no SSID

// ---- DIGIPEATER ----------------------------------------------
//   D3_MODE_FILLIN  WIDE1-1 and packets addressed to this callsign (a low, local fill-in digi)
//   D3_MODE_WIDE    also WIDE2-1 and WIDE2-2 (a high site that covers a wide area)
#define DIGI_MODE           D3_MODE_WIDE
#define DIGI_DELAY_MIN_MS   100             // random wait before repeating, so two digis rarely key at once
#define DIGI_DELAY_MAX_MS   500
#define DIGI_QUEUE_SIZE     4

// ---- LISTEN BEFORE TALK (as the LoRa APRS Pi and the iGate firmware) ----
// Before every transmission the channel must be found free CAD_DIFS_SLOTS times in a row (one slot = one channel
// activity check, about 4 symbols: 131 ms at SF12). A busy channel means a random wait of 1 to CAD_BACKOFF_MAX slots,
// still receiving. After CAD_MAX_WAIT_S without a free channel a repeat is given up (a beacon is tried again in 30 s).
#define CAD_DIFS_SLOTS      2
#define CAD_BACKOFF_MAX     4
#define CAD_MAX_WAIT_S      10

// ---- CANCEL ON HEAR ----
// A repeat still waiting (for its delay or a free channel) is dropped when the same packet is heard repeated by
// another digipeater first (more used hops in its path than when this digi heard it).
#define CANCEL_ON_HEAR      1

// ---- MOVING STATIONS ONLY ----
// 1: repeat only what moves, plus whatever asks for this digi by name. Every packet heard updates a list of
// stations (position, when last heard, signal), printed with the counters on the serial port.
//   Always repeated: a path that names this digi (e.g. a tracker set to path K6ATV-10), and destinations (tocalls)
//   that start with a prefix in ALWAYS_REPEAT_TOCALLS.
//   Repeated: a station's first position, and any position MOVE_THRESHOLD_M or more from the last one repeated.
//   Not repeated: a fixed station heard again in the same place (unless STATIONARY_REPEAT_MIN is set).
// 0: an ordinary digipeater (the path rule alone decides). The station list is kept either way.
#define DIGI_ONLY_MOVING          0
#define MOVE_THRESHOLD_M          100
#define STATIONARY_REPEAT_MIN     0         // also repeat a fixed station's position after this many minutes (0 = never)
#define MOVING_REPEAT_MESSAGES    1         // messages and acks (they have no position)
#define MOVING_REPEAT_OTHER       0         // status, telemetry and other packets without a position
#define MOVING_REPEAT_UNREADABLE  1         // LACF frames with a model this build does not hold
#define ALWAYS_REPEAT_TOCALLS     ""        // e.g. "APLAIX,APLAB" (prefixes, comma-separated)
#define NEIGHBOR_SLOTS            48        // stations remembered (about 60 bytes each)
#define DEDUPE_SECONDS      30              // the same packet heard again within this time is not repeated again
#define DEDUPE_SLOTS        32

// ---- LACF 3.0 WITH THE MODEL (this branch carries the PPM tables) ----
// LACF 3.0 frames are always decoded in full and checked before they are relayed; their compressed content is then copied byte for byte.
// TX_FORMAT_LACF3 1: plain text packets (and the beacon) are re-encoded as LACF 3.0 before they go out, saving about a third of the airtime.
// Only LACF-aware stations can read those frames (stock LoRa APRS firmware cannot), so leave it at 0 unless every receiver in range
// that matters understands LACF 3.0.
#define TX_FORMAT_LACF3     0
#define LOG_DECODED         1               // print each decoded LACF 3.0 packet on the serial port

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
#define DEBUG_SERIAL        1           // 1: per-packet log (silence at run time with "set log off"); 0: remove it from the program. The command line is always on.
#define DEBUG_BAUD          115200
#define STATS_EVERY_MIN     10              // a counters line on the serial port
#define WATCHDOG_S          20              // restart if the loop stops for this long (a remote site has nobody to press reset)
