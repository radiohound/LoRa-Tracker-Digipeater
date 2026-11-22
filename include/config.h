#pragma once

// ============================================================
// STM32WLE5 LoRa APRS Tracker + Digipeater
// Target: Ebyte E77-400MBL-01 / E77-900MBL-01 (STM32WLE5CC)
//
// RF switch / GPS wiring verified against PicoTrack (K6ATV):
//   https://github.com/radiohound/PicoTrack
//   PA9 = I2C SCL, PA10 = I2C SDA
// ============================================================

// ============================================================
// DEBUG
// ============================================================
#define DEBUG_SERIAL        1
#define DEBUG_BAUD          115200

// ============================================================
// OPERATING MODE
//   1 = TRACKER_ONLY      TX beacon, sleep, repeat
//   2 = TRACKER_DIGI      TX beacon, then RX/digipeat until next interval
//   3 = DIGI_ONLY         RX/digipeat continuously, no GPS/beacon
//   4 = DIGI_CAD_SOLAR    Two-stage CAD, digipeats all stations
//   5 = DIGI_CAD_BATTERY  Two-stage CAD, moving stations only
// ============================================================
#define MODE_TRACKER_ONLY      1
#define MODE_TRACKER_DIGI      2
#define MODE_DIGI_ONLY         3
#define MODE_DIGI_CAD_SOLAR    4
#define MODE_DIGI_CAD_BATTERY  5
#define MODE_DIGI_CAD          MODE_DIGI_CAD_SOLAR  // backwards-compatible alias

#define OPERATING_MODE  2

// ============================================================
// IDENTITY
// ============================================================
#define MY_CALLSIGN         "W2HAB"
#define MY_SSID             11
#define MY_COMMENT          " STM32WLE5 LoRa"

// ============================================================
// APRS
// ============================================================
#define APRS_DESTINATION    "APLAB1"           // registered tocall for this project
#define BEACON_PATH         "WIDE1-1"
#define APRS_SYMBOL_TABLE   '/'
#define APRS_SYMBOL_CODE    'O'             // 'O' = balloon

// ============================================================
// LORA (APRS) 433.775, BW 125 kHz, SF 12, CR 4/5, sync word 0x12 are standard for LoRa APRS in North America & Europe.
// ============================================================
#define APRS_ENABLED        1
#define LORA_FREQ           433.775         // MHz — EU LoRa APRS
#define LORA_BW             125.0           // kHz
#define LORA_SF             12              // spreading factor (6-12, higher = longer range but slower)  
#define LORA_CR             5               // coding rate (5-8, higher = more robust but slower)
#define LORA_TX_POWER       22              // dBm
#define LORA_OCP_MA         140             // over-current protection (mA)
#define LORA_SYNC_WORD      0x12            // LoRa APRS sync word
#define LORA_PREAMBLE       8

// ============================================================
// TCXO
// E77 modules with SN >= 3202995 use a TCXO (1.8 V).
// Older crystal modules: remove setTCXO() in radioInit() and
// use firmware labelled "-xtal".
// ============================================================
#define TCXO_VOLTAGE        1.7f            // volts

// ============================================================
// BEACON
// ============================================================
#define BEACON_INTERVAL_S   5              // seconds between beacons

// ============================================================
// GPS
// SparkFun u-blox library over I2C.
// PA9 = SCL, PA10 = SDA (E77 MBL dev board standard wiring).
// ============================================================
#define GPS_I2C_CLOCK       400000          // 400 kHz fast mode
#ifndef GPS_FIX_TIMEOUT_MS
#define GPS_FIX_TIMEOUT_MS  120000          // 2 min max wait for fix
#endif
#define GPS_POWER_PIN       RADIOLIB_NC     // set to a pin if GPS has power switch

// ============================================================
// BMP280 (OPTIONAL)
// Set to 1 if a BMP280 is on the I2C bus.
// If enabled but absent, bmpReady stays false and the code
// silently falls back to GPS altitude — no other change needed.
//
// Standard I2C addresses:
//   0x76 — SDO pulled LOW (most breakouts)
//   0x77 — SDO pulled HIGH
// ============================================================
#define BMP280_ENABLED      1
#define BMP280_I2C_ADDR     0x76
#define BMP280_CAL_SAMPLES  5               // GPS samples averaged for pressure cal

// ============================================================
// CUTDOWN (OPTIONAL)
// ============================================================
#define CUTDOWN_ENABLED     0
#define CUTDOWN_PIN         PA0
#define CUTDOWN_ALTITUDE_M  30000
#define CUTDOWN_ARM_ASCENT_M 500
#define CUTDOWN_CONFIRM_COUNT 3
#define CUTDOWN_PULSE_MS    5000

// ============================================================
// DIGIPEATER
// ============================================================
#define DIGI_PATH_1         "WIDE1-1"
#define DIGI_PATH_2         "WIDE2-1"
#define DIGI_QUEUE_SIZE     4
#define DIGI_DELAY_MIN_MS   80
#define DIGI_DELAY_MAX_MS   450

// ============================================================
// CAD DIGIPEATER
// Two-stage design:
//   Stage 1 — CAD sentinel: 65ms scans every CAD_SCAN_INTERVAL_MS, continuously.
//             Guaranteed to catch any transmission >= CAD_SCAN_INTERVAL_MS long.
//   Stage 2 — Active RX: entered on CAD trigger. Listens for up to
//             the mode's RX window. Timer resets on each qualifying packet.
//             Returns to Stage 1 after silence.
// ============================================================
#define CAD_SCAN_INTERVAL_MS        1000    // ms between CAD scans — must be <= transmission length

// Solar mode (MODE_DIGI_CAD_SOLAR): power is not a concern.
// Any station heard resets the window.
#define CAD_RX_WINDOW_SOLAR_MS      360000  // 6 minutes

// Battery mode (MODE_DIGI_CAD_BATTERY):
// Set CAD_BEACON_INTERVAL_MS to the expected beacon interval of the
// moving stations you want to capture. 15 seconds is automatically
// added as a buffer to handle timing variation.
// Only moving/new stations reset the timer — fixed ground stations
// are ignored. Pick one:
//
//#define CAD_BEACON_INTERVAL_MS    360000  // 6 min  — infrequent tracker
#define CAD_BEACON_INTERVAL_MS      180000  // 3 min  — typical vehicle or balloon (default)
//#define CAD_BEACON_INTERVAL_MS     60000  // 1 min  — fast vehicle
//#define CAD_BEACON_INTERVAL_MS     30000  // 30 sec — high-rate balloon or tracker
//
#define CAD_RX_WINDOW_BATTERY_MS    (CAD_BEACON_INTERVAL_MS + 15000UL)  // adds 15 s buffer

// ============================================================
// STATION TRACKING (MODE_DIGI_CAD_BATTERY only)
// Tracks last known position per callsign. Packets from stations
// that have not moved are not digipeated and do not open the RX
// window, saving power at sites with fixed ground stations.
// ============================================================
#define STATION_TRACK_COUNT   20    // max number of stations to remember
#define STATION_MOVE_THRESH_M 100   // meters of movement to consider a station "moved"

// Return values for classifyAndUpdate()
#define STATION_NEW           0     // never heard before
#define STATION_MOVED         1     // heard before, has moved >= STATION_MOVE_THRESH_M
#define STATION_STATIONARY    2     // heard before, has not moved

// ============================================================
// RF SWITCH — E77 MBL board
// Verified against PicoTrack / mLRS E77 MBL documentation.
// Update to match your carrier board schematic if different.
// ============================================================
#define RFSWITCH_PINS   {PA6, PA7, PB3, RADIOLIB_NC, RADIOLIB_NC}
// The E77-400M22S antenna is fed from the high-power (HP) PA. With a TX_LP entry in the table,
// RadioLib uses the low-power PA for 14 dBm and below, which is NOT connected to the antenna
// on this module (very weak signal). Set RF_HP_PA_ONLY 1 to drop the LP entry so every
// power level uses the HP PA (range then -9..22 dBm). Default 0 keeps the previous behaviour.
#ifndef RF_HP_PA_ONLY
#define RF_HP_PA_ONLY   0
#endif
#if RF_HP_PA_ONLY
#define RFSWITCH_TABLE  \
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},  \
    {STM32WLx::MODE_RX,    {LOW,  HIGH, LOW}},  \
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}}, \
    END_OF_MODE_TABLE
#else
#define RFSWITCH_TABLE  \
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},  \
    {STM32WLx::MODE_RX,    {LOW,  HIGH, LOW}},  \
    {STM32WLx::MODE_TX_LP, {HIGH, LOW,  HIGH}}, \
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}}, \
    END_OF_MODE_TABLE
#endif

// ============================================================
// HORUS BINARY v2 (4FSK, 100 baud) — OPTIONAL
// Sends a Horus v2 packet from the GPS fix after each APRS beacon (tracker modes only).
// Needs the system clock on the TCXO (HSE_SYSCLK, switched on automatically below), because
// the core's default clock runs ~1.9% slow and the symbols would go out at ~98 baud.
// Horus v2 needs a payload ID: ask for one at
// https://github.com/projecthorus/horusdemodlib (payload_id_list.txt). 256 is the shared test ID.
// Third-party LGPL code lives in src/horus/ (see src/horus/NOTICE.md).
// ============================================================
#ifndef HORUS_ENABLED
#define HORUS_ENABLED          0
#endif
#ifndef HORUS_PAYLOAD_ID
#define HORUS_PAYLOAD_ID       256              // 256..65535; request your own
#endif
#ifndef HORUS_FREQ_MHZ
#define HORUS_FREQ_MHZ         431.051          // lowest tone; nominal 431.050 MHz (local Horus frequency) + 1 kHz
                                                // 434.200 nominal (434.201 here) is the usual frequency elsewhere
#endif
#ifndef HORUS_TONE_SPACING_HZ
#define HORUS_TONE_SPACING_HZ  270.0
#endif
#ifndef HORUS_BAUD
#define HORUS_BAUD             100
#endif
#ifndef HORUS_TX_POWER_DBM
#define HORUS_TX_POWER_DBM     20               // above 14 uses the HP PA; -9..22 needs RF_HP_PA_ONLY 1
#endif
// Time slots (same scheme as RS41ng): transmit only when the GPS time, counted in seconds from
// the top of the hour, satisfies  (t - OFFSET) mod SYNC == 0.  SYNC 0 = off (send right after the beacon).
// Payloads sharing a frequency must use the same SYNC and different OFFSETs, with the cycle long
// enough for every packet (a Horus v2 packet takes about 3.3 s). Examples, two slots per cycle:
//   10 s cycle: slot 1 = SYNC 10, OFFSET 0   slot 2 = SYNC 10, OFFSET 5   (an RS41ng sonde on a 10 s cycle)
//   30 s cycle: slot 1 = SYNC 30, OFFSET 0   slot 2 = SYNC 30, OFFSET 15
#ifndef HORUS_TIME_SYNC_SECONDS
#define HORUS_TIME_SYNC_SECONDS         0
#endif
#ifndef HORUS_TIME_SYNC_OFFSET_SECONDS
#define HORUS_TIME_SYNC_OFFSET_SECONDS  0
#endif
#ifndef HORUS_BEACON_DIVIDER
#define HORUS_BEACON_DIVIDER   1                // send Horus on every Nth beacon
#endif

#ifndef HSE_SYSCLK
#define HSE_SYSCLK             HORUS_ENABLED    // 1 = run the MCU from the TCXO instead of MSI
#endif
