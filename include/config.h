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
// OPERATING MODE — pick exactly one
// ============================================================
#define MODE_TRACKER_ONLY   1   // TX beacon, sleep, repeat
#define MODE_TRACKER_DIGI   2   // TX beacon, then RX/digipeat until next interval
#define MODE_DIGI_ONLY      3   // RX/digipeat continuously, no GPS/beacon
#define MODE_DIGI_CAD       4   // Low-power CAD digipeater

#define OPERATING_MODE      MODE_TRACKER_ONLY

// ============================================================
// IDENTITY
// ============================================================
#define MY_CALLSIGN         "K6ATV"
#define MY_SSID             8
#define MY_COMMENT          " STM32WLE5 LoRa"

// ============================================================
// APRS
// ============================================================
#define APRS_DESTINATION    "APZP01"
#define BEACON_PATH         "WIDE1-1"
#define APRS_SYMBOL_TABLE   '/'
#define APRS_SYMBOL_CODE    'O'             // 'O' = balloon

// ============================================================
// LORA (APRS)
// ============================================================
#define APRS_ENABLED        1
#define LORA_FREQ           433.775         // MHz — EU LoRa APRS
#define LORA_BW             125.0           // kHz
#define LORA_SF             12
#define LORA_CR             5
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
#define GPS_FIX_TIMEOUT_MS  120000          // 2 min max wait for fix
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
// ============================================================
#define CAD_BATCH_SIZE      15
#define CAD_SCAN_INTERVAL_MS 2000
#define CAD_BATCH_SLEEP_MS  30000

// ============================================================
// RF SWITCH — E77 MBL board
// Verified against PicoTrack / mLRS E77 MBL documentation.
// Update to match your carrier board schematic if different.
// ============================================================
#define RFSWITCH_PINS   {PA6, PA7, PB3, RADIOLIB_NC, RADIOLIB_NC}
#define RFSWITCH_TABLE  \
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},  \
    {STM32WLx::MODE_RX,    {LOW,  HIGH, LOW}},  \
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}}, \
    END_OF_MODE_TABLE
