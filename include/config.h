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
#define MODE_SF_TEST       4

#define OPERATING_MODE     MODE_SF_TEST   // <-- change this

// ------------------------------------------------------------
//  STATION IDENTIFICATION
// ------------------------------------------------------------
#define MY_CALLSIGN        "K6ATV"
#define MY_SSID            6
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
//  SF TEST MODE  (MODE_SF_TEST only)
//
//  Experimental: transmits a beacon every SF_TEST_INTERVAL_MS,
//  alternating between LORA_SF_TEST_A and LORA_SF_TEST_B so two
//  back-to-back packets on the same SF arrive ~2× the interval
//  apart. The comment is replaced with "#<counter> HH:MM:SS"
//  (UTC from GPS) so receiver logs can be matched up by counter
//  and timestamp regardless of which SF reached them.
//
//  Same TX power for both SFs (LORA_TX_POWER) so SF is the only
//  variable in the A/B comparison. Radio and GPS are kept on
//  continuously — power draw is high; this mode is for bench /
//  drive testing, not flight.
//
//  Air time at BW=125 kHz, CR=4/5, ~70-byte payload:
//    SF7  ≈ 110 ms,   SF12 ≈ 2.5 s
//  At a 5 s interval, the SF12 slot has only ~2 s of margin.
// ------------------------------------------------------------
#define SF_TEST_INTERVAL_MS  5000UL
#define LORA_SF_TEST_A       7        // sent first, on odd counters
#define LORA_CR_TEST_A       5        //   paired CR (4/5) for SF7
#define LORA_SF_TEST_B       12       // sent second, on even counters
#define LORA_CR_TEST_B       6        //   paired CR (4/6) for SF12

// ------------------------------------------------------------
//  CATS  (Communication And Telemetry System) — dual-protocol
//
//  When CATS_ENABLED, the tracker modes (TRACKER_ONLY /
//  TRACKER_DIGI) send an extra position beacon on the CATS
//  network after each APRS beacon. CATS is raw 2-FSK at 9600
//  bit/s on 70 cm with a software FEC stack (CRC-16 + whitening
//  + LDPC + interleaving) handled by the vendored libCATS. The
//  packet carries Identification + GPS + Comment whiskers.
//
//  PHY values are from the CATS standard
//  (https://gitlab.scd31.com/cats/cats-standard — rf.tex/header.tex):
//    Carrier 430.500 MHz, 2-FSK, 9600 bit/s, deviation 4.8 kHz.
//    On-air frame: preamble 0x55 x4, sync 0xAB 0xCD 0xEF 0x12,
//    2-byte little-endian payload length, then libCATS-encoded
//    data. The radio supplies the preamble + sync word; libCATS
//    output is payload only. (Framing constants live in main.cpp.)
//
//  The radio is the SAME STM32WL chip used for LoRa APRS — it is
//  reconfigured to FSK for the CATS TX and back to LoRa after.
//  Costs one extra FSK packet (~tens of ms at 9600 bps) plus a
//  radio re-init per beacon cycle. Set CATS_ENABLED to 0 to
//  exclude all CATS code and the libCATS dependency entirely.
// ------------------------------------------------------------
#define CATS_ENABLED       1            // 1 = also TX CATS in tracker modes
#define CATS_FREQ          430.500      // MHz, 70cm CATS calling frequency
#define CATS_BITRATE       9.6          // kbps (9600 bit/s) — RadioLib units
#define CATS_FREQ_DEV      4.8          // kHz frequency deviation
#define CATS_TX_POWER      LORA_TX_POWER // dBm (same PA as APRS)
#define CATS_SSID          MY_SSID      // CATS SSID (reuses station SSID)
#define CATS_ICON          0            // CATS map icon id (0 = default)
// Callsign and comment reuse MY_CALLSIGN / MY_COMMENT.

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
//  CUTDOWN  (tracker modes only — TRACKER_ONLY / TRACKER_DIGI)
//
//  When the confirmed altitude reaches CUTDOWN_ALTITUDE_M, the
//  cutdown pin pulses HIGH for CUTDOWN_PULSE_MS milliseconds then
//  returns LOW. This fires once only — the system will not trigger
//  again even if altitude readings remain high.
//
//  Safeguards:
//  - CUTDOWN_ARM_ASCENT_M: the balloon must rise this many metres
//    above its launch altitude before the system arms. Prevents
//    triggering at a high-altitude launch site or from a ground-
//    level noise spike.
//  - CUTDOWN_CONFIRM_COUNT: this many consecutive altitude readings
//    must all be above CUTDOWN_ALTITUDE_M before the pin fires.
//    The counter resets if any reading falls back below the threshold.
//  - CUTDOWN_PULSE_MS: pin is held HIGH for exactly this duration
//    (blocking), then driven LOW before any further code runs.
//
//  Set CUTDOWN_ENABLED to 0 to exclude all cutdown code entirely.
// ------------------------------------------------------------
#define CUTDOWN_ENABLED        0          // 1 = enable, 0 = disable
#define CUTDOWN_PIN            PA0        // GPIO pin to pulse at cutdown
#define CUTDOWN_ALTITUDE_M     30000      // trigger altitude in metres MSL
#define CUTDOWN_ARM_ASCENT_M   500        // metres above launch alt before armed
#define CUTDOWN_CONFIRM_COUNT  5          // consecutive readings required
#define CUTDOWN_PULSE_MS       5000       // pin HIGH duration in milliseconds

// ------------------------------------------------------------
//  DEBUG
// ------------------------------------------------------------
#define DEBUG_SERIAL       1
#define DEBUG_BAUD         115200
