// ============================================================
// Horus Binary 4FSK test: can the STM32WLE5 send clean 4FSK if the
// MCU keeps the TCXO powered, so the radio never restarts it?
//
// Built only by the horus_test_* environments in platformio.ini;
// the normal tracker build ignores this file.
//
//   horus_test_old   the known-bad method: radio controls the TCXO,
//                    normal 5 ms startup delay (the comparison)
//   horus_test_hold  the experiment: RCC_CR.HSEBYPPWR + HSEON keep the
//                    TCXO supply on from the MCU side, and the radio's
//                    TCXO startup delay is set to 0
//
// Both loop the same fixed Horus Binary v2 packet (payload ID 256,
// "4FSKTEST-V2"), encoded with horusdemodlib's own horus_l2 encoder
// (Golay, interleaver, scrambler) and checked to decode with horus_demod.
//
// Serial (115200) prints, per packet, whether HSE32 is running and how
// long each tone change (transmitDirect) blocked. If the radio still waits
// for the TCXO, that often shows up here as ~5000 us per tone; the RF
// recording is the real verdict either way.
// ============================================================
#include <Arduino.h>
#include <RadioLib.h>
#include "config.h"
#if USE_TIMED_PKTS
#include "horus_packets.h"
#endif

#ifndef HORUS_FREQ_MHZ
#define HORUS_FREQ_MHZ   431.051      // base (lowest) tone
#endif
#ifndef HORUS_POWER_DBM
#define HORUS_POWER_DBM  -9           // HP PA minimum; 10 dBm overloaded the RSPduo on the bench
#endif
// micros() runs ~1.9% slow on this MCU: symbols measured 10.195 ms at 10000 us
// (IQ recording, 2026-11-21). 10000 / 1.0195 = 9809 us gives a true 100 baud.
#ifndef SYSCLK_HSE
#define SYSCLK_HSE 0   // 1 = run the MCU from HSE32 (the TCXO) instead of MSI, so micros() is accurate
#endif
#ifndef SYMBOL_US
#if SYSCLK_HSE
#define SYMBOL_US 10000
#else
#define SYMBOL_US 9809
#endif
#endif
#ifndef USE_TIMED_PKTS
#define USE_TIMED_PKTS 0   // 1 = cycle through pre-built, UTC-stamped packets (src/horus_packets.h)
#endif
#ifndef USE_FSK4
#define USE_FSK4 0     // 1 = send with RadioLib FSK4Client instead of the hand-rolled tone loop
#endif
#ifndef TCXO_HOLD
#define TCXO_HOLD        0
#endif
#ifndef TCXO_DELAY_US
#define TCXO_DELAY_US    5000
#endif
#define HORUS_BAUD       100
#define TONE_SPACING_HZ  270.0
#define PACKET_GAP_MS    1000

// Horus v2, 32-byte payload: ID 930 (K6ATV), counter 1, 12:34:56,
// 37.0058 / -121.5683, 100 m, 8 sats. 65 bytes after horus_l2 encoding.
static const uint8_t HORUS_PKT[65] = {
    0x24,0x24,0x54,0x6D,0x78,0x24,0x6C,0x5F,0xA7,0x1D,0xCD,0x34,0xE9,0xEF,0xCF,0xF4,
    0xD1,0x6B,0x7F,0x59,0x21,0x09,0xD2,0x0D,0xCF,0xC7,0xF2,0x1A,0x21,0x19,0x61,0x7B,
    0x96,0x63,0x3E,0x71,0x09,0x8C,0x76,0x4C,0xF6,0xD0,0x18,0x4A,0x80,0xBA,0x2E,0x93,
    0x54,0x0D,0x48,0x5F,0x20,0x98,0xA5,0xEE,0xD7,0x7A,0x08,0xA9,0x11,0x9D,0x4A,0x72,
    0x55
};
static const uint8_t PREAMBLE_BYTE = 0x1B;   // tones 0,1,2,3
static const int     PREAMBLE_LEN  = 16;

STM32WLx radio = new STM32WLx_Module();
#if USE_FSK4
FSK4Client fsk4(&radio);
#endif
static const uint32_t rfswitch_pins[] = RFSWITCH_PINS;
// HP-only table: the E77-400M22S antenna is fed by the high-power PA, so
// without a TX_LP entry RadioLib uses the HP PA even at 10 dBm.
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},
    {STM32WLx::MODE_RX,    {LOW,  HIGH, LOW}},
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}},
    END_OF_MODE_TABLE };

static uint32_t baseRaw, shiftRaw;
static uint32_t nextSymbol;
static uint32_t tdSum, tdMax, tdCount;

static void halt(const char* what, int code) {
    Serial.print(what); Serial.print(" failed, code "); Serial.println(code);
    while (true) delay(100);
}

static void tcxoHoldOn() {
    // Ask the clock controller for HSE32 with the TCXO supply (PB0-VDDTCXO)
    // driven from the MCU side. HSEBYPPWR must be set before HSEON.
    RCC->CR |= RCC_CR_HSEBYPPWR;
    RCC->CR |= RCC_CR_HSEON;
    uint32_t t0 = millis();
    while (!(RCC->CR & RCC_CR_HSERDY) && millis() - t0 < 100) {}
    Serial.print("[TCXO] MCU hold on, HSE32 ");
    Serial.println((RCC->CR & RCC_CR_HSERDY) ? "ready" : "NOT ready after 100 ms");
}

#if SYSCLK_HSE
// MSI (the core's default, 48 MHz) is an on-chip RC oscillator and runs ~1.9% slow
// here. Switch SYSCLK to HSE32 (TCXO) and re-scale SysTick so micros() is accurate.
static void sysclkToHse() {
    tcxoHoldOn();                                   // HSEBYPPWR + HSEON, wait for HSERDY
    if (!(RCC->CR & RCC_CR_HSERDY)) return;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_1;      // SW = 0b10: HSE32
    uint32_t t0 = millis();
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_1 && millis() - t0 < 100) {}
    SystemCoreClock = 32000000UL;
    SysTick->LOAD = SystemCoreClock / 1000UL - 1UL;
    SysTick->VAL = 0;
}
#endif

static void tone(uint8_t s) {
    uint32_t t0 = micros();
    radio.transmitDirect(baseRaw + s * shiftRaw);
    uint32_t dt = micros() - t0;
    tdSum += dt; tdCount++; if (dt > tdMax) tdMax = dt;
    nextSymbol += SYMBOL_US;
    while ((int32_t)(micros() - nextSymbol) < 0) {}
}

static void sendBytes(const uint8_t* d, int n) {
    for (int i = 0; i < n; i++)
        for (int k = 0; k < 4; k++)
            tone((d[i] >> (6 - 2 * k)) & 0x03);   // MSB first, as RadioLib FSK4
}

void setup() {
    Serial.begin(DEBUG_BAUD);
    delay(2000);
    Serial.println();
    Serial.print("Horus TCXO test, method: ");
    Serial.println(TCXO_HOLD ? "MCU holds TCXO on" : "old (radio controls TCXO)");

    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    int st = radio.beginFSK(HORUS_FREQ_MHZ);
    if (st != RADIOLIB_ERR_NONE) halt("beginFSK", st);
    st = radio.setTCXO(TCXO_VOLTAGE, TCXO_DELAY_US);
    if (st != RADIOLIB_ERR_NONE) halt("setTCXO", st);
    st = radio.setOutputPower(HORUS_POWER_DBM);
    if (st != RADIOLIB_ERR_NONE) halt("setOutputPower", st);

    if (TCXO_HOLD) tcxoHoldOn();
#if SYSCLK_HSE
    sysclkToHse();
#endif

#if USE_FSK4
    st = fsk4.begin(HORUS_FREQ_MHZ, (uint32_t)(TONE_SPACING_HZ + 0.5f), HORUS_BAUD);
    if (st != RADIOLIB_ERR_NONE) halt("fsk4.begin", st);
#endif
    float step = radio.freqStep;          // synthesizer step in Hz (public in RadioLib 7.x)
    baseRaw  = (uint32_t)((HORUS_FREQ_MHZ * 1e6) / step);
    shiftRaw = (uint32_t)(TONE_SPACING_HZ / step + 0.5f);
    Serial.print("[Radio] "); Serial.print(HORUS_FREQ_MHZ, 3); Serial.print(" MHz, ");
    Serial.print(HORUS_POWER_DBM); Serial.print(" dBm, tone step ");
    Serial.print(shiftRaw * step, 1); Serial.print(" Hz, TCXO delay ");
    Serial.print(TCXO_DELAY_US); Serial.println(" us");
}

void loop() {
    static uint32_t n = 0;
#if USE_TIMED_PKTS
    if (n >= (uint32_t)HORUS_PKT_COUNT) {   // timestamps would run past the receiver's time window: stop
        radio.standby();
        while (true) delay(1000);
    }
    const uint8_t* curPkt = HORUS_PKTS[n];
#else
    const uint8_t* curPkt = HORUS_PKT;
#endif
    tdSum = tdMax = tdCount = 0;
    nextSymbol = micros();
#if USE_FSK4
    static uint8_t frame[PREAMBLE_LEN + sizeof(HORUS_PKT)];
    memset(frame, PREAMBLE_BYTE, PREAMBLE_LEN);
    memcpy(frame + PREAMBLE_LEN, curPkt, sizeof(HORUS_PKT));
    fsk4.write(frame, sizeof(frame));      // one call: preamble + packet, standby at the end
    tdCount = 1;                           // (per-tone timing not measured on this path)
#else
    for (int i = 0; i < PREAMBLE_LEN; i++) sendBytes(&PREAMBLE_BYTE, 1);
    sendBytes(curPkt, sizeof(HORUS_PKT));
#endif
    radio.standby();

    Serial.print("pkt "); Serial.print(++n);
    Serial.print("  HSE32 "); Serial.print((RCC->CR & RCC_CR_HSERDY) ? "on" : "off");
    Serial.print("  tone change avg "); Serial.print(tdSum / tdCount);
    Serial.print(" us, max "); Serial.print(tdMax); Serial.println(" us");
    delay(PACKET_GAP_MS);
}
