// RF chain check: transmits right after reset, then every 5 s. No GPS needed.
// Per cycle: one LoRa APRS-style packet (same init as the tracker), then
// a 2 s unmodulated carrier on the same frequency, easy to spot on an SDR.
#include <Arduino.h>
#include <RadioLib.h>
#include "config.h"

STM32WLx radio = new STM32WLx_Module();
static const uint32_t rfswitch_pins[] = RFSWITCH_PINS;
// HP-only table: the E77-400M22S antenna is fed by the high-power PA, so
// without a TX_LP entry RadioLib uses the HP PA even at 10 dBm.
static const Module::RfSwitchMode_t rfswitch_table[] = {
    {STM32WLx::MODE_IDLE,  {LOW,  LOW,  LOW}},
    {STM32WLx::MODE_RX,    {LOW,  HIGH, LOW}},
    {STM32WLx::MODE_TX_HP, {HIGH, LOW,  HIGH}},
    END_OF_MODE_TABLE };

void setup() {
    Serial.begin(DEBUG_BAUD);
    radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
    int st = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR);
    if (st == RADIOLIB_ERR_NONE) st = radio.setTCXO(TCXO_VOLTAGE);
    if (st == RADIOLIB_ERR_NONE) st = radio.setOutputPower(10);
    if (st == RADIOLIB_ERR_NONE) radio.setSyncWord(LORA_SYNC_WORD);
    Serial.print("rf_check init: "); Serial.println(st);
}

void loop() {
    int a = radio.transmit("<\xff\x01" "RFCHECK>APLAB1:>rf check");
    int b = radio.transmitDirect();          // carrier at the set frequency
    delay(2000);
    radio.standby();
    Serial.print("tx lora="); Serial.print(a);
    Serial.print(" cw="); Serial.println(b);
    delay(5000);
}
