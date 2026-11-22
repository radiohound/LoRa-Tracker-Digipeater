#pragma once
// Glue between the settings engine (tk_settings.c) and the tracker firmware: the saved record in flash,
// the serial command line, and the settings the rest of the code reads (tkCfg).
#include <stdint.h>
#include "tk_settings.h"

extern tk_settings_t tkCfg;             // the live settings; config.h values are the defaults
extern volatile bool tkBeaconNow;       // set by the "beacon" command: the main loop runs a beacon cycle now
extern volatile bool tkHorusNow;        // set by the "horus" command: that cycle also sends a Horus packet (ignoring the divider)

void tkKeysAtBoot();                    // call first thing in setup(): Key 1 held = jump to the bootloader, user key = defaults
void tkSettingsBegin();                 // open the serial port, load the record from flash, print the boot lines
void tkBootWindow(uint32_t ms);         // wait a moment for a key; if one comes, stay in setup until "exit" or "reboot"
void tkSerialPoll();                    // non-blocking: read typed characters, run finished command lines
void tkPushHorus();                     // copy the Horus settings into the Horus transmitter (does nothing without Horus)

// Implemented in main.cpp:
void tkApplyRadio();                    // re-initialise the LoRa radio from tkCfg
void tkOnSettingsChanged();             // refresh what depends on tkCfg (the callsign string)
