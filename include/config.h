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
//   DIGI_CAD      – CAD-based digipeater (~0.1 mA, misses first
//                   packet of each burst but catches all others)
// ------------------------------------------------------------
#define MODE_TRACKER_ONLY  0
#define MODE_TRACKER_DIGI  1
#define MODE_DIGI_ONLY     2
#define MODE_DIGI_CAD      3

#define OPERATING_MODE     MODE_DIGI_CAD   // <-- change this

