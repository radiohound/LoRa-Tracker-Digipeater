# STM32WLE5 Low-Power LoRa APRS — Tracker + Digipeater

Built around the **Ebyte E77-400MBL-01** (STM32WLE5CC, 433 MHz) with support
for the ST Nucleo-WL55JC and other STM32WLE5-based modules.

Libraries: **RadioLib** (jgromes) + **APRSPacketLib** (richonguzman).

---

## File overview

```
platformio.ini        — build/upload config and library deps
include/config.h      — ALL user settings (callsign, mode, pins…)
src/main.cpp          — firmware
```

---

## Quick start

1. Install [PlatformIO](https://platformio.org/).
2. Open `include/config.h` and set:
   - `OPERATING_MODE` (tracker / tracker+digi / digi-only)
   - `MY_CALLSIGN`, `MY_SSID`, `MY_COMMENT`
   - `LORA_FREQ` for your region
   - The correct `RFSWITCH_PINS` / `RFSWITCH_TABLE` block for your board
3. `pio run --target upload`

---

## Operating modes

| Mode | GPS needed | Radio state | MCU sleep |
|------|-----------|-------------|-----------|
| `TRACKER_ONLY` | Yes | TX then sleep | Stop2 between beacons (~2 µA) |
| `TRACKER_DIGI` | Yes | TX then continuous RX | Light idle (~5 mA) |
| `DIGI_ONLY` | **No** | Continuous RX | Light idle (~5 mA) |

---

## RF switch pin configurations

The STM32WLE5's internal radio is connected to an external RF switch.
Pinout varies by module — uncomment the right block in `config.h`.

| Module | Pins | Notes |
|--------|------|-------|
| Ebyte E77-400MBL-01 | PA4, PA5 | HP path only |
| Seeed LoRa-E5 / E5-mini | PA4, PA5 | HP path only |
| ST Nucleo-WL55JC | PC3, PC4, PC5 | Both LP and HP |
| RAK3172-E (no TCXO) | PB8, PC13 | LP path only |

> **Important:** Call `radio.setRfSwitchTable()` *before* `radio.begin()`.

---

## Power budget (approximate, Ebyte E77 @ 3.3 V)

| State | Current |
|-------|---------|
| Stop2 (MCU only) | ~2 µA |
| Idle / light sleep | ~1.5 mA |
| LoRa RX | ~5 mA |
| GPS acquiring fix | ~20–30 mA |
| LoRa TX 22 dBm | ~100 mA peak |

For a 120 s beacon interval in `TRACKER_ONLY` mode with a 30 s GPS warm
start, average current is roughly 6–8 mA (dominated by GPS acquisition).
With a pre-warmed GPS or a fast-fix module it drops below 2 mA average.

---

## Digipeater behaviour

- Listens for LoRa-APRS packets continuously.
- Checks for an **unactivated** `WIDE1-1` or `WIDE2-N` path field.
- Inserts `CALLSIGN-SSID*` and decrements the hop count.
- Applies a random delay (default 80–450 ms) before retransmitting to
  reduce collision probability with other digipeaters.
- Duplicate suppression: an 8-element CRC32 ring buffer discards packets
  heard within the same burst.
- Does **not** digipeat its own beacon packets.

---

## Adjusting the beacon rate

Change `BEACON_INTERVAL_S` in `config.h`.  Recommended minimums:

- Fixed station: 600 s (10 min)
- Slow vehicle: 120 s (2 min)
- Fast vehicle / balloon: 30–60 s

---

## Known caveats

- **APRSPacketLib field names**: the library evolves quickly. If
  `APRSPacket` struct fields differ from what's in `main.cpp`, check
  the library header and adjust the `buildBeaconPacket()` function.
- **Stop2 and Serial**: serial output is flushed before entering Stop2.
  After wakeup the USART restarts automatically under the STM32 Arduino
  core but a short `delay(10)` may be needed on some core versions.
- **GPS power switching**: the GPS_POWER_PIN logic assumes a high-side
  P-FET (HIGH = on). Invert if you use an N-FET or load switch with
  active-low enable.
