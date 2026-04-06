# STM32WLE5 Low-Power LoRa APRS — Tracker + Digipeater

Built around the **Ebyte E77-400MBL-01** (STM32WLE5CC, 433 MHz) with support
for the ST Nucleo-WL55JC and other STM32WLE5-based modules.

Libraries: **RadioLib** (jgromes) + **APRSPacketLib** (richonguzman).

Verified against the working [PicoTrack](https://github.com/radiohound/PicoTrack)
firmware by K6ATV for RF switch pinout, TCXO voltage, and radio init sequence.

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
   - `OPERATING_MODE` — choose your mode (see table below)
   - `MY_CALLSIGN`, `MY_SSID`, `MY_COMMENT`
   - `LORA_FREQ` for your region
   - The correct `RFSWITCH_PINS` / `RFSWITCH_TABLE` block for your board
3. `pio run --target upload`

---

## Operating modes

| Mode | GPS needed | Radio state | Avg current |
|------|-----------|-------------|-------------|
| `TRACKER_ONLY` | Yes | TX then Stop2 sleep | ~2–8 mA (GPS-dominated) |
| `TRACKER_DIGI` | Yes | TX then continuous RX | ~5–6 mA |
| `DIGI_ONLY` | No | Continuous RX always | ~6 mA |
| `DIGI_CAD` | No | CAD scan + deep sleep | **~0.04–0.1 mA** |

`DIGI_CAD` is the recommended mode for battery or solar operation. It uses
Channel Activity Detection to wake only when a LoRa preamble is present on
the channel, missing the triggering packet but catching all subsequent ones.
In a typical APRS environment this is fully acceptable digipeater behaviour.

---

## Power budget (Ebyte E77 @ 3.3 V)

| State | Current |
|-------|---------|
| Stop2 — MCU only, radio off | ~2 µA |
| MCU idle / light sleep | ~1.5 mA |
| LoRa RX (continuous) | ~5 mA |
| CAD scan (~65 ms at SF12) | ~1.5 mA |
| GPS acquiring fix | ~20–30 mA |
| LoRa TX at 22 dBm (peak) | ~100 mA |

### Mode-by-mode averages

**TRACKER_ONLY** (120 s interval, 30 s GPS warm start):
Average ~2–3 mA, dominated by GPS. With GPS power gating and a fast-fix
module this can drop below 1 mA average.

**DIGI_ONLY** (continuous RX):
Flat ~6 mA regardless of traffic. Suitable for mains or large solar.

**DIGI_CAD** (15-scan batch, 2 s scan interval, 30 s inter-batch sleep):
- Idle with no traffic: ~0.04 mA average
- With moderate traffic (1 packet/min): ~0.1 mA average
- With heavy traffic (1 packet/10 s): ~0.5 mA average

---

## Battery life estimates

### Two AA Energizer Ultimate Lithium (L91) — 3000 mAh each in series

| Mode | Average current | Estimated life |
|------|----------------|----------------|
| `DIGI_ONLY` continuous RX | ~6 mA | ~3 weeks |
| `DIGI_CAD` quiet rural site | ~0.04 mA | **~8 years** |
| `DIGI_CAD` moderate traffic | ~0.1 mA | **~3.5 years** |
| `TRACKER_ONLY` 120 s interval | ~2.5 mA | ~7 weeks |
| `TRACKER_ONLY` 600 s interval | ~0.8 mA | ~5 months |

> Battery life = 3000 mAh ÷ average current. Actual life depends on
> temperature, traffic load, and GPS fix time. L91 cells have a ~10-year
> shelf life and perform well at temperature extremes, making them ideal
> for remote installations.

### Solar operation

`DIGI_CAD` mode draws so little power at idle that even a very small solar
panel can sustain it indefinitely. A typical 1 W garden solar panel with a
small LiPo cell provides far more energy than the digipeater consumes in
any realistic APRS traffic environment.

For continuous RX (`DIGI_ONLY`), a panel delivering at least 600–700 mWh
per day is needed, which is marginal for small panels in poor weather.
`DIGI_CAD` is strongly recommended for solar/battery installations.

---

## Voltage and battery wiring

### E77 module voltage limits

The E77-400M22S module requires **1.8–3.6 V**. Fresh AA lithium cells in
series measure ~3.58 V open circuit — uncomfortably close to the 3.6 V
maximum. Under load the voltage drops quickly into the safe range, but it
is good practice to add protection.

### Recommended: 1N5817 Schottky diode in series

Place a single **1N5817** (or equivalent Schottky diode) in series between
the battery positive terminal and the module VDD pin.

```
[Battery +] ──── Anode | 1N5817 | Cathode ──── [E77 VDD]
[Battery −] ──────────────────────────────────── [E77 GND]
```

The 1N5817 drops approximately **0.2 V** at low currents, bringing
fresh-battery voltage from ~3.58 V down to ~3.38 V — well within the safe
operating range. At TX peaks (~100 mA) the drop increases slightly to
~0.3 V, still safe.

**Energy cost of the diode:** negligible. At 0.1 mA average current the
power lost in the diode is ~0.02 mW — less than one day's battery life
over a multi-year deployment.

**Alternative:** A low-dropout regulator (LDO) such as the MCP1700-3302
gives a precise 3.3 V output and handles the full input voltage range of
two AA cells (fresh at 3.58 V down to ~1.8 V depleted). Efficiency is
~92% at low currents, acceptable for this use case.

### Two AA cells — wiring summary

```
[AA+][AA−][AA+][AA−]
  Series = ~3.0–3.58 V, 3000 mAh

With diode:
  [+]──1N5817──[E77 VDD]   ~3.38 V fresh, ~2.8 V depleted
  [−]──────────[E77 GND]

Module operates down to 1.8 V, so cells are used to near full depletion.
```

---

## RF switch pin configurations

The STM32WLE5's internal radio is connected to an external RF switch.
Pinout varies by module — uncomment the right block in `config.h`.

| Module | Pins | Notes |
|--------|------|-------|
| Ebyte E77-400MBL-01 | PA6, PA7, PB3 | Verified against PicoTrack |
| Seeed LoRa-E5 / E5-mini | PA4, PA5 | Different logic from E77 |
| ST Nucleo-WL55JC | PC3, PC4, PC5 | Both LP and HP paths |
| RAK3172-E (no TCXO) | PB8, PC13 | LP path only |

> **Important:** `radio.setRfSwitchTable()` must be called *before* `radio.begin()`.

---

## Digipeater behaviour

- **DIGI_ONLY:** Listens continuously. Digipeats any packet with an
  unactivated `WIDE1-1` or `WIDE2-N` path.
- **DIGI_CAD:** Runs CAD scans every 2 seconds. On preamble detection,
  switches to full RX and waits for the next complete packet.
- Both modes apply a random 80–450 ms delay before retransmitting to
  reduce collision probability with other digipeaters.
- An 8-element CRC32 ring buffer suppresses duplicate packets.
- Own beacon packets are never digipeated.

---

## Adjusting the beacon rate

Change `BEACON_INTERVAL_S` in `config.h`. Recommended minimums:

- Fixed station: 600 s (10 min)
- Slow vehicle: 120 s (2 min)
- Fast vehicle / balloon: 30–60 s

---

## Known caveats

- **APRSPacketLib field names** — the library evolves quickly. If `APRSPacket`
  struct fields differ from `main.cpp`, check the library header and adjust
  `buildAPRSPacket()` accordingly.
- **Stop2 and Serial** — serial output is flushed before entering Stop2.
  The USART restarts automatically on wakeup under the STM32 Arduino core.
- **GPS power switching** — `GPS_POWER_PIN` logic assumes a high-side P-FET
  (HIGH = GPS on). Invert if using an N-FET or active-low load switch.
- **TCXO vs passive crystal** — E77 modules with SN batch ≥ 3202995 have a
  TCXO. Modules below this batch have a passive crystal. Both work with
  `TCXO_VOLTAGE 1.7f` in `config.h`; passive crystal modules are tolerant
  of the TCXO init call.
