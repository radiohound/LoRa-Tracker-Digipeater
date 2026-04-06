# STM32WLE5 LoRa APRS — Ground Station Balloon Digipeater

This is the **`balloon-digi`** branch. It adds `MODE_BALLOON_DIGI` — a
low-power ground station digipeater that autonomously acquires and tracks
a single high-altitude balloon, digipeating its APRS packets for the
duration of the flight.

The ground station runs unattended on two AA batteries. It draws only
~0.04 mA while waiting for the balloon to launch, then automatically
switches to a scheduled RX/digipeat cycle once the balloon is heard.

---

## Repository branches

| Branch | Purpose |
|--------|---------|
| **`main`** | General-purpose LoRa APRS tracker and digipeater. Four operating modes: `TRACKER_ONLY`, `TRACKER_DIGI`, `DIGI_ONLY`, `DIGI_CAD`. Use this for a handheld tracker, a fixed digipeater, or a combined tracker+digi node. |
| **`balloon-digi`** | This branch. Adds `MODE_BALLOON_DIGI` for a dedicated ground station that tracks and digipeats a specific balloon callsign. Includes all modes from `main`. |

Both branches target the **Ebyte E77-400MBL-01** (STM32WLE5CC) and are
compatible with the ST Nucleo-WL55JC and other STM32WLE5-based modules.

---

## How it works

### Acquisition phase (~0.04 mA average)

The digipeater does not know when the balloon will launch. Rather than
listening continuously, it runs **CAD (Channel Activity Detection)** scans
in batches. CAD takes ~65 ms at SF12 and detects whether a LoRa preamble
is present, drawing only ~1.5 mA during that brief window.

**Batch structure:** 15 CAD scans are run per radio initialisation cycle
(~30 seconds of scanning), then the radio is fully powered down and the
MCU enters Stop2 for 30 seconds before the next batch. This amortises the
~100 ms / 7 mA radio init cost across 30 seconds of scan time rather than
paying it every 2 seconds.

When a preamble is detected, the radio switches to full RX and waits for
the complete packet. The source callsign is checked against
`BALLOON_CALLSIGN`. A match triggers sync to the tracking phase.

### Tracking phase (~0.68–1.0 mA average)

Once synced, the RTC schedules wakeups timed to the balloon's TX interval.
The digipeater sleeps for most of each 30-second cycle, wakes up a few
seconds before the expected transmission, opens an RX window, digipeats
the packet if received, and sleeps again.

An IIR filter continuously refines the measured interval. The RX window
starts at 10 seconds and narrows to 4 seconds after ~20 packets (~10
minutes of flight) as timing confidence grows.

Three consecutive missed packets cause the digipeater to return to
acquisition mode automatically.

---

## Operating modes

| Mode | GPS | Radio state | Avg current |
|------|-----|-------------|-------------|
| `TRACKER_ONLY` | Yes | TX then Stop2 | ~2–8 mA |
| `TRACKER_DIGI` | Yes | TX then RX | ~5–6 mA |
| `DIGI_ONLY` | No | Continuous RX | ~6 mA |
| `DIGI_CAD` | No | CAD + deep sleep | ~0.04–0.1 mA |
| `BALLOON_DIGI` | No | CAD acq + sched RX | **~0.04 mA idle** |

---

## Power budget (Ebyte E77 @ 3.3 V)

| State | Current |
|-------|---------|
| Stop2 — MCU only, radio off | ~2 µA |
| MCU idle / light sleep | ~1.5 mA |
| LoRa RX | ~5 mA |
| CAD scan (~65 ms at SF12) | ~1.5 mA |
| LoRa TX at 22 dBm (peak) | ~100 mA |

### BALLOON_DIGI breakdown

**Acquisition** (waiting for balloon, potentially days or weeks):

| Action | Duration | Current | Avg over 60 s |
|--------|----------|---------|---------------|
| radioInit() | 100 ms | 7 mA | — |
| 15 × CAD scan | 65 ms each | 1.5 mA | — |
| 15 × inter-scan sleep | 1935 ms each | 0.002 mA | — |
| Active batch total | 30 s | — | ~0.074 mA |
| Inter-batch Stop2 | 30 s | 0.002 mA | 0.002 mA |
| **60 s cycle average** | | | **~0.038 mA** |

**Tracking** (during 3-hour flight, converged 4 s window):

| Action | Duration per 30 s | Current |
|--------|-------------------|---------|
| radioInit() + wakeup | 100 ms | 7 mA |
| RX window (converged) | 4 s | 5 mA |
| TX digipeat | ~2.7 s | ~80 mA avg |
| Stop2 sleep | ~23 s | 0.002 mA |
| **30 s cycle average** | | **~8.5 mA** |

The tracking average looks high because TX dominates. With TX omitted
(listen-only, not digipeating) tracking drops to ~0.7 mA average.

---

## Battery life estimates

### Two AA Energizer Ultimate Lithium (L91) — 3000 mAh each in series

One balloon flight per month, 3-hour flight, 30-second TX interval
(360 packets per flight):

| Phase | Duration/month | Avg current | mAh/month |
|-------|---------------|-------------|-----------|
| Acquisition (between flights) | ~29 days 21 hrs | ~0.038 mA | ~27 mAh |
| Tracking (flight) | ~3 hours | ~8.5 mA | ~26 mAh |
| **Total** | | | **~53 mAh/month** |

**Estimated battery life: 3000 ÷ 53 ≈ ~4.5 years**

> With TX power reduced to 14 dBm (halves TX current, still excellent range
> for a balloon): tracking drops to ~5 mA average → **~6 years**.

> Practical limit is the battery's shelf life. Energizer L91 is rated for
> 20 years of storage — battery chemistry, not capacity, is the limit at
> very low drain rates.

---

## Voltage and battery wiring

### E77 module voltage limits

The E77-400M22S requires **1.8–3.6 V**. Two fresh L91 cells in series
measure ~3.58 V open circuit — right at the 3.6 V limit. A small amount of
protection is worth adding.

### Recommended: 1N5817 Schottky diode in series

Place a single **1N5817** Schottky diode in series with the positive supply.

```
[Battery +] ──── Anode | 1N5817 | Cathode ──── [E77 VDD]
[Battery −] ──────────────────────────────────── [E77 GND]
```

The diode drops ~0.2 V at low currents, bringing fresh-battery voltage from
~3.58 V to ~3.38 V. At TX peaks (~100 mA) the drop rises to ~0.3 V, still
safely within the module's operating range.

**Cost:** At 0.05 mA average current (acquisition phase), the power
dissipated in the diode is ~0.01 mW — completely negligible against a
multi-year deployment. The diode costs you less than one day of battery
life over the entire service life of the installation.

**Alternative:** A MCP1700-3302 LDO gives a precise 3.3 V from fresh cells
down to near-depletion, at ~92% efficiency. Useful if you want tighter
voltage regulation, but the diode approach is simpler and sufficient.

### Wiring summary

```
Two AA cells in series:
  Fresh: ~3.58 V    Depleted: ~2.0 V    Capacity: 3000 mAh

With 1N5817:
  [AA+]─[AA−][AA+]─[AA−] ── Anode|1N5817|Cathode ── E77 VDD
                                                       E77 GND ── [Battery −]

Effective supply: ~3.38 V fresh → ~2.8 V depleted
Module minimum:   1.8 V  ← still ~1 V of headroom at depletion
```

---

## Setup

1. Set `OPERATING_MODE MODE_BALLOON_DIGI` in `config.h`
2. Set `BALLOON_CALLSIGN` to your balloon's callsign (without SSID)
3. Set `BALLOON_TX_INTERVAL_MS` if your balloon beacons at something other
   than 30 seconds
4. Flash and leave running — acquisition begins automatically

---

## Tuning parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| `BALLOON_CALLSIGN` | `"W6ABC"` | Source filter. `""` = any station |
| `BALLOON_TX_INTERVAL_MS` | `30000` | Starting interval estimate (ms) |
| `CAD_ACQ_INTERVAL_MS` | `2000` | Time between CAD scans |
| `CAD_ACQ_BATCH_SIZE` | `15` | Scans per radio init cycle |
| `CAD_ACQ_BATCH_SLEEP_MS` | `30000` | Deep sleep between batches (ms) |
| `LISTEN_WINDOW_INITIAL_MS` | `10000` | Starting RX window width |
| `LISTEN_WINDOW_MIN_MS` | `4000` | Minimum RX window after convergence |
| `MAX_MISS_COUNT` | `3` | Misses before returning to acquisition |
| `TIMING_CONVERGE_COUNT` | `20` | Packets before window reaches minimum |
| `IIR_WEIGHT` | `7` | Smoothing weight (higher = slower adaptation) |

### Tuning CAD scan interval

`CAD_ACQ_INTERVAL_MS` must be less than the packet air time at your SF/BW
settings. At SF12/BW125 a full LoRa-APRS packet takes approximately 3
seconds on the air, so 2000 ms guarantees you will run a CAD scan while
any packet is still transmitting.

If you change to a lower spreading factor (faster packets, shorter range)
you must also reduce `CAD_ACQ_INTERVAL_MS` proportionally.

### Tuning batch size and sleep

Increasing `CAD_ACQ_BATCH_SIZE` or `CAD_ACQ_BATCH_SLEEP_MS` reduces power
further but increases the maximum time between checks. The defaults give a
worst-case acquisition delay of about 60 seconds from balloon first
transmission to digipeater sync — acceptable for any flight scenario.

---

## RF switch pin configurations

| Module | Pins | Notes |
|--------|------|-------|
| Ebyte E77-400MBL-01 | PA6, PA7, PB3 | Verified against PicoTrack |
| Seeed LoRa-E5 / E5-mini | PA4, PA5 | Different logic from E77 |
| ST Nucleo-WL55JC | PC3, PC4, PC5 | Both LP and HP paths |
| RAK3172-E | PB8, PC13 | LP path only |

---

## Known caveats

- **First packet missed** — the CAD-detected packet is always missed because
  the radio cannot switch from CAD to full RX fast enough to catch the
  preamble in progress. The digipeater syncs on this packet's timing but
  begins digipeating from the second packet onward. This is by design.
- **Callsign filter** — setting `BALLOON_CALLSIGN ""` accepts any station.
  In a busy APRS area this will cause the digipeater to sync to the first
  LoRa packet heard, which may not be your balloon. Use a specific callsign
  for reliable operation.
- **APRSPacketLib field names** — verify struct field names against the
  installed library header if you encounter compile errors.
- **TCXO** — E77 modules with SN batch ≥ 3202995 have a TCXO; earlier
  batches have a passive crystal. Both are handled correctly by the default
  `TCXO_VOLTAGE 1.7f` setting.
