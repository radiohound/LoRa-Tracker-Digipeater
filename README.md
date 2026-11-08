# STM32WLE5 Low-Power LoRa APRS — Tracker + Digipeater

Built around the **Ebyte E77-400MBL-01** (STM32WLE5CC, 433 MHz) with support
for the ST Nucleo-WL55JC and other STM32WLE5-based modules.

Libraries: **[RadioLib](https://github.com/jgromes/RadioLib)** (jgromes) + **[APRSPacketLib](https://github.com/richonguzman/APRSPacketLib)** (richonguzman).

Licensed under the [MIT License](LICENSE) — free to use, modify, and distribute with attribution.

Verified against the working [PicoTrack](https://github.com/radiohound/PicoTrack)
firmware by K6ATV for RF switch pinout, TCXO voltage, and radio init sequence.

---

## Repository branches

| Branch | Purpose |
|--------|---------|
| **`main`** | General-purpose LoRa APRS tracker and digipeater. Four operating modes: `TRACKER_ONLY`, `TRACKER_DIGI`, `DIGI_ONLY`, `DIGI_CAD`. |
| **`balloon-digi`** | Adds `MODE_BALLOON_DIGI` — a low-power ground station digipeater that autonomously acquires and tracks a specific balloon callsign. |
| **`lacf3-digi`** | Adds a separate build, `e77_lacf3_digi`: a digipeat-only firmware that relays **LACF 3.0** frames without the PPM tables, and plain LoRa APRS frames. See [LACF 3.0 digipeater](#lacf-30-digipeater-lacf3-digi-branch). |
| **`lacf3-digi-full`** | The same digipeater **with the two PPM tables**, build `e77_lacf3_digi_full`: decodes and checks every LACF 3.0 frame before relaying it, and can re-encode plain packets as LACF 3.0. See [With the PPM tables](#with-the-ppm-tables-lacf3-digi-full-branch). |

---

## File overview

```
platformio.ini        — build/upload config and library deps
include/config.h      — ALL user settings (callsign, mode, pins…)
src/main.cpp          — firmware
src/digi3/            — the LACF 3.0 digipeater (environment e77_lacf3_digi; settings in config_digi3.h)
lib/lacf3/            — LACF 3.0 frame code (hb3.c, from lora-aprs-ideas)
test/host/            — PC test of the LACF 3.0 digipeater against the reference vectors
```

---

## Quick start

### 1. Install VS Code and PlatformIO

1. Download and install [Visual Studio Code](https://code.visualstudio.com/).
2. Open VS Code, go to the **Extensions** panel (Ctrl+Shift+X / Cmd+Shift+X),
   and search for **PlatformIO IDE**. Install it and restart VS Code when prompted.
3. PlatformIO will automatically install the STM32 platform and all required
   libraries the first time you build — no manual library installation needed.

### 2. Get the code

**Option A — Clone with Git:**
```bash
git clone https://github.com/radiohound/LoRa-Tracker-Digipeater.git
```
Then in VS Code: **File → Open Folder** and select the cloned folder.

**Option B — Download ZIP:**
1. Click the green **Code** button on GitHub, then **Download ZIP**.
2. Extract the ZIP to a folder of your choice.
3. In VS Code: **File → Open Folder** and select the extracted folder.

PlatformIO recognises the project automatically when it sees `platformio.ini`
in the root.

### 3. Configure

Open `include/config.h` and set:
- `OPERATING_MODE` — choose your mode (see table below)
- `MY_CALLSIGN`, `MY_SSID`, `MY_COMMENT`
- `LORA_FREQ` for your region (433.775 MHz is used almost world wide — check local LoRa APRS frequency)
- Verify `RFSWITCH_PINS` / `RFSWITCH_TABLE` matches your board (see RF switch table below)

### 4. Connect the hardware

Connect the E77 dev board to your computer via the ST-Link USB port (the
built-in programmer on the dev board). On first connection, Windows may
install ST-Link drivers automatically; if not, install the
[ST-Link drivers](https://www.st.com/en/development-tools/stsw-link009.html)
manually.

### 5. Compile and upload

**Using the VS Code UI:**
- Click the **PlatformIO** icon in the left sidebar (the alien head).
- Under **PROJECT TASKS → ebyte_e77_dev**, click **Upload** to compile and
  flash in one step. Click **Build** to compile only.

**Using the terminal:**
```bash
pio run                        # compile only
pio run --target upload        # compile and flash
```

### 6. Monitor serial output

- In VS Code PlatformIO sidebar: **Monitor** under PROJECT TASKS.
- Or in the terminal:
```bash
pio device monitor --baud 115200
```
You should see `[Radio] Init LoRa... OK` followed by GPS and TX log lines
within a few seconds of powering on.

---

## Operating modes

| Mode | GPS needed | Radio state | Avg current |
|------|-----------|-------------|-------------|
| `TRACKER_ONLY` | Yes | TX then Stop2 sleep | ~2–8 mA (GPS-dominated) |
| `TRACKER_DIGI` | Yes | TX then continuous RX | ~5–6 mA |
| `DIGI_ONLY` | No | Continuous RX always | ~6 mA |
| `DIGI_CAD_SOLAR` | No | Two-stage: CAD sentinel + full RX window, digipeats all | **~0.1 mA idle, ~6 mA active** |
| `DIGI_CAD_BATTERY` | No | Two-stage: CAD sentinel + RX window for moving stations only | **~0.1 mA idle** |

Both CAD modes use the same two-stage design to maximise detection
reliability while minimising power:

**Stage 1 — CAD sentinel (~100 µA):** The radio performs a 65 ms Channel
Activity Detection scan every `CAD_SCAN_INTERVAL_MS` (default 1000 ms)
continuously. Because LoRa CAD detects any chirp energy on the channel —
not just a preamble — a scan interval equal to or shorter than the expected
transmission length guarantees detection on the first transmission heard.

**Stage 2 — Active RX window (~6 mA):** On detecting a signal the radio
switches to full continuous receive. The window stays open as long as
qualifying packets continue to arrive, then returns to Stage 1 after
silence. What counts as "qualifying" differs between modes:

- **`DIGI_CAD_SOLAR`** — any received packet resets the 6-minute window
  and is digipeated. Suitable for solar or mains-powered installations
  where power is not a constraint.

- **`DIGI_CAD_BATTERY`** — only packets from stations that have **moved**
  since they were last heard reset the window and are digipeated. Fixed
  ground stations are received but silently ignored, saving significant
  power at sites with regular beaconing ground stations nearby. The RX
  window duration is set by `CAD_BEACON_INTERVAL_MS` plus a 15-second
  buffer (see configuration below).

---

## Power budget (Ebyte E77 @ 3.3 V)

| State | Current |
|-------|---------|
| Stop2 — MCU only, radio off | ~2 µA |
| MCU idle / light sleep | ~1.5 mA |
| LoRa RX (continuous) | ~5 mA |
| Channel Activity Detection (CAD) scan (~65 ms at SF12) | ~1.5 mA |
| GPS acquiring fix | ~20–30 mA |
| LoRa TX at 22 dBm (peak) | ~100 mA |

### Mode-by-mode averages

**TRACKER_ONLY** (120 s interval, 30 s GPS warm start):
Average ~2–3 mA, dominated by GPS. With GPS power gating and a fast-fix
module this can drop below 1 mA average.

**DIGI_ONLY** (continuous RX):
Flat ~6 mA regardless of traffic. Suitable for mains or large solar.

**DIGI_CAD_SOLAR** (1 s CAD scan interval, 6 min RX window, all stations):
- Idle with no traffic: ~0.1 mA average (continuous CAD sentinel)
- With moderate traffic (1 packet/min): ~0.5 mA average
- With heavy traffic (1 packet/10 s): ~6 mA (effectively continuous RX)

**DIGI_CAD_BATTERY** (1 s CAD scan interval, moving stations only):
- Quiet site, no moving stations: ~0.1 mA (same as solar idle)
- Fixed ground station transmitting every 20 min: ~0.32 mA
- Moving station active (balloon pass, vehicle): ~6 mA while in range

---

## Battery life estimates

### Two AA Energizer Ultimate Lithium (L91) — 3000 mAh each in series

| Mode | Average current | Estimated life |
|------|----------------|----------------|
| `DIGI_ONLY` continuous RX | ~6 mA | ~3 weeks |
| `DIGI_CAD_SOLAR` quiet site | ~0.1 mA | **~3.4 years** |
| `DIGI_CAD_SOLAR` moderate traffic | ~0.5 mA | **~8 months** |
| `DIGI_CAD_BATTERY` quiet site | ~0.1 mA | **~3.4 years** |
| `DIGI_CAD_BATTERY` fixed ground station nearby | ~0.32 mA | **~1 year** |
| `TRACKER_ONLY` 120 s interval | ~2.5 mA | ~7 weeks |
| `TRACKER_ONLY` 600 s interval | ~0.8 mA | ~5 months |

> Battery life = 3000 mAh ÷ average current. Actual life depends on
> temperature, traffic load, and GPS fix time. L91 cells have a ~10-year
> shelf life and perform well at temperature extremes, making them ideal
> for remote installations.

### Solar operation

`DIGI_CAD` mode at idle (~0.1 mA) is easily sustained by even a small
salvaged solar panel — a typical lawn light panel produces far more current
than the CAD sentinel consumes. When active traffic triggers Stage 2, RX
current rises to ~6 mA, still well within the output of any small panel
in reasonable daylight.

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
- **DIGI_CAD_SOLAR / DIGI_CAD_BATTERY:** Both run CAD scans every
  `CAD_SCAN_INTERVAL_MS` (default 1 s) continuously. On detection, Stage 2
  switches to full continuous RX. `DIGI_CAD_SOLAR` digipeats all stations
  and resets the 6-minute window on every packet. `DIGI_CAD_BATTERY`
  digipeats only stations that have moved since last heard, and only resets
  the window for those stations — fixed ground stations are silently ignored.
  See the battery mode configuration section for setting the RX window.
- Both modes apply a random 80–450 ms delay before retransmitting to
  reduce collision probability with other digipeaters.
- An 8-element CRC32 ring buffer suppresses duplicate packets.
- Own beacon packets are never digipeated.

---

## DIGI_CAD_BATTERY — configuring the RX window

In battery mode the active RX window is set by specifying the **expected
beacon interval** of the moving stations you want to capture. The firmware
automatically adds a 15-second buffer so timing variation never causes a
missed packet.

In `config.h`, uncomment the line that best matches your use case:

```c
// Uncomment ONE of these to match the expected moving station beacon interval:

//#define CAD_BEACON_INTERVAL_MS    360000  // 6 min  — infrequent tracker
#define CAD_BEACON_INTERVAL_MS      180000  // 3 min  — typical vehicle or balloon (default)
//#define CAD_BEACON_INTERVAL_MS     60000  // 1 min  — fast vehicle
//#define CAD_BEACON_INTERVAL_MS     30000  // 30 sec — high-rate balloon or tracker
```

The actual RX window will be your chosen interval plus 15 seconds:

| `CAD_BEACON_INTERVAL_MS` | Actual RX window | Best for |
|--------------------------|-----------------|----------|
| 360000 (6 min) | 6 min 15 sec | Infrequent trackers |
| **180000 (3 min)** | **3 min 15 sec** | **Typical vehicle / balloon — covers any interval ≤ 3 min, including 30 s (default)** |
| 60000 (1 min) | 1 min 15 sec | Fast vehicle — covers any interval ≤ 1 min |
| 30000 (30 sec) | 45 sec | High-rate balloon / tracker — tightest window |

**How to choose:** set this to match the slowest beacon interval you expect
from stations you want to digipeat. If a balloon transmits every 30 seconds
and you set 3 minutes, you will still catch every packet — the window simply
stays open longer than necessary. If you set 30 seconds and a station
transmits every 3 minutes, the window will expire between transmissions and
the station will be re-detected by the CAD sentinel on its next transmission,
typically within one beacon cycle.

Fixed ground stations are never digipeated regardless of this setting.

---

## BMP280 barometric altimeter (optional)

A BMP280 pressure sensor can be added to the same I2C bus as the GPS
(PA9 = SCL, PA10 = SDA) to improve altitude accuracy. GPS altitude is
typically accurate to ±10–20 m; a barometer with a good sea-level
reference is significantly more stable.

Enable in `config.h`:

```c
#define BMP280_ENABLED     1       // 1 = use if present, 0 = disable
#define BMP280_I2C_ADDR    0x76    // 0x76 (SDO→GND) or 0x77 (SDO→VCC)
#define BMP280_CAL_SAMPLES 8       // GPS altitude readings to average
```

**How calibration works:** immediately after each GPS fix, the firmware
takes `BMP280_CAL_SAMPLES` altitude readings from the GPS at 1 Hz,
averages them to reduce GPS noise, then back-calculates the sea-level
pressure reference the BMP280 needs to report accurate absolute altitude.
From that point until the next fix, all transmitted altitude values come
from the barometer.

If no BMP280 is detected at boot the firmware falls back to GPS altitude
automatically — no configuration change is needed.

---

## Cutdown (tracker modes only)

A GPIO pin can be pulsed HIGH at a set altitude to trigger a cutdown
mechanism (nichrome wire, pyro charge, servo, etc.). Intended for
balloon payloads running `TRACKER_ONLY` or `TRACKER_DIGI`.

Enable and configure in `config.h`:

```c
#define CUTDOWN_ENABLED        1
#define CUTDOWN_PIN            PA0
#define CUTDOWN_ALTITUDE_M     30000   // meters MSL
#define CUTDOWN_ARM_ASCENT_M   500     // meters above launch before armed
#define CUTDOWN_CONFIRM_COUNT  5       // consecutive readings required
#define CUTDOWN_PULSE_MS       5000    // pin HIGH duration in milliseconds
```

**Safeguards:**
- **Arming delay** — the system records altitude at the first valid GPS fix
  (launch altitude) and will not arm until the balloon has risen
  `CUTDOWN_ARM_ASCENT_M` above that baseline. Prevents triggering at a
  high-altitude launch site or from a noisy reading on the ground.
- **Confirmation count** — once armed, `CUTDOWN_CONFIRM_COUNT` consecutive
  altitude readings must all be at or above `CUTDOWN_ALTITUDE_M`. The
  counter resets if any reading falls below the threshold, so a single
  spike cannot trigger the cutdown.
- **Timed pulse** — the pin is held HIGH for exactly `CUTDOWN_PULSE_MS`
  milliseconds (blocking), then driven LOW before any further code runs.
  The pin is guaranteed to return LOW regardless of what happens next.
- **One-shot** — fires once only. The system will not trigger again even
  if altitude readings remain above the threshold.

Altitude source follows the same priority as the beacon: BMP280 if
detected and calibrated, otherwise GPS altitude.

---

## Adjusting the beacon rate

Change `BEACON_INTERVAL_S` in `config.h`. Recommended minimums:

- Fixed station: 600 s (10 min)
- Slow vehicle: 120 s (2 min)
- Fast vehicle / balloon: 30–60 s

---

## Horus Binary 4FSK — not supported on this hardware

Horus Binary v1/v2 4FSK (100 baud, 270 Hz tone spacing) was investigated for
this platform and is **not feasible** on the STM32WLE5 with a TCXO-equipped
E77 module.

The STM32WLE5 SubGhz IP generates 4FSK by switching between four CW
frequencies using `SET_TX_CONTINUOUS_WAVE`. Each call to that command forces a
full TCXO startup sequence (~5 ms) enforced in hardware, regardless of prior
radio state. At 100 baud (10 ms/symbol), this creates ~50% dead time per
symbol — effectively transmitting OOK at 100 Hz rather than 4FSK. The
resulting OOK sidebands fall directly on adjacent tones (270 Hz spacing) and
prevent the modem from distinguishing symbols.

Approaches tried and confirmed not to work:
- Setting fallback mode to STDBY_XOSC or FS before transmission
- Calling `SetRfFrequency` while staying in TX CW mode
- Writing the PLL frequency register directly via `writeRegister`
- Reducing `tcxoDelay` to 0 or 1 ms
- Single vs multiple `fsk4.write()` calls

The root cause is a hardware limitation: the STM32WLE5 SubGhz IP always
enforces the TCXO startup delay on every `SET_TX_CONTINUOUS_WAVE` command.
This cannot be worked around in software.

**Possible hardware solutions** (not implemented):
- Always-on TCXO: solder the TCXO VCC line directly to 3.3V, bypassing DIO3
  control. Tone transitions would then only require PLL relock (~52 µs).
- Separate radio: add a Si4032 or Si4463 with native 4FSK hardware modulation
  on SPI alongside the E77. This is how RS41 sondes work.

---

## LACF 3.0 digipeater (`lacf3-digi` branch)

A second firmware in the same project, built by the PlatformIO environment **`e77_lacf3_digi`**
(in VS Code: PlatformIO sidebar, `e77_lacf3_digi`, Upload). It is digipeat only: no GPS, no iGate,
no tracking. Settings are in `src/digi3/config_digi3.h`; the tracker settings in `include/config.h` are not used.

What it repeats:

| Frame received | What it sends |
|---|---|
| Plain LoRa APRS (`3C FF 01` + text) | the same text with the new path |
| **LACF 3.0** (`3C FF 03`, proposal 3.0.0-dev.1) | relayed "without the model" (spec 12, Relay step 8): structure and CRC checked, path rewritten, the compressed content copied byte for byte, this relay's RXT tuple (RSSI, SNR, frequency offset, time to hop) added |
| LACF 2.0 (`3C FF 02`) | nothing: checking a 2.0 frame needs the PPM tables (about 152 KB), which this build leaves out |

Path rule (the same as the LoRa APRS Pi): the first unused hop decides. `fill-in` repeats `WIDE1-1` and
packets addressed to its callsign; `wide` (the default, for a high site) also `WIDE2-1` and `WIDE2-2`
(`WIDE2-2` becomes `MYCALL*,WIDE2-1`). It never repeats a path that breaks the WIDEn-N rule, its own
packets, a packet it already repeated, or the same packet twice within 30 seconds.

Before each transmission it waits a random 100 to 500 ms and checks the channel with CAD; if the channel
is busy it waits 0.5 to 2 s and looks again (6 times, then drops the packet). A watchdog restarts it if
the loop ever stops for 20 seconds. It beacons a fixed position as plain text (set `BEACON_LAT`/`BEACON_LON`),
and does nothing at all until `MY_CALLSIGN` is changed from `N0CALL`.

Size, built 8 November 2026 (RadioLib 7.8.1, Arduino STM32 core, arm-none-eabi-gcc 13.2, `-Os`):

| | Used | Of |
|---|---|---|
| Flash | 73 KB (Arduino core and HAL 25 KB, RadioLib 20 KB, C library 17 KB, digipeater 6 KB, LACF 3 frame code 5.5 KB) | 256 KB |
| RAM, fixed | 8 KB | 64 KB |
| Stack, deepest path (receive, decide, relay) | about 6.5 KB | |

The digipeater logic is in `src/digi3/digi3_core.c`, plain C with no Arduino code, and is tested on a PC
against the reference vectors of lora-aprs-ideas (1,013 relay vectors, all identical to the reference,
plus plain-frame and RXT cases):

```
make -C test/host VECTORS=/path/to/lora-aprs-ideas/test
```

Not yet tried on the air. The radio set-up (RF switch, TCXO 1.7 V) is the one PicoTrack uses on the
E77 MBL board. Power: it listens all the time, so expect about 5 to 7 mA at 3.3 V (the radio in receive
plus the processor in sleep mode between interrupts), plus the transmissions.

### With the PPM tables (`lacf3-digi-full` branch)

This branch adds LACF's two PPM tables (the *info-native* table for frames without a position block, kind 2,
and the *comment-only* table for the comment after a position block, kind 3) and the PPM coder, and builds
as **`e77_lacf3_digi_full`** (`-D D3_WITH_MODEL -D HB_FAST`).

* Every LACF 3.0 frame is **decoded in full** and held to every receiver rule, the PPM content included,
  before it is relayed. The relay then copies the content byte for byte, as spec 12 requires (re-encoding
  would give the same bytes, so it is not redone). A frame whose model version or destination code this build
  does not hold is still relayed without the model; a frame that fails a check is dropped.
* The decoded packet is printed on the serial port (`LOG_DECODED`).
* With **`TX_FORMAT_LACF3 1`**, plain packets heard (and the beacon) are **re-encoded as LACF 3.0** before
  they are repeated: LACF content when it is shorter, otherwise LACF 3.0 plain content (spec 12, "A plain
  text frame received, and the digipeater's transmit format is LACF"). Stock LoRa APRS firmware cannot read
  those frames, so it is off by default.
* `HB_FAST` uses the faster `build_freq` from lora-aprs-ideas (`tools/integrity/fast_freq.h`): the same
  output, with 32-bit instead of 64-bit divisions, which the Cortex-M4 has no instruction for.

Size, built 8 November 2026 with the same tools:

| | Without the tables (`lacf3-digi`) | With the tables (`lacf3-digi-full`) |
|---|---|---|
| Flash | 73 KB | **236 KB of 256 KB (about 25 KB free)** |
| of which PPM tables and coder | | 156 KB |
| RAM, fixed | 8 KB | 9.5 KB |
| Deepest stack (decode a frame) | about 6.5 KB | about 8 KB |

Host test with the tables (`make -C test/host VECTORS=...`, also `FAST=0`): all 1,013 relay vectors, and all
554 encode vectors of lora-aprs-ideas, which the digipeater's own conversion path encodes byte for byte as
the reference and decodes back to the same packet. Decode and encode time on the STM32 is not yet measured
(1.4 to 4.2 ms per decode on an ESP32 at 240 MHz; expect several times that at 48 MHz, still small next to a
2-second SF12 packet).

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
