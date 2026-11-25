# STM32WLE5 Low-Power LoRa APRS — Tracker + Digipeater

Built around the **Ebyte E77-400MBL-01** (STM32WLE5CC, 433 MHz) with support
for the ST Nucleo-WL55JC and other STM32WLE5-based modules.

**[Open the browser setup page](https://radiohound.github.io/LoRa-Tracker-Digipeater/tools/setup.html)** to change settings and update firmware over USB (Chrome or Edge).

Libraries: **[RadioLib](https://github.com/jgromes/RadioLib)** (jgromes) + **[APRSPacketLib](https://github.com/richonguzman/APRSPacketLib)** (richonguzman).

Licensed under the [MIT License](LICENSE) — free to use, modify, and distribute with attribution.

Verified against the working [PicoTrack](https://github.com/radiohound/PicoTrack)
firmware by K6ATV for RF switch pinout, TCXO voltage, and radio init sequence.

---

## Repository branches

| Branch | Purpose |
|--------|---------|
| **`main`** | General-purpose LoRa APRS tracker and digipeater. Four operating modes: `TRACKER_ONLY`, `TRACKER_DIGI`, `DIGI_ONLY`, `DIGI_CAD`. |
| **`low-power-ground-digi-for-balloons`** | (formerly `balloon-digi`) Adds `MODE_BALLOON_DIGI` — a super low power sleep-and-wait ground station digipeater that autonomously acquires and tracks a specific balloon callsign. |

---

## File overview

```
platformio.ini        — build/upload config and library deps
include/config.h      — ALL user settings (callsign, mode, pins…)
src/main.cpp          — firmware
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
- Verify `RFSWITCH_PINS` / `RFSWITCH_TABLE` matches your board (see RF switch table below)

The rest (callsign, comment, beacon interval, LoRa frequency, Horus...) can be changed later on the serial
port, without rebuilding: see [Settings on the serial port](#settings-on-the-serial-port). The values in
`config.h` are only the defaults. **The tracker transmits nothing until you set a callsign**
(it starts as `N0CALL`).

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

## Settings on the serial port

Callsign, comment, beacon interval, the LoRa radio settings and the Horus settings are kept in the chip's
flash and changed over the serial port, as in [lora-aprs-digi](https://github.com/radiohound/lora-aprs-digi):
no rebuild. The values in `config.h` are the defaults.

Open a terminal on the USB serial port (115200 baud, local echo off) and **press a key within 3 seconds
of power-up** to enter setup mode (the tracker waits there until you type `exit` or `reboot`). The commands also
work while it is running in the tracker modes.

```
set call W2HAB-11        callsign and SSID (set call W2HAB changes only the callsign)
set beacon_s 60          check each value as you type it; a refused value changes nothing
set freq 433.775         LoRa radio settings (freq bw sf cr power sync preamble) need:  apply
save                     keep the changes after a power-off (refused until radio changes are applied)
show   get NAME   help   defaults   beacon   horus   reboot [force]   exit
```

- **Nothing is transmitted while the callsign is `N0CALL`** (the default). `beacon` and `horus` refuse too.
- `save` writes one record into the last 2 KB flash page (`board_upload.maximum_size` keeps the program out of
  it). A damaged or out-of-range record is ignored entry by entry and the defaults are used.
- Reflashing the program keeps the saved settings. To clear them, erase that page or type `defaults`, `apply`, `save`.
- `OPERATING_MODE` is still a compile-time choice, and the CAD digipeater modes do not read the command line yet.
- The engine is `src/settings/tk_settings.c` (plain C, tested on a PC: `cd test/host && make test`), the flash and
  serial glue is `src/settings/tk_glue.cpp`.
- **LoRa APRS and Horus can use different callsigns.** `call` + `ssid` is the APRS source; `horus_call` is the Horus v3
  callsign (empty = the APRS one). Each is checked on its own: with `call` at `N0CALL` Horus still sends if `horus_call`
  is a real callsign, and the other way round.

### Setup page (browser)

`tools/setup.html` does the same from a browser (Chrome or Edge, Web Serial, no terminal program): connect, read, change
the settings in a form, save, and check that every value reads back as sent. It also updates the firmware over the USB
cable. **Open it here: https://radiohound.github.io/LoRa-Tracker-Digipeater/tools/setup.html** (GitHub Pages, serving the `main`
branch of this repository), or open `tools/setup.html` as a saved file. The page shows the basic settings first (callsign, SSID,
comment and map symbol with its icon, how often to beacon, the "also transmit on the other SF" box, and the main Horus settings); the rest sits behind **Advanced
settings**, which opens by itself when a value in it is refused. Saving covers every setting, open or not. Close any program that
has the serial port open first (a terminal, `pio device monitor`): two readers split the bytes. If the tracker does not answer, press
Reset and Connect within 3 seconds.

Update over USB, no ST-LINK: with the page connected, **Flash firmware** types `bootloader`, and the chip's built-in
serial bootloader takes over; or hold **Key 1** (PA1) and tap Reset. Only the program area is rewritten, so the saved
settings stay. The first firmware on a board still needs an ST-LINK once. `tools/flash_serial.py` does the same from the
command line. **Held at power-up, the user key (PA0) puts the defaults back and saves them.** Set `KEYS_ENABLED 0` in `config.h` on
a flight board that uses PA0 or PA1 for something else. `firmware/manifest.json` (optional) lists firmware that comes with the page.

The page is tested without hardware against a simulated tracker built from the same settings engine
(`cd test/host && make test-page`, needs `node`). Tested on an E77: the `bootloader` command puts the chip into its
ROM bootloader (device ID 0x497) and `tools/flash_serial.py` then erased, wrote, verified and started a 130 KB build in
37 s with the saved settings kept. *Not yet tested on a board:* the same from the browser page, the Key 1 and user key
(PA0) presses at power-up, and the page against real hardware.

---

## Horus Binary v2 (4FSK) — optional

The tracker can send Horus Binary v2 (4FSK, 100 baud, 270 Hz tone spacing) after each APRS
beacon, from the same E77. It is **off by default**. Build the `ebyte_e77_horus` environment, or
set `HORUS_ENABLED 1` in `config.h`.

| Setting (`config.h`) | Meaning |
|---|---|
| `HORUS_VERSION` | `3` (default) = the callsign is in the packet, no ID needed. `2` = payload ID; v2 is deprecated and new v2 IDs are no longer issued. A v3 packet with the usual fields fits a 32-byte frame, so it takes the same ~3.3 s of airtime as v2. |
| `horus_call` (setting) | v3 payload callsign. Empty (the default) means the tracker callsign with its SSID (e.g. `W2HAB-11`). Letters, digits, `-` and `/`, up to 15 characters, case sensitive. |
| `HORUS_PAYLOAD_ID` | v2 only: payload ID, 256–65535. 256 is the shared test ID. |
| `HORUS_FREQ_MHZ` | Lowest of the four tones, set 1 kHz above the nominal frequency so receivers tuned to the nominal frequency (USB) hear the tones in the audio band. Default 431.051 = nominal 431.050 MHz, the local Horus frequency here; 434.200 MHz (set 434.201) is the usual frequency in other regions. |
| `HORUS_TX_POWER_DBM` | Above 14 dBm uses the high-power PA. See the PA note below for lower power. |
| `HORUS_BEACON_DIVIDER` | Send Horus on every Nth beacon. |
| `HORUS_TIME_SYNC_SECONDS`, `HORUS_TIME_SYNC_OFFSET_SECONDS` | Time slots, same scheme as RS41ng: transmit when GPS time (seconds from the top of the hour) satisfies `(t − OFFSET) mod SYNC == 0`. `0` = off. For two payloads on one frequency use the same SYNC and different OFFSETs, e.g. SYNC 10 with OFFSET 0 and 5 (a Horus v2 packet takes ~3.3 s, so two fit in 10 s). Both payloads need a GPS time lock for the slots to hold. |

The packet is built from the GPS fix and the BMP280 altitude and temperature (if present). v3 also
sends speed, satellites and an ascent rate worked out from the last two packets. The settings are
copied into a runtime structure (`horusSettings()` in `src/horus/horus.h`) so a serial or web
settings page can change them without a rebuild. Horus runs in the
tracker modes only (`TRACKER_ONLY`, `TRACKER_DIGI`).

**Clock.** Horus needs accurate symbol timing. The Arduino STM32 core runs this MCU from MSI
(48 MHz), an on-chip RC oscillator that measured about 1.9 % slow on an E77 — symbols went
out at ~98 baud and receivers could not decode them. With Horus enabled the firmware runs the
system clock from the TCXO instead (`HSE_SYSCLK`, `src/clock_hse.cpp`), which gives 100 baud.
If HSE does not start (crystal-only module) it falls back to MSI.

**Power amplifier.** The E77-400M22S antenna is fed from the high-power PA. With a `TX_LP`
entry in the RF switch table, RadioLib uses the low-power PA at 14 dBm and below, which is not
connected to the antenna on this module. Set `RF_HP_PA_ONLY 1` to use the HP PA at every
power level (range −9…22 dBm). The default of 0 keeps the previous behaviour.

**v3 encoder.** `src/horus/horus_v3.cpp` is a small hand-written ASN.1 (UPER) encoder for the
fields above. `tools/test_horus_v3.py` checks it against the horusdemodlib ASN.1 definition: random
packets must match the reference encoder byte for byte and decode back (3000 packets, 0 failures;
packets too long for a 32-byte frame move to a 48-byte frame).

**Third-party code.** The Horus layer-2 coding (Golay FEC, interleaver, scrambler) is in
`src/horus/`, copied unchanged from horusdemodlib under the **LGPL 2.1** (see
`src/horus/NOTICE.md`). The rest of this repository is MIT. It is only compiled when Horus is
enabled.

**What was tested.** Over the air on an E77-400M22S (STM32WLE5) with an RSPduo recording and a
horusdemodlib 0.6.x station (RTL-SDR): the packets (v2 and v3) decode with CRC OK, and upload to SondeHub
(with correct UTC time stamps) from a test firmware; the tracker build decodes with a fake
GPS fix. **Not tested:** a real GPS fix in flight, and current draw with the TCXO clock.

**Earlier conclusion corrected.** This section used to say Horus was impossible because every
`SET_TX_CONTINUOUS_WAVE` forces a ~5 ms TCXO restart. In I/Q recordings no carrier gaps were
found in any configuration: the failure was the slow MSI clock. One related finding: a radio
TCXO delay of 0 µs stops the radio transmitting at all (500 µs and 5 ms work).

---

### Also transmit on the other SF (`dual_sf`)

A checkbox on the setup page, named after the SF it adds: **Also transmit on SF7** when the main SF is 12, **SF12** when the main SF is 7 (over serial: `set dual_sf on`). Each LoRa APRS beacon is then sent a second time, right after the first, with the same position on the other common spreading factor. For any other main SF it does nothing (and is greyed out on the page).

Why: to see whether SF7 receivers hear a flight, and how well, and for chase cars that listen on SF7. At 125 kHz SF7 takes about 0.1 s of airtime where SF12 takes about 2.4 s, so the extra copy costs little. The second copy uses the same frequency, bandwidth, coding rate and sync word, and **has no digipeater path**, so it is heard and gated by iGates but not repeated by every digipeater in sight of a balloon at altitude. The board keeps listening on the main SF only, so it still digipeats on that SF but not on the other. The extra copy follows the APRS rate of the flight phase.

Tested: the transmit sequence on the bench (SF12 beacon with its path, then the SF7 copy without a path, same position), and the page label and settings on a PC. Not yet checked over the air: that an SF7 receiver decodes the copy, and the SF7 airtime on a recording.

### Extra comment on some LoRa APRS beacons

`comment2` is added after the normal `comment` (with a space between) on every `comment2_every`-th beacon: 1 is every beacon, 3 every third, 0 never (the default). It suits text that need not go out every time, such as a contact note, and saves airtime on the other beacons. Up to 43 characters. The count restarts at each power-up, so the first extra comment goes out on beacon number `comment2_every`.

### Seeing LoRa APRS on SondeHub

A LoRa APRS balloon appears on amateur.sondehub.org when an iGate puts its beacon on APRS-IS and SondeHub's APRS gateway accepts it. The gateway takes position reports with the balloon symbol (`/O`, which this firmware sends) and no `NOHUB` in the path. **The tracker page hides APRS payloads that have not been above 1,500 m**, so a board on the bench does not show until you switch on **Show Low Altitude APRS** in the page's settings (a browser setting). Once the payload has been above 1,500 m it stays visible. Horus packets are not affected. Use different callsigns for the two feeds (`call` + `ssid` for LoRa APRS, `horus_call` for Horus), as SondeHub's own wiki advises for payloads of different modulations.

Checked: the beacons were stored by SondeHub (API) and shown with the low-altitude setting on. Not checked: how the page treats a flight above 1,500 m.

### Horus note (v3)

`horus_note` is a short free text (up to 30 characters: letters, digits, space and `_ + / = - .`) sent in every Horus v3 packet, as an extra-sensor entry named `note` with a text value. It makes the packet longer, so it takes longer on the air: with a note the frame grows from 32 to 48 bytes (about 40 % more airtime, 4.4 s instead of 3.2 s at 100 baud), or 64 bytes for a long callsign and a long note. If a packet does not fit even in 64 bytes, the note is left out rather than the packet. Leave it empty for the shortest packets. Not used by version 2.

Tested: the encoder against the horusdemodlib ASN.1 definition (4,000 random packets, bit for bit), and over the air: horusdemodlib 0.6.2 decodes it, shows the `extraSensors` entry and uploads the packet to SondeHub. How SondeHub's own pages display the note was not checked.

## Send rates by flight phase (tracker + digipeater mode)

LoRa APRS and Horus each have their own interval for three phases, and the GPS decides which phase the board is in:

| Phase | LoRa APRS | Horus | Default |
|---|---|---|---|
| Ascent (also on the bench, before any flight) | `beacon_s` | `horus_asc` | 30 s / 30 s |
| Descent | `aprs_desc` | `horus_desc` | 15 s / 10 s |
| Landed | `aprs_gnd` | `horus_gnd` | 60 s / 60 s |

Set them with `set horus_desc 10` and so on (5 s to 24 h), or on the setup page. On the ground the board repeats the last good position.

How the phase is found (`src/flight/flight_phase.h`, thresholds are constants there):
- **Descent**: the board has climbed at least 500 m above its first fix, then sinks faster than 5 m/s for 5 s in a row. One bad GPS fix does not trigger it.
- **Landed**: it has flown, is at least 300 m below its highest point, and has stood still (under 0.7 m/s vertical and 3 km/h) for 60 s. A board on a desk never reaches it, because it needs a flight first.
- **Back to ascent**: climbing faster than 2 m/s for 10 s (a new launch).

The GPS stays on all the time in this mode and is read once a second; between packets the radio listens, so the board still digipeats. Horus packets stay on the time-slot grid (`horus_sync`, `horus_off`), and a 30 s interval is a whole number of 10 s cycles. The send is started about 1.5 s before the slot so the position is fresh. When a Horus packet is about to go, a due LoRa APRS beacon waits until the Horus packet is done.

Tested: the phase detector against simulated flights on a PC (`make -C test/host test`), and the rates at their defaults on the bench with a live GPS. Not tested: an actual flight, descent and landing detection with real GPS velocities, and the case where the GPS loses its fix for a long time. Tracker-only mode still sends one Horus packet per beacon (`horus_div`).

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
