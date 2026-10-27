# GaggiMate × Velofuso: closed-loop flow control fork

Two pieces of firmware, one serial cable between them.

| Part | What it is | File |
|---|---|---|
| **Pro controller fork** | GaggiMate controller firmware (base: master `b78beee`, Oct 18 2026) with a serial link to the meter board and flow phases closed loop on the measured ml/s. Version string `v1.99.0-meterlink.2`. | `controller/gaggimate-controller-meterlink-j4_*.bin`, `…-j5_*.bin` |
| **Meter board v1.7.1** | Your `digmesa-flow-display` v1.6.4 plus the serial link and a cap of 20 shot logs (oldest deleted first, also applied at boot). Display, Decent Scale emulation and BOOKOO pass-through are unchanged. | `meter/digmesa-flow-display_v1.7.1_*.bin`, `meter/digmesa-flow-display.ino` |
| **Source** | Controller fork as a git patch (the fork's commits on upstream `b78beee`); meter sketch as .ino + the patch script that produced it from v1.6.4. | `source/` |

## Read this first

1. **Display firmware must be the nightly build (Oct 18 2026 or later).** The fork is built from master, BLE protocol version **8**. The stable 1.9.0 display speaks protocol 5 and will show *"Version mismatch, update display"* and refuse to control. Switch the display to the nightly channel before flashing the controller.
2. **The BOOKOO path does not change.** The meter board still shows up as *Decent Scale FM* and relays the BOOKOO cup weight. Nothing to reconfigure.
3. **Flow phases need the meter.** With the link down, a flow phase runs the pump at 0 — on purpose, no fallback to the pump model. Pressure and power phases work as stock whether or not the meter is connected. So flash the Pro fork, then check the link status before brewing a flow profile.
4. **Don't press "Update controller" in the GaggiMate web UI** unless you want stock back. The fork reports `v1.99.0-meterlink.2`, so the UI should not offer it, but the display update itself is separate and fine.
5. **Rollback:** stock controller from the GaggiMate web flasher (nightly); meter board back to the v1.6.4 UPDATE bin. Nothing else changes.

## Which controller build

| Build | Meter on | Alba | Hardware scale (HX711 on J5) | When |
|---|---|---|---|---|
| `meterlink-j4` | **J4** (RX gpio44 / TX gpio43) | must be **off J4** — the plan's Y-harness with the Alba on the meter board (meter v1.7 does **not** drive the Alba yet; see "Not in this version") | stock, works | the all-wired plan |
| `meterlink-j5` | **J5** (RX gpio18 / TX gpio17) | stays stock on J4 | **disabled** (pins are the UART) | **start here**: nothing else moves, closed-loop flow today |

Recommendation: **J5 now.** The Alba stays where it is, the BOOKOO stays, you get flow control. Move to J4 when the scale board exists and the Alba moves onto the meter board.

## Wiring (J5 build)

Three wires, 3.3 V on both ends, **1 kΩ in series on each data line** (the meter board is USB-powered and can be on while the Pro is off).

| Meter board header | | Pro J5 (5-pin JST PH, order `+5V, SCK, DT1, DT2, GND`) |
|---|---|---|
| **TXD** (GPIO43) | → via 1 kΩ → | **DT1 / "SDA"** = gpio18 (Pro RX) |
| **RXD** (GPIO44) | ← via 1 kΩ ← | **SCK / "SCL"** = gpio17 (Pro TX) |
| **GND** | — | **GND** |

Leave J5 `+5V` and `DT2` empty. If the link stays silent, TX/RX swapped is the first thing to check.

J4 build: same three wires into J4 (`GND, RX, TX, +5V` as printed): TXD → J4 **RX**, RXD → J4 **TX**, GND → GND, `+5V` untouched. The Alba cable must be out of J4.

## Flashing

**Pro controller** (USB-C on the Pro, machine unplugged from mains, display can stay on):
- Chrome/Edge → https://espressif.github.io/esptool-js/ → Connect → add file `gaggimate-controller-meterlink-j5_BOOTSLOT+APP_flash-at-0xe000.bin` at address **`0xe000`** → Program. Don't tick *Erase flash*.
  Why 0xe000 and not 0x10000: the Pro has two app slots. Every OTA from the display flips between them, so a Pro that has been updated over BLE may be running from the second slot (`app1` at 0x340000). This file is the 8 KB boot-slot selector (points the bootloader back at `app0`) followed by the app at 0x10000, in one piece. It leaves the bootloader, partition table and NVS (BLE pairing, stored PID) untouched.
- `_UPDATE_flash-at-0x10000.bin` is the app alone: it only takes effect if `app0` is already the active slot — if the Controller Version doesn't change after flashing it, that's the reason; use the 0xe000 file.
- `_FULL_flash-at-0x0.bin` at **`0x0`** is the last resort (bootloader + partitions + selector + app). It also blanks the Pro's NVS, so the display has to be paired again (hold the steam switch while powering on to open the pairing window) and the controller's stored PID fallback is gone until the display sends settings.
- Alternative: PlatformIO `pio run -e controller-meterlink-j5 -t upload` from the fork source (that flashes the selector too).
- Check: GaggiMate web UI → Settings → System & Updates → **Controller Version** must read `v1.99.0-meterlink.2`.

**Meter board**: `digmesa-flow-display_v1.7.1_UPDATE_flash-at-0x10000.bin` at **`0x10000`**, same as every update so far. Wi-Fi, calibration and the BOOKOO setting are kept. (`_FULL_flash-at-0x0.bin` at `0x0` only if you want to start from scratch; that wipes the shot logs.)

## Checking the link (before coffee)

1. Meter board web page `http://flowmeter.local` → new card **GaggiMate Pro link**: *"● Pro connected"* means the Pro's heartbeat arrives. *"nothing from the Pro"* with lines sent counting up = the Pro isn't answering (wrong build on the Pro, or RX/TX swapped).
2. Pro USB console (115200): type `S` → the status now has a **Meter link** block: `Link: up, flow data: fresh`, measured flow, line counters, flow-loop state and gains. `G` prints the gains; `G 0.05 0.15 20 0 4 3 1.5` sets them until the next `$G` from the meter board (≤ 5 s) — so tune on the meter board's page, not here.
3. Run water through the meter (open group, power/pressure mode): the GaggiMate chart's **pump flow** must now be the meter's number (it was the model before). `water pumped` is now metered volume.
4. First flow profile on the needle-valve rig: target 2 ml/s, 9 bar ceiling, valve nearly closed. Expect: flow comes up within ~1 s, pressure climbs, the ceiling PI takes the pump just before 9 bar and holds 9.0 ± 0.2 bar without stalling 2 bar low (the stock bug) and without dropping to 5–6 bar (the meterlink.1 bug, shot 292). Open the valve: flow back to 2 ml/s, pressure drops. Then pull the Digmesa signal wire with the pump running in a flow phase: the flow loop sees 0 and pushes, the ceiling still holds; expect one overshoot of up to ~1.5 bar before it settles on 9.
5. Pull the meter's TXD during a flow phase: pump stops within 0.5 s, controller log says `FLOW FAULT`. Plug it back: the phase resumes. The display just shows 0 flow / 0 pump power during the fault — a proper on-screen message needs a display fork, not included.

## How the flow loop works

- **Inner loop** (60 Hz): pump duty = `startDuty` (pre-load) + `kp·(target − measured)` + `ki·∫(target − measured)`, clamped 0..100 %, integration frozen against the limits. Plant gain is ~0.3 ml/s per 1 % duty on the Velofuso.
- **Setpoint slew** `ramp` (ml/s per s): the target moves at most this fast, so a phase change or profile ramp doesn't kick the pump. A new flow phase starts from the flow that is actually running.
- **Pressure ceiling** (override PI, since meterlink.2): the phase's pressure setpoint. A second PI on `ceiling − pressure` (pressure looked 0.25 s ahead on its slope) proposes a duty too, and the lower of the two duties runs the pump: `limKp` in % duty per bar, `limKi` in % duty per bar per second. Both integrals are kept as reset feedback, i.e. each relaxes toward the duty actually applied, so the one not in charge sits right above it and takes over without a bump; neither can wind up. Stock did `min(flowDuty, pressureDuty)` with a zeroed integrator, which is why flow phases stalled ~2 bar under the limit.
- **Slip band guard**: on the Velofuso about 55–60 % duty at 9 bar only covers slip. When the pump already delivers nothing (meter < 0.05 ml/s) and the puck is visibly draining the pressure (falling > 0.1 bar/s), the ceiling integral stops going down, so the pump is never parked deep in the slip band and the pressure lands on a lowered ceiling without undershooting. meterlink.1's backstop did exactly that parking (bleed and cut on the flow integral), and shot 292 dipped to 6.5 and 5.3 bar in a 2 ml/s phase with a 9 bar ceiling because of it.
- **Bumpless phase start**: a flow phase starts from the duty that was already running (a pressure phase holding 9 bar), not from `startDuty`, when that is higher. Nothing pushes past 12.5 bar.
- **ff** (0..1): optional pump-model feedforward on top. 0 = measured flow only (default).
- **Fail-safe**: no `$F` for 500 ms → duty 0, loop reset, `FLOW FAULT` in the log; resumes when data is back.
- **Telemetry**: `pump flow`, `water pumped`, the puck-flow/puck-resistance estimator and the heater's thermal feedforward all use measured flow while the link is fresh. PRESSURE and POWER modes are byte-for-byte stock logic (there's a test asserting pressure-mode output is identical with and without the link).

Defaults (`FlowLoopGains.h`, and the meter page's *Defaults* button): `kp 0.05, ki 0.15, start 20 %, ff 0, ramp 4 ml/s², limKp 3.0, limKi 1.5` (the lim numbers kept their values in meterlink.2, so the meter board needs no update, but they now mean % duty per bar and % duty per bar·s). Chosen against a simulated Velofuso (your Oct-20 coefficients) with 150–500 ms pump lag, a Digmesa pulse meter and pressure noise: open-group step settles in ~0.5 s with < 0.1 ml/s overshoot; ceiling held within 0.1 bar, no dips on entering a flow phase at 9 bar, on stiff and soft systems, tight and choked pucks. limKp/limKi up to ~10 work too: faster on a soft system, slightly more ripple from sensor noise. Known trait with `ff 0`: while pressure is still climbing fast (≈1 bar/s) the flow trails the target by ~0.3 ml/s, because slip grows with pressure and the integral has to follow; it catches up once pressure levels off. If that bothers you: `ff 1` (adds the pump model as a head start, the measured loop still corrects it) or `ki 0.25`. If you see overshoot on step changes, lower `ki` first (0.10), then `kp`.

## The serial protocol

115200 8N1, ASCII, `$<type>[,<field>…]*XX\r\n`, XX = XOR of the bytes between `$` and `*`, two upper-case hex digits (NMEA style). Anything not a complete, checksummed line is dropped, so boot text on either side is harmless.

Meter board → Pro:

| Line | Rate | Fields |
|---|---|---|
| `$F,<ml_s>,<ml_total>,<seq>,<ml_s_display>` | 20 Hz | fast flow (150 ms window, for the loop), water since the last zero, sequence, the 0.5 s-window flow the screen shows |
| `$H,<uptime_s>` | 1 Hz | heartbeat |
| `$G,<kp>,<ki>,<start_%>,<ff>,<ramp>,<limKp>,<limKi>` | at boot, every 5 s, on save | flow-loop gains |
| `$T,<mm>` | 4 Hz | tank distance — **not sent yet** (Alba not on the meter board in v1.7.1) |

Pro → meter board:

| Line | When |
|---|---|
| `$Z` | display tare (every process start and shot save). Zeroes the `$F` volume counter only; shot logs on the meter still start from the BLE scale tare / from flow exactly as in v1.6 |
| `$B,<0|1>` | brew valve open/closed |
| `$L,<ch>,<brightness>` | LED command from the display (J4 build only; the meter counts them and ignores them) |
| `$H,<uptime_s>` | 1 Hz heartbeat, so the meter page can show "Pro connected" |

## Not in this version (deliberately)

- **Alba on the meter board** (VL53L0X tank distance, PCA9634 lights, chime). The J4 controller build already forwards `$T` and `$L`; the meter side needs an I²C master + the bench check of the Alba's bus voltage from the plan. Until then: use the J5 build and leave the Alba on J4.
- **Scale board / J5 `$W`** — you're keeping the BOOKOO for now.
- **On-screen fault message** for a lost meter — display is stock.
- **PRESSURE phases with a flow limit** still use the stock model-based limit (your pressure tune is untouched). Say if you want the measured limit there too.

## Changes

- **meterlink.2** (Oct 27 2026): the pressure ceiling is an override PI instead of the target-trimming limiter plus bleed/cut backstop; slip-band guard; a flow phase starts from the duty already running. Fixes the 6.5 / 5.3 bar dips of shot 292. Meter board unchanged (v1.7.1).
- **meterlink.1** (Oct 25 2026): first release; Pro sends the `$H` heartbeat.

## Source

- `source/gaggimate-meterlink.patch`: the fork as two commits against upstream `b78beee`. To get a working tree:
  `git clone https://github.com/jniebuhr/gaggimate.git && cd gaggimate && git checkout -b meterlink-flow-control b78beee && git am ../gaggimate-meterlink.patch && git tag v1.99.0-meterlink.2`
  then `pio run -e controller-meterlink-j5` (the tag gives the build its version string); tests: `pio test -e native -f test_meter_flow_loop -f test_line_protocol`. Rebasing onto a newer nightly is `git rebase <nightly-commit>`; the change touches `PressureController`, `DimmedPump`, `GaggiMateController`, adds `MeterLink`/`LineProtocol` and two test suites.
- `meter/digmesa-flow-display.ino`: v1.7.1. `source/patch_v17.py` is the script that turned your v1.6.4 into it (every anchor asserted). Build settings unchanged: ESP32S3 Dev Module, 16 MB, `app3M_fat9M_16MB`, OPI PSRAM, CDC on boot, esp32 core 3.3.10, GFX Library for Arduino 1.6.4 (upstream 1.6.4 needs the three `spiFrequencyToClockDiv` calls patched for core 3.3.x — Waveshare's copy already has that), U8g2.
