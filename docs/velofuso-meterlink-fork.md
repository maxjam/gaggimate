# GaggiMate × Velofuso: closed-loop flow control fork

Two pieces of firmware, one serial cable between them.

| Part | What it is | File |
|---|---|---|
| **Pro controller fork** | GaggiMate controller firmware (base: master `b78beee`, Oct 18 2026) with a serial link to the meter board and flow phases closed loop on the measured ml/s. Version string `v1.99.0-meterlink.4`. | `controller/gaggimate-controller-meterlink-j4_*.bin`, `…-j5_*.bin` |
| **Meter board v1.7.3** | Your `digmesa-flow-display` v1.6.4 plus the serial link, a cap of 20 shot logs (oldest deleted first, also applied at boot), the pump dead-zone setting and the Pro's pump diagnostics in every shot log. Display, Decent Scale emulation and BOOKOO pass-through are unchanged. | `meter/digmesa-flow-display_v1.7.3_*.bin`, `meter/digmesa-flow-display.ino` |
| **Calibration profiles** | Four test profiles for the needle-valve rig (see *Calibration runs*). | `profiles/cal-*.json` |
| **Source** | Controller fork as a git patch (the fork's commits on upstream `b78beee`); meter sketch as .ino + the patch script that produced it from v1.6.4. | `source/` |

## Read this first

1. **Display firmware must be the nightly build (Oct 18 2026 or later).** The fork is built from master, BLE protocol version **8**. The stable 1.9.0 display speaks protocol 5 and will show *"Version mismatch, update display"* and refuse to control. Switch the display to the nightly channel before flashing the controller.
2. **The BOOKOO path does not change.** The meter board still shows up as *Decent Scale FM* and relays the BOOKOO cup weight. Nothing to reconfigure.
3. **Flow phases need the meter.** With the link down, a flow phase runs the pump at 0 — on purpose, no fallback to the pump model. Pressure and power phases work as stock whether or not the meter is connected. So flash the Pro fork, then check the link status before brewing a flow profile.
4. **Don't press "Update controller" in the GaggiMate web UI** unless you want stock back. The fork reports `v1.99.0-meterlink.4`, so the UI should not offer it, but the display update itself is separate and fine.
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
- Check: GaggiMate web UI → Settings → System & Updates → **Controller Version** must read `v1.99.0-meterlink.4`.

**Meter board**: `digmesa-flow-display_v1.7.3_UPDATE_flash-at-0x10000.bin` at **`0x10000`**, same as every update so far. Wi-Fi, calibration and the BOOKOO setting are kept. (`_FULL_flash-at-0x0.bin` at `0x0` only if you want to start from scratch; that wipes the shot logs.)

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
- **Pump dead zone** (`deadzone`, meter page *dead zone %*, since meterlink.3 / meter v1.7.2): the pump power below which the pump stops turning. The two PIs then work on the range above it: pump power = deadzone + u·(100 − deadzone) %, so the loop never parks the pump where it doesn't turn and has to restart it (the on/off cycling of flush 295 at 2–5 ml/s and 0.4 bar). Since meterlink.4, u = 0 is the pump's **slowest speed, not off**: a flow target below what the pump gives at its slowest speed (≈1.1 ml/s on an open group at 10 %) runs the pump at that speed — a bit more flow than asked, but steady — instead of switching it on and off around the target (the first seconds of flush 301). In a flow phase the pump then only stops for the pressure ceiling (its PI asking for nothing), the 12.5 bar cutoff, a zero target or stale meter data. Set the dead zone to the lowest power that still keeps the pump running (step the power down with an open group and watch where the flow stops). Default 0 = off (plain duty, u = 0 is off, same as meterlink.2). Flow phases only; pressure and power phases are stock.
- **Bumpless phase start**: a flow phase starts from the duty that was already running (a pressure phase holding 9 bar), not from `startDuty`, when that is higher. Nothing pushes past 12.5 bar.
- **ff** (0..1): optional pump-model feedforward on top. 0 = measured flow only (default).
- **Fail-safe**: no `$F` for 500 ms → duty 0, loop reset, `FLOW FAULT` in the log; resumes when data is back.
- **Telemetry**: `pump flow`, `water pumped`, the puck-flow/puck-resistance estimator and the heater's thermal feedforward all use measured flow while the link is fresh. PRESSURE and POWER modes are byte-for-byte stock logic (there's a test asserting pressure-mode output is identical with and without the link).
- **Diagnostics** (since meterlink.4): the Pro sends `$D` to the meter board 10 times a second — the pump power it sends, its pressure reading, the mode, the profile targets and the loop state. Meter v1.7.3 writes them into every shot log next to the metered flow (see *Diagnostics log*). Nothing on the Pro or the display changes because of it.

Defaults (`FlowLoopGains.h`, and the meter page's *Defaults* button): `kp 0.05, ki 0.15, start 20 %, ff 0, ramp 4 ml/s², limKp 3.0, limKi 1.5, deadzone 0` (the lim numbers kept their values in meterlink.2, so the meter board needs no update, but they now mean % duty per bar and % duty per bar·s). Chosen against a simulated Velofuso (your Oct-20 coefficients) with 150–500 ms pump lag, a Digmesa pulse meter and pressure noise: open-group step settles in ~0.5 s with < 0.1 ml/s overshoot; ceiling held within 0.1 bar, no dips on entering a flow phase at 9 bar, on stiff and soft systems, tight and choked pucks. limKp/limKi up to ~10 work too: faster on a soft system, slightly more ripple from sensor noise. Known trait with `ff 0`: while pressure is still climbing fast (≈1 bar/s) the flow trails the target by ~0.3 ml/s, because slip grows with pressure and the integral has to follow; it catches up once pressure levels off. If that bothers you: `ff 1` (adds the pump model as a head start, the measured loop still corrects it) or `ki 0.25`. If you see overshoot on step changes, lower `ki` first (0.10), then `kp`.

## The serial protocol

115200 8N1, ASCII, `$<type>[,<field>…]*XX\r\n`, XX = XOR of the bytes between `$` and `*`, two upper-case hex digits (NMEA style). Anything not a complete, checksummed line is dropped, so boot text on either side is harmless.

Meter board → Pro:

| Line | Rate | Fields |
|---|---|---|
| `$F,<ml_s>,<ml_total>,<seq>,<ml_s_display>` | 20 Hz | fast flow (150 ms window, for the loop), water since the last zero, sequence, the 0.5 s-window flow the screen shows |
| `$H,<uptime_s>` | 1 Hz | heartbeat |
| `$G,<kp>,<ki>,<start_%>,<ff>,<ramp>,<limKp>,<limKi>[,<deadzone_%>]` | at boot, every 5 s, on save | flow-loop gains (8th field from meter v1.7.2; missing = unchanged) |
| `$T,<mm>` | 4 Hz | tank distance — **not sent yet** (Alba not on the meter board in v1.7.x) |

Pro → meter board:

| Line | When |
|---|---|
| `$Z` | display tare (every process start and shot save). Zeroes the `$F` volume counter only; shot logs on the meter still start from the BLE scale tare / from flow exactly as in v1.6 |
| `$B,<0|1>` | brew valve open/closed |
| `$L,<ch>,<brightness>` | LED command from the display (J4 build only; the meter counts them and ignores them) |
| `$H,<uptime_s>` | 1 Hz heartbeat, so the meter page can show "Pro connected" |
| `$D,<pump_%>,<bar>,<mode>,<target_bar>,<target_ml_s>,<ctrl_bar>,<ctrl_ml_s>,<model_ml_s>,<i_%>,<flags>` | 10 Hz, pump diagnostics (meterlink.4+), see below |

## Diagnostics log (meterlink.4 + meter v1.7.3)

Every shot log on the meter board gets ten more columns after `cup_g`, from the Pro's latest `$D` (empty when none came in the last 0.5 s, e.g. with an older Pro build):

| Column | Meaning |
|---|---|
| `pump_pct` | pump power the Pro sends, % (the DAC level on the Velofuso: 5 V × % × max pump power) |
| `pressure_bar` | the Pro's pressure reading (averaged sensor, before the controller's Kalman filter) |
| `pro_mode` | 0 power, 1 pressure, 2 flow |
| `target_bar`, `target_ml_s` | profile targets: the pressure target (pressure phase) or ceiling (flow phase); the flow target (flow phase) or limit (pressure phase) |
| `ctrl_bar` | the setpoint the stock pressure loop is tracking (the profile target through its 1 Hz setpoint filter) |
| `ctrl_ml_s` | the target the flow loop is tracking (after `ramp`), flow phases only |
| `model_ml_s` | the pump model's flow at this power and pressure (`pumpModelCoeffs` / `pumpSlipCoeffs`): compare it with `flow_ml_s` to see how far the model is off |
| `i_pct` | integral part of the active loop, %: pressure phase = the stock I term (duty it adds); flow phase = the flow PI's integral (share of the range above the dead zone) |
| `pro_flags` | 1 valve open, 2 pressure ceiling holds the pump, 4 flow fault (meter data stale), 8 flow loop running |

The meter page shows the same live on the Pro link card (*Pro now: pump … % · … bar*), and its shot chart draws pump power and pressure under the flow, with the flow target dashed. The timing is the meter's 10 Hz rows; the `$D` values are at most ~0.1 s older than the row.

## Calibration runs

Four profiles in `profiles/`, all for the needle-valve rig. Import them in the GaggiMate web UI (Profiles → import). Each run is an ordinary shot; the meter board logs everything — no notes needed. Keep the tank full (all seven runs use about 1.5 l) and something big under the rig.

| Profile | What it does | What it measures |
|---|---|---|
| `[Cal A] Power 100-8% down` | power steps 100, 90 … 10, 8 %, 5 s each, open loop. A step that reaches 10.5 bar is skipped. | flow vs power at whatever pressure the valve gives; where the pump stops |
| `[Cal B] Ceiling 9-2 bar, flow loop` | flow target 8 ml/s, pressure ceiling 9, 8 … 2 bar (first step 15 s, then 8 s) | pump power needed to hold each pressure at a given valve opening: the pump model's two curves |
| `[Cal C] Stock pressure 2-9 bar` | stock pressure phases 2, 3 … 9 bar, 6 s each | how the stock pressure loop drives the pump (the pulsing) |
| `[Cal D] Flow 0.5-4 ml/s` | flow phases 0.5, 1 … 4 ml/s, ceiling 9 bar, 6 s each | the flow loop across targets, including below the pump's slowest flow |

Order (fewest valve changes):

1. **A**, valve fully open.
2. **B**, setting the valve during the first step (9 bar) so the meter shows about **6 ml/s**; then hands off.
3. **B** again at about **4 ml/s**; then **D** without touching the valve.
4. **B** again at about **2 ml/s**; then **C** and **A** without touching the valve.

Then one normal shot of a real profile. Send the meter CSVs (`GM_…csv`); the GaggiMate shot files aren't needed for this. The CSVs give flow, pump power and pressure on one time base, which is what the new `pumpModelCoeffs` / `pumpSlipCoeffs` are fitted from.

The pump model is Q = d·(full(P) + slip(P)) − slip(P): straight in pump power d at each pressure P. Cal B gives, at each pressure, three (power, flow) points at three valve openings; the line through them is the model at that pressure (slope = full + slip, zero crossing = slip / (full + slip), the power that only covers slip). Cal A checks the result over the whole power range, including where the real pump isn't straight in d (the 34–40 % flat spot of shots 297/298).

## Not in this version (deliberately)

- **Alba on the meter board** (VL53L0X tank distance, PCA9634 lights, chime). The J4 controller build already forwards `$T` and `$L`; the meter side needs an I²C master + the bench check of the Alba's bus voltage from the plan. Until then: use the J5 build and leave the Alba on J4.
- **Scale board / J5 `$W`** — you're keeping the BOOKOO for now.
- **On-screen fault message** for a lost meter — display is stock.
- **PRESSURE phases with a flow limit** still use the stock model-based limit (your pressure tune is untouched). Say if you want the measured limit there too.

## Changes

- **meterlink.4** (Oct 27 2026): `$D` pump diagnostics to the meter board (logged by meter v1.7.3); with a dead zone set, a flow phase keeps the pump at its slowest speed instead of switching it off below the slowest flow. Dead zone 0 behaves as before. `LineProtocol` takes up to 11 fields per line.
- **meterlink.3** (Oct 27 2026): pump dead-zone setting for flow phases (8th `$G` field, console `G … [deadzone%]`). Default 0 = same behaviour as meterlink.2. Meter board v1.7.2 adds the field to its page.
- **meterlink.2** (Oct 27 2026): the pressure ceiling is an override PI instead of the target-trimming limiter plus bleed/cut backstop; slip-band guard; a flow phase starts from the duty already running. Fixes the 6.5 / 5.3 bar dips of shot 292. Meter board unchanged (v1.7.1).
- **meterlink.1** (Oct 25 2026): first release; Pro sends the `$H` heartbeat.

## Source

- `source/gaggimate-meterlink.patch`: the fork's commits against upstream `b78beee`. To get a working tree:
  `git clone https://github.com/jniebuhr/gaggimate.git && cd gaggimate && git checkout -b meterlink-flow-control b78beee && git am ../gaggimate-meterlink.patch && git tag v1.99.0-meterlink.4`
  then `pio run -e controller-meterlink-j5` (the tag gives the build its version string); tests: `pio test -e native -f test_meter_flow_loop -f test_line_protocol`. Rebasing onto a newer nightly is `git rebase <nightly-commit>`; the change touches `PressureController`, `DimmedPump`, `GaggiMateController`, adds `MeterLink`/`LineProtocol` and two test suites.
- `meter/digmesa-flow-display.ino`: v1.7.3. `source/patch_v17.py` turned your v1.6.4 into v1.7.1, `source/patch_v172.py` v1.7.1 into v1.7.2, `source/patch_v173.py` v1.7.2 into v1.7.3 (every anchor asserted). Build settings unchanged: ESP32S3 Dev Module, 16 MB, `app3M_fat9M_16MB`, OPI PSRAM, CDC on boot, esp32 core 3.3.10, GFX Library for Arduino 1.6.4 (upstream 1.6.4 needs the three `spiFrequencyToClockDiv` calls patched for core 3.3.x — Waveshare's copy already has that), U8g2.
