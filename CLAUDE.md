# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

This is a fork of **FluidNC** (CNC firmware for the ESP32, https://github.com/bdring/FluidNC), tracking upstream `bdring:main` via merges. Grbl-protocol-compatible; machines are described by a YAML config file uploaded to the ESP32's filesystem rather than by recompiling.

On top of upstream, this fork ("MillingStation") adds two things, both confined to a small set of files:

1. **OLED UI changes** — the status text shown on the SSD1306 OLED was translated to Spanish and a custom boot banner was added ([FluidNC/src/OLED.cpp](FluidNC/src/OLED.cpp)).
2. **Delta MS300 VFD support over Modbus** — real-time spindle power (`W`) reading and a dedicated `DeltaMS300` spindle type, built on top of the generic Modbus-VFD protocol layer and threaded through the status report.

See "Local additions vs. upstream FluidNC" below before touching spindle or OLED code — that section explains the moving parts and the places that must stay in sync.

## Build / upload / test commands

The firmware is a PlatformIO project rooted one level down: `src_dir = FluidNC`, `include_dir = FluidNC/include`, `data_dir = FluidNC/data` (see [platformio.ini](platformio.ini)). All `pio` commands below are run from the repo root.

```
# Build (default env is "wifi")
pio run
pio run -e wifi          # WiFi build (default_envs)
pio run -e bt             # Bluetooth build
pio run -e noradio        # No WiFi/BT (serial only)
pio run -e wifi_s3        # ESP32-S3 variant
pio run -e debug          # debug build_type

# Flash firmware + filesystem (config.yaml etc. from FluidNC/data)
pio run -e wifi -t upload
pio run -e wifi -t uploadfs

# Serial monitor (custom terminal with FluidNC-aware features, incl. xmodem file upload)
python fluidterm/fluidterm.py
python fluidterm/fluidterm.py --upload myconfig.yaml   # push a file over the wire

# Build a full release bundle (all environments + embedded web UI) into dist/
python build-release.py
```

Native unit tests (PlatformIO + GoogleTest, x86 host — not the ESP32 build):

```
pio test -e tests            # with ASan/UBSan
pio test -e tests_nosan       # without sanitizers
pio test -e tests -f "*PinOptionsParser*"   # run a single test filter
```

Test sources live in [FluidNC/tests/](FluidNC/tests/); the build filter for that env only compiles `src/Pins/PinOptionsParser.cpp` and `src/string_util.cpp` plus the test files, so most of the firmware is **not** exercised by `pio test` — it only covers isolated, host-buildable utility code. There is a bit-rotted `native`/googletest env commented out in [platformio.ini](platformio.ini) that historically covered more; it currently doesn't build.

Code style is enforced by `.clang-format` (see [CodingStyle.md](CodingStyle.md) for the naming/include conventions: `CamelCase` classes/namespaces, `snake_case` methods, `_snake_case` member variables, `#pragma once`, cpp-includes-its-own-header-first).

## Architecture

### Module system
Optional subsystems (OLED, WebUI, BT, SD, etc.) are `Module`/`ConfigurableModule` instances (see [FluidNC/src/Module.h](FluidNC/src/Module.h)) registered globally at static-init time via `ModuleFactory::InstanceBuilder<T>` / `ConfigurableModuleFactory::InstanceBuilder<T>`, e.g. the OLED module registers itself at the bottom of [OLED.cpp](FluidNC/src/OLED.cpp) (`oled_module`, init priority 104). The core loop calls `init()`, `poll()`, `status_report()`, `build_info()` etc. on every registered module — grep for `for (auto const& module : Modules())` to find call sites. A module can be compiled in/out purely via `build_src_filter` in [platformio.ini](platformio.ini) — no `#ifdef` needed.

### Configuration system
YAML config (`config.yaml` on littlefs) is parsed into a tree of `Configuration::Configurable` objects. Simple structural settings implement `group()` to bind YAML keys to member fields directly; polymorphic families (Spindles, Motors, VFD protocols, etc.) are built through a `GenericFactory<Base>` keyed by a string name in the YAML (e.g. `SpindleFactory::DependentInstanceBuilder<VFDSpindle, GenericProtocol> registration("ModbusVFD")` at the bottom of [GenericProtocol.cpp](FluidNC/src/Spindles/VFD/GenericProtocol.cpp)). No RTTI is used (ESP32 constraint) — see [FluidNC/src/Configuration/_Overview.md](FluidNC/src/Configuration/_Overview.md) for the full explanation and the registration boilerplate required (anonymous namespace, must live in the .cpp).

### Channels and reporting
`Channel` ([FluidNC/src/Channel.h](FluidNC/src/Channel.h)) is the abstraction for anything that can send/receive text lines (serial, telnet, WebUI websocket, SD, and the OLED — OLED is a *write-only* Channel that parses the same status-report text stream everything else consumes, rather than talking to internal state directly). `Report.cpp` builds the Grbl-style realtime status line (`<Idle|MPos:...|FS:...|...>`) that is broadcast to all registered channels; the OLED module's `parse_status_report()` re-parses that exact text format to drive the display, so a new field is really "visible" to the OLED only once it exists in the `<...>` status string.

### Spindle / VFD stack
`Spindles::Spindle` is the base class; `Spindles::VFDSpindle` ([VFDSpindle.h](FluidNC/src/Spindles/VFDSpindle.h)/[.cpp](FluidNC/src/Spindles/VFDSpindle.cpp)) drives spindles over RS485 Modbus RTU from a dedicated FreeRTOS task (`vfd_cmd_task` in [VFDProtocol.cpp](FluidNC/src/Spindles/VFD/VFDProtocol.cpp)), so VFD state (`_sync_dev_speed`, output power, drive status) is written from that task and read from the main/motion tasks — treat it as cross-task shared state. For protocols where `use_delay_settings()` returns `false` (currently `GenericProtocol`/`DeltaMS300`, `H100Protocol`, `H2AProtocol`, `SiemensV20Protocol`, `NowForeverProtocol`), `VFDSpindle::setState()` waits for the drive to actually report the target RPM instead of just sleeping a fixed delay: `vfd_cmd_task` pushes every real speed reading it parses onto `VFDProtocol::vfd_speed_queue`, and `setState()` blocks on `xQueueReceive(vfd_speed_queue, &_sync_dev_speed, 3000)` — a hard 3s timeout that raises `mc_critical(ExecAlarm::SpindleControl)` if the drive never confirms the requested speed. Per-VFD-model behavior (command byte sequences, response parsing) is implemented via `VFDProtocol` subclasses; most concrete VFDs (Huanyang, YL620, H100/H2A, Danfoss VLT2800, Siemens V20, NowForever, and the generic Modbus one used for Delta) live under [FluidNC/src/Spindles/VFD/](FluidNC/src/Spindles/VFD/), one `.cpp`/`.h` pair each, each self-registering with `SpindleFactory` at the bottom of its `.cpp`.

`GenericProtocol` ([GenericProtocol.h](FluidNC/src/Spindles/VFD/GenericProtocol.h)/[.cpp](FluidNC/src/Spindles/VFD/GenericProtocol.cpp)) is config-driven: each command (`cw_cmd`, `set_rpm_cmd`, `get_rpm_cmd`, ...) is a small DSL string of hex bytes / `rpm`/`power`/`status` placeholders and scale expressions (`*50/60`, `%`, ...), parsed by `send_vfd_command()`/`scale()`/`parser()`. Known VFD models (currently only `"DeltaMS300"`) have their command strings baked into the `VFDtypes[]` table in `GenericProtocol.cpp`, selected either via `model: DeltaMS300` under a generic `ModbusVFD:` YAML section, or via the dedicated `DeltaMS300:` section ([DeltaMS300.h](FluidNC/src/Spindles/VFD/DeltaMS300.h)/[.cpp](FluidNC/src/Spindles/VFD/DeltaMS300.cpp)), a one-line `GenericProtocol` subclass that just presets `_model` so no `model:` key is needed — both paths read the same `VFDtypes[]` entry, so the command strings only live in one place (any of the individual `*_cmd` YAML keys can still override a default from that table).

## Local additions vs. upstream FluidNC

This section documents the pieces that are custom to this fork (not present in upstream FluidNC) and their moving parts, since they cut across several files.

### OLED text/translation ([OLED.cpp](FluidNC/src/OLED.cpp))
- `show_state()` (~line 102) maps the Grbl state word (`Idle`, `Run`, `Alarm`, `Hold`, `Door`, `Sleep`, `Check`, `Jog`) to Spanish display strings (`Disponible`, `Ejecutando`, `¡Alarma!`, ...). Adding a new Grbl state requires adding a branch here or it silently falls through and shows the raw English state.
- `init()` (~line 66) draws a custom boot banner ("MillingStation ®") instead of the upstream FluidNC banner, then dwells 5s.
- `parse_AP()` (~line 460) also localizes the AP message prefix ("P. de acceso: ").

### Delta MS300 Modbus VFD support
Introduced across commits `8185f680` (Modbus support for DeltaMS300 + real-time power reading), `c4f6f2fc` (W display support), and `abb3cb0e` (status reading + speed-read improvements), then fixed up after a bug hunt into intermittent hangs on `M3`/`M4`. The current pieces:

- **`GenericProtocol`** ([GenericProtocol.h](FluidNC/src/Spindles/VFD/GenericProtocol.h)/[.cpp](FluidNC/src/Spindles/VFD/GenericProtocol.cpp)): `get_output_power()` and `get_status()` (register reads for output power and drive status), plus `power`/`status` tokens in the command DSL parser (`parser()`/`set_data()`). The `"DeltaMS300"` entry in `VFDtypes[]` has the Modbus register map, e.g. `get_power_cmd = "03 21 0F 00 01 > 03 02 power*100"` (register 210FH, tenths-of-kW → the `*100` converts it to Watts). `set_rpm_cmd = "06 20 01 rpm*50/60 > echo"` and `get_rpm_cmd = "03 21 03 00 01 > 03 02 rpm*6/5"` (registers 2001H/2103H) are exact inverses of each other by construction (`*6/5` undoes `*50/60`) and **the `50/60` constant is empirically calibrated against this specific motor/VFD/mechanical setup, not derived from the register's documented `Hz×100` format** — an earlier attempt to "fix" `set_rpm_cmd` to `*100/60` (reasoning from the register documentation and an assumed 2-pole 60-RPM/Hz ratio) made the spindle physically spin at 2x the commanded RPM; don't re-derive this constant from the Delta manual alone; treat it as measured, and re-measure (command a known RPM, read the VFD's own keypad/display) before changing it again. `get_min_rpm_cmd`/`get_max_rpm_cmd` are intentionally empty — register 211AH isn't in the MS300's documented status-monitor map and `min_RPM`/`max_RPM` are always supplied via YAML anyway, so auto-querying them was both unreliable and unused (matches upstream `bdring/FluidNC`'s own `DeltaMS300.cpp`, which does the same). The parser keeps "last known good" values (now per-instance fields — `_last_good_speed`/`_last_good_power`/`_last_good_status` — not the `static` locals they used to be) and rejects out-of-range readings (`sync <= 65535`, `_output_power <= 2200`, `_drive_status <= 0xFFFF`) — a bad/garbled Modbus frame is silently replaced with the previous good value rather than surfaced as an error. `get_output_power()`/`get_status()` return `nullptr` if their `*_cmd` string is empty, so a model that doesn't define one doesn't send a bogus 1-byte Modbus frame every poll.
- **`DeltaMS300`** ([DeltaMS300.h](FluidNC/src/Spindles/VFD/DeltaMS300.h)/[.cpp](FluidNC/src/Spindles/VFD/DeltaMS300.cpp)): a one-line subclass of `GenericProtocol` whose constructor presets `_model = "DeltaMS300"`, registered with `SpindleFactory` as `"DeltaMS300"`. This lets the YAML config use a plain `DeltaMS300:` section (no `model:` key) — matching upstream's naming — while still reading its command strings from the same `VFDtypes[]` table `GenericProtocol` uses, so there's exactly one place those strings live. The older `ModbusVFD: {model: DeltaMS300, ...}` YAML form still works unchanged.
- **`VFDProtocol`** ([VFDProtocol.h](FluidNC/src/Spindles/VFD/VFDProtocol.h)/[.cpp](FluidNC/src/Spindles/VFD/VFDProtocol.cpp)): `actionGetPower` (`VFDactionType`) is used to poll output power roughly every 500ms from `vfd_cmd_task()`'s idle branch, going through the same transaction pipeline as every other command (retries, response CRC check, `unresponsive`/`mc_critical` handling) — there used to be a second, hand-rolled UART transaction here that duplicated that pipeline without its safety checks; it's gone. `get_output_power_value()`/`get_drive_status_value()` are on the base class (default `0`) so `VFDSpindle`/`Report.cpp` can read them without downcasting. `vfd_speed_queue` (a `static QueueHandle_t`, alongside `vfd_cmd_queue`) is filled with every real `_sync_dev_speed` reading right after a successful parse — see "Spindle / VFD stack" above for how `VFDSpindle::setState()` consumes it.
- **`VFDSpindle`** ([VFDSpindle.h](FluidNC/src/Spindles/VFDSpindle.h)/[.cpp](FluidNC/src/Spindles/VFDSpindle.cpp)): `output_power()` forwards to the protocol. `setState()`'s speed-sync wait (for protocols with `use_delay_settings() == false`) blocks on `vfd_speed_queue` with a hard 3s timeout instead of the old `delay_ms`-based polling loop. That old loop also had a `drive_status`-based "still ramping" gate (`get_drive_status_value()`) that reset its `unchanged` stall-counter every single iteration — since nothing ever called `get_status()` to populate `_drive_status`, the gate was permanently stuck "still ramping", which made the counter-based timeout unreachable and could hang `setState()`'s caller indefinitely. Separately, `get_rpm_cmd`'s scale used to be `*60/100`, which was not the inverse of `set_rpm_cmd` (the RPM domain the two used didn't match by a factor of 2), so `_sync_dev_speed` could never land inside `dev_speed`'s tolerance window either — that mismatch, now fixed by making the two scales exact inverses (see above), is the actual root cause of the original hangs, more so than the drive-status gate. That whole mechanism (the gate, the counter, and two "force a power read via an all-zero fake Modbus response" blocks that never actually updated `_output_power` — `GenericProtocol::parser()` rejects the fake buffer's first byte before reaching the `power` token) is gone, replaced by the queue+timeout wait described above. `get_status()`/`get_drive_status_value()`/register 2101H are still implemented and available (e.g. for a future `|S:` status field or the OLED) but **intentionally not wired to anything** — nothing polls them today.
- **`Spindle`** base class ([Spindle.h](FluidNC/src/Spindles/Spindle.h)): `output_power()` virtual (default `0`) so any spindle type can report power.
- **`Report.cpp`**: the realtime status line gained a `|P:<power>` field (grep `report_realtime_status`) whenever the active spindle is non-null. **The OLED does not currently display this field** — `OLED::parse_status_report()` doesn't parse a `P:` tag, so wiring VFD power into the OLED display is still open work if wanted.

When touching VFD speed-sync or power/status polling, there is now a single mechanism for each: `vfd_speed_queue` (speed sync, hard 3s timeout) and the rate-limited `actionGetPower` poll (power, ~500ms) — both go through `vfd_cmd_task`'s normal transaction pipeline. If you're adding a new Modbus-register-based VFD model, add a `VFDtype` entry to `GenericProtocol.cpp`'s `VFDtypes[]` and, optionally, a one-line subclass like `DeltaMS300` if you want a dedicated YAML section name — no new C++ parsing logic should be needed.
