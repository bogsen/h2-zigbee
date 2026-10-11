# Aqara H2 Switch & Dimmer — Custom Zigbee Firmware

Custom firmware for the Aqara H2 switch and dimmer (EFR32MG24A410F1536IM48), built with Silicon Labs Simplicity SDK 2025.6.2.

## Features

- Mains-powered Zigbee routers with OTA updates.
- One- and two-gang switches, with gang count detected from the wall module at boot and a stored fallback.
- Dimmer with local On/Off and Level Control, configurable brightness limits and phase mode.
- Coupled mode (default) for local control; decoupled mode for controlling bound lights while the local output remains independently controllable over Zigbee.
- Persistent configuration and power-on behavior: on, previous, off, or inverted. Both variants restore their outputs at startup without waiting for a network; reconnecting does not reapply the setting.
- The dimmer remembers its local brightness across reboots, clamped to the configured limits. Brightness is saved after 500 ms without a level change; power loss during that interval can retain the older level.
- Red/blue LED feedback for local output or bound-light state.
- Current, active power and accumulated energy reporting; voltage reporting on the dimmer. Both variants report wall-module temperature with zero calibration offset.
- Aqara Opple configuration and action reports compatible with the existing Zigbee2MQTT converter; no custom converter is required. Compatibility has been checked through manual use, not a complete test of every stock setting.

The switch has two buttons and two bicolor LEDs per gang; the dimmer has a rotary push encoder and one bicolor LED. Both communicate with the secondary wall module over 115200-baud UART. Pin and timer assignments live in each variant's `h2_variant.h` and `config/` directory.

## Controls

### Switch

In coupled mode, pressing down turns the corresponding relay on; pressing up turns it off. Local control works without a Zigbee network.

In decoupled mode, button handling runs every 10 ms and starts dimming on press:

| Event | Down | Up |
|-------|------|----|
| Press | Move brightness up, turning the light on | Move brightness down |
| Quick tap (under 300 ms) | Move to full brightness with a 0.5 s transition | Move to off with a 0.5 s transition |
| Hold | Continue moving | Continue moving |
| Release after a hold | Stop | Stop |
| Double press | Stop; the first tap may already have acted | Stop; the first tap may already have acted |

### Dimmer

| Input | Coupled | Decoupled |
|-------|---------|-----------|
| Short press without rotation | Toggle local output | Toggle bound lights between off and full brightness with a 0.5 s transition |
| Rotate | Adjust local brightness within configured limits | Adjust bound-light brightness |
| Hold and turn (after 200 ms) | No color-temperature action | Adjust bound-light color temperature |

### Joining and factory reset

Allow joining on the coordinator first. On an unjoined switch, a long press starts network steering. On an unjoined dimmer, hold the knob still for five seconds to start steering; a short press also starts it in decoupled mode.

To factory-reset a switch, hold both buttons of the first physical pair for ten seconds (the right pair on two-gang hardware). On a joined dimmer, hold the knob without turning: the reset warning starts after five seconds, then keep holding for another ten seconds. Releasing or rotating cancels the dimmer reset. Factory reset clears saved settings, bindings and the accumulated energy total, leaves the network, and reboots.

### LED feedback

The normal indicator mode follows the local output in coupled mode and bound-light state in decoupled mode. Bound-light state is updated from reports, read after the network comes up, and updated optimistically after local commands. The Aqara indicator settings select behavior, swap the switch indicator position, and alter the red/blue color assignment.

The driver uses PWM with smooth transitions. Arbitrary remote color/brightness control and time-of-day gradients are not implemented as user-facing features.

## Endpoints and binding

| Endpoint | Switch | Dimmer |
|----------|--------|--------|
| 1 | Local relay 1, configuration and telemetry | Local output and level, configuration and telemetry |
| 2 | Local relay 2 on two-gang hardware | — |
| 4 / 5 | Button action endpoints | Action endpoint 4 |
| 11 | Bound-light control for gang 1 | Bound-light control for the knob |
| 12 | Bound-light control for gang 2 | — |
| 242 | Green Power proxy | Green Power proxy |

Local output endpoints have On/Off servers; the dimmer also has a Level Control server. OTA is on endpoint 1. On/Off and Level Control clients are on endpoints 11/12, with a Color Control client on dimmer endpoint 11. Control endpoints include Identify and Groups for commissioning.

Bind lights to the **control endpoints (11/12)**. Decoupled commands and light-state queries originate there. Direct reports and read responses must target the corresponding control endpoint; group feedback is matched against that endpoint's bindings. This prevents local output reports from reaching bulb-control bindings, but does not authenticate group-report senders as bulbs.

Find-and-bind starts from endpoint 11 after joining; configure gang 2's bindings explicitly. Stock Aqara model identifiers and local output/action endpoints are retained for Zigbee2MQTT.

## Building

Use Simplicity Studio 6 with Simplicity SDK **2025.6.2**, the ARM GCC toolchain, CMake, Ninja and Simplicity Commander.

Import and generate the relevant `.slcp` project in Studio/SLC before building a fresh checkout. SDK files, generated configuration and CMake presets contain local paths and are not all tracked. Preserve the custom sources and OTA post-build commands in the tracked `cmake_gcc/CMakeLists.txt` when regenerating.

After generation, configure and build the selected project:

```bash
cd h2_switch/cmake_gcc   # or h2_dimmer/cmake_gcc or bootloader/cmake_gcc
cmake --preset project
cmake --build --preset default_config
```

For an already configured checkout, from the repository root:

```bash
cmake --build h2_switch/cmake_gcc/build --config base
cmake --build h2_dimmer/cmake_gcc/build --config base
```

Application builds produce `.bin`, `.hex`, `.s37`, `.gbl` and `.ota` files in the selected project's `cmake_gcc/build/base/` directory. The [shared post-build script](common/create_ota.cmake) uses Commander to compress the GBL with LZMA and package the Zigbee OTA image. Set `POST_BUILD_EXE` to the Commander executable if it cannot be found automatically.

Firmware versions come from `H2_FIRMWARE_VERSION` in each variant's `app.c`; follow [the version policy](AGENTS.md) when changing firmware. OTA images use manufacturer `0x115F`, image type `0xEDCF` for the switch and `0x7217` for the dimmer.

## Flashing and updates

### Initial SWD installation

Use the [SWD flash tool and wiring instructions](swd_flasher/README.md). Disconnect the wall module before powering the logic board from a probe. Select the application explicitly; for example, from the repository root:

```bash
./swd_flasher/flash.sh \
  --bootloader bootloader/cmake_gcc/build/base/bootloader.bin \
  --application h2_dimmer/cmake_gcc/build/base/h2_dimmer.bin
```

For a switch, use `h2_switch/cmake_gcc/build/base/h2_switch.bin` instead. Build the [bootloader](bootloader/readme.md) first if its binary is unavailable.

The tool attempts a full flash backup, asks for confirmation, then performs a **DCI device erase on both locked and unlocked devices** before programming and verifying. This erases the installed firmware and settings. A debug-locked device cannot be backed up through this procedure; a failed backup does not prevent the erase confirmation. Keep a verified stock dump if available; restoring stock firmware is not a validated workflow here.

### OTA updates

Once custom firmware and its bootloader are installed, serve the matching variant's `.ota` file through the coordinator's OTA service. The OTA client starts when the network comes up. Transfer duration depends on the network; there is no fixed completion time.

## Validation and limitations

Run the host checks from the repository root (Python 3 and a host C compiler are required):

```bash
python3 tests/check_endpoints.py
python3 tests/check_power_on.py
python3 tests/check_measurements.py
```

These check endpoint routing, startup restoration and measurement handling without hardware. Dimmer voltage/current/power readings have been reported as plausible in live use; calibrated accuracy, timed energy accumulation and switch telemetry still need hardware validation. Energy is checkpointed every five minutes, so an abrupt power loss can lose the unsaved interval.

No-neutral power management and LED time gradients are not implemented. Host tests and successful builds do not replace hardware testing of each variant.

## Source layout

| Path | Purpose |
|------|---------|
| `h2_switch/app.c`, `h2_dimmer/app.c` | Variant controls, startup, commissioning and Zigbee callbacks |
| `h2_switch/h2_variant.h`, `h2_dimmer/h2_variant.h` | Variant capabilities and hardware assignments |
| `common/h2_button.*`, `common/h2_encoder.*` | Button and rotary encoder handling |
| `common/h2_led.*` | LED PWM and feedback |
| `common/h2_opple.*` | Aqara configuration, action reporting and persistence |
| `common/h2_relay.*` | Wall-module UART protocol and output control |
| `common/h2_power.*`, `common/h2_temperature.*` | Telemetry |
| `h2_switch/config/zcl/`, `h2_dimmer/config/zcl/` | Endpoint and cluster definitions |
| `h2_switch/h2_switch.slcp`, `h2_dimmer/h2_dimmer.slcp` | SDK project definitions |
| `bootloader/`, `swd_flasher/`, `tests/` | Bootloader, initial installation and host checks |
