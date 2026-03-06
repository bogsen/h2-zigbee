# Aqara H2 Wall Switch — Custom Zigbee 3.0 Firmware

Custom firmware for the Aqara H2 wall switch platform (EFR32MG24A410F1536IM48), replacing the stock Aqara firmware with a fully open Zigbee 3.0 implementation built on the Silicon Labs GSDK.

## Project Goals

- Mains-powered Zigbee 3.0 **router** with mesh routing capability
- **On/Off relay control** via Zigbee commands and local buttons
- **Coupled mode** (default): buttons control local relay directly
- **Decoupled mode**: buttons send On/Off/Level commands to bound lights, relay controlled independently via coordinator
- **Dual-color LED feedback** (red + blue per button) with smooth PWM transitions
- **Manufacturer-specific cluster 0xFCC0** (Aqara Opple) for configuration attributes compatible with Z2M
- **OTA firmware updates**
- Scalable from 1-gang to 2-gang by adding a second endpoint

## Hardware

- **MCU**: EFR32MG24A410F1536IM48 (Cortex-M33, 1536KB Flash, 256KB RAM)
- **Relay**: Secondary power module controlled via UART (115200 baud, CRC-16/MODBUS framing)
- **Buttons**: 2 per gang (up/down), active-low with debounce
- **LEDs**: 2 per gang (up/down), each dual-color R+B controlled via TIMER PWM
  - Top LED: TIMER0 CC0 (red, PD4) + CC1 (blue, PD5)
  - Bottom LED: TIMER1 CC0 (red, PD2) + CC1 (blue, PD3)
  - Active-low, 8-bit PWM resolution, ~2.4kHz

## Endpoint Architecture (1-Gang)

| Endpoint | Device Type | Purpose |
|----------|-------------|---------|
| 1 | HA On/Off Output (0x0002) | Relay + button config + LED config |
| 242 | Green Power Proxy (0x0061) | Required for Zigbee 3.0 routers |

### EP1 Server Clusters

- **Basic** (0x0000) — Device identification
- **Identify** (0x0003) — LED blink on identify
- **Groups** (0x0004) — Group membership
- **Scenes** (0x0005) — Scene recall
- **On/Off** (0x0006) — Relay state with StartUpOnOff

### EP1 Client Clusters

- **On/Off** (0x0006) — Send to bound lights (decoupled mode)
- **Level Control** (0x0008) — Brightness control to bound lights
- **Identify** (0x0003) — Find-and-bind initiator
- **Groups** (0x0004) — Group management on targets
- **OTA Upgrade** (0x0019) — Firmware updates

## Button Behavior

### Coupled Mode (default)
- **Down press** — Relay ON
- **Up press** — Relay OFF
- **Long press** — Same + reported to coordinator
- Works even without network

### Decoupled Mode (Low-Latency Dimming)

Light dimming starts immediately on button press (~10ms latency), before the press type is classified.

| Event | Down (brighten) | Up (dim) |
|-------|----------------|----------|
| **Press** (immediate) | Move up slowly + turn on | Move down slowly |
| **Quick tap** (<300ms) | Jump to max brightness | Jump to off |
| **Long hold** (>300ms) | No-op (slow move continues) | No-op (slow move continues) |
| **Release** (from hold) | Stop at current level | Stop at current level |
| **Double press** | Stop (cancel) | Stop (cancel) |

- Direct ZCL On/Off commands (to EP1) still control the local relay
- LED follows bound light state (On/Off reports + optimistic updates)
- LED syncs to bound light state on boot via Read Attributes

### Commissioning
- **Long press** (when not joined) — Start network steering

## LED Modes

Each button has an R+B dual-color LED. Color value 0=red, 100=blue.

| Mode | Description |
|------|-------------|
| Follow Relay (default) | ON=on_color (red), OFF=off_color (blue) |
| Follow Bound Light | Track On/Off reports from bound light |
| Manual | Coordinator controls color + brightness directly |
| Time Gradient | R-to-B interpolation based on time of day (planned) |

## Current Status

### Working
- [x] Zigbee 3.0 router (joins network, routes traffic)
- [x] On/Off cluster server — toggle from Z2M/ZHA
- [x] Button state machine — press/single/double/long detection with immediate press event
- [x] Coupled mode — buttons control relay + sync to On/Off cluster
- [x] Decoupled mode — low-latency dimming via Move/Stop/MoveToLevel to bindings
- [x] LED PWM driver — dual-color R+B with smooth transitions
- [x] LED follow-relay mode — color changes on On/Off state
- [x] Network steering — long press to join
- [x] Find-and-bind initiator — auto-bind after joining
- [x] Green Power proxy
- [x] Attribute reporting (On/Off state)

### In Progress / TODO
- [x] UART relay protocol — CRC-16/MODBUS framing to secondary power module
- [x] Manufacturer-specific cluster 0xFCC0 (Aqara Opple) — read/write attributes
- [x] Decoupled mode — buttons send to bindings, relay via coordinator
- [x] NVM-backed power-on behavior (on/previous/off/inverted)
- [x] Button action/side reporting via genMultistateInput
- [x] Factory reset (long press both buttons 10s)
- [x] OTA firmware updates (LZMA-compressed, ~15 min as router, instant start on boot)
- [x] LED follow-bound-light mode (decoupled: auto, coupled: follow relay)
- [x] LED sync to bound light state on boot (Read Attributes)
- [ ] LED time gradient mode
- [ ] 2-gang support (add EP2)
- [ ] No-neutral mode (maintain minimum current for MCU power)
- [ ] Power/energy monitoring (electrical measurement cluster)
- [ ] Temperature sensor (device temperature cluster)
- [ ] Dimmer switch variant (level control server + TRIAC/PWM dimming)

## Building & Flashing

Requires [Simplicity Studio 6](https://www.silabs.com/developer-tools/simplicity-studio) with Simplicity SDK 2025.6.2.

### Initial flashing via SWD

For first-time programming (or to recover a locked device), use the [SWD flash tool](swd_flasher/) with any SWD probe (Pico with debugprobe, J-Link, ST-Link, etc.). This performs a DCI device erase to clear the debug lock, then programs the bootloader + application via the exposed 5-pin header on the back of the board.

**Warning:** The DCI device erase permanently wipes the stock Aqara firmware. There is no way to recover it — the device will only run this custom firmware going forward.

```bash
cd swd_flasher
./flash.sh
```

### Subsequent updates via OTA

Once the firmware is running, updates can be pushed over-the-air via Zigbee OTA (LZMA-compressed, ~15 min as router). See `create_ota.sh` for building OTA images.

## Files

| File | Purpose |
|------|---------|
| `app.c` | Main application — button dispatch, relay sync, LED mode, commissioning |
| `h2_button.c/h` | Button state machine (press/single/double/long/release detection) |
| `h2_led.c/h` | LED PWM driver with smooth color transitions |
| `h2_led_config.h` | LED pin and timer assignments |
| `h2_opple.c/h` | Aqara Opple cluster 0xFCC0 — config attributes, NVM storage, LED control |
| `h2_relay.c/h` | Relay control via UART to secondary power module |
| `config/zcl/zcl_config.zap` | ZCL cluster/endpoint configuration |
| `h2_switch_1g.slcp` | Project component and configuration |
