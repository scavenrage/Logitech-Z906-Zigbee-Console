# Logitech Z906 — Zigbee replacement console (ESP32-H2)

A replacement for the original Logitech Z906 control console, built on an **ESP32-H2**.
It talks to the Z906 amplifier over the console DB-15 serial line and exposes the whole
system to **Home Assistant (ZHA)** over Zigbee. It also keeps the original IR remote
working (volume and mute) and shows status, input and volume on a WS2812B LED bar.

![Schematic](hardware/Schematic.png)

## Features

- **Power on/off** of the amplifier, with the real state reported to Home Assistant
  (also after a power loss of the console).
- **Main volume** (0–43), **input** (1–6), **rear / centre / subwoofer levels** (0–43).
- **3D effect** switch for input 3 (optical), stored in flash.
- **IR remote** (original Z906 remote, NEC): Vol+, Vol− (single press and hold) and Mute.
  Frames from other remotes (e.g. a TV remote) are ignored.
- **LED bar**: 13 × WS2812B-2020 — console status, selected input, 11-LED volume bar.
- **State restored after power loss**: input, levels and 3D setting are saved in NVS and
  re-applied when the amplifier is switched on (the amplifier itself only saves its settings
  on a regular power-off command).
- Logs on **USB Serial/JTAG** only — the UART is reserved for the amplifier.

## Repository layout

| Path | Content |
|---|---|
| `firmware/` | ESP-IDF project (ESP32-H2) |
| `firmware/main/` | `main.c` (amplifier protocol, IR, LEDs, Zigbee), `ota.c/.h` (version, rollback) |
| `hardware/` | EasyEDA Pro schematic and PCB (`.epro2`), schematic image, photos |
| `hardware/Enclosure/` | Enclosure (SketchUp `.skp` and `.stl`) |

## Hardware

| GPIO | Function |
|---|---|
| GPIO0 | WS2812B data (LED1–LED13 chain) |
| GPIO1 | MOSFET Q1 → amplifier pin 15 (power control) |
| GPIO4 | UART RX ← amplifier pin 12 |
| GPIO5 | UART TX → amplifier pin 13 |
| GPIO9 | Factory reset button (hold 5 s) / boot strap |
| GPIO10 | IR receiver VS1838B (NEC) |
| GPIO11 | Amplifier pin 8 (status input) |
| GPIO26/27 | USB D−/D+ (logs, flashing) |

Amplifier connector: pin 6 = GND, pin 8 = GPIO11, pin 12 = GPIO4, pin 13 = GPIO5,
pin 15 = GPIO1 through the MOSFET.

LED chain: LED1 = console status, LED2 = selected input, LED3–LED13 = volume bar
(11 LEDs, 4 brightness steps each → 44 steps for volume 0–43).

## Zigbee endpoints (ZHA)

Manufacturer `Handmade!`, model `Z906 Console`, router, channel 25.

| EP | Cluster | Entity | Notes |
|---|---|---|---|
| 1 | On/Off | Power | Real amplifier state, reported after every reboot |
| 2 | Analog Output | Volume | 0–43 |
| 3 | Analog Output | Input | 1–6 |
| 4 | Analog Output | Rear level | 0–43 |
| 5 | Analog Output | Centre level | 0–43 |
| 6 | Analog Output | Subwoofer level | 0–43 |
| 7 | On/Off | 3D effect | Input 3 only, saved in NVS |

The firmware version is in the Basic cluster, attribute `sw_build_id` (0x4000).

## Amplifier protocol — what matters

Serial 57600 baud, 8 data bits, **odd parity**, 1 stop bit. The protocol reference is
[nomis/logitech-z906](https://github.com/nomis/logitech-z906/blob/main/protocol.rst);
command codes were cross-checked with [zarpli/Logitech-Z906](https://github.com/zarpli/Logitech-Z906).

- Single-byte commands are echoed; long messages are `AA <type> <len> <data> <checksum>`
  (sum from type to checksum = 0). Received messages are checksum-verified.
- **One request at a time**: the amplifier cannot handle new requests while it is sending a
  long reply (nomis, "Message formats"). All writes go through one serialized function that
  waits for the end of long replies.
- Volume and levels are changed with single-step commands (`08/09` volume, `0A/0B` sub,
  `0C/0D` centre, `0E/0F` rear). **Verified on the amplifier: steps above 43 are ignored.**
- Full status write uses the same format as the status read (`AA 0A 14` + 20 bytes);
  out-of-range values are replaced before writing.
- Mute works like the original console: volume to 0 with `09`, restore with `38`, `08`…, `39`.
- Effects: `14` = 3D, `35` = disabled. Verified: 3D also applies with a 2.1 signal
  (decode state `1B`), not only with 2.0.

## Building

- **ESP-IDF v5.5.6**, target `esp32h2`.
- **esp-zigbee-lib 2.0.5** (no ZBOSS) through its 1.x compatibility layer
  (`CONFIG_ZB_SDK_1xx=y`), plus `espressif/led_strip` 2.5.x. Components are downloaded by the
  IDF Component Manager.

From an ESP-IDF terminal, in `firmware/`:

```
idf.py set-target esp32h2
idf.py -p COMx build flash monitor
```

`sdkconfig` is generated from `sdkconfig.defaults`. Updates are done over USB: Zigbee OTA is not
available in the esp-zigbee-lib 2.0.5 compatibility layer.

Factory reset (leave the Zigbee network): hold the button on GPIO9 for 5 seconds.

## Lessons learned

- **Never share the amplifier UART with the console UART.** On ESP32-H2, ROM/ISR prints
  (`esp_rom_printf`, `ESP_DRAM_LOGx`) go to UART0 even when the console is on USB Serial/JTAG
  (ESP32-H2-MINI-1 datasheet, ROM messages printing). With the amplifier on UART0, driver error
  messages were sent to the amplifier as garbage and the audio dropped. The amplifier is on
  **UART1** (same GPIO4/5 through the GPIO matrix).
- **IR receiver buffer**: other remotes send frames longer than 64 RMT symbols; the user buffer is
  512 symbols so they are received whole and discarded silently.
- **ZBOSS (esp-zigbee-lib 1.6.x)**: the neighbour table reported LQI = 255 towards every
  neighbour, attracting routes through this device. Moving to esp-zigbee-lib 2.0.5 (no ZBOSS);
  Zigbee credentials are migrated on first boot with `esp_zigbee_nvram_migrate()`, so the device
  stays paired.
- **Corrupted status**: before checksum verification and serialized writes, a corrupted status
  was written back to the amplifier (rear level 170, invalid effect values).

## Version history

- **v3** — esp-zigbee-lib 2.0.5 (no ZBOSS), ESP-IDF 5.5.6, serialized amplifier writes with
  checksum check, rear/centre/subwoofer levels, 3D switch, state saved in NVS.
- **v2.0** — first version for this PCB (console replacement, IR, LED bar).

## Credits

- [nomis/logitech-z906](https://github.com/nomis/logitech-z906) — protocol documentation.
- [zarpli/Logitech-Z906](https://github.com/zarpli/Logitech-Z906) — Arduino library, command codes.

## License

MIT — see [LICENSE](LICENSE).
