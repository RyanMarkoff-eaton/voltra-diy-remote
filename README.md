# Voltra DIY Remote

Standalone firmware for the Elecrow ESP32-S3 2.1-inch 480×480 rotary touch
screen. It connects directly to a VOLTRA I over Bluetooth Low Energy; it does
not require Wi-Fi, a phone, a Raspberry Pi, or a Flipper.

> [!WARNING]
> This is an unofficial, hardware-dependent prototype. It can change resistance.
> Keep the Voltra controls accessible, begin unloaded or at low resistance, and
> stop using it if device behavior differs from the screen. BLE write success is
> not proof that Voltra applied a motor state.

## Hardware

The display implementation targets the PCB marked **ESP32 Display 2.1 V1.0**:

- ESP32-S3 with 16 MB flash and 8 MB PSRAM
- 480×480 round RGB display and CST826 touch controller
- rotary encoder surrounding the display
- USB 5 V input

Other Elecrow rotary display revisions use different panels and pins. Do not
flash the `elecrow21_ui` environment until you have confirmed this board.

## Features

- Direct BLE discovery, handshake, automatic reconnect, and current-settings
  reads. Connection and reconnect never send a resistance, GO, or load command.
- 5–230 lb base-weight control in 1 lb encoder detents. Values above 200 lb are
  an unverified beta extension and require a Voltra that supports them.
- Mode-gated writes: controller changes are sent only after a fresh Voltra
  response reports Weight Training mode.
- Touch activation, long-hold experimental guided load, and a dedicated STOP
  touch target.
- Eccentric, Chains, and Inverse Chains controls with colored rings and saved
  values. Inverse Chains remains device-dependent and is refreshed by a
  read-only settings query while idle.
- Drop-set screen: configurable drop amount and delay after the last detected
  cable return.
- Optional 15-minute auto sleep. It only enters deep sleep while the controller
  has no active/pending load action. Wake it with RESET or a power cycle.

## Setup

1. Install PlatformIO.
2. Copy `include/VoltraConfig.h.example` to `include/LocalConfig.h`.
3. Set `VOLTRA_BLE_ADDRESS` to your Voltra's BLE address. `LocalConfig.h` is
   ignored by Git so it is safe to keep private.
4. Build and flash the confirmed board:

```powershell
pio run -e elecrow21_ui
pio run -e elecrow21_ui -t upload
pio device monitor -b 115200
```

Use `esp32dev` or `esp32s3` only for development hardware. `elecrow21_ui` has a
fixed `COM6` upload port in `platformio.ini`; change it for your computer.

On boot, the remote scans for the configured Voltra, completes the BLE
handshake, and reads settings without applying weight. The bottom status changes
to `BLE CONNECTED` when ready.

## Controls

- **Rotate main screen:** adjust base weight.
- **Tap weight:** activate the selected Weight Training weight; tap again to
  send STOP/unload.
- **Hold weight for one second:** experimental guided load.
- **Tap a modifier row:** enable/disable it while preserving the selected value.
  Rotate while selected to edit its value.
- **Swipe right:** open Drop Sets. Tap `DROP` or `HOLD`, then rotate to change
  that value. Tap Auto Drop or Auto Sleep to toggle each option.
- **Tap STOP:** sends Voltra's STOP/unload packet.

## Power

The board accepts regulated **5 V** on its existing USB input. A USB power bank
rated for at least 1 A can power it after it is unplugged from the computer.
Do not connect a raw Li-ion/LiPo cell or 9/12 V supply directly to the board.

## Validation

Run packet and pinned-upstream checks from this directory:

```powershell
python tools/check_packets.py
python tools/check_upstream.py path\to\voltra-node-sdk
```

The packet checks validate the embedded protocol data and generated setting
packets. They do not validate motor behavior on your Voltra. See
[VALIDATION.md](VALIDATION.md) for the current hardware-validation limits.

## Attribution

Protocol data was ported from the public
[voltra-node-sdk](https://github.com/HJewkes/voltra-node-sdk) at the commit
recorded in `include/ProtocolData.h`. Its MIT license is retained in
`LICENSE-SDK`.