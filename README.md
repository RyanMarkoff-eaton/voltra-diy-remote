# Voltra DIY Remote

Standalone firmware for the Elecrow ESP32-S3 2.1-inch 480×480 rotary touch
screen. It connects directly to a VOLTRA I over Bluetooth Low Energy; it does
not require Wi-Fi, a phone, a Raspberry Pi, or a Flipper.

> [!WARNING]
> This is an unofficial, hardware-dependent prototype. It can change resistance.
> Keep the Voltra controls accessible, begin unloaded or at low resistance, and
> stop using it if device behavior differs from the screen. BLE write success is
> not proof that Voltra applied a motor state.

## Safety, warranty, and support

This project is an independent community project. It is not made by, endorsed
by, or supported by Beyond Power. Use it only if you understand the risks of
controlling exercise resistance with experimental firmware. You accept
responsibility for assembly, configuration, operation, and any resulting
damage, injury, warranty impact, or data loss. This repository provides no
warranty or guarantee of fitness for a particular purpose.

Test every change with the cable unloaded and at a low resistance before using
it for training. Do not rely on the screen as the source of truth for an active
load; keep the Voltra controls within reach and stop immediately if the actual
device state differs from the remote.

If the remote does not work with your hardware or Voltra firmware, please
[fork this repository](https://github.com/RyanMarkoff-eaton/voltra-diy-remote/fork)
and make the changes in your own copy. Forking keeps your hardware-specific
settings and experiments separate, and makes it easy to propose a tested fix
back to this project through a pull request.

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
  values. **Inverse Chains is currently known not to apply reliably on the
  Voltra**; it is refreshed by a read-only settings query while idle and should
  be treated as unavailable until it is validated on your firmware.
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

## Usage

1. Turn on the Voltra, place it in **Weight Training** mode, and make sure it
   is not actively loading resistance.
2. Power the remote over USB. It scans for the address in `LocalConfig.h`; wait
   for `BLE CONNECTED`. If it remains offline, confirm the address, that the
   Voltra is awake and nearby, and that another app is not holding its BLE
   connection.
3. Turn the encoder to choose a requested base weight. The screen shows the
   requested value separately from settings read back from the Voltra. Turning
   the encoder alone does not engage resistance.
4. Tap the large weight number once to request the normal activation sequence.
   Tap it again, or tap **STOP**, to unload. Always confirm the actual Voltra
   response before beginning an exercise.
5. Hold the large weight number for one second only when deliberately testing
   the experimental guided-load behavior. A successful BLE write does not prove
   that the Voltra accepted or reached that motor state.

The remote never sends an activation, resistance, GO, or load command merely
because it boots or reconnects.

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
