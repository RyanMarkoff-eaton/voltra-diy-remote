# Validation status

## Automated checks

- `python tools/check_packets.py` validates all embedded packet checksums and
  packet shapes, plus generated weight and modifier commands.
- `python tools/check_upstream.py <path-to-voltra-node-sdk>` validates the
  expected upstream commit, BLE identifiers, handshake packets, STOP/SETUP/GO,
  and generated packet data.
- `elecrow21_ui` compiles with PlatformIO, NimBLE-Arduino 2.3.6, and Arduino
  GFX Library 1.4.7.

## Hardware checks completed

- ESP32-S3 upload and reset on the Elecrow 2.1 V1.0 board.
- 480×480 display, CST826 touch, and rotary encoder initialization.
- BLE scan, Voltra connection, handshake, mode read, and base-weight read.
- No resistance command is sent on boot or reconnect.

## Still experimental

- Guided load and automatic drop sets have packet-level validation but require
  controlled physical validation on each Voltra firmware version.
- A successful BLE write is not proof that Voltra changed its motor state.
- Inverse Chain writes its direction selector as a typed one-byte setting and
  then uses the standard Chain value command. This path passed packet checks
  and was verified on the reference Voltra at a low setting. Voltra firmware
  and account feature availability may still vary; confirm the setting on the
  Voltra display before training.
- Auto sleep is ESP32 deep sleep, not a physical battery disconnect. It wakes
  with RESET or restored 5 V.
- The 201–230 lb range is a user-specific beta extension; public upstream
  packet vectors cover 5–200 lb only.