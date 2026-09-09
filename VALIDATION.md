# Validation status

## Automated checks

- `python tools/check_packets.py` validates all embedded packet checksums and
  packet shapes, generated weight and eccentric commands, and typed native
  Chain/Inverse Chain fixtures (variant uint8, direction uint8, amount uint32).
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
- The corrected Chain/Inverse Chain build reconnects and reads the inactive
  shared-chain state without writing a setting.

## Still experimental

- Guided load and automatic drop sets have packet-level validation but require
  controlled physical validation on each Voltra firmware version.
- A successful BLE write is not proof that Voltra changed its motor state.
- Chain and Inverse Chain direction/amount behavior uses a reviewed protocol
  model and has packet-level tests. It still needs a controlled low-percentage
  hardware test on each Voltra firmware version. A successful BLE write is not
  confirmation that the Voltra applied the selected direction or percentage.
- Auto sleep is ESP32 deep sleep, not a physical battery disconnect. It wakes
  with RESET or restored 5 V.
- The 201–230 lb range is a user-specific beta extension; public upstream
  packet vectors cover 5–200 lb only.
