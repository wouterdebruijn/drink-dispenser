# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Shot Machine 4.0 — an ESP32-based embedded firmware that reads UHF RFID tags to trigger a pump dispenser and reports usage via LoRaWAN (EU868). Hardware target is the LilyGO T-Display board.

## Build & Flash

Uses PlatformIO. Requires a `include/secrets.h` with LoRaWAN credentials (App EUI, Device EUI, App Key) — not committed to git.

```bash
pio run -e lilygo-t-display              # compile
pio run -t upload -e lilygo-t-display    # compile and flash
pio device monitor                       # serial monitor at 115200 baud
```

No linting or automated tests are configured.

## Architecture

The firmware is single-threaded, driven by a **TaskScheduler** with three non-blocking tasks defined in `src/main.cpp`:

| Task | Interval | Purpose |
|------|----------|---------|
| `rfidTask` | 250 ms | Poll RFID reader, update tag storage, trigger pump |
| `pumpOffTask` | 100 ms × 6 steps | Timed pump shutoff (600 ms dispense) |
| `displayTask` | 10 s | Refresh OLED display |

**Data flow:** RFID scan → `RfidReader` → `RfidStorage` (increment visit count + lockout) → pump callback → `Display` update. Separately, `loramac` transmits pending tag counts over LoRaWAN every 60 s, repeating each tag 3 times for redundancy.

### Key modules

- **`src/rfid/`** — `RfidReader` (serial protocol to UHF reader on Serial2, RX=13/TX=2) and `RfidStorage` (tag ID → visit count map with 30-read lockout to prevent duplicate dispenses).
- **`src/lora/loramac.cpp`** — LMIC-based LoRaWAN join + uplink. Reads credentials from `secrets.h`. Handles the LMIC event loop via `os_runloop_once()` called from `loop()`.
- **`src/lora/LoRaBoards`** — Hardware init: XPowersLib power management, SPI/I2C bus setup, pin mappings for the LilyGO T-Display.
- **`src/display/Display`** — U8G2 wrapper for SH1106 OLED (I2C 0x3C). Shows LoRaWAN join status, last scanned tag, drink count, and a 4-frame shot glass animation during dispensing.
- **`src/peripherals/Pump`** — GPIO 25 relay control via `digitalWrite`.

### Notable pin assignments

| Signal | GPIO |
|--------|------|
| RFID enable | 14 |
| RFID RX/TX | 13 / 2 |
| Pump relay | 25 |
| LoRa radio | SX1276 via SPI |
| OLED display | SH1106 via I2C |
