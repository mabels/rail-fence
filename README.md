# RailFence

**RailFence** is open-source firmware for a motorized parallel fence (guide rail) for a saw table.
The fence rides on HONGDUI H040 T-slot rail guides and is driven by two synchronized NEMA stepper motors,
one at each end of the fence, controlled by ESP32-S3 microcontrollers communicating over ESP-NOW.

---

## Features

- **Synchronized dual-axis motion** — Leader/follower architecture over ESP-NOW keeps both ends of the fence in lock-step
- **High-resolution positioning** — 800 steps/mm (0.1 mm, 1 mm, 10 mm step modes selectable)
- **OLED display** — 128×64 SH1106 screen shows current position and a scrollable menu
- **Rotary encoder** — Physical dial for precise positioning with debounced ISR
- **Locked / unlocked mode** — Locked mode clamps travel to 0–190 mm; unlocked removes limits
- **Web UI** — Responsive single-page app for remote control, position entry, lock toggle, and OTA reset
- **ArduinoOTA** — Wireless firmware updates; no USB cable needed after initial flash
- **NVS persistence** — Role (leader/follower) and follower MAC address survive power cycles
- **Boot diagnostics** — RGB LED cycles through colour stages so you can pinpoint startup failures

---

## Hardware

| Component | Part |
|-----------|------|
| MCU | ESP32-S3 N16R8 (16 MB flash, 8 MB PSRAM octal) |
| Stepper driver | TMC2209 (UART + SPREAD pin) |
| Motor | NEMA 17 or NEMA 23 (adjust current in firmware) |
| Display | SH1106 128×64 OLED (I²C) |
| Encoder | Incremental rotary encoder with push switch |
| Rail guide | HONGDUI H040 T-slot (190 mm travel) |

### Pin Assignments

| Signal | GPIO |
|--------|------|
| STEP | 13 |
| DIR | 14 |
| ENABLE (active-LOW) | 10 |
| TMC2209 UART TX | 9 |
| TMC2209 UART RX | 12 |
| TMC2209 SPREAD | 11 |
| OLED SDA | 42 |
| OLED SCL | 41 |
| Encoder CLK | 4 |
| Encoder DT | 5 |
| Encoder SW | 6 |

---

## Project Structure

```
rail-fence/
├── platformio.ini          # PlatformIO build config (two targets: USB + OTA)
├── src/
│   ├── main.cpp            # All firmware (leader + follower roles unified)
│   ├── MoveCmd.h           # ESP-NOW wire protocol structs
│   ├── Config.h            # ← NOT committed (WiFi credentials, OTA password)
│   └── Config.h.example    # Safe template — copy to Config.h and fill in
└── .gitignore
```

---

## Getting Started

### 1. Prerequisites

- [PlatformIO](https://platformio.org/) (VSCode extension or CLI)
- Python 3.11–3.13 (3.14 is too new for the platform builder)
- Two ESP32-S3 N16R8 boards

### 2. Clone and configure

```bash
git clone https://github.com/mabels/rail-fence.git
cd rail-fence
cp src/Config.h.example src/Config.h
# Edit src/Config.h with your WiFi SSID, password, and OTA password
```

### 3. Create a Python venv (required for PlatformIO on Python 3.13+)

```bash
python3 -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install esptool --break-system-packages 2>/dev/null || pip install esptool
```

### 4. Flash via USB (first time)

```bash
pio run -e slider_usb -t upload
```

### 5. Flash via OTA (subsequent updates)

```bash
pio run -e slider -t upload
```

The OTA target points to `slider-c58aa4.local` — update `upload_port` in `platformio.ini` to match your board's mDNS hostname.

---

## Role Assignment

On first boot both devices come up as **leader**. To designate a follower:

1. Open the web UI (`http://<ip>/`)
2. Navigate to **Menu → Set Role → Follower**
3. Reboot — the device will now await ESP-NOW commands from the leader

The leader automatically discovers the follower's MAC address during the pairing handshake and stores it in NVS.

---

## Web UI

| Control | Description |
|---------|-------------|
| Position display | Shows current and target position in mm |
| Left / Right buttons | Move fence in 0.1 mm steps |
| Step selector | Switch between 0.1 mm, 1 mm, 10 mm per click |
| Go-to field | Type an exact position and press Go |
| Lock toggle | Enable/disable travel limits (0–190 mm) |
| Reset button | Hold 1.5 s to reboot (prevents accidental press) |

---

## Calibration

Default calibration assumes **800 steps/mm** (8 µsteps × 200 steps/rev, 5 mm/rev lead screw).
Adjust `STEPS_PER_MM` in `main.cpp` if your mechanical setup differs.

---

## ESP-NOW Protocol

```
Leader → Follower:  MoveCmd  { steps, speed_hz, seq }
Follower → Leader:  MoveAck  { seq, position, success }
```

Both structs are defined in `src/MoveCmd.h`.

---

## License

Apache 2.0 — see [LICENSE](LICENSE).
