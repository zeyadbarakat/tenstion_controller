# ESP32-S3 Tension Control System

A production-ready cascaded PI tension control system for unwinding applications using ESP32-S3 and ESP-IDF v5.x.

![ESP32-S3](https://img.shields.io/badge/ESP32--S3-Supported-blue)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

## Features

- **Cascaded PI Control** — Outer tension loop, inner speed loop for smooth unwinding
- **Relay Feedback Auto-Tuning** — Åström-Hägglund method with No-Overshoot tuning rule (safe for material handling)
- **Real-Time WebSocket Dashboard** — 50Hz Chart.js graphs via WebSocket push, HTTP polling fallback
- **PCNT Quadrature Encoder** — Hardware 4x decoding, 600 PPR supported
- **HX711 Load Cell** — 24-bit ADC with NVS calibration storage
- **Safety Monitoring** — E-STOP, fault detection, state machine with watchdog
- **UART Terminal** — Menu-driven LCD interface via serial
- **Persistent Tuning** — PI gains saved to NVS, restored on boot automatically

---

## How It Works

The user sets a **tension setpoint** (e.g., 5.0 kg). The system automatically adjusts motor speed to maintain constant tension as the roll unwinds:

```
Full Roll → Slow RPM → Tension = 5.0 kg ✅
Half Roll → Medium RPM → Tension = 5.0 kg ✅
Empty Roll → Fast RPM → Tension = 5.0 kg ✅
```

The cascaded PI controller continuously adjusts the PWM output — no jerky start/stop behavior.

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| MCU | ESP32-S3-WROOM-1 |
| Motor | DC 80V 600W 2000RPM |
| Encoder | 600 PPR Quadrature |
| Load Cell | 10-50kg with HX711 |
| Motor Driver | H-Bridge (BTS7960 or similar) |
| Buttons | RUN, STOP, E-STOP (NC) |

---

## Wiring Diagram

```
                           ESP32-S3
                        ┌─────────────┐
                        │             │
   Encoder A ───────────┤ GPIO4       │
   Encoder B ───────────┤ GPIO5       │
                        │             │
   HX711 DATA ──────────┤ GPIO6       │
   HX711 CLK ───────────┤ GPIO7       │
                        │             │
   Motor PWM ───────────┤ GPIO15      ├───── H-Bridge IN1
   Motor DIR ───────────┤ GPIO16      ├───── H-Bridge IN2
                        │             │
   RUN Button ──┬───────┤ GPIO10      │      (NC to GND)
                │       │             │
   STOP Button ─┬───────┤ GPIO11      │      (NC to GND)
                │       │             │
   E-STOP ──────┬───────┤ GPIO12      │      (NC to GND)
                │       │             │
              10kΩ      │             │
              Pullup    └─────────────┘
```

### Pin Assignment Table

| Function | GPIO | Notes |
|----------|------|-------|
| Encoder A | 4 | PCNT capable |
| Encoder B | 5 | PCNT capable |
| HX711 Data | 6 | General purpose |
| HX711 Clock | 7 | General purpose |
| Motor PWM | 15 | LEDC, 25kHz |
| Motor Direction | 16 | HIGH/LOW |
| RUN Button | 10 | Internal pull-up |
| STOP Button | 11 | Internal pull-up |
| E-STOP | 12 | NC contacts, highest priority |

> **⚠️ Safety Note**: The E-STOP should also have an independent relay to cut motor power directly, regardless of software state.

---

## Building and Flashing

### Prerequisites

- [ESP-IDF v5.5](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/) installed
- ESP32-S3 board connected via USB

### Build Steps

```bash
# Clone the repository
git clone https://github.com/zeyadbarakat/tenstion_controller.git
cd tenstion_controller

# Set target (first time only)
idf.py set-target esp32s3

# Configure (optional)
idf.py menuconfig

# Build
idf.py build

# Flash and monitor (replace COM4 with your port)
idf.py -p COM4 flash monitor
```

### Configuration Options

Use `idf.py menuconfig` → **"Tension Controller Configuration"** to set:

- GPIO pin assignments
- Motor PWM frequency and max RPM
- Encoder PPR
- Control loop periods
- Safety limits
- WiFi credentials (SSID/password)
- Default PI gains

---

## Control Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    CASCADED PI CONTROL                       │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  Tension     ┌──────────┐     Speed      ┌──────────┐        │
│  Setpoint───►│ Tension  ├────Setpoint───►│  Speed   ├──►PWM  │
│              │    PI    │                │    PI    │        │
│          ┌──►│Controller│                │Controller│◄─┐     │
│          │   └──────────┘                └──────────┘  │     │
│          │                                             │     │
│       Tension                                       Speed    │
│       Feedback                                   Feedback    │
│          │                                             │     │
│   ┌──────┴───────┐                           ┌─────────┴┐    │
│   │   HX711      │                           │  PCNT    │    │
│   │  Load Cell   │                           │ Encoder  │    │
│   └──────────────┘                           └──────────┘    │
└──────────────────────────────────────────────────────────────┘
```

The user only sets **tension**. The tension PI controller automatically calculates the required motor speed (speed setpoint), and the speed PI controller drives the motor PWM. As the roll diameter decreases during unwinding, the motor automatically speeds up to maintain constant tension.

---

## Usage

### Web Interface

1. Connect to WiFi:
   - **AP Mode** (default): Connect to `TensionCTRL` (password: `tension123`)
   - **Station Mode**: Configure SSID/password in menuconfig

2. Open browser: `http://tension-ctrl.local` or `http://192.168.4.1`

3. Dashboard features:
   - Real-time tension, speed, PWM graphs (WebSocket @ 50Hz)
   - Start/Stop/E-Stop buttons
   - Tension setpoint adjustment
   - Auto-tune trigger (speed & tension loops)
   - PI gain configuration
   - Fault display

### UART Commands

Connect to USB serial at 115200 baud:

| Command | Description |
|---------|-------------|
| `R` | Run/Start system |
| `S` | Stop system |
| `T` | Start speed loop auto-tune |
| `C` | Tare load cell (zero calibration) |
| `C5.0` | Span calibration with 5kg weight |
| `+` | Increase tension setpoint by 0.5kg |
| `-` | Decrease tension setpoint by 0.5kg |

---

## Calibration & Auto-Tuning

### Load Cell Calibration

1. **Tare** (zero calibration):
   - Remove all load from sensor
   - Send `C` command or click "Tare" in web UI
   - Wait for "Tare complete" message

2. **Span** (scale calibration):
   - Apply known weight (e.g., 5.0 kg)
   - Send `C5.0` command (replace 5.0 with actual weight)
   - Calibration is automatically saved to NVS

### Auto-Tuning (Åström-Hägglund Relay Feedback)

The system uses the relay feedback method to automatically find optimal PI gains. **Always tune in this order:**

| Step | Action | What Happens |
|------|--------|--------------|
| 1 | **Tune Speed Loop** | Motor oscillates at 600 RPM, measures response |
| 2 | Verify speed gains | Expected: Kp = 0.3–2.0, Ki = 0.05–1.0 |
| 3 | **Tune Tension Loop** | Adjusts speed to oscillate tension around setpoint |
| 4 | Verify tension gains | Expected: Kp = 0.1–1.0, Ki = 0.02–0.5 |

> **Important**: Speed loop must be tuned **before** tension loop. The system enforces this order.

**Safety features in auto-tuning:**
- No-Overshoot tuning rule (prevents material breakage)
- Gain sanity validation (rejects faulty values like 0.00000215)
- Oscillation amplitude check (rejects noise-induced results)
- Results automatically saved to NVS and persist across reboots

---

## Safety Features

| Feature | Description |
|---------|-------------|
| E-STOP | Immediate motor stop, requires manual reset |
| Over-tension | Fault triggered if tension exceeds limit |
| Over-speed | Fault triggered if speed exceeds limit |
| Motor Stall | Detected if encoder stops while PWM active |
| Encoder Failure | Detected if no pulses received |
| Load Cell Failure | Detected if HX711 returns invalid data |
| Watchdog | System restart if main loop stalls |

---

## WebSocket Real-Time Updates

The web dashboard uses WebSocket for smooth 50Hz chart updates:

- **WebSocket endpoint**: `ws://<device-ip>/ws`
- **Broadcast rate**: 50Hz (20ms period)
- **Fallback**: Automatic HTTP polling if WebSocket connection fails
- **Data format**: Compact JSON with tension, speed, PWM, state, faults

---

## Troubleshooting

### Motor not spinning
1. Check H-bridge power supply
2. Verify PWM/DIR GPIO connections
3. Confirm motor driver enable signal

### Inaccurate tension reading
1. Perform tare calibration
2. Check load cell wiring
3. Ensure HX711 is receiving 5V power

### WebSocket disconnecting
1. Check WiFi signal strength
2. Verify mDNS is working: `ping tension-ctrl.local`
3. Use IP address directly if mDNS fails

### Auto-tune fails
1. Ensure motor is free to move
2. Check encoder is producing pulses
3. Verify load cell is reading properly
4. Try increasing timeout in menuconfig

---

## Project Structure

```
tension_controller/
├── CMakeLists.txt
├── sdkconfig.defaults
├── partitions.csv
├── README.md
├── main/
│   ├── main.c              # Application entry, WebSocket broadcast task
│   ├── system_config.h     # Pin assignments, constants
│   └── Kconfig.projbuild   # Menuconfig definitions
└── components/
    ├── encoder_driver/     # PCNT quadrature decoder
    ├── loadcell_driver/    # HX711 load cell ADC
    ├── motor_controller/   # LEDC PWM motor control
    ├── pid_controller/     # PI controller with anti-windup
    ├── auto_tuner/         # Relay feedback auto-tuning (Åström-Hägglund)
    ├── safety_monitor/     # State machine, fault handling
    ├── data_logger/        # Circular buffer, metrics
    ├── button_handler/     # Debouncing, E-STOP
    ├── lcd_interface/      # UART terminal UI
    ├── web_interface/      # WiFi, HTTP server, WebSocket, Chart.js dashboard
    └── control_manager/    # Central orchestrator, cascaded control
```

---

## License

MIT License - See LICENSE file for details.

## Author

Zeyad Barakat — [@zeyadbarakat](https://github.com/zeyadbarakat)
