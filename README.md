# ESP32-S3 Tension Control System

A production-ready cascaded PI tension control system for unwinding applications using ESP32-S3 and ESP-IDF v5.x.

![ESP32-S3](https://img.shields.io/badge/ESP32--S3-Supported-blue)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.5-green)
![License](https://img.shields.io/badge/License-MIT-yellow)
![Version](https://img.shields.io/badge/Version-1.1.0-orange)

## Features

- **Cascaded PI Control** — Outer tension loop, inner speed loop for smooth unwinding
- **Relay Feedback Auto-Tuning** — Åström-Hägglund method with No-Overshoot tuning rule
- **20x4 I2C LCD & 1x4 Keypad** — Full physical interface for standalone operation
- **Real-Time WebSocket Dashboard** — 50Hz Chart.js graphs via WebSocket push
- **PCNT Quadrature Encoder** — Hardware 4x decoding, 600 PPR supported
- **HX711 Load Cell** — 24-bit ADC with NVS calibration storage
- **Safety Monitoring** — E-STOP, fault detection, state machine with hardware TWDT watchdog
- **Persistent Tuning** — PI gains saved to NVS (x10000 precision), restored on boot automatically

---

## How It Works

The user sets a **tension setpoint** (e.g., 5.0 kg). The system automatically adjusts motor speed to maintain constant tension as the roll unwinds:

```
Full Roll  → Slow RPM   → Tension = 5.0 kg ✅
Half Roll  → Medium RPM → Tension = 5.0 kg ✅
Empty Roll → Fast RPM   → Tension = 5.0 kg ✅
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
| LCD | 20x4 Character LCD + PCF8574 I2C |
| Keypad | 1x4 Membrane (or 4 generic buttons) |
| Motor Driver | H-Bridge (BTS7960 or similar) |
| Buttons | RUN, STOP, E-STOP (NC) |

---

## Wiring Diagram

```
                           ESP32-S3
                        ┌─────────────┐
                        │             │
    Encoder A ──────────┤ GPIO4       │
    Encoder B ──────────┤ GPIO5       │
                        │             │
    KEY1 (UP) ──────────┤ GPIO1       │
    KEY2 (OK) ──────────┤ GPIO2       │
    KEY3 (LEFT) ────────┤ GPIO6       │
    KEY4 (RIGHT) ───────┤ GPIO7       │
                        │             │
    LCD SDA ────────────┤ GPIO8       │
    LCD SCL ────────────┤ GPIO9       │
                        │             │
    HX711 DATA ─────────┤ GPIO13      │
    HX711 CLK ──────────┤ GPIO14      │
                        │             │
    Motor PWM ──────────┤ GPIO15      ├───── H-Bridge IN1
    Motor DIR ──────────┤ GPIO16      ├───── H-Bridge IN2
                        │             │
    RUN Button ──┬──────┤ GPIO10      │      (NC to GND)
                 │      │             │
    STOP Button ─┬──────┤ GPIO11      │      (NC to GND)
                 │      │             │
    E-STOP ──────┬──────┤ GPIO12      │      (NC to GND)
                 │      │             │
               10kΩ     │             │
               Pullup   └─────────────┘
```

### Pin Assignment Table

| Function | GPIO | Notes |
|----------|------|-------|
| KEY1 (▲) | 1 | Tension up / Menu enter |
| KEY2 (OK)| 2 | Confirm / Menu OK |
| Encoder A| 4 | PCNT quadrature A |
| Encoder B| 5 | PCNT quadrature B |
| KEY3 (◄) | 6 | Jog left / Navigate up |
| KEY4 (►) | 7 | Jog right / Navigate down |
| LCD SDA  | 8 | I2C Data (0x27) |
| LCD SCL  | 9 | I2C Clock |
| RUN Button| 10 | Standalone Start |
| STOP Button| 11 | Standalone Stop |
| E-STOP   | 12 | Hardware Emergency Stop (NC) |
| HX711 Data| 13 | Load cell Data |
| HX711 CLK | 14 | Load cell Clock |
| Motor PWM | 15 | 25kHz PWM Output |
| Motor DIR | 16 | Direction Signal |
| LED Run   | 17 | Green status LED |
| LED Fault | 18 | Red fault LED |

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

### Physical Interface (LCD + Keypad)

The 20x4 LCD provides a standalone interface for the tension controller:

- **Main Screen**: Displays Tension (Actual/Setpoint), Speed (Actual/PWM%), and System State.
- **Navigation**:
  - **Single Press ▲/▼**: Adjust tension setpoint.
  - **Hold ▲**: Enter Settings Menu.
  - **Hold ◄/►**: Jog motor (only in IDLE).
- **Menu System**:
  - **Calibration**: Tare and Span calibrate the load cell.
  - **Auto-Tune**: Trigger speed/tension auto-tuning.
  - **Config**: Edit Encoder PPR, Motor direction, etc.
  - **Status**: View active faults and system info.

### Web Interface

1. Connect to WiFi:
   - **AP Mode** (default): Connect to the AP SSID configured in menuconfig (default: `TensionAP`, password: `tension123`)
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
| Watchdog | Hardware TWDT resets system if control task stalls |

---

## WebSocket Real-Time Updates

The web dashboard uses WebSocket for smooth 50Hz chart updates:

- **WebSocket endpoint**: `ws://<device-ip>/ws`
- **Broadcast rate**: 50Hz (20ms period)
- **Fallback**: Automatic HTTP polling if WebSocket connection fails
- **Data format**: Compact JSON with tension, speed, PWM, state, faults

---

## Troubleshooting

### PWM showing 0.0% unexpectedly
- **Fixed in v1.2.0**. Ensure you are in the `RUNNING` or `JOGGING` state. If the speed is below the deadband threshold (<15% PWM by default in safety checks), the system may prevent startup if no encoder movement is detected.

### Automatic Stop / Encoder Fail
- The system expects encoder feedback if `PWM > 15.0%`. If the motor doesn't spin at this level, check your motor power supply or deadband settings. You can increase the threshold in `safety_monitor/safety.c`.

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
    ├── lcd_interface/      # Physical LCD & Keypad driver
    ├── web_interface/      # WiFi, HTTP server, WebSocket, Chart.js dashboard
    └── control_manager/    # Central orchestrator, cascaded control
```

---

## License

MIT License - See LICENSE file for details.

---

## Changelog

### v1.2.0 — Physical UI & Stability (MAR 2026)

**New Hardware Support:**
- Added support for 20x4 I2C LCD and 1x4 Keypad for standalone operation.
- Added "Jog" functionality via keypad long-press.

**Critical Bug Fixes:**
- Fixed **PWM display bug** where Web UI would report 0.0% PWM during operation.
- Fixed **Autotune bug** where Autotune would start but never transition the motor state from IDLE.
- Fixed **Encoder reliability**: Increased failure threshold from 5.0% to 15.0% to allow for motor starting inertia.

### v1.1.0 — Bug Fixes & Robustness (Feb 2026)

**Critical Bug Fixes:**
- Fixed `SYSTEM_STATE_STARTING` never transitioning to `RUNNING`.
- Fixed WebSocket clients and UART NVS precision.

**Robustness Improvements:**
- Added ESP32 hardware Task Watchdog Timer (TWDT) for control task.
- Added load cell calibration check before tension loop auto-tune.

### v1.0.0 — Initial Release

- Cascaded PI control with auto-tuning.
- WebSocket real-time dashboard.
- Full safety monitoring.

## Author

Zeyad Barakat — [@zeyadbarakat](https://github.com/zeyadbarakat)
