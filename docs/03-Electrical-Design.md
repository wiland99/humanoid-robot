# Electrical System Design

## Overview

This document details the complete electrical and electronics design for the 150cm humanoid robot, including power systems, control architecture, wiring, and safety features.

**Last Updated**: 2025-11-26

---

## System Architecture

### High-Level Block Diagram

```
┌─────────────┐
│   Battery   │ (14.8V LiPo 4S 8000mAh)
│  + BMS      │
└──────┬──────┘
       │
       ├─────► Emergency Stop Switch
       │
┌──────▼──────────────────────┐
│  Power Distribution Board   │
│  - Fuses                    │
│  - Main Switch              │
│  - Voltage Monitoring       │
└──┬────┬────┬────┬───────────┘
   │    │    │    │
   │    │    │    └──► Buck Converter (5V 5A) → Jetson/Pi + Sensors
   │    │    │
   │    │    └───────► Buck Converter (5V 5A) → Arduino + Logic
   │    │
   │    └────────────► Buck Converter (12V 10A) → Servo Power Rail 1 (Lower Body)
   │
   └─────────────────► Buck Converter (12V 10A) → Servo Power Rail 2 (Upper Body)

Control Hierarchy:
┌────────────────┐
│ Jetson Nano    │ ◄── High-level planning, vision, AI
│ (Main Brain)   │
└────────┬───────┘
         │ USB
    ┌────▼────┐
    │ Arduino │ ◄── Motor control, sensor fusion
    │  Mega   │
    └────┬────┘
         │ TTL/RS485
    ┌────▼────────────┐
    │ Dynamixel Servos│ (27 servos)
    └─────────────────┘
```

---

## Power System

### Battery Specifications

**Primary Battery**: LiPo 4S (4 cells in series)
- Nominal Voltage: 14.8V (3.7V per cell)
- Fully Charged: 16.8V (4.2V per cell)
- Minimum Safe: 12.4V (3.1V per cell) - cutoff!
- Capacity: 8000mAh
- Discharge Rate: 50C (continuous 400A max)
- Connector: XT90
- Weight: ~380g

**Backup Battery** (recommended):
- Same specifications as primary
- Allows continuous operation during primary charging
- Hot-swap capability (with proper switching)

### Power Budget Analysis

| Component | Voltage | Current (typ) | Current (max) | Power (max) |
|-----------|---------|---------------|---------------|-------------|
| **Servos** |
| 10× MX-64AT (hip/knee) | 12V | 0.3A each | 1.4A each | 168W |
| 12× MX-28AT (shoulder/elbow) | 12V | 0.2A each | 0.9A each | 130W |
| 5× AX-12A (head/gripper) | 12V | 0.15A each | 0.9A each | 54W |
| **Subtotal Servos** | | **7.65A** | **31.7A** | **352W** |
| **Controllers** |
| Jetson Nano | 5V | 2A | 4A | 20W |
| Arduino Mega | 5V | 0.3A | 0.5A | 2.5W |
| U2D2 (Dynamixel) | 5V | 0.1A | 0.2A | 1W |
| **Sensors** |
| IMU (×2) | 5V | 0.015A | 0.02A | 0.1W |
| Cameras (×2) | 5V | 0.5A | 0.7A | 3.5W |
| Force Sensors (×8) | 5V | 0.08A | 0.1A | 0.5W |
| Misc Sensors | 5V | 0.2A | 0.3A | 1.5W |
| **Subtotal 5V** | | **3.165A** | **5.82A** | **29.1W** |
| **Cooling & Misc** |
| Cooling Fans (×2) | 12V | 0.2A | 0.3A | 3.6W |
| LED Indicators | 5V | 0.1A | 0.15A | 0.75W |
| **TOTAL** | | **11.115A** | **37.97A** | **385.45W** |

**Peak Power Draw**: 385W (all servos at max load - rare!)
**Typical Power Draw**: 150-200W (normal operation)
**Idle Power Draw**: 30-40W (standing still)

### Battery Runtime Estimates

**Formula**: Runtime (hours) = (Battery Capacity × Voltage × Efficiency) / Power Draw

**At Typical Load** (150W):
- Runtime = (8Ah × 14.8V × 0.85) / 150W = **0.67 hours (40 minutes)**

**At Idle** (40W):
- Runtime = (8Ah × 14.8V × 0.85) / 40W = **2.5 hours**

**At Peak Load** (385W):
- Runtime = (8Ah × 14.8V × 0.85) / 385W = **0.26 hours (15 minutes)**

**Recommendation**: Plan for 30-40 minute operation sessions, then recharge

### Power Distribution Board Design

**Custom PCB or Protoboard Layout**:

```
INPUT:
  XT90 Connector ──► Fuse 40A ──► Main Switch (60A rated)
                                    │
                     ┌──────────────┴──────────────┐
                     │                              │
                Emergency Stop Switch          Voltage Sensor
                (Normally Closed)              (to Arduino)
                     │
        ┌────────────┼────────────────┬─────────┬─────────┐
        │            │                │         │         │
    Output 1     Output 2        Output 3   Output 4   Output 5
    12V Rail1    12V Rail2       5V Logic   5V Brain   Reserved
    (10A fuse)   (10A fuse)      (5A fuse)  (5A fuse)  (10A fuse)
        │            │                │         │
    Buck 12V     Buck 12V        Buck 5V   Buck 5V
    ──► Legs     ──► Arms/Head   ──► Ard   ──► Jetson
```

**Features**:
- Individual fuses for each rail (safety)
- LED indicators for each output
- Screw terminals for easy connection
- Mounting holes for torso installation
- Voltage monitoring on all outputs

### Buck Converter Specifications

**12V Rails** (×2):
- Input: 12.4-16.8V (battery range)
- Output: 12.0V ±0.1V
- Current: 10A continuous, 12A peak
- Efficiency: >90%
- Recommended: LM2596-based or similar
- Features: Adjustable output, over-current protection

**5V Rails** (×2):
- Input: 12.4-16.8V
- Output: 5.0V ±0.05V
- Current: 5A continuous, 6A peak
- Efficiency: >85%
- Recommended: MP1584 or LM2596
- Features: Low ripple (<50mV), thermal protection

### Battery Management System (BMS)

**Functions**:
1. **Overcharge Protection**: Disconnect at 4.25V per cell
2. **Over-discharge Protection**: Disconnect at 3.0V per cell
3. **Overcurrent Protection**: Disconnect at >50A
4. **Cell Balancing**: Equalize cell voltages during charge
5. **Temperature Monitoring**: Shutdown if >60°C

**Implementation**:
- Integrated BMS board for 4S LiPo
- Connected between battery and power distribution
- Balancing connector (5-pin JST-XH)
- Status LEDs for monitoring

### Charging System

**Charger Specifications**:
- Type: LiPo balance charger (IMAX B6 or similar)
- Input: AC 100-240V or DC 11-18V
- Output: 0.1-6A adjustable
- Supports: 1-6S LiPo, LiFe, NiMH, etc.
- Features: Balance charging, storage mode, discharge

**Charging Procedure**:
1. Remove battery from robot (or use external charging port)
2. Connect balance lead (5-pin white connector)
3. Connect main power lead (XT90)
4. Select: LiPo Balance mode, 4S, 2-3A charge rate
5. Charge time: ~2-3 hours from empty
6. Never charge unattended!

**Safety**:
- Always charge in LiPo-safe bag
- Charge on non-flammable surface
- Monitor temperature (should stay cool)
- Set storage voltage (3.8V/cell) if not using for >1 week

---

## Control Electronics

### Main Computer: NVIDIA Jetson Nano

**Purpose**: High-level control, vision processing, AI/ML

**Specifications**:
- CPU: Quad-core ARM Cortex-A57 @ 1.43 GHz
- GPU: 128-core NVIDIA Maxwell
- RAM: 4GB LPDDR4
- Storage: MicroSD card (128GB recommended)
- OS: Ubuntu 20.04 + JetPack SDK

**Power**:
- Input: 5V DC, 4A max
- Connector: Barrel jack (5.5mm×2.5mm) or GPIO header
- Power Mode: 5W (efficiency) or 10W (performance)

**Interfaces**:
- GPIO: 40-pin header (compatible with Raspberry Pi)
- USB: 4× USB 3.0 Type-A
- Network: Gigabit Ethernet, WiFi (via USB dongle)
- Display: HDMI, DisplayPort (for debugging)
- Camera: 2× MIPI CSI-2 (for Pi cameras)

**Connections**:
- Arduino Mega: USB (serial communication)
- Cameras (×2): CSI ribbon cables
- WiFi Dongle: USB (for remote control)
- Cooling Fan: GPIO PWM control

### Motor Controller: Arduino Mega 2560

**Purpose**: Real-time servo control, sensor data acquisition

**Specifications**:
- MCU: ATmega2560 (16 MHz)
- Flash: 256 KB
- RAM: 8 KB
- EEPROM: 4 KB
- GPIO: 54 digital, 16 analog

**Power**:
- Input: 5V via USB or 7-12V via barrel jack
- Recommended: 5V from buck converter (cleaner power)

**Interfaces**:
- Serial: 4× UART (Serial, Serial1, Serial2, Serial3)
- I2C: 1× (SDA, SCL) - for sensors
- SPI: 1× - for expansion
- PWM: 15× pins

**Connections**:
- Jetson Nano: USB (Serial, 115200 baud)
- Dynamixel U2D2: Serial1 (TTL, 1 Mbps) or Serial2
- IMU (BNO055): I2C (address 0x28)
- Force Sensors: Analog pins A0-A7
- Emergency Stop: Digital pin 2 (interrupt)
- Status LED: Digital pin 13

### Dynamixel Interface: U2D2

**Purpose**: USB to TTL/RS485 converter for Dynamixel servos

**Specifications**:
- Connects to PC/Jetson via USB
- Outputs: TTL (3-pin), RS485 (4-pin)
- Supports: Dynamixel Protocol 1.0 and 2.0
- Baud Rate: Up to 4.5 Mbps

**Connections**:
- Jetson/Arduino: USB
- Servo Chain: Daisy-chained via 3-pin cables
- Power: Servos powered separately (not via U2D2!)

**Servo Chain Layout**:
```
U2D2 ──► Servo 1 (ID:1) ──► Servo 2 (ID:2) ──► ... ──► Servo 27 (ID:27)
          [data cable only, power from separate rail]
```

**Alternative**: Arduino controls servos directly via Serial TTL
- Pros: No extra hardware, lower cost
- Cons: Arduino must handle all servo communication

---

## Sensor Systems

### Inertial Measurement Unit (IMU)

**Model**: Adafruit BNO055

**Specifications**:
- 9-DOF: 3-axis accelerometer, gyroscope, magnetometer
- On-board sensor fusion (quaternion output)
- I2C interface (address 0x28 or 0x29)
- Sample Rate: Up to 100 Hz
- Accuracy: ±1° (heading), ±0.5° (pitch/roll)

**Placement**:
- IMU #1: Center of torso (main balance sensing)
- IMU #2: Head (optional, for head stabilization)

**Wiring**:
```
BNO055        Arduino Mega
VIN (3.3-5V)  ─── 5V
GND           ─── GND
SDA           ─── SDA (pin 20)
SCL           ─── SCL (pin 21)
RST           ─── Pin 7 (optional, for reset)
```

**Power**: 5V, 12mA typical

### Force Sensors (FSR)

**Model**: Interlink FSR 402

**Specifications**:
- Sensing Range: 0-10 kg
- Resistance: 100kΩ (no load) to <1kΩ (max load)
- Active Area: 12.7mm × 12.7mm
- Response Time: <2ms

**Placement**: 4 sensors per foot (8 total)
- Front-left, front-right, rear-left, rear-right

**Circuit** (per sensor):
```
         5V
          │
         ┌┴┐
         │ │ 10kΩ (pull-down resistor)
         └┬┘
          ├────► To Arduino Analog Pin (A0-A7)
         ┌┴┐
         │F│ FSR (variable resistance)
         │S│
         │R│
         └┬┘
          │
         GND
```

**Reading**:
- Voltage divider: V_out = 5V × (R_fixed / (R_FSR + R_fixed))
- Arduino reads 0-1023 (10-bit ADC)
- Convert to force: Calibration required

### Vision: Raspberry Pi Camera Module V3

**Specifications**:
- Sensor: Sony IMX708 (12MP)
- Resolution: 4608 × 2592 (still), 1920 × 1080 @ 60fps (video)
- Lens: Wide-angle (approx. 75° diagonal FOV)
- Interface: MIPI CSI-2 (15-pin ribbon cable)
- Power: 5V, 0.5-0.7A

**Placement**: 2 cameras in head (stereo vision)
- Baseline: 6-7 cm apart (eye spacing)
- Forward-facing, slight downward tilt (10-15°)

**Connection**:
```
Camera 1 ──► Jetson Nano CSI Port 0
Camera 2 ──► Jetson Nano CSI Port 1
```

**Software**:
- GStreamer for camera access
- OpenCV for image processing
- Stereo vision libraries (e.g., StereoBM)

### Distance Sensors: Ultrasonic (HC-SR04)

**Specifications**:
- Range: 2 cm - 400 cm
- Accuracy: ±3mm
- Interface: Digital (Trigger + Echo pins)
- Power: 5V, 15mA

**Placement** (optional, 4 total):
- Front torso, back torso, left side, right side

**Wiring** (per sensor):
```
HC-SR04       Arduino
VCC           ─── 5V
GND           ─── GND
Trig          ─── Digital Pin (e.g., 22, 24, 26, 28)
Echo          ─── Digital Pin (e.g., 23, 25, 27, 29)
```

**Usage**:
- Trigger pulse: 10µs HIGH
- Measure Echo pulse width
- Distance (cm) = Pulse Width (µs) / 58

### Current & Voltage Monitoring

**Voltage Sensor**: Simple resistor divider

**Circuit**:
```
Battery+ ──┬──► To Power Distribution
           │
          ┌┴┐ 10kΩ
          └┬┘
           ├───► To Arduino Analog Pin A8
          ┌┴┐ 2.2kΩ
          └┬┘
           │
          GND

Voltage Reading = ADC_Value × (5V / 1024) × ((10k + 2.2k) / 2.2k)
                = ADC_Value × 0.0276 V
```

**Current Sensor**: INA219

**Specifications**:
- Measures voltage and current
- I2C interface (address 0x40, 0x41, 0x44, 0x45)
- Range: ±3.2A (default) or ±1A (high precision)
- Accuracy: ±0.5%

**Placement**: On each major power rail
- Rail 1 (12V servos legs): INA219 #1
- Rail 2 (12V servos arms): INA219 #2
- 5V main: INA219 #3

---

## Servo Configuration

### Dynamixel Servo Network

**Protocol**: Dynamixel Protocol 2.0 (recommended)
- Supports higher baud rates (up to 4.5 Mbps)
- Better error checking
- More features

**Daisy Chain Topology**:
```
U2D2/Arduino TTL ──► [ID:1] ──► [ID:2] ──► ... ──► [ID:27]
                      └─ Power from 12V Rail (separate wiring)
```

**Servo ID Assignment**:

| ID | Location | Model | Notes |
|----|----------|-------|-------|
| 1 | Left Hip Yaw | MX-64AT | Rotation |
| 2 | Left Hip Pitch | MX-64AT | Flexion |
| 3 | Left Hip Roll | MX-64AT | Abduction |
| 4 | Left Knee Pitch | XM430-W350 | Flexion |
| 5 | Left Ankle Pitch | MX-28AT | Dorsiflexion |
| 6 | Left Ankle Roll | MX-28AT | Inversion |
| 7 | Right Hip Yaw | MX-64AT | Rotation |
| 8 | Right Hip Pitch | MX-64AT | Flexion |
| 9 | Right Hip Roll | MX-64AT | Abduction |
| 10 | Right Knee Pitch | XM430-W350 | Flexion |
| 11 | Right Ankle Pitch | MX-28AT | Dorsiflexion |
| 12 | Right Ankle Roll | MX-28AT | Inversion |
| 13 | Waist Yaw | MX-28AT | Torso rotation |
| 14 | Waist Pitch | MX-28AT | Torso lean |
| 15 | Left Shoulder Yaw | MX-28AT | Rotation |
| 16 | Left Shoulder Pitch | MX-28AT | Flexion |
| 17 | Left Shoulder Roll | MX-28AT | Abduction |
| 18 | Left Elbow Pitch | MX-28AT | Flexion |
| 19 | Left Wrist Roll | AX-12A | Rotation |
| 20 | Left Gripper | AX-12A | Open/close |
| 21 | Right Shoulder Yaw | MX-28AT | Rotation |
| 22 | Right Shoulder Pitch | MX-28AT | Flexion |
| 23 | Right Shoulder Roll | MX-28AT | Abduction |
| 24 | Right Elbow Pitch | MX-28AT | Flexion |
| 25 | Right Wrist Roll | AX-12A | Rotation |
| 26 | Right Gripper | AX-12A | Open/close |
| 27 | Neck Yaw | AX-12A | Pan |
| 28 | Neck Pitch | AX-12A | Tilt |
| 29 | Neck Roll | AX-12A | Roll |

**Configuration** (via Dynamixel Wizard):
- Set unique ID for each servo
- Set baud rate: 1,000,000 (1 Mbps) or higher
- Set return delay: 0 or minimal
- Enable torque limit (70-80% of max for safety)
- Set temperature limit: 70°C

### Servo Power Distribution

**Power Rails**:
- **Rail 1 (12V, 10A)**: Lower body servos (IDs 1-12)
- **Rail 2 (12V, 10A)**: Upper body servos (IDs 13-29)

**Wiring**:
- Use 18 AWG silicone wire for power (red/black)
- Power taps every 3-5 servos (star topology for power, daisy for data)
- Minimize voltage drop: Keep wires short, thick gauge

**Example Power Distribution**:
```
Buck Converter 12V ──┬──► Servo 1, 2, 3 (power)
                     ├──► Servo 4, 5, 6 (power)
                     ├──► Servo 7, 8, 9 (power)
                     └──► Servo 10, 11, 12 (power)

Data chain:
U2D2 ──► Servo 1 ──► Servo 2 ──► ... ──► Servo 12 (daisy chain, 22 AWG)
```

---

## Wiring Specifications

### Wire Gauge Selection

| Purpose | Gauge | Max Current | Notes |
|---------|-------|-------------|-------|
| Battery to PDB | 14 AWG | 40A | Short run (<30 cm) |
| 12V Power Rails | 18 AWG | 10A | Multiple taps |
| 5V Power | 22 AWG | 3A | Low current |
| Servo Signal | 24-26 AWG | N/A | Data only |
| Sensor Signal | 26-28 AWG | N/A | Low current |

### Wire Color Coding

**Standard**:
- Red: Positive power
- Black: Ground/negative
- Yellow: Signal/data (servo PWM)
- Other colors: Various signals (label clearly)

**Custom Coding** (for multi-wire bundles):
- Power: Red (+), Black (-)
- I2C: Blue (SDA), Green (SCL)
- Serial: Orange (TX), Purple (RX)
- Analog: Gray or white

### Cable Management

**Inside Limbs**:
- Flat ribbon cable or spiral wrap
- Secure every 10 cm with zip ties or clips
- Service loops at joints (extra length for movement)

**Torso**:
- Central wiring harness spine (3D printed channel)
- Branch to each limb
- Velcro straps for organization

**Connectors**:
- JST-XH for permanent connections
- Dupont for removable/prototyping
- XT90 for high-current battery
- Labeled with wire labels or heat shrink

---

## Safety Features

### Emergency Stop (E-Stop) Circuit

**Hardware**:
- Physical button: Normally Closed (NC) switch
- Placement: Accessible on torso back
- Type: Mushroom head, red, twist-to-release

**Circuit**:
```
Battery+ ──► E-Stop Switch (NC) ──► Power Distribution Board
                │
                └── When pressed: Opens circuit, cuts all power
```

**Software Backup**:
- E-Stop signal to Arduino (digital pin 2)
- Interrupt-driven
- Action: Disable all servos (torque off), save state, shutdown

### Overcurrent Protection

**Implementation**:
- Fuses on each power rail (power distribution board)
- Fuse ratings: 10A (servo rails), 5A (logic rails), 40A (main)
- Fast-blow type for electronics

### Over-temperature Protection

**Monitoring**:
- Servo internal temperature (read via Dynamixel protocol)
- Battery temperature (BMS or external thermistor)
- Jetson temperature (software monitoring)

**Actions**:
- Warning: Reduce servo torque limit, increase fan speed
- Critical (>70°C): Disable servos, shutdown system

### Low Voltage Cutoff

**Implementation**:
- BMS cuts power at 3.0V per cell (12.0V total)
- Software monitoring: Arduino reads battery voltage
- Warning at 3.3V per cell (13.2V total) - "low battery"
- Controlled shutdown at 3.1V per cell (12.4V total)

### Fault Indicators

**LEDs on Power Distribution Board**:
- Green: Power OK
- Yellow: Low battery warning
- Red: Fault (overcurrent, over-temp, e-stop)
- Blue: Charging (if charging port installed)

**Buzzer**:
- Low battery: Intermittent beep (every 5 seconds)
- Critical: Continuous beep
- E-stop: 3 quick beeps, then silence

---

## Communication Protocols

### Inter-Controller Communication

**Jetson ↔ Arduino**:
- Interface: USB (appears as /dev/ttyACM0 on Jetson)
- Protocol: Custom serial protocol (or ROS serial)
- Baud Rate: 115200 bps
- Format: ASCII or binary (define in software)

**Example Messages**:
```
Jetson → Arduino:
  CMD:MOVE,JOINT:5,POS:1024,SPEED:100\n
  CMD:READ,SENSOR:IMU\n

Arduino → Jetson:
  DATA:IMU,ROLL:0.5,PITCH:-2.3,YAW:45.2\n
  DATA:FSR,FL:450,FR:480,RL:420,RR:440\n
  STATUS:OK\n
```

### Dynamixel Communication

**Protocol**: Dynamixel Protocol 2.0

**Packet Structure**:
- Header: 0xFF 0xFF 0xFD 0x00
- ID: Servo ID (0-252)
- Length: Packet length
- Instruction: Read/write/sync write/etc.
- Parameters: Register address, data
- CRC: 16-bit checksum

**Common Instructions**:
- **Sync Write**: Control multiple servos simultaneously (efficient)
- **Read**: Read servo registers (position, temp, voltage)
- **Bulk Read**: Read from multiple servos

**Software Libraries**:
- Dynamixel SDK (C++, Python, Java, etc.)
- Arduino Dynamixel library

---

## PCB Designs (Optional Custom Boards)

### Power Distribution PCB

**Features**:
- Screw terminals for battery input
- Fuse holders (automotive blade fuses)
- Buck converter modules (solderable)
- LED indicators for each rail
- Voltage sense lines to Arduino
- Compact: 10 cm × 15 cm

**Layers**: 2-layer board (sufficient for power distribution)

**Manufacturing**: OSH Park, JLCPCB, PCBWay (~$50 for 3 boards)

### Sensor Breakout Board (Torso)

**Purpose**: Consolidate sensor connections

**Features**:
- I2C bus with multiple connectors
- Pull-up resistors (4.7kΩ for SDA/SCL)
- Analog breakout (screw terminals)
- Power distribution (5V, GND)
- Mounting holes for torso interior

---

## Testing & Debugging Tools

### Required Test Equipment

1. **Multimeter**: Voltage, current, continuity checks
2. **Power Supply**: Bench supply (0-30V, 5A) for testing without battery
3. **USB-to-TTL Adapter**: For servo testing and debugging
4. **Oscilloscope** (optional): For signal integrity, noise analysis
5. **Dynamixel Wizard**: PC software for servo configuration

### Test Procedures

**Power System**:
1. Test battery voltage (should be 16.8V fully charged)
2. Test BMS cutoff (discharge to 12.0V, verify disconnect)
3. Test buck converters (verify 12V and 5V outputs under load)
4. Measure voltage drop (should be <0.5V under load)

**Servos**:
1. Test each servo individually (Dynamixel Wizard)
2. Check current draw (should match specifications)
3. Verify temperature (shouldn't exceed 50°C under normal load)
4. Test daisy chain (all servos respond)

**Sensors**:
1. IMU: Verify orientation output (tilt robot, check values)
2. FSR: Apply known weight, calibrate output
3. Cameras: Capture test images, verify focus and exposure

---

## Maintenance & Troubleshooting

### Regular Checks (Every 10 hours of operation)

- Inspect all wiring for wear, damage
- Check connector tightness
- Verify battery voltage and balance
- Test E-stop functionality
- Clean sensors (cameras, IMU)

### Common Issues

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| Servo not responding | Wiring, wrong ID, power | Check data cable, verify ID, check voltage |
| Robot shuts down unexpectedly | Low battery, BMS cutoff | Charge battery, check cell balance |
| Jerky movements | Voltage drop, loose connection | Thicker power wires, secure connections |
| Overheating | Insufficient cooling, overload | Add fans, reduce torque limit |
| IMU drift | Magnetic interference | Calibrate, move away from motors |

---

## Future Upgrades

### Potential Improvements

1. **Wireless Communication**: WiFi or Bluetooth for remote control
2. **Onboard Display**: Small LCD for status monitoring
3. **Sound System**: Speaker for audio feedback
4. **Additional Sensors**: LIDAR, depth cameras, microphones
5. **Higher Capacity Battery**: 10,000-12,000 mAh for longer runtime
6. **Backup Power**: Supercapacitors for ride-through during battery swap

---

## Appendix: Useful Formulas

### Power Calculations

**Power (W)** = Voltage (V) × Current (A)

**Energy (Wh)** = Power (W) × Time (h)

**Battery Capacity (Wh)** = Voltage (V) × Capacity (Ah)

### Voltage Divider

**V_out** = V_in × (R2 / (R1 + R2))

### Ohm's Law

**V** = I × R
**I** = V / R
**R** = V / I

### Servo Torque to Force

**Force (N)** = Torque (Nm) / Distance (m)

Example: 6.0 Nm torque at 0.1m radius = 60 N force

---

**Document Status**: Initial Draft
**Next Review**: After component procurement
**Maintained By**: Project owner

