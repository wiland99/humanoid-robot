# Bill of Materials (BOM)

## Overview

This document contains a comprehensive list of all components needed to build the 150cm humanoid robot. Prices are approximate and based on mid-range quality components.

**Total Estimated Cost**: $8,000 - $12,000
**Last Updated**: 2025-11-26

---

## 1. Actuators & Motors ($3,000 - $5,000)

### High-Torque Servos (Legs & Major Joints)
**Quantity**: 10
**Location**: Hip joints (6), Knee joints (2), Ankle joints (2)

| Component | Specifications | Unit Price | Total | Notes |
|-----------|---------------|------------|-------|-------|
| Dynamixel MX-64AT | 6.0 Nm (61 kg-cm) @ 12V, 310° range | $200 | $2,000 | Recommended for hips/knees |
| Alternative: Feetech SCS0009 | 35 kg-cm @ 12V | $80 | $800 | Budget option |
| Alternative: Robotis XM430-W350 | 4.1 Nm @ 12V | $180 | $1,800 | Good middle ground |

**Recommended Choice**: 6× Dynamixel MX-64AT for hips, 4× XM430-W350 for knees/ankles = $2,920

### Medium-Torque Servos (Arms & Torso)
**Quantity**: 12
**Location**: Shoulders (6), Elbows (2), Wrists (4)

| Component | Specifications | Unit Price | Total | Notes |
|-----------|---------------|------------|-------|-------|
| Dynamixel MX-28AT | 2.5 Nm (25.5 kg-cm) @ 12V | $150 | $1,800 | Recommended |
| Alternative: XL430-W250 | 1.4 Nm @ 12V | $50 | $600 | Budget option |

**Recommended Choice**: 12× Dynamixel MX-28AT = $1,800

### Low-Torque Servos (Head, Grippers, Torso)
**Quantity**: 5
**Location**: Neck (3), Grippers (2)

| Component | Specifications | Unit Price | Total | Notes |
|-----------|---------------|------------|-------|-------|
| Dynamixel AX-12A | 1.5 Nm @ 12V | $45 | $225 | Standard choice |
| Alternative: Standard digital servo | 15-20 kg-cm | $25 | $125 | Budget option |

**Recommended Choice**: 5× Dynamixel AX-12A = $225

**Actuator Subtotal**: ~$4,945 (premium) or ~$1,525 (budget mix)
**Recommended Mid-Range**: ~$3,500 (mix of Dynamixel and alternatives)

---

## 2. Main Controllers & Processing ($400 - $800)

### Primary Computer

| Component | Specifications | Price | Notes |
|-----------|---------------|-------|-------|
| **NVIDIA Jetson Nano 4GB** (Recommended) | Quad-core ARM A57 @ 1.43GHz, 128-core GPU | $150 | Best for vision/AI |
| Raspberry Pi 5 8GB | Quad-core ARM Cortex-A76 @ 2.4GHz | $80 | Good all-rounder |
| Raspberry Pi 4B 8GB | Quad-core ARM Cortex-A72 @ 1.5GHz | $75 | Budget option |

**Recommended**: NVIDIA Jetson Nano 4GB = $150

### Motor Controller(s)

| Component | Specifications | Price | Notes |
|-----------|---------------|-------|-------|
| **Dynamixel U2D2** | USB interface for Dynamixel servos | $50 | If using all Dynamixel |
| Arduino Mega 2560 R3 | 54 digital I/O, 16 analog inputs | $40 | Alternative controller |
| Teensy 4.1 | 600MHz ARM Cortex-M7, 55 I/O | $30 | Powerful alternative |
| **PCA9685 16-Ch PWM Driver** (×2) | I2C servo driver, 16 channels | $15 × 2 | For standard PWM servos |

**Recommended**: 1× Dynamixel U2D2 ($50) + 1× Arduino Mega ($40) for auxiliary = $90

### Additional Boards

| Component | Purpose | Price | Qty | Total |
|-----------|---------|-------|-----|-------|
| USB Hub (powered, 7-port) | Connect all peripherals | $25 | 1 | $25 |
| MicroSD Card (128GB, Class 10) | OS and data storage | $20 | 2 | $40 |

**Controllers Subtotal**: $150 + $90 + $65 = **$305**

---

## 3. Sensors ($600 - $1,000)

### Inertial Measurement

| Component | Specifications | Price | Qty | Total |
|-----------|---------------|-------|-----|-------|
| **BNO055 9-DOF IMU** | Accelerometer, gyro, magnetometer, fusion | $35 | 2 | $70 |
| Alternative: MPU6050 | 6-DOF (no magnetometer) | $5 | 2 | $10 |

**Recommended**: 2× BNO055 (torso + head) = $70

### Force/Pressure Sensors

| Component | Specifications | Price | Qty | Total |
|-----------|---------------|-------|-----|-------|
| **FSR 402 Force Sensors** | 0-10kg range | $10 | 8 | $80 |
| Load Cells (50kg) | More accurate, requires HX711 amp | $8 + $3 | 4 sets | $44 |

**Recommended**: 8× FSR sensors (4 per foot, 2 locations each) = $80

### Vision

| Component | Specifications | Price | Qty | Total |
|-----------|---------------|-------|-----|-------|
| **Raspberry Pi Camera Module V3** | 12MP, wide angle | $25 | 2 | $50 |
| Intel RealSense D435i (optional) | Depth camera with IMU | $380 | 1 | $380 |
| USB Webcam (Logitech C270) | Budget option | $25 | 2 | $50 |

**Recommended**: 2× Pi Camera Module V3 = $50
**Optional Upgrade**: 1× RealSense D435i = $380

### Encoders (if not using smart servos)

| Component | Specifications | Price | Qty | Total |
|-----------|---------------|-------|-----|-------|
| Incremental Rotary Encoder | 600 P/R | $8 | 10 | $80 |

**Note**: Not needed if using Dynamixel (has built-in encoders)

### Additional Sensors

| Component | Purpose | Price | Qty | Total |
|-----------|---------|-------|-----|-------|
| Ultrasonic Sensors (HC-SR04) | Distance sensing | $3 | 4 | $12 |
| Touch/Bump Sensors | Safety | $2 | 6 | $12 |
| Current Sensors (INA219) | Power monitoring | $8 | 3 | $24 |
| Voltage Sensor Modules | Battery monitoring | $3 | 2 | $6 |

**Sensors Subtotal**: $70 + $80 + $50 + $12 + $12 + $24 + $6 = **$254**
**With Optional Depth Camera**: $634

---

## 4. Power System ($600 - $1,200)

### Batteries

| Component | Specifications | Price | Qty | Total |
|-----------|---------------|-------|-----|-------|
| **LiPo 4S 14.8V 8000mAh** | High discharge (50C+), XT90 connector | $120 | 2 | $240 |
| LiPo 3S 11.1V 5000mAh | Alternative lower voltage | $60 | 2 | $120 |

**Recommended**: 2× 4S 8000mAh (one primary, one backup) = $240

### Battery Management & Charging

| Component | Purpose | Price | Qty | Total |
|-----------|---------|-------|-----|-------|
| **LiPo Balance Charger** | Multi-chemistry, 6S capable | $60 | 1 | $60 |
| Battery Management System (4S) | Overcharge/discharge protection | $25 | 2 | $50 |
| LiPo Battery Bag | Fire-safe storage/charging | $15 | 2 | $30 |
| Low Voltage Alarm | Battery protection | $5 | 2 | $10 |

### Power Distribution & Regulation

| Component | Specifications | Price | Qty | Total |
|-----------|---------------|-------|-----|-------|
| **Buck Converter 12V 10A** | For servo power | $15 | 2 | $30 |
| Buck Converter 5V 5A | For logic/sensors | $8 | 2 | $16 |
| Power Distribution Board | Fused outputs | $20 | 1 | $20 |
| XT90 Connectors | High-current battery connector | $2 | 10 | $20 |
| Emergency Stop Switch | Safety cutoff | $12 | 1 | $12 |
| Power Switch (high current) | Main power on/off | $8 | 1 | $8 |

**Power System Subtotal**: $240 + $150 + $106 = **$496**

---

## 5. Structural Materials ($1,200 - $2,000)

### 3D Printing Materials

| Component | Specifications | Price | Qty | Total |
|-----------|---------------|-------|-----|-------|
| **PETG Filament 1kg** | Strong, moderate flexibility | $25 | 10 | $250 |
| ABS Filament 1kg | Alternative, more rigid | $22 | 5 | $110 |
| TPU Filament 1kg | Flexible (for feet/grippers) | $30 | 2 | $60 |

**Recommended**: 10kg PETG + 2kg TPU = $310

### Metal Structural Components

| Component | Specifications | Price | Total | Notes |
|-----------|---------------|-------|-------|-------|
| **Aluminum Extrusion 2020** | 20×20mm, 6m total length | $50 | $50 | Main frame |
| Aluminum Flat Bar (various) | 1/8" × 1", multiple lengths | $80 | $80 | Custom brackets |
| Aluminum Tube (various) | For limbs | $60 | $60 | Lightweight structure |
| Aluminum Sheet (2mm) | For panels | $40 | $40 | Body panels |

### Reinforcement (Optional)

| Component | Specifications | Price | Total |
|-----------|---------------|-------|-------|
| Carbon Fiber Sheet 2mm | High strength-to-weight | $60 | $120 |
| Fiberglass Sheet | Alternative reinforcement | $30 | $60 |

**Recommended**: Skip carbon fiber initially, add if needed

### Hardware & Fasteners

| Component | Specifications | Price | Total |
|-----------|---------------|-------|-------|
| **Assorted M2-M6 Bolts Kit** | Stainless steel, various lengths | $40 | $40 |
| Hex Nuts Assortment | M2-M6 | $15 | $15 |
| Lock Nuts Assortment | M2-M6 | $20 | $20 |
| Washers Assortment | Various sizes | $10 | $10 |
| Threaded Inserts (M3, M4) | For 3D printed parts | $15 | $30 |

### Bearings & Motion Components

| Component | Specifications | Price | Qty | Total |
|-----------|---------------|-------|-----|-------|
| **Ball Bearings 608** | 8mm ID, 22mm OD | $1.50 | 30 | $45 |
| Thrust Bearings | For rotating joints | $3 | 10 | $30 |
| Timing Belts GT2 | For gearing (optional) | $15 | 2 | $30 |
| Pulleys GT2 | Various tooth counts | $5 | 8 | $40 |

**Structural Materials Subtotal**: $310 + $230 + $115 + $145 = **$800**

---

## 6. Electronics & Wiring ($400 - $700)

### Wiring & Cables

| Component | Specifications | Price | Total |
|-----------|---------------|-------|-------|
| **Silicone Wire 18AWG** | Red/Black, 10m each | $15 | $30 |
| Silicone Wire 22AWG | Signal wires, multiple colors | $10 | $30 |
| Servo Extension Cables | Various lengths | $20 | $40 |
| USB Cables (various) | A-B, micro, type-C | $20 | $20 |
| Dupont Connector Kit | Male/female pins | $15 | $15 |
| Heat Shrink Tubing Kit | Assorted sizes | $12 | $12 |
| Cable Management | Zip ties, spiral wrap, velcro | $20 | $20 |

### Connectors & Terminals

| Component | Price | Total |
|-----------|-------|-------|
| JST Connector Kit | $15 | $15 |
| Screw Terminal Blocks | $10 | $10 |
| Pin Headers (male/female) | $8 | $8 |

### PCB & Prototyping

| Component | Specifications | Price | Qty | Total |
|-----------|---------------|-------|-----|-------|
| Perfboard/Protoboard | Various sizes | $3 | 10 | $30 |
| PCB Manufacturing | Custom boards (optional) | $50 | 2 | $100 |

### Electronic Components

| Component | Price | Total |
|-----------|-------|-------|
| Resistor Kit (1/4W) | $10 | $10 |
| Capacitor Kit | $15 | $15 |
| Diodes (various) | $8 | $8 |
| Transistors/MOSFETs | $10 | $10 |
| LED Indicators | $5 | $5 |
| Buzzer/Speaker | $8 | $8 |

### Tools for Electronics

| Component | Price | Total |
|-----------|-------|-------|
| Soldering Iron Kit | $40 | $40 |
| Multimeter | $30 | $30 |
| Wire Strippers | $15 | $15 |
| Helping Hands | $12 | $12 |
| Solder & Flux | $15 | $15 |

**Electronics Subtotal**: $167 + $33 + $130 + $56 + $112 = **$498**

---

## 7. Tools & Equipment ($800 - $1,500)

### 3D Printer (if not already owned)

| Component | Specifications | Price | Notes |
|-----------|---------------|-------|-------|
| **Creality Ender 3 V3** | 220×220×250mm build | $200 | Budget option |
| Prusa MK4 | 250×210×220mm, reliable | $800 | Premium option |
| Bambu Lab P1S | Fast, enclosed | $700 | Good middle ground |
| **3D Printing Service** | Per-part pricing | ~$500 | If not buying printer |

**Recommended**: Buy Ender 3 V3 or use printing service = $200-500

### Hand Tools (if not already owned)

| Tool | Price | Notes |
|------|-------|-------|
| Screwdriver Set | $25 | Precision and standard |
| Hex Key Set (Metric) | $15 | Essential for assembly |
| Pliers Set | $30 | Needle nose, cutting, etc. |
| Adjustable Wrench Set | $25 | Various sizes |
| Files Set | $20 | For finishing parts |
| Hobby Knife Set | $15 | Deburring, trimming |
| Drill Bit Set | $30 | Metric sizes |
| Tap & Die Set (Metric) | $40 | Threading |

### Power Tools (if not already owned)

| Tool | Price | Notes |
|------|-------|-------|
| Cordless Drill | $80 | Essential |
| Dremel/Rotary Tool | $50 | Cutting, grinding |
| Calipers (Digital) | $25 | Measuring |

### Workbench Supplies

| Item | Price |
|------|-------|
| Cutting Mat | $15 |
| Clamps (various) | $30 |
| Safety Glasses | $10 |
| Work Gloves | $15 |

**Tools Subtotal**: $200 (printer) + $200 (hand tools) + $155 (power tools) + $70 (supplies) = **$625**
**Note**: Much lower if you already own basic tools

---

## 8. Miscellaneous & Contingency ($500 - $1,000)

### Consumables

| Item | Price |
|------|-------|
| Sandpaper (assorted grits) | $15 |
| Threadlocker (Loctite) | $10 |
| Lubricants | $15 |
| Cleaning supplies | $20 |
| Adhesives (CA glue, epoxy) | $25 |

### Replacement Parts Budget

| Category | Estimated Cost |
|----------|---------------|
| Broken servos | $200 |
| Damaged 3D parts (reprint) | $50 |
| Burned electronics | $100 |
| Misc hardware | $50 |

### Upgrades & Iterations

| Item | Budget |
|------|-------|
| Design improvements | $200 |
| Additional sensors | $150 |
| Better components | $200 |

**Miscellaneous Subtotal**: $85 + $400 + $550 = **$1,035**

---

## Total Cost Summary

| Category | Low Estimate | High Estimate | Recommended |
|----------|-------------|---------------|-------------|
| 1. Actuators & Motors | $1,500 | $5,000 | $3,500 |
| 2. Controllers | $300 | $800 | $305 |
| 3. Sensors | $250 | $1,000 | $634 |
| 4. Power System | $500 | $1,200 | $496 |
| 5. Structural Materials | $800 | $2,000 | $800 |
| 6. Electronics & Wiring | $400 | $700 | $498 |
| 7. Tools & Equipment | $500 | $1,500 | $625 |
| 8. Miscellaneous | $500 | $1,000 | $1,035 |
| **TOTAL** | **$4,750** | **$13,200** | **$7,893** |

**Recommended Budget**: **$8,000 - $10,000** (includes safety margin)

---

## Purchasing Strategy

### Phase 1 (Months 1-3): Research & Initial Purchases
**Budget**: $500-800
- CAD software subscription (if needed)
- One test servo motor (Dynamixel MX-28)
- Arduino/Raspberry Pi starter kit
- Basic sensors (IMU, force sensor)
- Small amount of 3D printing filament

### Phase 2 (Months 4-8): Leg Prototype
**Budget**: $2,000-3,000
- All leg servos (6× high-torque)
- Structural materials for legs
- Power system basics
- 3D printer or printing service setup

### Phase 3 (Months 9-12): Lower Body Completion
**Budget**: $1,500-2,000
- Ankle/foot servos
- Additional sensors (force sensors for feet)
- Complete power system
- Torso structure begins

### Phase 4 (Year 2): Upper Body
**Budget**: $2,000-3,000
- Arm and shoulder servos
- Head/neck servos
- Vision sensors
- Remaining structural materials

### Phase 5 (Year 2-3): Integration & Refinement
**Budget**: $1,000-2,000
- Replacement parts
- Upgrades and improvements
- Additional sensors
- Refinement materials

---

## Supplier Recommendations

### Electronics & Servos
- **RobotShop** - Wide selection of Dynamixel and robotics components
- **Pololu** - Motors, sensors, controllers
- **Adafruit** - Sensors, breakout boards
- **SparkFun** - Electronics and development boards
- **Amazon** - General components, often cheaper
- **AliExpress** - Budget alternatives (longer shipping)

### Mechanical Parts
- **McMaster-Carr** - High-quality hardware, fast shipping (US)
- **Misumi** - Precision parts, aluminum extrusions
- **OnlineMetals** - Aluminum stock
- **SendCutSend** - Custom laser cutting (optional)

### 3D Printing
- **Prusa Research** - Filament and printers
- **MatterHackers** - Variety of filaments
- **Printed Solid** - Premium filaments
- **Local print shops** - For parts you can't print

### Tools
- **Harbor Freight** - Budget tools
- **Home Depot/Lowe's** - General tools
- **Amazon** - Everything else

---

## Notes

- Prices are estimates and will vary by region and supplier
- Consider buying in bulk where possible (fasteners, wire, connectors)
- Watch for sales on major components (servos, controllers)
- Start with budget alternatives and upgrade components as needed
- Keep detailed records of all purchases for budget tracking
- Factor in shipping costs (can be significant)
- Some items can be salvaged or 3D printed instead of purchased

**Last Updated**: 2025-11-26
**Next Review**: After completing initial prototyping phase
