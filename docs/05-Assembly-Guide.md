# Assembly and Build Guide

## Overview

This document provides step-by-step instructions for assembling the 150cm humanoid robot. Follow these instructions carefully and in order for best results.

**Estimated Total Build Time**: 200-300 hours over 24-36 months
**Skill Level Required**: Intermediate (soldering, 3D printing, basic programming)

**Last Updated**: 2025-11-26

---

## Safety First

### Before You Begin

**Personal Safety**:
- Wear safety glasses when cutting, drilling, or soldering
- Use proper ventilation when soldering or working with adhesives
- Wear gloves when handling fiberglass or sharp metal edges
- Never work on live electrical circuits

**Battery Safety**:
- LiPo batteries can catch fire if mishandled
- Never charge batteries unattended
- Use LiPo-safe bags for charging and storage
- Keep fire extinguisher nearby
- Dispose of damaged batteries properly (at hazmat facility)

**Tool Safety**:
- Unplug power tools when changing bits
- Keep soldering iron in stand when not in use
- Use proper eye protection with rotary tools
- Ensure good lighting in work area

---

## Prerequisites

### Required Skills

Before starting, you should be comfortable with:
- Basic electronics (reading schematics, soldering)
- 3D printing (setup, troubleshooting, post-processing)
- CAD software (creating and modifying 3D models)
- Basic programming (Python, C/C++)
- Using hand tools (screwdrivers, pliers, drills)

### Workspace Requirements

**Minimum Space**:
- Workbench: 4 ft × 2 ft (120 cm × 60 cm)
- 3D printer area
- Storage for parts and tools
- Testing area: 6 ft × 6 ft clear space for robot testing

**Environment**:
- Good lighting (overhead + task lighting)
- Ventilation for soldering
- Dry environment (protect electronics from moisture)
- Stable temperature (3D printing works best at 20-25°C)

---

## Build Phases Overview

### Phase 1: Preparation (Weeks 1-4)
- Set up workspace
- Acquire and test tools
- Order initial components
- Learn CAD software
- Study reference projects

### Phase 2: Prototype Single Leg (Months 2-4)
- Design leg in CAD
- 3D print leg parts
- Assemble single leg
- Test servos and joints
- Basic programming (servo control)

### Phase 3: Complete Lower Body (Months 5-8)
- Build second leg
- Design and build pelvis
- Integrate power system
- Implement balance control
- Standing tests

### Phase 4: Torso Construction (Months 9-12)
- Design torso structure
- 3D print torso parts
- Install electronics (Jetson, Arduino, power distribution)
- Wire management
- Integration with lower body

### Phase 5: Arms (Months 13-18)
- Design arms in CAD
- Build both arms
- Integrate with torso
- Test arm movements

### Phase 6: Head (Months 19-20)
- Design head and neck
- Install cameras and sensors
- Mount to torso

### Phase 7: Integration & Walking (Months 21-30)
- Full assembly
- Software development (kinematics, gait)
- Testing and iteration
- Walking implementation

### Phase 8: Refinement (Months 31-36)
- Polish mechanics
- Advanced software features
- Cosmetic improvements
- Documentation

---

## Phase 1: Preparation

### Week 1-2: Workspace Setup

**Tasks**:
1. Clear and organize workbench
2. Install good lighting
3. Set up 3D printer (if purchased)
4. Organize tool storage
5. Create inventory system for parts

**3D Printer Setup** (if new):
- Assemble printer (follow manufacturer instructions)
- Level bed carefully
- Run test prints (calibration cube, benchy)
- Tune settings (temperature, speed, retraction)
- Learn how to handle common issues (warping, stringing)

### Week 3-4: Learning Phase

**CAD Software** (Choose one):
- **Fusion 360** (Recommended):
  - Sign up for free hobbyist license
  - Complete official tutorials (10-20 hours)
  - Practice: Design simple brackets and parts
  - Learn: Sketching, extrusion, joints, assemblies

**Study Reference Projects**:
- InMoov: http://inmoov.fr
  - Download STL files, study design
  - Note how joints are constructed
- Poppy Project: https://www.poppy-project.org
  - Study modular design approach
  - Review documentation

**Initial Component Testing**:
- Order 1-2 Dynamixel servos
- Download Dynamixel Wizard software
- Test servo control via USB
- Learn servo programming basics

---

## Phase 2: Prototype Single Leg (Months 2-4)

### Month 2: Leg Design in CAD

**Step 1: Create Servo Models**

Import or model your servos first (critical for accurate fit):

1. Find or create CAD models for:
   - MX-64AT (hip/knee)
   - MX-28AT (ankle)
2. Measure actual servos with calipers
3. Verify model dimensions match real servos

**Step 2: Design Hip Joint**

```
Hip Assembly (bottom-up):
1. Pelvis mounting plate (3D print, PETG, 5mm thick)
2. Hip Yaw servo mount
   - Create bracket to hold MX-64AT
   - Add mounting holes (M3)
   - Include wire routing channels
3. Hip Pitch bracket
   - Connects to Yaw servo horn
   - Mounts Pitch servo
4. Hip Roll bracket
   - Connects to Pitch servo horn
   - Mounts Roll servo
5. Thigh connection plate
   - Connects to Roll servo horn
```

**Design Checklist**:
- [ ] Clearance: 3-5mm between all moving parts
- [ ] Servo horn: Exact fit (no slop)
- [ ] Mounting holes: +0.5mm for easy assembly
- [ ] Wire channels: 10mm × 10mm minimum
- [ ] Threaded insert locations marked
- [ ] Joint travel: Verify full range of motion (no collisions)

**Step 3: Design Thigh**

Options:
- **Option A**: Aluminum tube (20mm OD) + 3D printed end caps
- **Option B**: Fully 3D printed shell (15mm × 30mm profile)

**Recommendation**: Option B for first prototype (easier to modify)

Design features:
- Internal wire channel (full length)
- Mounting points top (hip) and bottom (knee)
- Wall thickness: 3mm minimum
- Optional: Mounting holes for accessories every 5 cm

**Step 4: Design Knee Joint**

```
Knee Assembly:
1. Thigh connection bracket (top)
2. MX-64AT or XM430-W350 servo
   - Side-mounted to reduce width
   - Bracket holds servo body
3. Shin connection bracket (bottom)
   - Connects to servo horn
```

**Step 5: Design Shin, Ankle, Foot**

Similar to thigh and knee, but lighter construction.

**Ankle**:
- 2-axis design (pitch and roll)
- MX-28AT servos
- Connection to foot plate

**Foot**:
- 20cm × 10cm × 3cm
- Aluminum plate or reinforced PETG
- 4× mounting holes for FSR sensors (corners)
- TPU rubber sole (3mm thick, print separately)

**CAD Assembly**:
1. Create assembly file
2. Add all parts (servos, brackets, tubes)
3. Define joints and constraints
4. Test range of motion (simulation)
5. Check for interferences
6. Generate exploded view for assembly reference

### Month 3: 3D Printing and Part Preparation

**Print Settings** (PETG):
- Layer height: 0.2mm
- Infill: 30-40% (gyroid)
- Wall thickness: 4 perimeters (1.6mm)
- Top/bottom layers: 5
- Speed: 40-50 mm/s
- Supports: Use where needed (avoid if possible with good orientation)

**Print Order** (estimated total: ~800g filament, ~60 hours):

| Part | Quantity | Est. Time | Notes |
|------|----------|-----------|-------|
| Hip brackets (×3) | 3 | 12h | Print orientation: minimize supports |
| Thigh tube | 1 | 8h | Print vertically for strength |
| Knee bracket | 1 | 4h | |
| Shin tube | 1 | 6h | |
| Ankle brackets (×2) | 2 | 6h | |
| Foot plate | 1 | 3h | Print flat, add supports for sensor holes |
| Foot sole (TPU) | 1 | 4h | Slower print speed (20-30mm/s) |

**Post-Processing**:
1. Remove supports carefully (needle-nose pliers)
2. Clean up with hobby knife
3. Sand surfaces if needed (220 grit)
4. Install threaded inserts:
   - Heat soldering iron to 200-220°C
   - Place insert on hole
   - Press gently until flush
   - Let cool completely

**Metal Parts**:
- Cut aluminum tube (if using): 35cm for thigh, 32cm for shin
- Deburr edges (file or sandpaper)
- Optional: Anodize or paint for corrosion resistance

### Month 4: Leg Assembly and Testing

**Tools Needed**:
- Hex key set (for servo horns and brackets)
- Screwdrivers (Phillips and flat)
- Threadlocker (Loctite 243 blue)
- Zip ties
- Heat shrink tubing
- Wire (18 AWG power, 22 AWG signal)

**Assembly Steps**:

**Step 1: Prepare Servos**

For each servo:
1. Test servo operation (Dynamixel Wizard)
2. Set servo ID (1-6 for right leg)
3. Center servo (position 2048 for Dynamixel)
4. Attach servo horn in centered position
5. Label servo with ID (masking tape and marker)

**Step 2: Assemble Hip Joint**

```
1. Install Hip Yaw servo (ID:1)
   - Bolt servo to pelvis mounting plate
   - M3 × 8mm bolts + lock washers
   - Apply threadlocker

2. Attach Pitch bracket to Yaw servo horn
   - Center horn on servo
   - M2.5 × 6mm bolts (come with servo)
   - Check rotation: Should be smooth, no binding

3. Install Hip Pitch servo (ID:2)
   - Mount to Pitch bracket
   - Connect servo horn to Roll bracket

4. Install Hip Roll servo (ID:3)
   - Mount to Roll bracket
   - Connect servo horn to Thigh connection plate

5. Check range of motion
   - Manually move each joint
   - Verify no collisions
   - Ensure smooth motion throughout range
```

**Step 3: Attach Thigh**

1. Connect thigh to hip roll servo horn (M3 bolts)
2. Route servo wires through thigh tube
3. Secure wires with zip ties every 10cm
4. Leave service loop at hip (extra 5cm slack)

**Step 4: Assemble Knee**

1. Install knee servo (ID:4) to knee bracket
2. Bolt knee bracket to thigh bottom
3. Connect servo horn to shin bracket
4. Route wires through shin

**Step 5: Assemble Ankle and Foot**

1. Install ankle roll servo (ID:6) to ankle bracket
2. Install ankle pitch servo (ID:5)
3. Attach foot plate
4. Install FSR sensors (corners of foot):
   - Apply double-sided tape
   - Solder wires (4× 2-wire leads)
   - Route wires up through shin
5. Attach TPU sole (hot glue or contact cement)

**Step 6: Wire Management**

Create main wire bundle in leg:
- 6× servo power (red/black, 18 AWG)
- 6× servo signal (3-wire, 22 AWG)
- 4× FSR sensor wires (2-wire each, 26 AWG)

Bundle with:
- Spiral wrap or flat ribbon cable organizer
- Zip ties every 10 cm
- Heat shrink at connectors

**Step 7: Power and Control Electronics**

For testing single leg:
1. Assemble simple test setup:
   - 1× 12V power supply (bench supply or battery)
   - 1× Dynamixel U2D2 (USB to TTL)
   - 1× PC with Dynamixel Wizard
2. Connect daisy-chain:
   - U2D2 → Servo 1 → Servo 2 → ... → Servo 6
3. Power: Connect 12V to all servo power pins (parallel)

**Step 8: Testing**

**Individual Servo Test**:
1. Open Dynamixel Wizard
2. Scan for servos (should find IDs 1-6)
3. Test each servo:
   - Move to different positions
   - Check current draw (should be <0.5A at no load)
   - Verify temperature (should stay cool)
4. Document servo positions at key angles

**Full Leg Test**:
1. Mount leg vertically in vise or test stand
2. Write simple test program (or use Wizard):
   ```python
   # Test script (Python + Dynamixel SDK)
   # Move through predefined positions
   positions = [
       [2048, 2048, 2048, 2048, 2048, 2048],  # All centered
       [2048, 2248, 2048, 1024, 2048, 2048],  # Hip pitch + knee flex
       # ... more positions
   ]
   for pos in positions:
       move_servos(pos)
       time.sleep(2)
   ```
3. Observe:
   - Smooth motion
   - No binding or jerking
   - Servos not overheating
   - Proper range of motion

**Load Testing**:
1. Attach weight to foot (2-3 kg)
2. Test hip and knee servos under load
3. Measure current draw (should stay under 1.5A)
4. Check for any mechanical slippage

**Success Criteria**:
- [ ] All servos respond correctly
- [ ] Full range of motion achieved
- [ ] No mechanical binding
- [ ] No overheating
- [ ] Wiring secure and organized
- [ ] Leg can support expected loads

---

## Phase 3: Complete Lower Body (Months 5-8)

### Month 5: Build Second Leg

**Repeat Phase 2** for left leg:
- Mirror the right leg design in CAD
- Print all parts
- Assemble with IDs 7-12
- Test independently

**Tip**: Now that you've built one leg, the second should go faster (~half the time)

### Month 6: Pelvis Design and Construction

**Pelvis Requirements**:
- Mount both legs (hip joints)
- House lower torso electronics
- Provide stable base
- Allow waist joint attachment

**Design**:

```
Pelvis Structure:
┌─────────────────────────┐
│   [Battery Compartment] │
│                         │
│ Hip_Left      Hip_Right │
└─────────────────────────┘
     /               \
  Leg Left         Leg Right

Dimensions: 25cm wide × 16cm deep × 15cm tall
```

**Components**:
1. Base plate (3mm aluminum or PETG)
2. Side walls (3D printed)
3. Hip mounting brackets (reinforced)
4. Battery tray (removable)
5. Wire routing channels
6. Waist connection (top)

**Assembly**:
1. Build pelvis frame
2. Install hip joints (already assembled with legs)
3. Test standing position:
   - Both legs vertical
   - Feet flat on ground
   - Even weight distribution

### Month 7: Power System Integration

**Build Power Distribution Board**:

See Electrical Design doc for schematic. Summary:
1. Use protoboard or custom PCB
2. Install:
   - XT90 input connector
   - Main fuse (40A)
   - Emergency stop switch
   - 4× buck converters (2× 12V, 2× 5V)
   - Output terminals
   - Voltage monitoring circuit
3. Test:
   - Connect battery
   - Verify all outputs: 12V and 5V
   - Load test with servos

**Battery Installation**:
1. Create battery mount in pelvis
   - Velcro straps (removable)
   - Ventilation holes
   - BMS integration
2. Wire battery to power distribution board
3. Add voltage monitoring to Arduino

**First Powered Test**:
1. Connect battery
2. Power on (main switch)
3. Verify:
   - All power rails active
   - LEDs lit
   - No smoke or unusual smells!
4. Test servos on battery power (all 12 leg servos)

### Month 8: Basic Balance Control

**Electronics Integration**:
1. Install Arduino Mega in pelvis
2. Mount IMU (BNO055) at pelvis center
3. Wire:
   - FSR sensors (8 analog inputs)
   - IMU (I2C)
   - Dynamixel servos (TTL serial)
   - Emergency stop button

**Software Development**:
1. Arduino firmware:
   - Read IMU data
   - Read FSR sensors
   - Control servos via Dynamixel protocol
   - Basic balance algorithm (PID)
2. Test standing:
   - Robot stands still
   - Balance controller active
   - Manually push robot (should resist)

**Standing Tests**:
1. Initial test: Use support frame or hang from above
2. Gradually reduce support
3. Full standing (no support):
   - Duration: 30+ seconds
   - Stability: No excessive swaying
   - Balance: Recovers from small pushes

---

## Phase 4: Torso Construction (Months 9-12)

### Month 9-10: Torso Design and Build

**Frame Construction**:
1. Cut aluminum extrusions:
   - 4× vertical posts (45cm)
   - 4× horizontal (top, 2× middle, bottom)
2. Assemble with corner brackets
3. 3D print panels:
   - Front (access door)
   - Back (removable)
   - Sides (solid or vented)

**Interior Layout**:
```
Top: Jetson Nano + Cooling Fan
Mid-Upper: Servo controller boards
Mid-Lower: Waist joint servos
Bottom: Connection to pelvis
```

**Waist Joint**:
- 2× MX-28AT servos (yaw and pitch)
- Allows torso rotation and lean
- Strong connection (supports upper body weight)

### Month 11: Electronics Installation

**Components to Install**:
1. Jetson Nano (top section)
   - Heat sink + cooling fan
   - Mount on standoffs
   - microSD card installed
2. Arduino Mega (if not in pelvis, move to torso)
3. Additional buck converters (if needed)
4. Second IMU (optional, for torso)

**Wiring**:
1. Power from pelvis (run through waist joint)
2. Signal wires from pelvis (servos, sensors)
3. Internal torso wiring:
   - USB: Jetson to Arduino
   - Power: To all components
   - Servo connections (arms, head - to be added)
4. Cable management:
   - Create wire harness backbone
   - Label all wires
   - Use zip ties and velcro

### Month 12: Torso-Lower Body Integration

**Mechanical**:
1. Connect torso to pelvis via waist joint
2. Test waist motion (rotation, lean)
3. Verify stability

**Electrical**:
1. Connect all power and signal wires
2. Test complete system:
   - All leg servos
   - Waist servos
   - All sensors (IMU, FSR)

**Software**:
1. Update firmware for waist control
2. Test complete lower body + torso:
   - Standing
   - Waist rotation
   - Lean forward/back
   - Weight shifting

---

## Phase 5: Arms (Months 13-18)

### Month 13-15: Arm Design and Build

**Repeat similar process to legs**:
1. Design in CAD:
   - Shoulder (3-DOF, similar to hip)
   - Upper arm (25cm)
   - Elbow (1-DOF)
   - Forearm (22cm)
   - Wrist (2-DOF)
   - Gripper
2. 3D print all parts
3. Assemble one arm (test)
4. Build second arm

**Gripper Design**:

Simple 2-finger gripper:
```
Components:
- Palm plate
- 2× finger linkages (3D printed)
- 1× AX-12A servo
- TPU finger pads

Mechanism:
- Servo rotates
- Linkage converts rotation to linear finger motion
- Fingers open/close in parallel
```

### Month 16-18: Arm Integration

**Mounting**:
1. Attach shoulder assemblies to torso (both sides)
2. Wire arms (similar to legs):
   - Power wires
   - Servo signal wires
   - Bundle and route through shoulder

**Testing**:
1. Test each arm independently
2. Test both arms together
3. Test arm-torso coordination
4. Gripper tests:
   - Grip force measurement
   - Pick up objects (various sizes, weights)
   - Release reliability

---

## Phase 6: Head (Months 19-20)

### Month 19: Head Design and Build

**Head Shell**:
1. Design aesthetic shell (humanoid or robot-like)
2. Include mounting for:
   - 2× cameras (eyes)
   - Sensors (ultrasonic, LEDs)
   - Microphone/speaker (optional)
3. Lightweight (minimize neck load)

**Neck Mechanism**:
- 3-DOF (pan, tilt, roll)
- 3× AX-12A servos
- Compact design

**Cameras**:
- 2× Raspberry Pi Camera Module V3
- Stereo baseline: 6-7 cm
- Forward-facing, slight downward tilt

### Month 20: Head Integration

**Installation**:
1. Mount neck to torso top
2. Wire head (power, servos, cameras)
   - Route through neck (rotating joint - use slip ring or service loop)
3. Connect cameras to Jetson Nano (CSI ports)

**Testing**:
1. Neck motion (smooth pan/tilt)
2. Camera functionality:
   - Capture images
   - Stereo vision
   - Proper exposure and focus

---

## Phase 7: Integration & Walking (Months 21-30)

### Month 21-24: Full Integration

**Complete Assembly**:
1. All mechanical parts assembled
2. All wiring complete and organized
3. All electronics installed and tested

**System Checkout**:
1. Power system test (full load)
2. All servos responding
3. All sensors functional
4. Emergency stop tested
5. Software communication (Jetson ↔ Arduino)

### Month 25-28: Walking Implementation

**Software Development**:
1. Implement kinematics (FK/IK)
2. Implement balance controller (tuned)
3. Implement gait generator
4. Integrate all subsystems

**Progressive Testing**:
1. **Standing** (already done)
2. **Weight shifting**:
   - Shift CoM left/right
   - Prepare for single-leg support
3. **Single-leg balance**:
   - Lift one foot slightly
   - Balance on one leg (even briefly)
4. **Stepping in place**:
   - Lift foot, lower in same spot
   - Alternate legs
5. **Forward stepping**:
   - Small steps (5-10cm)
   - Increase gradually
6. **Continuous walking**:
   - Multiple steps in sequence
   - Turn walking into continuous gait

**Safety During Testing**:
- Use overhead support (rope/harness)
- Padded testing area
- Emergency stop always accessible
- Start slow, increase speed gradually

### Month 29-30: Refinement

**Gait Optimization**:
1. Tune PID parameters
2. Adjust step height, length, timing
3. Optimize power consumption
4. Increase walking speed

**Testing Scenarios**:
- Walk forward
- Walk backward
- Turn in place
- Walk and stop
- Walk on slight inclines
- Recover from disturbances

---

## Phase 8: Refinement (Months 31-36)

### Mechanical Improvements

**Identify Weak Points**:
- Parts that broke or cracked
- Joints with slop or backlash
- Areas of excessive wear

**Upgrades**:
- Reinforce weak parts (thicker walls, carbon fiber)
- Replace worn bearings
- Improve wire routing
- Add cosmetic covers

### Software Enhancements

**Advanced Features**:
- Vision-based navigation
- Object recognition
- Autonomous behaviors
- Remote control (WiFi/Bluetooth)
- Voice control (optional)

### Documentation

**As-Built Documentation**:
1. Photograph all assemblies
2. Record final CAD designs
3. Document wiring (create diagram)
4. Write software API documentation
5. Create user manual
6. Make demo videos

---

## Troubleshooting Guide

### Common Issues During Assembly

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| 3D printed parts don't fit | Tolerance issues, warping | Sand/file parts, adjust CAD model |
| Servo horn slipping | Loose screw, worn splines | Tighten, replace horn, use thread lock |
| Joint binding | Misalignment, no clearance | Check assembly, add clearance, sand |
| Servo overheating | Overload, binding, low voltage | Reduce load, fix binding, check power |
| Wires breaking | Excessive movement, sharp bends | Add service loops, larger bend radius |
| Battery draining quickly | High current draw, old battery | Check for shorts, replace battery |
| IMU drift | Magnetic interference, calibration | Recalibrate, move away from motors |
| Servos not responding | Wiring, ID conflict, protocol error | Check wiring, verify IDs, test individually |

---

## Assembly Tips & Best Practices

### General

1. **Test Before Permanent Assembly**:
   - Dry-fit all parts first
   - Test electronics before installing in tight spaces
   - Program and test servos before mounting

2. **Label Everything**:
   - Servos (ID numbers)
   - Wires (source and destination)
   - Connectors (mating pairs)
   - Parts (left/right, version number)

3. **Take Photos**:
   - Document each assembly step
   - Useful for troubleshooting and reassembly
   - Share progress online for feedback

4. **Version Control**:
   - Save CAD models with version numbers
   - Keep old versions (in case you need to revert)
   - Document changes in a log file

5. **Measure Twice, Cut Once**:
   - Verify dimensions before cutting
   - Check servo positions before printing
   - Test range of motion in CAD before building

### 3D Printing

1. **Print Orientation**:
   - Orient for strength (layer lines perpendicular to stress)
   - Minimize supports (saves material and post-processing time)
   - Flat surfaces on bed (better adhesion)

2. **First Layer is Critical**:
   - Clean bed before each print
   - Proper bed leveling
   - Correct bed temperature
   - Good "squish" (nozzle height)

3. **Large Prints**:
   - Add brim or raft for adhesion
   - Monitor first few layers
   - Ensure enclosure/consistent temperature
   - Have backup plan if print fails

### Electronics

1. **Bench Test First**:
   - Test all electronics on bench before installing
   - Verify voltages before connecting components
   - Use multimeter frequently

2. **Strain Relief**:
   - All wires should have strain relief at connectors
   - Use zip ties, heat shrink, or cable glands
   - No sharp bends (minimum 10mm radius)

3. **Avoid Ground Loops**:
   - Single ground point for power system
   - Star topology for ground connections
   - Twisted pairs for sensitive signals

4. **Static Protection**:
   - Use anti-static wrist strap
   - Work on anti-static mat
   - Store electronics in anti-static bags

---

## Appendix: Tool List

### Essential Tools

**Hand Tools**:
- [ ] Screwdriver set (Phillips, flat, precision)
- [ ] Hex key set (metric, 1.5mm - 6mm)
- [ ] Pliers (needle-nose, cutting, standard)
- [ ] Wire strippers
- [ ] Tweezers
- [ ] Hobby knife (X-Acto)
- [ ] Files (flat, round, needle files)
- [ ] Calipers (digital, 150mm)
- [ ] Ruler/straight edge

**Power Tools**:
- [ ] Drill (corded or cordless)
- [ ] Drill bits (metric set)
- [ ] Dremel/rotary tool
- [ ] Soldering iron (adjustable temp, 60W+)

**Electronics**:
- [ ] Multimeter
- [ ] Power supply (bench, 0-30V, 5A)
- [ ] Wire (various gauges)
- [ ] Heat shrink tubing kit
- [ ] Solder + flux
- [ ] Helping hands
- [ ] Anti-static wrist strap

**3D Printing**:
- [ ] 3D printer
- [ ] Filament (PETG, TPU)
- [ ] Spatula (for print removal)
- [ ] Isopropyl alcohol (bed cleaning)
- [ ] Needle files (print cleanup)

### Nice to Have

- Oscilloscope (signal debugging)
- Label maker (wire labeling)
- Heat gun (heat shrink, print post-processing)
- Tap and die set (threading)
- Vise (holding parts)
- Magnifying glass / loupe
- Cable tester
- Digital angle gauge

---

## Appendix: Fastener Reference

### Metric Bolt Sizes

| Size | Drill Hole (clearance) | Thread Insert | Common Use |
|------|------------------------|---------------|------------|
| M2 | 2.2-2.4mm | M2 insert | Servo horns, small brackets |
| M2.5 | 2.7-2.9mm | M2.5 insert | Servo horns (Dynamixel) |
| M3 | 3.2-3.4mm | M3 insert | Most brackets, general assembly |
| M4 | 4.2-4.5mm | M4 insert | Load-bearing joints |
| M5 | 5.2-5.5mm | N/A (thread directly) | Aluminum frame |

### Bolt Lengths

General rule: Bolt length = Material thickness + nut/thread engagement + 2mm

Example: 5mm bracket + 3mm nut = M3 × 10mm bolt

---

**Document Status**: Initial Draft
**Next Review**: After leg prototype completion
**Maintained By**: Project owner
