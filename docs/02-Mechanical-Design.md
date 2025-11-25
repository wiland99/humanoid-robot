# Mechanical Design Specifications

## Overview

This document outlines the complete mechanical design for the 150cm humanoid robot, including dimensions, materials, joint specifications, and design principles.

**Last Updated**: 2025-11-26

---

## Design Philosophy

### Core Principles

1. **Modularity**: Each limb and subsystem should be independently serviceable
2. **Lightweight**: Minimize weight while maintaining structural integrity
3. **Accessibility**: Easy access to electronics and wiring for maintenance
4. **Safety**: Rounded edges, proper clearances, emergency stop capabilities
5. **Scalability**: Design allows for future improvements and modifications
6. **Manufacturability**: Primarily 3D printed with minimal custom machining

### Material Selection Strategy

- **3D Printed Parts**: PETG for structural components (strong, less brittle than ABS)
- **Metal Frame**: Aluminum extrusions and flat bar for main skeleton
- **Flexible Parts**: TPU for foot pads, gripper surfaces
- **Reinforcement**: Threaded inserts in 3D printed parts for durability
- **Panels**: Thin aluminum or 3D printed for cosmetic covers

---

## Overall Dimensions & Proportions

### Total Height: 150 cm

Based on human body proportions (adjusted for robotics):

```
HEAD (20 cm - 13.3%)
├─ Skull/Housing: 15 cm
└─ Neck: 5 cm

TORSO (45 cm - 30%)
├─ Upper Torso: 25 cm
├─ Waist Joint: 5 cm
└─ Lower Torso/Pelvis: 15 cm

ARMS (55 cm each - 36.7%)
├─ Shoulder to Elbow: 25 cm
├─ Elbow to Wrist: 22 cm
└─ Wrist to Fingertips: 8 cm

LEGS (75 cm - 50%)
├─ Hip to Knee: 35 cm
├─ Knee to Ankle: 32 cm
└─ Ankle to Ground: 8 cm
```

### Width Dimensions

```
Shoulder Width: 35 cm
Hip Width: 25 cm
Chest Depth: 18 cm
Hip Depth: 16 cm
Head Width: 15 cm
Foot Length: 20 cm
Foot Width: 10 cm
```

### Weight Distribution (Target)

| Section | Weight (kg) | Percentage |
|---------|------------|------------|
| Head | 0.8 | 8% |
| Torso (incl. battery) | 3.5 | 35% |
| Arms (both) | 2.0 | 20% |
| Legs (both) | 3.7 | 37% |
| **Total** | **10.0** | **100%** |

**Center of Mass**: Approximately 70-75 cm from ground (mid-torso) when standing

---

## Degrees of Freedom (DOF) Detailed Breakdown

### Total: 27 DOF

#### Lower Body: 14 DOF

**Left Leg (7 DOF)**
1. Hip Flexion/Extension (pitch): -30° to +120°
2. Hip Abduction/Adduction (roll): -45° to +45°
3. Hip Rotation (yaw): -45° to +45°
4. Knee Flexion/Extension: 0° to 140°
5. Ankle Dorsiflexion/Plantarflexion: -30° to +45°
6. Ankle Inversion/Eversion: -20° to +20°
7. (Optional) Toe Joint: -20° to +20°

**Right Leg (7 DOF)**: Mirror of left leg

#### Upper Body: 13 DOF

**Left Arm (5 DOF)**
1. Shoulder Flexion/Extension: -30° to +180°
2. Shoulder Abduction/Adduction: -30° to +180°
3. Shoulder Rotation: -90° to +90°
4. Elbow Flexion/Extension: 0° to 150°
5. Wrist Rotation/Roll: -90° to +90°
6. (Wrist Pitch): -90° to +90° [combined with gripper]
7. Gripper/Hand: 0° to 90° (open/close)

**Right Arm (5 DOF)**: Mirror of left arm

**Torso (2 DOF)**
1. Waist Yaw (rotation): -45° to +45°
2. Waist Pitch (lean): -20° to +20°

**Head/Neck (3 DOF)**
1. Neck Yaw (pan): -90° to +90°
2. Neck Pitch (tilt): -45° to +45°
3. Neck Roll: -30° to +30°

---

## Leg Assembly Design

### Design Goals
- Support full robot weight (10 kg) + safety factor (2×) = 20 kg per leg
- Smooth, stable walking gait
- Good ground clearance (8 cm)
- Wide stance for stability

### Hip Joint (3 DOF)

**Construction**: Universal joint with 3 servo motors

```
Component Stack (top to bottom):
1. Mounting Plate to Pelvis (3mm aluminum)
2. Yaw Servo (MX-64AT) - Hip rotation
   - Torque: 6.0 Nm
   - Output: Servo horn to pitch bracket
3. Pitch Bracket (PETG 3D print)
4. Pitch Servo (MX-64AT) - Hip flexion/extension
   - Torque: 6.0 Nm
   - Output: Servo horn to roll bracket
5. Roll Bracket (PETG 3D print)
6. Roll Servo (MX-64AT) - Hip abduction/adduction
   - Torque: 6.0 Nm
   - Output: Connection to thigh
```

**Dimensions**:
- Hip assembly width: 12 cm
- Hip assembly height: 14 cm
- Hip assembly depth: 10 cm
- Total weight per hip: ~650g

### Thigh Section

**Length**: 35 cm (hip to knee)
**Structure**:
- Main tube: Aluminum tube 20mm OD, 1.5mm wall thickness
- Alternative: 3D printed PETG shell (15mm × 30mm rectangular profile)

**Internal Components**:
- Wiring channel (10mm × 10mm)
- Optional: Reinforcement ribs every 10 cm

**Weight**: ~200g per thigh

### Knee Joint (1 DOF)

**Construction**: Single axis hinge

```
Component:
1. Knee Bracket (3D printed PETG, reinforced)
2. Pitch Servo (XM430-W350 or MX-64AT)
   - Torque: 4.1 Nm (or 6.0 Nm for heavier builds)
   - Mounting: Side-mounted to reduce profile
3. Connection to thigh (top) and shin (bottom)
```

**Dimensions**:
- Knee width: 10 cm
- Knee depth: 12 cm
- Knee height (assembled): 8 cm
- Weight: ~350g

### Shin Section

**Length**: 32 cm (knee to ankle)
**Structure**: Same as thigh
- Aluminum tube or 3D printed shell
- Lighter gauge acceptable (less load)

**Weight**: ~180g per shin

### Ankle Joint (2 DOF)

**Construction**: Two-axis joint

```
Component Stack:
1. Shin mounting bracket
2. Roll Servo (MX-28AT) - Inversion/eversion
   - Torque: 2.5 Nm
   - Side-mounted
3. Pitch Servo (MX-28AT) - Dorsiflexion/plantarflexion
   - Torque: 2.5 Nm
   - Front-mounted
4. Foot plate connection
```

**Dimensions**:
- Ankle width: 9 cm
- Ankle depth: 10 cm
- Weight: ~280g

### Foot Design

**Dimensions**: 20 cm (L) × 10 cm (W) × 3 cm (H)

**Structure**:
```
Layers (bottom to top):
1. Rubber/TPU sole (2-3mm) - Grip and shock absorption
2. Sensor layer - 4× FSR force sensors (corners)
3. Foot plate (3mm aluminum or reinforced PETG)
4. Ankle connection bracket
```

**Sensor Placement**:
- Front-left corner (toe lateral)
- Front-right corner (toe medial)
- Rear-left corner (heel lateral)
- Rear-right corner (heel medial)

**Weight**: ~150g per foot (including sensors)

**Total Leg Weight**: 650g + 200g + 350g + 180g + 280g + 150g = **1,810g per leg**

---

## Arm Assembly Design

### Design Goals
- Sufficient reach and workspace
- 1 kg payload capacity per arm
- Smooth, controlled movements
- Compact design to minimize inertia

### Shoulder Joint (3 DOF)

**Construction**: Similar to hip, smaller servos

```
Component Stack:
1. Mounting to upper torso
2. Yaw Servo (MX-28AT) - Shoulder rotation
3. Pitch Bracket
4. Pitch Servo (MX-28AT) - Shoulder flexion/extension
5. Roll Bracket
6. Roll Servo (MX-28AT) - Shoulder abduction/adduction
7. Connection to upper arm
```

**Dimensions**:
- Shoulder assembly: 10 cm × 10 cm × 8 cm
- Weight: ~520g per shoulder

### Upper Arm

**Length**: 25 cm (shoulder to elbow)
**Structure**:
- 3D printed PETG tube (12mm × 25mm oval profile)
- Wall thickness: 3mm
- Internal wire routing

**Weight**: ~120g per upper arm

### Elbow Joint (1 DOF)

**Construction**: Single axis

```
Component:
1. Elbow bracket (3D printed)
2. Pitch Servo (MX-28AT)
   - Torque: 2.5 Nm
3. Connection points
```

**Dimensions**:
- Elbow: 8 cm × 7 cm × 6 cm
- Weight: ~180g per elbow

### Forearm

**Length**: 22 cm (elbow to wrist)
**Structure**:
- 3D printed PETG tube (10mm × 22mm oval)
- Lighter than upper arm
- Wire routing to wrist/hand

**Weight**: ~100g per forearm

### Wrist Joint (2 DOF combined with gripper)

**Construction**: Compact 2-axis

```
Component:
1. Forearm connection
2. Roll Servo (AX-12A) - Wrist rotation
3. Gripper Servo (AX-12A) - Open/close
   - Can include pitch via gripper mechanism
```

**Dimensions**:
- Wrist: 6 cm × 5 cm × 5 cm
- Weight: ~120g

### Gripper/Hand

**Design**: Simple 2-finger gripper (can upgrade later to multi-finger)

**Structure**:
```
Components:
1. Palm plate (3D printed)
2. Two finger assemblies (mirrored)
   - Powered by single servo via linkage
   - 3-segment fingers (optional)
3. TPU fingertip pads for grip
```

**Grip Range**: 0 cm (closed) to 8 cm (open)
**Grip Force**: ~5-8 N (sufficient for light objects)

**Dimensions**:
- Hand length: 8 cm
- Hand width: 7 cm (open)
- Weight: ~80g per hand

**Total Arm Weight**: 520g + 120g + 180g + 100g + 120g + 80g = **1,120g per arm**

---

## Torso Design

### Design Goals
- House main electronics (computer, batteries, controllers)
- Provide stable platform for arms and head
- Allow waist articulation for better balance
- Provide access panels for maintenance

### Structure

**Dimensions**: 35 cm (W) × 18 cm (D) × 45 cm (H)

**Division**:
```
Upper Torso (25 cm height):
- Shoulder mounts (left/right)
- Main computer (Jetson Nano)
- Camera/sensor breakout boards
- Cooling fan

Waist Joint (5 cm):
- Yaw servo (MX-28AT) - Rotation
- Pitch servo (MX-28AT) - Lean forward/back
- Power distribution board

Lower Torso/Pelvis (15 cm):
- Battery compartment (main)
- Motor controller boards
- Buck converters
- Hip mounting points (left/right)
```

### Construction

**Frame**:
- 2020 aluminum extrusion framework
- Corner brackets and gussets
- Vertical: 4 posts
- Horizontal: Top, middle (×2), bottom plates

**Panels**:
- Front: 3D printed PETG (access panels with snap fits)
- Back: 3D printed PETG (removable for wiring)
- Sides: Thin aluminum or 3D printed
- Top: Integrated with neck mount

**Internal Layout**:
```
Top Section:
┌─────────────────────┐
│  Neck Mount (center)│
│  [Jetson] [Cooling] │
│  Shoulder_L   Shoulder_R
│                      │
└─────────────────────┘

Mid Section (waist joint):
┌─────────────────────┐
│  [Waist Servos]     │
│  [Power Dist Board] │
└─────────────────────┘

Bottom Section:
┌─────────────────────┐
│  [Battery_Primary]  │
│  [Motor Controllers]│
│  Hip_L        Hip_R │
└─────────────────────┘
```

**Weight**: ~3,500g (including electronics and battery)

---

## Head Design

### Design Goals
- House cameras and sensors
- 3-DOF neck for natural head movement
- Lightweight to reduce neck servo load
- Aesthetic appearance (humanoid features optional)

### Dimensions

- Head: 15 cm (W) × 15 cm (H) × 18 cm (D)
- Neck: 8 cm (W) × 5 cm (H)

### Structure

**Head Shell**:
- 3D printed PETG (thin walls, 2-3mm)
- Modular: Front face, back, top pieces
- Snap-fit or bolt assembly

**Neck Joint**:
```
Component Stack (bottom to top):
1. Mount to upper torso
2. Yaw Servo (AX-12A) - Pan
   - Torque: 1.5 Nm
3. Tilt Bracket
4. Pitch Servo (AX-12A) - Tilt
   - Torque: 1.5 Nm
5. Roll Bracket (optional)
6. Roll Servo (AX-12A) - Roll
   - Torque: 1.5 Nm
7. Head mounting plate
```

### Internal Components

- 2× Raspberry Pi Camera Module V3 (stereo vision)
- 1× BNO055 IMU (head stabilization)
- 1× Ultrasonic sensor (front)
- LED indicators (eyes/status)
- Microphone (optional)
- Speaker (optional)

**Camera Placement**:
- Eye spacing: 6-7 cm apart (stereo baseline)
- Height from bottom: 10 cm
- Forward facing, slight downward tilt (10°)

**Weight**: ~350g (head) + 450g (neck assembly) = **800g total**

---

## Material Specifications

### 3D Printing Guidelines

**PETG (Primary Structural)**:
- Layer Height: 0.2mm (balance of strength and speed)
- Infill: 30-40% (gyroid or grid pattern)
- Wall Thickness: 3-4 perimeters (1.2-1.6mm)
- Top/Bottom Layers: 5 layers minimum
- Print Speed: 40-50 mm/s
- Temperature: 230-240°C nozzle, 70-80°C bed
- Use for: Brackets, housings, structural tubes

**ABS (Alternative)**:
- Similar settings to PETG
- Requires enclosure for larger parts
- Better heat resistance
- More brittle than PETG

**TPU (Flexible Parts)**:
- Layer Height: 0.2-0.3mm
- Infill: 20-30%
- Print Speed: 20-30 mm/s (slow for accuracy)
- Temperature: 220-230°C nozzle, 60°C bed
- Use for: Foot pads, gripper pads, shock absorption

### Metal Components

**Aluminum 6061-T6**:
- Tensile Strength: 310 MPa
- Yield Strength: 276 MPa
- Density: 2.70 g/cm³
- Use for: Main frame, critical load-bearing parts

**Fasteners**:
- Stainless Steel (A2/304): Corrosion resistant
- Strength Class 8.8 or higher for critical joints
- Use threadlocker (Loctite 243) on all threaded connections

### Reinforcement Techniques

**Threaded Inserts**:
- Use heat-set brass inserts in all 3D printed parts
- Sizes: M3 (most common), M4 (high load), M2 (small parts)
- Install with soldering iron or insertion tool

**Composite Reinforcement** (if needed):
- Carbon fiber strips in high-stress areas
- Fiberglass mat for large panels
- Embedded in epoxy resin

---

## Joint Design Details

### Servo Mounting Methods

**Type 1: Direct Mount**
- Servo body bolted directly to bracket
- Servo horn connects to next link
- Use: Simple single-DOF joints (knee, elbow)

**Type 2: Side Mount**
- Servo mounted perpendicular to rotation axis
- Output shaft via bevel gears or belt
- Use: Compact profiles, reduced width

**Type 3: Embedded**
- Servo housed within the link structure
- Output through gear reduction
- Use: High torque joints (hip pitch)

### Bearing Integration

**Radial Loads** (perpendicular to shaft):
- Use 608 ball bearings (skateboard bearings)
- Press fit into 3D printed or aluminum housings
- 22mm OD, 8mm ID, 7mm width

**Thrust Loads** (along shaft axis):
- Use thrust bearings at shoulders and hips
- Reduces friction in rotating joints

**Bushings** (low-load applications):
- Nylon or bronze bushings for light joints
- Less expensive than bearings
- Use: Wrist, fingers

### Gear Reduction (Optional)

For increased torque at the cost of speed:
- **Ratio**: 2:1 or 3:1 typical
- **Type**: Spur gears (simple), planetary (compact)
- **Material**: 3D printed PLA or PETG (sufficient for low loads)
- **Use case**: If using cheaper, lower-torque servos

---

## Cable Routing & Wire Management

### Routing Strategy

**Principle**: All wires run through internal channels, not externally

**Leg Routing**:
```
Hip Servos → Internal thigh tube → Knee area → Internal shin tube → Ankle → Foot
- Bundle: Power (2 wires), Signals (3 wires per servo), Sensor (4-6 wires)
- Total per leg: ~30-40 wires
- Group in flat ribbon or spiral wrap
```

**Arm Routing**:
```
Shoulder → Upper arm tube → Elbow → Forearm tube → Wrist → Hand
- Bundle: Similar to leg, fewer wires (~20-25)
```

**Torso**:
- Central wire management spine
- Zip tie mounting points every 5 cm
- Service loops at all joints
- Color coding: Red/Black (power), colored (signals)

### Connector Strategy

**Quick Disconnects**:
- JST or Dupont connectors at each joint
- Allows limb removal for service
- Label all connectors clearly

**Strain Relief**:
- Rubber grommets at entry/exit points
- Slack loops before connectors
- No sharp bends (min 10mm radius)

---

## Center of Mass & Stability

### Static Stability

**Standing Position**:
- Center of Mass (CoM) projection must stay within support polygon (foot area)
- Support polygon: Rectangle between both feet
- Typical stance: Feet 20-25 cm apart

**Stability Margin**:
- Minimum distance from CoM projection to support polygon edge: 3 cm
- Larger margin = more stable but less dynamic

### Dynamic Stability (Walking)

**Zero Moment Point (ZMP)**:
- Point where net moment from ground reaction forces is zero
- Must stay within support polygon during walking
- Control strategy: Shift CoM before lifting foot

**Balance Sensors**:
- IMU in torso: Detect tilt/acceleration
- Force sensors in feet: Measure ground reaction forces
- Feedback to adjust posture in real-time

---

## Assembly Sequence Planning

### Recommended Build Order

1. **Prototype Single Leg**
   - Hip → Thigh → Knee → Shin → Ankle → Foot
   - Test: Standing support, range of motion

2. **Complete Lower Body**
   - Pelvis structure
   - Install second leg
   - Basic electronics (balance control)
   - Test: Standing, weight shifting

3. **Build Torso**
   - Frame assembly
   - Electronics integration
   - Mount to pelvis
   - Test: Standing with upper body weight

4. **Build Arms** (parallel task)
   - One arm completely
   - Test: Range of motion, gripper
   - Second arm (replicate)

5. **Build Head**
   - Neck mechanism
   - Head shell and sensors
   - Test: Camera views, tracking

6. **Final Integration**
   - Mount arms to torso
   - Mount head to torso
   - Complete all wiring
   - Full system calibration

---

## Design Tolerances & Clearances

### Critical Dimensions

**Joint Clearances**:
- Minimum: 2mm between moving parts
- Recommended: 3-5mm for safety
- Account for: 3D print tolerance (±0.2mm), assembly tolerance

**Shaft Fits**:
- Servo output shaft: Design horn holes at exact diameter (tight fit)
- Bearing press fits: 0.1mm interference
- Bolt holes: +0.5mm for easy assembly

**Range of Motion**:
- Design mechanical stops 5° before servo limits
- Software limits 10° before mechanical stops
- Prevents servo damage from over-extension

### Print-in-Place vs Assembly

**Print-in-Place** (single print, moving parts):
- Good for: Hinges, simple linkages
- Requires: 0.3-0.5mm clearance
- May need: Post-print cleanup

**Assembly** (multiple parts, bolted):
- Preferred for: Most structural components
- Allows: Material mixing, easier reprints
- Requires: Threaded inserts, alignment features

---

## Maintenance Considerations

### Serviceable Design

**Easy Access**:
- Snap-fit panels on torso
- Modular limbs (disconnect at single point)
- Clear labeling of all components

**Common Replacements**:
- Servos: Quick-swap mounts
- Bearings: Standard sizes, press-fit
- 3D printed parts: Keep STL library organized

**Wear Points**:
- Bearing surfaces: Inspect every 100 hours operation
- Gear teeth (if used): Replace if worn
- TPU pads: Replace yearly or as needed

### Upgrade Paths

Design allows for future improvements:
- Stronger servos (same mounting pattern)
- Additional sensors (mounting bosses built-in)
- Better batteries (compartment sized for growth)
- Carbon fiber reinforcement (pre-planned layup areas)

---

## Design Software & CAD Standards

### Recommended CAD Software

**Fusion 360** (Recommended for beginners):
- Free for hobbyists
- Parametric design (easy modifications)
- Integrated CAM for machining
- Good simulation tools
- Cloud-based collaboration

**FreeCAD** (Open-source alternative):
- Completely free
- Parametric design
- Steep learning curve
- Active community

**SolidWorks** (Professional):
- Industry standard
- Expensive licensing
- Best simulation tools

### CAD Organization

**File Structure**:
```
/cad
  /assemblies
    - full_robot.f3d
    - lower_body.f3d
    - upper_body.f3d
    - single_leg.f3d
    - single_arm.f3d
    - head.f3d
  /parts
    /leg
      - hip_bracket.f3d
      - thigh_tube.f3d
      - knee_bracket.f3d
      ...
    /arm
      ...
    /torso
      ...
    /head
      ...
  /exports
    /stl (for 3D printing)
    /step (for machining)
    /drawings (PDF)
```

**Naming Conventions**:
- Format: `[section]_[part]_[version].f3d`
- Example: `leg_hip_bracket_v2.f3d`
- Version control: Increment v# for each major revision

### Simulation & Analysis

**Static Analysis**:
- Test critical parts under load
- Safety factor: 2-3× expected load
- Focus on: Hip brackets, knee joints, ankle mounts

**Motion Simulation**:
- Validate range of motion
- Check for collisions
- Verify clearances

---

## Next Steps

1. **Begin CAD Design**:
   - Start with single leg (bottom-up approach)
   - Model servos first (accurate dimensions critical)
   - Build brackets around servos

2. **Create Detailed Drawings**:
   - Dimension all critical features
   - Note tolerances
   - Assembly instructions

3. **Prototype in CAD**:
   - Full assembly model
   - Identify interferences
   - Plan wire routing

4. **Prepare for Manufacturing**:
   - Export STL files
   - Generate parts list from CAD
   - Plan print order and orientation

---

**Document Status**: Initial Draft
**Next Review**: After CAD modeling begins
**Maintained By**: Project owner
