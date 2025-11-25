# Knee Joint Assembly

## Overview

This directory contains the CAD models for the knee joint assembly of the 150cm humanoid robot. The knee joint is a **1-DOF (degree of freedom)** hinge joint that connects the thigh to the shin.

**Generated**: 2025-11-26
**Software**: CadQuery (Python-based parametric CAD)
**Export Formats**: STEP (for CAD import) and STL (for 3D printing)

---

## Parts List

### 1. Upper Bracket (`01_knee_upper_bracket.py`)
**File**: `knee_upper_bracket.step` / `knee_upper_bracket.stl`

**Function**: Connects the thigh tube to the knee servo body

**Features**:
- Mounting flange for thigh tube (4× M3 mounting holes)
- Side walls for servo mounting (4× M3 holes)
- Wire routing channel on bottom
- Reinforcement ribs for strength
- Dimensions: 60mm × 50mm × 30mm (approx)

**Material**: PETG (3D printed)
**Infill**: 40%
**Print Time**: ~8 hours

### 2. Servo Holder (`02_knee_servo_holder.py`)
**File**: `knee_servo_holder.step` / `knee_servo_holder.stl`

**Function**: Side-mounted bracket that secures the Dynamixel MX-64AT servo

**Features**:
- U-shaped holder for servo body
- Ventilation slots for cooling
- Mounting tabs (top and bottom)
- Wire passage slot
- Dimensions: 70mm × 5mm × 45mm

**Material**: PETG (3D printed)
**Infill**: 40%
**Print Time**: ~4 hours

### 3. Lower Bracket (`03_knee_lower_bracket.py`)
**File**: `knee_lower_bracket.step` / `knee_lower_bracket.stl`

**Function**: Connects servo horn to the shin tube

**Features**:
- Servo horn mounting pattern (8× M2.5 holes, Dynamixel standard)
- Center boss for horn registration
- Shin mounting (4× M3 holes)
- Cross-pattern reinforcement ribs
- Wire routing hole
- Corner gussets for strength
- Dimensions: 60mm × 50mm × 21mm

**Material**: PETG (3D printed)
**Infill**: 40%
**Print Time**: ~5 hours

---

## Assembly Instructions

### Hardware Required:
- 1× Dynamixel MX-64AT servo motor
- 8× M2.5 × 6mm bolts (servo horn mounting)
- 4× M3 × 10mm bolts (servo to upper bracket)
- 4× M3 × 10mm bolts (thigh to upper bracket)
- 4× M3 × 10mm bolts (shin to lower bracket)
- 12× M3 heat-set inserts (brass, 5mm long)
- Loctite 243 (blue threadlocker)

### Assembly Steps:

**1. Prepare 3D Printed Parts**
- Remove supports
- Clean up surfaces with hobby knife
- Install heat-set inserts:
  - Upper bracket: 4 inserts (top surface)
  - Lower bracket: 4 inserts (bottom surface)
  - Use soldering iron at 220°C

**2. Attach Servo to Upper Bracket**
- Position MX-64AT servo between side walls
- Align mounting holes
- Install 4× M3 bolts through side walls into servo
- Apply threadlocker, tighten firmly

**3. Attach Servo Horn to Lower Bracket**
- Center servo horn on lower bracket center boss
- Align 8 mounting holes
- Install 8× M2.5 bolts
- Apply threadlocker, tighten in star pattern

**4. Connect Horn to Servo**
- Ensure servo is at center position (2048)
- Attach lower bracket assembly to servo output shaft
- Align with desired zero position (straight leg)
- Tighten servo horn center screw

**5. Wire Management**
- Route servo power wires through bottom channel
- Route servo data cable through wire passage
- Secure with zip ties
- Leave service loop for joint movement

**6. Connect to Thigh and Shin**
- Attach thigh tube end cap to upper bracket (4× M3 bolts)
- Attach shin tube end cap to lower bracket (4× M3 bolts)

### Range of Motion:
- **Full extension**: 0° (straight leg)
- **Full flexion**: 140° (knee bent)
- **Mechanical stops**: ±5° from servo limits
- **Software limits**: 0° to 135° (safe range)

---

## Using the STEP Files

### Import to Fusion 360:
1. Open Fusion 360
2. File → Upload → Select .step file
3. File will import as a solid body
4. Can combine with other parts in assembly
5. Can modify (but not parametrically)

### Import to SolidWorks:
1. File → Open → Select .step file
2. Import as Part
3. Can use in assemblies
4. Can edit features

### Import to FreeCAD:
1. File → Open → Select .step file
2. Imports as Part object
3. Fully editable

---

## Using the STL Files

### 3D Printing:

**Recommended Settings** (PETG):
- Layer height: 0.2mm
- Infill: 40% (gyroid pattern)
- Wall thickness: 4 perimeters (1.6mm)
- Top/bottom layers: 5
- Print speed: 40-50 mm/s
- Nozzle temp: 230-240°C
- Bed temp: 70-80°C

**Print Orientation**:
- **Upper bracket**: Print with mounting flange (base) down
- **Servo holder**: Print standing on edge (side down)
- **Lower bracket**: Print with horn boss facing up

**Supports**:
- Minimal supports needed with correct orientation
- Use tree supports if needed
- Support overhang angle: 50°

**Post-Processing**:
- Remove supports carefully
- Clean mounting holes with 3mm drill bit
- Install heat-set inserts
- Test fit with servo before final assembly

---

## Modifying the Designs

### Edit Python Scripts:

All parts are **parametric** - you can modify dimensions by editing the Python scripts:

```python
# Example: Make bracket wider
BRACKET_WIDTH = 70  # Changed from 60mm
```

**Key parameters you might want to change**:
- `BRACKET_WIDTH`, `BRACKET_DEPTH` - Overall dimensions
- `BRACKET_THICKNESS` - Wall thickness (affects strength)
- `HOLE_SPACING` - Mounting hole positions
- `WIRE_CHANNEL_WIDTH` - Cable routing size

**After editing**:
1. Save the .py file
2. Run: `python3 01_knee_upper_bracket.py`
3. New STEP/STL files will be generated
4. Reprint and test

### Version Control:
- Original files are in Git
- Make changes on a branch
- Test before committing
- Tag working versions

---

## Testing Checklist

Before assembly:
- [ ] All parts printed cleanly
- [ ] No warping or layer separation
- [ ] Mounting holes clear (test with bolts)
- [ ] Heat-set inserts installed flush
- [ ] Servo fits in brackets (dry fit)

After assembly:
- [ ] Servo powers on
- [ ] Full range of motion (0-140°)
- [ ] No binding or rubbing
- [ ] All bolts tight (with threadlocker)
- [ ] Wires properly routed
- [ ] No excessive play in joints

Load testing:
- [ ] Can support 3kg weight (torso + upper body)
- [ ] Servo current <1.5A under load
- [ ] No creaking or flexing
- [ ] Temperature stays <50°C

---

## Specifications

| Parameter | Value |
|-----------|-------|
| Degrees of Freedom | 1 (pitch/flexion) |
| Servo Model | Dynamixel MX-64AT |
| Servo Torque | 6.0 Nm @ 12V |
| Range of Motion | 0° to 140° |
| Total Weight | ~250g (all parts) |
| Material | PETG (3D printed) |
| Fasteners | M3 bolts, M2.5 bolts |
| Assembly Time | ~2 hours |

---

## Troubleshooting

**Issue**: Parts don't fit together
- **Solution**: Check print dimensions (may need scaling adjustment)
- **Solution**: Clearance holes may need reaming

**Issue**: Servo doesn't fit
- **Solution**: MX-64AT dimensions vary slightly, sand interior if needed
- **Solution**: Check servo model (ensure it's MX-64AT not MX-64R)

**Issue**: Bracket flexes under load
- **Solution**: Increase infill to 50-60%
- **Solution**: Add more perimeters (5-6 walls)
- **Solution**: Consider ABS or nylon instead of PETG

**Issue**: Joint has play/slop
- **Solution**: Tighten all bolts with threadlocker
- **Solution**: Check servo horn screw (center)
- **Solution**: Add brass bushings if needed

---

## Next Steps

After completing the knee joint:

1. **Build second knee** (mirror for left leg)
2. **Create thigh tube** with end caps
3. **Create shin tube** with end caps
4. **Test single leg assembly**
5. **Move on to ankle joint** (2-DOF, more complex)

---

## Files in This Directory

```
knee_joint/
├── README.md (this file)
├── 01_knee_upper_bracket.py       # Upper bracket generator
├── 02_knee_servo_holder.py        # Servo holder generator
├── 03_knee_lower_bracket.py       # Lower bracket generator
└── (generated files in ../exports/)
```

**Generated STEP files**: `../exports/step/knee_*.step`
**Generated STL files**: `../exports/stl/knee_*.stl`

---

## License

These CAD files are part of the open-source humanoid robot project.
**License**: MIT
**Attribution**: Please credit if you use or modify these designs

---

## Support

Questions or issues?
- Check the main project documentation
- Open an issue on GitHub
- Review the mechanical design spec (docs/02-Mechanical-Design.md)

**Last Updated**: 2025-11-26
