#!/usr/bin/env python3
"""
Knee Lower Bracket - Connects servo horn to shin tube
Part of 150cm Humanoid Robot Project
Connects to: Servo horn (top), Shin tube top (bottom)
"""

import cadquery as cq

# Parameters (all dimensions in mm)
BRACKET_WIDTH = 60
BRACKET_DEPTH = 50
BRACKET_THICKNESS = 6  # Thicker for strength (connects to servo horn)

# Servo horn mounting (Dynamixel MX-64AT horn pattern)
HORN_CENTER_HOLE_DIA = 7  # Center shaft hole
HORN_MOUNTING_HOLES = 8  # Number of holes in horn pattern
HORN_HOLE_DIA = 2.7  # M2.5 clearance for horn screws
HORN_BOLT_CIRCLE_DIA = 20  # Diameter of hole pattern

# Shin connection (at bottom)
SHIN_CONNECTION_HOLES = 4  # 4 mounting holes for shin tube end cap
SHIN_HOLE_SPACING = 40
SHIN_HOLE_DIA = 3.2  # M3 clearance

# Reinforcement ribs
RIB_THICKNESS = 3
RIB_HEIGHT = 15

# Wire routing
WIRE_HOLE_DIA = 15

# Create main bracket plate
result = (
    cq.Workplane("XY")
    .box(BRACKET_WIDTH, BRACKET_DEPTH, BRACKET_THICKNESS, centered=(True, True, False))
)

# Servo horn mounting holes (top surface)
# Center hole for servo shaft
result = (
    result
    .faces(">Z").workplane()
    .circle(HORN_CENTER_HOLE_DIA / 2)
    .cutThruAll()

    # Mounting holes in bolt circle pattern (8 holes, evenly spaced)
    .faces(">Z").workplane()
    .polarArray(HORN_BOLT_CIRCLE_DIA / 2, 0, 360, HORN_MOUNTING_HOLES)
    .circle(HORN_HOLE_DIA / 2)
    .cutThruAll()
)

# Add boss around center for servo horn registration
result = (
    result
    .faces(">Z").workplane()
    .circle(HORN_BOLT_CIRCLE_DIA / 2 + 3)
    .circle(HORN_CENTER_HOLE_DIA / 2)
    .extrude(2)
)

# Shin mounting holes (bottom surface, 4 corners)
result = (
    result
    .faces("<Z").workplane()
    .rect(SHIN_HOLE_SPACING, SHIN_HOLE_SPACING, forConstruction=True)
    .vertices()
    .circle(SHIN_HOLE_DIA / 2)
    .cutThruAll()

    # Counterbore for M3 heat-set inserts
    .faces("<Z").workplane()
    .rect(SHIN_HOLE_SPACING, SHIN_HOLE_SPACING, forConstruction=True)
    .vertices()
    .cboreHole(SHIN_HOLE_DIA, 5.5, 4.5)
)

# Add reinforcement ribs (cross pattern on bottom)
result = (
    result
    .faces("<Z").workplane()
    .rect(BRACKET_WIDTH - 10, RIB_THICKNESS, centered=True)
    .extrude(-RIB_HEIGHT)

    .faces("<Z").workplane()
    .rect(RIB_THICKNESS, BRACKET_DEPTH - 10, centered=True)
    .extrude(-RIB_HEIGHT)
)

# Wire routing hole (offset from center, connects to shin wire channel)
result = (
    result
    .faces("<Z").workplane()
    .center(BRACKET_WIDTH/4, 0)
    .circle(WIRE_HOLE_DIA / 2)
    .cutBlind(BRACKET_THICKNESS)
)

# Add corner reinforcements (gussets)
for x in [-1, 1]:
    for y in [-1, 1]:
        result = (
            result
            .faces("<Z").workplane()
            .center(x * (BRACKET_WIDTH/2 - 8), y * (BRACKET_DEPTH/2 - 8))
            .circle(6)
            .extrude(-10)
        )

# Chamfer all edges for safety and print quality
# Select only the outer edges for chamfering (more selective to avoid errors)
try:
    result = result.edges("|Z").chamfer(0.5)
except:
    print("Note: Some chamfers skipped due to geometry constraints")

print("Lower bracket created successfully")
print(f"Dimensions: {BRACKET_WIDTH}mm × {BRACKET_DEPTH}mm × {BRACKET_THICKNESS + RIB_HEIGHT}mm")
print(f"Mounting: 8× M2.5 for servo horn, 4× M3 for shin")
print(f"Features: Center boss, cross ribs, wire routing")

# Export to STEP file
output_path = "../exports/step/knee_lower_bracket.step"
cq.exporters.export(result, output_path)
print(f"✓ Exported to: {output_path}")

# Also export STL
output_stl = "../exports/stl/knee_lower_bracket.stl"
cq.exporters.export(result, output_stl)
print(f"✓ Exported STL to: {output_stl}")

if __name__ == "__cq_main__":
    show_object(result)
