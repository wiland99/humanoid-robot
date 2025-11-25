#!/usr/bin/env python3
"""
Knee Upper Bracket - Connects thigh tube to knee servo
Part of 150cm Humanoid Robot Project
Connects to: Thigh bottom, Knee servo body
"""

import cadquery as cq

# Parameters (all dimensions in mm)
BRACKET_WIDTH = 60
BRACKET_DEPTH = 50
BRACKET_THICKNESS = 5

# Servo mounting (Dynamixel MX-64AT dimensions)
SERVO_WIDTH = 40.2
SERVO_DEPTH = 61
SERVO_HEIGHT = 41
SERVO_MOUNT_HOLE_SPACING_WIDTH = 31  # Distance between side mounting holes
SERVO_MOUNT_HOLE_SPACING_DEPTH = 51  # Distance between front/back holes
SERVO_MOUNT_HOLE_DIA = 3.2  # M3 clearance

# Thigh connection (at top)
THIGH_CONNECTION_HOLES = 4  # 4 mounting holes for thigh tube end cap
THIGH_HOLE_SPACING = 40
THIGH_HOLE_DIA = 3.2  # M3 clearance

# Wire routing channel
WIRE_CHANNEL_WIDTH = 12
WIRE_CHANNEL_DEPTH = 10

# Create the main bracket body
result = (
    cq.Workplane("XY")
    .box(BRACKET_WIDTH, BRACKET_DEPTH, BRACKET_THICKNESS, centered=(True, True, False))

    # Add mounting flanges for servo (side walls)
    .faces(">Z").workplane()
    .transformed(offset=(BRACKET_WIDTH/2 - 2.5, 0, 0))
    .box(5, SERVO_DEPTH, 25, centered=(False, True, False))
    .faces(">Z").workplane()
    .transformed(offset=(-BRACKET_WIDTH/2 - 2.5, 0, 0))
    .box(5, SERVO_DEPTH, 25, centered=(False, True, False))

    # Create thigh mounting holes (top surface, 4 corners)
    .faces(">Z").workplane(offset=-BRACKET_THICKNESS)
    .rect(THIGH_HOLE_SPACING, THIGH_HOLE_SPACING, forConstruction=True)
    .vertices()
    .circle(THIGH_HOLE_DIA / 2)
    .cutThruAll()

    # Add threaded insert pilot holes (actual holes for M3 heat-set inserts)
    .faces(">Z").workplane(offset=-BRACKET_THICKNESS)
    .rect(THIGH_HOLE_SPACING, THIGH_HOLE_SPACING, forConstruction=True)
    .vertices()
    .cboreHole(THIGH_HOLE_DIA, 5, 4)  # M3 hole with counterbore for insert
)

# Servo mounting holes (on the side flanges)
result = (
    result
    .faces(">X").workplane(centerOption="CenterOfBoundBox")
    .pushPoints([
        (SERVO_MOUNT_HOLE_SPACING_DEPTH/2 - SERVO_DEPTH/2, 10),
        (-SERVO_MOUNT_HOLE_SPACING_DEPTH/2 + SERVO_DEPTH/2, 10)
    ])
    .circle(SERVO_MOUNT_HOLE_DIA / 2)
    .cutThruAll()

    .faces("<X").workplane(centerOption="CenterOfBoundBox")
    .pushPoints([
        (SERVO_MOUNT_HOLE_SPACING_DEPTH/2 - SERVO_DEPTH/2, 10),
        (-SERVO_MOUNT_HOLE_SPACING_DEPTH/2 + SERVO_DEPTH/2, 10)
    ])
    .circle(SERVO_MOUNT_HOLE_DIA / 2)
    .cutThruAll()
)

# Wire routing channel (bottom)
result = (
    result
    .faces("<Z").workplane()
    .rect(WIRE_CHANNEL_WIDTH, BRACKET_DEPTH + 10, centered=True)
    .cutBlind(3)
)

# Add mounting bosses for extra strength (ribs)
result = (
    result
    .faces(">Y").workplane(offset=-BRACKET_DEPTH/2, centerOption="CenterOfBoundBox")
    .center(0, 15)
    .lineTo(15, 0)
    .lineTo(15, -10)
    .lineTo(0, -10)
    .close()
    .extrude(5)

    .faces("<Y").workplane(offset=BRACKET_DEPTH/2, centerOption="CenterOfBoundBox")
    .center(0, 15)
    .lineTo(15, 0)
    .lineTo(15, -10)
    .lineTo(0, -10)
    .close()
    .extrude(5)
)

# Chamfer all edges for printability and safety
result = result.edges("|Z").chamfer(0.5)

print("Upper bracket created successfully")
print(f"Dimensions: {BRACKET_WIDTH}mm × {BRACKET_DEPTH}mm × ~30mm (with flanges)")
print(f"Mounting: 4× M3 holes for thigh, 4× M3 holes for servo")

# Export to STEP file
output_path = "../exports/step/knee_upper_bracket.step"
cq.exporters.export(result, output_path)
print(f"✓ Exported to: {output_path}")

# Also export STL for 3D printing
output_stl = "../exports/stl/knee_upper_bracket.stl"
cq.exporters.export(result, output_stl)
print(f"✓ Exported STL to: {output_stl}")

# For viewing (optional - shows the part if run interactively)
if __name__ == "__cq_main__":
    show_object(result)
