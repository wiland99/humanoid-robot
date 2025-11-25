#!/usr/bin/env python3
"""
Knee Servo Holder - Side-mounted bracket that secures the Dynamixel MX-64AT servo
Part of 150cm Humanoid Robot Project
Mounts to: Upper and lower brackets, holds servo body
"""

import cadquery as cq

# Parameters (all dimensions in mm)
HOLDER_LENGTH = 70  # Length along servo body
HOLDER_HEIGHT = 45  # Height (holds servo)
HOLDER_THICKNESS = 5  # Wall thickness

# Servo body dimensions (Dynamixel MX-64AT)
SERVO_WIDTH = 40.2
SERVO_HEIGHT = 41
SERVO_BODY_LENGTH = 61

# Servo mounting holes
SERVO_MOUNT_HOLE_DIA = 3.2  # M3 clearance
SERVO_MOUNT_HOLE_DEPTH = 51  # Distance between holes

# Bracket mounting holes (connects to upper and lower brackets)
BRACKET_MOUNT_HOLES = 4
BRACKET_HOLE_DIA = 3.2

# Create main U-shaped holder body
result = (
    cq.Workplane("XZ")
    .rect(HOLDER_LENGTH, HOLDER_HEIGHT)
    .extrude(HOLDER_THICKNESS)

    # Cut out center to create U-shape (for servo body)
    .faces(">Y").workplane()
    .rect(SERVO_BODY_LENGTH, SERVO_HEIGHT - 8)
    .cutBlind(-HOLDER_THICKNESS + 1.5)
)

# Add mounting tabs at top and bottom for connection to brackets
result = (
    result
    .faces(">Z").workplane()
    .rect(HOLDER_LENGTH + 20, 15, centered=True)
    .extrude(HOLDER_THICKNESS)

    .faces("<Z").workplane()
    .rect(HOLDER_LENGTH + 20, 15, centered=True)
    .extrude(HOLDER_THICKNESS)
)

# Servo mounting holes (through the side walls)
result = (
    result
    .faces(">Y").workplane(centerOption="CenterOfBoundBox")
    .pushPoints([
        (SERVO_MOUNT_HOLE_DEPTH/2 - SERVO_BODY_LENGTH/2, 0),
        (-SERVO_MOUNT_HOLE_DEPTH/2 + SERVO_BODY_LENGTH/2, 0)
    ])
    .circle(SERVO_MOUNT_HOLE_DIA / 2)
    .cutThruAll()
)

# Bracket mounting holes (on the tabs)
result = (
    result
    .faces(">Z").workplane(offset=HOLDER_HEIGHT/2, centerOption="CenterOfBoundBox")
    .pushPoints([
        (25, 0),
        (-25, 0)
    ])
    .circle(BRACKET_HOLE_DIA / 2)
    .cutThruAll()

    .faces("<Z").workplane(offset=-HOLDER_HEIGHT/2, centerOption="CenterOfBoundBox")
    .pushPoints([
        (25, 0),
        (-25, 0)
    ])
    .circle(BRACKET_HOLE_DIA / 2)
    .cutThruAll()
)

# Wire passage slot (bottom)
result = (
    result
    .faces(">Y").workplane(centerOption="CenterOfBoundBox")
    .center(0, -HOLDER_HEIGHT/2 + 5)
    .rect(10, 8)
    .cutBlind(-HOLDER_THICKNESS)
)

# Add ventilation slots (helps with servo cooling)
for i in range(-1, 2):
    result = (
        result
        .faces(">Y").workplane(centerOption="CenterOfBoundBox")
        .center(i * 15, 0)
        .rect(3, 25)
        .cutBlind(-1.5)
    )

# Chamfer edges
result = result.edges("|Y").chamfer(0.5)

print("Servo holder created successfully")
print(f"Dimensions: {HOLDER_LENGTH}mm × {HOLDER_THICKNESS}mm × {HOLDER_HEIGHT}mm")
print(f"Fits: Dynamixel MX-64AT (side-mount)")

# Export to STEP file
output_path = "../exports/step/knee_servo_holder.step"
cq.exporters.export(result, output_path)
print(f"✓ Exported to: {output_path}")

# Also export STL
output_stl = "../exports/stl/knee_servo_holder.stl"
cq.exporters.export(result, output_stl)
print(f"✓ Exported STL to: {output_stl}")

if __name__ == "__cq_main__":
    show_object(result)
