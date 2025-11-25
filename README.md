# 150cm Humanoid Robot Project

## Project Overview

This is a comprehensive multi-year project to build a fully functional humanoid robot from scratch. The robot will stand 150cm tall and feature autonomous balance, walking capabilities, and arm manipulation.

**Project Timeline**: 3 years
**Budget Range**: $8,000 - $12,000 (mid-range)
**Target Height**: 150 cm
**Degrees of Freedom**: 25-30 DOF

## Project Goals

- **Primary**: Build a humanoid robot capable of standing, balancing, and walking
- **Secondary**: Implement arm manipulation and object interaction
- **Tertiary**: Develop advanced features like vision-based navigation and voice interaction

## Robot Specifications

### Physical Dimensions
- **Total Height**: 150 cm
- **Head**: ~20 cm
- **Torso**: ~45 cm
- **Arms**: ~55 cm each (shoulder to fingertips)
- **Legs**: ~75 cm each (hip to foot)
- **Estimated Weight**: 8-12 kg (depending on materials)

### Degrees of Freedom (DOF) Breakdown
- **Legs**: 12 DOF total
  - Hip: 3 DOF per leg (flexion/extension, abduction/adduction, rotation)
  - Knee: 1 DOF per leg (flexion/extension)
  - Ankle: 2 DOF per leg (dorsiflexion/plantarflexion, inversion/eversion)
- **Arms**: 10 DOF total
  - Shoulder: 3 DOF per arm
  - Elbow: 1 DOF per arm
  - Wrist: 2 DOF per arm
  - Gripper/Hand: 1 DOF per arm
- **Torso**: 2 DOF
  - Waist rotation: 1 DOF
  - Waist tilt: 1 DOF
- **Head/Neck**: 3 DOF
  - Pan: 1 DOF
  - Tilt: 1 DOF
  - Roll (optional): 1 DOF

**Total**: 27 DOF

## Project Structure

```
Human Robot/
├── README.md                           # This file
├── docs/
│   ├── 01-BOM.md                      # Bill of Materials
│   ├── 02-Mechanical-Design.md        # Mechanical specifications
│   ├── 03-Electrical-Design.md        # Electronics and wiring
│   ├── 04-Software-Architecture.md    # Software design
│   ├── 05-Assembly-Guide.md           # Build instructions
│   ├── 06-Testing-Calibration.md      # Testing procedures
│   └── 07-Resources.md                # References and links
├── cad/                                # CAD models (to be created)
├── firmware/                           # Motor controller code (to be created)
├── software/                           # High-level control software (to be created)
├── electronics/                        # Circuit diagrams and PCB designs (to be created)
└── research/                           # Research notes and papers (to be created)
```

## Three-Year Development Plan

### Year 1: Foundation & Lower Body
**Months 1-3: Research & Design**
- Study existing humanoid robots (InMoov, Poppy, THOR)
- Learn CAD software (Fusion 360 or FreeCAD)
- Create complete CAD model of robot
- Finalize component selections

**Months 4-8: Procurement & Leg Prototyping**
- Order initial components (servos, sensors, materials)
- Build and test single leg assembly
- Validate torque requirements and joint design
- Iterate on mechanical design

**Months 9-12: Lower Body Assembly**
- Build complete lower body (pelvis + both legs)
- Implement basic IMU-based balance control
- Test standing stability
- Refine power distribution system

### Year 2: Upper Body & Software Foundation
**Months 1-6: Upper Body Construction**
- Build torso structure
- Construct both arm assemblies
- Build head with sensor mounts
- Full mechanical integration

**Months 7-12: Core Software Development**
- Implement forward/inverse kinematics
- Develop basic balance controller
- Create initial gait generation
- Set up sensor fusion system
- Build control interface

### Year 3: Walking & Advanced Features
**Months 1-6: Integration & Walking**
- Full system integration
- Progressive testing (standing → shifting → stepping → walking)
- Gait refinement and optimization
- Arm coordination during walking

**Months 7-12: Advanced Capabilities**
- Vision-based navigation
- Object recognition and manipulation
- Advanced balance recovery
- Autonomous behavior development
- Documentation and demonstration

## Key Technologies

### Hardware
- **Actuators**: Digital servo motors (Dynamixel or equivalent)
- **Main Controller**: Raspberry Pi 5 or NVIDIA Jetson Nano
- **Motor Controllers**: Arduino Mega/Teensy 4.1 or dedicated servo controllers
- **Sensors**: IMU, force sensors, encoders, cameras
- **Power**: LiPo batteries (3S-4S, 5000-10000mAh)
- **Structure**: Aluminum extrusions + 3D printed parts (ABS/PETG)

### Software
- **Programming Languages**: Python (high-level), C++ (low-level control)
- **Framework**: ROS 2 (Robot Operating System) - recommended
- **Simulation**: Gazebo or PyBullet
- **CAD**: Fusion 360 or FreeCAD
- **Version Control**: Git

## Getting Started

1. **Read all documentation** in the `docs/` folder in order
2. **Start with research**: Study recommended open-source projects
3. **Learn CAD**: Complete basic tutorials in Fusion 360 or FreeCAD
4. **Set up development environment**: Install Python, ROS, and simulation tools
5. **Order initial components**: Start with a small test set of servos and electronics
6. **Build test prototypes**: Create simple mechanisms before committing to full design

## Safety Considerations

- **Electrical**: Use proper fuses, voltage regulation, and battery management
- **Mechanical**: Ensure all moving parts have proper clearance and safety margins
- **Testing**: Always test new features in safe, controlled environment
- **Emergency Stop**: Implement hardware e-stop button
- **Power Management**: Include voltage monitoring and automatic shutdown
- **Weight**: Be cautious of robot falling during testing (use support frame initially)

## Current Status

**Phase**: Planning and Documentation
**Completed**: Initial project plan and documentation structure
**Next Steps**: Begin research phase and component selection

## Contributing

This is a personal project, but ideas and suggestions are welcome.

## License

This project documentation is released under MIT License. Component selections and original designs are provided as-is for educational purposes.

## Contact & Notes

**Project Start Date**: 2025-11-26
**Last Updated**: 2025-11-26

---

*"The journey of a thousand miles begins with a single step"* - And this robot will take many steps!
