# Resources and References

## Overview

This document provides links to useful resources, reference projects, learning materials, and communities for building humanoid robots.

**Last Updated**: 2025-11-26

---

## Reference Humanoid Robot Projects

### Open-Source Projects

#### InMoov
- **Website**: http://inmoov.fr
- **Description**: Most popular open-source 3D printed humanoid robot
- **Height**: ~180cm (life-size)
- **Actuators**: Hobby servos (HK15298, MG996R)
- **Control**: Arduino Mega + Raspberry Pi
- **License**: Creative Commons
- **Pros**: Extensive documentation, large community, proven design
- **Cons**: Uses weaker servos, less dynamic than our design
- **Learn From**: Mechanical design, 3D printing techniques, hand/finger mechanisms

#### Poppy Project
- **Website**: https://www.poppy-project.org
- **Description**: Modular, open-source humanoid platform
- **Variants**: Poppy Humanoid, Poppy Torso, Poppy Ergo Jr
- **Actuators**: Dynamixel servos (same as our project!)
- **Control**: Raspberry Pi, Python
- **License**: GPL v3
- **Pros**: Professional design, great software (pypot library), modular
- **Cons**: Expensive to replicate fully
- **Learn From**: Software architecture, modular design, inverse kinematics

#### THOR (Tactical Hazardous Operations Robot)
- **Paper**: "THOR: An Open Source Humanoid for Full-Body Control"
- **Institution**: Virginia Tech
- **Description**: Research platform for humanoid robotics
- **Height**: ~145cm (close to our target!)
- **Actuators**: Various (Dynamixel, custom)
- **Pros**: Well-documented, research-grade design
- **Learn From**: Full-body control strategies, balance algorithms

#### Robotis OP3
- **Website**: https://emanual.robotis.com/docs/en/platform/op3/introduction/
- **Description**: Commercial research humanoid (but open-source)
- **Height**: 50.8cm (smaller scale)
- **Actuators**: Dynamixel XM and XL series
- **Control**: Intel NUC, ROS
- **Pros**: Complete documentation, proven software stack
- **Learn From**: ROS integration, walking algorithms, vision processing

### Commercial Reference Robots

#### Boston Dynamics Atlas
- **Videos**: YouTube "Boston Dynamics Atlas"
- **Description**: State-of-the-art humanoid robot
- **Learn From**: Advanced dynamics, parkour algorithms, balance recovery
- **Note**: Not open-source, but excellent inspiration for capabilities

#### Honda ASIMO
- **Description**: Historic humanoid robot (discontinued)
- **Learn From**: ZMP-based walking, early humanoid design principles

#### Tesla Optimus
- **Description**: Recent humanoid robot for general tasks
- **Learn From**: Modern design approaches, practical task focus

---

## Learning Resources

### Robotics Fundamentals

#### Books

**"Introduction to Robotics: Mechanics and Control" by John J. Craig**
- Topics: Kinematics, dynamics, trajectory planning
- Level: Intermediate
- Why: Classic textbook, excellent for understanding fundamentals

**"Modern Robotics" by Kevin Lynch and Frank Park**
- Free PDF: http://hades.mech.northwestern.edu/index.php/Modern_Robotics
- Topics: Kinematics, dynamics, control
- Why: Modern approach, free online resource, includes software

**"Springer Handbook of Robotics"**
- Topics: Comprehensive robotics reference
- Level: Advanced
- Why: In-depth coverage of all topics

**"Probabilistic Robotics" by Sebastian Thrun, Wolfram Burgard, Dieter Fox**
- Topics: Localization, mapping, sensor fusion
- Why: Essential for autonomous navigation

#### Online Courses

**"Underactuated Robotics" by Russ Tedrake (MIT)**
- URL: http://underactuated.mit.edu
- Topics: Dynamic walking, balance, control
- Format: Free online textbook + video lectures
- Why: Directly applicable to humanoid walking

**"Robotics: Aerial Robotics" (Coursera)**
- URL: https://www.coursera.org/learn/robotics-flight
- Topics: Dynamics, control (transferable to humanoids)
- Why: Good introduction to control systems

**"Robot Operating System (ROS)" (Various platforms)**
- Construct: https://www.theconstructsim.com
- Udemy: Multiple ROS courses
- Why: Essential for software architecture

### Kinematics and Dynamics

#### Video Lectures

**"Robot Kinematics" by Angela Sodemann**
- YouTube playlist
- Topics: DH parameters, forward/inverse kinematics
- Why: Clear explanations with examples

**"Modern Robotics Course" by Kevin Lynch**
- YouTube playlist (Northwestern University)
- Companion to textbook
- Why: Complete course with software examples

#### Software Tools

**Peter Corke's Robotics Toolbox (MATLAB/Python)**
- URL: https://petercorke.com/toolboxes/robotics-toolbox/
- What: MATLAB/Python library for robotics
- Why: Quickly prototype kinematics and dynamics

**Robot Visualizations (RViz, PyBullet)**
- RViz: ROS visualization tool
- PyBullet: Physics simulation
- Why: Visualize and test before building

### Control Systems

#### Online Resources

**"Control Tutorials for MATLAB and Simulink"**
- URL: http://ctms.engin.umich.edu/CTMS/
- Topics: PID, state-space, optimal control
- Why: Interactive tutorials with examples

**Brian Douglas's Control Systems YouTube Channel**
- Search: "Brian Douglas Control"
- Topics: PID, root locus, frequency response
- Why: Excellent visual explanations

### Computer Vision

**"Programming Computer Vision with Python" by Jan Erik Solem**
- Free PDF available
- Topics: Image processing, feature detection, stereo vision
- Why: Practical Python examples

**OpenCV Documentation and Tutorials**
- URL: https://docs.opencv.org/
- What: Official OpenCV documentation
- Why: Comprehensive reference for vision algorithms

---

## Hardware Resources

### Servo Motor Information

#### Dynamixel

**Official Resources**:
- E-Manual: https://emanual.robotis.com/
- Dynamixel Wizard: Configuration software
- Dynamixel SDK: Programming libraries (C++, Python, Java, etc.)

**Community Resources**:
- Robotis Forum: https://forum.robotis.com/
- Reddit: r/dynamixel

**Alternatives to Consider**:
- Feetech SCS series (budget-friendly, Dynamixel-compatible protocol)
- Herkulex DRS series
- Custom brushless motor + encoder + driver

### Electronics

#### Jetson Nano

**Official Resources**:
- Developer Kit Guide: https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit
- JetPack SDK: https://developer.nvidia.com/embedded/jetpack
- Forums: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/

**Alternative SBCs**:
- Raspberry Pi 5 (cheaper, less powerful)
- Jetson Xavier NX (more powerful, more expensive)
- Khadas VIM4 (good middle ground)

#### Arduino

**Resources**:
- Official tutorials: https://www.arduino.cc/en/Tutorial/HomePage
- Dynamixel2Arduino library: For controlling Dynamixel servos

**Alternatives**:
- Teensy 4.1 (faster, more memory)
- ESP32 (WiFi/Bluetooth built-in)
- STM32 (powerful, steeper learning curve)

### Sensors

**IMU (BNO055)**:
- Adafruit tutorial: https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor

**Force Sensors (FSR)**:
- Interlink guide: https://www.interlinkelectronics.com/fsr-guide

**Cameras**:
- Raspberry Pi Camera Guide: https://www.raspberrypi.com/documentation/accessories/camera.html

---

## Software Resources

### Robot Operating System (ROS)

**Official Resources**:
- ROS 2 Documentation: https://docs.ros.org/en/foxy/
- Tutorials: https://docs.ros.org/en/foxy/Tutorials.html

**Why ROS 2 (not ROS 1)**:
- Better real-time performance
- Improved security
- Active development
- Modern C++ and Python APIs

**Essential ROS 2 Packages**:
- `robot_state_publisher`: Publish robot state from URDF
- `joint_state_publisher`: Control joint positions
- `tf2`: Transform library (coordinate frames)
- `moveit2`: Motion planning (for arms)
- `navigation2`: Path planning and SLAM

### Simulation

#### PyBullet

**Resources**:
- GitHub: https://github.com/bulletphysics/bullet3
- Quickstart: https://pybullet.org/wordpress/
- Why: Fast, Python-based, easy to use, free

**Example Use**:
```python
import pybullet as p
p.connect(p.GUI)
robot = p.loadURDF("humanoid.urdf")
# Test control algorithms before hardware
```

#### Gazebo

**Resources**:
- Official site: https://gazebosim.org/
- ROS 2 Integration: Built-in

**Why**: Industry-standard, realistic physics, ROS integration

**Note**: Heavier than PyBullet, but more features

### CAD to URDF

**Fusion 360 to URDF Exporter**:
- Plugin: "fusion2urdf"
- GitHub: https://github.com/syuntoku14/fusion2urdf
- Why: Automatically generate URDF from CAD

**SolidWorks to URDF**:
- Official ROS plugin
- Tutorial: http://wiki.ros.org/sw_urdf_exporter

### Programming Languages

**Python**:
- Learn: https://www.learnpython.org/
- Why: High-level control, rapid prototyping, ROS support

**C++**:
- Learn: https://www.learncpp.com/
- Why: Real-time control, ROS 2, performance

**Arduino (C/C++)**:
- Learn: https://www.arduino.cc/en/Tutorial/HomePage
- Why: Embedded control, hardware interfacing

---

## Communities and Forums

### General Robotics

**Reddit**:
- r/robotics - General robotics discussion
- r/ros - ROS-specific
- r/arduino - Arduino help

**Forums**:
- Robotics Stack Exchange: https://robotics.stackexchange.com/
- ROS Discourse: https://discourse.ros.org/
- Let's Make Robots: https://www.robotshop.com/community/

### Humanoid-Specific

**Robotis Forum**:
- https://forum.robotis.com/
- Great for Dynamixel and OP3-related questions

**InMoov Community**:
- Forum: http://inmoov.fr/forum/
- Discord: Active community

**Poppy Project Forum**:
- https://forum.poppy-project.org/
- Helpful for Dynamixel-based humanoids

### DIY and Maker Communities

**Hackaday**:
- https://hackaday.io/
- Great for project documentation and sharing

**Instructables**:
- https://www.instructables.com/circuits/robots/
- Step-by-step project guides

---

## Supplier Resources

### Electronics and Components

**Global**:
- Digi-Key: https://www.digikey.com/ - Huge selection, fast shipping
- Mouser: https://www.mouser.com/ - Similar to Digi-Key
- Adafruit: https://www.adafruit.com/ - Maker-friendly, great tutorials
- SparkFun: https://www.sparkfun.com/ - Similar to Adafruit
- Pololu: https://www.pololu.com/ - Robotics-focused

**Budget Options**:
- AliExpress: Cheap components, long shipping
- Banggood: Similar to AliExpress
- **Warning**: Quality varies, not for critical components

### Robotics-Specific

**Servo Motors**:
- RobotShop: https://www.robotshop.com/
- Trossen Robotics: https://www.trossenrobotics.com/
- Robotis (Dynamixel): https://www.robotis.us/

**Mechanical**:
- McMaster-Carr: https://www.mcmaster.com/ - Everything mechanical (US)
- Misumi: https://us.misumi-ec.com/ - Precision components
- OnlineMetals: https://www.onlinemetals.com/ - Aluminum stock

**3D Printing**:
- Prusa Research: https://www.prusa3d.com/ - Filament and printers
- MatterHackers: https://www.matterhackers.com/ - Wide filament selection
- 3D Hubs: https://www.hubs.com/ - 3D printing service (if no printer)

---

## Academic Papers

### Humanoid Walking

**"Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point"**
- Authors: Kajita et al.
- Year: 2003
- Why: Foundational ZMP-based walking

**"The 3D Linear Inverted Pendulum Mode: A simple modeling for a biped walking pattern generation"**
- Authors: Kajita et al.
- Year: 2001
- Why: Simplified dynamics for walking

### Balance and Control

**"Balance Control Framework for Biped Robots"**
- Search on Google Scholar
- Why: Modern balance control techniques

**"Capture Point: A Step toward Humanoid Push Recovery"**
- Authors: Pratt et al.
- Why: Important concept for balance recovery

### General Humanoid Design

**"Design and Control of Humanoid Robots"**
- Search IEEE Xplore
- Why: Overview of design principles

---

## YouTube Channels

### Educational

**"MIT OpenCourseWare - Underactuated Robotics"**
- Russ Tedrake's lectures
- Essential for dynamic walking

**"MATLAB"**
- Official MATLAB channel
- Control systems tutorials

**"sentdex"**
- Python robotics and AI
- OpenCV and computer vision

### Inspiration

**"Boston Dynamics"**
- State-of-the-art humanoid robots
- See what's possible

**"Robotics Today"**
- Latest robotics news and demos

**"James Bruton"**
- DIY robotics projects
- 3D printing and mechatronics

---

## Software Tools Summary

### CAD Design
- **Fusion 360** (recommended): Free for hobbyists
- **FreeCAD**: Open-source alternative
- **SolidWorks**: Professional (expensive)

### Simulation
- **PyBullet**: Fast Python simulation
- **Gazebo**: ROS-integrated, realistic
- **Webots**: Commercial, user-friendly

### Programming IDEs
- **Visual Studio Code**: Best for Python, ROS, Arduino
- **Arduino IDE 2.0**: For Arduino development
- **PyCharm**: Professional Python IDE

### Version Control
- **Git**: Essential
- **GitHub**: Cloud hosting (what we'll use next!)
- **GitKraken**: GUI for Git (optional)

### Documentation
- **Markdown**: For docs (what we're using)
- **Doxygen**: Code documentation (C++)
- **Sphinx**: Python documentation

---

## Recommended Development Workflow

### Phase 1: Learning (Months 1-3)
1. Study "Modern Robotics" textbook (free PDF)
2. Complete CAD tutorials (Fusion 360)
3. Watch "Underactuated Robotics" lectures
4. Build simple Arduino projects (practice)
5. Study InMoov and Poppy designs

### Phase 2: Prototyping (Months 4-12)
1. Design in CAD
2. Simulate in PyBullet
3. 3D print and test mechanical assemblies
4. Develop basic Arduino firmware
5. Join communities, ask questions

### Phase 3: Building (Year 2)
1. Implement full hardware
2. Develop advanced software (IK, balance)
3. Integrate ROS 2
4. Document progress (photos, videos, blog)
5. Share with community, get feedback

### Phase 4: Refinement (Year 3)
1. Walking and advanced behaviors
2. Vision and autonomy
3. Publish project online (GitHub, Hackaday)
4. Create videos and tutorials
5. Contribute back to community

---

## Data Sheets and Manuals

### Servo Motors

**Dynamixel MX-64AT**:
- E-Manual: https://emanual.robotis.com/docs/en/dxl/mx/mx-64/

**Dynamixel MX-28AT**:
- E-Manual: https://emanual.robotis.com/docs/en/dxl/mx/mx-28/

**Dynamixel AX-12A**:
- E-Manual: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/

### Controllers

**NVIDIA Jetson Nano**:
- Module Datasheet: https://developer.nvidia.com/embedded/dlc/jetson-nano-system-module-datasheet
- Developer Kit User Guide: https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit

**Arduino Mega 2560**:
- Datasheet: https://docs.arduino.cc/hardware/mega-2560

### Sensors

**BNO055 IMU**:
- Datasheet: https://www.bosch-sensortec.com/products/smart-sensors/bno055/
- Adafruit Guide: https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor

**Raspberry Pi Camera Module V3**:
- Product Brief: https://www.raspberrypi.com/products/camera-module-v3/

---

## Standards and Protocols

### Communication Protocols

**Dynamixel Protocol 2.0**:
- Documentation: https://emanual.robotis.com/docs/en/dxl/protocol2/

**I2C (Inter-Integrated Circuit)**:
- Specification: NXP I2C-bus specification
- Tutorial: https://learn.sparkfun.com/tutorials/i2c

**UART/Serial**:
- Arduino reference: https://www.arduino.cc/reference/en/language/functions/communication/serial/

### File Formats

**URDF (Unified Robot Description Format)**:
- ROS documentation: http://wiki.ros.org/urdf
- Tutorial: http://wiki.ros.org/urdf/Tutorials

**STL (for 3D printing)**:
- Standard file format for 3D models

**STEP (for CAD exchange)**:
- Industry-standard CAD format

---

## Safety Resources

### LiPo Battery Safety

**Guide**: "A Comprehensive Guide to LiPo Battery Safety"
- Available on RC forums and battery manufacturer sites

**Key Points**:
- Never charge unattended
- Use fireproof LiPo bag
- Never discharge below 3.0V per cell
- Dispose of damaged batteries properly

### Electrical Safety

**Guide**: "Electronics Safety" by SparkFun
- URL: https://learn.sparkfun.com/tutorials/safety

### Robotics Safety

**ISO 13482**: Safety requirements for personal care robots
- Not legally required for hobby projects, but good reference
- Principles: Emergency stop, collision avoidance, safe speeds

---

## Staying Updated

### News and Trends

**Websites**:
- IEEE Spectrum Robotics: https://spectrum.ieee.org/robotics
- Robotics Business Review: https://www.roboticsbusinessreview.com/
- The Robot Report: https://www.therobotreport.com/

**Conferences (Videos Often Free)**:
- ICRA (International Conference on Robotics and Automation)
- IROS (International Conference on Intelligent Robots and Systems)
- RSS (Robotics: Science and Systems)

**YouTube Channels**:
- "Two Minute Papers" - AI and robotics research highlights

---

## Contribution and Sharing

### When Your Robot is Working

**Consider Sharing**:
1. **GitHub**: All CAD files, code, documentation
2. **Hackaday.io**: Project page with build log
3. **YouTube**: Build videos, walking demos
4. **Research Paper**: Write up your approach (optional)

**Benefits of Sharing**:
- Help others learn
- Get feedback and improvements
- Build reputation in community
- Potential collaborations
- Personal satisfaction

### How to Document

**Photos and Videos**:
- Document every stage
- Before/after modifications
- Testing and failures (people learn from these!)

**Write-Ups**:
- Blog posts (Medium, personal blog)
- Instructables step-by-step guide
- Academic paper (if applicable)

**Code**:
- Comment thoroughly
- README with setup instructions
- License (MIT or GPL recommended for hobby projects)

---

## Next Steps

After completing your humanoid robot project:

1. **Teach Others**: Create tutorials, give talks, mentor
2. **Improve Design**: Iterate based on lessons learned
3. **Advanced Features**:
   - Whole-body manipulation
   - Rough terrain walking
   - Human-robot interaction
4. **Related Projects**:
   - Quadruped robot
   - Robotic arm with more DOF
   - Wheeled mobile robot
5. **Professional Path** (if interested):
   - Robotics engineering
   - Research (Master's/PhD)
   - Industry jobs (many robotics companies hiring!)

---

## Conclusion

This is a challenging but rewarding project. Key to success:

- **Be Patient**: 3 years is realistic
- **Learn Continuously**: Use resources above
- **Engage Community**: Ask questions, share progress
- **Iterate**: Don't expect perfection first try
- **Document**: You'll thank yourself later
- **Have Fun**: This is an amazing journey!

**Good luck with your humanoid robot project!**

---

**Document Status**: Initial Draft
**Next Review**: Periodically update with new resources
**Maintained By**: Project owner

## Your Contribution

If you discover useful resources not listed here, please add them! This document should grow with the project.

