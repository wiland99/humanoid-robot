# Software Architecture

## Overview

This document outlines the complete software architecture for the 150cm humanoid robot, including control systems, kinematics, planning algorithms, and implementation details.

**Last Updated**: 2025-11-26

---

## Software Stack Overview

### Layer Architecture

```
┌─────────────────────────────────────────────┐
│  Application Layer (Jetson Nano)            │
│  - High-level behaviors                     │
│  - Vision processing                        │
│  - Path planning                            │
│  - User interface                           │
└────────────────┬────────────────────────────┘
                 │ ROS 2 / Custom Protocol
┌────────────────▼────────────────────────────┐
│  Control Layer (Jetson + Arduino)           │
│  - Inverse kinematics                       │
│  - Balance controller                       │
│  - Gait generation                          │
│  - Trajectory planning                      │
└────────────────┬────────────────────────────┘
                 │ Serial / Dynamixel Protocol
┌────────────────▼────────────────────────────┐
│  Hardware Abstraction Layer (Arduino)       │
│  - Servo control                            │
│  - Sensor reading                           │
│  - Safety monitoring                        │
└────────────────┬────────────────────────────┘
                 │ PWM / I2C / Analog
┌────────────────▼────────────────────────────┐
│  Hardware (Servos, Sensors, Actuators)      │
└─────────────────────────────────────────────┘
```

---

## Development Environment Setup

### Jetson Nano Setup

**Operating System**: Ubuntu 20.04 LTS (JetPack 4.6+)

**Installation Steps**:
1. Flash microSD card with JetPack image (NVIDIA SDK Manager)
2. Boot Jetson, complete initial setup
3. Update system:
   ```bash
   sudo apt update && sudo apt upgrade
   ```

**Core Software**:
```bash
# Python 3.8+
sudo apt install python3-pip python3-dev

# ROS 2 Foxy (recommended) or Humble
# Follow: https://docs.ros.org/en/foxy/Installation.html
sudo apt install ros-foxy-desktop

# Computer Vision
sudo apt install python3-opencv
pip3 install opencv-contrib-python

# Numerical computing
pip3 install numpy scipy matplotlib

# Dynamixel SDK
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/python
sudo python3 setup.py install

# PyBullet (simulation)
pip3 install pybullet
```

### Arduino Setup

**IDE**: Arduino IDE 2.0+ or PlatformIO (VS Code)

**Required Libraries**:
```cpp
// Install via Library Manager:
- Dynamixel2Arduino (for Dynamixel servos)
- Adafruit BNO055 (IMU)
- Wire (I2C, built-in)
- SPI (built-in)

// Install manually:
- Custom robot control library (to be developed)
```

### Development Tools

**Version Control**: Git
```bash
cd ~/git/wiland99/Human\ Robot
git init
git add .
git commit -m "Initial humanoid robot project structure"
```

**Simulation**: PyBullet or Gazebo
**CAD Integration**: URDF (Unified Robot Description Format)
**Debugging**: GDB (C++), pdb (Python), Serial Monitor

---

## Coordinate Systems & Conventions

### Global Coordinate Frame

```
       Z (up)
       │
       │
       └─── Y (forward)
      /
     /
    X (right)
```

**Conventions**:
- **X-axis**: Robot's right (positive) to left (negative)
- **Y-axis**: Robot's forward (positive) to backward (negative)
- **Z-axis**: Upward (positive) to downward (negative)
- **Origin**: Center of pelvis at ground level

### Joint Angle Conventions

**Sign Convention**:
- **Positive rotation**: Counter-clockwise when looking along the positive axis
- **Zero position**: Robot standing upright, arms at sides

**Joint Ranges** (in radians and degrees):

| Joint | Min (rad) | Max (rad) | Min (deg) | Max (deg) |
|-------|-----------|-----------|-----------|-----------|
| Hip Pitch | -0.52 | 2.09 | -30° | 120° |
| Hip Roll | -0.79 | 0.79 | -45° | 45° |
| Hip Yaw | -0.79 | 0.79 | -45° | 45° |
| Knee Pitch | 0 | 2.44 | 0° | 140° |
| Ankle Pitch | -0.52 | 0.79 | -30° | 45° |
| Ankle Roll | -0.35 | 0.35 | -20° | 20° |

---

## Kinematics

### Forward Kinematics (FK)

**Purpose**: Calculate end-effector position from joint angles

**Denavit-Hartenberg (DH) Parameters**:

For each joint, define:
- **θ (theta)**: Joint angle (variable for revolute joints)
- **d**: Link offset along previous Z
- **a**: Link length along X
- **α (alpha)**: Link twist (rotation about X)

**Example: Right Leg DH Table**

| Joint | θ | d | a | α |
|-------|---|---|---|---|
| Hip Yaw | θ1 | 0 | 0 | π/2 |
| Hip Roll | θ2 | 0 | 0 | π/2 |
| Hip Pitch | θ3 | 0 | 0 | -π/2 |
| Knee Pitch | θ4 | 0.35m | 0 | 0 |
| Ankle Pitch | θ5 | 0.32m | 0 | π/2 |
| Ankle Roll | θ6 | 0 | 0.08m | 0 |

**Transformation Matrix** (from joint i-1 to joint i):
```
T_i = [ cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) ]
      [ sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) ]
      [   0         sin(α)         cos(α)          d     ]
      [   0           0               0            1     ]
```

**End-Effector Position**:
```
T_total = T_1 × T_2 × T_3 × T_4 × T_5 × T_6
Position = [T_total[0,3], T_total[1,3], T_total[2,3]]
```

**Implementation** (Python):
```python
import numpy as np

def dh_transform(theta, d, a, alpha):
    """Create transformation matrix from DH parameters"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d   ],
        [0,   0,      0,     1   ]
    ])

def forward_kinematics_leg(joint_angles):
    """
    Calculate foot position from leg joint angles
    joint_angles: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
    Returns: 4x4 transformation matrix
    """
    # DH parameters (example values)
    dh_params = [
        (joint_angles[0], 0,     0,    np.pi/2),  # Hip Yaw
        (joint_angles[1], 0,     0,    np.pi/2),  # Hip Roll
        (joint_angles[2], 0,     0,   -np.pi/2),  # Hip Pitch
        (joint_angles[3], 0.35,  0,    0),        # Knee
        (joint_angles[4], 0.32,  0,    np.pi/2),  # Ankle Pitch
        (joint_angles[5], 0,     0.08, 0),        # Ankle Roll
    ]

    T = np.eye(4)
    for theta, d, a, alpha in dh_params:
        T = T @ dh_transform(theta, d, a, alpha)

    return T
```

### Inverse Kinematics (IK)

**Purpose**: Calculate joint angles to reach a desired end-effector position

**Methods**:

1. **Analytical IK** (preferred for legs, faster):
   - Closed-form solution using geometry
   - Fast, predictable
   - Works well for 6-DOF legs

2. **Numerical IK** (for complex cases):
   - Jacobian-based methods
   - Damped Least Squares
   - FABRIK (Forward And Backward Reaching IK)

**Analytical IK for Leg** (Simplified 2D example):

```python
def inverse_kinematics_leg_2d(target_x, target_z, thigh_length, shin_length):
    """
    Simple 2D IK for sagittal plane (pitch only)
    Returns: (hip_pitch, knee_pitch)
    """
    # Distance to target
    distance = np.sqrt(target_x**2 + target_z**2)

    # Check reachability
    if distance > (thigh_length + shin_length):
        return None  # Target unreachable

    # Knee angle (law of cosines)
    cos_knee = (thigh_length**2 + shin_length**2 - distance**2) / \
               (2 * thigh_length * shin_length)
    knee_angle = np.pi - np.arccos(np.clip(cos_knee, -1, 1))

    # Hip angle
    angle_to_target = np.arctan2(target_z, target_x)
    cos_hip = (thigh_length**2 + distance**2 - shin_length**2) / \
              (2 * thigh_length * distance)
    hip_angle = angle_to_target - np.arccos(np.clip(cos_hip, -1, 1))

    return hip_angle, knee_angle
```

**Full 6-DOF Leg IK** (3D):
- More complex, involves multiple steps
- First solve position (hip pitch, roll, knee, ankle pitch)
- Then solve orientation (hip yaw, ankle roll)
- Requires iterative numerical methods or lookup tables

---

## Balance & Stability Control

### Zero Moment Point (ZMP)

**Concept**: Point on the ground where the net moment of ground reaction forces is zero

**Stability Criterion**:
- ZMP must remain inside the support polygon (area of foot contact)
- If ZMP moves outside, robot will tip over

**Support Polygon**:
- **Single Support**: Area of one foot (during swing phase)
- **Double Support**: Convex hull of both feet (during stance phase)

**ZMP Calculation**:
```python
def calculate_zmp(com_position, com_acceleration, total_mass, gravity=9.81):
    """
    Calculate ZMP position
    com_position: [x, y, z] - Center of Mass position
    com_acceleration: [ax, ay, az] - CoM acceleration
    total_mass: robot mass (kg)
    Returns: [zmp_x, zmp_y] on ground plane
    """
    # Simplified ZMP equation (flat ground, z=0)
    zmp_x = com_position[0] - (com_position[2] / (com_acceleration[2] + gravity)) * com_acceleration[0]
    zmp_y = com_position[1] - (com_position[2] / (com_acceleration[2] + gravity)) * com_acceleration[1]

    return np.array([zmp_x, zmp_y])
```

### Balance Controller

**Implementation**: PID control on torso orientation

```python
class BalanceController:
    def __init__(self, kp_roll=1.0, ki_roll=0.1, kd_roll=0.5,
                       kp_pitch=1.0, ki_pitch=0.1, kd_pitch=0.5):
        self.kp_roll, self.ki_roll, self.kd_roll = kp_roll, ki_roll, kd_roll
        self.kp_pitch, self.ki_pitch, self.kd_pitch = kp_pitch, ki_pitch, kd_pitch

        self.integral_roll = 0
        self.integral_pitch = 0
        self.prev_error_roll = 0
        self.prev_error_pitch = 0

    def update(self, imu_roll, imu_pitch, target_roll=0, target_pitch=0, dt=0.01):
        """
        Calculate ankle corrections to maintain balance
        imu_roll, imu_pitch: Current orientation from IMU (radians)
        target_roll, target_pitch: Desired orientation
        dt: Time step (seconds)
        Returns: (ankle_roll_correction, ankle_pitch_correction)
        """
        # Errors
        error_roll = target_roll - imu_roll
        error_pitch = target_pitch - imu_pitch

        # Integral
        self.integral_roll += error_roll * dt
        self.integral_pitch += error_pitch * dt

        # Derivative
        derivative_roll = (error_roll - self.prev_error_roll) / dt
        derivative_pitch = (error_pitch - self.prev_error_pitch) / dt

        # PID output
        correction_roll = (self.kp_roll * error_roll +
                          self.ki_roll * self.integral_roll +
                          self.kd_roll * derivative_roll)

        correction_pitch = (self.kp_pitch * error_pitch +
                           self.ki_pitch * self.integral_pitch +
                           self.kd_pitch * derivative_pitch)

        # Update previous errors
        self.prev_error_roll = error_roll
        self.prev_error_pitch = error_pitch

        return correction_roll, correction_pitch
```

### Center of Mass (CoM) Calculation

```python
def calculate_com(link_positions, link_masses):
    """
    Calculate overall center of mass
    link_positions: List of [x, y, z] positions for each link
    link_masses: List of masses for each link
    Returns: [x, y, z] position of CoM
    """
    total_mass = sum(link_masses)
    com = np.zeros(3)

    for pos, mass in zip(link_positions, link_masses):
        com += np.array(pos) * mass

    com /= total_mass
    return com
```

---

## Gait Generation (Walking)

### Walking Phases

1. **Double Support**: Both feet on ground
2. **Single Support (Right)**: Right foot support, left foot swinging
3. **Double Support**: Both feet on ground
4. **Single Support (Left)**: Left foot support, right foot swinging

### Simple Walking Gait

**Parameters**:
- **Step Length**: 0.10 m (10 cm)
- **Step Height**: 0.05 m (5 cm)
- **Step Duration**: 1.0 second
- **Double Support Duration**: 0.2 seconds

**Trajectory Generation**:

```python
class WalkingGait:
    def __init__(self, step_length=0.10, step_height=0.05, step_time=1.0):
        self.step_length = step_length
        self.step_height = step_height
        self.step_time = step_time
        self.phase = 0  # 0 to 1 (one step cycle)

    def get_swing_foot_trajectory(self, phase):
        """
        Calculate swing foot position
        phase: 0 to 1 (0=liftoff, 1=touchdown)
        Returns: (x, y, z) relative to initial position
        """
        # X: Linear progression
        x = phase * self.step_length

        # Z: Arc (parabolic or sinusoidal)
        z = self.step_height * np.sin(np.pi * phase)

        # Y: Constant (no lateral movement)
        y = 0

        return x, y, z

    def step(self, dt):
        """Update gait phase"""
        self.phase += dt / self.step_time
        if self.phase >= 1.0:
            self.phase = 0
            return True  # Step complete
        return False
```

**CoM Trajectory**:
- During swing: Shift CoM over support foot
- Smooth transition using splines or sinusoids

```python
def get_com_trajectory(support_foot_position, swing_progress):
    """
    Calculate desired CoM position during walking
    support_foot_position: [x, y, z] of support foot
    swing_progress: 0 to 1 (progress through swing phase)
    """
    # Shift CoM towards support foot
    lateral_shift = 0.05  # 5 cm shift
    com_x = support_foot_position[0] + lateral_shift * (1 - swing_progress)
    com_y = support_foot_position[1]
    com_z = 0.70  # Height of CoM (approximately torso height)

    return np.array([com_x, com_y, com_z])
```

---

## Sensor Fusion

### IMU Data Processing

**Raw IMU Data**:
- Accelerometer: Linear acceleration (m/s²)
- Gyroscope: Angular velocity (rad/s)
- Magnetometer: Magnetic field (µT)

**Fusion Algorithm**: BNO055 has onboard fusion (outputs quaternion or Euler angles)

**Alternative: Complementary Filter** (if using MPU6050):

```python
class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha  # Weight for gyro (0.98 = 98% gyro, 2% accel)
        self.roll = 0
        self.pitch = 0

    def update(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, dt):
        """
        Fuse accelerometer and gyroscope data
        accel_*: Acceleration (m/s²)
        gyro_*: Angular velocity (rad/s)
        dt: Time step (s)
        """
        # Accelerometer-based angles (noisy but no drift)
        accel_roll = np.arctan2(accel_y, accel_z)
        accel_pitch = np.arctan2(-accel_x, np.sqrt(accel_y**2 + accel_z**2))

        # Gyroscope integration (smooth but drifts)
        gyro_roll = self.roll + gyro_x * dt
        gyro_pitch = self.pitch + gyro_y * dt

        # Complementary filter
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch

        return self.roll, self.pitch
```

### Force Sensor Calibration

```python
def calibrate_force_sensors(raw_readings, num_samples=100):
    """
    Calibrate FSR sensors (determine zero offset)
    raw_readings: List of ADC readings with no load
    Returns: Zero offset value
    """
    return np.mean(raw_readings)

def force_from_fsr(adc_value, zero_offset, calibration_weight=1.0, calibration_adc=500):
    """
    Convert ADC reading to force (kg)
    adc_value: Current ADC reading (0-1023)
    zero_offset: Calibrated zero reading
    calibration_weight: Known weight used for calibration (kg)
    calibration_adc: ADC reading at calibration weight
    """
    if adc_value <= zero_offset:
        return 0

    # Linear approximation
    force = (adc_value - zero_offset) / (calibration_adc - zero_offset) * calibration_weight
    return max(0, force)
```

---

## Software Components

### Arduino Firmware

**Main Loop Structure**:

```cpp
// Arduino sketch: humanoid_controller.ino

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <DynamixelShield.h>

// Hardware objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
DynamixelShield dxl;

// State variables
float imu_roll, imu_pitch, imu_yaw;
int fsr_values[8];
uint16_t servo_positions[27];

// Constants
const int E_STOP_PIN = 2;
const float LOOP_RATE = 100; // Hz
const float DT = 1.0 / LOOP_RATE;

void setup() {
  Serial.begin(115200);  // Communication with Jetson

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("ERROR:IMU_INIT_FAILED");
    while(1);
  }

  // Initialize Dynamixel
  dxl.begin(1000000);  // 1 Mbps

  // Emergency stop
  pinMode(E_STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(E_STOP_PIN), eStopISR, FALLING);

  Serial.println("STATUS:READY");
}

void loop() {
  static unsigned long last_time = millis();
  unsigned long current_time = millis();

  if (current_time - last_time >= (1000 / LOOP_RATE)) {
    // Read sensors
    readIMU();
    readForceSensors();

    // Process commands from Jetson
    processSerialCommands();

    // Safety checks
    checkBatteryVoltage();
    checkServoTemperatures();

    // Send sensor data to Jetson
    sendSensorData();

    last_time = current_time;
  }
}

void readIMU() {
  sensors_event_t event;
  bno.getEvent(&event);

  imu_roll = event.orientation.x;
  imu_pitch = event.orientation.y;
  imu_yaw = event.orientation.z;
}

void readForceSensors() {
  for (int i = 0; i < 8; i++) {
    fsr_values[i] = analogRead(A0 + i);
  }
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');

    // Parse command (example: "MOVE,5,1024,100")
    if (command.startsWith("MOVE")) {
      // Extract servo ID, position, speed
      int servo_id = getValue(command, ',', 1).toInt();
      int position = getValue(command, ',', 2).toInt();
      int speed = getValue(command, ',', 3).toInt();

      // Send to servo
      dxl.setGoalPosition(servo_id, position, UNIT_RAW);
      dxl.setGoalVelocity(servo_id, speed, UNIT_RAW);
    }
  }
}

void sendSensorData() {
  // Format: "IMU,roll,pitch,yaw;FSR,f0,f1,...,f7"
  Serial.print("IMU,");
  Serial.print(imu_roll); Serial.print(",");
  Serial.print(imu_pitch); Serial.print(",");
  Serial.print(imu_yaw); Serial.print(";");

  Serial.print("FSR");
  for (int i = 0; i < 8; i++) {
    Serial.print(",");
    Serial.print(fsr_values[i]);
  }
  Serial.println();
}

void eStopISR() {
  // Emergency stop interrupt
  for (int i = 1; i <= 27; i++) {
    dxl.torqueOff(i);  // Disable all servos
  }
  Serial.println("EMERGENCY_STOP");
}
```

### Jetson Python Control Software

**Main Control Loop**:

```python
#!/usr/bin/env python3
# jetson_main_controller.py

import serial
import time
import numpy as np
from kinematics import InverseKinematics
from balance_controller import BalanceController
from gait_generator import WalkingGait

class HumanoidController:
    def __init__(self, arduino_port='/dev/ttyACM0', baud_rate=115200):
        # Serial communication with Arduino
        self.arduino = serial.Serial(arduino_port, baud_rate, timeout=0.1)
        time.sleep(2)  # Wait for Arduino to initialize

        # Controllers
        self.ik = InverseKinematics()
        self.balance = BalanceController()
        self.gait = WalkingGait()

        # State
        self.imu_data = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.fsr_data = [0] * 8
        self.mode = 'IDLE'  # IDLE, STANDING, WALKING

        print("Humanoid controller initialized")

    def read_sensors(self):
        """Read sensor data from Arduino"""
        if self.arduino.in_waiting:
            line = self.arduino.readline().decode('utf-8').strip()

            # Parse sensor data
            if line.startswith('IMU'):
                parts = line.split(';')
                imu_parts = parts[0].split(',')
                self.imu_data['roll'] = float(imu_parts[1])
                self.imu_data['pitch'] = float(imu_parts[2])
                self.imu_data['yaw'] = float(imu_parts[3])

                if len(parts) > 1:
                    fsr_parts = parts[1].split(',')
                    self.fsr_data = [int(x) for x in fsr_parts[1:]]

    def send_servo_command(self, servo_id, position, speed=100):
        """Send servo movement command to Arduino"""
        cmd = f"MOVE,{servo_id},{position},{speed}\n"
        self.arduino.write(cmd.encode())

    def standing_mode(self):
        """Maintain standing balance"""
        # Get balance corrections from PID controller
        roll_correction, pitch_correction = self.balance.update(
            np.radians(self.imu_data['roll']),
            np.radians(self.imu_data['pitch'])
        )

        # Apply corrections to ankle joints (simplified)
        # In reality, would use IK to calculate all leg joint angles
        ankle_roll_offset = int(roll_correction * 100)  # Scale to servo units
        ankle_pitch_offset = int(pitch_correction * 100)

        # Send to servos (example for left ankle)
        self.send_servo_command(5, 2048 + ankle_pitch_offset)  # Left ankle pitch
        self.send_servo_command(6, 2048 + ankle_roll_offset)   # Left ankle roll

    def walking_mode(self):
        """Execute walking gait"""
        # Get current phase foot positions
        swing_foot_pos = self.gait.get_swing_foot_trajectory(self.gait.phase)

        # Calculate inverse kinematics for swing leg
        swing_leg_angles = self.ik.leg_ik(swing_foot_pos)

        # Send to servos
        # (Details depend on which leg is swinging)

        # Update gait phase
        self.gait.step(dt=0.01)

    def run(self):
        """Main control loop"""
        print("Starting main control loop (Ctrl+C to exit)")

        try:
            while True:
                # Read sensors
                self.read_sensors()

                # Execute mode-specific control
                if self.mode == 'STANDING':
                    self.standing_mode()
                elif self.mode == 'WALKING':
                    self.walking_mode()

                # Loop timing
                time.sleep(0.01)  # 100 Hz

        except KeyboardInterrupt:
            print("\nShutting down...")
            self.arduino.close()

if __name__ == '__main__':
    controller = HumanoidController()
    controller.mode = 'STANDING'
    controller.run()
```

---

## ROS 2 Integration (Recommended)

### Why ROS 2?

**Benefits**:
- Modular architecture (nodes, topics, services)
- Built-in communication (DDS)
- Extensive libraries (navigation, vision, etc.)
- Simulation integration (Gazebo)
- Community support

**ROS 2 Packages for Humanoid**:

```
humanoid_robot/
├── humanoid_description/     # URDF models
├── humanoid_bringup/         # Launch files
├── humanoid_control/         # Controllers (IK, balance, gait)
├── humanoid_vision/          # Camera processing
├── humanoid_navigation/      # Path planning
└── humanoid_msgs/            # Custom message types
```

### Example ROS 2 Node (Balance Controller):

```python
#!/usr/bin/env python3
# balance_controller_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

class BalanceControllerNode(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )

        # Timer (control loop at 100 Hz)
        self.timer = self.create_timer(0.01, self.control_loop)

        # State
        self.current_roll = 0.0
        self.current_pitch = 0.0

    def imu_callback(self, msg):
        # Extract roll and pitch from quaternion
        # (Simplified - use tf_transformations for real implementation)
        self.current_roll = msg.orientation.x
        self.current_pitch = msg.orientation.y

    def control_loop(self):
        # Calculate corrections (PID control)
        roll_correction = -self.current_roll * 0.5  # Simple P control
        pitch_correction = -self.current_pitch * 0.5

        # Create joint command message
        cmd = Float64MultiArray()
        cmd.data = [roll_correction, pitch_correction]

        self.joint_cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BalanceControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Simulation

### PyBullet Setup

**Purpose**: Test controllers before deploying to real hardware

**Installation**:
```bash
pip3 install pybullet
```

**Simple Simulation**:

```python
import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load ground plane
planeId = p.loadURDF("plane.urdf")

# Load robot (URDF created from CAD)
robotId = p.loadURDF("humanoid_robot.urdf", [0, 0, 1])

# Simulation loop
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
```

**URDF Creation**:
- Export CAD model to URDF using Fusion 360 plugin or SolidWorks to URDF
- Define links, joints, masses, inertias
- Test in simulation before building hardware

---

## Vision Processing

### Camera Calibration

**Purpose**: Correct lens distortion, calculate camera intrinsics

**Using OpenCV**:

```python
import cv2
import numpy as np

def calibrate_camera(images_path, checkerboard_size=(9, 6)):
    """
    Calibrate camera using checkerboard images
    checkerboard_size: (columns, rows) of inner corners
    """
    # Prepare object points
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0],
                            0:checkerboard_size[1]].T.reshape(-1, 2)

    objpoints = []  # 3D points
    imgpoints = []  # 2D points

    images = glob.glob(images_path + '/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)

    # Calibrate
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    return mtx, dist  # Camera matrix and distortion coefficients
```

### Stereo Vision (Depth Estimation)

```python
def stereo_depth_map(left_image, right_image, camera_matrix, baseline=0.06):
    """
    Calculate depth map from stereo images
    baseline: Distance between cameras (m)
    """
    # Stereo matcher
    stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=15)

    # Convert to grayscale
    left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

    # Compute disparity
    disparity = stereo.compute(left_gray, right_gray)

    # Calculate depth
    focal_length = camera_matrix[0, 0]  # Focal length in pixels
    depth_map = (focal_length * baseline) / (disparity + 1e-6)

    return depth_map
```

---

## State Machine

### Behavior States

```python
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    INITIALIZING = 1
    STANDING = 2
    WALKING = 3
    TURNING = 4
    SITTING = 5
    FALLING = 6
    EMERGENCY_STOP = 7

class StateMachine:
    def __init__(self):
        self.state = RobotState.IDLE
        self.previous_state = RobotState.IDLE

    def transition(self, new_state):
        """Transition to new state"""
        print(f"State transition: {self.state.name} -> {new_state.name}")
        self.previous_state = self.state
        self.state = new_state
        self.on_enter(new_state)

    def on_enter(self, state):
        """Actions to perform when entering a state"""
        if state == RobotState.STANDING:
            print("Entering standing mode - activating balance controller")
        elif state == RobotState.WALKING:
            print("Entering walking mode - starting gait generator")
        elif state == RobotState.EMERGENCY_STOP:
            print("EMERGENCY STOP - disabling all motors")

    def update(self, imu_data, fsr_data):
        """Update state machine based on sensor data"""
        # Example: Detect fall
        if abs(imu_data['roll']) > 30 or abs(imu_data['pitch']) > 30:
            self.transition(RobotState.FALLING)
```

---

## File Structure

```
/home/jetson/humanoid_robot/
├── src/
│   ├── main_controller.py         # Main control loop
│   ├── kinematics.py               # FK/IK implementations
│   ├── balance_controller.py      # Balance PID controller
│   ├── gait_generator.py          # Walking gait
│   ├── sensor_fusion.py           # IMU and sensor processing
│   ├── vision/
│   │   ├── camera_calibration.py
│   │   ├── stereo_vision.py
│   │   └── object_detection.py
│   └── utils/
│       ├── serial_comm.py         # Arduino communication
│       └── config.py              # Configuration parameters
├── config/
│   ├── robot_config.yaml          # Robot parameters (dimensions, etc.)
│   ├── servo_ids.yaml             # Servo ID mappings
│   └── calibration.yaml           # Sensor calibration data
├── urdf/
│   └── humanoid_robot.urdf        # Robot model for simulation
├── tests/
│   ├── test_kinematics.py
│   ├── test_balance.py
│   └── test_gait.py
└── docs/
    └── API.md                     # Software API documentation
```

---

## Next Steps

1. **Set up development environment** on Jetson and Arduino
2. **Implement basic serial communication** between Jetson and Arduino
3. **Test individual servos** using Dynamixel Wizard
4. **Implement forward kinematics** in Python
5. **Build simulation** in PyBullet with URDF
6. **Test inverse kinematics** in simulation
7. **Implement basic balance controller**
8. **Test on hardware** incrementally (single leg → full lower body → complete robot)

---

**Document Status**: Initial Draft
**Next Review**: After software development begins
**Maintained By**: Project owner
