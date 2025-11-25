# Testing and Calibration Procedures

## Overview

This document provides detailed procedures for testing and calibrating all systems of the humanoid robot. Proper calibration is essential for safe and reliable operation.

**Last Updated**: 2025-11-26

---

## Testing Philosophy

### Progressive Testing Approach

**Never skip steps!** Each level builds on the previous:

1. **Component Level**: Test individual parts
2. **Subsystem Level**: Test assemblies (single leg, arm, etc.)
3. **System Level**: Test integrated systems (lower body, full robot)
4. **Functional Level**: Test behaviors (standing, walking, manipulation)

### Safety During Testing

**Always**:
- Have emergency stop readily accessible
- Use support/safety harness for walking tests
- Test in padded area
- Start with low power/speed
- Monitor servo temperatures
- Keep battery fire safety equipment nearby

**Never**:
- Test with damaged components
- Override safety limits in software
- Test alone (have someone nearby for safety)
- Rush through procedures

---

## Component-Level Testing

### Servo Testing

**Equipment Needed**:
- Dynamixel Wizard software
- U2D2 USB interface
- 12V power supply (or battery)
- Multimeter

**Procedure for Each Servo**:

1. **Visual Inspection**:
   - Check for physical damage
   - Verify connector integrity
   - Check for loose screws

2. **Initial Connection**:
   ```
   Connect: Power Supply (12V) ─┬─► Servo power
                                 └─► U2D2 power (if required)

   Connect: U2D2 ─► Servo data cable

   Connect: U2D2 ─► Computer USB
   ```

3. **Scan and Identify**:
   - Open Dynamixel Wizard
   - Click "Scan" (try all baud rates if unknown)
   - Verify servo is detected
   - Record: Model, ID, firmware version

4. **Set Servo ID**:
   - Change ID to assigned value (see Electrical Design doc)
   - Example: Right hip yaw = ID 1
   - Verify: Rescan and confirm new ID

5. **Movement Test**:
   - Move servo to center position (2048 for Dynamixel)
   - Command full range of motion
   - Observe: Smooth movement, no jerking
   - Listen: No unusual noise (grinding, clicking)

6. **Performance Test**:
   - Measure no-load current: ~0.2-0.4A
   - Measure stall current (brief!): Should match spec
   - Check temperature after 30 seconds: Should be warm, not hot
   - Verify position accuracy: ±1% of commanded position

7. **Configure Settings**:
   ```
   Recommended Settings:
   - Baud Rate: 1000000 (1 Mbps)
   - Return Delay: 0
   - Operating Mode: Position Control (default)
   - Temperature Limit: 70°C
   - Voltage Limit: 12-16V (for 4S LiPo)
   - Torque Limit: 80% of max (safety margin)
   ```

8. **Document Results**:
   - Servo ID, model, firmware
   - Center position value
   - Min/max positions
   - Any issues or anomalies

**Acceptance Criteria**:
- [ ] Servo responds to commands
- [ ] Smooth motion throughout range
- [ ] Current draw within specification
- [ ] No excessive heat
- [ ] No unusual noise
- [ ] Position accuracy within spec

### IMU Testing

**Equipment**: Arduino/Jetson, I2C connection

**Procedure**:

1. **Connection Test**:
   ```python
   # Python test script
   from adafruit_bno055 import BNO055
   import board

   i2c = board.I2C()
   sensor = BNO055(i2c)

   print("IMU connected:", sensor.temperature)
   ```

2. **Calibration**:
   - Power on IMU
   - Perform figure-8 motion (magnetometer)
   - Place on flat surface (accelerometer/gyro)
   - Wait for calibration (status: 3,3,3,3 = fully calibrated)
   - Save calibration offsets

3. **Orientation Test**:
   - Place IMU flat (0° roll, 0° pitch)
   - Read values: Should be near 0°
   - Tilt 90°: Read values, should match
   - Rotate 180°: Verify yaw change

4. **Drift Test**:
   - Place IMU stationary
   - Record orientation for 5 minutes
   - Calculate drift rate (°/hour)
   - Acceptable: <1°/hour

**Acceptance Criteria**:
- [ ] I2C communication working
- [ ] Calibration achievable
- [ ] Orientation accuracy: ±2°
- [ ] Minimal drift

### Force Sensor (FSR) Testing

**Equipment**: Known weights, multimeter

**Procedure**:

1. **Zero Calibration**:
   - Read FSR with no load (100 samples)
   - Calculate average: This is zero offset
   - Record: ~50-100 (ADC value)

2. **Load Calibration**:
   - Place known weight (e.g., 1 kg)
   - Read ADC value (100 samples, average)
   - Calculate calibration factor:
     ```
     Factor = (ADC_loaded - ADC_zero) / Weight
     ```

3. **Linearity Test**:
   - Test with multiple weights (0.5, 1, 2, 3 kg)
   - Plot ADC vs Weight
   - Check linearity (R² > 0.95)

4. **Repeatability**:
   - Apply and remove weight 10 times
   - Verify consistent readings (±5%)

**Acceptance Criteria**:
- [ ] Responds to applied load
- [ ] Reasonably linear response
- [ ] Repeatable readings
- [ ] No drift over time

### Camera Testing

**Procedure**:

1. **Connection**:
   - Connect camera to Jetson CSI port
   - Run test capture:
     ```bash
     nvgstcapture-1.0
     ```

2. **Image Quality**:
   - Check focus (adjust if needed)
   - Check exposure (auto-exposure working)
   - Check color balance
   - Verify resolution (1920×1080 or higher)

3. **Stereo Alignment**:
   - Mount both cameras
   - Capture simultaneous images
   - Verify:
     - Horizontal alignment (same height)
     - Epipolar lines roughly horizontal
     - Baseline distance correct (6-7 cm)

4. **Latency Test**:
   - Measure capture to processing time
   - Should be <100ms for real-time control

**Acceptance Criteria**:
- [ ] Clear images captured
- [ ] Good focus and exposure
- [ ] Stereo pair aligned
- [ ] Low latency

---

## Subsystem Testing

### Single Leg Test

**Prerequisites**: Leg fully assembled, servos configured

**Setup**:
- Mount leg vertically in test stand
- Connect power and control
- Attach safety strap (in case servo fails)

**Test 1: Joint Range of Motion**

For each joint:
1. Move to minimum position
2. Move to center position
3. Move to maximum position
4. Verify no mechanical interference

**Test 2: Coordinated Movement**

1. Execute preset poses:
   ```
   Pose 1: Straight leg (all joints centered)
   Pose 2: Bent knee (knee at 90°)
   Pose 3: Extended (hip and knee fully extended)
   Pose 4: Flexed (hip and knee fully flexed)
   ```

2. Verify smooth transitions between poses

**Test 3: Force Sensor Integration**

1. Press on foot (known force)
2. Verify FSR readings match applied force
3. Test all 4 sensors per foot

**Test 4: Inverse Kinematics Test**

1. Command foot to specific position (X, Y, Z)
2. Measure actual foot position
3. Verify position error <5mm

**Acceptance Criteria**:
- [ ] Full range of motion achieved
- [ ] No binding or interference
- [ ] Smooth coordinated motion
- [ ] Force sensors working
- [ ] IK accuracy acceptable

### Power System Test

**Test 1: No Load**

1. Connect battery (fully charged, 16.8V)
2. Measure output voltages:
   - 12V rails: 12.0V ±0.2V
   - 5V rails: 5.0V ±0.05V
3. Check current draw: <0.5A (idle)

**Test 2: Load Test**

1. Connect all servos
2. Move all servos to center (low load)
3. Measure:
   - Battery voltage: Should stay >14V
   - 12V rail: Should stay >11.5V
   - Current: ~5-10A (typical)

**Test 3: Peak Load**

1. Command multiple servos to move simultaneously
2. Measure peak current: ~20-30A
3. Verify no voltage sag below safe limits
4. Check for overheating (buck converters, wiring)

**Test 4: Battery Endurance**

1. Run robot in standing mode
2. Measure time to low voltage cutoff (13.2V)
3. Should achieve >30 minutes typical operation

**Test 5: Safety Systems**

1. Test emergency stop:
   - Press e-stop
   - Verify all servos disable immediately
2. Test low voltage cutoff:
   - Discharge battery to cutoff point
   - Verify system shuts down safely
3. Test overcurrent protection:
   - Simulate overcurrent (carefully!)
   - Verify fuse blows or BMS disconnects

**Acceptance Criteria**:
- [ ] All voltage rails stable
- [ ] No excessive voltage drop under load
- [ ] Battery runtime meets target
- [ ] All safety systems functional

---

## System Integration Testing

### Lower Body Integration Test

**Prerequisites**: Both legs, pelvis, power system integrated

**Test 1: Static Standing**

Setup:
- Place robot on flat surface
- Support robot with overhead harness (minimal tension)
- Power on, enable servos

Procedure:
1. Command standing pose (all joints to standing positions)
2. Verify:
   - Both feet flat on ground
   - Even weight distribution (check FSR sensors)
   - Torso level (check IMU)
3. Duration: 5 minutes minimum

Measurements:
- FSR values: Should be balanced (±10%)
- Servo currents: <0.5A per servo (holding torque)
- IMU: Roll and pitch <2°

**Test 2: Weight Shifting**

Procedure:
1. Shift weight to left foot
   - Reduce right FSR readings to near zero
   - Increase left FSR readings
2. Shift weight to right foot
   - Opposite of above
3. Return to center
4. Repeat 10 times

Verify:
- Smooth weight transition
- FSR readings match expected pattern
- Robot remains stable throughout

**Test 3: Single-Leg Balance**

Procedure:
1. Shift weight to one leg (support leg)
2. Lift opposite foot 2-3 cm off ground
3. Hold for 5 seconds
4. Lower foot
5. Repeat for other leg

Verify:
- Robot maintains balance
- Support leg FSRs show full weight
- IMU shows minimal tilt
- Balance controller compensating

**Test 4: Balance Perturbation**

Procedure:
1. Robot standing in balanced pose
2. Apply gentle push (front, back, left, right)
3. Observe:
   - Robot resists push
   - Returns to balanced state
   - No excessive oscillation

Tune:
- Adjust PID gains if needed
- Increase if too slow to respond
- Decrease if oscillating

**Acceptance Criteria**:
- [ ] Stable standing for 5+ minutes
- [ ] Smooth weight shifting
- [ ] Can balance on one leg
- [ ] Recovers from perturbations

### Full Robot Integration Test

**Prerequisites**: Arms and head attached, all systems integrated

**Test 1: Power-On Sequence**

1. Visual inspection (all connections secure)
2. Battery voltage check (>14V)
3. Power on main switch
4. Verify all systems initialize:
   - Power LEDs on
   - Jetson boots (monitor via HDMI)
   - Arduino initializes (serial output)
   - All servos detected (27 servos)
   - Sensors responding (IMU, FSR, cameras)

**Test 2: Full System Check**

1. Test each subsystem:
   - Legs (both): Range of motion
   - Arms (both): Range of motion
   - Head: Pan/tilt/roll
   - Grippers: Open/close
   - Waist: Rotation, lean
2. Test sensors:
   - IMU: Orientation reading
   - FSR: Weight detection
   - Cameras: Image capture

**Test 3: Integrated Standing**

1. Command full-body standing pose
2. Verify:
   - Stable balance
   - Arms at sides or in balanced position
   - Head forward
3. Duration: 10 minutes

**Test 4: Upper Body Movement While Standing**

1. Robot standing
2. Move arms (one at a time, then both):
   - Raise forward
   - Raise sideways
   - Combined movements
3. Move head:
   - Look left/right
   - Look up/down
4. Verify:
   - Balance maintained (CoM compensation)
   - Smooth movements
   - No interference between parts

**Acceptance Criteria**:
- [ ] All 27+ servos functional
- [ ] All sensors working
- [ ] Stable standing with full body
- [ ] Can move upper body while maintaining balance

---

## Calibration Procedures

### Servo Position Calibration

**Purpose**: Align servo positions with actual joint angles

**Procedure for Each Joint**:

1. **Find Physical Zero**:
   - Manually position joint to neutral/zero position
     - Example: Knee fully straight = 0°
   - Use angle gauge or protractor to verify
   - Note servo position value (e.g., 2048)

2. **Record Position Mapping**:
   ```
   Joint: Right Knee
   Physical Angle | Servo Position
   0°            | 2048
   45°           | 2560
   90°           | 3072
   140°          | 3800
   ```

3. **Create Calibration Table**:
   ```python
   # In software
   KNEE_CALIBRATION = {
       'center': 2048,
       'min': 2048,      # 0°
       'max': 3800,      # 140°
       'degrees_per_unit': 140 / (3800 - 2048)
   }
   ```

4. **Verify**:
   - Command specific angle (e.g., 90°)
   - Measure actual angle with gauge
   - Should match within ±2°

**Repeat for All 27 Joints**

### IMU Calibration

**Accelerometer Calibration**:

1. Place IMU on level surface
2. Record readings (ax, ay, az)
3. Expected: ax=0, ay=0, az=9.81 m/s²
4. Calculate offsets:
   ```
   offset_x = 0 - measured_ax
   offset_y = 0 - measured_ay
   offset_z = 9.81 - measured_az
   ```

**Gyroscope Calibration**:

1. Place IMU stationary
2. Record readings for 30 seconds
3. Calculate average (this is bias)
4. Subtract bias in software

**Magnetometer Calibration**:

1. Perform figure-8 motion in all axes
2. BNO055 auto-calibrates
3. Save calibration coefficients:
   ```python
   calibration = sensor.calibration_status()
   offsets = sensor.offsets()
   # Save offsets to file for next boot
   ```

### Force Sensor Calibration Matrix

**Purpose**: Account for crosstalk between sensors

**Procedure**:

1. **Zero Calibration** (no load):
   - Record all 8 FSR values
   - Average over 100 samples
   - Store as zero offsets

2. **Individual Sensor Calibration**:
   - Place 1kg weight on sensor 1
   - Record all 8 sensor readings
   - Note: Sensor 1 should read high, others may read low crosstalk
   - Repeat for all 8 sensors

3. **Create Calibration Matrix**:
   ```python
   # If sensor 1 shows 500 with 1kg, and sensor 2 shows 50 crosstalk:
   # Calibration matrix accounts for this
   calibration_matrix = np.array([
       [1.0, -0.1, -0.05, ...],  # Sensor 1 coefficients
       [-0.1, 1.0, -0.05, ...],  # Sensor 2 coefficients
       ...
   ])

   # Apply calibration:
   true_forces = calibration_matrix @ raw_sensor_values
   ```

### Center of Mass Calibration

**Purpose**: Find actual CoM position (may differ from CAD)

**Procedure**:

1. **Suspend Robot**:
   - Hang robot from known point (e.g., torso center)
   - Let it hang freely
   - Mark plumb line from suspension point

2. **Repeat from Different Points**:
   - Suspend from different locations
   - Draw plumb lines
   - CoM is where lines intersect

3. **Verify**:
   - Place robot on balance beam (knife edge)
   - Adjust until balanced
   - CoM should be directly above edge

4. **Record**:
   ```python
   COM_POSITION = {
       'x': 0.02,    # 2cm from geometric center
       'y': 0.00,    # On centerline
       'z': 0.72     # 72cm from ground
   }
   ```

### Stereo Camera Calibration

**Equipment**: Checkerboard pattern (9×6, 25mm squares)

**Procedure**:

1. **Individual Camera Calibration**:
   - Capture 20-30 images of checkerboard from different angles
   - Run OpenCV calibration:
     ```python
     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(...)
     ```
   - Repeat for both cameras

2. **Stereo Calibration**:
   - Capture simultaneous images from both cameras
   - Run stereo calibration:
     ```python
     ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(...)
     ```
   - R = Rotation matrix between cameras
   - T = Translation vector (baseline)

3. **Verify**:
   - Compute depth map
   - Compare with known distances
   - Accuracy should be ±5% at 1m distance

---

## Functional Testing

### Walking Test Progression

**Level 1: Stepping in Place**

1. Start in standing position
2. Lift one foot 3cm
3. Hold for 1 second
4. Lower foot to original position
5. Repeat with other foot
6. Continue for 10 cycles

Success Criteria:
- Maintains balance throughout
- Consistent lift height
- Smooth motion

**Level 2: Forward Steps**

1. Shift weight to left foot
2. Move right foot forward 5cm
3. Place foot down
4. Shift weight to right foot
5. Move left foot forward 5cm
6. Repeat

Start with:
- Small steps (5cm)
- Slow speed (3 seconds per step)
- Few steps (5-10)

Gradually increase:
- Step length (up to 15cm)
- Speed (down to 1 second per step)
- Duration (continuous walking)

**Level 3: Continuous Walking**

1. Walk forward 10 steps
2. Stop in balanced position
3. Repeat

Tune gait parameters:
- Step length
- Step height
- Step frequency
- Double support time
- CoM shift amount

**Level 4: Advanced Walking**

1. Walk and turn (curved path)
2. Walk backward
3. Side stepping
4. Walk on slight incline
5. Walk while moving arms
6. Recover from pushes while walking

### Manipulation Testing

**Gripper Force Calibration**:

1. Place force gauge between gripper fingers
2. Close gripper
3. Record maximum force (should be 5-10N)
4. Verify force is controllable (not just on/off)

**Object Manipulation**:

Test with objects of various:
- Sizes (3-8cm diameter)
- Weights (10-500g)
- Shapes (spherical, cylindrical, rectangular)
- Materials (smooth, rough, soft, hard)

Tasks:
1. Pick up object from table
2. Hold for 10 seconds
3. Move to different location
4. Place down gently
5. Release

Success Criteria:
- Reliable grasp (>90% success)
- Doesn't drop object during transfer
- Controlled release

### Vision-Based Tasks

**Object Detection**:

1. Place known object in view
2. Run detection algorithm
3. Verify:
   - Object detected
   - Correct classification
   - Accurate position estimate

**Depth Perception**:

1. Place objects at known distances (0.5m, 1m, 2m)
2. Calculate distance using stereo vision
3. Compare with actual distance
4. Error should be <10%

**Visual Servoing**:

1. Place marker/target in view
2. Command robot to reach toward it
3. Verify:
   - Arm moves toward target
   - Gripper reaches correct position
   - Error <5cm

---

## Performance Benchmarks

### Target Performance Metrics

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Standing Duration | 30+ min | Timed test |
| Balance Recovery | <2 sec | Push test, measure recovery time |
| Walking Speed | 0.2-0.5 m/s | Distance / time |
| Battery Life (walking) | 30+ min | Continuous walking until cutoff |
| Battery Life (standing) | 2+ hours | Standing until cutoff |
| Servo Response Time | <50ms | Command to motion start |
| IMU Update Rate | 100 Hz | Software measurement |
| Camera Frame Rate | 30 fps | Software measurement |
| Gripper Success Rate | >90% | 100 grasp attempts |

### Data Logging

**Create logging system** to track:
- All sensor data (timestamped)
- All servo positions and currents
- Battery voltage/current
- Software state
- Events (falls, errors, etc.)

**Log Format** (CSV or ROS bag):
```
timestamp, imu_roll, imu_pitch, imu_yaw, fsr_0, fsr_1, ..., servo_1_pos, servo_1_current, ...
```

**Use logs for**:
- Debugging
- Performance analysis
- Tuning
- Failure analysis

---

## Troubleshooting Guide

### Walking Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Robot falls forward | CoM too far forward | Adjust CoM trajectory, lean back slightly |
| Robot falls backward | CoM too far back | Adjust CoM trajectory, lean forward |
| Robot falls sideways | Insufficient lateral shift | Increase lateral CoM shift during swing |
| Feet slip | Insufficient friction | Add rubber sole, reduce speed, smaller steps |
| Jerky walking | Poor trajectory | Smooth trajectories (use splines), tune PID |
| Can't lift foot | Insufficient CoM shift | Increase shift before swing phase |

### Balance Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Constant oscillation | PID gains too high | Reduce D and/or P gain |
| Slow to recover | PID gains too low | Increase P gain |
| Drift over time | IMU bias | Recalibrate IMU, add integral term |
| Falls when pushed | Insufficient correction | Increase P gain, faster response |

---

## Documentation of Results

### Test Report Template

```
Test: [Test Name]
Date: [YYYY-MM-DD]
Tester: [Name]

Setup:
- [Description of test setup]
- [Environmental conditions]
- [Software version]

Procedure:
1. [Step 1]
2. [Step 2]
...

Results:
- [Quantitative measurements]
- [Observations]

Pass/Fail: [Status]

Issues:
- [Any problems encountered]

Next Steps:
- [What to do next]
```

### Calibration Data Storage

Store all calibration data in YAML files:

```yaml
# calibration.yaml
servos:
  1:  # Right hip yaw
    center: 2048
    min: 1548
    max: 2548
    offset: 0
  2:  # Right hip pitch
    center: 2048
    ...

imu:
  offsets:
    accel: [0.1, -0.2, 0.05]
    gyro: [0.01, -0.01, 0.00]
    mag: [...]

force_sensors:
  left_foot:
    fl: {zero: 50, factor: 450}
    fr: {zero: 48, factor: 460}
    rl: {zero: 52, factor: 440}
    rr: {zero: 51, factor: 455}
  right_foot:
    ...

cameras:
  left:
    matrix: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
    distortion: [k1, k2, p1, p2, k3]
  right:
    ...
  stereo:
    baseline: 0.065  # meters
    rotation: [[...], [...], [...]]
```

---

## Maintenance Schedule

### Daily (if operated)
- [ ] Visual inspection
- [ ] Battery voltage check
- [ ] Test emergency stop

### Weekly
- [ ] Check all fasteners (tighten if loose)
- [ ] Inspect wiring for wear
- [ ] Clean sensors (cameras, IMU)
- [ ] Charge batteries fully

### Monthly
- [ ] Re-calibrate IMU
- [ ] Check servo performance (current, temperature)
- [ ] Inspect 3D printed parts for cracks
- [ ] Update software/firmware

### Quarterly
- [ ] Full system test
- [ ] Replace worn parts
- [ ] Deep clean
- [ ] Performance benchmarking

---

**Document Status**: Initial Draft
**Next Review**: After initial testing phase
**Maintained By**: Project owner
