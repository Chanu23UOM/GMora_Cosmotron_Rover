# Project Cosmotron - Mars Rover Competition
## University of Moratuwa - Team Cosmotron

### 📁 Project Structure
```
finalproject file/
├── protos/
│   └── GMoraRover.proto      # Custom 6-wheel Mars rover robot
├── controllers/
│   └── G_moracos/
│       └── G_moracos.py      # Autonomous navigation controller
└── worlds/
    ├── SampleTask.wbt        # Competition world file
    ├── tag25h9-0.jpg         # AprilTag ID=0 (Stop/Arrival)
    ├── tag25h9-1.jpg         # AprilTag ID=1 (Turn Right)
    ├── tag25h9-2.jpg         # AprilTag ID=2 (Turn Left)
    ├── Boundry.obj           # Arena boundary mesh
    └── Military-Fence-Gate/  # Fence textures
```

### 🚀 How to Run
1. Open Webots R2025a
2. Open `worlds/SampleTask.wbt`
3. Press Play (▶) to start the simulation
4. The rover will automatically:
   - Exit the green start zone
   - Navigate using AprilTag flags
   - Navigate to the Crimson Impact Site (red platform)
   - Pick up the asteroid
   - Store it onboard

### 🤖 Robot Specifications (GMoraRover)

#### Dimensions
- **Size**: Fits within 1.1m × 1.1m × 1.1m cube at start
- **Controller**: G_moracos

#### Mobility System
- **Drive Type**: 6-wheel differential drive with rocker-bogie suspension
- **Wheels**: FrontLeft, FrontRight, MiddleLeft, MiddleRight, BackLeft, BackRight
- **Max Velocity**: 15.0 rad/s per wheel

#### Robotic Arm (5-DoF)
- **arm1**: Base rotation (±0.35 rad)
- **arm2**: Shoulder joint (-2.35619 to 1.39626 rad)
- **arm3**: Elbow joint (-2.63545 to 2.54818 rad)
- **arm4**: Wrist pitch (-1.0472 to 3.14159 rad)
- **arm5**: Wrist rotation (±1.5708 rad)
- **Gripper**: Dual-finger with 0.025m travel

#### Sensors
- **Camera**: 640×480, FOV 1.0 rad, Recognition enabled (maxRange 20m)
- **GPS**: Position tracking
- **Gyroscope**: Angular velocity (for precise turns)
- **IMU**: Orientation sensing
- **Compass**: Heading reference
- **Distance Sensors**: 5× (front_left, front, front_right, left, right)

### 🎯 Navigation via AprilTags (25H9 Family)

| Tag ID | Texture File    | Action        |
|--------|-----------------|---------------|
| 0      | tag25h9-0.jpg   | Stop/Arrival  |
| 1      | tag25h9-1.jpg   | Turn Right    |
| 2      | tag25h9-2.jpg   | Turn Left     |

### 📋 Mission State Machine

1. **INIT** → Initialize all sensors and motors
2. **EXIT_GREEN** → Exit the green start zone  
3. **MOVE_TO_FLAG** → Navigate toward detected flags
4. **FOLLOW_FLAG** → Track and approach flags
5. **EXECUTE_TURN** → Perform 90° turns based on flag ID
6. **STOP_AT_RED** → Stop at Crimson Impact Site
7. **FIND_SAMPLE** → Locate asteroid using camera
8. **APPROACH_SAMPLE** → Move toward asteroid
9. **ALIGN_PICKUP** → Position for pickup
10. **PICKUP** → Execute arm sequence to grab asteroid
11. **STORE** → Move asteroid to storage compartment
12. **MISSION_COMPLETE** → Stop all motors

### ⚙️ Controller Configuration

```python
# Key Parameters (in G_moracos.py)
TIME_STEP = 32           # Controller timestep (ms)
MAX_WHEEL_VELOCITY = 15.0  # rad/s
TURN_ANGLE_DEGREES = 90    # Turn angle
```

### 🔧 Arm Positions

```python
ARM_POSITIONS = {
    'ARM_HOME': [0, 0, 0, 0, 0],           # Stowed position
    'ARM_PRE_GRAB': [0, -1.5, 1.2, 0.5, 0], # Ready to grab
    'ARM_GRAB': [0, -2.3, 2.5, 2.0, 0],     # Grabbing position
    'ARM_LIFT': [0, -0.8, 1.5, 0.8, 0],     # Lift asteroid
    'ARM_STORE_*': [...]                    # Storage sequence
}
```

### 🏆 Competition Scoring (Total: 100%)

| Category           | Weight |
|--------------------|--------|
| Innovation         | 30%    |
| Asteroid Retrieval | 15%    |
| Navigation         | 15%    |
| Custom Rover       | 15%    |
| Flag Recognition   | 15%    |
| Constraints        | 10%    |

### 🌍 Environment Settings

- **Gravity**: 3.73 m/s² (Mars)
- **Basic Time Step**: 16 ms
- **Fog**: Exponential, 800m visibility
- **Terrain**: Mars soil with rocks and obstacles

### 📝 Changes Made from Original

1. ✅ Fixed EXTERNPROTO path for GMoraRover.proto
2. ✅ Added `model` attribute to all AprilTag flags for camera recognition
3. ✅ Added `recognitionColors` to all flags
4. ✅ Replaced Sojourner robot with custom GMoraRover
5. ✅ Fixed relative texture paths for portability
6. ✅ Updated WorldInfo with competition details

### 🔗 Competition Requirements Met

- [x] Rover fits in 1.1m × 1.1m × 1.1m cube
- [x] Fully autonomous operation (no manual control)
- [x] AprilTag 25H9 recognition for navigation
- [x] Robotic arm for asteroid retrieval
- [x] Onboard storage compartment
- [x] Mars gravity (3.73 m/s²) compatible

---
**Team Cosmotron - University of Moratuwa**
*Project Cosmotron - Mars Rover Navigation Challenge*
