"""
G_MORACOS - Fully Autonomous Rover Controller
==============================================
Project Cosmotron Competition - Webots R2025a

Competition Requirements:
- Navigate using AprilTag flags (25H9 family)
  - ID=0: Arrival Marker (stop, proceed to asteroid)
  - ID=1: Turn Right Directive
  - ID=2: Turn Left Directive
- Detect Crimson Impact Site (red 1.5m x 1.5m stage)
- Locate white circle target zone
- Retrieve Argentis asteroid and store onboard

Obstacle Avoidance System:
- Smart obstacle detection with multiple thresholds
- Small obstacles (rocks): Drive over slowly if passable
- Large obstacles: 5-phase avoidance with path memory
  Phase 0: Backup to create clearance
  Phase 1: Turn away from obstacle
  Phase 2: Move forward parallel to path
  Phase 3: Turn back toward original heading
  Phase 4: Return to original path
- Compass-based heading recovery ensures rover returns
  to same direction after passing obstacles

State Machine:
- INIT: Initialize sensors, calibrate
- SEARCH_FLAG: Look for AprilTag navigation flags
- FOLLOW_FLAG: Navigate towards detected flag
- AVOID_OBSTACLE: Smart obstacle avoidance with path recovery
- APPROACH_RED_STAGE: Navigate to Crimson Impact Site
- SEARCH_ASTEROID: Search for asteroid in white circle
- APPROACH_ASTEROID: Move towards asteroid
- ALIGN_PICKUP: Fine alignment for pickup
- PICKUP: Execute arm pickup sequence
- STORE: Store asteroid in cargo area
- MISSION_COMPLETE: Park and signal completion

Controls (Manual Mode):
- M: Toggle Manual/Autonomous mode
- WASD: Movement
- Space: Stop
- H: Arm home position
- G: Arm grab position
- O/P: Open/Close gripper

Author: G_moracos Autonomous Controller
"""

from controller import Robot, Keyboard, Camera, GPS, Gyro, Compass, InertialUnit, DistanceSensor
import math
import struct

# ============================================================================
# CONFIGURATION
# ============================================================================

TIME_STEP = 32  # ms

# Wheel speeds - MUST match GMoraRover maxVelocity = 0.6
MAX_SPEED = 0.6       # Maximum motor velocity (rad/s)
CRUISE_SPEED = 0.5    # Normal forward speed
SLOW_SPEED = 0.25     # Slow approach speed
TURN_SPEED = 0.4      # Turning speed

# Obstacle detection thresholds (0-1000 scale from distance sensors)
OBSTACLE_CRITICAL = 300   # Very close - must stop immediately
OBSTACLE_THRESHOLD = 450   # Close obstacle - must avoid
OBSTACLE_WARNING = 600     # Approaching obstacle - slow down and prepare
OBSTACLE_FAR = 800         # Obstacle detected but far - can continue
OBSTACLE_PASSABLE = 350    # Obstacle low enough to drive over (rocks)

# Obstacle avoidance behavior settings
AVOID_BACKUP_STEPS = 30    # Steps to back up before turning
AVOID_TURN_STEPS = 50      # Steps for avoidance turn
AVOID_FORWARD_STEPS = 60   # Steps to move forward past obstacle
AVOID_RETURN_STEPS = 40    # Steps to return toward original path

# AprilTag IDs for Pathfinder Array (25H9 family)
APRILTAG_ARRIVAL = 0      # ID=0: Arrival Marker - Stop and proceed to asteroid
APRILTAG_TURN_RIGHT = 1   # ID=1: Turn Right Directive
APRILTAG_TURN_LEFT = 2    # ID=2: Turn Left Directive

# Color detection for environment
# Red stage (Crimson Impact Site) - iron oxide colors
RED_STAGE_COLOR = {
    'min': [0.5, 0.0, 0.0],
    'max': [1.0, 0.35, 0.35]
}

# White circle (asteroid target zone)
WHITE_CIRCLE_COLOR = {
    'min': [0.75, 0.75, 0.75],
    'max': [1.0, 1.0, 1.0]
}

# Green start area (Verdant Outpost)
GREEN_START_COLOR = {
    'min': [0.0, 0.5, 0.0],
    'max': [0.4, 1.0, 0.4]
}

# Asteroid (Argentis) - gray/metallic (fine-tuned for better detection)
ASTEROID_COLOR = {
    'min': [0.15, 0.15, 0.15],  # Lowered threshold for darker asteroids
    'max': [0.7, 0.7, 0.7]  # Increased upper bound for lighter metallic asteroids
}

# Arm positions for different actions [arm1, arm2, arm3, arm4, arm5]
# arm1 = base rotation (Z axis), arm2 = shoulder (Y axis), arm3 = elbow (Y axis), arm4 = wrist (Y axis), arm5 = wrist rotation
# LIMITS: arm1: -2.95 to 2.95, arm2: -2.35 to 1.39, arm3: -2.63 to 2.54, arm4/5: -3.14 to 3.14
# The arm structure: base->shoulder->elbow->wrist, gripper at end
# Arm segment lengths: ~0.16m each (arm2-3, arm3-4), gripper ~0.05m below arm5

# HOME: Arm folded up and back
ARM_HOME = [0.0, 0.0, 0.0, 0.0, 0.0]

# SEARCH: Arm slightly forward to see ground ahead
ARM_SEARCH = [0.0, -0.5, 0.5, 0.3, 0.0]

# PRE-GRAB: Arm extended forward, positioned above target
ARM_PRE_GRAB = [0.0, -1.5, 1.8, 1.0, 0.0]

# GRAB: Arm fully extended down to ground level
# Shoulder down (-2.3), elbow bent back (2.5), wrist angled down (2.0) to reach ground
ARM_GRAB = [0.0, -2.3, 2.5, 2.5, 0.0]

# GRAB_SECURE: Slight lift after grabbing to secure object
ARM_GRAB_SECURE = [0.0, -2.0, 2.2, 1.8, 0.0]

# LIFT: Object lifted to safe carry position
ARM_LIFT = [0.0, -0.5, 1.0, 0.5, 0.0]

# STORE positions - rotate arm 180° (pi radians) to face backward toward storage box
# Storage box is behind the rover, so arm1 needs to rotate ~3.0 radians (170°)
ARM_STORE_ROTATE = [2.9, -0.3, 0.8, 0.3, 0.0]  # Rotate toward back of rover
ARM_STORE_POSITION = [2.9, -1.0, 1.2, 0.8, 0.0]  # Position over storage box
ARM_STORE_DROP = [2.9, -1.5, 1.5, 1.0, 0.0]  # Lower into storage box
ARM_STORE_RELEASE = [2.9, -1.2, 1.3, 0.9, 0.0]  # Slight lift after release

# Arm joint limits
ARM_LIMITS = {
    'arm1': (-2.9496, 2.9496),
    'arm2': (-2.35619, 1.39626),
    'arm3': (-2.63545, 2.54818),
    'arm4': (-3.14159, 3.14159),
    'arm5': (-3.14159, 3.14159),
}


# ============================================================================
# STATE MACHINE STATES
# ============================================================================

class State:
    INIT = "INIT"
    INITIAL_SCAN = "INITIAL_SCAN"      # NEW: 360° scan to find first flag
    TURN_TO_FLAG = "TURN_TO_FLAG"      # NEW: Turn to face detected flag after scan
    EXIT_GREEN = "EXIT_GREEN"          # Step 1: Exit the green starting box
    MOVE_TO_FLAG = "MOVE_TO_FLAG"      # Step 2: Move forward until flag detected
    SEARCH_FLAG = "SEARCH_FLAG"        # Search/scan for flags
    SEARCH_RECOVER = "SEARCH_RECOVER"  # NEW: Recovery scan when flags lost
    FOLLOW_FLAG = "FOLLOW_FLAG"        # Navigate towards detected flag
    EXECUTE_TURN = "EXECUTE_TURN"      # Step 3: Execute turn based on flag ID
    AVOID_OBSTACLE = "AVOID_OBSTACLE"
    STOP_AT_RED = "STOP_AT_RED"        # Step 4: Stop at red box (ID=0 arrived)
    FIND_SAMPLE = "FIND_SAMPLE"        # Step 5: Find sample in white circle
    APPROACH_SAMPLE = "APPROACH_SAMPLE" # Approach the sample
    ALIGN_PICKUP = "ALIGN_PICKUP"      # Align for pickup
    PICKUP = "PICKUP"                  # Step 6: Pick up sample with arm
    STORE = "STORE"                    # Step 7: Store sample in rover box
    MISSION_COMPLETE = "MISSION_COMPLETE"
    MANUAL = "MANUAL"


# ============================================================================
# AUTONOMOUS ROVER CONTROLLER
# ============================================================================

class AutonomousRover:
    """Fully autonomous rover controller for Cosmotron competition."""
    
    def __init__(self):
        """Initialize the rover and all systems."""
        self.robot = Robot()
        self.timestep = TIME_STEP
        
        # State machine
        self.state = State.INIT
        self.prev_state = None
        self.state_timer = 0
        self.state_data = {}
        
        # Mission tracking
        self.flags_passed = 0
        self.total_flags_expected = 5  # Max 5 flags as per competition
        self.sample_collected = False
        self.mission_start_time = 0
        self.arrival_marker_detected = False  # ID=0 flag detected
        self.exited_green = False  # Has rover left green starting area
        
        # AprilTag tracking
        self.current_flag_id = None
        self.last_flag_id = None
        self.last_processed_flag_position = None  # Track position of last processed flag
        self.pending_turn_direction = None  # 'left', 'right', or None
        self.ignore_flags_until_distance = 0  # Ignore flags until we're this far from last one
        
        # Navigation
        self.search_direction = 1  # 1 = right, -1 = left
        self.turn_timer = 0
        self.turn_angle_accumulated = 0.0  # Gyro-integrated turn angle
        self.exit_timer = 0  # Timer for exiting green box
        
        # Initial scan and search recovery tracking
        self.initial_scan_complete = False  # Has initial 360° scan been done
        self.scan_start_heading = None      # Heading when scan started
        self.scan_angle_accumulated = 0.0   # Total angle scanned
        self.scan_flags_found = []          # Flags found during scan
        self.no_flag_timer = 0              # Timer since last flag seen
        self.search_move_timer = 0          # Timer for move-and-search pattern
        self.total_search_rotations = 0     # Count of full rotations during search
        
        # Solution 1+4: Initial scan and continuous search tracking
        self.scan_rotation_accumulated = 0.0  # Gyro-integrated rotation for 360° scan
        self.scan_complete = False            # Flag: has current scan completed
        self.scan_direction = 1               # 1 = right, -1 = left
        self.initial_heading = None           # Heading at start of scan
        self.scan_timer_in_move = 0           # Timer for no-flag timeout in MOVE_TO_FLAG
        self.recover_rotation = None          # Rotation tracking for SEARCH_RECOVER
        self.recover_direction = 1            # Direction for recovery scan
        
        # Turn-to-flag tracking (after 360° scan)
        self.turn_to_flag_angle = 0.0         # Angle needed to turn to face flag
        self.target_flag_position = None      # Position of target flag
        self.target_flag_heading = None       # Heading when flag was seen
        
        # Red box and sample tracking
        self.at_red_box = False
        self.sample_detected = False
        
        # Avoidance state - enhanced for path recovery
        self.avoid_direction = 1  # 1 = left, -1 = right
        self.avoid_timer = 0
        self.avoid_phase = 0      # 0=backup, 1=turn away, 2=forward, 3=turn back, 4=return
        
        # Path memory for obstacle avoidance recovery
        self.pre_avoid_heading = None      # Heading before avoidance started
        self.pre_avoid_position = None     # GPS position before avoidance
        self.pre_avoid_state = None        # State to return to after avoidance
        self.path_recovery_active = False  # True when trying to return to original path
        self.obstacle_count = 0            # Count obstacles encountered
        self.last_obstacle_time = 0        # Time of last obstacle encounter
        
        # Manual override
        self.manual_mode = False
        self.keyboard = Keyboard()
        self.keyboard.enable(TIME_STEP)
        
        # Initialize all devices
        self._init_sensors()
        self._init_wheels()
        self._init_arm()
        
        print("=" * 70)
        print("   G_MORACOS AUTONOMOUS ROVER - PROJECT COSMOTRON")
        print("=" * 70)
        print("Competition Mode: AprilTag Navigation (25H9)")
        print("  - ID=0: Arrival Marker (proceed to asteroid)")
        print("  - ID=1: Turn Right")
        print("  - ID=2: Turn Left")
        print("Press 'M' to toggle Manual/Autonomous mode")
        print("=" * 70)
    
    # ========================================================================
    # INITIALIZATION
    # ========================================================================
    
    def _init_sensors(self):
        """Initialize all sensors."""
        # Camera
        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(TIME_STEP)
            self.camera.recognitionEnable(TIME_STEP)
            print(f"[OK] Camera: {self.camera.getWidth()}x{self.camera.getHeight()}")
        else:
            print("[WARN] Camera not found")
        
        # GPS
        self.gps = self.robot.getDevice("gps")
        if self.gps:
            self.gps.enable(TIME_STEP)
            print("[OK] GPS enabled")
        
        # Gyro
        self.gyro = self.robot.getDevice("gyro")
        if self.gyro:
            self.gyro.enable(TIME_STEP)
            print("[OK] Gyro enabled")
        
        # IMU
        self.imu = self.robot.getDevice("imu")
        if self.imu:
            self.imu.enable(TIME_STEP)
            print("[OK] IMU enabled")
        
        # Compass
        self.compass = self.robot.getDevice("compass")
        if self.compass:
            self.compass.enable(TIME_STEP)
            print("[OK] Compass enabled")
        
        # Distance sensors
        self.distance_sensors = {}
        ds_names = ["ds_front_left", "ds_front", "ds_front_right", "ds_left", "ds_right"]
        for name in ds_names:
            sensor = self.robot.getDevice(name)
            if sensor:
                sensor.enable(TIME_STEP)
                self.distance_sensors[name] = sensor
                print(f"[OK] Distance sensor: {name}")
            else:
                print(f"[WARN] Distance sensor not found: {name}")
    
    def _init_wheels(self):
        """Initialize wheel motors for velocity control."""
        wheel_names = [
            "FrontLeftWheel", "FrontRightWheel",
            "MiddleLeftWheel", "MiddleRightWheel",
            "BackLeftWheel", "BackRightWheel"
        ]
        
        self.wheels = {}
        for name in wheel_names:
            motor = self.robot.getDevice(name)
            if motor:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                self.wheels[name] = motor
        
        self.left_wheels = ["FrontLeftWheel", "MiddleLeftWheel", "BackLeftWheel"]
        self.right_wheels = ["FrontRightWheel", "MiddleRightWheel", "BackRightWheel"]
        print(f"[OK] Wheels initialized: {len(self.wheels)} motors")
    
    def _init_arm(self):
        """Initialize arm motors and sensors."""
        self.arm_motors = {}
        self.arm_sensors = {}
        
        for i in range(1, 6):
            name = f"arm{i}"
            motor = self.robot.getDevice(name)
            if motor:
                motor.setVelocity(1.0)
                self.arm_motors[name] = motor
            
            sensor = self.robot.getDevice(f"{name}sensor")
            if sensor:
                sensor.enable(TIME_STEP)
                self.arm_sensors[name] = sensor
        
        # Gripper
        self.gripper_left = self.robot.getDevice("finger::left")
        self.gripper_right = self.robot.getDevice("finger::right")
        if self.gripper_left:
            self.gripper_left.setVelocity(0.1)
        if self.gripper_right:
            self.gripper_right.setVelocity(0.1)
        
        self.gripper_left_sensor = self.robot.getDevice("finger::leftsensor")
        self.gripper_right_sensor = self.robot.getDevice("finger::rightsensor")
        if self.gripper_left_sensor:
            self.gripper_left_sensor.enable(TIME_STEP)
        if self.gripper_right_sensor:
            self.gripper_right_sensor.enable(TIME_STEP)
        
        print(f"[OK] Arm initialized: {len(self.arm_motors)} joints + gripper")
    
    # ========================================================================
    # WHEEL CONTROL
    # ========================================================================
    
    def set_wheel_speeds(self, left, right):
        """Set left and right wheel velocities.
        
        Positive velocity moves rover forward (toward camera side).
        """
        left = max(-MAX_SPEED, min(MAX_SPEED, left))
        right = max(-MAX_SPEED, min(MAX_SPEED, right))
        
        # Set velocities directly - positive = forward
        for name in self.left_wheels:
            if name in self.wheels:
                self.wheels[name].setVelocity(left)
        for name in self.right_wheels:
            if name in self.wheels:
                self.wheels[name].setVelocity(right)
    
    def stop(self):
        """Stop all wheels."""
        self.set_wheel_speeds(0, 0)
    
    def move_forward(self, speed=CRUISE_SPEED):
        """Move forward (toward the front/camera side, -X direction)."""
        self.set_wheel_speeds(speed, speed)
    
    def move_backward(self, speed=CRUISE_SPEED):
        """Move backward (away from camera, +X direction)."""
        self.set_wheel_speeds(-speed, -speed)
    
    def turn_left(self, speed=TURN_SPEED):
        """Turn left in place."""
        self.set_wheel_speeds(speed, -speed)
    
    def turn_right(self, speed=TURN_SPEED):
        """Turn right in place."""
        self.set_wheel_speeds(-speed, speed)
    
    def curve_left(self, speed=CRUISE_SPEED, ratio=0.5):
        """Curve to the left while moving forward."""
        self.set_wheel_speeds(speed, speed * ratio)
    
    def curve_right(self, speed=CRUISE_SPEED, ratio=0.5):
        """Curve to the right while moving forward."""
        self.set_wheel_speeds(speed * ratio, speed)
    
    # ========================================================================
    # ARM CONTROL
    # ========================================================================
    
    def set_arm_position(self, positions, speed=1.0):
        """Set arm to specific joint positions."""
        joint_names = ["arm1", "arm2", "arm3", "arm4", "arm5"]
        for i, name in enumerate(joint_names):
            if name in self.arm_motors:
                motor = self.arm_motors[name]
                motor.setVelocity(speed)
                limits = ARM_LIMITS.get(name, (-3.14, 3.14))
                pos = max(limits[0], min(limits[1], positions[i]))
                motor.setPosition(pos)
    
    def get_arm_position(self):
        """Get current arm joint positions."""
        positions = []
        for i in range(1, 6):
            sensor = self.arm_sensors.get(f"arm{i}")
            if sensor:
                positions.append(sensor.getValue())
            else:
                positions.append(0.0)
        return positions
    
    def arm_at_position(self, target, tolerance=0.15):
        """Check if arm has reached target position."""
        current = self.get_arm_position()
        for i in range(5):
            if abs(current[i] - target[i]) > tolerance:
                return False
        return True
    
    def open_gripper(self):
        """Open gripper."""
        if self.gripper_left:
            self.gripper_left.setPosition(0.025)
        if self.gripper_right:
            self.gripper_right.setPosition(0.025)
    
    def close_gripper(self):
        """Close gripper."""
        if self.gripper_left:
            self.gripper_left.setPosition(0.0)
        if self.gripper_right:
            self.gripper_right.setPosition(0.0)
    
    # ========================================================================
    # SENSOR READING
    # ========================================================================
    
    def get_distance_readings(self):
        """Get all distance sensor readings."""
        readings = {}
        for name, sensor in self.distance_sensors.items():
            readings[name] = sensor.getValue()
        return readings
    
    def check_obstacle(self):
        """Check for obstacles and return detailed obstacle info.
        
        Returns comprehensive obstacle analysis including:
        - Detection at different distances (critical, close, warning)
        - Directional information (front, left, right)
        - Whether obstacle might be passable (low enough to drive over)
        - Best avoidance direction recommendation
        """
        readings = self.get_distance_readings()
        
        front = readings.get("ds_front", 1000)
        front_left = readings.get("ds_front_left", 1000)
        front_right = readings.get("ds_front_right", 1000)
        left = readings.get("ds_left", 1000)
        right = readings.get("ds_right", 1000)
        
        obstacle = {
            # Critical - must stop immediately
            'critical': front < OBSTACLE_CRITICAL,
            
            # Close obstacles requiring avoidance
            'front': front < OBSTACLE_THRESHOLD,
            'front_warning': front < OBSTACLE_WARNING,
            'left': front_left < OBSTACLE_THRESHOLD or left < OBSTACLE_THRESHOLD,
            'right': front_right < OBSTACLE_THRESHOLD or right < OBSTACLE_THRESHOLD,
            
            # Raw distance values
            'front_left_dist': front_left,
            'front_right_dist': front_right,
            'front_dist': front,
            'left_dist': left,
            'right_dist': right,
            
            # Passable check - obstacle might be low enough to drive over
            # If front is blocked but sides are more clear, try to pass over
            'maybe_passable': (front < OBSTACLE_THRESHOLD and 
                              front > OBSTACLE_PASSABLE and
                              front_left > OBSTACLE_WARNING and 
                              front_right > OBSTACLE_WARNING),
        }
        
        # Combined obstacle presence checks
        obstacle['any'] = obstacle['front'] or obstacle['left'] or obstacle['right']
        obstacle['clear'] = not obstacle['any'] and front > OBSTACLE_FAR
        
        # Determine best avoidance direction (prefer the more open side)
        if front_left > front_right and left > right:
            obstacle['best_avoid_direction'] = 1   # Left is more open
        elif front_right > front_left and right > left:
            obstacle['best_avoid_direction'] = -1  # Right is more open
        else:
            # Default based on which front-side is clearer
            obstacle['best_avoid_direction'] = 1 if front_left > front_right else -1
        
        # Calculate urgency score (0-100, higher = more urgent)
        min_dist = min(front, front_left, front_right)
        if min_dist < OBSTACLE_CRITICAL:
            obstacle['urgency'] = 100
        elif min_dist < OBSTACLE_THRESHOLD:
            obstacle['urgency'] = 70
        elif min_dist < OBSTACLE_WARNING:
            obstacle['urgency'] = 40
        else:
            obstacle['urgency'] = 0
        
        return obstacle
    
    def should_avoid_obstacle(self):
        """Determine if obstacle avoidance should be triggered.
        
        Returns:
            tuple: (should_avoid: bool, can_pass_over: bool, obstacle_info: dict)
        """
        obstacle = self.check_obstacle()
        
        # Critical obstacle - must avoid immediately
        if obstacle['critical']:
            return True, False, obstacle
        
        # Front obstacle detected
        if obstacle['front']:
            # Check if we can drive over it (like a small rock)
            if obstacle['maybe_passable']:
                return False, True, obstacle  # Can try to pass over
            return True, False, obstacle
        
        # Warning level but not blocking - can continue
        if obstacle['front_warning'] and not obstacle['front']:
            return False, False, obstacle
        
        # Side obstacles only - can usually continue with slight adjustment
        if obstacle['left'] or obstacle['right']:
            # If only side obstacles, we can curve away slightly
            return False, False, obstacle
        
        return False, False, obstacle
    
    def save_path_state(self):
        """Save current heading and position before obstacle avoidance."""
        self.pre_avoid_heading = self.get_heading()
        self.pre_avoid_position = self.get_position()
        self.pre_avoid_state = self.state
        self.path_recovery_active = True
        
        print(f"[PATH SAVED] Heading: {self.pre_avoid_heading:.1f}°, Position: ({self.pre_avoid_position[0]:.2f}, {self.pre_avoid_position[1]:.2f})")
    
    def get_heading_difference(self, target_heading):
        """Calculate difference between current heading and target.
        
        Returns signed angle difference (-180 to 180 degrees).
        Positive = need to turn right, Negative = need to turn left.
        """
        current = self.get_heading()
        diff = target_heading - current
        
        # Normalize to -180 to 180
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        
        return diff
    
    def is_back_on_path(self):
        """Check if rover has returned to approximately the original path."""
        if self.pre_avoid_heading is None:
            return True
        
        # Check heading alignment (within 15 degrees)
        heading_diff = abs(self.get_heading_difference(self.pre_avoid_heading))
        heading_aligned = heading_diff < 15
        
        return heading_aligned
    
    def get_heading(self):
        """Get current heading from compass (0-360 degrees)."""
        if self.compass:
            values = self.compass.getValues()
            heading = math.atan2(values[0], values[2])
            heading = math.degrees(heading)
            if heading < 0:
                heading += 360
            return heading
        return 0
    
    def get_position(self):
        """Get current GPS position."""
        if self.gps:
            return self.gps.getValues()
        return [0, 0, 0]
    
    # ========================================================================
    # VISION PROCESSING - APRILTAG AND COLOR DETECTION
    # ========================================================================
    
    def detect_apriltag_flags(self):
        """Detect AprilTag flags from camera recognition.
        
        AprilTag IDs (25H9 family):
        - ID=0: Arrival Marker (proceed to asteroid)
        - ID=1: Turn Right Directive
        - ID=2: Turn Left Directive
        
        Returns list of detected flags with their IDs, positions, and angles.
        STRICT detection - only matches objects with explicit apriltag in model name.
        """
        if not self.camera:
            return []
        
        objects = self.camera.getRecognitionObjects()
        flags = []
        
        cam_width = self.camera.getWidth()
        cam_height = self.camera.getHeight()
        
        for obj in objects:
            pos = obj.getPosition()
            img_pos = obj.getPositionOnImage()
            size = obj.getSizeOnImage()
            model = obj.getModel() if hasattr(obj, 'getModel') else ''
            model_lower = model.lower() if model else ''
            
            # Normalize image position (-1 to 1, where 0 is center)
            norm_x = (img_pos[0] - cam_width/2) / (cam_width/2)
            
            # STRICT detection - ONLY match specific model names:
            # "apriltag_id0", "apriltag_id1", "apriltag_id2"
            # OR "apriltag_flag_0", "apriltag_flag_1", "apriltag_flag_2"
            apriltag_id = None
            
            # Pattern 1: apriltag_id0, apriltag_id1, apriltag_id2
            if 'apriltag_id0' in model_lower or 'apriltag_flag_0' in model_lower:
                apriltag_id = 0
            elif 'apriltag_id1' in model_lower or 'apriltag_flag_1' in model_lower:
                apriltag_id = 1
            elif 'apriltag_id2' in model_lower or 'apriltag_flag_2' in model_lower:
                apriltag_id = 2
            
            # Only add if we found a valid AprilTag ID
            if apriltag_id is not None:
                # Get current rover heading when flag is seen
                current_heading = self.get_heading() if hasattr(self, 'get_heading') else 0.0
                
                flag_info = {
                    'apriltag_id': apriltag_id,
                    'position': pos,
                    'distance': math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2),
                    'angle': norm_x,
                    'size': size,
                    'model': model,
                    'raw_id': obj.getId(),
                    'heading_when_seen': current_heading  # Store heading for later turn-to-flag
                }
                flags.append(flag_info)
                print(f"[FLAG DETECTED] ID={apriltag_id}, model='{model}', dist={flag_info['distance']:.2f}m")
        
        return flags
    
    def detect_environment_colors(self):
        """Detect environmental features by color.
        
        Returns dict with:
        - red_stage: Crimson Impact Site detection
        - white_circle: Target zone detection
        - green_area: Verdant Outpost detection
        - asteroid: Argentis asteroid detection
        """
        if not self.camera:
            return {'red_stage': None, 'white_circle': None, 'green_area': None, 'asteroid': None}
        
        objects = self.camera.getRecognitionObjects()
        result = {
            'red_stage': None,
            'white_circle': None, 
            'green_area': None,
            'asteroid': None
        }
        
        cam_width = self.camera.getWidth()
        cam_height = self.camera.getHeight()
        
        for obj in objects:
            # getColors() returns ctypes array - use getNumberOfColors() for length
            num_colors = obj.getNumberOfColors()
            if num_colors < 3:
                continue
            
            colors = obj.getColors()
            r, g, b = colors[0], colors[1], colors[2]
                
            pos = obj.getPosition()
            img_pos = obj.getPositionOnImage()
            size = obj.getSizeOnImage()
            norm_x = (img_pos[0] - cam_width/2) / (cam_width/2)
            
            obj_info = {
                'position': pos,
                'distance': math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2),
                'angle': norm_x,
                'size': size,
                'colors': [r, g, b]
            }
            
            # Check for red stage (Crimson Impact Site)
            if (RED_STAGE_COLOR['min'][0] <= r <= RED_STAGE_COLOR['max'][0] and
                RED_STAGE_COLOR['min'][1] <= g <= RED_STAGE_COLOR['max'][1] and
                RED_STAGE_COLOR['min'][2] <= b <= RED_STAGE_COLOR['max'][2]):
                if result['red_stage'] is None or obj_info['size'][0] * obj_info['size'][1] > \
                   result['red_stage']['size'][0] * result['red_stage']['size'][1]:
                    result['red_stage'] = obj_info
            
            # Check for white circle (target zone)
            if (WHITE_CIRCLE_COLOR['min'][0] <= r <= WHITE_CIRCLE_COLOR['max'][0] and
                WHITE_CIRCLE_COLOR['min'][1] <= g <= WHITE_CIRCLE_COLOR['max'][1] and
                WHITE_CIRCLE_COLOR['min'][2] <= b <= WHITE_CIRCLE_COLOR['max'][2]):
                if result['white_circle'] is None or obj_info['distance'] < result['white_circle']['distance']:
                    result['white_circle'] = obj_info
            
            # Check for green area (start zone)
            if (GREEN_START_COLOR['min'][0] <= r <= GREEN_START_COLOR['max'][0] and
                GREEN_START_COLOR['min'][1] <= g <= GREEN_START_COLOR['max'][1] and
                GREEN_START_COLOR['min'][2] <= b <= GREEN_START_COLOR['max'][2]):
                if result['green_area'] is None or obj_info['size'][0] * obj_info['size'][1] > \
                   result['green_area']['size'][0] * result['green_area']['size'][1]:
                    result['green_area'] = obj_info
            
            # Check for asteroid (gray/metallic) - improved detection
            in_range = (ASTEROID_COLOR['min'][0] <= r <= ASTEROID_COLOR['max'][0] and
                       ASTEROID_COLOR['min'][1] <= g <= ASTEROID_COLOR['max'][1] and
                       ASTEROID_COLOR['min'][2] <= b <= ASTEROID_COLOR['max'][2])
            
            if in_range:
                # Verify grayscale (all channels similar) - more lenient for metallic objects
                color_variance = max(abs(r - g), abs(g - b), abs(r - b))
                if color_variance < 0.25:  # Increased from 0.2 for better detection
                    # Additional check: not too bright (white) or too dark (black)
                    avg_brightness = (r + g + b) / 3.0
                    if 0.15 <= avg_brightness <= 0.75:  # Reasonable brightness range
                        if result['asteroid'] is None or obj_info['distance'] < result['asteroid']['distance']:
                            result['asteroid'] = obj_info
        
        return result
    
    def get_nearest_apriltag_flag(self):
        """Get the nearest AprilTag flag."""
        flags = self.detect_apriltag_flags()
        if flags:
            # Sort by distance
            flags = sorted(flags, key=lambda f: f['distance'])
            return flags[0]
        return None
    
    def get_asteroid(self):
        """Get detected asteroid info."""
        env = self.detect_environment_colors()
        return env['asteroid']
    
    def get_red_stage(self):
        """Get detected red stage (Crimson Impact Site)."""
        env = self.detect_environment_colors()
        return env['red_stage']
    
    def get_white_circle(self):
        """Get detected white circle (target zone)."""
        env = self.detect_environment_colors()
        return env['white_circle']
    
    # ========================================================================
    # STATE MACHINE BEHAVIORS
    # ========================================================================
    
    def state_init(self):
        """Initialization state - prepare rover for mission."""
        self.stop()
        self.set_arm_position(ARM_HOME)
        self.open_gripper()
        
        self.state_timer += 1
        if self.state_timer > 30:  # Wait ~1 second
            self.mission_start_time = self.robot.getTime()
            print("\n" + "=" * 60)
            print("   MISSION START - COSMOTRON COMPETITION")
            print("=" * 60)
            print("NEW: Starting with 360° SCAN to locate first flag...")
            print("Flag IDs: 0=Arrival(Stop), 1=Turn Right, 2=Turn Left")
            print("=" * 60)
            self.state_timer = 0
            self.scan_start_heading = self.get_heading()
            self.scan_angle_accumulated = 0.0
            self.scan_flags_found = []
            return State.INITIAL_SCAN
        return State.INIT
    
    def state_initial_scan(self):
        """NEW: 360° scan at start to find first flag regardless of initial orientation.
        
        This ensures the rover can find flags from ANY starting position/orientation.
        The rover rotates in place, detecting all visible flags, then moves toward
        the nearest one.
        """
        self.state_timer += 1
        
        # Rotate slowly and scan for flags
        self.turn_right(TURN_SPEED * 0.6)
        
        # Track rotation using gyro
        if self.gyro:
            gyro_values = self.gyro.getValues()
            angular_velocity_z = abs(gyro_values[2])
            dt = TIME_STEP / 1000.0
            self.scan_angle_accumulated += angular_velocity_z * dt
        else:
            # Fallback: estimate rotation
            self.scan_angle_accumulated += 0.015
        
        # Check for flags during rotation
        flags = self.detect_apriltag_flags()
        for flag in flags:
            # Check if we already recorded this flag (by ID and approximate distance)
            is_new = True
            for recorded in self.scan_flags_found:
                # Compare by AprilTag ID and similar distance (simpler, no overflow risk)
                if flag['apriltag_id'] == recorded['apriltag_id']:
                    dist_diff = abs(flag['distance'] - recorded['distance'])
                    if dist_diff < 2.0:  # Within 2 meters = same flag
                        is_new = False
                        break
            
            if is_new:
                self.scan_flags_found.append(flag)
                print(f"[SCAN] Found flag ID={flag['apriltag_id']} at {flag['distance']:.2f}m")
        
        # Progress update
        degrees_scanned = math.degrees(self.scan_angle_accumulated)
        if self.state_timer % 30 == 0:
            print(f"[SCANNING] {degrees_scanned:.0f}°/360° - Found {len(self.scan_flags_found)} flags")
        
        # Check if we've completed ~360° rotation
        if self.scan_angle_accumulated >= 2 * math.pi:
            self.stop()
            self.initial_scan_complete = True
            
            print("\n" + "=" * 50)
            print(f"[SCAN COMPLETE] 360° scan finished!")
            print(f"  Flags detected: {len(self.scan_flags_found)}")
            
            if self.scan_flags_found:
                # Sort by distance and target the nearest flag
                self.scan_flags_found.sort(key=lambda f: f['distance'])
                nearest = self.scan_flags_found[0]
                print(f"  Nearest flag: ID={nearest['apriltag_id']} at {nearest['distance']:.2f}m")
                
                # Store target flag info for turning toward it
                self.current_flag_id = nearest['apriltag_id']
                self.state_data['target'] = nearest
                self.target_flag_heading = nearest.get('heading_when_seen', None)
                
                # Use the HEADING when the flag was seen to turn back to that direction
                # This is more reliable than trying to calculate world position from camera coords
                if self.target_flag_heading is not None:
                    current_heading = self.get_heading()
                    # We need to turn back to the heading we had when we saw the flag
                    heading_diff = self.target_flag_heading - current_heading
                    # Normalize to [-pi, pi]
                    while heading_diff > math.pi: heading_diff -= 2 * math.pi
                    while heading_diff < -math.pi: heading_diff += 2 * math.pi
                    self.turn_to_flag_angle = heading_diff
                    print(f"  Need to turn {math.degrees(heading_diff):.1f}° to face flag")
                    print("  Turning toward flag...")
                    print("=" * 50 + "\n")
                    self.state_timer = 0
                    self.exited_green = True
                    return State.TURN_TO_FLAG  # New state to turn toward flag
                else:
                    print("  Moving toward nearest flag...")
                    print("=" * 50 + "\n")
                    self.state_timer = 0
                    self.no_flag_timer = 0
                    self.exited_green = True
                    return State.FOLLOW_FLAG
            else:
                # No flags found - exit green and search while moving
                print("  No flags found - will search while moving forward")
                print("=" * 50 + "\n")
                self.state_timer = 0
                self.exit_timer = 0
                return State.EXIT_GREEN
        
        return State.INITIAL_SCAN
    
    def state_turn_to_flag(self):
        """Turn to face the detected flag after 360° scan.
        
        Uses gyro integration to accurately turn the required angle.
        """
        self.state_timer += 1
        
        # Track rotation using gyro
        if self.gyro:
            gyro_values = self.gyro.getValues()
            angular_velocity = gyro_values[1]  # Y-axis rotation (yaw)
            dt = self.timestep / 1000.0
            
            # Track how much we've turned
            if not hasattr(self, 'turn_accumulated'):
                self.turn_accumulated = 0.0
            self.turn_accumulated += angular_velocity * dt
        
        # Calculate remaining angle to turn
        remaining = self.turn_to_flag_angle - getattr(self, 'turn_accumulated', 0.0)
        
        # Check if we're close enough (within 5 degrees)
        if abs(remaining) < math.radians(5):
            self.stop()
            print(f"[TURN TO FLAG] Complete! Now facing flag.")
            self.turn_accumulated = 0.0  # Reset for next time
            self.state_timer = 0
            self.no_flag_timer = 0
            return State.FOLLOW_FLAG
        
        # Turn in the appropriate direction
        if remaining > 0:
            self.turn_right(TURN_SPEED * 0.5)
        else:
            self.turn_left(TURN_SPEED * 0.5)
        
        # Progress feedback
        if self.state_timer % 30 == 0:
            remaining_deg = math.degrees(remaining)
            print(f"[TURN TO FLAG] Turning... {remaining_deg:.1f}° remaining")
        
        # Safety timeout (shouldn't need more than ~5 seconds to turn)
        if self.state_timer > 300:
            print("[TURN TO FLAG] Timeout - proceeding to follow")
            self.turn_accumulated = 0.0
            return State.FOLLOW_FLAG
        
        return State.TURN_TO_FLAG

    def state_exit_green(self):
        """Step 1: Exit the green starting box by moving STRAIGHT forward."""
        # Increment exit timer
        self.exit_timer += 1
        
        # Move STRAIGHT forward - NO obstacle avoidance during exit
        # This ensures we definitely exit the green box
        if self.exit_timer < 150:  # ~4.8 seconds of straight movement
            self.move_forward(CRUISE_SPEED)
            
            if self.exit_timer % 30 == 1:
                print(f"[EXIT GREEN] Moving STRAIGHT forward... ({self.exit_timer}/150)")
            
            return State.EXIT_GREEN
        else:
            # Exited green box, now do 360° scan to find flags
            self.exited_green = True
            print("\n" + "=" * 40)
            print("[EXIT GREEN] Left green starting area!")
            print("[STEP 1.5] Starting 360° scan to find flags...")
            print("=" * 40)
            self.state_timer = 0
            # Reset scan tracking for initial scan
            self.scan_angle_accumulated = 0.0
            self.scan_start_heading = self.get_heading()
            self.scan_flags_found = []
            return State.INITIAL_SCAN
    
    def state_move_to_flag(self):
        """Step 2: Move STRAIGHT forward until a flag is detected.
        
        Includes smart obstacle avoidance that:
        - Detects obstacles ahead
        - Tries to pass over small obstacles (rocks)
        - Avoids larger obstacles while maintaining path memory
        - Returns to original heading after passing obstacles
        """
        # INCREMENT TIMER FIRST
        self.state_timer += 1
        
        # ================================================================
        # OBSTACLE CHECK - Smart detection and avoidance
        # ================================================================
        should_avoid, can_pass_over, obstacle = self.should_avoid_obstacle()
        
        if should_avoid:
            # Must avoid this obstacle
            self.save_path_state()  # Remember current heading
            print(f"\n[OBSTACLE DETECTED] Cannot pass - initiating avoidance")
            print(f"  Front distance: {obstacle['front_dist']:.0f}")
            self.avoid_timer = 0
            self.avoid_phase = 0
            return State.AVOID_OBSTACLE
        
        if can_pass_over:
            # Small obstacle like a rock - try to drive over slowly
            if self.state_timer % 50 == 1:
                print(f"[PASSABLE] Small obstacle detected - driving over slowly")
            self.move_forward(SLOW_SPEED)  # Slow down to pass over
            return State.MOVE_TO_FLAG
        
        # Minor obstacle on sides - curve away slightly while continuing
        if obstacle['left'] and not obstacle['right']:
            self.curve_right(CRUISE_SPEED, 0.85)  # Gentle curve away
        elif obstacle['right'] and not obstacle['left']:
            self.curve_left(CRUISE_SPEED, 0.85)  # Gentle curve away
        elif obstacle['front_warning'] and not obstacle['front']:
            # Approaching obstacle - slow down slightly
            self.move_forward(CRUISE_SPEED * 0.8)
        else:
            # Clear path - full speed ahead
            self.move_forward(CRUISE_SPEED)
        
        # Periodic status
        if self.state_timer % 100 == 0:
            objects = self.camera.getRecognitionObjects() if self.camera else []
            obs_status = "CLEAR" if obstacle['clear'] else f"DIST={obstacle['front_dist']:.0f}"
            print(f"[MOVING] Timer={self.state_timer}, Objects={len(objects)}, Obstacle={obs_status}")
        
        # ================================================================
        # APRILTAG FLAG DETECTION
        # ================================================================
        flag = self.get_nearest_apriltag_flag()
        
        if flag:
            tag_id = flag.get('apriltag_id')
            distance = flag.get('distance', 0)
            flag_pos = flag.get('position', [0, 0, 0])
            
            # Check if this is the same flag we already processed
            if self.last_processed_flag_position is not None:
                last_pos = self.last_processed_flag_position
                pos_diff = math.sqrt(
                    (flag_pos[0] - last_pos[0])**2 + 
                    (flag_pos[1] - last_pos[1])**2 + 
                    (flag_pos[2] - last_pos[2])**2
                )
                
                if pos_diff < 2.0:  # Same flag - skip it
                    if distance > 2.0:
                        # Far enough away - clear the tracking
                        self.last_processed_flag_position = None
                        print("[CLEARED] Past processed flag, ready for new flags")
                    # Keep moving forward
                    return State.MOVE_TO_FLAG
            
            print(f"\n[FLAG FOUND] ID={tag_id}, distance={distance:.2f}m")
            self.current_flag_id = tag_id
            self.state_data['target'] = flag
            self.state_timer = 0
            self.no_flag_timer = 0  # Reset no-flag timer
            return State.FOLLOW_FLAG
        
        # ================================================================
        # NO FLAG VISIBLE - Track time and trigger recovery search
        # ================================================================
        self.no_flag_timer += 1
        
        # If no flag seen for a while, do a recovery scan
        if self.no_flag_timer > 200:  # ~6.4 seconds without seeing a flag
            print(f"\n[NO FLAGS] No flag visible for {self.no_flag_timer} steps")
            print("[RECOVERY] Starting recovery scan...")
            self.state_timer = 0
            self.scan_angle_accumulated = 0.0
            self.scan_flags_found = []
            return State.SEARCH_RECOVER
        
        return State.MOVE_TO_FLAG

    def state_search_recover(self):
        """Recovery scan when flags are lost during navigation.
        
        Performs a 360° scan to re-acquire flags, then continues toward nearest.
        If no flags found after full rotation, moves forward and tries again.
        """
        self.state_timer += 1
        
        # Rotate and scan for flags
        self.turn_right(TURN_SPEED * 0.5)
        
        # Track rotation using gyro
        if self.gyro:
            gyro_values = self.gyro.getValues()
            angular_velocity_z = abs(gyro_values[2])
            dt = TIME_STEP / 1000.0
            self.scan_angle_accumulated += angular_velocity_z * dt
        else:
            self.scan_angle_accumulated += 0.012
        
        # Check for flags during rotation
        flags = self.detect_apriltag_flags()
        for flag in flags:
            # Check if already recorded (by ID and distance - safer than position)
            is_new = True
            for recorded in self.scan_flags_found:
                if flag['apriltag_id'] == recorded['apriltag_id']:
                    dist_diff = abs(flag['distance'] - recorded['distance'])
                    if dist_diff < 2.0:  # Same flag
                        is_new = False
                        break
            
            if is_new:
                self.scan_flags_found.append(flag)
                print(f"[RECOVER] Found flag ID={flag['apriltag_id']} at {flag['distance']:.2f}m")
        
        # Progress update
        degrees_scanned = math.degrees(self.scan_angle_accumulated)
        if self.state_timer % 40 == 0:
            print(f"[RECOVER SCAN] {degrees_scanned:.0f}°/360° - Found {len(self.scan_flags_found)} flags")
        
        # If we find a flag, we can stop scanning early and go to it
        if len(self.scan_flags_found) > 0 and self.scan_angle_accumulated > math.pi / 2:
            # Found at least one flag after 90° - go to nearest
            self.scan_flags_found.sort(key=lambda f: f['distance'])
            nearest = self.scan_flags_found[0]
            
            print(f"[RECOVER] Early exit - heading to flag ID={nearest['apriltag_id']}")
            self.current_flag_id = nearest['apriltag_id']
            self.state_data['target'] = nearest
            self.state_timer = 0
            self.no_flag_timer = 0
            return State.FOLLOW_FLAG
        
        # Check if we've completed 360° rotation
        if self.scan_angle_accumulated >= 2 * math.pi:
            self.stop()
            self.total_search_rotations += 1
            
            if self.scan_flags_found:
                # Found flags - go to nearest
                self.scan_flags_found.sort(key=lambda f: f['distance'])
                nearest = self.scan_flags_found[0]
                
                print(f"[RECOVER COMPLETE] Found {len(self.scan_flags_found)} flags")
                print(f"  Heading to flag ID={nearest['apriltag_id']} at {nearest['distance']:.2f}m")
                
                self.current_flag_id = nearest['apriltag_id']
                self.state_data['target'] = nearest
                self.state_timer = 0
                self.no_flag_timer = 0
                return State.FOLLOW_FLAG
            else:
                # No flags found - move forward and try again
                self.search_move_timer += 1
                
                if self.total_search_rotations < 5:
                    print(f"[RECOVER] No flags found - moving forward and re-scanning...")
                    print(f"  Total rotations: {self.total_search_rotations}")
                    
                    # Move forward for a bit, then scan again
                    self.state_timer = 0
                    self.scan_angle_accumulated = 0.0
                    self.no_flag_timer = 0
                    
                    # Temporarily move forward before next scan
                    return State.MOVE_TO_FLAG
                else:
                    # Too many rotations without finding flags - something is wrong
                    print("[WARN] Many rotations without finding flags - continuing forward")
                    self.total_search_rotations = 0
                    self.state_timer = 0
                    self.no_flag_timer = 0
                    return State.MOVE_TO_FLAG
        
        return State.SEARCH_RECOVER

    def state_search_flag(self):
        """Search for AprilTag flags - rotate and search."""
        self.state_timer += 1
        
        # Rotate slowly while searching
        if self.search_direction > 0:
            self.turn_right(TURN_SPEED * 0.4)
        else:
            self.turn_left(TURN_SPEED * 0.4)
        
        # Check for AprilTag flags
        flag = self.get_nearest_apriltag_flag()
        
        if flag:
            tag_id = flag.get('apriltag_id')
            print(f"[DETECT] Flag found! ID={tag_id}, distance={flag['distance']:.2f}m")
            self.current_flag_id = tag_id
            self.state_data['target'] = flag
            self.state_timer = 0
            self.no_flag_timer = 0
            return State.FOLLOW_FLAG
        
        # Alternate search direction periodically
        if self.state_timer % 150 == 0:
            self.search_direction *= -1
            print(f"[SEARCH] Switching direction, timer={self.state_timer}")
        
        # If searching too long, move forward a bit
        if self.state_timer > 300:
            print("[SEARCH] Timeout - moving forward and resuming search")
            self.state_timer = 0
            return State.MOVE_TO_FLAG
        
        return State.SEARCH_FLAG
    
    def state_follow_flag(self):
        """Navigate towards detected AprilTag flag and execute its command.
        
        Includes obstacle avoidance while approaching flags.
        """
        # ================================================================
        # OBSTACLE CHECK - Even while following a flag
        # ================================================================
        should_avoid, can_pass_over, obstacle = self.should_avoid_obstacle()
        
        if should_avoid:
            # Must avoid obstacle - save path and avoid
            self.save_path_state()
            print(f"[OBSTACLE] Detected while following flag - avoiding")
            self.avoid_timer = 0
            self.avoid_phase = 0
            return State.AVOID_OBSTACLE
        
        flag = self.get_nearest_apriltag_flag()
        
        if not flag:
            # Lost the flag - KEEP GOING STRAIGHT
            self.state_timer += 1
            if self.state_timer % 30 == 1:
                print("[FOLLOW] No flag visible, going straight...")
            
            # Check obstacles while moving forward
            if can_pass_over:
                self.move_forward(SLOW_SPEED)
            else:
                self.move_forward(CRUISE_SPEED)
            
            if self.state_timer > 100:  # After ~3 seconds
                self.state_timer = 0
                return State.MOVE_TO_FLAG
            return State.FOLLOW_FLAG
        
        self.state_timer = 0
        distance = flag['distance']
        angle = flag['angle']
        tag_id = flag.get('apriltag_id')
        flag_pos = flag.get('position', [0, 0, 0])
        
        # ====================================================================
        # CHECK IF THIS IS THE SAME FLAG WE JUST PROCESSED
        # ====================================================================
        if self.last_processed_flag_position is not None:
            last_pos = self.last_processed_flag_position
            # Calculate distance between current flag and last processed flag
            pos_diff = math.sqrt(
                (flag_pos[0] - last_pos[0])**2 + 
                (flag_pos[1] - last_pos[1])**2 + 
                (flag_pos[2] - last_pos[2])**2
            )
            
            # If this flag is very close to the last processed one, skip it
            if pos_diff < 2.0:  # Within 2 meters = same flag
                if distance > 1.5:
                    # We're far enough from processed flag, clear tracking
                    print(f"[SKIP] Cleared past flag, looking for next one...")
                    self.last_processed_flag_position = None
                else:
                    # Still near the same flag - keep moving forward to get away
                    self.move_forward(CRUISE_SPEED)
                    return State.FOLLOW_FLAG
        
        # Debug output every few frames
        if self.robot.getTime() % 0.5 < 0.05:
            print(f"[FOLLOW] Flag ID={tag_id}, dist={distance:.2f}m, angle={angle:.2f}")
        
        # ====================================================================
        # FLAG REACHED - Close enough to execute command (0.6m threshold)
        # ====================================================================
        if distance < 0.6:
            self.flags_passed += 1
            self.last_flag_id = tag_id
            self.last_processed_flag_position = flag_pos  # Track this flag's position
            self.stop()
            
            print("\n" + "=" * 50)
            print(f"   FLAG #{self.flags_passed} REACHED - AprilTag ID={tag_id}")
            print("=" * 50)
            
            # Step 3: Process AprilTag ID directive
            if tag_id == APRILTAG_ARRIVAL:  # ID=0: Arrival Marker - STOP at red box
                print("[COMMAND] ID=0 → ARRIVAL MARKER - STOP!")
                print("Proceeding to find sample in white circle...")
                print("=" * 50)
                self.arrival_marker_detected = True
                self.at_red_box = True
                return State.STOP_AT_RED
                
            elif tag_id == APRILTAG_TURN_RIGHT:  # ID=1: Turn Right
                print("[COMMAND] ID=1 → TURN RIGHT 90°")
                print("=" * 50)
                self.pending_turn_direction = 'right'
                self.turn_timer = 0
                return State.EXECUTE_TURN
                
            elif tag_id == APRILTAG_TURN_LEFT:  # ID=2: Turn Left
                print("[COMMAND] ID=2 → TURN LEFT 90°")
                print("=" * 50)
                self.pending_turn_direction = 'left'
                self.turn_timer = 0
                return State.EXECUTE_TURN
            
            else:
                # Unknown ID - continue forward
                print(f"[INFO] Unknown ID={tag_id}, continuing straight...")
                return State.MOVE_TO_FLAG
        
        # ====================================================================
        # APPROACHING FLAG - Navigate towards it with steering correction
        # ====================================================================
        
        # Determine speed based on obstacle proximity
        current_speed = CRUISE_SPEED
        if can_pass_over:
            current_speed = SLOW_SPEED  # Slow for passable obstacles
        elif obstacle['front_warning']:
            current_speed = CRUISE_SPEED * 0.8  # Slightly slower near obstacles
        
        # Steer towards flag center (angle correction)
        if abs(angle) > 0.15:
            # Significant angle offset - curve towards flag
            if angle < 0:
                self.curve_left(current_speed, 0.6)
            else:
                self.curve_right(current_speed, 0.6)
        elif abs(angle) > 0.05:
            # Minor angle offset - gentle curve
            if angle < 0:
                self.curve_left(current_speed, 0.85)
            else:
                self.curve_right(current_speed, 0.85)
        else:
            # Aligned - go straight
            self.move_forward(current_speed)
        
        return State.FOLLOW_FLAG
    
    def state_execute_turn(self):
        """Step 3: Execute turn directive (90 degrees) after reading flag.
        
        Uses gyroscope integration for precise 90° turns.
        """
        self.turn_timer += 1
        
        # Phase 1: Back up slightly to clear the flag (first 25 steps)
        if self.turn_timer <= 25:
            self.move_backward(SLOW_SPEED)
            if self.turn_timer == 1:
                print(f"[TURN] Backing up to clear flag...")
                # Initialize gyro integration for turn
                self.turn_angle_accumulated = 0.0
            return State.EXECUTE_TURN
        
        # Phase 2: Execute 90° turn using gyroscope for precise angle
        TARGET_ANGLE = math.pi / 2  # 90 degrees in radians (1.5708 rad)
        
        # Get gyro reading (angular velocity around Z axis in rad/s)
        if self.gyro:
            gyro_values = self.gyro.getValues()
            # Z-axis rotation (yaw) - index 2
            angular_velocity_z = gyro_values[2]
            
            # Integrate angular velocity to get angle turned
            # dt = TIME_STEP in seconds
            dt = TIME_STEP / 1000.0  # Convert ms to seconds
            self.turn_angle_accumulated += abs(angular_velocity_z) * dt
        else:
            # Fallback: use timer-based turning if no gyro
            self.turn_angle_accumulated += 0.02  # Estimate ~0.02 rad per step
        
        # Check if we've reached 90 degrees
        if self.turn_angle_accumulated < TARGET_ANGLE:
            # Still turning - use higher speed for more decisive turn
            turn_speed = TURN_SPEED * 1.5  # Increased speed
            
            if self.pending_turn_direction == 'right':
                self.turn_right(turn_speed)
            else:
                self.turn_left(turn_speed)
            
            # Progress update every ~0.5 seconds
            if self.turn_timer % 15 == 0:
                degrees_done = math.degrees(self.turn_angle_accumulated)
                print(f"[TURNING] {self.pending_turn_direction.upper()}... {degrees_done:.1f}° / 90°")
            
            return State.EXECUTE_TURN
        
        # Reached 90 degrees - stop and brief pause
        self.stop()
        
        # Phase 3: Move forward after turn to clear the flag area
        turn_complete_step = 25 + int(self.turn_angle_accumulated * 50)  # Dynamic calculation
        forward_start = turn_complete_step + 5  # Brief pause after turn
        forward_duration = 50  # ~1.6 seconds of forward movement
        
        steps_since_turn_complete = self.turn_timer - forward_start
        
        if steps_since_turn_complete < 0:
            # Brief pause after turn
            return State.EXECUTE_TURN
        
        if steps_since_turn_complete < forward_duration:
            self.move_forward(CRUISE_SPEED)
            if steps_since_turn_complete == 0:
                degrees_done = math.degrees(self.turn_angle_accumulated)
                print(f"[TURN COMPLETE] Turned {degrees_done:.1f}°, moving forward to clear flag area...")
            return State.EXECUTE_TURN
        
        # Turn sequence complete
        self.turn_timer = 0
        direction = self.pending_turn_direction
        self.pending_turn_direction = None
        self.turn_angle_accumulated = 0.0
        print(f"\n[TURN DONE] {direction.upper()} 90° turn executed!")
        print("[CONTINUE] Moving STRAIGHT to find next flag...\n")
        self.state_timer = 0  # Reset state timer for next state
        return State.MOVE_TO_FLAG
    
    def state_stop_at_red(self):
        """Step 4: Stop at red box after ID=0 flag.
        
        After reaching the arrival marker (ID=0), the rover must:
        1. Stop and prepare
        2. Look for the white circle within the red area
        3. Find the asteroid/object in the white circle
        """
        self.stop()
        self.state_timer += 1
        
        if self.state_timer == 1:
            print("\n" + "=" * 50)
            print("   ARRIVED AT CRIMSON IMPACT SITE")
            print("=" * 50)
            print("[TASK] Find white circle with asteroid inside")
            print("[ARM] Preparing arm for search and pickup...")
        
        if self.state_timer < 60:
            # Prepare arm to search position
            if self.state_timer == 10:
                self.set_arm_position(ARM_SEARCH, speed=0.5)
                self.open_gripper()
            
            # Give arm time to move
            return State.STOP_AT_RED
        else:
            print("[STEP 5] Searching for white circle and object...")
            self.state_timer = 0
            self.search_direction = 1  # Reset search direction
            return State.FIND_SAMPLE
    
    def detect_any_object(self):
        """Detect ANY object that could be the sample (not just asteroid).
        
        Looks for objects that are:
        - Not red (not the stage)
        - Not white (not the circle)
        - Not green (not start area)
        - Not black/very dark (not flags)
        - Small enough to be a sample
        """
        if not self.camera:
            return None
        
        objects = self.camera.getRecognitionObjects()
        cam_width = self.camera.getWidth()
        cam_height = self.camera.getHeight()
        
        candidates = []
        
        for obj in objects:
            # getColors() returns ctypes array - use getNumberOfColors() for length
            num_colors = obj.getNumberOfColors()
            if num_colors < 3:
                continue
            
            colors = obj.getColors()
            r, g, b = colors[0], colors[1], colors[2]
            pos = obj.getPosition()
            img_pos = obj.getPositionOnImage()
            size = obj.getSizeOnImage()
            norm_x = (img_pos[0] - cam_width/2) / (cam_width/2)
            
            # Calculate distance
            distance = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
            
            # Skip if too far (more than 2 meters)
            if distance > 2.0:
                continue
            
            # Skip pure red objects (red stage)
            if r > 0.5 and g < 0.4 and b < 0.4:
                continue
            
            # Skip pure white objects (white circle)
            if r > 0.8 and g > 0.8 and b > 0.8:
                continue
            
            # Skip pure green objects (start area)
            if g > 0.5 and r < 0.4 and b < 0.4:
                continue
            
            # Skip very dark objects (flags, shadows)
            avg_brightness = (r + g + b) / 3.0
            if avg_brightness < 0.1:
                continue
            
            # Skip very large objects (stage, ground)
            obj_area = size[0] * size[1]
            if obj_area > (cam_width * cam_height * 0.3):
                continue
            
            # This could be a sample!
            obj_info = {
                'position': pos,
                'distance': distance,
                'angle': norm_x,
                'size': size,
                'colors': colors,
                'area': obj_area
            }
            candidates.append(obj_info)
        
        # Return the closest candidate
        if candidates:
            candidates = sorted(candidates, key=lambda x: x['distance'])
            return candidates[0]
        
        return None
    
    def state_find_sample(self):
        """Step 5: Find sample in white circle within red box.
        
        Includes obstacle detection when navigating to sample.
        """
        self.state_timer += 1
        
        # Check for obstacles while searching
        should_avoid, can_pass_over, obstacle = self.should_avoid_obstacle()
        
        if should_avoid:
            self.save_path_state()
            print("[OBSTACLE] Detected while finding sample - avoiding")
            self.avoid_timer = 0
            self.avoid_phase = 0
            return State.AVOID_OBSTACLE
        
        # Try to detect any object (universal detection)
        sample = self.detect_any_object()
        
        # Also try specific asteroid detection as backup
        if not sample:
            sample = self.get_asteroid()
        
        if sample:
            distance = sample['distance']
            angle = sample['angle']
            print(f"[DETECT] Object found at {distance:.2f}m, angle={angle:.2f}")
            self.sample_detected = True
            self.state_data['target'] = sample
            return State.APPROACH_SAMPLE
        
        # Look for white circle to navigate towards it
        white_circle = self.get_white_circle()
        
        if white_circle:
            distance = white_circle['distance']
            angle = white_circle['angle']
            
            if distance < 0.6:
                # Very close to white circle - start looking around
                print(f"[SEARCH] At white circle, scanning for object...")
                if self.state_timer % 80 < 40:
                    self.turn_right(TURN_SPEED * 0.25)
                else:
                    self.turn_left(TURN_SPEED * 0.25)
                
                # After extensive search, try pickup anyway
                if self.state_timer > 200:
                    print("[TIMEOUT] Proceeding to pickup position...")
                    self.state_timer = 0
                    return State.ALIGN_PICKUP
                
                return State.FIND_SAMPLE
            
            # Navigate to white circle
            print(f"[NAV] Moving to white circle at {distance:.2f}m")
            if abs(angle) < 0.15:
                self.move_forward(SLOW_SPEED)
            elif angle < 0:
                self.curve_left(SLOW_SPEED, 0.5)
            else:
                self.curve_right(SLOW_SPEED, 0.5)
            
            return State.FIND_SAMPLE
        
        # No white circle visible - search by rotating
        if self.state_timer % 60 == 1:
            print(f"[SEARCH] Looking for object/white circle...")
        
        if self.state_timer > 200:
            # Timeout - assume we're in position and try pickup
            print("[TIMEOUT] Search timeout, attempting pickup...")
            self.state_timer = 0
            return State.ALIGN_PICKUP
        
        # Rotate to search
        if self.state_timer % 100 < 60:
            if self.search_direction > 0:
                self.turn_right(TURN_SPEED * 0.3)
            else:
                self.turn_left(TURN_SPEED * 0.3)
        else:
            self.move_forward(SLOW_SPEED * 0.5)
        
        return State.FIND_SAMPLE
    
    def state_approach_sample(self):
        """Approach the sample for pickup.
        
        Includes obstacle detection - important for avoiding rocks near sample.
        """
        # Check for obstacles (but be more lenient near sample)
        should_avoid, can_pass_over, obstacle = self.should_avoid_obstacle()
        
        if should_avoid and obstacle.get('urgency', 0) > 80:
            # Only avoid critical obstacles near sample
            self.save_path_state()
            print("[OBSTACLE] Critical obstacle near sample - avoiding")
            self.avoid_timer = 0
            self.avoid_phase = 0
            return State.AVOID_OBSTACLE
        
        # Try both detection methods
        sample = self.detect_any_object()
        if not sample:
            sample = self.get_asteroid()
        
        if not sample:
            print("[LOST] Object not visible, searching...")
            self.state_timer += 1
            if self.state_timer > 50:
                self.state_timer = 0
                return State.FIND_SAMPLE
            self.turn_right(TURN_SPEED * 0.2)
            return State.APPROACH_SAMPLE
        
        self.state_timer = 0
        distance = sample['distance']
        angle = sample['angle']
        
        # Prepare arm as we get closer
        if distance < 1.0:
            self.set_arm_position(ARM_PRE_GRAB, speed=0.7)
            self.open_gripper()
        
        # In pickup range
        if distance < 0.5:
            print("[RANGE] Sample in pickup range!")
            self.stop()
            return State.ALIGN_PICKUP
        
        # Adjust speed based on obstacles
        move_speed = SLOW_SPEED
        if can_pass_over:
            move_speed = SLOW_SPEED * 0.5  # Extra slow for passable obstacles
        elif obstacle['front_warning']:
            move_speed = SLOW_SPEED * 0.7
        
        # Navigate towards sample
        if abs(angle) < 0.1:
            speed = move_speed if distance < 0.8 else CRUISE_SPEED * 0.6
            self.move_forward(speed)
        elif angle < 0:
            self.curve_left(move_speed, 0.4)
        else:
            self.curve_right(move_speed, 0.4)
        
        return State.APPROACH_SAMPLE
    
    def state_avoid_obstacle(self):
        """Smart obstacle avoidance with path recovery.
        
        Phases:
        0 - BACKUP: Move backward to create clearance
        1 - TURN_AWAY: Turn away from obstacle direction
        2 - FORWARD: Move forward parallel to original path
        3 - TURN_BACK: Turn back toward original heading
        4 - RETURN: Move forward while aligning to original path
        
        The rover remembers its original heading and works to return
        to the same path after passing the obstacle.
        """
        obstacle = self.check_obstacle()
        
        # Initialize avoidance on entry
        if self.avoid_timer == 0:
            self.obstacle_count += 1
            self.last_obstacle_time = self.robot.getTime()
            
            # Choose best avoidance direction
            self.avoid_direction = obstacle.get('best_avoid_direction', 1)
            self.avoid_phase = 0
            
            print(f"\n{'='*50}")
            print(f"[OBSTACLE #{self.obstacle_count}] Avoidance started")
            print(f"  Direction: {'LEFT' if self.avoid_direction > 0 else 'RIGHT'}")
            print(f"  Front dist: {obstacle['front_dist']:.0f}")
            print(f"  Saved heading: {self.pre_avoid_heading:.1f}°" if self.pre_avoid_heading else "  No heading saved")
            print(f"{'='*50}")
        
        self.avoid_timer += 1
        
        # ================================================================
        # PHASE 0: BACKUP - Create clearance from obstacle
        # ================================================================
        if self.avoid_phase == 0:
            self.move_backward(SLOW_SPEED)
            
            if self.avoid_timer >= AVOID_BACKUP_STEPS:
                print(f"[PHASE 1] Backed up, now turning {'left' if self.avoid_direction > 0 else 'right'}...")
                self.avoid_phase = 1
                self.avoid_timer = 0
            
            return State.AVOID_OBSTACLE
        
        # ================================================================
        # PHASE 1: TURN_AWAY - Turn away from obstacle (~45-60 degrees)
        # ================================================================
        elif self.avoid_phase == 1:
            if self.avoid_direction > 0:
                self.turn_left(TURN_SPEED)
            else:
                self.turn_right(TURN_SPEED)
            
            if self.avoid_timer >= AVOID_TURN_STEPS:
                print("[PHASE 2] Turned away, moving forward to pass obstacle...")
                self.avoid_phase = 2
                self.avoid_timer = 0
            
            return State.AVOID_OBSTACLE
        
        # ================================================================
        # PHASE 2: FORWARD - Move forward to pass the obstacle
        # ================================================================
        elif self.avoid_phase == 2:
            # Check if new obstacle appears in front
            if obstacle['critical']:
                print("[ALERT] New obstacle during avoidance! Turning more...")
                self.avoid_phase = 1  # Go back to turning
                self.avoid_timer = 0
                return State.AVOID_OBSTACLE
            
            # Move forward (slightly curved away from original obstacle side)
            if obstacle['front_warning']:
                # Still sensing obstacle - curve away more
                if self.avoid_direction > 0:
                    self.curve_left(CRUISE_SPEED, 0.6)
                else:
                    self.curve_right(CRUISE_SPEED, 0.6)
            else:
                self.move_forward(CRUISE_SPEED)
            
            if self.avoid_timer >= AVOID_FORWARD_STEPS:
                print("[PHASE 3] Passed obstacle, turning back toward original path...")
                self.avoid_phase = 3
                self.avoid_timer = 0
            
            return State.AVOID_OBSTACLE
        
        # ================================================================
        # PHASE 3: TURN_BACK - Turn back toward original heading
        # ================================================================
        elif self.avoid_phase == 3:
            # Turn opposite direction to return toward path
            if self.avoid_direction > 0:
                self.turn_right(TURN_SPEED)  # Was going left, now turn right
            else:
                self.turn_left(TURN_SPEED)   # Was going right, now turn left
            
            # Check if we're aligned with original heading
            if self.pre_avoid_heading is not None:
                heading_diff = abs(self.get_heading_difference(self.pre_avoid_heading))
                if heading_diff < 15:
                    print(f"[ALIGNED] Back to original heading ({self.pre_avoid_heading:.1f}°)")
                    self.avoid_phase = 4
                    self.avoid_timer = 0
                    return State.AVOID_OBSTACLE
            
            if self.avoid_timer >= AVOID_RETURN_STEPS:
                print("[PHASE 4] Turn complete, returning to path...")
                self.avoid_phase = 4
                self.avoid_timer = 0
            
            return State.AVOID_OBSTACLE
        
        # ================================================================
        # PHASE 4: RETURN - Move forward on original path
        # ================================================================
        elif self.avoid_phase == 4:
            # Fine-tune heading while moving forward
            if self.pre_avoid_heading is not None:
                heading_diff = self.get_heading_difference(self.pre_avoid_heading)
                
                if abs(heading_diff) > 10:
                    # Need to adjust heading while moving
                    if heading_diff > 0:
                        self.curve_right(CRUISE_SPEED, 0.7)
                    else:
                        self.curve_left(CRUISE_SPEED, 0.7)
                else:
                    self.move_forward(CRUISE_SPEED)
            else:
                self.move_forward(CRUISE_SPEED)
            
            # Check if path is clear ahead
            if obstacle['clear']:
                self.avoid_timer += 1
            
            if self.avoid_timer >= 40 or obstacle['clear']:
                # Avoidance complete - reset and return to navigation
                print(f"\n[OBSTACLE AVOIDED] Returning to {self.pre_avoid_state}")
                print(f"  Obstacles encountered: {self.obstacle_count}")
                print(f"{'='*50}\n")
                
                # Reset avoidance state
                self.avoid_timer = 0
                self.avoid_phase = 0
                self.path_recovery_active = False
                
                # Return to appropriate navigation state
                return_state = self.pre_avoid_state
                self.pre_avoid_state = None
                
                if return_state:
                    return return_state
                elif self.sample_collected:
                    return State.STORE
                elif self.at_red_box:
                    return State.FIND_SAMPLE
                elif self.exited_green:
                    return State.MOVE_TO_FLAG
                else:
                    return State.EXIT_GREEN
        
        # Timeout safety
        if self.avoid_timer > 200:
            print("[TIMEOUT] Avoidance timeout - forcing return to navigation")
            self.avoid_timer = 0
            self.avoid_phase = 0
            self.path_recovery_active = False
            
            if self.at_red_box:
                return State.FIND_SAMPLE
            return State.MOVE_TO_FLAG
        
        return State.AVOID_OBSTACLE
    
    def state_align_pickup(self):
        """Step 6a: Fine alignment for sample pickup."""
        self.stop()
        self.state_timer += 1
        
        # Try both detection methods
        sample = self.detect_any_object()
        if not sample:
            sample = self.get_asteroid()
        
        if not sample:
            # No object detected - but we're at the white circle
            # After timeout, proceed to pickup anyway (blind grab)
            if self.state_timer > 60:
                print("[BLIND] No object detected, attempting blind pickup...")
                self.state_timer = 0
                return State.PICKUP
            
            # Try rotating to find object
            if self.state_timer % 30 < 15:
                self.turn_right(TURN_SPEED * 0.2)
            else:
                self.turn_left(TURN_SPEED * 0.2)
            return State.ALIGN_PICKUP
        
        angle = sample['angle']
        distance = sample['distance']
        
        print(f"[ALIGN] Object at dist={distance:.2f}m, angle={angle:.2f}")
        
        # Too far - move closer
        if distance > 0.5:
            self.move_forward(SLOW_SPEED * 0.5)
            return State.ALIGN_PICKUP
        
        # Fine angle adjustment
        if abs(angle) > 0.1:
            if angle < 0:
                self.turn_left(TURN_SPEED * 0.2)
            else:
                self.turn_right(TURN_SPEED * 0.2)
            return State.ALIGN_PICKUP
        
        # Aligned!
        self.stop()
        print("[ALIGNED] Object centered, ready for pickup!")
        self.state_timer = 0
        return State.PICKUP
    
    def state_pickup(self):
        """Step 6: Execute pickup sequence with arm.
        
        Sequence:
        1. Open gripper and position arm above target
        2. Lower arm to ground level
        3. Close gripper to grab object
        4. Secure grip with slight adjustment
        5. Lift object to carry position
        6. Transition to store
        """
        self.stop()
        self.state_timer += 1
        
        # Multi-phase pickup sequence - each phase gets sufficient time for arm movement
        phase = self.state_timer // 100  # ~3.2s per phase
        
        if phase == 0:
            # Phase 1: Open gripper and position arm above target
            if self.state_timer == 1:
                print("\n" + "=" * 40)
                print("   PICKUP SEQUENCE STARTED")
                print("=" * 40)
                print("[PICKUP 1/6] Opening gripper, positioning arm...")
            self.open_gripper()
            self.set_arm_position(ARM_PRE_GRAB, speed=0.6)
            
        elif phase == 1:
            # Phase 2: Lower arm to grab position (ground level)
            if self.state_timer == 101:
                print("[PICKUP 2/6] Lowering arm to ground level...")
            self.set_arm_position(ARM_GRAB, speed=0.4)
            
        elif phase == 2:
            # Phase 3: Close gripper to grab object
            if self.state_timer == 201:
                print("[PICKUP 3/6] Closing gripper to grab object...")
            self.close_gripper()
            
        elif phase == 3:
            # Phase 4: Secure grip with slight lift
            if self.state_timer == 301:
                print("[PICKUP 4/6] Securing grip...")
            self.set_arm_position(ARM_GRAB_SECURE, speed=0.3)
            
        elif phase == 4:
            # Phase 5: Lift object to carry position
            if self.state_timer == 401:
                print("[PICKUP 5/6] Lifting object to carry position...")
            self.set_arm_position(ARM_LIFT, speed=0.4)
            
        elif phase == 5:
            # Phase 6: Confirm and transition to store
            self.sample_collected = True
            print("[PICKUP 6/6] Object secured!")
            print("\n" + "=" * 40)
            print("   PICKUP COMPLETE - STORING SAMPLE")
            print("=" * 40)
            self.state_timer = 0
            return State.STORE
        
        return State.PICKUP

    def state_store(self):
        """Step 7: Store sample in rover's storage box."""
        self.stop()
        self.state_timer += 1
        
        # Multi-phase storage sequence - arm rotates to back and drops object
        phase = self.state_timer // 120  # ~3.8s per phase for smooth arm rotation
        
        if phase == 0:
            # Phase 1: Ensure sample is securely lifted
            if self.state_timer == 1:
                print("\n" + "=" * 40)
                print("   STORAGE SEQUENCE STARTED")
                print("=" * 40)
                print("[STORE 1/7] Securing lift position...")
            self.set_arm_position(ARM_LIFT, speed=0.4)
            
        elif phase == 1:
            # Phase 2: Rotate arm toward back of rover (storage area)
            if self.state_timer == 121:
                print("[STORE 2/7] Rotating arm toward storage box...")
            self.set_arm_position(ARM_STORE_ROTATE, speed=0.3)
            
        elif phase == 2:
            # Phase 3: Position over storage compartment
            if self.state_timer == 241:
                print("[STORE 3/7] Positioning over storage box...")
            self.set_arm_position(ARM_STORE_POSITION, speed=0.3)
            
        elif phase == 3:
            # Phase 4: Lower into storage box
            if self.state_timer == 361:
                print("[STORE 4/7] Lowering into storage box...")
            self.set_arm_position(ARM_STORE_DROP, speed=0.3)
            
        elif phase == 4:
            # Phase 5: Release sample
            if self.state_timer == 481:
                print("[STORE 5/7] Releasing sample...")
            self.open_gripper()
            
        elif phase == 5:
            # Phase 6: Retract slightly and rotate back
            if self.state_timer == 601:
                print("[STORE 6/7] Retracting arm...")
            self.set_arm_position(ARM_STORE_RELEASE, speed=0.3)
            
        elif phase == 6:
            # Phase 7: Return arm to home position
            if self.state_timer == 721:
                print("[STORE 7/7] Returning arm to home position...")
            self.set_arm_position(ARM_HOME, speed=0.4)
            
        elif phase == 7:
            # Complete
            self.close_gripper()
            print("\n" + "=" * 50)
            print("   ✓ SAMPLE STORED IN ROVER BOX!")
            print("   ✓ ARM RETURNED TO HOME POSITION!")
            print("=" * 50)
            return State.MISSION_COMPLETE
        
        return State.STORE
    
    def state_mission_complete(self):
        """Mission complete - stop and report success."""
        self.stop()
        self.set_arm_position(ARM_HOME)
        
        self.state_timer += 1
        
        # Report status periodically
        if self.state_timer % 150 == 1:
            elapsed = self.robot.getTime() - self.mission_start_time
            print("\n" + "=" * 60)
            print("          🏆 MISSION COMPLETE! 🏆")
            print("=" * 60)
            print(f"  Flags navigated: {self.flags_passed}")
            print(f"  Sample collected: {self.sample_collected}")
            print(f"  Mission time: {elapsed:.1f} seconds")
            print("=" * 60 + "\n")
        
        return State.MISSION_COMPLETE
    
    # ========================================================================
    # MANUAL CONTROL
    # ========================================================================
    
    def handle_keyboard(self):
        """Handle keyboard input."""
        key = self.keyboard.getKey()
        movement = False
        
        while key >= 0:
            # Mode toggle
            if key == ord('M'):
                self.manual_mode = not self.manual_mode
                mode = "MANUAL" if self.manual_mode else "AUTONOMOUS"
                print(f"\n{'='*30}")
                print(f"  MODE: {mode}")
                print(f"{'='*30}\n")
                if not self.manual_mode:
                    self.state = State.SEARCH_FLAG
                    self.state_timer = 0
            
            # Manual controls
            if self.manual_mode:
                if key == ord('W'):
                    self.move_forward()
                    movement = True
                elif key == ord('S'):
                    self.move_backward()
                    movement = True
                elif key == ord('A'):
                    self.turn_left()
                    movement = True
                elif key == ord('D'):
                    self.turn_right()
                    movement = True
                elif key == ord(' '):
                    self.stop()
                elif key == ord('H'):
                    self.set_arm_position(ARM_HOME)
                    print("[ARM] Home position")
                elif key == ord('G'):
                    self.set_arm_position(ARM_GRAB)
                    print("[ARM] Grab position")
                elif key == ord('L'):
                    self.set_arm_position(ARM_LIFT)
                    print("[ARM] Lift position")
                elif key == ord('O'):
                    self.open_gripper()
                    print("[GRIPPER] Open")
                elif key == ord('P'):
                    self.close_gripper()
                    print("[GRIPPER] Close")
                elif key == ord('C'):
                    # Debug: show detected objects
                    flags = self.detect_apriltag_flags()
                    env = self.detect_environment_colors()
                    print(f"[CAMERA] AprilTag Flags: {len(flags)}")
                    for f in flags:
                        print(f"  - ID={f.get('apriltag_id','?')} at {f['distance']:.2f}m, angle={f['angle']:.2f}")
                    if env['red_stage']:
                        print(f"  - Red Stage at {env['red_stage']['distance']:.2f}m")
                    if env['white_circle']:
                        print(f"  - White Circle at {env['white_circle']['distance']:.2f}m")
                    if env['asteroid']:
                        print(f"  - Asteroid at {env['asteroid']['distance']:.2f}m")
            
            key = self.keyboard.getKey()
        
        if self.manual_mode and not movement:
            self.stop()
    
    # ========================================================================
    # MAIN LOOP
    # ========================================================================
    
    def step(self):
        """Execute one control step."""
        self.handle_keyboard()
        
        if self.manual_mode:
            return
        
        # State transition logging
        if self.state != self.prev_state:
            print(f"[STATE] {self.prev_state} -> {self.state}")
            self.prev_state = self.state
            self.state_timer = 0
        
        # Execute current state - Competition Flow
        state_handlers = {
            State.INIT: self.state_init,
            State.EXIT_GREEN: self.state_exit_green,       # Step 1: Exit green box
            State.INITIAL_SCAN: self.state_initial_scan,   # Step 1.5: 360° scan for first flag
            State.TURN_TO_FLAG: self.state_turn_to_flag,   # Step 1.6: Turn to face flag
            State.MOVE_TO_FLAG: self.state_move_to_flag,   # Step 2: Move to flags
            State.SEARCH_FLAG: self.state_search_flag,     # Search for flags
            State.SEARCH_RECOVER: self.state_search_recover, # Recovery scan when flag lost
            State.FOLLOW_FLAG: self.state_follow_flag,     # Navigate to flag
            State.EXECUTE_TURN: self.state_execute_turn,   # Step 3: Turn as directed
            State.AVOID_OBSTACLE: self.state_avoid_obstacle,
            State.STOP_AT_RED: self.state_stop_at_red,     # Step 4: Stop at red box
            State.FIND_SAMPLE: self.state_find_sample,     # Step 5: Find sample
            State.APPROACH_SAMPLE: self.state_approach_sample,
            State.ALIGN_PICKUP: self.state_align_pickup,
            State.PICKUP: self.state_pickup,               # Step 6: Pick up sample
            State.STORE: self.state_store,                 # Step 7: Store sample
            State.MISSION_COMPLETE: self.state_mission_complete,
        }
        
        handler = state_handlers.get(self.state)
        if handler:
            self.state = handler()
        else:
            print(f"[ERROR] Unknown state: {self.state}")
            self.state = State.INIT
    
    def run(self):
        """Main control loop."""
        print("\n[SYSTEM] Autonomous control loop starting...")
        print("[SYSTEM] AprilTag Navigation Mode")
        print("[SYSTEM] Press 'M' for manual override\n")
        
        while self.robot.step(self.timestep) != -1:
            self.step()


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    rover = AutonomousRover()
    rover.run()
