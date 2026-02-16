# AUV Motion Control System

A ROS2-based motion control system for autonomous underwater vehicles (AUVs) using pymavlink to communicate with ArduSub/Pixhawk autopilots.

## Overview

This package provides a complete action-based motion control system for your BlueROV2 Heavy configuration AUV. It consists of:

- **Action Server**: Low-level controller that communicates with Pixhawk via MAVLink
- **Action Client**: High-level mission coordinator that sends motion commands
- **Custom ROS2 Interfaces**: Action and message definitions for motion control

### Features

- Direct MAVLink communication with Pixhawk (no MAVROS dependency)
- Action-based architecture for asynchronous command execution
- Real-time vehicle state monitoring and publishing
- Comprehensive safety checks (depth limits, battery monitoring)
- Configurable parameters via YAML file
- Periodic feedback during action execution
- Support for basic motion primitives:
  - Timed movements (forward, backward, left, right, up, down)
  - Depth changes with automatic depth hold
  - Position holding
  
## System Requirements

### Hardware
- BlueROV2 Heavy (8 thruster configuration)
- Pixhawk autopilot running ArduSub firmware
- Companion computer (Raspberry Pi or similar) running Ubuntu 20.04/22.04
- MAVLink connection (serial or UDP) between companion and Pixhawk

### Software
- Ubuntu 20.04 or 22.04
- ROS2 (Humble recommended, Foxy also supported)
- Python 3.8+
- pymavlink

## Installation

### Step 1: Install ROS2

If you don't have ROS2 installed, follow the official installation guide:
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)

### Step 2: Install pymavlink

```bash
sudo apt update
sudo apt install -y python3-pip
pip3 install pymavlink
```

### Step 3: Create ROS2 Workspace

```bash
# Create workspace directory
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src

# Note: The package files have been created in /home/claude/auv_motion_control
# Copy them to your workspace
cp -r /home/claude/auv_motion_control ~/auv_ws/src/
```

### Step 4: Build the Package

```bash
cd ~/auv_ws

# Source ROS2
source /opt/ros/humble/setup.bash  # or 'foxy' if using Foxy

# Build the package
colcon build --packages-select auv_motion_control

# Source the workspace
source install/setup.bash
```

## Configuration

### MAVLink Connection

Edit the configuration file to match your setup:

```bash
cd ~/auv_ws/src/auv_motion_control/config
nano motion_control_params.yaml
```

**For Serial Connection:**
```yaml
motion_control:
  mavlink:
    connection_string: "/dev/ttyACM0"  # or /dev/ttyUSB0
    baud_rate: 115200
```

**For UDP Connection (from topside computer):**
```yaml
motion_control:
  mavlink:
    connection_string: "udpin:0.0.0.0:14550"
    baud_rate: 115200  # ignored for UDP
```

**For UDP Connection (from companion computer):**
```yaml
motion_control:
  mavlink:
    connection_string: "udpout:0.0.0.0:9000"
    baud_rate: 115200  # ignored for UDP
```

### Other Important Parameters

```yaml
# Safety limits
max_depth: 10.0  # Maximum allowed depth in meters
min_depth: 0.3   # Minimum depth (stay below surface)

# Control rates
control_loop_rate: 20  # Hz - how often to send commands
feedback_rate: 2       # Hz - how often to send feedback (0.5s)

# PWM settings
default_speed_percent: 50.0  # Default movement speed
```

## Usage

### Method 1: Run Server and Client Separately

**Terminal 1 - Start the Action Server:**
```bash
cd ~/auv_ws
source install/setup.bash
ros2 launch auv_motion_control motion_server.launch.py
```

You should see:
```
[INFO] [motion_action_server]: Motion Action Server initialized
[INFO] [motion_action_server]: MAVLink connection: udpin:0.0.0.0:14550
[INFO] [motion_action_server]: Waiting for heartbeat from autopilot...
[INFO] [motion_action_server]: MAVLink connection established!
```

**Terminal 2 - Start the Action Client:**
```bash
cd ~/auv_ws
source install/setup.bash
ros2 launch auv_motion_control motion_client.launch.py
```

The client will wait for you to press ENTER before starting the mission.

### Method 2: Run Complete System

```bash
cd ~/auv_ws
source install/setup.bash
ros2 launch auv_motion_control motion_system.launch.py
```

This launches both server and client with a 3-second delay for the client.

## Basic Mission Sequence

The included basic mission performs the following sequence:

1. **Go down 1 meter**
   - Changes depth from surface to 1 meter
   - Uses ArduSub's ALT_HOLD (depth hold) mode
   - Waits until depth is reached (within tolerance)

2. **Hold depth for 30 seconds**
   - Maintains current depth using depth hold mode
   - Sends neutral RC commands

3. **Move forward for 3 seconds**
   - Moves forward at 50% speed for 3 seconds
   - Uses STABILIZE mode
   - Approximately 3 meters forward (depends on currents)

4. **Resurface**
   - Returns to surface (0 meters depth)
   - Uses depth hold mode

## Monitoring

### View Vehicle State

In a new terminal:
```bash
cd ~/auv_ws
source install/setup.bash
ros2 topic echo /auv/vehicle_state
```

You'll see real-time updates of:
- Depth
- Heading, pitch, roll
- Battery voltage, current, remaining
- Flight mode
- Armed status

### View Action Feedback

The action client automatically prints feedback to the terminal:
```
[INFO] [motion_action_client]: Feedback: Moving forward... (1.5s / 3.0s) | 
       Progress: 50.0% | Depth: 1.02m | Battery: 16.2V
```

## Troubleshooting

### "Failed to connect to MAVLink"

**Problem:** Can't connect to Pixhawk

**Solutions:**
1. Check connection string in config file
2. For serial: Verify device path (`ls /dev/tty*`)
3. For serial: Check permissions (`sudo chmod 666 /dev/ttyACM0`)
4. For UDP: Verify MAVProxy or companion computer is forwarding messages
5. Test connection manually:
   ```bash
   python3
   from pymavlink import mavutil
   master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
   master.wait_heartbeat()
   print("Connected!")
   ```

### "No vehicle state received"

**Problem:** Server is running but not receiving MAVLink messages

**Solutions:**
1. Check that ArduSub is running on Pixhawk
2. Verify MAVLink connection with QGroundControl
3. Check firewall settings (for UDP connections)
4. Ensure heartbeat messages are being sent (look for "Sending heartbeat" in server logs)

### "Goal rejected by server"

**Problem:** Action server rejected the goal

**Solutions:**
1. Check server logs for error messages
2. Verify Pixhawk is armed (if required)
3. Check safety limits (depth, battery)
4. Ensure correct flight mode can be set

### "Mode change rejected"

**Problem:** Can't change flight mode

**Solutions:**
1. Verify mode name is correct (STABILIZE, ALT_HOLD, MANUAL)
2. Check ArduSub parameters (some modes require specific setup)
3. Ensure vehicle is in a state where mode change is allowed
4. Check ArduSub documentation for mode requirements

## File Structure

```
auv_motion_control/
├── action/
│   └── MotionControl.action          # Action definition
├── msg/
│   └── VehicleState.msg              # Vehicle state message
├── config/
│   └── motion_control_params.yaml    # Configuration file
├── launch/
│   ├── motion_server.launch.py       # Server launch file
│   ├── motion_client.launch.py       # Client launch file
│   └── motion_system.launch.py       # Complete system launch
├── auv_motion_control/
│   ├── __init__.py
│   ├── motion_action_server.py       # Action server implementation
│   ├── motion_action_client.py       # Action client implementation
│   └── utils/
│       ├── __init__.py
│       └── mavlink_helper.py         # MAVLink utility functions
├── resource/
│   └── auv_motion_control
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README.md
```

## Next Steps

### Integration with Vision System

To integrate with your YOLO-based gate detection system:

1. **Create a vision node** that publishes gate detections:
   ```python
   # In vision_node.py
   gate_pub = self.create_publisher(GateDetection, '/vision/gate_detection', 10)
   ```

2. **Modify the action client** to subscribe to vision data:
   ```python
   # In motion_action_client.py
   self.gate_sub = self.create_subscription(
       GateDetection,
       '/vision/gate_detection',
       self.gate_callback,
       10
   )
   ```

3. **Add vision-guided behaviors**:
   - Center gate in frame (calculate yaw correction)
   - Approach gate (move forward until gate reaches target size)
   - Pass through gate (combined maneuver)

### Add More Motion Primitives

Extend the action server with new action types:

1. **Add to MotionControl.action**:
   ```
   uint8 ROTATE_TO_HEADING = 4
   ```

2. **Implement in action server**:
   ```python
   def execute_rotate_to_heading(self, goal_handle, goal):
       # Implementation here
       pass
   ```

3. **Add to action client**:
   ```python
   def rotate_to_heading(self, target_heading):
       # Send goal to server
       pass
   ```

### Implement Visual Odometry

Add feature-based visual odometry for better position estimation:

1. Use OpenCV's ORB features or similar
2. Publish estimated motion to `/vision/visual_odometry`
3. Fuse with IMU data using Extended Kalman Filter
4. Use for closed-loop control corrections

## Safety Notes

1. **Always test in a controlled environment first** (pool, tether line)
2. **Monitor battery levels** - low battery can cause unexpected behavior
3. **Set appropriate depth limits** in config file
4. **Have an emergency stop mechanism** (killswitch, surface command)
5. **Verify thruster directions** match your configuration
6. **Check for objects/obstacles** before autonomous operation
7. **Keep vehicle positively buoyant** for passive surfacing on failure

## Contributing

This is a starting point for your AUV motion control system. Feel free to extend and modify based on your needs!

## License

MIT License

## References

- [ArduSub Documentation](https://www.ardusub.com/)
- [pymavlink Documentation](https://mavlink.io/en/mavgen_python/)
- [ROS2 Documentation](https://docs.ros.org/)
- [BlueROV2 Technical Details](https://bluerobotics.com/store/rov/bluerov2/)