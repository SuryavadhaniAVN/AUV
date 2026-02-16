# QUICK START GUIDE - AUV Motion Control System

## Complete Step-by-Step Instructions

### Prerequisites Checklist
- [ ] Ubuntu 20.04 or 22.04 installed
- [ ] ROS2 Humble or Foxy installed
- [ ] Pixhawk with ArduSub firmware
- [ ] MAVLink connection configured

---

## PART 1: Installation

### 1. Install Required System Packages

```bash
# Update system
sudo apt update
sudo apt upgrade -y

# Install Python pip
sudo apt install -y python3-pip

# Install pymavlink
pip3 install pymavlink

# Verify installation
python3 -c "import pymavlink; print('pymavlink installed successfully!')"
```

### 2. Set Up ROS2 Workspace

```bash
# Create workspace
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src

# Copy the package from /home/claude/auv_motion_control to your workspace
# (Assuming you're running this on the same system where the files were created)
cp -r /home/claude/auv_motion_control ./

# OR if you're on a different machine, transfer the entire folder:
# scp -r user@source:/home/claude/auv_motion_control ~/auv_ws/src/
```

### 3. Build the Package

```bash
cd ~/auv_ws

# Source ROS2 (adjust for your ROS2 version)
source /opt/ros/humble/setup.bash  # or 'foxy'

# Install dependencies
sudo apt install -y python3-colcon-common-extensions

# Build
colcon build --packages-select auv_motion_control

# Check for errors - you should see "Summary: 1 package finished"
```

### 4. Source Your Workspace

```bash
# Add to your .bashrc for automatic sourcing
echo "source ~/auv_ws/install/setup.bash" >> ~/.bashrc

# Source now for current terminal
source ~/auv_ws/install/setup.bash
```

---

## PART 2: Configuration

### 1. Configure MAVLink Connection

```bash
cd ~/auv_ws/src/auv_motion_control/config
nano motion_control_params.yaml
```

**Choose your connection type:**

**Option A: USB/Serial Connection**
```yaml
connection_string: "/dev/ttyACM0"
baud_rate: 115200
```

**Option B: UDP from Topside Computer**
```yaml
connection_string: "udpin:0.0.0.0:14550"
baud_rate: 115200  # ignored for UDP
```

**Option C: UDP from Companion Computer**
```yaml
connection_string: "udpout:0.0.0.0:9000"
baud_rate: 115200  # ignored for UDP
```

### 2. Adjust Safety Parameters (Optional)

```yaml
# Safety parameters
safety:
  enable_depth_limit: true
  max_depth: 10.0  # meters
  min_depth: 0.3   # meters
  min_battery_voltage: 14.0  # volts
```

### 3. Test MAVLink Connection

Before running ROS2 nodes, verify your MAVLink connection works:

```bash
python3
```

```python
from pymavlink import mavutil

# Test connection (use your connection string)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected! System ID: {master.target_system}")
exit()
```

If this fails, fix your MAVLink connection before proceeding.

---

## PART 3: First Run

### Test 1: Start the Action Server Only

```bash
# Terminal 1
cd ~/auv_ws
source install/setup.bash
ros2 launch auv_motion_control motion_server.launch.py
```

**Expected output:**
```
[INFO] [motion_action_server]: Motion Action Server initialized
[INFO] [motion_action_server]: MAVLink connection: udpin:0.0.0.0:14550
[INFO] [motion_action_server]: Waiting for heartbeat from autopilot...
[INFO] [motion_action_server]: MAVLink connection established!
[INFO] [motion_action_server]: System ID: 1
[INFO] [motion_action_server]: Component ID: 1
```

**If you see this, your server is working! Press Ctrl+C to stop.**

### Test 2: Check Vehicle State Topic

```bash
# Terminal 1 - Keep server running
ros2 launch auv_motion_control motion_server.launch.py

# Terminal 2 - Check if vehicle state is being published
source ~/auv_ws/install/setup.bash
ros2 topic echo /auv/vehicle_state
```

**You should see:**
```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: base_link
depth: 0.0
heading: 45.2
pitch: 1.3
roll: -0.8
battery_voltage: 16.4
...
```

**If you see data, everything is working correctly!**

### Test 3: Run Complete Mission

**IMPORTANT: Only run this in a safe environment (pool, controlled area)**

```bash
# Method 1: Run server and client in one command
cd ~/auv_ws
source install/setup.bash
ros2 launch auv_motion_control motion_system.launch.py
```

The system will:
1. Start the action server
2. Wait 3 seconds
3. Start the action client
4. Wait for you to press ENTER
5. Execute the mission:
   - Go down 1 meter
   - Hold for 30 seconds
   - Move forward 3 seconds
   - Resurface

**OR Method 2: Run separately for more control**

```bash
# Terminal 1 - Server
ros2 launch auv_motion_control motion_server.launch.py

# Terminal 2 - Client (after server is running)
ros2 launch auv_motion_control motion_client.launch.py
```

---

## PART 4: Troubleshooting

### Problem: "Failed to connect to MAVLink"

**For Serial Connection:**
```bash
# Check device exists
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in for group change to take effect

# Set permissions
sudo chmod 666 /dev/ttyACM0
```

**For UDP Connection:**
```bash
# Check if something is listening on the port
sudo netstat -tulpn | grep 14550

# Try pinging Pixhawk with mavproxy
mavproxy.py --master=udpin:0.0.0.0:14550
```

### Problem: "No vehicle state received"

**Solutions:**
1. Verify ArduSub is running on Pixhawk (LED should be blinking)
2. Check QGroundControl can connect to vehicle
3. Verify MAVLink port in companion computer settings
4. Check firewall: `sudo ufw status`

### Problem: colcon build fails

**Common fixes:**
```bash
# Install missing dependencies
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-action-msgs \
    ros-humble-std-msgs

# Clean and rebuild
cd ~/auv_ws
rm -rf build/ install/ log/
colcon build --packages-select auv_motion_control
```

### Problem: "Package not found"

```bash
# Make sure you sourced your workspace
source ~/auv_ws/install/setup.bash

# Verify package is built
ros2 pkg list | grep auv_motion

# Rebuild if necessary
cd ~/auv_ws
colcon build --packages-select auv_motion_control
source install/setup.bash
```

---

## PART 5: Next Steps

### Customize the Mission

Edit the mission client to create your own sequence:

```bash
cd ~/auv_ws/src/auv_motion_control/auv_motion_control
nano motion_action_client.py
```

Find the `execute_basic_mission` function and modify the steps:

```python
def execute_basic_mission(self):
    # Step 1: Custom depth
    self.go_to_depth(target_depth=2.0, timeout_sec=60.0)
    
    # Step 2: Custom hold time
    self.hold_position(duration=60.0)
    
    # Step 3: Move in different direction
    self.move_direction('left', duration=5.0, speed_percent=30.0)
    
    # Step 4: Return to surface
    self.resurface()
```

After editing, rebuild:
```bash
cd ~/auv_ws
colcon build --packages-select auv_motion_control
source install/setup.bash
```

### Monitor Logs

```bash
# View server logs
ros2 run auv_motion_control motion_server 2>&1 | tee server.log

# View all ROS2 topics
ros2 topic list

# Monitor action status
ros2 action list
ros2 action info /motion_control
```

### Safety Testing Checklist

Before water testing:
- [ ] Tested in simulation/bench test
- [ ] Verified all thrusters respond correctly
- [ ] Confirmed emergency stop works
- [ ] Set appropriate depth limits
- [ ] Checked battery voltage monitoring
- [ ] Verified vehicle is positively buoyant
- [ ] Have a safety line/tether attached
- [ ] Tested in shallow water first

---

## PART 6: Quick Command Reference

```bash
# Build package
cd ~/auv_ws && colcon build --packages-select auv_motion_control

# Source workspace
source ~/auv_ws/install/setup.bash

# Run server only
ros2 launch auv_motion_control motion_server.launch.py

# Run client only
ros2 launch auv_motion_control motion_client.launch.py

# Run complete system
ros2 launch auv_motion_control motion_system.launch.py

# View vehicle state
ros2 topic echo /auv/vehicle_state

# List all topics
ros2 topic list

# Check action server status
ros2 action list
```

---

## Getting Help

If you encounter issues:

1. Check the README.md for detailed troubleshooting
2. Review ArduSub documentation: https://www.ardusub.com/
3. Check pymavlink examples: https://www.ardusub.com/developers/pymavlink.html
4. Verify your MAVLink connection with QGroundControl first

---

## Success Checklist

- [ ] pymavlink installed and working
- [ ] ROS2 workspace created and sourced
- [ ] Package built successfully
- [ ] MAVLink connection configured correctly
- [ ] Action server starts and connects to Pixhawk
- [ ] Vehicle state topic is publishing data
- [ ] Mission runs successfully in safe environment

Once all items are checked, you're ready for autonomous operations!