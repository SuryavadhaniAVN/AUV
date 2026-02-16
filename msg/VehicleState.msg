# VehicleState.msg
# Custom ROS2 message to publish the current state of the AUV
# This aggregates data from multiple MAVLink messages into a single topic

std_msgs/Header header        # Standard ROS2 header with timestamp and frame_id

# Position and orientation
float32 depth                 # Current depth in meters (positive below surface)
float32 heading               # Current heading in degrees (0-360, 0 = North)
float32 pitch                 # Current pitch angle in degrees (-90 to +90)
float32 roll                  # Current roll angle in degrees (-180 to +180)

# Power status
float32 battery_voltage       # Battery voltage in volts
float32 battery_current       # Battery current in amperes
float32 battery_remaining     # Battery remaining as percentage (0-100)

# Flight mode and arming status
string mode                   # Current flight mode (e.g., "MANUAL", "STABILIZE", "ALT_HOLD")
bool armed                    # True if vehicle is armed, False if disarmed

# Connection status
bool mavlink_connected        # True if MAVLink connection is active
