#!/usr/bin/env python3
"""
motion_action_server.py

ROS2 Action Server for controlling AUV motion primitives.
This server receives high-level motion commands from the action client and
translates them into low-level MAVLink commands sent to the Pixhawk autopilot.

Key responsibilities:
1. Maintain MAVLink connection to Pixhawk
2. Send periodic heartbeats to prevent failsafe
3. Monitor vehicle state (depth, heading, battery, etc.)
4. Execute motion commands via RC override
5. Provide periodic feedback during action execution
6. Handle mode changes and safety checks

SAFETY CHECKS DISABLED (except armed check) for testing purposes.
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import threading
import time
import math

# Import MAVLink
from pymavlink import mavutil

# Import custom messages and actions
from auv_control.action import MotionControl
from auv_control.msg import VehicleState

# Import utility functions
from auv_control.utils.mavlink_helper import (
    PWMConverter, 
    normalize_angle, 
    angle_difference, 
    clamp
)


class MotionActionServer(Node):
    """
    ROS2 Action Server for AUV motion control.
    Handles MAVLink communication and motion primitive execution.
    """
    
    def __init__(self):
        """Initialize the Motion Action Server node."""
        super().__init__('motion_action_server')
        
        # =====================================================================
        # PARAMETER DECLARATION
        # Load parameters from YAML config file
        # =====================================================================
        self.declare_parameters(
            namespace='',
            parameters=[
                # MAVLink parameters
                ('mavlink.connection_string', 'udpin:0.0.0.0:14550'),
                ('mavlink.baud_rate', 115200),
                ('mavlink.heartbeat_rate', 1.0),
                
                # Control parameters
                ('control.default_speed_percent', 50.0),
                ('control.max_depth', 10.0),
                ('control.min_depth', 0.3),
                ('control.control_loop_rate', 20),
                ('control.feedback_rate', 2),
                ('control.depth_tolerance', 0.1),
                ('control.heading_tolerance', 5.0),
                
                # PWM parameters
                ('pwm.neutral', 1500),
                ('pwm.min', 1100),
                ('pwm.max', 1900),
                ('pwm.deadband', 50),
                ('pwm.speed_50_offset', 200),
                
                # Safety parameters (kept for compatibility but mostly disabled)
                ('safety.enable_depth_limit', False),  # DISABLED
                ('safety.enable_battery_check', False),  # DISABLED
                ('safety.min_battery_voltage', 14.0),
                ('safety.critical_battery_voltage', 12.0),
                ('safety.action_timeout_multiplier', 2.0),
                ('safety.command_timeout', 10.0),
                
                # Flight mode parameters
                ('flight_modes.movement_mode', 'STABILIZE'),
                ('flight_modes.depth_mode', 'ALT_HOLD'),
                ('flight_modes.surface_mode', 'MANUAL'),
            ]
        )
        
        # Retrieve parameters
        self.connection_string = self.get_parameter('mavlink.connection_string').value
        self.baud_rate = self.get_parameter('mavlink.baud_rate').value
        self.heartbeat_rate = self.get_parameter('mavlink.heartbeat_rate').value
        
        self.default_speed = self.get_parameter('control.default_speed_percent').value
        self.max_depth = self.get_parameter('control.max_depth').value
        self.min_depth = self.get_parameter('control.min_depth').value
        self.control_rate = self.get_parameter('control.control_loop_rate').value
        self.feedback_rate = self.get_parameter('control.feedback_rate').value
        self.depth_tolerance = self.get_parameter('control.depth_tolerance').value
        self.heading_tolerance = self.get_parameter('control.heading_tolerance').value
        
        pwm_neutral = self.get_parameter('pwm.neutral').value
        pwm_min = self.get_parameter('pwm.min').value
        pwm_max = self.get_parameter('pwm.max').value
        pwm_deadband = self.get_parameter('pwm.deadband').value
        self.speed_50_offset = self.get_parameter('pwm.speed_50_offset').value
        
        self.enable_depth_limit = self.get_parameter('safety.enable_depth_limit').value
        self.enable_battery_check = self.get_parameter('safety.enable_battery_check').value
        self.min_battery_voltage = self.get_parameter('safety.min_battery_voltage').value
        self.critical_battery_voltage = self.get_parameter('safety.critical_battery_voltage').value
        self.action_timeout_multiplier = self.get_parameter('safety.action_timeout_multiplier').value
        
        self.movement_mode = self.get_parameter('flight_modes.movement_mode').value
        self.depth_mode = self.get_parameter('flight_modes.depth_mode').value
        self.surface_mode = self.get_parameter('flight_modes.surface_mode').value
        
        # =====================================================================
        # INITIALIZE COMPONENTS
        # =====================================================================
        
        # PWM converter for translating percentages to PWM values
        self.pwm_converter = PWMConverter(
            neutral=pwm_neutral,
            min_pwm=pwm_min,
            max_pwm=pwm_max,
            deadband=pwm_deadband
        )
        
        # Vehicle state variables (updated from MAVLink messages)
        self.vehicle_state = {
            'depth': 0.0,
            'heading': 0.0,
            'pitch': 0.0,
            'roll': 0.0,
            'battery_voltage': 0.0,
            'battery_current': 0.0,
            'battery_remaining': 0.0,
            'mode': 'UNKNOWN',
            'armed': False,
        }
        
        # Thread-safe lock for vehicle state
        self.state_lock = threading.Lock()
        
        # MAVLink connection (initialized in connect_mavlink)
        self.master = None
        self.boot_time = time.time()
        self.mavlink_connected = False
        
        # Action execution state
        self.current_goal_handle = None
        self.action_active = False
        
        # =====================================================================
        # ROS2 PUBLISHERS AND SERVICES
        # =====================================================================
        
        # Publisher for vehicle state
        self.state_publisher = self.create_publisher(
            VehicleState,
            '/auv/vehicle_state',
            10
        )
        
        # Timer to publish vehicle state at regular intervals
        self.state_publish_timer = self.create_timer(
            1.0 / self.control_rate,  # Publish at control loop rate
            self.publish_vehicle_state
        )
        
        # =====================================================================
        # ACTION SERVER
        # =====================================================================
        
        # Create action server with reentrant callback group
        # This allows multiple callbacks to execute concurrently
        self.callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            MotionControl,
            'motion_control',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )
        
        # =====================================================================
        # MAVLINK CONNECTION AND BACKGROUND THREADS
        # =====================================================================
        
        # Connect to MAVLink
        self.connect_mavlink()
        
        # Start heartbeat thread (sends periodic heartbeat to prevent failsafe)
        self.heartbeat_thread = threading.Thread(
            target=self.heartbeat_loop,
            daemon=True
        )
        self.heartbeat_thread.start()
        
        # Start MAVLink message receiving thread
        self.mavlink_thread = threading.Thread(
            target=self.mavlink_receive_loop,
            daemon=True
        )
        self.mavlink_thread.start()
        
        self.get_logger().info('Motion Action Server initialized')
        self.get_logger().info(f'MAVLink connection: {self.connection_string}')
        self.get_logger().info(f'Control loop rate: {self.control_rate} Hz')
        self.get_logger().info(f'Feedback rate: {self.feedback_rate} Hz')
        self.get_logger().warn('SAFETY CHECKS DISABLED (except armed check) - USE WITH CAUTION!')
        
    # =========================================================================
    # MAVLINK CONNECTION AND COMMUNICATION
    # =========================================================================
    
    def connect_mavlink(self):
        """
        Establish connection to the Pixhawk via MAVLink.
        This sets up the communication link that will be used for all
        subsequent commands and telemetry.
        """
        try:
            self.get_logger().info(f'Connecting to MAVLink: {self.connection_string}')
            
            # Create MAVLink connection
            # For serial connections, baudrate is used
            # For UDP connections, baudrate is ignored
            if self.connection_string.startswith('/dev/'):
                # Serial connection
                self.master = mavutil.mavlink_connection(
                    self.connection_string,
                    baud=self.baud_rate
                )
            else:
                # UDP connection
                self.master = mavutil.mavlink_connection(
                    self.connection_string
                )
            
            # Wait for a heartbeat from the autopilot
            # This confirms the connection is working
            self.get_logger().info('Waiting for heartbeat from autopilot...')
            self.master.wait_heartbeat()
            
            self.mavlink_connected = True
            self.get_logger().info('MAVLink connection established!')
            self.get_logger().info(f'System ID: {self.master.target_system}')
            self.get_logger().info(f'Component ID: {self.master.target_component}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MAVLink: {str(e)}')
            self.mavlink_connected = False
            raise
    
    def heartbeat_loop(self):
        """
        Background thread that sends periodic heartbeat messages.
        
        ArduSub expects to receive a heartbeat at least once per second.
        If no heartbeat is received, it will trigger a failsafe.
        This thread ensures we always send heartbeats at the configured rate.
        """
        rate = 1.0 / self.heartbeat_rate
        
        while rclpy.ok():
            if self.master and self.mavlink_connected:
                try:
                    # Send heartbeat message
                    # This identifies us as a Ground Control Station (GCS)
                    self.master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,  # Type: Ground Control Station
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # Autopilot type
                        0,  # Base mode
                        0,  # Custom mode
                        mavutil.mavlink.MAV_STATE_ACTIVE  # System status
                    )
                except Exception as e:
                    self.get_logger().error(f'Error sending heartbeat: {str(e)}')
            
            time.sleep(rate)
    
    def mavlink_receive_loop(self):
        """
        Background thread that continuously receives and processes MAVLink messages.
        
        This thread listens for various message types from the autopilot and
        updates the vehicle state accordingly. Key messages monitored:
        - ATTITUDE: Roll, pitch, yaw
        - VFR_HUD: Heading, altitude, airspeed
        - SYS_STATUS: Battery voltage/current
        - HEARTBEAT: Mode, armed status
        """
        while rclpy.ok():
            if not self.master or not self.mavlink_connected:
                time.sleep(0.1)
                continue
            
            try:
                # Receive a MAVLink message (non-blocking)
                msg = self.master.recv_match(blocking=False)
                
                if msg is not None:
                    msg_type = msg.get_type()
                    
                    # Process different message types
                    if msg_type == 'ATTITUDE':
                        # Attitude data: roll, pitch, yaw (in radians)
                        with self.state_lock:
                            self.vehicle_state['roll'] = math.degrees(msg.roll)
                            self.vehicle_state['pitch'] = math.degrees(msg.pitch)
                            # Yaw is converted to heading (0-360)
                            yaw_deg = math.degrees(msg.yaw)
                            self.vehicle_state['heading'] = normalize_angle(yaw_deg)
                    
                    elif msg_type == 'VFR_HUD':
                        # VFR HUD data: airspeed, groundspeed, heading, throttle, alt, climb
                        # For underwater vehicle, 'alt' is actually depth
                        with self.state_lock:
                            self.vehicle_state['heading'] = float(msg.heading)
                            # Alt is negative when below surface, we store as positive depth
                            self.vehicle_state['depth'] = abs(float(msg.alt))
                    
                    elif msg_type == 'SYS_STATUS':
                        # System status: battery voltage, current, remaining
                        with self.state_lock:
                            self.vehicle_state['battery_voltage'] = msg.voltage_battery / 1000.0  # mV to V
                            self.vehicle_state['battery_current'] = msg.current_battery / 100.0  # cA to A
                            self.vehicle_state['battery_remaining'] = float(msg.battery_remaining)
                        
                        # SAFETY CHECK DISABLED - Battery voltage warnings commented out
                        # if self.enable_battery_check:
                        #     if self.vehicle_state['battery_voltage'] < self.critical_battery_voltage:
                        #         self.get_logger().error(
                        #             f'CRITICAL BATTERY VOLTAGE: {self.vehicle_state["battery_voltage"]:.1f}V'
                        #         )
                        #     elif self.vehicle_state['battery_voltage'] < self.min_battery_voltage:
                        #         self.get_logger().warn(
                        #             f'Low battery voltage: {self.vehicle_state["battery_voltage"]:.1f}V'
                        #         )
                    
                    elif msg_type == 'HEARTBEAT':
                        # Heartbeat: mode, armed status
                        with self.state_lock:
                            self.vehicle_state['armed'] = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                            
                            # Get mode name from custom_mode
                            mode_id = msg.custom_mode
                            # ArduSub modes are mapped in the mode_mapping
                            mode_mapping_inv = {v: k for k, v in self.master.mode_mapping().items()}
                            self.vehicle_state['mode'] = mode_mapping_inv.get(mode_id, f'UNKNOWN({mode_id})')
                
                time.sleep(0.01)  # Small delay to prevent CPU spinning
                
            except Exception as e:
                self.get_logger().error(f'Error in MAVLink receive loop: {str(e)}')
                time.sleep(0.1)
    
    def publish_vehicle_state(self):
        """
        Publish current vehicle state to ROS2 topic.
        This is called periodically by a timer at the control loop rate.
        """
        if not self.mavlink_connected:
            return
        
        # Create and populate message
        msg = VehicleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Copy state data with thread safety
        with self.state_lock:
            msg.depth = self.vehicle_state['depth']
            msg.heading = self.vehicle_state['heading']
            msg.pitch = self.vehicle_state['pitch']
            msg.roll = self.vehicle_state['roll']
            msg.battery_voltage = self.vehicle_state['battery_voltage']
            msg.battery_current = self.vehicle_state['battery_current']
            msg.battery_remaining = self.vehicle_state['battery_remaining']
            msg.mode = self.vehicle_state['mode']
            msg.armed = self.vehicle_state['armed']
        
        msg.mavlink_connected = self.mavlink_connected
        
        # Publish
        self.state_publisher.publish(msg)
    
    # =========================================================================
    # MODE MANAGEMENT
    # =========================================================================
    
    def set_mode(self, mode_name):
        """
        Change the flight mode of the autopilot.
        
        Args:
            mode_name (str): Name of the mode (e.g., "STABILIZE", "ALT_HOLD", "MANUAL")
            
        Returns:
            bool: True if mode change was successful, False otherwise
        """
        try:
            # Check if mode exists
            if mode_name not in self.master.mode_mapping():
                self.get_logger().error(f'Unknown mode: {mode_name}')
                self.get_logger().info(f'Available modes: {list(self.master.mode_mapping().keys())}')
                return False
            
            # Get mode ID
            mode_id = self.master.mode_mapping()[mode_name]
            
            self.get_logger().info(f'Setting mode to {mode_name} (ID: {mode_id})')
            
            # Send mode change command
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            # Wait for acknowledgment
            # We'll wait up to 2 seconds for a COMMAND_ACK
            start_time = time.time()
            while time.time() - start_time < 2.0:
                ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=False, timeout=0.1)
                if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.get_logger().info(f'Mode changed to {mode_name}')
                        return True
                    else:
                        self.get_logger().error(f'Mode change rejected: {ack_msg.result}')
                        return False
            
            # Timeout waiting for ACK
            self.get_logger().warn(f'Timeout waiting for mode change acknowledgment')
            return False
            
        except Exception as e:
            self.get_logger().error(f'Error setting mode: {str(e)}')
            return False
    
    def get_current_mode(self):
        """
        Get the current flight mode.
        
        Returns:
            str: Current mode name
        """
        with self.state_lock:
            return self.vehicle_state['mode']
    
    # =========================================================================
    # RC OVERRIDE (THRUSTER CONTROL)
    # =========================================================================
    
    def send_rc_override(self, pitch=1500, roll=1500, throttle=1500, yaw=1500):
        """
        Send RC override command to control thrusters.
        
        This directly controls the vehicle by overriding the RC channels.
        For BlueROV2 Heavy configuration:
        - Channel 1 (pitch): Forward/backward
        - Channel 2 (roll): Left/right lateral
        - Channel 3 (throttle): Up/down
        - Channel 4 (yaw): Rotate left/right
        
        Args:
            pitch (int): PWM for forward/backward (1100-1900, 1500=neutral)
            roll (int): PWM for left/right (1100-1900, 1500=neutral)
            throttle (int): PWM for up/down (1100-1900, 1500=neutral)
            yaw (int): PWM for rotation (1100-1900, 1500=neutral)
        """
        try:
            # MAVLink 2 supports up to 18 channels
            # Unused channels are set to 65535 (UINT16_MAX) to indicate "no change"
            rc_channel_values = [65535] * 18
            
            # Set the channels we're controlling
            rc_channel_values[0] = pitch      # Channel 1
            rc_channel_values[1] = roll       # Channel 2
            rc_channel_values[2] = throttle   # Channel 3
            rc_channel_values[3] = yaw        # Channel 4
            
            # Send RC override command
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                *rc_channel_values
            )
            
        except Exception as e:
            self.get_logger().error(f'Error sending RC override: {str(e)}')
    
    def stop_all_motion(self):
        """
        Stop all motion by sending neutral PWM values to all channels.
        This is the emergency stop function.
        """
        self.send_rc_override(
            pitch=1500,
            roll=1500,
            throttle=1500,
            yaw=1500
        )
    
    # =========================================================================
    # ACTION EXECUTION
    # =========================================================================
    
    def execute_callback(self, goal_handle):
        """
        Main callback for executing action goals.
        This is called when the action client sends a new goal.
        
        Args:
            goal_handle: ROS2 action goal handle containing the goal parameters
            
        Returns:
            MotionControl.Result: Result of the action execution
        """
        self.get_logger().info('Received new motion goal')
        
        # Store current goal handle
        self.current_goal_handle = goal_handle
        self.action_active = True
        
        # Get goal parameters
        goal = goal_handle.request
        action_type = goal.action_type
        
        # Create result object
        result = MotionControl.Result()
        
        try:
            # Execute the appropriate action based on type
            if action_type == MotionControl.Goal.MOVE_TIMED:
                success, message = self.execute_move_timed(goal_handle, goal)
            
            elif action_type == MotionControl.Goal.CHANGE_DEPTH:
                success, message = self.execute_change_depth(goal_handle, goal)
            
            elif action_type == MotionControl.Goal.HOLD_POSITION:
                success, message = self.execute_hold_position(goal_handle, goal)
            
            else:
                success = False
                message = f'Unknown action type: {action_type}'
                self.get_logger().error(message)
            
            # Populate result
            result.success = success
            result.result_message = message
            
            with self.state_lock:
                result.final_depth = self.vehicle_state['depth']
                result.final_heading = self.vehicle_state['heading']
            
        except Exception as e:
            self.get_logger().error(f'Exception during action execution: {str(e)}')
            result.success = False
            result.result_message = f'Exception: {str(e)}'
        
        finally:
            # Always stop motion when action completes
            self.stop_all_motion()
            self.action_active = False
            self.current_goal_handle = None
        
        # Mark goal as succeeded or aborted
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        return result
    
    def execute_move_timed(self, goal_handle, goal):
        """
        Execute a timed movement in a specified direction.
        
        Args:
            goal_handle: ROS2 action goal handle
            goal: Goal parameters containing duration, direction, and speed
            
        Returns:
            tuple: (success, message)
        """
        duration = goal.duration
        direction = goal.direction
        speed_percent = goal.speed_percent if goal.speed_percent > 0 else self.default_speed
        
        self.get_logger().info(
            f'Executing MOVE_TIMED: {direction} for {duration}s at {speed_percent}% speed'
        )
        
        # Set appropriate mode (STABILIZE for timed movements)
        if not self.set_mode(self.movement_mode):
            return False, f'Failed to set {self.movement_mode} mode'
        
        # Convert speed percentage to PWM offset
        # Speed is symmetric: 50% speed means +/-200 from neutral
        pwm_offset = int((speed_percent / 50.0) * self.speed_50_offset)
        
        # Determine PWM values based on direction
        pitch_pwm = 1500
        roll_pwm = 1500
        throttle_pwm = 1500
        yaw_pwm = 1500
        
        if direction == 'forward':
            pitch_pwm = 1500 + pwm_offset
        elif direction == 'backward':
            pitch_pwm = 1500 - pwm_offset
        elif direction == 'left':
            roll_pwm = 1500 - pwm_offset
        elif direction == 'right':
            roll_pwm = 1500 + pwm_offset
        elif direction == 'up':
            throttle_pwm = 1500 + pwm_offset
        elif direction == 'down':
            throttle_pwm = 1500 - pwm_offset
        else:
            return False, f'Unknown direction: {direction}'
        
        # Execute timed movement
        start_time = time.time()
        feedback = MotionControl.Feedback()
        last_feedback_time = start_time
        
        # Control loop
        while time.time() - start_time < duration:
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                self.stop_all_motion()
                goal_handle.canceled()
                return False, 'Goal canceled by client'
            
            # Send RC override commands
            self.send_rc_override(
                pitch=pitch_pwm,
                roll=roll_pwm,
                throttle=throttle_pwm,
                yaw=yaw_pwm
            )
            
            # Send feedback at specified rate
            current_time = time.time()
            if current_time - last_feedback_time >= (1.0 / self.feedback_rate):
                elapsed = current_time - start_time
                progress = (elapsed / duration) * 100.0
                
                # Populate feedback
                feedback.status_message = f'Moving {direction}... ({elapsed:.1f}s / {duration:.1f}s)'
                feedback.time_elapsed = elapsed
                feedback.progress_percent = progress
                
                with self.state_lock:
                    feedback.current_depth = self.vehicle_state['depth']
                    feedback.current_heading = self.vehicle_state['heading']
                    feedback.current_pitch = self.vehicle_state['pitch']
                    feedback.current_roll = self.vehicle_state['roll']
                    feedback.battery_voltage = self.vehicle_state['battery_voltage']
                
                # Publish feedback
                goal_handle.publish_feedback(feedback)
                last_feedback_time = current_time
                
                self.get_logger().info(
                    f'Moving {direction}: {progress:.1f}% complete, depth: {feedback.current_depth:.2f}m'
                )
            
            # Sleep to maintain control loop rate
            time.sleep(1.0 / self.control_rate)
        
        # Stop motion
        self.stop_all_motion()
        
        total_time = time.time() - start_time
        return True, f'Moved {direction} for {total_time:.2f} seconds'
    
    def execute_change_depth(self, goal_handle, goal):
        """
        Change to a target depth using ArduSub's depth hold mode.
        
        Args:
            goal_handle: ROS2 action goal handle
            goal: Goal parameters containing target depth
            
        Returns:
            tuple: (success, message)
        """
        target_depth = goal.target_depth
        
        self.get_logger().info(f'Executing CHANGE_DEPTH: target={target_depth}m')
        
        # SAFETY CHECK DISABLED - Depth limit check commented out
        # if self.enable_depth_limit:
        #     if target_depth > self.max_depth:
        #         return False, f'Target depth {target_depth}m exceeds maximum {self.max_depth}m'
        #     if target_depth < self.min_depth:
        #         return False, f'Target depth {target_depth}m below minimum {self.min_depth}m'
        
        # Set to DEPTH_HOLD mode (ALT_HOLD in ArduSub)
        if not self.set_mode(self.depth_mode):
            return False, f'Failed to set {self.depth_mode} mode'
        
        # Use SET_POSITION_TARGET_GLOBAL_INT to set target depth
        # In ArduSub, altitude is actually depth (negative below surface)
        try:
            self.master.mav.set_position_target_global_int_send(
                int(1e3 * (time.time() - self.boot_time)),  # ms since boot
                self.master.target_system,
                self.master.target_component,
                coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                type_mask=(  # Ignore everything except Z position
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
                ),
                lat_int=0,
                lon_int=0,
                alt=-target_depth,  # Negative because below surface
                vx=0, vy=0, vz=0,
                afx=0, afy=0, afz=0,
                yaw=0, yaw_rate=0
            )
        except Exception as e:
            return False, f'Failed to set target depth: {str(e)}'
        
        # Monitor depth until target is reached or timeout
        start_time = time.time()
        feedback = MotionControl.Feedback()
        last_feedback_time = start_time
        timeout = 30.0  # 30 second timeout for depth change
        
        while time.time() - start_time < timeout:
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False, 'Goal canceled by client'
            
            # Get current depth
            with self.state_lock:
                current_depth = self.vehicle_state['depth']
            
            # Check if we've reached target depth (within tolerance)
            depth_error = abs(current_depth - target_depth)
            if depth_error < self.depth_tolerance:
                total_time = time.time() - start_time
                return True, f'Reached target depth {current_depth:.2f}m in {total_time:.1f}s'
            
            # Send feedback
            current_time = time.time()
            if current_time - last_feedback_time >= (1.0 / self.feedback_rate):
                elapsed = current_time - start_time
                
                feedback.status_message = f'Changing depth to {target_depth}m (current: {current_depth:.2f}m)'
                feedback.time_elapsed = elapsed
                # Progress based on depth error
                max_depth_change = abs(target_depth - current_depth)
                if max_depth_change > 0:
                    progress = (1.0 - depth_error / max_depth_change) * 100.0
                else:
                    progress = 100.0
                feedback.progress_percent = min(100.0, max(0.0, progress))
                
                with self.state_lock:
                    feedback.current_depth = self.vehicle_state['depth']
                    feedback.current_heading = self.vehicle_state['heading']
                    feedback.current_pitch = self.vehicle_state['pitch']
                    feedback.current_roll = self.vehicle_state['roll']
                    feedback.battery_voltage = self.vehicle_state['battery_voltage']
                
                goal_handle.publish_feedback(feedback)
                last_feedback_time = current_time
                
                self.get_logger().info(
                    f'Depth: {current_depth:.2f}m / {target_depth:.2f}m (error: {depth_error:.2f}m)'
                )
            
            time.sleep(1.0 / self.control_rate)
        
        # Timeout
        with self.state_lock:
            current_depth = self.vehicle_state['depth']
        return False, f'Timeout reaching depth. Current: {current_depth:.2f}m, Target: {target_depth:.2f}m'
    
    def execute_hold_position(self, goal_handle, goal):
        """
        Hold current position (depth and heading) for a specified duration.
        
        Args:
            goal_handle: ROS2 action goal handle
            goal: Goal parameters containing hold duration
            
        Returns:
            tuple: (success, message)
        """
        hold_duration = goal.hold_duration
        
        self.get_logger().info(f'Executing HOLD_POSITION: duration={hold_duration}s')
        
        # Set to DEPTH_HOLD mode to maintain depth
        if not self.set_mode(self.depth_mode):
            return False, f'Failed to set {self.depth_mode} mode'
        
        # In ALT_HOLD mode, ArduSub will automatically maintain depth
        # We just need to send neutral commands and wait
        start_time = time.time()
        feedback = MotionControl.Feedback()
        last_feedback_time = start_time
        
        while time.time() - start_time < hold_duration:
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False, 'Goal canceled by client'
            
            # Send neutral commands (let depth hold do its job)
            self.send_rc_override(
                pitch=1500,
                roll=1500,
                throttle=1500,
                yaw=1500
            )
            
            # Send feedback
            current_time = time.time()
            if current_time - last_feedback_time >= (1.0 / self.feedback_rate):
                elapsed = current_time - start_time
                progress = (elapsed / hold_duration) * 100.0
                
                feedback.status_message = f'Holding position... ({elapsed:.1f}s / {hold_duration:.1f}s)'
                feedback.time_elapsed = elapsed
                feedback.progress_percent = progress
                
                with self.state_lock:
                    feedback.current_depth = self.vehicle_state['depth']
                    feedback.current_heading = self.vehicle_state['heading']
                    feedback.current_pitch = self.vehicle_state['pitch']
                    feedback.current_roll = self.vehicle_state['roll']
                    feedback.battery_voltage = self.vehicle_state['battery_voltage']
                
                goal_handle.publish_feedback(feedback)
                last_feedback_time = current_time
                
                self.get_logger().info(
                    f'Holding position: {progress:.1f}% complete, depth: {feedback.current_depth:.2f}m'
                )
            
            time.sleep(1.0 / self.control_rate)
        
        total_time = time.time() - start_time
        return True, f'Held position for {total_time:.2f} seconds'


def main(args=None):
    """Main function to start the motion action server."""
    rclpy.init(args=args)
    
    try:
        # Create server node
        server = MotionActionServer()
        
        # Use MultiThreadedExecutor to handle concurrent callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(server)
        
        # Spin
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            # Cleanup
            server.get_logger().info('Shutting down motion action server')
            server.stop_all_motion()
            server.destroy_node()
            
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()