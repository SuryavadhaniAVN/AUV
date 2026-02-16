#!/usr/bin/env python3
"""
motion_action_client.py

ROS2 Action Client for coordinating high-level AUV missions.
This client sends action goals to the motion action server to execute
pre-programmed mission sequences.

Basic mission sequence:
1. Go down 1 meter
2. Hold depth for 30 seconds
3. Move forward for 3 meters (estimated 3 seconds at moderate speed)
4. Resurface

In the future, this client will integrate with the vision system to make
intelligent decisions based on camera feed and object detection.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

# Import custom action and messages
from auv_control.action import MotionControl
from auv_control.msg import VehicleState
#from auv_control.utils.mavlink_helper import PWMConverter


class MotionActionClient(Node):
    """
    ROS2 Action Client for sending motion commands to the action server.
    Coordinates mission execution and monitors vehicle state.
    """
    
    def __init__(self):
        """Initialize the Motion Action Client node."""
        super().__init__('motion_action_client')
        
        # =====================================================================
        # ACTION CLIENT
        # =====================================================================
        
        # Create action client
        self._action_client = ActionClient(
            self,
            MotionControl,
            'motion_control'
        )
        
        # =====================================================================
        # SUBSCRIBERS
        # =====================================================================
        
        # Subscribe to vehicle state
        self.vehicle_state = None
        self.state_subscriber = self.create_subscription(
            VehicleState,
            '/auv/vehicle_state',
            self.vehicle_state_callback,
            10
        )
        
        # Mission state variables
        self.mission_active = False
        self.current_goal_handle = None
        
        self.get_logger().info('Motion Action Client initialized')
        self.get_logger().info('Waiting for action server...')
        
        # Wait for action server to be available
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')
    
    # =========================================================================
    # CALLBACKS
    # =========================================================================
    
    def vehicle_state_callback(self, msg):
        """
        Callback for receiving vehicle state updates.
        
        Args:
            msg (VehicleState): Current vehicle state message
        """
        self.vehicle_state = msg
    
    def goal_response_callback(self, future):
        """
        Callback when server accepts or rejects a goal.
        
        Args:
            future: Future object containing the goal handle
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted by server')
        
        # Request result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        Callback when action completes and result is available.
        
        Args:
            future: Future object containing the action result
        """
        result = future.result().result
        
        self.get_logger().info('=' * 60)
        if result.success:
            self.get_logger().info('ACTION SUCCEEDED')
            self.get_logger().info(f'Result: {result.result_message}')
            self.get_logger().info(f'Total time: {result.total_time:.2f}s')
            self.get_logger().info(f'Final depth: {result.final_depth:.2f}m')
            self.get_logger().info(f'Final heading: {result.final_heading:.1f}°')
        else:
            self.get_logger().error('ACTION FAILED')
            self.get_logger().error(f'Result: {result.result_message}')
        self.get_logger().info('=' * 60)
        
        self.current_goal_handle = None
    
    def feedback_callback(self, feedback_msg):
        """
        Callback for receiving periodic feedback during action execution.
        
        Args:
            feedback_msg: Feedback message from the action server
        """
        feedback = feedback_msg.feedback
        
        self.get_logger().info(
            f'Feedback: {feedback.status_message} | '
            f'Progress: {feedback.progress_percent:.1f}% | '
            f'Depth: {feedback.current_depth:.2f}m | '
            f'Battery: {feedback.battery_voltage:.1f}V'
        )
    
    # =========================================================================
    # ACTION SENDING METHODS
    # =========================================================================
    
    def send_goal(self, goal):
        """
        Send a goal to the action server.
        
        Args:
            goal (MotionControl.Goal): Goal to send
            
        Returns:
            Future: Future object for the goal response
        """
        self.get_logger().info('Sending goal to action server...')
        
        # Send goal with feedback callback
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        # Add response callback
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return send_goal_future
    
    def wait_for_result(self, timeout_sec=None):
        """
        Wait for the current action to complete.
        
        Args:
            timeout_sec (float): Maximum time to wait in seconds (None = wait forever)
            
        Returns:
            bool: True if action completed, False if timeout
        """
        if self.current_goal_handle is None:
            self.get_logger().warn('No active goal to wait for')
            return False
        
        # Spin until result is received or timeout
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check if result has been received
            if self.current_goal_handle is None:
                return True
            
            # Check timeout
            if timeout_sec is not None:
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error(f'Timeout waiting for action result')
                    return False
        
        return False
    
    # =========================================================================
    # HIGH-LEVEL MISSION COMMANDS
    # =========================================================================
    
    def go_to_depth(self, target_depth, timeout_sec=60.0):
        """
        Command the AUV to go to a specific depth.
        
        Args:
            target_depth (float): Target depth in meters (positive = below surface)
            timeout_sec (float): Maximum time to wait for completion
            
        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info(f'Mission command: Go to depth {target_depth}m')
        
        # Create goal
        goal = MotionControl.Goal()
        goal.action_type = MotionControl.Goal.CHANGE_DEPTH
        goal.target_depth = target_depth
        
        # Send goal
        self.send_goal(goal)
        
        # Wait for result
        return self.wait_for_result(timeout_sec=timeout_sec)
    
    def move_direction(self, direction, duration, speed_percent=50.0, timeout_sec=None):
        """
        Command the AUV to move in a direction for a specified duration.
        
        Args:
            direction (str): Direction to move ("forward", "backward", "left", "right", "up", "down")
            duration (float): Duration in seconds
            speed_percent (float): Speed as percentage (0-100)
            timeout_sec (float): Maximum time to wait for completion (None = auto-calculate)
            
        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info(
            f'Mission command: Move {direction} for {duration}s at {speed_percent}% speed'
        )
        
        # Auto-calculate timeout if not provided
        if timeout_sec is None:
            # Timeout is 2x the expected duration plus 5 seconds buffer
            timeout_sec = (duration * 2.0) + 5.0
        
        # Create goal
        goal = MotionControl.Goal()
        goal.action_type = MotionControl.Goal.MOVE_TIMED
        goal.direction = direction
        goal.duration = duration
        goal.speed_percent = speed_percent
        
        # Send goal
        self.send_goal(goal)
        
        # Wait for result
        return self.wait_for_result(timeout_sec=timeout_sec)
    
    def hold_position(self, duration, timeout_sec=None):
        """
        Command the AUV to hold its current position for a specified duration.
        
        Args:
            duration (float): Duration to hold in seconds
            timeout_sec (float): Maximum time to wait for completion (None = auto-calculate)
            
        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info(f'Mission command: Hold position for {duration}s')
        
        # Auto-calculate timeout if not provided
        if timeout_sec is None:
            timeout_sec = (duration * 1.5) + 5.0
        
        # Create goal
        goal = MotionControl.Goal()
        goal.action_type = MotionControl.Goal.HOLD_POSITION
        goal.hold_duration = duration
        
        # Send goal
        self.send_goal(goal)
        
        # Wait for result
        return self.wait_for_result(timeout_sec=timeout_sec)
    
    def resurface(self, timeout_sec=60.0):
        """
        Command the AUV to resurface (go to 0 meters depth).
        
        Args:
            timeout_sec (float): Maximum time to wait for completion
            
        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info('Mission command: Resurface')
        
        # Go to depth 0 (surface)
        return self.go_to_depth(0.0, timeout_sec=timeout_sec)
    
    # =========================================================================
    # MISSION SEQUENCES
    # =========================================================================
    
    def execute_basic_mission(self):
        """
        Execute the basic mission sequence:
        1. Go down 1 meter
        2. Hold depth for 30 seconds
        3. Move forward for 3 seconds (approximately 3 meters)
        4. Resurface
        
        Returns:
            bool: True if entire mission completed successfully, False otherwise
        """
        self.get_logger().info('=' * 70)
        self.get_logger().info('STARTING BASIC MISSION SEQUENCE')
        self.get_logger().info('=' * 70)
        
        self.mission_active = True
        mission_start_time = time.time()
        
        try:
            # ================================================================
            # Step 1: Go down 1 meter
            # ================================================================
            self.get_logger().info('\n--- STEP 1: Go down to 1 meter depth ---')
            if not self.go_to_depth(target_depth=1.0, timeout_sec=60.0):
                self.get_logger().error('Failed to reach target depth')
                return False
            
            self.get_logger().info('✓ Step 1 complete\n')
            time.sleep(2.0)  # Brief pause between steps
            
            # ================================================================
            # Step 2: Hold depth for 30 seconds
            # ================================================================
            self.get_logger().info('--- STEP 2: Hold depth for 30 seconds ---')
            if not self.hold_position(duration=30.0, timeout_sec=50.0):
                self.get_logger().error('Failed to hold position')
                return False
            
            self.get_logger().info('✓ Step 2 complete\n')
            time.sleep(2.0)
            
            # ================================================================
            # Step 3: Move forward for 3 seconds
            # ================================================================
            self.get_logger().info('--- STEP 3: Move forward for 3 seconds ---')
            if not self.move_direction(
                direction='forward',
                duration=3.0,
                speed_percent=50.0,
                timeout_sec=15.0
            ):
                self.get_logger().error('Failed to move forward')
                return False
            
            self.get_logger().info('✓ Step 3 complete\n')
            time.sleep(2.0)
            
            # ================================================================
            # Step 4: Resurface
            # ================================================================
            self.get_logger().info('--- STEP 4: Resurface ---')
            if not self.resurface(timeout_sec=60.0):
                self.get_logger().error('Failed to resurface')
                return False
            
            self.get_logger().info('✓ Step 4 complete\n')
            
            # ================================================================
            # Mission complete!
            # ================================================================
            mission_duration = time.time() - mission_start_time
            
            self.get_logger().info('=' * 70)
            self.get_logger().info('MISSION COMPLETE!')
            self.get_logger().info(f'Total mission duration: {mission_duration:.1f} seconds')
            self.get_logger().info('=' * 70)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Exception during mission: {str(e)}')
            return False
        
        finally:
            self.mission_active = False


def main(args=None):
    """Main function to start the motion action client and execute mission."""
    rclpy.init(args=args)
    
    try:
        # Create client node
        client = MotionActionClient()
        
        # Give the system a moment to fully initialize
        time.sleep(2.0)
        
        # Check if vehicle state is being received
        if client.vehicle_state is None:
            client.get_logger().warn('No vehicle state received yet, waiting...')
            timeout = 10.0
            start_time = time.time()
            while client.vehicle_state is None and time.time() - start_time < timeout:
                rclpy.spin_once(client, timeout_sec=0.1)
            
            if client.vehicle_state is None:
                client.get_logger().error('Failed to receive vehicle state')
                return
        
        client.get_logger().info('Vehicle state is being received')
        client.get_logger().info(f'Current depth: {client.vehicle_state.depth:.2f}m')
        client.get_logger().info(f'Current mode: {client.vehicle_state.mode}')
        client.get_logger().info(f'Armed: {client.vehicle_state.armed}')
        
        # Wait for user confirmation before starting mission
        client.get_logger().info('\n' + '=' * 70)
        client.get_logger().info('Ready to execute basic mission sequence')
        client.get_logger().info('=' * 70)
        input('Press ENTER to start mission (or Ctrl+C to cancel)...')
        
        # Execute the basic mission
        success = client.execute_basic_mission()
        
        if success:
            client.get_logger().info('Mission completed successfully!')
        else:
            client.get_logger().error('Mission failed or was interrupted')
        
        # Keep node alive for a moment to ensure all messages are processed
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        print('\nMission canceled by user')
    
    finally:
        # Cleanup
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
