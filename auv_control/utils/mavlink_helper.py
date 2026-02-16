#!/usr/bin/env python3
"""
mavlink_helper.py

Utility functions for working with MAVLink and ArduSub.
Includes PWM conversion, message builders, and common calculations.
"""

import math
from pymavlink import mavutil


class PWMConverter:
    """
    Helper class for converting between different PWM representations.
    
    PWM (Pulse Width Modulation) is used to control thrusters:
    - 1500 microseconds = neutral (no movement)
    - 1100-1499 = reverse thrust (increasing power)
    - 1501-1900 = forward thrust (increasing power)
    """
    
    def __init__(self, neutral=1500, min_pwm=1100, max_pwm=1900, deadband=50):
        """
        Initialize PWM converter with limits.
        
        Args:
            neutral (int): Neutral PWM value (no movement)
            min_pwm (int): Minimum PWM value
            max_pwm (int): Maximum PWM value
            deadband (int): Range around neutral considered as "stopped"
        """
        self.neutral = neutral
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
        self.deadband = deadband
        
    def percent_to_pwm(self, percent):
        """
        Convert percentage (-100 to +100) to PWM value.
        
        Args:
            percent (float): Percentage from -100 (full reverse) to +100 (full forward)
                           0 = neutral
        
        Returns:
            int: PWM value between min_pwm and max_pwm
        
        Examples:
            percent_to_pwm(0) -> 1500 (neutral)
            percent_to_pwm(50) -> 1700 (half forward)
            percent_to_pwm(-50) -> 1300 (half reverse)
            percent_to_pwm(100) -> 1900 (full forward)
        """
        # Clamp percentage to valid range
        percent = max(-100, min(100, percent))
        
        # Handle neutral zone (deadband)
        if abs(percent) < (self.deadband / (self.max_pwm - self.neutral) * 100):
            return self.neutral
        
        # Convert percentage to PWM
        if percent > 0:
            # Forward thrust
            pwm_range = self.max_pwm - self.neutral
            pwm = self.neutral + (percent / 100.0) * pwm_range
        else:
            # Reverse thrust
            pwm_range = self.neutral - self.min_pwm
            pwm = self.neutral + (percent / 100.0) * pwm_range
            
        return int(pwm)
    
    def pwm_to_percent(self, pwm):
        """
        Convert PWM value to percentage.
        
        Args:
            pwm (int): PWM value
            
        Returns:
            float: Percentage from -100 to +100
        """
        if pwm > self.neutral:
            # Forward
            pwm_range = self.max_pwm - self.neutral
            percent = ((pwm - self.neutral) / pwm_range) * 100
        elif pwm < self.neutral:
            # Reverse
            pwm_range = self.neutral - self.min_pwm
            percent = ((pwm - self.neutral) / pwm_range) * 100
        else:
            percent = 0
            
        return percent


def normalize_angle(angle):
    """
    Normalize angle to range [0, 360) degrees.
    
    Args:
        angle (float): Angle in degrees
        
    Returns:
        float: Normalized angle in range [0, 360)
    
    Examples:
        normalize_angle(370) -> 10
        normalize_angle(-10) -> 350
    """
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle


def angle_difference(target, current):
    """
    Calculate the shortest angular difference between two headings.
    Result is positive for clockwise rotation, negative for counter-clockwise.
    
    Args:
        target (float): Target heading in degrees [0, 360)
        current (float): Current heading in degrees [0, 360)
        
    Returns:
        float: Angular difference in degrees [-180, 180]
        
    Examples:
        angle_difference(10, 350) -> 20 (need to turn 20° clockwise)
        angle_difference(350, 10) -> -20 (need to turn 20° counter-clockwise)
    """
    diff = target - current
    
    # Normalize to [-180, 180]
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
        
    return diff


def clamp(value, min_value, max_value):
    """
    Clamp a value between minimum and maximum.
    
    Args:
        value: Value to clamp
        min_value: Minimum allowed value
        max_value: Maximum allowed value
        
    Returns:
        Clamped value
    """
    return max(min_value, min(max_value, value))


def build_rc_override_dict(pitch=0, roll=0, throttle=0, yaw=0, 
                          forward=0, lateral=0, channel_12=0):
    """
    Build a dictionary of RC channel values for BlueROV2 Heavy configuration.
    
    BlueROV2 Heavy RC Channel Mapping:
    - Channel 1: Pitch (forward/backward)
    - Channel 2: Roll (left/right lateral)
    - Channel 3: Throttle (up/down)
    - Channel 4: Yaw (rotate left/right)
    - Channel 5: Forward (alternative forward control)
    - Channel 6: Lateral (alternative lateral control)
    - Channels 7-11: Reserved
    - Channel 12: Custom (can be mapped to servo/light/etc)
    
    Args:
        All arguments are PWM values (1100-1900), 1500 = neutral
        
    Returns:
        dict: Dictionary with channel numbers as keys and PWM values
    """
    return {
        1: pitch,
        2: roll,
        3: throttle,
        4: yaw,
        5: forward,
        6: lateral,
        12: channel_12
    }


def get_mode_id(master, mode_name):
    """
    Get the mode ID for a given mode name.
    
    Args:
        master: MAVLink connection object
        mode_name (str): Name of the mode (e.g., "STABILIZE", "ALT_HOLD")
        
    Returns:
        int: Mode ID, or None if mode not found
    """
    mode_mapping = master.mode_mapping()
    return mode_mapping.get(mode_name, None)


def degrees_to_radians(degrees):
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0


def radians_to_degrees(radians):
    """Convert radians to degrees."""
    return radians * 180.0 / math.pi


if __name__ == "__main__":
    """
    Simple test of PWM converter functionality.
    Run this file directly to test the PWM conversion functions.
    """
    print("Testing PWM Converter...")
    converter = PWMConverter()
    
    # Test cases
    test_values = [-100, -50, 0, 50, 100]
    
    print("\nPercent -> PWM -> Percent")
    print("-" * 40)
    for percent in test_values:
        pwm = converter.percent_to_pwm(percent)
        back_to_percent = converter.pwm_to_percent(pwm)
        print(f"{percent:6.1f}% -> {pwm:4d} PWM -> {back_to_percent:6.1f}%")
    
    # Test angle functions
    print("\n\nTesting Angle Functions...")
    print("-" * 40)
    print(f"normalize_angle(370) = {normalize_angle(370)}")
    print(f"normalize_angle(-10) = {normalize_angle(-10)}")
    print(f"angle_difference(10, 350) = {angle_difference(10, 350)}")
    print(f"angle_difference(350, 10) = {angle_difference(350, 10)}")