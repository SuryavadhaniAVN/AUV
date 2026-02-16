#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('auv_control')
    config_file = os.path.join(pkg_dir, 'config', 'motion_control_params.yaml')
    
    # Get the install directory
    install_dir = os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir)))
    
    # Add both paths
    dist_packages = os.path.join(install_dir, 'auv_control', 'local', 'lib', 'python3.10', 'dist-packages')
    site_packages = os.path.join(install_dir, 'auv_control', 'lib', 'python3.10', 'site-packages')
    
    current_pythonpath = os.environ.get('PYTHONPATH', '')
    new_pythonpath = f"{dist_packages}:{site_packages}:{current_pythonpath}"
    
    motion_server_node = Node(
        package='auv_control',
        executable='motion_action_server.py',
        name='motion_action_server',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )
    
    motion_client_node = Node(
        package='auv_control',
        executable='motion_action_client.py',
        name='motion_action_client',
        output='screen',
        emulate_tty=True,
    )
    
    delayed_client = TimerAction(
        period=3.0,
        actions=[motion_client_node]
    )
    
    return LaunchDescription([
        SetEnvironmentVariable('PYTHONPATH', new_pythonpath),
        motion_server_node,
        delayed_client
    ])