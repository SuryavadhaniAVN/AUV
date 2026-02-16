#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the install directory
    pkg_dir = get_package_share_directory('auv_control')
    install_dir = os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir)))
    
    # Add both dist-packages (for generated interfaces) and site-packages (for your code)
    dist_packages = os.path.join(install_dir, 'auv_control', 'local', 'lib', 'python3.10', 'dist-packages')
    site_packages = os.path.join(install_dir, 'auv_control', 'lib', 'python3.10', 'site-packages')
    
    # Combine with existing PYTHONPATH
    current_pythonpath = os.environ.get('PYTHONPATH', '')
    new_pythonpath = f"{dist_packages}:{site_packages}:{current_pythonpath}"
    
    return LaunchDescription([
        SetEnvironmentVariable('PYTHONPATH', new_pythonpath),
        
        Node(
            package='auv_control',
            executable='motion_action_client.py',
            name='motion_action_client',
            output='screen',
            emulate_tty=True,
        )
    ])