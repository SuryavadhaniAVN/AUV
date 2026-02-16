from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'auv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Suryavadhani AVN',
    maintainer_email='surya06ammu@gmail.com',
    description='ROS2 package for AUV motion control',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'motion_server = auv_control.motion_action_server:main',
            'motion_client = auv_control.motion_action_client:main',
        ],
    },
)
