from setuptools import setup
import os
from glob import glob

package_name = 'auv_core'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pratham Ahuja',
    maintainer_email='pratham.ahuja309@gmail.com',
    description='SAUVC',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
   
            'optical_flow_node = auv_core.optical_flow_node:main',
            'depth_processor_node = auv_core.depth_processor_node:main',
            'control_node = auv_core.control_node:main',
            'simple_thruster_mapper = auv_core.simple_thruster_mapper:main',
            'bluerov_teleop = auv_core.bluerov_teleop_ign:main',
            
            # Vision perception
            'gate_detector_node = auv_core.gate_detector_node:main',
            'flare_detector_node = auv_core.flare_Detection:main',
            
            # Navigation & control
            'gate_navigator_node = auv_core.gate_navigator_node:main',
            
            # NEW: Enhanced safety & state management
            'safety_monitor = auv_core.safety_monitor_node:main',
            'mission_state_manager = auv_core.mission_state_manager:main',
            
            # NEW: Acoustic detection
            'acoustic_pinger_detector = auv_core.acoustic_pinger_detector:main',
            
            # Diagnostics
            'diagnostic_node = auv_core.diagnostic_node:main',

            #Mission Planner
            'behavior_tree_node = auv_core.BT:main',
            'emergency_test = auv_core.emergency_test:main' ##########
        ],
    },
)