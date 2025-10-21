from setuptools import setup
import os
from glob import glob

package_name = 'auv_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # --- THIS LINE IS NOW CORRECTED ---
        (os.path.join('share', package_name, 'meshes'), glob('models/orca4_ign/meshes/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Core package for SAUVC AUV with autonomous gate navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'optical_flow_node = auv_core.optical_flow_node:main',
            'depth_processor_node = auv_core.depth_processor_node:main',
            'perception_node = auv_core.perception_node:main',
            'mission_planner_node = auv_core.mission_planner_node:main',
            'control_node = auv_core.control_node:main',
            'serial_bridge_node = auv_core.serial_bridge_node:main',
            'sim_thruster_mapper = auv_core.sim_thruster_mapper:main',
            'bluerov_teleop = auv_core.bluerov_teleop_ign:main',
            'gate_detector_node = auv_core.gate_detector_node:main',
            'gate_navigator_node = auv_core.gate_navigator_node:main',
        ],
    },
)