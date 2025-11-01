from setuptools import setup
import os
from glob import glob

package_name = 'map_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'steering_lut'), glob('config/steering_lut/*.csv')),
        (os.path.join('share', package_name, 'rviz2_config'), glob('rviz2_config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joon',
    maintainer_email='joon@f1tenth.org',
    description='MAP Controller Unified - Multi-lane support with TF-based localization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_controller = map_control.controller_manager:main',
            'drive_relay = map_control.drive_relay:main',
        ],
    },
)
