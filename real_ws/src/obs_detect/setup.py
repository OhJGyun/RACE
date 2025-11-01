from setuptools import setup, find_packages
from glob import glob

package_name = 'obs_detect'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch / config 전체 설치 (확실히 잡히게 *.py도 허용)
        ('share/' + package_name + '/launch', glob('launch/*.launch.py') + glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='...',
    maintainer_email='...',
    description='...',
    license='...',
    entry_points={
        'console_scripts': [
            'obs_detect_node = obs_detect.obs_detect_node:main',
            'ring_viz_node = obs_detect.ring_viz_node:main',
            'ring_viz_optimized = obs_detect.ring_viz_node_optimized:main',
            'boundary_viz_node = obs_detect.boundary_viz_node:main',
            'obs_detect_real_node = obs_detect.joon_obs_node:main',
        ],
    },
)
