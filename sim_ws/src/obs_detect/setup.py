from setuptools import setup

package_name = 'obs_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/obs_detect.yaml',
            'config/obs_detect_params.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/obs_detect_launch.py',
            'launch/ring_viz_optimized.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joon',
    maintainer_email='joon@example.com',
    description='Obstacle detection using LiDAR within predefined bounds with RViz2 visualization.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obs_detect_node = obs_detect.obs_detect_node:main',
            'ring_obs_node = obs_detect.ring_obs_node:main',
            'boundary_viz_node = obs_detect.boundary_viz_node:main',
        ],
    },
)
