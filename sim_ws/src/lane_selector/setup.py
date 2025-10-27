from setuptools import setup

package_name = 'lane_selector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/lane_selector.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/lane_selector_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joon',
    maintainer_email='joon@example.com',
    description='Lane selection based on obstacle detection with RViz2 visualization.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_selector_node = lane_selector.lane_selector_node:main',
        ],
    },
)
