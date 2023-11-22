from setuptools import setup
import os
from glob import glob

package_name = 'md100a_ros2_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasheed',
    maintainer_email='rasheedo.kit@gmail.com',
    description='A helper package to use AT_MOTOR-DRIVER_100A_c2h_Ver.9.0 on ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'cmd_vel_converter = md100a_ros2_utils.cmd_vel_converter:main'
        ],
    },
)
