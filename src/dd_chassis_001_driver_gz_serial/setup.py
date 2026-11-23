from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dd_chassis_001_driver_gz_serial'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u22',
    maintainer_email='Rui@sz.com',
    description='假 MCU：本地虚拟串口 <-> Gazebo 双向转发（模拟 DD_Chassis_001 固件）',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gz_serial_node = dd_chassis_001_driver_gz_serial.gz_serial_node:main',
        ],
    },
)
