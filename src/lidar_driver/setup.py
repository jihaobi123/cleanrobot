from setuptools import setup
import os
from glob import glob

package_name = 'lidar_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='LD14P激光雷达驱动包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_driver_node = lidar_driver.lidar_driver_node:main',
        ],
    },
)

