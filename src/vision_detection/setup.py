from setuptools import setup
import os
from glob import glob

package_name = 'vision_detection'

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
        (os.path.join('share', package_name, 'models'), glob('models/*.rknn')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='基于YOLOv8和RKNN的视觉检测包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_detection_node = vision_detection.vision_detection_node:main',
        ],
    },
)

