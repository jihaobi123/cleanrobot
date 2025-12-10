from setuptools import setup

package_name = 'robot_web_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/web_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Maintainer',
    maintainer_email='robot@example.com',
    description='Expose robot ROS2 data via HTTP and WebSocket using FastAPI.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_bridge_node = robot_web_bridge.web_bridge_node:main',
        ],
    },
)
