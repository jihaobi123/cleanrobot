from setuptools import setup

package_name = 'stm32_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/stm32_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='cat@todo.todo',
    description='cmd_vel to STM32 serial bridge',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_stm32 = stm32_interface.cmd_vel_to_stm32:main',
        ],
    },
)

