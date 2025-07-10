from setuptools import setup

package_name = 'chassis_enable'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Relay cmd_vel and enable chassis service',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = chassis_enable.cmd_vel_relay_node:main',
            'chassis_enable_client = chassis_enable.chassis_enable_client:main',
        ],
    },
)
