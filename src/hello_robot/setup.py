import os
from setuptools import setup

package_name = 'hello_robot'
utils = os.path.join(package_name, 'utils')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, utils],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vitalija Alisauskaite',
    maintainer_email='v.alishauskaite@gmail.com',
    description='Robot programming assingment.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_log_service = hello_robot.velocity_logging_service:main',
            'velocity_log_client = hello_robot.velocity_posting_client:main',
            
            'command_publisher = hello_robot.command_publisher:main',
            'command_subscriber = hello_robot.command_demo_subscriber:main'
            
        ],
    },
)
