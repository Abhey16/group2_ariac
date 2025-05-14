#!/usr/bin/env python3

from setuptools import setup

package_name = 'group2_ariac'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rey',
    maintainer_email='reyroque@umd.edu',
    description='Python nodes for group2_ariac package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'python_logger_node = group2_ariac.python_logger_node:main',
            'robot_controller_node = group2_ariac.robot_controller_node:main',
        ],
    },
)
