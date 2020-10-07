#!/usr/bin/env python3


"""I2C Interface ROS::2 package build tool."""


from os import path
from setuptools import setup, find_packages


package_name = 'strategix'


setup(
    name=package_name,
    version='0.8.1',
    packages=find_packages(),
    data_files=[
        (path.join('share', package_name), ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name])
    ],
    zip_safe=True,
    install_requires=['setuptools'],
    author='Ewen BRUN',
    author_email='ewen.brun@ecam.fr',
    maintainer='Ewen BRUN',
    maintainer_email='ewen.brun@ecam.fr',
    keywords=['ROS2', '', 'CDFR'],
    description='Code node strategix.',
    license='ECAM Makers :: CDFR 2020',
    entry_points={
        'console_scripts': [
            'strategix = strategix.strategix:main',
        ],
    },
)
