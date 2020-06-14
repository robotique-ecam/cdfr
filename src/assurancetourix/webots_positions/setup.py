#!/usr/bin/env python3


""" webots_positions package """

from os import path
from setuptools import setup, find_packages

package_name = 'webots_positions'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        (path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Philéas LAMBERT',
    maintainer_email='phileas.lambert@gmail.com',
    keywords=['ROS2', 'Webots', 'CDFR'],
    description='send back positions of elements of webots',
    license='ECAM Makers :: CDFR 2020',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webots_positions = webots_positions.webots_positions:main',
        ],
    },
)
