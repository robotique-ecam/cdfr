#!/usr/bin/env python3


"""Teb_obstacles package."""


from os import path
from setuptools import setup, find_packages

package_name = 'teb_obstacles'

setup(
    name=package_name,
    version='0.8.3',
    packages=find_packages(),
    data_files=[
        (path.join("share", package_name), ["package.xml"]),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author="Philéas LAMBERT",
    maintainer='Phileas LAMBERT',
    maintainer_email='phileas.lambert@gmail.com',
    keywords=["ROS2", "teb_obstacles", "CDFR"],
    description='teb_obstacles node compute and send dynamic teb_obstacles',
    license="ECAM Makers :: CDFR 2021",
    entry_points={
        'console_scripts': [
            'teb_obstacles = teb_obstacles.teb_obstacles:main'
        ],
    },
)
