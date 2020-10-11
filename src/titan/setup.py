#!/usr/bin/env python3


"""Robot base package."""


from os import path
from glob import glob
from setuptools import setup, find_packages


package_name = 'titan'


setup(
    name=package_name,
    version='0.8.1',
    packages=find_packages(),
    data_files=[
        (path.join('share', package_name), ['package.xml']),
        (path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (path.join('share', package_name, 'launch'), glob('launch/*')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name])
    ],
    zip_safe=True,
    install_requires=['setuptools'],
    author='Ewen BRUN',
    author_email='ewen.brun@ecam.fr',
    maintainer='Ewen BRUN',
    maintainer_email='ewen.brun@ecam.fr',
    keywords=['ROS2', 'Robot', 'CDFR'],
    description='Robot base package',
    license='ECAM Makers :: CDFR 2020',
    entry_points={
        'console_scripts': [
            'pharaon = titan.pharaon:main',
            'game_manager = titan.game_manager:main',
        ],
    },
)
