#!/usr/bin/env python3


"""Asterix metapackage."""


from os import path
from setuptools import setup


package_name = 'actuators'


setup(
    name=package_name,
    version='0.8.3',
    packages=[package_name],
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
    description='Actuators Hardware Interface',
    license='ECAM Makers :: CDFR 2020',
)
