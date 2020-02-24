#!/usr/bin/env python3


"""Asterix metapackage."""


from os import path
from setuptools import setup


package_name = 'actuators'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (path.join('share', package_name), ['package.xml']),
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
    entry_points={
        'console_scripts': [
            'actuators = actuators.main:main',
        ],
    },
)
