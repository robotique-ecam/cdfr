#!/usr/bin/env python3


"""Obelix ROS2."""


from setuptools import setup, find_packages


package_name = 'obelix'


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml', 'launch/launch.py']),
    ],
    zip_safe=True,
    install_requires=['setuptools'],
    author='Ewen BRUN',
    author_email='ewen.brun@ecam.fr',
    maintainer='Ewen BRUN',
    maintainer_email='ewen.brun@ecam.fr',
    keywords=['ROS2', 'Robot', 'CDFR'],
    description='Obelix ROS2',
    license='ECAM Makers :: CDFR 2020',
)
