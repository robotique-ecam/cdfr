#!/usr/bin/env python3


"""Pharaon ROS2."""


from setuptools import find_packages, setup

package_name = 'pharaon'


setup(
    name=package_name,
    version='0.7.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name])
    ],
    zip_safe=True,
    install_requires=['setuptools'],
    author='Ewen BRUN',
    author_email='ewen.brun@ecam.fr',
    maintainer='Ewen BRUN',
    maintainer_email='ewen.brun@ecam.fr',
    description='Pharaon Action Server',
    license='ECAM Makers :: CDFR 2020',
    entry_points={
        'console_scripts': [
            'pharaon = pharaon.main:main',
        ],
    },
)
