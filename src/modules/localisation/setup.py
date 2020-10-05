#!/usr/bin/env python3


"""Robot localisation module."""


from os import path
from setuptools import setup, find_packages


package_name = 'localisation'


setup(
    name=package_name,
    version='0.7.0',
    packages=find_packages(),
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
    description='Code node cetautomatix.',
    license='ECAM Makers :: CDFR 2020',
    entry_points={
        'console_scripts': [
            'localisation = localisation.localisation_node:main',
        ],
    },
)
