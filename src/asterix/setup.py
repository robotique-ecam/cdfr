#!/usr/bin/env python3


"""Asterix metapackage."""


from setuptools import setup


package_name = 'asterix'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml', 'launch/launch.py', 'robot/asterix.urdf']),
    ],
    zip_safe=True,
    install_requires=['setuptools'],
    author='Ewen BRUN',
    author_email='ewen.brun@ecam.fr',
    maintainer='Ewen BRUN',
    maintainer_email='ewen.brun@ecam.fr',
    keywords=['ROS2', '', 'CDFR'],
    description='Asterix RO2 System',
    license='Funtech Makers :: CDFR 2020 :: Equipe 1',
    entry_points={
        'console_scripts': [
            'asterix = asterix.main:main',
        ],
    },
)
