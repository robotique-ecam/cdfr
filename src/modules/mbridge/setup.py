#!/usr/bin/env python3


"""I2C Interface ROS::2 package build tool."""


from setuptools import setup, find_packages


package_name = 'mbridge'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    install_requires=['setuptools'],
    author='Ewen BRUN',
    author_email='ewen.brun@ecam.fr',
    maintainer='Ewen BRUN',
    maintainer_email='ewen.brun@ecam.fr',
    keywords=['ROS2', 'CDFR', 'mbed', 'FRDM-K64F', 'UDP'],
    description='Interface mbed - ROS over UDP sockets',
    license='Club Robotique ECAM :: CDFR 2020 :: Equipe 1',
    entry_points={
        'console_scripts': [
            'main = mbridge.mbridge:main',
        ],
    },
)
