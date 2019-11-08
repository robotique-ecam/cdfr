#!/usr/bin/env python3


"""I2C Interface ROS::2 package build tool."""


from setuptools import setup, find_packages


package_name = 'assurancetourix'


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
    keywords=['ROS2', '', 'CDFR'],
    description='Code balise Assurancetourix.',
    license='Funtech Makers :: CDFR 2020 :: Equipe 1',
    entry_points={
        'console_scripts': [
            'main = assurancetourix.main:main',
            'leds = assurancetourix.led_indicators:main',
            'controller = assurancetourix.controller:main',
        ],
    },
)
