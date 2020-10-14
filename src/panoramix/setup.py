#!/usr/bin/env python3


"""Panoramix web server."""


from setuptools import setup, find_packages


package_name = 'panoramix'


setup(
    name=package_name,
    version='0.8.2',
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
    description='Panoramix web server.',
    license='ECAM Makers :: CDFR 2020',
    entry_points={
        'console_scripts': [
            'panoramix = panoramix.app:main',
        ],
    },
)
