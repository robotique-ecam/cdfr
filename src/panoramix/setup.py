#!/usr/bin/env python3


"""Panoramix web server."""


from setuptools import setup, find_packages


package_name = 'panoramix'


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
    description='Panoramix web server.',
    license='Funtech Makers :: CDFR 2020',
    entry_points={
        'console_scripts': [
            'main = panoramix.main:main',
        ],
    },
)
