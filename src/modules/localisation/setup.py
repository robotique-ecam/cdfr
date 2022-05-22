#!/usr/bin/env python3


"""Robot localisation module."""


from os import path
from setuptools import setup, find_packages


package_name = "localisation"


setup(
    name=package_name,
    version="0.8.4",
    packages=find_packages(),
    data_files=[
        (path.join("share", package_name), ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    zip_safe=True,
    install_requires=["setuptools"],
    author="Ewen BRUN",
    author_email="ewen.brun@ecam.fr",
    maintainer="Philéas LAMBERT",
    maintainer_email="phileas.lambert@ecam.fr",
    keywords=["ROS2", "localisation", "CDFR"],
    description="Code node localisation.",
    license="ECAM Makers :: CDFR 2021",
    entry_points={
        "console_scripts": [
            "localisation = localisation.localisation_node:main",
            "lh_tracker_serial = lh_tracker_serial.lh_tracker_serial_node:main",
        ],
    },
)
