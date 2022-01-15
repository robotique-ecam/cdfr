from os import path
from setuptools import setup, find_packages

package_name = "manual_command"

setup(
    name=package_name,
    version="0.9.0",
    packages=find_packages(),
    data_files=[
        (path.join("share", package_name), ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Philéas LAMBERT",
    maintainer_email="phileas.lambert@ecam.fr",
    description="Remote controller package for ECAM MAKERS robots",
    license="ECAM Makers :: CDFR 2022",
    entry_points={
        "console_scripts": [
            "manual_command = manual_command.manual_command_node:main",
        ],
    },
)
