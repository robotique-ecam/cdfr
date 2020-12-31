from os import path
from setuptools import setup, find_packages

package_name = "onboard_vision"

setup(
    name=package_name,
    version="0.8.3",
    packages=find_packages(),
    data_files=[
        (path.join("share", package_name), ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="phileas",
    maintainer_email="phileas.lambert@gmail.com",
    keywords=["ROS2", "", "CDFR"],
    description="Code node onboard_vision",
    license="ECAM Makers :: CDFR 2020",
    entry_points={
        "console_scripts": ["onboard_vision = onboard_vision.onboard_vision:main"],
    },
)
