from os import path
from setuptools import setup, find_packages

package_name = 'odrive'

setup(
    name=package_name,
    version='0.8.3',
    packages=find_packages(),
    data_files=[
        (path.join('share', package_name), ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='3wnbr1',
    maintainer_email='ewen.brun@ecam.fr',
    keywords=['ROS2', '', 'CDFR'],
    description='New drive node interfacing with an oDrive board',
    license='ECAM Makers :: CDFR 2020',
    entry_points={
        'console_scripts': [
            'onboard_vision = onboard_vision.onboard_vision:main'
        ],
    },
)
