import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pitfusion3d_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vish',
    maintainer_email='vishy300@gmail.com',
    description='Real-hardware sensor drivers and bringup for PitFusion3D.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_camera_node = pitfusion3d_bringup.rgb_camera_node:main',
            'thermal_camera_node = pitfusion3d_bringup.thermal_camera_node:main',
            'camera_viewer = pitfusion3d_bringup.camera_viewer:main',
            'livox_restamp_node = pitfusion3d_bringup.livox_restamp_node:main',
        ],
    },
)
