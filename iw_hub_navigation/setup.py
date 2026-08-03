import os
from glob import glob
from setuptools import setup

package_name = 'iw_hub_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sharath',
    maintainer_email='sharath@example.com',
    description='Nav2 bringup for the idealworks iw_hub AMR in Isaac Sim',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
