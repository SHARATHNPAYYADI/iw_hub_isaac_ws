import os
from glob import glob
from setuptools import setup

package_name = 'iw_hub_pick_place_executor'

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
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Sharath',
    maintainer_email='sharath@example.com',
    description='Loops the iw_hub through a pick-and-place mission: Nav2 goals + lift joint actions',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_executor_node = iw_hub_pick_place_executor.mission_executor_node:main',
        ],
    },
)
