import os
from glob import glob
from setuptools import setup

package_name = 'iw_hub_lift_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sharath',
    maintainer_email='sharath@example.com',
    description='Slow, velocity-limited lift joint control for the iw_hub in Isaac Sim',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lift_ramp_node = iw_hub_lift_control.lift_ramp_node:main',
        ],
    },
)
