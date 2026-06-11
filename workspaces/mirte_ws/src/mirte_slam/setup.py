from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mirte_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Group 23',
    maintainer_email='group23@tudelft.nl',
    description='SLAM launch and config for the MIRTE Master robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_autosaver = mirte_slam.map_autosaver:main',
        ],
    },
)
