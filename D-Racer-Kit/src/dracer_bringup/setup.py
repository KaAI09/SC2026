import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dracer_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='topst',
    maintainer_email='sooyong.park@telechips.com',
    description='Launch files (bring-up) for the D-Racer track-test pipeline.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    # launch-only package: no console_scripts.
)
