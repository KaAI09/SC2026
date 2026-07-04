from setuptools import find_packages, setup

package_name = 'driving_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='topst',
    maintainer_email='sooyong.park@telechips.com',
    description='ROS-independent perception + control core shared by online '
                'nodes and offline evaluation tools.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    # library package: no console_scripts (imported, not run as a node)
)
