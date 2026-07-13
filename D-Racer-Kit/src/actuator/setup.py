from setuptools import find_packages, setup

package_name = 'actuator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='avees',
    maintainer_email='avees@todo.todo',
    description='Control node for PiRacerPro steering/throttle actuation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuator_node = actuator.actuator_node:main',
        ],
    },
)
