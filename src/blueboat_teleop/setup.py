from setuptools import find_packages, setup
import os
from glob import glob


def package_files(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]

package_name = 'blueboat_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), package_files('launch/*')),
        (os.path.join('share', package_name, 'config'), package_files('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mario-cirtesu',
    maintainer_email='mario-cirtesu@example.com',
    description='Joystick teleoperation for BlueBoat.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_gamepad = blueboat_teleop.teleop_node:main',
        ],
    },
)
