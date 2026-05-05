from glob import glob
import os

from setuptools import find_packages, setup


def package_files(pattern):
    return [path for path in glob(pattern) if os.path.isfile(path)]


package_name = 'blueboat_missions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), package_files('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mario-cirtesu',
    maintainer_email='mario-cirtesu@example.com',
    description='Mission executor nodes for BlueBoat.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'square_mission = blueboat_missions.square_mission:main',
        ],
    },
)
