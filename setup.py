import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'maze_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Walid Alsafadi',
    maintainer_email='walsafadi@smail.ucas.edu.ps',
    description='ROS 2 maze navigation using TurtleBot3 and the Potential Field Method.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'potential_field_planner = maze_navigation.potential_field_planner:main',
            'potential_field_bonus_planner = maze_navigation.potential_field_bonus_planner:main',
        ],
    },
)