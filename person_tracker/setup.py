from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'person_tracker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuelz',
    maintainer_email='manuelz@todo.todo',
    description='Detección de personas con YOLOv8 sobre RGB de Azure Kinect',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_tracker_node = person_tracker.person_tracker_node:main',
        ],
    },
)
