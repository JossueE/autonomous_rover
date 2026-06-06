from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'odom_comparison'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'images'), glob('images/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='snorlix',
    maintainer_email='snorlix@example.com',
    description='Independent benchmark package for wheel odom vs RTAB-Map odom.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_compare_recorder = odom_comparison.recorder:main',
            'trial_runner = odom_comparison.trial_runner:main',
            'topic_guard = odom_comparison.topic_guard:main',
        ],
    },
)
