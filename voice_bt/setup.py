from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'voice_bt'


def recursive_data_files(src_dir, install_dir):
    """Return data_files entries for all files under src_dir, preserving structure."""
    result = []
    for dirpath, dirnames, filenames in os.walk(src_dir):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, start='.')
        dest = os.path.join('share', package_name, rel)
        files = [os.path.join(dirpath, f) for f in filenames]
        result.append((dest, files))
    return result


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Piper executable must be installed with execute permission; colcon copies it.
        *recursive_data_files('voice_assets', 'voice_assets'),
    ],
    install_requires=['setuptools', 'sounddevice', 'vosk'],
    zip_safe=True,
    maintainer='ggm',
    maintainer_email='gusgarciarrealm@gmail.com',
    description='Voice-commanded behavior tree for the autonomous rover.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'voice_bt_node = voice_bt.bt_node:main',
        ],
    },
)
