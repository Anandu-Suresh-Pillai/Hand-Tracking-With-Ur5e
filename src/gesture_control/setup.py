from setuptools import setup
import os
from glob import glob

package_name = 'gesture_control'

def collect_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    # Launch files
    data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.py')))
    # RViz configs
    data_files.append((os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')))
    # URDF files
    data_files.append((os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')))
    # Meshes — walk the entire meshes directory
    for dirpath, dirnames, filenames in os.walk('meshes'):
        if filenames:
            install_dir = os.path.join('share', package_name, dirpath)
            data_files.append((install_dir, [os.path.join(dirpath, f) for f in filenames]))
    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=collect_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Real-time UR5e gesture control via webcam hand tracking.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_control_node = gesture_control.gesture_control_node:main',
        ],
    },
)
