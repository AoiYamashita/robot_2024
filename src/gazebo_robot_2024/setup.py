from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_robot_2024'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py')),
        (os.path.join('share',package_name,'rviz'),glob('rviz/*')),
        (os.path.join('share',package_name,'urdf'),glob('urdf/*')),
        (os.path.join('share',package_name,'worlds'),glob('worlds/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yamashita',
    maintainer_email='yamashitaaoi1230@icloud.com',
    description='gazebo robot 2024',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "controll = gazebo_robot_2024.controll:main",
        ],
    },
)
