from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_2024'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yamashita',
    maintainer_email='yamashitaaoi1230@icloud.com',
    description='2024 robocon',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "yaslam = robot_2024.ya_slam:main",
            "serial = robot_2024.serial:main",
            "webControll = robot_2024.webControll:main",
        ],
    },
)
