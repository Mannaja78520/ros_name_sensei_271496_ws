from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'assign5_660610840'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mannaja',
    maintainer_email='manman7852078520@gmail.com',
    description='Github : https://github.com/Mannaja78520/ros_name_sensei_ws.git',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = assign5_660610840.whisper_660610840:main',
            'listener1 = assign5_660610840.hearer1_660610840:main',
            'listener2 = assign5_660610840.hearer2_660610840:main',
        ],
    },
)
