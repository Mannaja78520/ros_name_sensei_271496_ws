from setuptools import find_packages, setup

package_name = 'assign2_660610840'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mannaja',
    maintainer_email='manman7852078520@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'talker = assign2_660610840.whisper_660610840:main',
            'listener1 = assign2_660610840.hearer1_660610840:main',
            'listener2 = assign2_660610840.hearer2_660610840:main',
        ],
    },
)
