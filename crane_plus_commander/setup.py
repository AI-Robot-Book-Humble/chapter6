import os
from glob import glob
from setuptools import setup

package_name = 'crane_plus_commander'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MASUTANI Yasuhiro',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='Commander for crane_plus_control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander1 = crane_plus_commander.commander1:main',
            'commander2 = crane_plus_commander.commander2:main',
            'commander3 = crane_plus_commander.commander3:main',
            'commander4 = crane_plus_commander.commander4:main',
            'commander5 = crane_plus_commander.commander5:main',
            'commander6 = crane_plus_commander.commander6:main',
            'test_client = crane_plus_commander.test_client:main',
            'commander4_moveit = crane_plus_commander.commander4_moveit:main',
            'commander2_moveit = crane_plus_commander.commander2_moveit:main',
            'commander1_moveit = crane_plus_commander.commander1_moveit:main',
            'commander1_moveit_box = crane_plus_commander.commander1_moveit_box:main',
            'commander5_moveit = crane_plus_commander.commander5_moveit:main',
            'commander6_moveit = crane_plus_commander.commander6_moveit:main',
            'commander7 = crane_plus_commander.commander7:main',
            'commander7_moveit = crane_plus_commander.commander7_moveit:main',
            'challenge6_1 = crane_plus_commander.challenge6_1:main',
            'challenge6_2 = crane_plus_commander.challenge6_2:main',
            'challenge6_4 = crane_plus_commander.challenge6_4:main',
            'challenge6_5 = crane_plus_commander.challenge6_5:main',
            'challenge6_6 = crane_plus_commander.challenge6_6:main',
            'challenge6_7 = crane_plus_commander.challenge6_7:main',
            'challenge6_8 = crane_plus_commander.challenge6_8:main',
        ],
    },
)
