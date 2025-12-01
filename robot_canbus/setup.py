from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_canbus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='j1nzo',
    maintainer_email='jhoelm874@gmail.com',
    description='Robot CAN Bus bridge + teleop launch',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'can_ps4_bridge = robot_canbus.can_ps4_bridge:main',
            'mission_logger = robot_canbus.mission_logger:main',  

        ],
    },
)
