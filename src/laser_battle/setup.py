from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'laser_battle'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    scripts=[
        'laser_battle/keyboard_control.py',
        'laser_battle/enemy_detector.py',
        'laser_battle/attack_controller.py',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Laser battle robot control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = laser_battle.keyboard_control:main',
            'enemy_detector = laser_battle.enemy_detector:main',
            'attack_controller = laser_battle.attack_controller:main',
        ],
    },
)
