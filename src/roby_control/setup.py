import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roby_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sam',
    maintainer_email='rmurawka@4cad.fr',
    description='Contrôle haut niveau de Roby : suivi visuel du cube ArUco via MoveIt Servo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = roby_control.aruco_node:main',
            'visual_servo_node = roby_control.visual_servo_node:main',
        ],
    },
)
