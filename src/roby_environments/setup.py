import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roby_environments'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'environments'),
            glob('environments/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='rmurawka@4cad.fr',
    description='Environnements de collision chargeables pour MoveIt.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scene_loader = roby_environments.scene_loader:main',
        ],
    },
)
