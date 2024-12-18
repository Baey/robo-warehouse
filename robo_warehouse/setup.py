import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robo_warehouse'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),

    ],
    install_requires=['setuptools', 'gazebo_ros_pkgs', 'geometry_msgs', 'slam_toolbox', 'tf2'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tugbot_driver = robo_warehouse.tugbot_driver:main',
        ],
    },
)
