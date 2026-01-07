import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_depth_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py|xml]'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neorefraction',
    maintainer_email='johnnyastudilloso@gmail.com',
    description='This package takes profit on depth camera to locate, not just objects, but their distance from the robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_predict = my_robot_depth_navigation.nodes.yolo_prediction_node:main',
            'navigation_control = my_robot_depth_navigation.nodes.navigation_control_node:main',
        ],
    },
)
