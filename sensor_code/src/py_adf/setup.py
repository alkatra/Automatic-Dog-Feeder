from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'py_adf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs'],
    zip_safe=True,
    maintainer='rooot',
    maintainer_email='rooot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bowl_weight_publisher = py_adf.bowl_weight_publisher:main',
            'motion_sensor_node = py_adf.motion_sensor_node:main',
            'actuation_node = py_adf.actuation_node:main',
            'video_recorder_node = py_adf.video_recorder_node:main',
        ],
    },
)
