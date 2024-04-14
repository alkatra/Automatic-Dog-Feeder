from setuptools import find_packages, setup

package_name = 'adf_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/adf_package_launch.py']),
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
            'motion_sensor_node = adf_package.motion_sensor_node:main',
            'bowl_weight_node = adf_package.bowl_weight_node:main',
            'actuation_node = adf_package.actuation_node:main',
            'video_recorder_node = adf_package.video_recorder_node:main',
        ],
    },
)
