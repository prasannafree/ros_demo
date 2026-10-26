from setuptools import find_packages, setup

package_name = 'robot_dashboard'

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
    maintainer='prasannafree',
    maintainer_email='prasannafootball25@gmail.com',
    description='GUI dashboard for visualizing mobile robot telemetry data from ROS 2 topics.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dashboard = robot_dashboard.visualizer_node:main',
        ],
    },
)
