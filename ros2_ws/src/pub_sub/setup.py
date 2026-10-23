from setuptools import find_packages, setup

package_name = 'pub_sub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prasannafree',
    maintainer_email='prasannafootball25@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'left_wheel_pub = pub_sub.left_wheel_pub_node:main',
            'right_wheel_pub = pub_sub.right_wheel_pub_node:main',
            'wheel_speed_sub = pub_sub.wheel_speed_sub_node:main',
        ],
    },
)
