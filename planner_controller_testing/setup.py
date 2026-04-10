from setuptools import find_packages, setup
import os
import glob

package_name = 'planner_controller_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'external'), glob.glob('../config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quin',
    maintainer_email='iqmattso@mtu.edu',
    description='implements nav2 path planner and path follower params',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'combination_node = planner_controller_testing.run_combinations:main'
        ],
    },
)
