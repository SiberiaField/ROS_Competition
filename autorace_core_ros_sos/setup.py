import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autorace_core_ros_sos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='polyanskij.06@list.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'road_vision_node = autorace_core_ros_sos.road_vision_node:main',
            'motion_controller = autorace_core_ros_sos.motion_controller:main',
            'traffic_light_node = autorace_core_ros_sos.traffic_light_node:main',
            'intersection_detector = autorace_core_ros_sos.intersection_detector:main',
            'distance_node = autorace_core_ros_sos.distance_node:main'
        ],
    },
)
