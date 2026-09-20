from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'camera_perception'

setup(
    url='https://github.com/chao142857/ros2-visual-manipulator',
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bbabbr123-sketch',
    maintainer_email='chao070503@gmail.com',
    description='RGB-D target localization and fixed-camera calibration.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_monitor_node = camera_perception.depth_monitor_node:main',
            'rgb_monitor_node = camera_perception.rgb_monitor_node:main',
            'tag_localizer_node = camera_perception.tag_localizer_node:main',
            'camera_extrinsic_calibrator_node = '
            'camera_perception.camera_extrinsic_calibrator_node:main',
            'camera_extrinsics_publisher_node = '
            'camera_perception.camera_extrinsics_publisher_node:main',
            'target_transform_node = '
            'camera_perception.target_transform_node:main',
            'click_target_node = camera_perception.click_target_node:main',
        ],
    },
)
