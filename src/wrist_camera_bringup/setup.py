from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'wrist_camera_bringup'


setup(
    url='https://github.com/chao142857/ros2-visual-manipulator',
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'docs'),
            glob('docs/*.md'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bbabbr123-sketch',
    maintainer_email='chao070503@gmail.com',
    description=(
        'Bringup, diagnostics, and smoke tests for a wrist-mounted D405.'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'wrist_d405_health_monitor = '
            'wrist_camera_bringup.health_monitor:main',
            'wrist_d405_smoke_test = '
            'wrist_camera_bringup.smoke_test:main',
        ],
    },
)
