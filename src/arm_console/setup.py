from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'arm_console'


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
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'resource'),
            glob('resource/*.qss'),
        ),
    ],
    install_requires=['numpy', 'setuptools'],
    zip_safe=True,
    maintainer='bbabbr123-sketch',
    maintainer_email='chao070503@gmail.com',
    description='Qt desktop control console for the six-axis ROS 2 arm.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'arm_console = arm_console.main:main',
        ],
    },
)
