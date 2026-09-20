from setuptools import find_packages, setup

package_name = 'arm_driver'

setup(
    url='https://github.com/chao142857/ros2-visual-manipulator',
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
    maintainer='bbabbr123-sketch',
    maintainer_email='chao070503@gmail.com',
    description=(
        'Serial driver, feedback monitoring, and command tools '
        'for the six-axis arm.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_driver_node = arm_driver.arm_driver_node:main',
            'arm_command_node = arm_driver.arm_command_node:main',
            'arm_monitor_node = arm_driver.arm_monitor_node:main',
        ],
    },
)
