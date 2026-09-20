from setuptools import find_packages, setup

package_name = 'gripper_driver'

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
    description='Serial PWM gripper driver and explicitly invoked command tool.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gripper_driver_node = gripper_driver.gripper_driver_node:main',
            'gripper_command_node = gripper_driver.gripper_command_node:main',
        ],
    },
)
