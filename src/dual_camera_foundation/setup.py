from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'dual_camera_foundation'


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
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'docs'),
            glob('docs/*.md'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bbabbr123-sketch',
    maintainer_email='chao070503@gmail.com',
    description=(
        'Dual-camera perception, D405 RGB-D targeting, and hand-eye tools.'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'd405_apriltag_pose = '
            'dual_camera_foundation.d405_apriltag_pose:main',
            'rgbd_target_point = '
            'dual_camera_foundation.rgbd_target_point:main',
            'target_pose_transform = '
            'dual_camera_foundation.target_pose_transform:main',
            'dual_camera_refiner = '
            'dual_camera_foundation.dual_camera_refiner:main',
            'dry_run_grasp_targets = '
            'dual_camera_foundation.dry_run_grasp_targets:main',
            'hand_eye_capture = '
            'dual_camera_foundation.hand_eye_capture:main',
            'hand_eye_calibration = '
            'dual_camera_foundation.hand_eye_calibration:main',
            'hand_eye_validate = '
            'dual_camera_foundation.hand_eye_calibration:validate_main',
            'mount_calibration = '
            'dual_camera_foundation.mount_calibration:main',
            'calibrated_mount_tf_publisher = '
            'dual_camera_foundation.mount_tf_publisher:main',
            'visual_ik = dual_camera_foundation.visual_ik_node:main',
            'visual_grasp = '
            'tabletop_grasp.controller:main',
            'astra_click_grasp_validator = '
            'dual_camera_foundation.runtime_validator:main',
        ],
    },
)
