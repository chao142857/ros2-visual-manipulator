from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

setup(
    url='https://github.com/chao142857/ros2-visual-manipulator',
    name='tabletop_grasp', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/tabletop_grasp']),
        ('share/tabletop_grasp', ['package.xml', 'README.md']),
        ('share/tabletop_grasp/config', glob('config/*')),
        ('share/tabletop_grasp/launch', glob('launch/*.launch.py')),
        ('share/tabletop_grasp/docs', glob('docs/*.md')),
        ('share/tabletop_grasp/scripts', glob('scripts/*.py') + glob('scripts/*.html')),
        ('share/tabletop_grasp/meshes', [path for path in glob('meshes/*')
                                       if Path(path).is_file()]),
    ] + [
        (str(Path('share/tabletop_grasp') / directory),
         [str(path) for path in Path(directory).iterdir() if path.is_file()])
        for directory in [Path('meshes/gripper_motion')]
        + [path for path in Path('meshes/gripper_motion').rglob('*') if path.is_dir()]
        if directory.is_dir()
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='bbabbr123-sketch', maintainer_email='chao070503@gmail.com',
    description='RGB-D tabletop geometry and collision-aware planning only.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'perception = tabletop_grasp.perception_node:main',
        'planner = tabletop_grasp.controller:main',
        'extrinsics = tabletop_grasp.extrinsics:main',
        'synthetic_scene = tabletop_grasp.synthetic_scene:main',
        'cad_preview = tabletop_grasp.cad_preview:main',
        'gripper_motion = tabletop_grasp.gripper_motion:main',
        'gripper_motion_preview = tabletop_grasp.gripper_motion_preview:main',
    ]},
)
