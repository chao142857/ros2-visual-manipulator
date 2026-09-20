#!/usr/bin/env python3
"""Check source syntax, ROS manifests, and package asset references offline."""

import argparse
import ast
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml


IMPORT_DEPENDENCIES = {
    'serial': 'python3-serial', 'cv2': 'python3-opencv',
    'numpy': 'python3-numpy', 'scipy': 'python3-scipy',
    'yaml': 'python3-yaml', 'open3d': 'python3-open3d',
    'urdf_parser_py': 'urdfdom_py',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--software-only', action='store_true',
                        help='Report unavailable model assets as pending, without a full pass.')
    parser.add_argument('--report', type=Path, help='Optional JSON evidence output.')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    errors, pending, notes = [], [], []
    counts = {'python': 0, 'shell': 0, 'yaml': 0, 'xml': 0, 'packages': 0}
    packages = {p.parent.name: p.parent for p in (root / 'src').glob('*/package.xml')}
    trees = {}

    def label(path):
        return str(path.relative_to(root))

    def asset_exists(value, owner):
        if not value.startswith('package://'):
            return
        package, separator, relative = value[len('package://'):].partition('/')
        if not separator or package not in packages:
            errors.append(f'{label(owner)}: unresolved package asset {value}')
        elif not (packages[package] / relative).is_file():
            pending.append(f'{label(owner)}: missing asset {value}')

    paths = sorted((root / 'src').rglob('*')) + sorted((root / 'scripts').rglob('*'))
    paths += sorted((root / 'tools').rglob('*'))
    paths += sorted((root / 'config').rglob('*'))
    paths += sorted((root / '.github').rglob('*'))
    paths += sorted(root.glob('*.sh'))
    for path in paths:
        if not path.is_file() or any(
            part.startswith('.') and part != '.github'
            for part in path.relative_to(root).parts
        ):
            continue
        try:
            if path.suffix == '.py':
                trees[path] = ast.parse(path.read_text(encoding='utf-8'), filename=label(path))
                counts['python'] += 1
            elif path.suffix == '.sh':
                subprocess.run(['bash', '-n', str(path)], check=True,
                               capture_output=True, text=True)
                counts['shell'] += 1
            elif path.suffix in ('.yaml', '.yml'):
                document = yaml.safe_load(path.read_text(encoding='utf-8'))
                counts['yaml'] += 1

                def walk(value):
                    if isinstance(value, dict):
                        for child in value.values():
                            walk(child)
                    elif isinstance(value, list):
                        for child in value:
                            walk(child)
                    elif isinstance(value, str):
                        asset_exists(value, path)
                walk(document)
            elif path.suffix in ('.xml', '.urdf', '.srdf', '.xacro'):
                document = ET.parse(path).getroot()
                counts['xml'] += 1
                for element in document.iter():
                    for value in element.attrib.values():
                        asset_exists(value, path)
        except (SyntaxError, UnicodeError, ET.ParseError, yaml.YAMLError,
                subprocess.CalledProcessError) as error:
            errors.append(f'{label(path)}: {error}')

    for name, directory in sorted(packages.items()):
        try:
            manifest = ET.parse(directory / 'package.xml').getroot()
        except ET.ParseError:
            continue  # Already reported by the XML syntax pass above.
        counts['packages'] += 1
        if manifest.findtext('name') != name:
            errors.append(f'{name}: manifest name differs from directory')
        dependencies = {element.text for element in manifest if 'depend' in element.tag}
        for path, tree in trees.items():
            if path.parent != directory / name:
                continue
            for node in ast.walk(tree):
                imported = []
                if isinstance(node, ast.Import):
                    imported = [alias.name.split('.')[0] for alias in node.names]
                elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                    imported = [node.module.split('.')[0]]
                for module in imported:
                    if module in sys.stdlib_module_names or module == name:
                        continue
                    dependency = IMPORT_DEPENDENCIES.get(module, module)
                    if dependency not in dependencies:
                        errors.append(f'{label(path)}: undeclared runtime dependency {dependency}')
        setup = directory / 'setup.py'
        if setup in trees:
            if not (directory / 'resource' / name).is_file():
                errors.append(f'{name}: ament resource marker is missing')
            for node in ast.walk(trees[setup]):
                if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
                    continue
                match = re.fullmatch(r'([\w-]+)\s*=\s*([\w.]+):([\w]+)', node.value)
                if match:
                    _, module, function = match.groups()
                    top, *tail = module.split('.')
                    target = (packages.get(top, root / 'missing')
                              / Path(top, *tail).with_suffix('.py'))
                    target_tree = trees.get(target)
                    if target_tree is None or not any(
                        isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
                        and item.name == function
                        for item in target_tree.body
                    ):
                        errors.append(f'{name}: missing entry point target {module}:{function}')
        cmake = directory / 'CMakeLists.txt'
        if cmake.is_file():
            for value in re.findall(r'(?m)^\s+((?:urdf|launch|tools)/[^\s()]+)', cmake.read_text()):
                if not (directory / value).is_file():
                    pending.append(f'{label(cmake)}: missing configured input {value}')

    # These resources are directly read by the launch files and runtime model loaders.
    for relative in (
        'arm_description/urdf/dummy-v2.urdf',
        'tabletop_grasp/config/dummy_v2.srdf',
        'tabletop_grasp/config/cad_collision_proxy.json',
        'tabletop_grasp/config/gripper_motion_estimate.json',
        'tabletop_grasp/config/gripper_motion_parts.json',
        'tabletop_grasp/config/tabletop.yaml',
        'wrist_camera_bringup/config/d405_mount_calibration.yaml',
        'camera_perception/config/camera_extrinsics.yaml',
    ):
        if not (root / 'src' / relative).is_file():
            pending.append(f'src/{relative}: model-dependent features unavailable')
    errors, pending = sorted(set(errors)), sorted(set(pending))
    if args.software_only:
        notes.append('Software-only scope: model/MoveIt/CAD launch checks are not complete.')
    report = {'counts': counts, 'errors': errors, 'pending_assets': pending, 'notes': notes,
              'full_structure_ready': not errors and not pending}
    print(json.dumps(report, indent=2))
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return 1 if errors else (2 if pending and not args.software_only else 0)


if __name__ == '__main__':
    raise SystemExit(main())
