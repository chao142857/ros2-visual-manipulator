#!/usr/bin/env python3
"""Create a reviewed software-only publication tree without copying Git history.

Only paths explicitly listed in publication_manifest.json are copied. Unknown
files, models, local captures and existing Git history are never auto-included.
Every source transformation is recorded without storing its original value.
"""

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
import tempfile

from audit import audit_tree, COMPILED, PROHIBITED_SUFFIXES, scan_text


HERE = Path(__file__).resolve().parent


def safe_relative(value):
    """Validate a manifest path without interpreting wildcard patterns."""
    path = Path(value)
    if path.is_absolute() or '..' in path.parts or '.git' in path.parts:
        raise ValueError(f'Unsafe manifest path: {value!r}')
    if any(char in value for char in '*?[]'):
        raise ValueError(f'Manifest must name one exact file: {value!r}')
    if path.suffix.lower() in PROHIBITED_SUFFIXES:
        raise ValueError(f'Unapproved model/archive in manifest: {value!r}')
    return path


def read_sources(source, entries):
    """Read every allowlisted source once and fail if anything is unavailable."""
    result = {}
    for entry in entries:
        path = safe_relative(entry)
        full = source / path
        if not full.is_file() or full.is_symlink():
            raise ValueError(f'Missing or symlinked manifest input: {entry}')
        if not full.resolve().is_relative_to(source):
            raise ValueError(f'Manifest path escapes source tree: {entry}')
        data = full.read_bytes()
        if b'\0' in data or len(data) > 5 * 1024 * 1024:
            raise ValueError(f'Binary/oversized file requires separate review: {entry}')
        result[entry] = data.decode('utf-8')
    return result


def verify_sources_unchanged(source, sources):
    """Reject mixed snapshots if another editor changes a source during export."""
    for name, original in sources.items():
        path = source / name
        if (not path.is_file() or path.is_symlink()
                or not path.resolve().is_relative_to(source)
                or path.read_bytes() != original.encode('utf-8')):
            raise ValueError(f'Source changed during export; retry after edits finish: {name}')


def synthetic_test_copy(name, text, profile):
    """Replace one reviewed measured fixture and omit its private-profile test."""
    if hashlib.sha256(text.encode()).hexdigest() != profile['source_sha256']:
        raise ValueError(f'Synthetic test transform requires its exact reviewed source: {name}')
    tree = ast.parse(text)
    replacement = '''def calibration():
    # Synthetic intrinsics for offline math tests; not a recorded device calibration.
    fx, fy, cx, cy = 400., 405., 320., 240.
    k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
    return SimpleNamespace(
        width=640, height=480, k=k, p=[fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.],
        r=np.eye(3).ravel().tolist(), distortion_model='plumb_bob',
        d=[-.05, .06, -.0005, -.00001, -.02])
'''
    lines = text.splitlines(keepends=True)
    edits, changes = [], []
    for node in tree.body:
        category, value = None, ''
        if isinstance(node, ast.FunctionDef) and node.name == 'calibration':
            category, value = 'recorded_intrinsics_to_synthetic_test_fixture', replacement
        elif (isinstance(node, ast.FunctionDef)
              and node.name == 'test_diagnostic_profile_changes_only_explicit_deprojection_mode'):
            category = 'omit_test_requiring_excluded_private_scene_profiles'
        elif isinstance(node, ast.ImportFrom) and node.module == 'pathlib':
            category = 'remove_unused_private_profile_test_import'
        elif isinstance(node, ast.Import) and [item.name for item in node.names] == ['yaml']:
            category = 'remove_unused_private_profile_test_import'
        if category:
            edits.append((node.lineno - 1, node.end_lineno, value))
            changes.append({
                'path': name, 'line': node.lineno, 'end_line': node.end_lineno,
                'category': category, 'occurrences': 1,
            })
    if len(edits) != 4:
        raise ValueError('Reviewed synthetic test transform did not match all four expected edits')
    for start, end, value in sorted(edits, reverse=True):
        lines[start:end] = [value]
    return ''.join(lines), changes


def transform_sources(sources, synthetic_profile=None):
    """Replace device bindings in copies; fail on other unreviewed personal data."""
    identifiers = set()
    for name, text in sources.items():
        lines = text.splitlines()
        for finding in scan_text(text.encode(), name):
            if finding['category'] == 'device_identifier':
                identifiers.update(COMPILED['device_identifier'].findall(
                    lines[finding['line'] - 1]
                ))
    # Real D405 identifiers have many digits. One-digit sentinels deliberately
    # cannot match those units. They also keep integer-based test fixtures valid.
    replacements = {value: str(index) for index, value in enumerate(sorted(identifiers))}
    changes, result = [], {}
    for name, text in sources.items():
        output = []
        for line_number, line in enumerate(text.splitlines(keepends=True), 1):
            for value, sentinel in replacements.items():
                pattern = re.compile(r'(?<!\d)' + re.escape(value) + r'(?!\d)')
                new_line, count = pattern.subn(sentinel, line)
                if count:
                    changes.append({
                        'path': name, 'line': line_number,
                        'category': 'device_identifier_to_nonmatching_sentinel',
                        'occurrences': count, 'replacement': sentinel,
                    })
                    line = new_line
            output.append(line)
        result[name] = ''.join(output)
    for name, text in result.items():
        if not name.endswith('.py'):
            continue
        tree = ast.parse(text)
        encoded_lines = text.encode('utf-8').splitlines(keepends=True)
        offsets = [0]
        for line in encoded_lines:
            offsets.append(offsets[-1] + len(line))
        edits = []
        for node in ast.walk(tree):
            if (isinstance(node, ast.Constant) and isinstance(node.value, str)
                    and re.fullmatch(
                        r'/dev/serial/by-id/usb-[A-Za-z0-9_.-]+', node.value
                    )):
                replacement = '/dev/serial/by-id/CONFIGURE_GRIPPER_PORT'
                edits.append((
                    offsets[node.lineno - 1] + node.col_offset,
                    offsets[node.end_lineno - 1] + node.end_col_offset,
                    repr(replacement).encode(),
                ))
                changes.append({
                    'path': name, 'line': node.lineno, 'end_line': node.end_lineno,
                    'category': 'device_path_to_explicit_configuration_placeholder',
                    'occurrences': 1, 'replacement': replacement,
                })
        data = text.encode('utf-8')
        for start, end, replacement in sorted(edits, reverse=True):
            data = data[:start] + replacement + data[end:]
        result[name] = data.decode('utf-8')
    if synthetic_profile:
        name = synthetic_profile['path']
        if name not in sources:
            raise ValueError(f'Reviewed synthetic test missing from allowlist: {name}')
        # Exact source hash is verified before any transformation of this file.
        result[name], test_changes = synthetic_test_copy(name, sources[name], synthetic_profile)
        changes.extend(test_changes)
    return result, changes


def apply_metadata_overrides(originals, texts, overrides):
    """Apply reviewed, hash-pinned metadata corrections only to exported copies."""
    changes = []
    for name, profile in overrides.items():
        if name not in originals or not name.endswith('/package.xml'):
            raise ValueError(f'Unreviewed metadata override path: {name}')
        if hashlib.sha256(originals[name].encode()).hexdigest() != profile['source_sha256']:
            raise ValueError(f'Metadata override requires its exact reviewed source: {name}')
        for edit in profile['replacements']:
            if texts[name].count(edit['before']) != 1:
                raise ValueError(f'Metadata override must match exactly once: {name}')
            texts[name] = texts[name].replace(edit['before'], edit['after'], 1)
            changes.append({
                'path': name, 'category': edit['category'], 'occurrences': 1,
                'scope': profile['scope'],
            })
    return changes


def write_publication(stage, source, manifest, originals, texts, changes):
    """Write text files and auditable inventory; never copy a directory wholesale."""
    for name, text in texts.items():
        path = stage / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')
        path.chmod((source / name).stat().st_mode & 0o777)
    for name in manifest['model_placeholders']:
        path = stage / safe_relative(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            '# Model assets excluded\n\n'
            'No model is supplied here. Rights for the local hardware designs '
            'have not been established. See the root THIRD_PARTY_NOTICES.md and '
            'models/README.md before supplying your own authorized model.\n',
            encoding='utf-8',
        )
    for name, description in manifest.get('package_readmes', {}).items():
        path = stage / safe_relative(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            f'# {path.parent.name}\n\n{description}\n\n'
            'The private historical package guide is intentionally omitted. '
            'Use the reviewed documentation at the repository root.\n',
            encoding='utf-8',
        )
    inventory = []
    for path in sorted(stage.rglob('*')):
        if path.is_file():
            relative = path.relative_to(stage).as_posix()
            inventory.append({
                'path': relative,
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'source_sha256': (
                    hashlib.sha256(originals[relative].encode('utf-8')).hexdigest()
                    if relative in originals else None
                ),
                'bytes': path.stat().st_size,
            })
    (stage / 'PUBLICATION_MANIFEST.json').write_text(json.dumps({
        'schema': 1, 'status': 'software-only review candidate',
        'history_copied': False, 'files': inventory,
        'excluded_categories': manifest['excluded_categories'],
        'inventory_note': 'Generated manifest/transformation reports are not self-hashed.',
    }, indent=2) + '\n', encoding='utf-8')
    (stage / 'PUBLICATION_TRANSFORMATIONS.json').write_text(json.dumps({
        'schema': 1, 'local_source_modified': False, 'changes': changes,
        'line_convention': 'Line numbers refer to the private source snapshot before export.',
        'behavior_note': (
            'Exported camera identity bindings/fixtures use nonmatching sentinels; '
            'the gripper USB identity path becomes an explicit configuration placeholder. '
            'Supply your real serial/port explicitly before hardware bringup. '
            'Actuator mappings, directions, limits, zero offsets and PWM ranges '
            'are not transformed. The pinned inverse-Brown test uses synthetic '
            'intrinsics and omits one test requiring excluded private scene profiles.'
        ),
    }, indent=2) + '\n', encoding='utf-8')


def main(argv=None):
    """Create a new candidate or archive a prior generated candidate on refresh."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--source', type=Path, default=HERE.parents[1])
    parser.add_argument('--manifest', type=Path, default=HERE / 'publication_manifest.json')
    parser.add_argument('--refresh', action='store_true')
    args = parser.parse_args(argv)
    source = args.source.resolve()
    destination = args.destination.resolve()
    if destination == source or destination.is_relative_to(source):
        parser.error('Publication destination must be outside the source workspace')
    if source.is_relative_to(destination):
        parser.error('Publication destination must not contain the source workspace')
    if destination.exists() and (
        not args.refresh or (destination / '.git').exists()
        or not (destination / 'PUBLICATION_MANIFEST.json').is_file()
    ):
        parser.error('Destination exists; refresh requires a generated candidate without .git')
    manifest_bytes = args.manifest.read_bytes()
    manifest = json.loads(manifest_bytes)
    if len(manifest['files']) != len(set(manifest['files'])):
        parser.error('Duplicate manifest paths')
    sources = read_sources(source, manifest['files'])
    for name, expected in manifest.get('reviewed_source_hashes', {}).items():
        if name not in sources or hashlib.sha256(sources[name].encode()).hexdigest() != expected:
            raise ValueError(f'Source differs from the last reviewed revision: {name}')
    texts, changes = transform_sources(sources, manifest.get('synthetic_test_transform'))
    changes.extend(apply_metadata_overrides(
        sources, texts, manifest.get('metadata_overrides', {})))
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix='.arm-publication-stage-', dir=destination.parent))
    try:
        write_publication(stage, source, manifest, sources, texts, changes)
        verify_sources_unchanged(source, sources)
        if args.manifest.read_bytes() != manifest_bytes:
            raise ValueError(
                'Publication allowlist changed during export; retry after edits finish'
            )
        result = audit_tree(stage, strict_public=True)
        if result['findings']:
            report_path = stage.parent / f'{stage.name}-audit.json'
            report_path.write_text(json.dumps(result, indent=2) + '\n')
            raise ValueError(
                f'Publication audit failed; redacted report: {report_path}. '
                'Source and prior candidate were not modified.'
            )
        verify_sources_unchanged(source, sources)
        if args.manifest.read_bytes() != manifest_bytes:
            raise ValueError(
                'Publication allowlist changed during export; retry after edits finish'
            )
        if destination.exists():
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
            backup = destination.parent / f'{destination.name}-previous-{stamp}'
            destination.rename(backup)
            print(f'Previous generated candidate retained at {backup}')
        stage.rename(destination)
    except BaseException:
        if stage.exists():
            shutil.rmtree(stage)
        raise
    print(json.dumps({
        'destination': str(destination), 'source_files': len(texts),
        'transformations': len(changes), 'audit_findings': 0,
        'git_history_copied': False,
    }, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
