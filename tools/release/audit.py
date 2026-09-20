#!/usr/bin/env python3
"""Audit a source tree and optional Git history without printing secret values.

This is a small, reviewable release gate, not a complete secret-detection engine.
It never opens a serial device, starts ROS, changes Git, or uploads data.
"""

import argparse
import ast
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


GENERATED = {
    '.git', '__pycache__', '.pytest_cache', '.mypy_cache', '.ruff_cache',
    '.venv', '.venv-tabletop', 'build', 'install', 'log',
    'build-tabletop', 'install-tabletop', 'log-tabletop',
}
RULES = {
    'private_key': r'-----BEGIN (?:RSA |EC |DSA |OPENSSH )?PRIVATE KEY-----',
    'known_token': (
        r'(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}'
        r'|AKIA[0-9A-Z]{16}|xox[baprs]-[A-Za-z0-9-]{10,}'
        r'|sk-(?:proj-)?[A-Za-z0-9_-]{24,})'
    ),
    'credential_assignment': (
        r'''(?i)\b(?:password|passwd|api[_-]?key|client[_-]?secret|'''
        r'''access[_-]?token|auth[_-]?token)\s*[=:]\s*['"][^'"\n]{4,}['"]'''
    ),
    'private_ipv4': (
        r'\b(?:10\.(?:\d{1,3}\.){2}\d{1,3}'
        r'|192\.168\.\d{1,3}\.\d{1,3}'
        r'|172\.(?:1[6-9]|2\d|3[01])\.\d{1,3}\.\d{1,3})\b'
    ),
    'personal_home_path': (
        r'''(?:/ho''' r'''me/[^/\s"'<>]+|/Us''' r'''ers/[^/\s"'<>]+)'''
    ),
    'email': r'[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}',
    'device_identifier': r'(?<![\d.eE])\b\d{12,16}\b(?![\d.eE])',
    'ssh_public_key': r'\bssh-(?:rsa|ed25519)\s+[A-Za-z0-9+/]{30,}',
}
COMPILED = {name: re.compile(pattern) for name, pattern in RULES.items()}
SECRET_CATEGORIES = {'private_key', 'known_token', 'credential_assignment'}
PROHIBITED_SUFFIXES = {
    '.stl', '.step', '.stp', '.brep', '.iges', '.igs', '.urdf', '.xacro',
    '.srdf', '.db3', '.mcap', '.bag', '.zip', '.gz', '.tar', '.7z',
}


def public_email(value):
    """Allow GitHub privacy addresses and explicitly fictional examples only."""
    return value.endswith((
        '@users.noreply.github.com', '@example.com', '@example.invalid',
    ))


def scan_text(data, location, approved_emails=()):
    """Return locations and categories, never matched text."""
    if b'\0' in data[:8192]:
        return []
    findings = []
    decoded = data.decode('utf-8', errors='replace')
    for number, line in enumerate(decoded.splitlines(), 1):
        for category, pattern in COMPILED.items():
            matches = list(pattern.finditer(line))
            if category == 'email':
                matches = [m for m in matches
                           if not public_email(m.group()) and m.group() not in approved_emails]
            if category == 'device_identifier':
                # CAD decimals/hashes are not camera serial numbers. Match only
                # identity-related lines; test data is reviewed by the same gate.
                identity_line = re.search(
                    r'''(?i)serial|asic|firmware.update|d405|_\d{12}|['"]\d{12}['"]''', line
                )
                if not identity_line:
                    matches = []
            if matches:
                findings.append({
                    'path': location, 'line': number, 'category': category,
                })
        if re.search(r'/dev/serial/by-id/usb-[A-Za-z0-9_.-]+', line):
            findings.append({
                'path': location, 'line': number, 'category': 'device_path_identifier',
            })
    # Adjacent Python string literals can split a USB identity across lines.
    # AST constants recover their value without executing the module.
    try:
        tree = ast.parse(decoded)
    except (SyntaxError, ValueError):
        tree = None
    if tree is not None:
        for node in ast.walk(tree):
            if (isinstance(node, ast.Constant) and isinstance(node.value, str)
                    and re.fullmatch(
                        r'/dev/serial/by-id/usb-[A-Za-z0-9_.-]+', node.value
                    )):
                finding = {
                    'path': location, 'line': node.lineno,
                    'category': 'device_path_identifier',
                }
                if finding not in findings:
                    findings.append(finding)
    return findings


def source_files(root):
    """Walk all non-generated current files, including ignored local material."""
    for directory, children, names in os.walk(root, followlinks=False):
        children[:] = sorted(
            child for child in children
            if child not in GENERATED and not child.startswith('.venv-')
            and not child.startswith(('build-release', 'install-release', 'log-release'))
        )
        for name in sorted(names):
            yield Path(directory) / name


def audit_tree(root, strict_public=False):
    """Scan text and inventory every binary/large file without unpacking it."""
    findings, binaries, large_files = [], [], []
    approved_notices = {}
    approved_emails = []
    manifest_path = root / 'tools/release/publication_manifest.json'
    if manifest_path.is_file() and not manifest_path.is_symlink():
        try:
            manifest = json.loads(manifest_path.read_text())
            approved_notices = manifest.get('approved_third_party_notices', {})
            # Exact public contact explicitly authorized by the maintainer.
            # This does not exempt other addresses or any credential patterns.
            approved_emails = manifest.get('approved_public_emails', [])
        except (ValueError, OSError):
            pass
    count = 0
    for path in source_files(root):
        relative = path.relative_to(root).as_posix()
        count += 1
        if path.is_symlink():
            findings.append({'path': relative, 'line': 0, 'category': 'symlink'})
            continue
        data = path.read_bytes()
        if b'\0' in data[:8192]:
            binaries.append({
                'path': relative, 'bytes': len(data),
                'sha256': hashlib.sha256(data).hexdigest(),
            })
        else:
            text_findings = scan_text(data, relative, approved_emails)
            if (relative.startswith('third_party/')
                    and approved_notices.get(relative) == hashlib.sha256(data).hexdigest()):
                # Preserve licensed upstream copyright/contact notices verbatim.
                # Only email findings are exempt; credential rules still apply.
                text_findings = [item for item in text_findings if item['category'] != 'email']
            findings.extend(text_findings)
        if len(data) > 10 * 1024 * 1024:
            large_files.append({'path': relative, 'bytes': len(data)})
        if strict_public and (
            path.suffix.lower() in PROHIBITED_SUFFIXES
            or relative.startswith(('artifacts/', '.chao_restore_', 'd405_id3_'))
            or path.name.startswith('._')
        ):
            findings.append({
                'path': relative, 'line': 0, 'category': 'unapproved_asset_or_record',
            })
    return {
        'files': count, 'findings': findings,
        'binary_metadata': binaries, 'large_files': large_files,
    }


def git_output(root, *args):
    """Run a read-only Git query."""
    return subprocess.check_output(['git', '-C', str(root), *args])


def audit_history(root):
    """Scan all reachable blob objects and commit/tag identities and messages."""
    findings = []
    counts = Counter()
    for record in git_output(root, 'rev-list', '--objects', '--all').splitlines():
        object_id = record.split(b' ', 1)[0].decode('ascii')
        kind = git_output(root, 'cat-file', '-t', object_id).decode().strip()
        counts[kind] += 1
        if kind not in ('blob', 'commit', 'tag'):
            continue
        data = git_output(root, 'cat-file', '-p', object_id)
        # Use only the object id: historic filenames can themselves be private.
        findings.extend(scan_text(data, f'git:{object_id}'))
    return {'objects': dict(counts), 'findings': findings}


def main(argv=None):
    """Print a redacted report and fail on secrets or strict-public findings."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', nargs='?', type=Path, default=Path('.'))
    parser.add_argument('--history', action='store_true')
    parser.add_argument('--strict-public', action='store_true')
    parser.add_argument('--json', type=Path, help='Write the redacted full report here')
    args = parser.parse_args(argv)
    root = args.root.resolve()
    report = {'schema': 1, 'tree': audit_tree(root, args.strict_public)}
    if args.history:
        report['history'] = audit_history(root)
    findings = report['tree']['findings'] + report.get('history', {}).get('findings', [])
    categories = dict(Counter(item['category'] for item in findings))
    report['categories'] = categories
    report['scope_note'] = (
        'Pattern scan, not a guarantee. Archives and binary payloads are inventoried '
        'but not decoded. Common generated directories are excluded; custom output '
        'names may still be scanned. History includes '
        'reachable refs only; reflogs/unreachable objects and external repositories '
        'are not public candidates.'
    )
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({
        'files': report['tree']['files'], 'categories': categories,
        'binary_files': len(report['tree']['binary_metadata']),
        'large_files': len(report['tree']['large_files']),
        'history_objects': report.get('history', {}).get('objects', {}),
    }, indent=2))
    for finding in findings:
        print(f"{finding['category']}: {finding['path']}:{finding['line']}")
    failed = bool(findings) if args.strict_public else any(
        item['category'] in SECRET_CATEGORIES for item in findings
    )
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
