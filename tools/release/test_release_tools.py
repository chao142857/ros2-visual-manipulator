#!/usr/bin/env python3
"""Offline regression tests for publication boundaries and redacted reporting."""

import contextlib
import ast
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import audit  # noqa: E402
import export_publication as exporter  # noqa: E402


class PublicationTests(unittest.TestCase):
    """Exercise the export boundary with independent temporary repositories."""

    def test_secret_report_never_contains_matched_value(self):
        secret = 'gh' + 'p_' + 'AbCdEf' * 8
        findings = audit.scan_text(f'value={secret}\n'.encode(), 'sample.txt')
        self.assertEqual(findings[0]['category'], 'known_token')
        self.assertNotIn(secret, json.dumps(findings))

    def test_private_paths_emails_and_keys_are_classified(self):
        private_path = '/ho' + 'me/someone/work'
        private_email = 'someone' + '@' + 'private.invalid'
        key_header = '-----BEGIN ' + 'OPENSSH PRIVATE KEY-----'
        findings = audit.scan_text(
            ('\n'.join([private_path, private_email, key_header])).encode(), 'sample'
        )
        self.assertEqual(
            {f['category'] for f in findings},
            {'personal_home_path', 'email', 'private_key'},
        )

    def test_explicit_placeholder_and_noreply_mailbox_are_allowed(self):
        self.assertTrue(audit.public_email('TODO@example.invalid'))
        self.assertTrue(audit.public_email('name@users.noreply.github.com'))

    def test_confirmed_public_contact_exemption_is_exact(self):
        confirmed = 'confirmed' + '@' + 'public.invalid'
        unapproved = 'other' + '@' + 'private.invalid'
        self.assertEqual(audit.scan_text(confirmed.encode(), 'contact', [confirmed]), [])
        self.assertEqual(
            audit.scan_text(unapproved.encode(), 'contact', [confirmed])[0]['category'],
            'email',
        )
        secret = 'gh' + 'p_' + 'ZaYxWv' * 8
        self.assertEqual(
            audit.scan_text(secret.encode(), 'contact', [confirmed])[0]['category'],
            'known_token',
        )

    def test_prohibited_manifest_paths_are_rejected(self):
        for value in ('../secret', '/absolute', 'src/*.py', '.git/config', 'robot.stl'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                exporter.safe_relative(value)

    def test_serial_sentinel_keeps_python_integer_fixture_valid(self):
        serial = '123456' + '789012'
        source = f'serial_no = "{serial}"\nfixture = {serial}\n'
        transformed, changes = exporter.transform_sources({'sample.py': source})
        namespace = {}
        exec(compile(transformed['sample.py'], 'sample.py', 'exec'), namespace)
        self.assertEqual(namespace['serial_no'], '0')
        self.assertEqual(namespace['fixture'], 0)
        self.assertEqual(len(changes), 2)
        self.assertNotIn(serial, json.dumps(changes))
        self.assertEqual(source.count(serial), 2)

    def test_split_usb_identity_is_detected_and_exported_as_placeholder(self):
        private_suffix = 'usb-' + 'Vendor_FixtureSerial-if00'
        source = 'port = ("/dev/serial/by-id/"\n        "' + private_suffix + '")\n'
        findings = audit.scan_text(source.encode(), 'sample.py')
        self.assertTrue(any(f['category'] == 'device_path_identifier' for f in findings))
        transformed, changes = exporter.transform_sources({'sample.py': source})
        namespace = {}
        exec(compile(transformed['sample.py'], 'sample.py', 'exec'), namespace)
        self.assertEqual(namespace['port'], '/dev/serial/by-id/CONFIGURE_GRIPPER_PORT')
        self.assertNotIn(private_suffix, json.dumps(changes))
        self.assertEqual(audit.scan_text(transformed['sample.py'].encode(), 'sample.py'), [])

    def test_export_keeps_scanner_patterns_unchanged(self):
        sources = {path.name: path.read_text() for path in HERE.glob('*.py')}
        transformed, changes = exporter.transform_sources(sources)
        self.assertEqual(transformed, sources)
        self.assertEqual(changes, [])

    def test_pinned_test_copy_uses_synthetic_intrinsics_without_private_profiles(self):
        source = (
            'from pathlib import Path\nimport yaml\n'
            'def calibration():\n    return "private fixture"\n'
            'def test_diagnostic_profile_changes_only_explicit_deprojection_mode():\n'
            '    return Path("private-profile.yaml")\n'
        )
        profile = {'source_sha256': hashlib.sha256(source.encode()).hexdigest()}
        result, changes = exporter.synthetic_test_copy('fixture.py', source, profile)
        tree = ast.parse(result)
        self.assertEqual([node.name for node in tree.body], ['calibration'])
        namespace = {
            'SimpleNamespace': SimpleNamespace,
            'np': SimpleNamespace(eye=lambda _: SimpleNamespace(
                ravel=lambda: SimpleNamespace(tolist=lambda: [1, 0, 0, 0, 1, 0, 0, 0, 1])
            )),
        }
        exec(compile(tree, 'fixture.py', 'exec'), namespace)
        fixture = namespace['calibration']()
        self.assertEqual(fixture.k, [400, 0, 320, 0, 405, 240, 0, 0, 1])
        self.assertEqual(fixture.d, [-.05, .06, -.0005, -.00001, -.02])
        self.assertEqual(len(changes), 4)
        self.assertNotIn('private-profile.yaml', result)
        with self.assertRaises(ValueError):
            exporter.synthetic_test_copy('fixture.py', source + '\n', profile)

    def test_upstream_email_exemption_requires_exact_notice_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            notice = root / 'third_party/NOTICE'
            notice.parent.mkdir()
            notice.write_text('Copyright upstream' + '@' + 'external.invalid\n')
            manifest = root / 'tools/release/publication_manifest.json'
            manifest.parent.mkdir(parents=True)
            manifest.write_text(json.dumps({'approved_third_party_notices': {
                'third_party/NOTICE': hashlib.sha256(notice.read_bytes()).hexdigest(),
            }}))
            self.assertEqual(audit.audit_tree(root, True)['findings'], [])
            notice.write_text(notice.read_text() + 'modified\n')
            self.assertTrue(any(
                item['category'] == 'email' for item in audit.audit_tree(root, True)['findings']
            ))

    def test_metadata_override_is_copy_only_and_requires_reviewed_source(self):
        name = 'src/description/package.xml'
        original = '<license>unreviewed local model scope</license>\n'
        sources = {name: original}
        profile = {name: {
            'source_sha256': hashlib.sha256(original.encode()).hexdigest(),
            'scope': 'exported original software only; no model assets',
            'replacements': [{
                'before': original.strip(), 'after': '<license>Apache-2.0</license>',
                'category': 'source_only_package_license_declaration',
            }],
        }}
        copies = sources.copy()
        changes = exporter.apply_metadata_overrides(sources, copies, profile)
        self.assertEqual(sources[name], original)
        self.assertEqual(copies[name], '<license>Apache-2.0</license>\n')
        self.assertEqual(len(changes), 1)
        with self.assertRaisesRegex(ValueError, 'exact reviewed source'):
            exporter.apply_metadata_overrides({name: original + '\n'}, copies, profile)
        with self.assertRaisesRegex(ValueError, 'exactly once'):
            exporter.apply_metadata_overrides(sources, copies, profile)

    def test_cad_and_unconfirmed_scene_tests_are_not_public_payload(self):
        manifest = json.loads((HERE / 'publication_manifest.json').read_text())
        excluded = {
            'src/tabletop_grasp/test/test_gripper_motion.py',
            'src/tabletop_grasp/test/test_cad_config.py',
            'src/dual_camera_foundation/test/test_locked_j6_ik.py',
        }
        self.assertTrue(excluded.isdisjoint(manifest['files']))
        self.assertTrue(excluded.issubset(manifest['excluded_test_files']))

    def test_current_tree_includes_ignored_secrets(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / '.gitignore').write_text('.env\n')
            (root / '.env').write_text('password=' + '"temporary-secret"\n')
            result = audit.audit_tree(root)
            self.assertTrue(any(
                f['category'] == 'credential_assignment' for f in result['findings']
            ))

    def test_history_catches_deleted_secret(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            subprocess.run(['git', 'init', '-q', str(root)], check=True)
            subprocess.run(['git', '-C', str(root), 'config', 'user.name', 'Fixture'], check=True)
            subprocess.run([
                'git', '-C', str(root), 'config', 'user.email', 'fixture@example.invalid',
            ], check=True)
            secret = 'gh' + 'p_' + 'ZaYxWv' * 8
            (root / 'removed.txt').write_text(secret)
            subprocess.run(['git', '-C', str(root), 'add', '.'], check=True)
            subprocess.run(['git', '-C', str(root), 'commit', '-qm', 'fixture'], check=True)
            subprocess.run(['git', '-C', str(root), 'rm', '-q', 'removed.txt'], check=True)
            subprocess.run(['git', '-C', str(root), 'commit', '-qm', 'remove fixture'], check=True)
            report = audit.audit_history(root)
            self.assertTrue(any(f['category'] == 'known_token' for f in report['findings']))
            self.assertNotIn(secret, json.dumps(report))

    def test_export_is_allowlisted_does_not_copy_history_and_refresh_retains_old(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source'
            source.mkdir()
            (source / 'allowed.py').write_text('value = 3\n')
            (source / 'unreviewed.txt').write_text('private material')
            (source / '.git').mkdir()
            manifest = root / 'manifest.json'
            manifest.write_text(json.dumps({
                'files': ['allowed.py'],
                'model_placeholders': ['models/README.md'],
                'excluded_categories': ['models', 'private_history'],
            }))
            destination = root / 'publication'
            arguments = [str(destination), '--source', str(source), '--manifest', str(manifest)]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(exporter.main(arguments), 0)
            self.assertFalse((destination / 'unreviewed.txt').exists())
            self.assertFalse((destination / '.git').exists())
            self.assertEqual((source / 'allowed.py').read_text(), 'value = 3\n')
            self.assertTrue((destination / 'models/README.md').is_file())
            (source / 'allowed.py').write_text('value = 4\n')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(exporter.main(arguments + ['--refresh']), 0)
            backups = list(root.glob('publication-previous-*'))
            self.assertEqual(len(backups), 1)
            self.assertEqual((backups[0] / 'allowed.py').read_text(), 'value = 3\n')
            self.assertEqual((destination / 'allowed.py').read_text(), 'value = 4\n')

    def test_failed_export_preserves_source_and_existing_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source'
            source.mkdir()
            private = 'someone' + '@' + 'private.invalid'
            (source / 'source.py').write_text(f'email = "{private}"\n')
            manifest = root / 'manifest.json'
            manifest.write_text(json.dumps({
                'files': ['source.py'], 'model_placeholders': [], 'excluded_categories': [],
            }))
            destination = root / 'publication'
            destination.mkdir()
            (destination / 'PUBLICATION_MANIFEST.json').write_text('{}')
            (destination / 'keep.txt').write_text('previous candidate')
            with self.assertRaises(ValueError):
                exporter.main([
                    str(destination), '--source', str(source),
                    '--manifest', str(manifest), '--refresh',
                ])
            self.assertEqual((destination / 'keep.txt').read_text(), 'previous candidate')
            self.assertIn(private, (source / 'source.py').read_text())
            self.assertFalse([
                path for path in root.glob('.arm-publication-stage-*') if path.is_dir()
            ])

    def test_concurrent_edit_rejects_mixed_source_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / 'source'
            source.mkdir()
            (source / 'source.py').write_text('before = True\n')
            manifest = root / 'manifest.json'
            manifest.write_text(json.dumps({
                'files': ['source.py'], 'model_placeholders': [], 'excluded_categories': [],
            }))
            write = exporter.write_publication

            def concurrent_write(*args):
                write(*args)
                (source / 'source.py').write_text('after = True\n')

            with patch.object(exporter, 'write_publication', concurrent_write):
                with self.assertRaisesRegex(ValueError, 'Source changed during export'):
                    exporter.main([
                        str(root / 'publication'), '--source', str(source),
                        '--manifest', str(manifest),
                    ])
            self.assertFalse((root / 'publication').exists())
            self.assertEqual((source / 'source.py').read_text(), 'after = True\n')


if __name__ == '__main__':
    unittest.main()
