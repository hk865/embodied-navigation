"""Independent black-box acceptance of patch-only SCAN source staging."""
import hashlib
import importlib
import json
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from uuid import uuid4

from embodied_navigation.project import project_root
from scan_source_acceptance_fixtures import file_bytes, make_vendor, replacement_patch


class ScanSourceStagingAcceptance(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='scan-source-acceptance-')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.stage_source = importlib.import_module(
            'embodied_navigation.planning.scan_source').stage_scan_source

    def test_no_patch_copy_preserves_every_source_byte_and_returns_staged_vendor(self):
        vendor = make_vendor(self.root)
        original = file_bytes(vendor)
        build = self.root / 'fresh build'
        returned = self.stage_source(vendor, build)
        self.assertIsInstance(returned, Path)
        self.assertEqual(returned, build / 'source' / 'third_party' / 'scan')
        self.assertEqual(file_bytes(returned), original)
        self.assertEqual(file_bytes(vendor), original)
        self.assertFalse((build / 'install').exists())

    def test_patch_filename_order_and_repository_relative_paths(self):
        vendor = make_vendor(self.root)
        patches = vendor / 'patches'
        # Create in reverse order so filesystem enumeration is not the contract.
        (patches / '020-final.patch').write_bytes(replacement_patch(2, 3))
        (patches / '010-first.patch').write_bytes(replacement_patch(1, 2))
        (patches / 'not-a-patch.txt').write_text('This is not a patch.\n')
        original = file_bytes(vendor)
        build = self.root / 'ordered build'
        returned = self.stage_source(vendor, build)
        self.assertEqual((returned / 'src' / 'value.cpp').read_text().splitlines(), [
            '// context must survive staging', 'int value = 3;', '// trailing context'])
        for name, content in original.items():
            if name != 'src/value.cpp':
                self.assertEqual((returned / name).read_bytes(), content, name)
        self.assertEqual(file_bytes(vendor), original)
        self.assertFalse((build / 'install').exists())

    def test_lf_patches_apply_to_lf_and_crlf_sources_without_changing_original(self):
        for index, newline in enumerate((b'\n', b'\r\n')):
            with self.subTest(newline=newline):
                vendor = make_vendor(self.root / str(index), newline)
                (vendor / 'patches' / '010-change.patch').write_bytes(replacement_patch(1, 2))
                original = file_bytes(vendor)
                returned = self.stage_source(vendor, self.root / f'build-{index}')
                self.assertIn('int value = 2;', (returned / 'src' / 'value.cpp').read_text().splitlines())
                self.assertEqual((returned / 'README.txt').read_bytes(), original['README.txt'])
                self.assertEqual((returned / 'assets' / 'opaque.bin').read_bytes(), original['assets/opaque.bin'])
                self.assertEqual(file_bytes(vendor), original)

    def test_existing_empty_build_root_is_rejected_without_writes(self):
        vendor = make_vendor(self.root)
        build = self.root / 'existing build'
        build.mkdir()
        original = file_bytes(vendor)
        with self.assertRaises((ValueError, FileExistsError)):
            self.stage_source(vendor, build)
        self.assertEqual(list(build.iterdir()), [])
        self.assertEqual(file_bytes(vendor), original)

    def test_existing_build_artifacts_are_never_overwritten(self):
        vendor = make_vendor(self.root)
        build = self.root / 'existing build'
        build.mkdir()
        (build / 'keep.bin').write_bytes(b'old run evidence\x00\xff')
        before = file_bytes(build)
        with self.assertRaises((ValueError, FileExistsError)):
            self.stage_source(vendor, build)
        self.assertEqual(file_bytes(build), before)

    def test_build_root_inside_active_checkout_is_rejected_before_creation(self):
        vendor = make_vendor(self.root)
        build = project_root() / ('scan-stage-must-not-exist-' + uuid4().hex)
        self.assertFalse(build.exists())
        with self.assertRaises(ValueError):
            self.stage_source(vendor, build)
        self.assertFalse(build.exists())

    def test_build_root_inside_supplied_vendor_is_rejected_before_creation(self):
        vendor = make_vendor(self.root)
        for build in (vendor / 'build',):
            with self.subTest(build=build):
                self.assertFalse(build.exists())
                with self.assertRaises(ValueError):
                    self.stage_source(vendor, build)
                self.assertFalse(build.exists())

    def test_receipt_binds_source_patch_order_commands_and_staged_hashes(self):
        vendor = make_vendor(self.root)
        (vendor / 'patches' / '020-final.patch').write_bytes(replacement_patch(2, 3))
        (vendor / 'patches' / '010-first.patch').write_bytes(replacement_patch(1, 2))
        original = file_bytes(vendor)
        build = self.root / 'receipt build'
        with patch.object(subprocess, 'Popen', wraps=subprocess.Popen) as processes:
            staged = self.stage_source(vendor, build)
        receipt = json.loads((build / 'source_stage.json').read_text())
        self.assertIsInstance(receipt['status'], str)
        self.assertTrue(receipt['status'])
        self.assertEqual(receipt['patch_order'], 'filename')
        records = receipt['patches']
        names = ['010-first.patch', '020-final.patch']
        self.assertEqual([Path(record['path']).name for record in records], names)
        executed = [call.args[0] if call.args else call.kwargs['args']
                    for call in processes.call_args_list]
        for record, name in zip(records, names):
            self.assertEqual(record['sha256'], hashlib.sha256(original['patches/' + name]).hexdigest())
            self.assertEqual(record['returncode'], 0)
            self.assertTrue(record['command'])
            self.assertIn(record['command'], executed)
        for key, actual in (('source_files_sha256', original),
                            ('staged_files_sha256', file_bytes(staged))):
            hashes = receipt[key]
            if isinstance(hashes, dict):
                # Either vendor-relative or source-root-relative paths are
                # allowed; the actual file bytes must remain identifiable.
                for relative, contents in actual.items():
                    if relative.startswith('patches/'):
                        continue
                    matches = [digest for recorded_path, digest in hashes.items()
                               if recorded_path == relative or recorded_path.endswith('/' + relative)]
                    self.assertEqual(matches, [hashlib.sha256(contents).hexdigest()], relative)
            else:
                # Aggregate tree digest encoding was not fixed by the API.
                self.assertRegex(hashes, r'^[0-9a-f]{64}$')
        self.assertNotEqual(receipt['source_files_sha256'], receipt['staged_files_sha256'])
        self.assertEqual(file_bytes(vendor), original)

    def test_invalid_patch_preserves_staged_failure_scene_and_never_compiles(self):
        vendor = make_vendor(self.root)
        (vendor / 'patches' / '010-invalid.patch').write_bytes(replacement_patch(99, 100))
        original = file_bytes(vendor)
        build = self.root / 'failed build'
        with patch.object(subprocess, 'Popen', wraps=subprocess.Popen) as processes:
            with self.assertRaises((ValueError, RuntimeError, subprocess.CalledProcessError)):
                self.stage_source(vendor, build)
        staged = build / 'source' / 'third_party' / 'scan'
        self.assertTrue(staged.is_dir())
        self.assertEqual((staged / 'src' / 'value.cpp').read_bytes(), original['src/value.cpp'])
        self.assertEqual((staged / 'patches' / '010-invalid.patch').read_bytes(), original['patches/010-invalid.patch'])
        self.assertEqual(file_bytes(vendor), original)
        self.assertFalse((build / 'install').exists())
        receipt = json.loads((build / 'source_stage.json').read_text())
        self.assertEqual(receipt['patch_order'], 'filename')
        self.assertEqual(len(receipt['patches']), 1)
        failure = receipt['patches'][0]
        self.assertEqual(Path(failure['path']).name, '010-invalid.patch')
        self.assertEqual(failure['sha256'], hashlib.sha256(original['patches/010-invalid.patch']).hexdigest())
        self.assertNotEqual(failure['returncode'], 0)
        compilers = {'colcon', 'cmake', 'make', 'ninja', 'gcc', 'g++', 'clang', 'clang++', 'c++', 'cc'}
        for call in processes.call_args_list:
            command = call.args[0] if call.args else call.kwargs['args']
            tokens = shlex.split(command) if isinstance(command, str) else command
            self.assertFalse(compilers.intersection(Path(str(x)).name for x in tokens), command)


if __name__ == '__main__':
    unittest.main()
