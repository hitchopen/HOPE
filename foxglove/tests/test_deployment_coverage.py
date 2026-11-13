"""Offline regressions for omissions found during the old-P1 changeover."""
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('coverage_gate', ROOT / 'foxglove/a3/deployment_coverage.py')
gate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gate)


class DeploymentCoverageTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.evidence = self.root / 'readback.txt'
        self.evidence.write_text('synthetic reviewed fixture, not robot qualification\n')
        self.now = datetime(2026, 11, 13, 3, tzinfo=timezone.utc)
        self.inventory = json.loads(gate.INVENTORY.read_text())
        self.report = gate.template(self.inventory)
        self.report.update(release_commit='a' * 40, started_at=self.now.isoformat())
        for host in self.report['hosts'].values():
            host.update(machine_id='fixture-machine', boot_id='new-boot', before_boot_id='old-boot')
        for item in self.report['components'].values():
            item['source_paths'] = ['fixture/build-recipe.json']
            for stage, record in item['stages'].items():
                record.update(status='pass', captured_at=self.now.isoformat(),
                              machine_id='fixture-machine', boot_id='new-boot',
                              evidence=[{'path': self.evidence.name,
                                         'sha256': hashlib.sha256(self.evidence.read_bytes()).hexdigest()}])
                record['expected'] = {k: 'reviewed-fixture' for k in record['expected']}
                if stage == 'installed':
                    record['expected']['artifact_hashes'] = {'/fixture/binary': 'b' * 64}
                record['observed'] = copy.deepcopy(record['expected'])

    def check(self, level='persistent', **kwargs):
        return gate.validate(self.report, self.inventory, self.root, level, now=self.now,
                             resolve_source=kwargs.get('resolve_source', lambda *_: True))

    def test_complete_synthetic_coverage(self):
        self.assertEqual(self.check(), [])

    def test_model_catalog_alone_is_not_full_deployment(self):
        self.report['components'] = {'mdu_policy': self.report['components']['mdu_policy']}
        self.assertTrue(any('component set mismatch' in e for e in self.check()))

    def test_display_patch_missing_cannot_be_omitted(self):
        self.report['components']['hdu_display']['applicable'] = False
        self.report['components']['hdu_display']['reason'] = 'forgot this component'
        self.assertTrue(any('cannot omit mandatory' in e for e in self.check()))

    def test_old_ntp_source_and_disabled_bootstrap_fail(self):
        for component, fact, expected, observed in (
            ('laptop_clock', 'selected_hdu_source', 'current-hdu', 'previous-hdu'),
            ('hdu_clock', 'bootstrap_enabled', True, False),
            ('mdu_bsp_scheduler', 'vendor_hashes', {'aimrt': 'b'*64}, {'aimrt': 'c'*64}),
        ):
            with self.subTest(component=component):
                stage = self.report['components'][component]['stages']['running']
                stage['expected'][fact] = expected
                stage['observed'][fact] = observed
                self.assertTrue(any(f'{component}/running: expected/observed mismatch' in e for e in self.check()))

    def test_installed_does_not_imply_loaded(self):
        self.report['components']['laptop_console']['stages']['running']['status'] = 'not_checked'
        self.assertEqual(self.check('installed'), [])
        self.assertTrue(any('laptop_console/running: not passed' in e for e in self.check('running')))

    def test_no_reboot_and_stale_boot_evidence_fail(self):
        self.report['hosts']['hdu']['before_boot_id'] = 'new-boot'
        self.report['components']['mdu_runtime']['stages']['running']['boot_id'] = 'old-boot'
        errors = self.check()
        self.assertTrue(any('no distinct pre-reboot' in e for e in errors))
        self.assertTrue(any('wrong machine or stale boot' in e for e in errors))

    def test_other_machine_and_old_report_fail(self):
        self.report['components']['hdu_operations']['stages']['installed']['machine_id'] = 'other-robot'
        self.report['started_at'] = '2026-11-10T03:00:00+00:00'
        errors = self.check()
        self.assertTrue(any('wrong machine' in e for e in errors))
        self.assertTrue(any('24 hours' in e for e in errors))

    def test_temp_only_source_rejected(self):
        self.assertTrue(any('missing versioned source' in e for e in self.check(resolve_source=lambda *_: False)))

    def test_modified_attachment_rejected(self):
        self.evidence.write_text('changed after audit')
        self.assertTrue(any('invalid evidence attachment' in e for e in self.check()))

    def test_outside_attachment_rejected(self):
        record = self.report['components']['hdu_operations']['stages']['installed']
        record['evidence'][0]['path'] = '/etc/passwd'
        self.assertTrue(any('invalid evidence attachment' in e for e in self.check()))

    def test_blank_expectation_and_missing_evidence_rejected(self):
        record = self.report['components']['hdu_capture']['stages']['running']
        record['expected']['recording_role'] = None
        record['observed'] = copy.deepcopy(record['expected'])
        record['evidence'] = []
        errors = self.check()
        self.assertTrue(any('incomplete expected facts' in e for e in errors))
        self.assertTrue(any('no evidence attachments' in e for e in errors))

    def test_optional_scope_needs_reason(self):
        item = self.report['components']['optional_models']
        item['applicable'] = False
        self.assertTrue(self.check())
        item['reason'] = 'Only selected model14200best is in this release'
        self.assertEqual(self.check(), [])

    def test_artifact_hashes_cannot_be_a_pass_label(self):
        record = self.report['components']['mdu_policy']['stages']['installed']
        record['expected']['artifact_hashes'] = record['observed']['artifact_hashes'] = 'all match'
        self.assertTrue(any('absolute paths and full SHA256' in e for e in self.check()))

    def test_new_template_fails_closed(self):
        self.report = gate.template(self.inventory)
        self.assertTrue(self.check('installed'))


class HistoricalProfileTests(unittest.TestCase):
    def test_exact_historical_helpers_reconstruct_without_touching_checkout(self):
        profile = ROOT / 'foxglove/deployment_profiles/old_p1_20261110'
        manifest = json.loads((profile / 'provenance.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, hashes in manifest['files'].items():
                original = subprocess.check_output(['git', 'show', f"{manifest['base_commit']}:{name}"], cwd=ROOT)
                self.assertEqual(hashlib.sha256(original).hexdigest(), hashes['base_sha256'])
                target = root / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(original)
            subprocess.run(['git', 'apply', str(profile / 'historical.patch')], cwd=root, check=True)
            for name, hashes in manifest['files'].items():
                self.assertEqual(hashlib.sha256((root / name).read_bytes()).hexdigest(), hashes['historical_sha256'])
            subprocess.run(['bash', '-n', str(root / 'foxglove/helpers/hope-lifecycle')], check=True)
            compile((root / 'foxglove/helpers/hope_elink_scheduler.py').read_text(), 'historical_scheduler', 'exec')


class StagingComparisonTests(unittest.TestCase):
    def test_finds_outside_git_omissions_without_overwriting_current_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staging, checkout = root / 'staging', root / 'checkout'
            staging.mkdir()
            checkout.mkdir()
            for name, content in [('same.py', 'same'), ('changed.py', 'field'), ('missing.py', 'field-only')]:
                (staging / name).write_text(content)
            (checkout / 'same.py').write_text('same')
            (checkout / 'changed.py').write_text('newer')
            rows = gate.compare_staging(staging, checkout)
            self.assertEqual({r['path']: r['status'] for r in rows},
                             {'same.py': 'identical', 'changed.py': 'different', 'missing.py': 'missing'})
            self.assertEqual((checkout / 'changed.py').read_text(), 'newer')
            self.assertFalse((checkout / 'missing.py').exists())

    def test_empty_or_symlink_only_stage_never_qualifies_as_identical(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staging, checkout = root / 'staging', root / 'checkout'
            staging.mkdir()
            checkout.mkdir()
            with self.assertRaises(ValueError):
                gate.compare_staging(staging, checkout)
            (staging / 'external').symlink_to('/etc/passwd')
            self.assertEqual(gate.compare_staging(staging, checkout)[0]['status'], 'symlink_requires_review')


if __name__ == '__main__':
    unittest.main()
