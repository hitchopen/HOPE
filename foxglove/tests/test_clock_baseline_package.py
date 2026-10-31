from pathlib import Path
import hashlib
import stat
import subprocess
import unittest

FOXGLOVE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FOXGLOVE_DIR.parent

class ClockBaselinePackageTests(unittest.TestCase):
    def test_hdu_clock_baseline_package_and_installer_are_complete(self):
        package = REPO_ROOT / "agibot/ntp_sync"
        manifest = package / "MANIFEST.sha256"
        installer = FOXGLOVE_DIR / "a3/install_hdu_clock_baseline.sh"
        runbook = (
            REPO_ROOT / "docs/operations/clock_baseline_installation.md"
        ).read_text()

        self.assertEqual(
            hashlib.sha256(manifest.read_bytes()).hexdigest(),
            "92fa52faef8a89608be2c90bad3a4250803c624c61d87d96f127b3ed3fdc2168",
        )
        entries = {}
        for line in manifest.read_text().splitlines():
            digest, relative_path = line.split(maxsplit=1)
            self.assertRegex(digest, r"^[0-9a-f]{64}$")
            self.assertFalse(Path(relative_path).is_absolute())
            self.assertNotIn("..", Path(relative_path).parts)
            self.assertNotIn(relative_path, entries)
            entries[relative_path] = digest

        packaged_files = {
            path.relative_to(package).as_posix()
            for path in package.rglob("*")
            if path.is_file() and path != manifest
        }
        self.assertEqual(set(entries), packaged_files)
        self.assertEqual(len(entries), 14)
        for relative_path, expected_digest in entries.items():
            self.assertEqual(
                hashlib.sha256((package / relative_path).read_bytes()).hexdigest(),
                expected_digest,
            )

        installer_text = installer.read_text()
        self.assertTrue(installer.stat().st_mode & stat.S_IXUSR)
        self.assertIn(
            "92fa52faef8a89608be2c90bad3a4250803c624c61d87d96f127b3ed3fdc2168",
            installer_text,
        )
        self.assertIn(
            "80c35a34fef2f7b7abd845e09c7c3770dd9a863ed68e9caa71f6f4161afccc5b",
            installer_text,
        )
        self.assertIn(
            "9e6caeee749773437f36c43054594a3a4cb45e3963f88eadb81d79d77189c47a",
            installer_text,
        )
        for marker in (
            "INSTALL_HDU_CLOCK_BASELINE",
            "HDU_CLOCK_PACKAGE_VERIFY_OK",
            "HDU_CLOCK_BASELINE_ROLLED_BACK",
            "HDU_CLOCK_BASELINE_ROLLBACK_INCOMPLETE",
            "HDU_CLOCK_BASELINE_INSTALLED_OK",
        ):
            self.assertIn(marker, installer_text)
        self.assertNotIn("rm -rf", installer_text)
        self.assertLess(
            installer_text.index("MUTATION_STARTED=1"),
            installer_text.index("apt-get install"),
        )
        installed_unit_verify = (
            "systemd-analyze verify \\\n"
            "    /etc/systemd/system/agibot-clock-bootstrap.service"
        )
        self.assertEqual(installer_text.count(installed_unit_verify), 1)
        self.assertNotIn(
            'systemd-analyze verify \\\n'
            '    "$PKG_ROOT/etc/systemd/system/agibot-clock-bootstrap.service"',
            installer_text,
        )
        self.assertLess(
            installer_text.index(
                'install -D -o root -g root -m 0755 "$PKG_ROOT/$REL" "/$REL"'
            ),
            installer_text.index(installed_unit_verify),
        )

        result = subprocess.run(
            [str(installer), "--verify-package-only", str(package)],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("HDU_CLOCK_PACKAGE_VERIFY_OK", result.stdout)

        self.assertIn(hashlib.sha256(installer.read_bytes()).hexdigest(), runbook)
        for marker in (
            "HDU_CLOCK_PACKAGE_VERIFY_OK",
            "HDU_CLOCK_PACKAGE_STAGED_OK",
            "HDU_CLOCK_BASELINE_INSTALLED_OK",
            "HDU_CLOCK_BASELINE_ROLLED_BACK",
            "HDU_CLOCK_BASELINE_ROLLBACK_INCOMPLETE",
        ):
            self.assertIn(marker, runbook)
