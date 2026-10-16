import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from as2_vlm.preflight import collect_report


class PreflightTests(unittest.TestCase):
    def test_missing_sdk_does_not_claim_readiness(self):
        report = collect_report()
        self.assertEqual(report["status"], "LOCAL_CHECKS_INCOMPLETE")
        self.assertFalse(report["hardware_connected"])
        self.assertFalse(report["motion_enabled"])

    def test_source_checkout_detection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            header = root / "include/unitree/robot/as2/sport/sport_client.hpp"
            header.parent.mkdir(parents=True)
            header.touch()
            lib = root / "lib/aarch64/libunitree_sdk2.a"
            lib.parent.mkdir(parents=True)
            lib.touch()
            with patch("platform.system", return_value="Linux"), patch("platform.machine", return_value="aarch64"), patch("shutil.which", return_value="/tool"):
                report = collect_report(root)
            self.assertEqual(report["status"], "LOCAL_CHECKS_PASSED")
            self.assertIn("sdk_compatibility", report["unverified"])
            self.assertFalse(report["hardware_connected"])
