import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('buoy_bundle_installer', ROOT/'install_into_repo.py')
installer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(installer)


class InstallerTests(unittest.TestCase):
    def target(self, directory):
        root = Path(directory)
        (root/'boat_ws/src/boat_interfaces').mkdir(parents=True)
        cmake = root/'coordination_ws/src/robotx_coordination_interfaces/CMakeLists.txt'
        cmake.parent.mkdir(parents=True)
        cmake.write_text('project(robotx_coordination_interfaces)\n'
                         'rosidl_generate_interfaces(${PROJECT_NAME}\n'
                         '  "msg/SurveyBuoy.msg"\n'
                         '  DEPENDENCIES std_msgs)\n# Existing offline edit\n')
        return root, cmake

    def test_preflight_is_read_only(self):
        with tempfile.TemporaryDirectory() as d:
            root, cmake = self.target(d)
            original = cmake.read_text()
            count, changed = installer.install(ROOT, root)
            self.assertGreater(count, 15)
            self.assertTrue(changed)
            self.assertEqual(cmake.read_text(), original)
            self.assertFalse((root/'coordination_ws/src/robotx_buoy_interpretation').exists())

    def test_apply_preserves_existing_edits_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as d:
            root, cmake = self.target(d)
            installer.install(ROOT, root, True)
            text = cmake.read_text()
            self.assertIn('# Existing offline edit', text)
            for name in installer.MESSAGES:
                self.assertEqual(text.count(f'"msg/{name}.msg"'), 1)
            count, changed = installer.install(ROOT, root, True)
            self.assertEqual((count, changed), (0, False))

    def test_conflict_is_found_before_any_write(self):
        with tempfile.TemporaryDirectory() as d:
            root, cmake = self.target(d)
            old = cmake.read_text()
            conflict = root/'scripts/build_buoy_interpretation.sh'
            conflict.parent.mkdir()
            conflict.write_text('existing local version')
            with self.assertRaises(ValueError):
                installer.install(ROOT, root, True)
            self.assertEqual(cmake.read_text(), old)
            self.assertEqual(conflict.read_text(), 'existing local version')
            self.assertFalse((root/'coordination_ws/src/robotx_buoy_interpretation').exists())

    def test_wrong_repository_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):
                installer.install(ROOT, Path(d), True)


if __name__ == '__main__':
    unittest.main()
