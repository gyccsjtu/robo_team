"""Transactional tests; all generated files stay in temporary directories."""

from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

try:
    from . import check_official_map as checker
    from . import prepare_official_map as prepare
except ImportError:  # Direct invocation: python3 tools/test_prepare_official_map.py
    import check_official_map as checker
    import prepare_official_map as prepare


def fake_generator(fail=False):
    body = """import os
from pathlib import Path
output_path = os.path.expanduser('~/PX4_Firmware/Tools/sitl_gazebo/worlds/')
"""
    if fail:
        return (body + "raise SystemExit(4)\n").encode()
    actors = "".join(
        '<actor name="actor_%d"><plugin filename="libros_actor_cmd_pose_plugin.so">'
        '<init_pose>%d 0 1.25 0 0 0</init_pose></plugin></actor>' % (i, i)
        for i in range(6))
    return (body + """
Path(output_path).mkdir(parents=True, exist_ok=True)
Path(output_path, 'robocup.world').write_text(%r)
Path('black_box.txt').write_text('[[[0, 1], [0, 1]]]')
Path('obstacle.txt').write_text('0.5 0.5\\n')
""" % ("<sdf><world>" + actors + "</world></sdf>")).encode()


class PrepareOfficialMapTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.xt = self.base / "XTDrone"
        self.robocup = self.xt / "robocup"
        self.robocup.mkdir(parents=True)
        self.world = self.base / "PX4_Firmware/worlds/robocup.world"
        self.world.parent.mkdir(parents=True)
        self.stage_parent = self.base / "stages"
        (self.robocup / "map_generator.py").write_bytes(b"old generator")
        (self.robocup / "rover_static.world").write_bytes(b"old template")
        (self.robocup / "black_box.txt").write_bytes(b"old boxes")
        (self.robocup / "obstacle.txt").write_bytes(b"old obstacles")
        self.world.write_bytes(b"old world")
        self.reference = {name: (fake_generator() if name == "map_generator.py"
                                 else ("official " + name).encode())
                          for name in checker.SOURCE_FILES}

    def old_files_unchanged(self):
        self.assertEqual((self.robocup / "map_generator.py").read_bytes(), b"old generator")
        self.assertEqual((self.robocup / "rover_static.world").read_bytes(), b"old template")
        self.assertEqual((self.robocup / "black_box.txt").read_bytes(), b"old boxes")
        self.assertEqual((self.robocup / "obstacle.txt").read_bytes(), b"old obstacles")
        self.assertEqual(self.world.read_bytes(), b"old world")
        self.assertFalse((self.robocup / "base1.world").exists())

    def test_default_dry_run_does_not_write_any_file(self):
        with patch.object(prepare, "load_reference", return_value=self.reference):
            with redirect_stdout(io.StringIO()) as out:
                self.assertEqual(prepare.main([
                    "--xtdrone-root", str(self.xt), "--world", str(self.world),
                    "--stage-parent", str(self.stage_parent)]), 0)
        self.assertIn("DRY RUN", out.getvalue())
        self.old_files_unchanged()
        self.assertFalse(self.stage_parent.exists())

    def test_apply_backs_up_then_switches_complete_set(self):
        with patch.object(prepare, "load_reference", return_value=self.reference), \
                patch.object(prepare, "active_simulation_processes", return_value=[]):
            stage, risks = prepare.prepare(self.xt, self.world,
                                           stage_parent=self.stage_parent)
        self.assertTrue(stage.joinpath("manifest.json").is_file())
        self.assertEqual(stage.joinpath("backup/world").read_bytes(), b"old world")
        self.assertEqual(stage.joinpath("backup/source__map_generator.py").read_bytes(),
                         b"old generator")
        self.assertEqual((self.robocup / "map_generator.py").read_bytes(),
                         self.reference["map_generator.py"])
        self.assertEqual((self.robocup / "base1.world").read_bytes(),
                         self.reference["base1.world"])
        self.assertIn(b"<actor name=", self.world.read_bytes())
        self.assertIn(b"0.5 0.5", (self.robocup / "obstacle.txt").read_bytes())
        self.assertTrue(risks)  # Synthetic world is intentionally not full-size.

    def test_generator_failure_preserves_originals(self):
        reference = dict(self.reference)
        reference["map_generator.py"] = fake_generator(fail=True)
        with patch.object(prepare, "load_reference", return_value=reference), \
                patch.object(prepare, "active_simulation_processes", return_value=[]):
            with self.assertRaisesRegex(RuntimeError, "原文件保持或已还原"):
                prepare.prepare(self.xt, self.world, stage_parent=self.stage_parent)
        self.old_files_unchanged()
        stages = list(self.stage_parent.glob("official_map_stage_*"))
        self.assertEqual(len(stages), 1)
        self.assertIn("exit=4", (stages[0] / "generator.log").read_text())

    def test_partial_install_failure_rolls_back_every_file(self):
        real_install = prepare.install_one

        def fail_at_world(staged, destination):
            if destination == self.world:
                raise OSError("test switch fault")
            real_install(staged, destination)

        with patch.object(prepare, "load_reference", return_value=self.reference), \
                patch.object(prepare, "active_simulation_processes", return_value=[]), \
                patch.object(prepare, "install_one", side_effect=fail_at_world):
            with self.assertRaisesRegex(RuntimeError, "原文件保持或已还原"):
                prepare.prepare(self.xt, self.world, stage_parent=self.stage_parent)
        self.old_files_unchanged()

    def test_running_gazebo_blocks_apply_before_writes(self):
        with patch.object(prepare, "active_simulation_processes",
                          return_value=["123:gzserver"]):
            with self.assertRaisesRegex(RuntimeError, "仍有仿真/裁判进程"):
                prepare.prepare(self.xt, self.world, stage_parent=self.stage_parent)
        self.old_files_unchanged()
        self.assertFalse(self.stage_parent.exists())


if __name__ == "__main__":
    unittest.main()
