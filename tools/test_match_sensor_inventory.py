"""Offline regressions for installed-device counting and invalid declarations."""

from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from match_sensor_inventory import inventory, main


MODEL = """<sdf version="1.6"><model name="test_aircraft">
<link name="base_link">
  <sensor name="stereo_camera_left" type="camera"><camera/></sensor>
  <sensor name="stereo_camera_right" type="camera"><camera/></sensor>
  <sensor name="laser" type="ray"><ray><scan><horizontal><samples>512</samples>
  </horizontal><vertical><samples>1</samples></vertical></scan></ray></sensor>
</link><link name="cgo3_camera_link">
  <sensor name="camera" type="camera"><camera/></sensor>
  <sensor name="camera_imu" type="imu"/>
</link><plugin name="gimbal_controller" filename="libgazebo_gimbal_controller_plugin.so"/>
</model></sdf>"""


class MatchSensorInventoryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.models = self.folder / "models"
        self.model = self.models / "test_aircraft" / "test_aircraft.sdf"
        self.model.parent.mkdir(parents=True)
        self.model.write_text(MODEL, encoding="utf-8")
        self.scene = self.folder / "scene.launch"
        self.write_scene(range(6))

    def write_scene(self, ids, model="test_aircraft"):
        self.scene.write_text("<launch>" + "".join(
            '<group ns="typhoon_h480_%d"><include file="spawn.launch">'
            '<arg name="sdf" value="%s"/></include></group>' % (i, model)
            for i in ids) + "</launch>", encoding="utf-8")

    def test_six_complete_aircraft_cost_16200(self):
        result = inventory(self.scene, self.models)
        self.assertEqual(result["totals"], dict(mono_cam=6, stereo_cam=6,
                         laser1d=0, laser2d=6, laser3d=0, gimbal=6, bino_cam=0))
        self.assertEqual(result["total_cost"], 16200)
        self.assertEqual(result["sensor_penalty"], 48.6)
        for aircraft in result["aircraft"]:
            self.assertEqual(aircraft["cost"], 2700)
            self.assertEqual(aircraft["counts"]["mono_cam"], 1)
            self.assertEqual(aircraft["counts"]["stereo_cam"], 1)
            self.assertEqual(aircraft["sdf_sha256"], hashlib.sha256(self.model.read_bytes()).hexdigest())

    def test_multicamera_pair_is_one_stereo_device(self):
        replacement = '<sensor name="camera" type="multicamera"><camera name="left"/><camera name="right"/></sensor>'
        start = MODEL.index('  <sensor name="stereo_camera_left"')
        end = MODEL.index('  <sensor name="laser"')
        self.model.write_text(MODEL[:start] + replacement + MODEL[end:], encoding="utf-8")
        result = inventory(self.scene, self.models)
        self.assertEqual(result["totals"]["stereo_cam"], 6)
        self.assertEqual(result["totals"]["mono_cam"], 6)

    def test_missing_or_duplicate_aircraft_is_rejected(self):
        for ids in (range(5), [0, 1, 2, 3, 4, 4], range(7)):
            with self.subTest(ids=list(ids)):
                self.write_scene(ids)
                with self.assertRaises(ValueError):
                    inventory(self.scene, self.models)

    def test_orphan_stereo_camera_is_rejected(self):
        self.model.write_text(MODEL.replace(
            '  <sensor name="stereo_camera_right" type="camera"><camera/></sensor>\n', ''), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "orphan"):
            inventory(self.scene, self.models)

    def test_unknown_camera_is_rejected(self):
        self.model.write_text(MODEL.replace('name="camera" type="camera"',
                                            'name="mystery" type="camera"'), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "unknown camera"):
            inventory(self.scene, self.models)

    def test_model_traversal_and_escaping_symlink_are_rejected(self):
        self.write_scene(range(6), "../outside")
        with self.assertRaisesRegex(ValueError, "unsafe"):
            inventory(self.scene, self.models)
        self.write_scene(range(6))
        outside = self.folder / "outside.sdf"
        outside.write_text(MODEL, encoding="utf-8")
        self.model.unlink()
        self.model.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, "escapes"):
            inventory(self.scene, self.models)

    def test_ray_dimensions_and_gpu_ray_are_counted(self):
        for horizontal, vertical, expected in ((1, 1, "laser1d"),
                                              (512, 1, "laser2d"),
                                              (512, 16, "laser3d")):
            with self.subTest(expected=expected):
                text = MODEL.replace('type="ray"', 'type="gpu_ray"')
                text = text.replace('<samples>512</samples>', '<samples>%d</samples>' % horizontal)
                text = text.replace('<samples>1</samples>', '<samples>%d</samples>' % vertical)
                self.model.write_text(text, encoding="utf-8")
                self.assertEqual(inventory(self.scene, self.models)["totals"][expected], 6)

    def test_included_multicamera_model_and_fallback_filename(self):
        camera = self.models / "stereo_camera" / "model.sdf"
        camera.parent.mkdir()
        camera.write_text('<sdf><model name="stereo_camera"><link name="link">'
                          '<sensor name="camera" type="multicamera">'
                          '<camera name="left"/><camera name="right"/>'
                          '</sensor></link></model></sdf>', encoding="utf-8")
        start = MODEL.index('  <sensor name="stereo_camera_left"')
        end = MODEL.index('  <sensor name="laser"')
        text = (MODEL[:start] + MODEL[end:]).replace('</model>',
            '<include><uri>model://stereo_camera</uri></include></model>')
        self.model.write_text(text, encoding="utf-8")
        result = inventory(self.scene, self.models)
        self.assertEqual(result["totals"]["stereo_cam"], 6)
        self.assertEqual(len(result["aircraft"][0]["model_resources"]), 2)

    def test_cli_judge_args_stdout_and_json_file(self):
        output = self.folder / "report.json"
        stdout, stderr = io.StringIO(), io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            code = main(["--scene", str(self.scene), "--models", str(self.models),
                         "--output", str(output), "--judge-args"])
        self.assertEqual(code, 0)
        self.assertEqual(stderr.getvalue(), "")
        self.assertEqual(stdout.getvalue(),
            "_mono_cam:=6 _stereo_cam:=6 _laser1d:=0 _laser2d:=6 _laser3d:=0 _gimbal:=6 _bino_cam:=0\n")
        self.assertEqual(json.loads(output.read_text())["total_cost"], 16200)

    def test_cli_failure_does_not_write_report_or_arguments(self):
        self.write_scene(range(5))
        output = self.folder / "report.json"
        stdout, stderr = io.StringIO(), io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            code = main(["--scene", str(self.scene), "--models", str(self.models),
                         "--output", str(output), "--judge-args"])
        self.assertEqual(code, 1)
        self.assertEqual(stdout.getvalue(), "")
        self.assertIn("FAIL", stderr.getvalue())
        self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
