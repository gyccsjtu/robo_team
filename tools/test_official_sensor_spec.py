"""Regression: changing scan range or camera optics must fail preflight."""

import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_official_sensor_spec import check_spec


HOME = Path.home()
TEAM = HOME / "PX4_Firmware/Tools/sitl_gazebo/models/typhoon_h480_laser/typhoon_h480_laser.sdf"
MIRROR = HOME / "XTDrone/sitl_config/models/typhoon_h480_laser/typhoon_h480_laser.sdf"
HOKUYO = HOME / "XTDrone/sitl_config/models/hokuyo_lidar/model.sdf"
STEREO = HOME / "XTDrone/sitl_config/models/stereo_camera/model.sdf"


class OfficialSensorSpecTests(unittest.TestCase):
    def test_installed_sensor_specs_match_xtdrone(self):
        self.assertTrue(check_spec(TEAM, MIRROR, HOKUYO, STEREO))

    def test_changed_laser_range_is_rejected_even_when_mirrors_match(self):
        with tempfile.TemporaryDirectory() as folder:
            mutated = Path(folder) / "model.sdf"
            text = TEAM.read_text(encoding="utf-8")
            mutated.write_text(text.replace("<max>20</max>", "<max>12</max>", 1),
                               encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "ray/range/max"):
                check_spec(mutated, mutated, HOKUYO, STEREO)


if __name__ == "__main__":
    unittest.main()
