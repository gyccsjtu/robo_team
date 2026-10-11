"""Offline regression tests for official map provenance and file pairing."""

from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

try:
    from . import check_official_map as check
except ImportError:  # Direct invocation: python3 tools/test_official_map.py
    import check_official_map as check


class OfficialMapCheckTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.world = self.root / "robocup.world"
        self.black_box = self.root / "black_box.txt"
        self.obstacle = self.root / "obstacle.txt"
        self.positions = [(10.0 + 3.0 * i, -10.0 + 2.0 * i)
                          for i in range(len(check.BUILDINGS))]
        self.boxes = [((-48.0 + i, -47.0 + i), (45.0, 46.0))
                      for i in range(check.FIXED_BOXES)]
        self.boxes += [((x - 1.0, x + 1.0), (y - 1.0, y + 1.0))
                       for x, y in self.positions]
        self.boxes += [((60.0 + i, 62.0 + i), (-30.0, -28.0))
                       for i in range(check.ROVERS)]
        self.write_world()
        self.write_data()

    def write_world(self, omit_rover=False, wrong_building=None):
        root = ET.Element("sdf", version="1.6")
        world = ET.SubElement(root, "world", name="default")
        for index in range(6):
            actor = ET.SubElement(world, "actor", name="actor_%d" % index)
            plugin = ET.SubElement(actor, "plugin",
                                   filename="libros_actor_cmd_pose_plugin.so")
            ET.SubElement(plugin, "init_pose").text = "%d %d 1.25 1.57 0 0" % (
                index, index + 1)
        for index, name in enumerate(check.BUILDINGS):
            model = ET.SubElement(world, "model", name=name)
            x, y = self.positions[index]
            if name == wrong_building:
                x += 20.0
            pose_parent = ET.SubElement(model, "link") if name == "gas_station_73" else model
            ET.SubElement(pose_parent, "pose").text = "%s %s 0 0 0 0" % (x, y)
        for index in range(check.ROVERS - int(omit_rover)):
            ET.SubElement(world, "model", name="rover_static_%d" % index)
        ET.ElementTree(root).write(self.world, encoding="unicode")

    def write_data(self, stale_obstacle=False):
        self.black_box.write_text(repr(self.boxes), encoding="utf-8")
        points = check.expected_obstacle_points(self.boxes)
        if stale_obstacle:
            points = points - {(-50.0, -49.5)}
        self.obstacle.write_text(
            "".join("%s %s\n" % point for point in sorted(points)),
            encoding="utf-8",
        )

    def test_consistent_official_shape_passes(self):
        self.assertEqual(check.check_artifacts(
            self.world, self.black_box, self.obstacle), [])

    def test_world_and_black_box_position_mismatch_is_rejected(self):
        self.write_world(wrong_building="fast_food_93")
        errors = check.check_artifacts(self.world, self.black_box, self.obstacle)
        self.assertTrue(any("fast_food_93 坐标不匹配" in error for error in errors), errors)

    def test_gazebo_state_pose_overrides_model_definition_pose(self):
        self.write_world(wrong_building="fast_food_93")
        root = ET.parse(self.world).getroot()
        world = root.find("world")
        state = ET.SubElement(world, "state")
        model = ET.SubElement(state, "model", name="fast_food_93")
        x, y = self.positions[check.BUILDINGS.index("fast_food_93")]
        ET.SubElement(model, "pose").text = "%s %s 0 0 0 0" % (x, y)
        ET.ElementTree(root).write(self.world, encoding="unicode")
        errors = check.check_artifacts(self.world, self.black_box, self.obstacle)
        self.assertEqual(errors, [])

    def test_missing_rover_and_stale_obstacle_are_rejected(self):
        self.write_world(omit_rover=True)
        self.write_data(stale_obstacle=True)
        errors = check.check_artifacts(self.world, self.black_box, self.obstacle)
        self.assertTrue(any("19 个 rover model" in error for error in errors), errors)
        self.assertTrue(any("obstacle.txt 与 black_box.txt" in error for error in errors), errors)

    def test_duplicate_rover_models_are_rejected(self):
        root = ET.parse(self.world).getroot()
        world = root.find("world")
        ET.SubElement(world, "model", name="rover_static_0")
        ET.ElementTree(root).write(self.world, encoding="unicode")
        errors = check.check_artifacts(self.world, self.black_box, self.obstacle)
        self.assertTrue(any("名称重复" in error for error in errors), errors)

    def test_modified_generator_and_missing_template_are_rejected(self):
        source = self.root / "robocup"
        source.mkdir()
        reference = {name: name.encode() for name in check.SOURCE_FILES}
        for name, data in reference.items():
            (source / name).write_bytes(data)
        self.assertEqual(check.check_sources(self.root, reference), [])
        (source / "map_generator.py").write_text("altered", encoding="utf-8")
        (source / "base2.world").unlink()
        errors = check.check_sources(self.root, reference)
        self.assertTrue(any("map_generator.py" in error for error in errors), errors)
        self.assertTrue(any("base2.world" in error for error in errors), errors)

    def test_different_fixed_prefix_still_finds_geometry_problem(self):
        self.boxes = self.boxes[4:]
        self.write_data()
        self.write_world(wrong_building="house_1_66")
        errors = check.check_artifacts(self.world, self.black_box, self.obstacle)
        self.assertTrue(any("43 个矩形" in error for error in errors), errors)
        self.assertTrue(any("house_1_66 坐标不匹配" in error for error in errors), errors)


if __name__ == "__main__":
    unittest.main()
