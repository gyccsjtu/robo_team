import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from fixture_contacts import classify, sonar_virtual_collisions


class ContactTests(unittest.TestCase):
    def test_verified_virtual_shape_is_distinct_from_physical_sonar_body(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'sonar'
            path.mkdir()
            (path/'model.sdf').write_text('<sdf><model name="sonar"><link name="link"><sensor name="sonar" type="sonar"/></link></model></sdf>')
            virtual = sonar_virtual_collisions(directory, ['a'])
        cone = next(iter(virtual))
        self.assertEqual(classify(cone, 'fixture::box::collision', ['a'], virtual), 'SONAR_SENSOR_INTERSECTION')
        self.assertEqual(classify('a::sonar::link::body_collision', 'fixture::box::collision', ['a'], virtual), 'UAV_BODY_CONTACT')
        self.assertEqual(classify('a::base_link::collision', 'fixture::box::collision', ['a'], virtual), 'UAV_BODY_CONTACT')

    def test_changed_model_with_physical_collision_is_not_whitelisted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'sonar'
            path.mkdir()
            (path/'model.sdf').write_text('<sdf><model name="sonar"><link name="link"><collision name="sonarsensor_collision"/><sensor name="sonar" type="sonar"/></link></model></sdf>')
            with self.assertRaises(ValueError):
                sonar_virtual_collisions(directory, ['a'])
