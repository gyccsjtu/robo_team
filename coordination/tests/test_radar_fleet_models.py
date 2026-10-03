import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

spec = importlib.util.spec_from_file_location('models', Path(__file__).parents[1] / 'scripts/radar_fleet_models.py')
models = importlib.util.module_from_spec(spec)
spec.loader.exec_module(models)

SOURCE = '''<sdf version="1.6"><model name="typhoon_h480"><link name="base_link">
<sensor name="laser" type="ray"><update_rate>500</update_rate><ray><range><min>0.5</min><max>20</max></range></ray>
<plugin name="laser" filename="libgazebo_ros_laser.so"><robotNamespace/><topicName>scan</topicName><frameName>laser_2d</frameName></plugin></sensor>
<sensor name="camera" type="camera"><plugin name="camera" filename="libgazebo_ros_camera.so"><cameraName>/cgo3_camera</cameraName><frameName>optical</frameName></plugin></sensor></link>
<plugin name="motor" filename="libgazebo_motor_model.so"><robotNamespace/></plugin>
<plugin name="mavlink_interface" filename="libgazebo_mavlink_interface.so"><mavlink_tcp_port>4560</mavlink_tcp_port><mavlink_udp_port>14560</mavlink_udp_port><sdk_udp_port>14540</sdk_udp_port></plugin>
<plugin name="gps" filename="libgazebo_gps_plugin.so"/></model></sdf>'''


class ModelTests(unittest.TestCase):
    def test_gimbal_overlay_refuses_a_model_without_the_controller(self):
        with self.assertRaises(ValueError):
            models.adapt(SOURCE,dict(model_name='uav_1'),'/runtime/gps',gimbal_overlay='/runtime/gimbal')

    def test_gimbal_identity_and_endpoints_are_per_aircraft_with_sensors_unchanged(self):
        source=SOURCE.replace('</model>','<plugin name="gimbal_controller" filename="libgazebo_gimbal_controller_plugin.so"/></model>')
        for i in range(1,7):
            row=dict(model_name='typhoon_h480_%d'%(i-1),uav_id='uav_%d'%i,
                scan_topic='/uav_%d/scan'%i,simulator_tcp_port=4560+i,
                simulator_udp_port=14560+i,mavros_local_port=14540+i,
                mavlink_system_id=i+1,px4_gimbal_port=13030+i,gimbal_local_port=13280+i)
            root=ET.fromstring(models.adapt(source,row,'/runtime/gps',gimbal_overlay='/runtime/gimbal'))
            p=root.find(".//plugin[@name='gimbal_controller']")
            self.assertEqual(p.get('filename'),'/runtime/gimbal/librobocup_namespaced_gimbal.so')
            self.assertEqual(p.findtext('mavlink_system_id'),str(i+1))
            self.assertEqual(p.findtext('px4_udp_port'),str(13030+i))
            self.assertEqual(p.findtext('gimbal_udp_port'),str(13280+i))
            self.assertEqual(models.sensor_signature(root),models.sensor_signature(ET.fromstring(source)))

    def test_city_hides_only_debug_rays_preserving_physical_sensor_signature(self):
        source = SOURCE.replace('<update_rate>500</update_rate>',
            '<visualize>true</visualize><update_rate>500</update_rate>')
        row = dict(model_name='typhoon_h480_0', uav_id='uav_1', scan_topic='/uav_1/scan',
                   simulator_tcp_port=4561, simulator_udp_port=14561, mavros_local_port=14541)
        original = ET.fromstring(source)
        ordinary = ET.fromstring(models.adapt(source, row, '/runtime/gps'))
        city = ET.fromstring(models.adapt(source, row, '/runtime/gps', hide_ray_visuals=True))
        self.assertEqual(ordinary.find('.//visualize').text, 'true')
        self.assertEqual(city.find('.//visualize').text, 'false')
        self.assertEqual(models.sensor_signature(city), models.sensor_signature(original))
        self.assertEqual(city.find('.//update_rate').text, '500')
        self.assertEqual(city.find('.//ray/range/max').text, '20')

    def test_six_models_preserve_sensors_and_isolate_topics(self):
        for i in range(1, 7):
            uid = 'uav_%d' % i
            row = dict(model_name=uid, uav_id=uid, scan_topic='/%s/scan' % uid,
                       simulator_tcp_port=4560+i, simulator_udp_port=14560+i, mavros_local_port=14540+i)
            root = ET.fromstring(models.adapt(SOURCE, row, '/runtime/gps'))
            self.assertEqual(models.sensor_signature(root), models.sensor_signature(ET.fromstring(SOURCE)))
            self.assertEqual(root.find('.//topicName').text, '/%s/scan' % uid)
            self.assertEqual(root.find('.//frameName').text, uid + '/laser_2d')
            self.assertEqual(root.find('.//mavlink_tcp_port').text, str(4560+i))
            motor = root.find(".//plugin[@name='motor']/robotNamespace")
            self.assertIsNone(motor.text)  # Gazebo already includes the model in its transport topic.
            self.assertEqual(root.find('.//cameraName').text, 'cgo3_camera')
