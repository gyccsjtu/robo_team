import sys
from pathlib import Path
import unittest
import tempfile
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from observe_visual_accuracy import paired_error
import observe_visual_accuracy as observer


class VisualAccuracyObserverTests(unittest.TestCase):
    def test_coordinates_compare_at_original_image_time(self):
        truth = [(10.,0.,0.), (10.1,.08,0.)]
        self.assertAlmostEqual(paired_error(10.05,[.04,0.],truth),0.)
        self.assertAlmostEqual(paired_error(10.05,[.54,0.],truth),.5)

    def test_missing_old_future_or_sparse_truth_is_not_zero_error(self):
        for truth, stamp in (([],10.), ([(10.,0.,0.)],10.),
                ([(10.,0.,0.),(10.1,.08,0.)],9.),
                ([(10.,0.,0.),(10.1,.08,0.)],11.),
                ([(10.,0.,0.),(11.,.8,0.)],10.5)):
            self.assertIsNone(paired_error(stamp,[0.,0.],truth))

    def run_actual_main(self, directory, *, result_file=None, shutdown=False):
        sub = Mock()
        service = Mock(return_value=SimpleNamespace(success=False))
        ros = SimpleNamespace(init_node=Mock(), set_param=Mock(),
            Subscriber=Mock(return_value=sub), ServiceProxy=Mock(return_value=service),
            is_shutdown=Mock(return_value=shutdown), signal_shutdown=Mock(),
            Time=SimpleNamespace(now=lambda:SimpleNamespace(to_sec=lambda:10.)))
        modules = {'rospy':ros, 'gazebo_msgs.srv':SimpleNamespace(GetModelState=object),
                   'std_msgs.msg':SimpleNamespace(String=object),
                   'ros_actor_cmd_pose_plugin_msgs.msg':SimpleNamespace(ActorInfo=object)}
        argv = ['observer', '--run', str(directory), '--wall-seconds', '2']
        if result_file is not None:argv += ['--result-file', str(result_file)]
        with patch.dict(sys.modules, modules), patch.object(sys, 'argv', argv), \
             patch.object(observer.time, 'monotonic', return_value=0.), \
             patch.object(observer.time, 'sleep', side_effect=lambda _: setattr(ros.is_shutdown, 'return_value', True)):
            observer.main()
        self.assertTrue((directory/'independent_visual_accuracy.json').exists())
        self.assertEqual(sub.unregister.call_count, 2)
        return service

    def test_explicit_flight_result_stops_without_waiting_for_observer_directory_result(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output = root/'observers';output.mkdir()
            result = root/'flight_result.json';result.write_text('{}')
            service = self.run_actual_main(output, result_file=result)
            service.assert_not_called()
            self.assertFalse((output/'result.json').exists())

    def test_ros_shutdown_exits_before_new_truth_queries(self):
        with tempfile.TemporaryDirectory() as temporary:
            service = self.run_actual_main(Path(temporary), shutdown=True)
            service.assert_not_called()

    def test_shutdown_after_sampling_finishes_report_and_unregisters(self):
        with tempfile.TemporaryDirectory() as temporary:
            service = self.run_actual_main(Path(temporary))
            service.assert_called_once_with('actor_0', 'world')
