"""CPU inference keeps its budget after the real Ultralytics device setup."""
import io
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from contextlib import redirect_stdout

os.environ.setdefault('ROBOCUP_LOG_DIR', '/tmp/robocup_cpu_threads_tests')
os.environ.setdefault('YOLO_CONFIG_DIR', '/tmp/robocup_cpu_threads_tests/yolo_config')
(Path(os.environ['YOLO_CONFIG_DIR'])/'Ultralytics').mkdir(parents=True, exist_ok=True)
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from perception import perception_real as pr


class CpuInferenceThreadTests(unittest.TestCase):
    def test_first_real_predict_overrides_device_reset_and_later_predict_is_stable(self):
        import numpy as np
        import torch
        from ultralytics import YOLO
        from ultralytics.utils.torch_utils import NUM_THREADS

        original = torch.get_num_threads()
        self.addCleanup(torch.set_num_threads, original)
        model = YOLO(str(ROOT/'weights/best_yolo11n_bino_v1.pt'))
        pr.configure_cpu_inference_threads(model, 2)
        torch.set_num_threads(1)
        frame = np.zeros((480, 752, 3), dtype=np.uint8)
        output = io.StringIO()
        with redirect_stdout(output), patch.object(
                torch, 'set_num_threads', wraps=torch.set_num_threads) as setter:
            model.predict(frame, device='cpu', conf=.30, agnostic_nms=False,
                          verbose=False, save=False)
            self.assertEqual(torch.get_num_threads(), 2)
            calls = [call.args[0] for call in setter.call_args_list]
            self.assertEqual(calls, [NUM_THREADS] + ([2] if NUM_THREADS != 2 else []))
            setter.reset_mock()
            model.predict(frame, device='cpu', conf=.30, agnostic_nms=False,
                          verbose=False, save=False)
            self.assertEqual(torch.get_num_threads(), 2)
            setter.assert_not_called()
        self.assertEqual(output.getvalue().count('CPU_THREADS'), 1)
        self.assertIn('configured=2 actual=2 before=%d' % NUM_THREADS, output.getvalue())

    def test_gpu_callback_does_not_read_or_change_cpu_threads(self):
        import torch
        callback = pr.configure_cpu_inference_threads(Mock(), 2)
        with patch.object(torch, 'get_num_threads') as get_threads, \
                patch.object(torch, 'set_num_threads') as set_threads, \
                redirect_stdout(io.StringIO()) as output:
            callback(SimpleNamespace(device=torch.device('cuda:0')))
        get_threads.assert_not_called()
        set_threads.assert_not_called()
        self.assertEqual(output.getvalue(), '')

    def test_invalid_configuration_fails_before_registering_callback(self):
        for value in ('', 'abc', '1.5', 'nan', '0', '-1', None, True, 1.5):
            with self.subTest(value=value):
                model = Mock()
                with self.assertRaisesRegex(ValueError, 'PR_TORCH_THREADS.*positive integer'):
                    pr.configure_cpu_inference_threads(model, value)
                model.add_callback.assert_not_called()

    def test_explicit_thread_count_is_used_and_restored_if_later_reset(self):
        import torch
        original = torch.get_num_threads()
        self.addCleanup(torch.set_num_threads, original)
        callback = pr.configure_cpu_inference_threads(Mock(), '4')
        predictor = SimpleNamespace(device=torch.device('cpu'))
        output = io.StringIO()
        with redirect_stdout(output):
            torch.set_num_threads(1)
            callback(predictor)
            self.assertEqual(torch.get_num_threads(), 4)
            torch.set_num_threads(8)
            callback(predictor)
            self.assertEqual(torch.get_num_threads(), 4)
        self.assertEqual(output.getvalue().count('CPU_THREADS'), 1)
        self.assertIn('configured=4 actual=4', output.getvalue())


if __name__ == '__main__':
    unittest.main()
