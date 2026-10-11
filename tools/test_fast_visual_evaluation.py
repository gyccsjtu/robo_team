"""Tests for evaluator data boundaries, not claims about tracker accuracy."""
import json
from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from tools.evaluate_fast_visual_tracker import box_difference, evaluate, load_manifest, unique_matches


def detection(label, box):
    return dict(label=label, box=box)


class EvaluationBoundaryTests(unittest.TestCase):
    def test_separated_same_color_people_get_one_to_one_matches(self):
        predictions = [detection('red', [0, 0, 10, 30]), detection('red', [30, 0, 40, 30])]
        references = list(reversed(predictions))
        pairs, excluded = unique_matches(predictions, references)
        self.assertEqual(pairs, [(0, 1), (1, 0)])
        self.assertFalse(excluded)

    def test_ambiguous_crossing_is_excluded_instead_of_assigned_identity(self):
        predictions = [detection('red', [0, 0, 10, 30]), detection('red', [1, 0, 11, 30])]
        references = [detection('red', [.5, 0, 10.5, 30])]
        pairs, excluded = unique_matches(predictions, references)
        self.assertEqual(pairs, [])
        self.assertEqual(excluded['ambiguous_reference'], 2)

    def test_color_mismatch_is_never_a_match(self):
        pairs, excluded = unique_matches([detection('red', [0, 0, 10, 30])],
                                         [detection('blue', [0, 0, 10, 30])])
        self.assertEqual(pairs, [])
        self.assertEqual(excluded['no_reference_overlap'], 1)

    def test_bottom_is_reported_separately_from_center(self):
        delta = box_difference([0, 0, 10, 30], [0, 10, 10, 20])
        self.assertEqual(delta['center_px'], 0)
        self.assertEqual(delta['bottom_px'], 10)

    def test_manifest_keeps_camera_identity_and_capture_timestamps(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            (directory / 'frame.png').write_bytes(b'placeholder')
            path = directory / 'frames.jsonl'
            rows = [dict(image='frame.png', source_uav=source, image_stamp=stamp)
                    for source, stamp in [('camera_a', 2.), ('camera_b', 1.), ('camera_a', 1.)]]
            path.write_text('\n'.join(map(json.dumps, rows)))
            loaded = load_manifest(path, max_frames=2)
            self.assertEqual([(row['source_uav'], row['image_stamp']) for row in loaded],
                             [('camera_a', 1.), ('camera_b', 1.)])
            path.write_text('\n'.join(map(json.dumps, rows + [rows[0]])))
            with self.assertRaisesRegex(ValueError, 'Duplicate'):
                load_manifest(path)

    def test_intermediate_reference_boxes_never_seed_tracker(self):
        calls = []

        class SpyTracker:
            def __init__(self, **kwargs):
                self.seeds = []

            def seed(self, image, stamp, seeds):
                calls.append(('seed', stamp, [seed['box'] for seed in seeds]))
                self.seeds = seeds
                return [dict(seed, status='seeded', reason='') for seed in seeds]

            def step(self, image, stamp):
                calls.append(('step', stamp, None))
                return [dict(seed, status='tracked', reason='', report_eligible=False,
                             control_eligible=False, footpoint_valid=False) for seed in self.seeds]

        def reference(_self, image):
            offset = float(image[0, 0, 0])
            return [dict(label='blue', box=[10 + offset, 10, 30 + offset, 60], confidence=.8)], .001

        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            weights = directory / 'unused.pt'
            weights.write_bytes(b'not_loaded_by_mock')
            manifest = directory / 'frames.jsonl'
            rows = []
            for index in range(4):
                image = directory / ('%d.png' % index)
                cv2.imwrite(str(image), np.full((80, 80, 3), index, dtype=np.uint8))
                rows.append(dict(image=str(image), image_stamp=1 + index / 10., source_uav='camera'))
            manifest.write_text('\n'.join(map(json.dumps, rows)))
            args = SimpleNamespace(manifest=manifest, max_frames=4, source=None,
                output=directory / 'result', weights=weights, threads=1, confidence=.3,
                imgsz=640, device='cpu', labels=['blue'], reference_cache=None,
                seed_every=3, max_age=.6, max_gap=.3, max_tracks=6, max_comparisons=0)
            with patch('perception.fast_visual_tracker.FastVisualTracker', SpyTracker), \
                    patch('tools.evaluate_fast_visual_tracker.ReferenceDetector.detect', reference), \
                    redirect_stdout(io.StringIO()):
                summary = evaluate(args)
            self.assertEqual([(kind, stamp) for kind, stamp, _ in calls],
                             [('seed', 1.), ('step', 1.1), ('step', 1.2), ('seed', 1.3)])
            self.assertEqual(calls[0][2], [[10., 10, 30., 60]])
            self.assertEqual(calls[-1][2], [[13., 10, 33., 60]])
            self.assertEqual(summary['counts']['intermediate_object_opportunities'], 2)


if __name__ == '__main__':
    unittest.main()
