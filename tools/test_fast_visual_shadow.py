"""ROS-free tests for bounded buffering and delayed detector handoff."""
import unittest
from tools.fast_visual_shadow import Frame, FrameBuffer, ShadowState, catchup_window, validate_seed


class FakeTracker:
    def __init__(self):
        self.steps = []
    def seed(self, image, stamp, detections):
        self.seed_stamp = stamp
        return [dict(id=d['id'], status='seeded') for d in detections]
    def step(self, image, stamp):
        self.steps.append(stamp)
        return [dict(sample_s=stamp, report_eligible=False)]


DETECTION = dict(id=7, label='blue', box=[10, 10, 40, 70], confidence=.9)


def frames(stamps):
    return [Frame(t, 'image-%.3f' % t) for t in stamps]


class FastVisualShadowTests(unittest.TestCase):
    def test_buffer_bounded_by_count_and_age(self):
        buf = FrameBuffer()
        for frame in frames([10+i*.01 for i in range(100)]):
            self.assertIsNone(buf.add(frame))
        self.assertEqual(16, len(buf.snapshot()))
        buf.add(Frame(12, None))
        self.assertEqual([12], [f.stamp for f in buf.snapshot()])

    def test_duplicate_and_reordered_images_do_not_rewind(self):
        buf = FrameBuffer()
        buf.add(Frame(10, None))
        self.assertEqual('duplicate_image', buf.add(Frame(10, None)))
        self.assertEqual('out_of_order_image', buf.add(Frame(9.9, None)))
        self.assertEqual(1, len(buf.snapshot()))

    def test_seed_requires_exact_frame_not_nearest(self):
        history = frames([10, 10.1, 10.2])
        self.assertEqual('seed_image_not_cached', catchup_window(history, 10.1001)[1])
        self.assertEqual([10.1, 10.2], [f.stamp for f in catchup_window(history, 10.1)[0]])

    def test_catchup_is_bounded_by_time_and_frames(self):
        self.assertEqual('catchup_age_exceeded', catchup_window(frames([10, 10.7]), 10)[1])
        self.assertEqual('catchup_frames_exceeded', catchup_window(frames([10+i*.01 for i in range(14)]), 10)[1])

    def test_delayed_seed_catches_up_before_handoff(self):
        state = ShadowState(FakeTracker, lambda f: f.payload)
        result, _, reason = state.seed(frames([10, 10.1, 10.2]), 10, [DETECTION])
        self.assertIsNone(reason)
        self.assertEqual(10.2, result['frame'].stamp)
        self.assertEqual([10.1, 10.2], state.tracker.steps)
        self.assertEqual('seeded', state.last_seed_results[0]['status'])
        state.step(frames([10.2, 10.3, 10.4]))
        newer = state.tracker
        self.assertEqual('out_of_order_seed', state.seed(frames([10, 10.4]), 10, [DETECTION])[2])
        self.assertIs(newer, state.tracker)
        self.assertEqual('seed_would_rewind_tracker', state.seed(frames([10.1, 10.2]), 10.1, [DETECTION])[2])
        self.assertIs(newer, state.tracker)

    def test_same_image_correction_does_not_emit_duplicate_tracking(self):
        state = ShadowState(FakeTracker, lambda f: f.payload)
        history = frames([10, 10.1, 10.2])
        state.seed(history, 10, [DETECTION])
        result, _, reason = state.seed(history, 10.1, [DETECTION])
        self.assertIsNone(result)
        self.assertIsNone(reason)
        self.assertIsNone(state.step(history))

    def test_latest_only_step_records_skipped_images(self):
        state = ShadowState(FakeTracker, lambda f: f.payload)
        state.seed(frames([10]), 10, [DETECTION])
        result = state.step(frames([10, 10.1, 10.2, 10.3]))
        self.assertEqual(2, result['skipped_cached_frames'])
        self.assertEqual([10.3], state.tracker.steps)

    def test_empty_seed_does_not_renew_detector_age(self):
        state = ShadowState(FakeTracker, lambda f: f.payload)
        state.seed(frames([10]), 10, [DETECTION])
        tracker = state.tracker
        self.assertEqual('empty_detector_result_retained_until_ttl', state.seed(frames([10.1]), 10.1, [])[2])
        self.assertIs(tracker, state.tracker)
        self.assertEqual(10, state.last_seed_stamp)

    def test_validation_caps_instances_and_rejects_bad_seeds(self):
        payload = dict(schema=1, source_uav='typhoon_h480_5', image_stamp=10.,
                       detections=[dict(DETECTION, id=i, confidence=i/10.) for i in range(8)])
        _, _, selected, omitted = validate_seed(payload, ['typhoon_h480_5'])
        self.assertEqual(6, len(selected))
        self.assertEqual(2, omitted)
        self.assertEqual(7, selected[0]['id'])
        for bad in (dict(payload, schema=2), dict(payload, image_stamp=float('nan')),
                    dict(payload, detections=[DETECTION, DETECTION]),
                    dict(payload, source_uav='typhoon_h480_2')):
            with self.assertRaises(ValueError):
                validate_seed(bad, ['typhoon_h480_5'])


if __name__ == '__main__':
    unittest.main()
