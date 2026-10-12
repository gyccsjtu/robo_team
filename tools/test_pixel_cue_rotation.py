"""Camera rotation may keep a unique complete pixel track, never qualify depth."""
import json
import math
import unittest
from pathlib import Path

import numpy as np

from perception import perception_real as pr


class PixelCueRotationTests(unittest.TestCase):
    def box(self, rotation=None, ray=None, color='blue'):
        rotation = np.eye(3) if rotation is None else rotation
        foot = np.array([12., 0., -4.]) if ray is None else np.array(ray, dtype=float)
        head = foot + np.array([0., 0., 1.7])

        def project(point):
            optical = pr.M_OPT2LINK.T @ rotation.T @ point
            self.assertGreater(optical[2], 0.)
            return (pr.CX+pr.FX*optical[0]/optical[2],
                    pr.CY+pr.FY*optical[1]/optical[2])

        u, bottom = project(foot)
        _, top = project(head)
        height = bottom-top
        item = dict(cls=color, conf=.9, u=u, v=(top+bottom)/2., h=height,
                    bearing=pr.image_world_bearing(u, (top+bottom)/2., rotation),
                    cam_x=0., cam_y=0., box=[u-12., top, u+12., bottom],
                    width=pr.IMG_W, height=pr.IMG_H, fy=pr.FY,
                    appearance_status='actor', small_box_continuation_only=False)
        world_ray = pr.foot_ground_ray(u, bottom, rotation)
        item['foot_down_sin'] = float(-world_ray[2]/np.linalg.norm(world_ray))
        return item

    def rotation(self, pitch=0., roll=0., yaw=0.):
        p, r, y = map(math.radians, (pitch, roll, yaw))
        ry = np.array([[math.cos(p), 0., math.sin(p)], [0., 1., 0.],
                       [-math.sin(p), 0., math.cos(p)]])
        rx = np.array([[1., 0., 0.], [0., math.cos(r), -math.sin(r)],
                       [0., math.sin(r), math.cos(r)]])
        rz = np.array([[math.cos(y), -math.sin(y), 0.],
                       [math.sin(y), math.cos(y), 0.], [0., 0., 1.]])
        return rz @ ry @ rx

    def established(self, box):
        tracker = pr.PixelCueTracker()
        for sample in (100., 100.2, 100.4):
            cues = tracker.update(sample, [dict(box)])
        self.assertEqual(len(cues), 1)
        return tracker

    def fixture(self):
        return json.loads((Path(__file__).parent/'fixtures/blue_pitch_id_switch_000511.json').read_text())

    def fixture_box(self, image):
        data = self.fixture()
        bounds = image['box']
        return dict(box=bounds, width=data['width'], height=data['height'],
                    fy=data['fy'], h=bounds[3]-bounds[1],
                    v=(bounds[1]+bounds[3])/2.,
                    foot_down_sin=image['foot_down_sin'],
                    sample_s=image['sample_s'], appearance_status='actor')

    def test_fixed_world_foot_direction_is_rotation_invariant(self):
        first = self.box()
        for angles in ((20., 0., 0.), (20., 15., 10.)):
            turned = self.box(self.rotation(*angles))
            self.assertAlmostEqual(turned['foot_down_sin'], first['foot_down_sin'])
            self.assertGreater(abs(turned['v']-first['v']), 90.)
            previous = dict(first, sample_s=100.)
            self.assertAlmostEqual(pr.pixel_foot_angle_score(turned, previous, 100.2), 0.)

    def test_unique_rotation_keeps_pixel_identity_without_changing_report_gates(self):
        first = self.box()
        tracker = self.established(first)
        turned = self.box(self.rotation(pitch=20.))
        self.assertIsNone(tracker._association_distance(turned, tracker.tracks[0], 100.6))
        cues = tracker.update(100.6, [turned])
        self.assertEqual(cues[0]['candidate_id'], 'blue:0')
        self.assertTrue(cues[0]['association_unique'])
        self.assertNotIn('report_eligible', cues[0])
        # Angular continuity does not relax the independent ground-ray gate.
        self.assertAlmostEqual(cues[0]['foot_down_sin'], first['foot_down_sin'])

    def test_recorded_complete_pair_crosses_pixel_gate_but_not_world_angle_gate(self):
        data = self.fixture()
        old, new = map(self.fixture_box, data['complete_pair'])
        self.assertGreater(abs(new['v']-old['v']), 90.)
        self.assertAlmostEqual(new['sample_s']-old['sample_s'], .188)
        score = pr.pixel_foot_angle_score(new, old, new['sample_s'])
        self.assertIsNotNone(score)
        self.assertLess(score, .13)
        old, clipped = map(self.fixture_box, data['clipped_pair_must_not_be_rescued'])
        self.assertIsNone(pr.pixel_foot_angle_score(clipped, old, clipped['sample_s']))

    def test_changed_world_elevation_is_not_camera_rotation(self):
        first = self.box()
        changed = self.box(self.rotation(pitch=20.), ray=[12., 0., -12.])
        self.assertIsNone(pr.pixel_foot_angle_score(changed, dict(first, sample_s=100.), 100.2))

    def test_ambiguous_same_colour_pairs_cannot_use_rotation_rescue(self):
        for color in ('blue', 'red'):
            first = self.box(color=color)
            turned = self.box(self.rotation(pitch=20.), color=color)
            tracker = self.established(first)
            # One old instance, two equally compatible new detections.
            another = dict(turned, conf=.88)
            another['bearing'] += .01
            tracker.update(100.6, [turned, another])
            self.assertEqual(tracker.tracks[0]['sample_s'], 100.4)
            tracker = self.established(first)
            other = dict(tracker.tracks[0], id=1, bearing=first['bearing']+.01)
            tracker.tracks.append(other)
            tracker.next_id = 2
            tracker.update(100.6, [turned])
            self.assertEqual([track['sample_s'] for track in tracker.tracks[:2]], [100.4, 100.4])

    def test_missing_invalid_clipped_and_old_ray_cannot_rescue(self):
        first = self.box()
        previous = dict(first, sample_s=100.)
        valid = self.box(self.rotation(pitch=20.))
        variants = []
        for key in ('box', 'foot_down_sin', 'fy', 'width', 'height'):
            value = dict(valid)
            value.pop(key)
            variants.append(value)
        for key, value in (('foot_down_sin', float('nan')), ('foot_down_sin', 1.1),
                           ('fy', pr.FY+1.), ('appearance_status', 'ambiguous')):
            variants.append(dict(valid, **{key: value}))
        clipped = dict(valid, box=list(valid['box']))
        clipped['box'][3] = pr.IMG_H
        variants.append(clipped)
        for item in variants:
            self.assertIsNone(pr.pixel_foot_angle_score(item, previous, 100.2))
        self.assertIsNone(pr.pixel_foot_angle_score(valid, previous, 100.350001))
        self.assertIsNone(pr.pixel_foot_angle_score(valid, previous, 100.))
        self.assertIsNone(pr.pixel_foot_angle_score(valid, previous, 99.9))

    def test_new_image_with_missing_ray_clears_previous_ray_metadata(self):
        tracker = self.established(self.box())
        next_image = self.box()
        next_image.pop('foot_down_sin')
        tracker.update(100.6, [next_image])
        self.assertIsNone(tracker.tracks[0]['foot_down_sin'])
        turned = self.box(self.rotation(pitch=20.))
        tracker.update(100.8, [turned])
        self.assertEqual(tracker.tracks[0]['sample_s'], 100.6)

    def test_legacy_passing_pair_is_preserved_without_new_metadata(self):
        tracker = self.established(self.box())
        next_image = self.box()
        for key in ('foot_down_sin', 'fy', 'box', 'width', 'height'):
            next_image.pop(key)
        cues = tracker.update(100.6, [next_image])
        self.assertEqual(cues[0]['candidate_id'], 'blue:0')
        self.assertTrue(cues[0]['association_unique'])


if __name__ == '__main__':
    unittest.main()
