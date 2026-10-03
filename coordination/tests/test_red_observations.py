import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'src/robocup_swarm/scripts'))
from red_observations import RedObservations, actor_slot_remaining


class RedObservationTests(unittest.TestCase):
    def test_cameras_observing_same_red_person_share_one_geometric_slot(self):
        r=RedObservations()
        self.assertEqual(r.observe(10.,(0.,0.)),'red1')
        self.assertEqual(r.observe(10.1,(.2,.1)),'red1')
        self.assertEqual(r.observe(10.2,(20.,0.)),'red2')
        self.assertEqual(r.observe(10.3,(20.2,0.)),'red2')
        self.assertEqual(r.observe(10.4,(.6,.1)),'red1')
        self.assertIsNone(r.observe(10.5,(50.,50.)))

    def test_old_frame_cannot_rewind_slots_and_expired_slot_can_reacquire(self):
        r=RedObservations();r.observe(10.,(0.,0.));r.observe(10.,(20.,0.))
        self.assertIsNone(r.observe(9.,(0.,0.)))
        self.assertEqual(r.observe(17.,(100.,0.)),'red1')
        self.assertIsNone(r.observe(float('nan'),(0.,0.)))

    def test_one_official_red_elimination_does_not_invalidate_an_arbitrary_camera_slot(self):
        for remaining in ([4],[5]):
            self.assertTrue(actor_slot_remaining(4,remaining))
            self.assertTrue(actor_slot_remaining(5,remaining))
        self.assertFalse(actor_slot_remaining(4,[1,2]))
        self.assertTrue(actor_slot_remaining(1,[1,2]))
