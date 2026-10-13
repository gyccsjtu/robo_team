"""Counterexamples from the frozen 19-map audit, using production functions."""
import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from height_datum import GroundHeightDatum
from swarm_task import LineOfSight
import swarm_agent as sa


class LosTraversalTests(unittest.TestCase):
    def test_recorded_diagonal_endpoint_obstacle_is_checked(self):
        seen=[]
        def blocked(x,y):
            seen.append((x,y))
            return (x,y)==(10,11)
        self.assertFalse(LineOfSight(blocked,.5).visible(.25,.25,6.25,6.25))
        self.assertIn((10,11),seen)

    def test_all_octants_visit_every_intermediate_cell(self):
        for sx,sy in ((1,1),(-1,1),(1,-1),(-1,-1)):
            visited=[]
            free=LineOfSight(lambda x,y:visited.append((x,y)) or False,.5)
            self.assertTrue(free.visible(.25,.25,.25+6*sx,.25+6*sy))
            self.assertEqual(len(visited),23)
            for cell in visited:
                los=LineOfSight(lambda x,y:(x,y)==cell,.5)
                self.assertFalse(los.visible(.25,.25,.25+6*sx,.25+6*sy))

    def test_endpoint_exemption_and_straight_paths_unchanged(self):
        for end in ((6.25,.25),(.25,6.25),(.25,.25)):
            target=(math.floor(end[0]/.5),math.floor(end[1]/.5))
            self.assertTrue(LineOfSight(lambda x,y:(x,y) in ((0,0),target),.5).visible(.25,.25,*end))
        self.assertFalse(LineOfSight(lambda x,y:(x,y)==(10,0),.5).visible(.25,.25,6.25,.25))


class GroundCommitTests(unittest.TestCase):
    def datum(self):
        d=GroundHeightDatum(.22)
        for i in range(41):
            d.observe(100.+i*.05,-.1,False,0.,now=100.+i*.05)
        return d

    def test_arm_transition_no_longer_freezes_none_after_qualified_commit(self):
        d=self.datum(); value=d.offset
        self.assertTrue(d.commit_for_takeoff(102.05))
        # State callback arrives after the first armed pose/ground-window expiry.
        self.assertEqual(d.observe(102.6,.8,False,.1,now=102.6),value)
        self.assertEqual(d.observe(102.7,1.,True,.1,now=102.7),value)
        self.assertEqual(d.observe(104.,3.,True,0.,now=104.),value)

    def test_without_commit_same_failure_is_reproduced(self):
        d=self.datum()
        self.assertIsNone(d.observe(102.6,.8,False,.1,now=102.6))
        self.assertIsNone(d.observe(102.7,1.,True,.1,now=102.7))

    def test_stale_unqualified_armed_datum_never_commits(self):
        self.assertFalse(GroundHeightDatum(.22).commit_for_takeoff(100.))
        d=self.datum(); self.assertFalse(d.commit_for_takeoff(102.3))
        d.observe(102.1,1.,True,0.,now=102.1)
        self.assertFalse(d.commit_for_takeoff(102.1))
        d=self.datum(); d.observe(102.1,-.3,False,0.,now=102.1)
        self.assertFalse(d.commit_for_takeoff(102.1))

    def test_ground_estimator_reset_blocks_repeat_arm_request(self):
        d=self.datum();self.assertTrue(d.commit_for_takeoff(102.))
        d.observe(102.1,-.8,False,0.,now=102.1)
        self.assertFalse(d.commit_for_takeoff(102.1))
        self.assertEqual(d.offset,.32)

    def test_agent_requires_successful_own_camera_commit(self):
        a=sa.SwarmAgent.__new__(sa.SwarmAgent); a.uav_id='typhoon_h480_4'
        with patch.dict(sa.os.environ,{'TAKEOFF_REQUIRE_CAMERA':'1'}), patch.object(sa.rospy,'logwarn_throttle'), patch.object(sa.rospy,'ServiceProxy') as service:
            service.return_value.return_value=NS(success=False,message='not ready')
            self.assertFalse(a._commit_camera_datum())
            service.return_value.return_value=NS(success=True,message='ready')
            self.assertTrue(a._commit_camera_datum())
            self.assertEqual(service.call_args.args[0],'/typhoon_h480_4/perception/prepare_takeoff')


if __name__=='__main__': unittest.main()
