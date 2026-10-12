"""Camera attachment must compare measurements and poses from the same image."""
import math
import unittest
from perception import perception_real as pr


class ImageTimeParallaxTests(unittest.TestCase):
    def test_recorded_white_raw_point_survives_filtered_state_bias(self):
        initial = (118.40234595680519, 31.722612046251392, .4262259913933992)
        current = (118.92289870172512, 31.946431508809063, .542273778795729)
        tr = pr.Track('white', 128.0002634422391, 36.36681472428118,
                      .9, (369.,336.), (27.,56.), 12.207226727231847,
                      1.756, 2463., uav=initial)
        tr.observed_s = 2464.576
        tr.recent_observed_xy.append((tr.observed_s,128.7700131177132,38.14231128941772))
        tr.uav = current
        tr.x,tr.y = 128.07671717171718,37.34858585858586
        tr.remember_parallax()
        self.assertAlmostEqual(tr.d_rel,.9733665134468761)
        self.assertAlmostEqual(tr.d_view,1.9832523183771014)
        self.assertFalse(tr.attached_to_cam())
        # Changing processing-time prediction must not change image evidence.
        before = (tr.d_rel,tr.d_view)
        tr.coast(.5)
        tr.remember_parallax()
        self.assertEqual((tr.d_rel,tr.d_view),before)

    def test_camera_attached_false_box_still_fails(self):
        tr = pr.Track('blue',8.,0.,.9,(376.,400.),(40.,100.),8.,1.7,100.)
        tr.observed_s = 101.
        tr.recent_observed_xy.append((101.,11.,0.))
        tr.uav = (3.,0.,0.)
        tr.x,tr.y = 9.,2.  # filter lag must not rescue a genuine body artifact
        tr.remember_parallax()
        self.assertEqual(tr.d_rel,0.)
        self.assertTrue(tr.attached_to_cam())

    def test_actual_update_retains_the_raw_image_measurement(self):
        tr = pr.Track('green',8.,0.,.9,(376.,400.),(40.,100.),8.,1.7,100.)
        self.assertTrue(tr.update(10.,1.,.2,.9,(376.,400.),(40.,100.),
                                  8.,1.7,100.2,uav=(1.,0.,0.),observation_s=100.2))
        rx,ry = pr.cam_rel(1.,0.,0.,10.,1.)
        self.assertAlmostEqual(tr.d_rel,math.hypot(rx-8.,ry))


if __name__=='__main__': unittest.main()
