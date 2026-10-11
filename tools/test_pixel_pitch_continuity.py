"""Image-time attitude must not split a person merely because the UAV brakes."""
import collections,json,math,unittest
from pathlib import Path
import numpy as np
from perception import perception_real as pr

class PixelPitchTests(unittest.TestCase):
    def box(self,v=260.,elevation=-.2,bearing=0.,color='green',confidence=.9):
        return dict(cls=color,conf=confidence,u=376.,v=v,h=60.,bearing=bearing,
                    elevation=elevation,cam_x=0.,cam_y=0.,appearance_status='actor',
                    box=[364.,v-30.,388.,v+30.])

    def test_pitch_moves_pixels_but_preserves_real_person_identity(self):
        tracker=pr.PixelCueTracker();ids=[]
        for i,v in enumerate((180.,260.,440.,330.,270.,190.,360.)):
            cues=tracker.update(100.+i*.2,[self.box(v=v)])
            ids.extend(c['candidate_id'] for c in cues)
        self.assertGreater(len(ids),3)
        self.assertEqual(set(ids),{'green:0'})

    def test_changed_world_elevation_does_not_inherit_old_identity(self):
        tracker=pr.PixelCueTracker()
        for i in range(3):tracker.update(100.+i*.2,[self.box()])
        tracker.update(100.6,[self.box(elevation=.2)])
        self.assertEqual(tracker.next_id,2)

    def test_two_reds_keep_separate_evidence_through_pitch_and_box_order(self):
        tracker=pr.PixelCueTracker()
        for i in range(5):
            a=self.box(v=180.+i*40,elevation=-.1,bearing=-.06,color='red',confidence=.9)
            b=self.box(v=270.+i*20,elevation=-.35,bearing=.06,color='red',confidence=.92)
            cues=tracker.update(100.+i*.2,[a,b] if i%2 else [b,a])
            if i>=2:
                self.assertEqual(len(cues),2)
                self.assertEqual({c['candidate_id'] for c in cues},{'red:0','red:1'})
                self.assertGreater(next(c['bearing'] for c in cues if c['candidate_id']=='red:0'),0.)

    def test_recorded_green_stays_one_instance_across_braking_episode(self):
        path=Path(__file__).parent/'fixtures/pixel_pitch_041016.json'
        tracker=pr.PixelCueTracker();ids=collections.defaultdict(set);episode=[]
        for frame in json.loads(path.read_text())['frames']:
            boxes=[]
            for row in frame['boxes']:
                x1,y1,x2,y2=row['box'];R=np.array(row['rotation']);cam=row['camera_xyz']
                if not pr.pixel_cue_box_ok(row['cls'],row['conf'],x1,y1,x2,y2):continue
                u,v=(x1+x2)/2.,(y1+y2)/2.;ray=pr.foot_ground_ray(u,v,R)
                boxes.append(dict(cls=row['cls'],conf=row['conf'],u=u,v=v,h=y2-y1,
                    box=row['box'],bearing=math.atan2(ray[1],ray[0]),
                    elevation=math.atan2(ray[2],math.hypot(*ray[:2])),
                    cam_x=cam[0],cam_y=cam[1],appearance_status='actor',
                    body_height_m=pr.cue_body_height(row['box'],cam[2],R)))
            for cue in tracker.update(frame['sample_s'],boxes):
                cls=cue['candidate_id'].split(':')[0];ids[cls].add(cue['candidate_id'])
                if cls=='green' and 1950<=frame['sample_s']<=1961.9:episode.append(cue)
        # Recorded baseline: 9 green IDs; 3 IDs within this continuous episode.
        self.assertLessEqual(len(ids['green']),3)
        self.assertGreaterEqual(len(episode),27)
        self.assertEqual(len({c['candidate_id'] for c in episode}),1)

if __name__=='__main__':unittest.main()
