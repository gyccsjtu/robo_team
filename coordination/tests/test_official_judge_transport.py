"""Exercise actual publisher setup and emit against official topic/type contract."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/robocup_swarm/scripts'))
import yolo_target_bridge as bridge


class OfficialTransportTests(unittest.TestCase):
    def publishers(self):
        source = (ROOT/'src/robocup_swarm/scripts/yolo_target_bridge.py').read_text()
        tree = ast.parse(source)
        node = next(n for n in ast.walk(tree) if isinstance(n, ast.For)
                    and isinstance(n.iter, ast.Name) and n.iter.id == 'TAG_TO_TID')
        obj = NS(_actor_pubs={})
        def publisher(topic, message_type, **kwargs):
            return NS(topic=topic, message_type=message_type, messages=[],
                      publish=lambda msg: obj._actor_pubs_by_topic[topic].messages.append(msg))
        obj._actor_pubs_by_topic={}
        def register(topic, message_type, **kwargs):
            pub=publisher(topic,message_type,**kwargs)
            obj._actor_pubs_by_topic[topic]=pub
            return pub
        env=dict(self=obj,rospy=NS(Publisher=register),ActorInfo=NS,TAG_TO_TID=bridge.TAG_TO_TID)
        exec(compile(ast.Module(body=[node],type_ignores=[]),'actual_publishers','exec'),env)
        return obj

    def test_every_report_topic_is_subscribed_by_unmodified_official_judge(self):
        tree=ast.parse((ROOT/'vendor/official_robocup/20261006/score_cal.py').read_text())
        topics={n.args[0].value for n in ast.walk(tree) if isinstance(n,ast.Call)
                and isinstance(n.func,ast.Attribute) and n.func.attr=='Subscriber'
                and n.args and isinstance(n.args[0],ast.Constant)}
        pubs=self.publishers()._actor_pubs
        self.assertEqual(len({p.topic for p in pubs.values()}),6)
        self.assertTrue(all(p.topic in topics for p in pubs.values()))
        self.assertNotIn('/actor_red_info',{p.topic for p in pubs.values()})

    def test_actual_emit_sends_each_red_slot_once_with_red_payload(self):
        b=bridge.YoloTargetBridge.__new__(bridge.YoloTargetBridge)
        b.core=bridge.TargetBridgeCore(); b._actor_pubs=self.publishers()._actor_pubs
        b._now=lambda:100.; b._ActorInfo=NS
        b._TargetState=lambda:NS(header=NS())
        b._rospy=NS(Time=NS(from_sec=lambda x:x)); b.pub=NS(publish=lambda m:None)
        for tag in ('red1','red2'):
            b.core.tracks[tag].t_obs=100.
            b._emit(dict(tag=tag,tid=bridge.TAG_TO_TID[tag],x=3.,y=4.,vx=0.,vy=0.,state=1,eliminated=False))
        for tag in ('red1','red2'):
            self.assertEqual(len(b._actor_pubs[tag].messages),1)
            self.assertEqual(b._actor_pubs[tag].messages[0].cls,'red')

    def test_runtime_and_repo_judge_use_official_message_type(self):
        for file in (ROOT/'vendor/official_robocup/20261006/score_cal.py',
                     ROOT/'src/robocup_swarm/scripts/score_cal.py',
                     ROOT/'src/robocup_swarm/scripts/yolo_target_bridge.py'):
            imports=[n for n in ast.walk(ast.parse(file.read_text())) if isinstance(n,ast.ImportFrom)
                     and any(a.name=='ActorInfo' for a in n.names)]
            self.assertEqual([n.module for n in imports],['ros_actor_cmd_pose_plugin_msgs.msg'])


if __name__=='__main__':
    unittest.main()
