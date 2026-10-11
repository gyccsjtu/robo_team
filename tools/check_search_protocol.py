"""Fail startup if generated search messages differ from this checkout."""
import sys
from pathlib import Path
import genmsg
import genmsg.gentools
import genmsg.msg_loader
import rospkg
from robocup_swarm.msg import SearchAssignment, UavStatus


def check(root=None):
    root = Path(root) if root else Path(__file__).resolve().parents[1]
    context = genmsg.msg_loader.MsgContext.create_default()
    ros = rospkg.RosPack()
    paths = {name:[str(Path(ros.get_path(name))/'msg')]
             for name in ('std_msgs','geometry_msgs')}
    for name,cls in [('SearchAssignment',SearchAssignment),('UavStatus',UavStatus)]:
        spec = genmsg.msg_loader.load_msg_from_file(context,
            str(root/'coordination/src/robocup_swarm/msg'/(name+'.msg')),'robocup_swarm/'+name)
        genmsg.msg_loader.load_depends(context,spec,paths)
        expected = genmsg.gentools.compute_md5(context,spec)
        if expected != cls._md5sum:
            raise RuntimeError('%s compiled message is stale: expected %s, loaded %s; rebuild coordination'
                               % (name,expected,cls._md5sum))
        print('[search protocol] %s %s OK'%(name,expected))


if __name__=='__main__':
    try:
        check()
    except Exception as exc:
        print('[search protocol] FAIL: '+str(exc),file=sys.stderr)
        sys.exit(1)
