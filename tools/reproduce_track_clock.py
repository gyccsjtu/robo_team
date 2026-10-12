#!/usr/bin/env python3
"""Compare image updates with/without empty-loop coast calls; no ROS master."""
import argparse
import ast
import json
import math
import subprocess
from pathlib import Path
from unittest.mock import patch

from perception import perception_real as pr


def probe(image_hz, loop_hz):
    tr = pr.Track('white',0.,0.,.9,(376,400),(40,100),8.,1.7,100.,observation_s=100.)
    rows = []
    for i in range(1, int(10*image_hz)+1):
        stamp = 100.+i/image_hz
        if loop_hz:
            while tr.t+1./loop_hz < stamp-1e-6:
                tr.coast(1./loop_hz)
        x = stamp-100.+(.1 if abs(stamp-105.)<1e-6 else 0.)
        tr.update(x,0.,stamp-tr.t,.9,(376,400),(40,100),8.,1.7,stamp,observation_s=stamp)
        rows.append(dict(t=stamp,x=tr.x,vx=tr.vx,error=tr.x-(stamp-100.)))
    return dict(image_hz=image_hz,loop_hz=loop_hz,last_v=tr.vx,
                max_error=max(abs(r['error']) for r in rows),
                max_speed=max(abs(r['vx']) for r in rows),
                min_vx=min(r['vx'] for r in rows),samples=rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',default='7bdc05f')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    revision=subprocess.check_output(['git','rev-parse',args.reference],cwd=root,text=True).strip()
    source=subprocess.check_output(['git','show',revision+':perception/perception_real.py'],cwd=root,text=True)
    cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='Track')
    update=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update')
    namespace=dict(vars(pr))
    exec(compile(ast.Module(body=[update],type_ignores=[]),revision,'exec'),namespace)
    # Only the old measurement update is substituted; constants, predictor,
    # and recorded/synthetic inputs are identical on both sides.
    output={'reference':revision,'change':'image interval in velocity correction',
            'parameters':{'alpha':pr.ALPHA,'beta':pr.BETA,'v_max':pr.V_MAX},
            'before':[],'after':[]}
    for name,method in (('before',namespace['update']),('after',pr.Track.update)):
        with patch.object(pr.Track,'update',method):
            output[name]=[probe(image_hz,loop_hz) for image_hz in (4.,5.)
                          for loop_hz in (0,10,15,30,100)]
    print(json.dumps(output,indent=2))


if __name__=='__main__':main()
