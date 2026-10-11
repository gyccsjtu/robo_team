"""Create an isolated tracker environment without changing the verified environments."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import venv

ROOT = Path('/root/robo_team_build/botsort_env_20261011')
LAP_WHEEL = Path(__file__).with_name('lap-0.5.12-cp38-cp38-manylinux_2_5_x86_64.manylinux1_x86_64.manylinux_2_17_x86_64.manylinux2014_x86_64.whl')
LAP_SHA256 = '72237a267e0245512a58a23a604f1a2590a52cfe43695e1ad84d69d1f51b1a0e'
REFERENCES = [
    Path('/root/robo_team_build/vision_cuda_20261003/lib/python3.8/site-packages'),
    Path('/root/robo_team_build/vision_env/lib/python3.8/site-packages'),
]

def setup():
    if ROOT.exists():
        raise RuntimeError('Target exists; inspect it instead of replacing it: '+str(ROOT))
    for item in REFERENCES:
        if not item.is_dir():
            raise RuntimeError('Missing verified reference directory: '+str(item))
    if not LAP_WHEEL.is_file() or hashlib.sha256(LAP_WHEEL.read_bytes()).hexdigest() != LAP_SHA256:
        raise RuntimeError('Missing or wrong pinned lap wheel: '+str(LAP_WHEEL))
    venv.EnvBuilder(system_site_packages=True, with_pip=True).create(str(ROOT))
    refresh_references()
    print('Created isolated environment at '+str(ROOT), flush=True)
    subprocess.run([str(ROOT/'bin/python'), '-m', 'pip', 'install', '--no-index',
                    '--no-deps', '--disable-pip-version-check', str(LAP_WHEEL)], check=True)

def refresh_references():
    site = ROOT/'lib/python3.8/site-packages'
    # Process the referenced distributions' namespace .pth files as well as
    # their directories. A plain path entry misses matplotlib's mpl_toolkits
    # namespace setup and can select the incompatible Ubuntu copy.
    (site/'verified_vision_references.pth').write_text(
        'import site; '+ '; '.join('site.addsitedir(%r)' % str(p) for p in REFERENCES)+'\n',
        encoding='utf-8')

def verify():
    import cv2
    import lap
    import numpy
    import scipy
    import torch
    import ultralytics
    record = dict(executable=sys.executable, torch=torch.__version__,
                  torch_path=torch.__file__, ultralytics=ultralytics.__version__,
                  ultralytics_path=ultralytics.__file__, numpy=numpy.__version__,
                  opencv=cv2.__version__, scipy=scipy.__version__, lap=lap.__version__,
                  lap_path=lap.__file__, cuda_available=torch.cuda.is_available(),
                  cuda_device=torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
                  sys_path=sys.path)
    if torch.cuda.is_available():
        record['cuda_tensor_sum'] = float(torch.ones((2, 2), device='cuda:0').sum().item())
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from perception.image_mot import ImageMOT
    tracker = ImageMOT(('red', 'blue'), 'u0')
    image = numpy.zeros((120, 160, 3), dtype=numpy.uint8)
    record['empty_image_update'] = tracker.annotate(image, [], 1.)
    boxes = [SimpleNamespace(cls=0, conf=.9,
                             xyxy=numpy.array([[20., 20., 40., 80.]]))]
    record['detection_image_update'] = tracker.annotate(image, boxes, 1.2)
    record['real_detection_identity'] = boxes[0].image_track_id
    record['tracker_statistics'] = tracker.stats
    print(json.dumps(record, ensure_ascii=False, indent=2))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['setup', 'verify', 'refresh_references'])
    args = parser.parse_args()
    globals()[args.mode]()
