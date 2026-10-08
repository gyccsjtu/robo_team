"""Offline PNG/render association by exact raw-pixel checksum; no ROS/control."""
import argparse
import json
from pathlib import Path
import numpy as np


def fnv1a64(data):
    value = 14695981039346656037
    for byte in data:
        value = ((value ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return str(value)


def rotation(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    import cv2
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = Path(args.run)
    candidates, wanted = [], set()
    for index in sorted((root/'flight/algorithm').glob('evidence*_pass/index.jsonl')):
        for line in index.read_text().splitlines():
            row = json.loads(line)
            image = cv2.imread(str(index.parent/row['png']))
            if image is None:
                continue
            # Render format is checked again before accepting a match.
            key = fnv1a64(image[:, :, ::-1].tobytes())
            candidates.append((row, key, str(index.parent/row['png'])))
            wanted.add(key)
    matches = {}
    with (root/'flight/render_binding.jsonl').open() as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except ValueError:
                continue  # A running writer can leave a partial last line.
            if row.get('fnv1a64') in wanted and row.get('format') == 'R8G8B8':
                matches.setdefault(row['fnv1a64'], []).append(row)
    output = []
    opt_to_link = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    for candidate, key, png in candidates:
        binding = candidate.get('camera_binding')
        item = dict(candidate=candidate, png=png, fnv1a64=key,
                    render_matches=matches.get(key, []), control_input=False)
        item['comparisons'] = []
        if binding:
            original_r = np.array(binding['camera_rotation']).reshape(3, 3)
            original_xyz = np.array(binding['camera_xyz'])
            fx, fy, cx, cy = binding['intrinsics']
            for rendered in item['render_matches']:
                rp = rendered['camera_pose']
                render_r = rotation(rp['quaternion_xyzw'])
                render_xyz = np.array(rp['xyz'])
                projections = []
                for visual in rendered['visuals']:
                    for bone in visual['bones']:
                        world = np.array(bone['node_world'])
                        optical = opt_to_link.T @ render_r.T @ (world-render_xyz)
                        projections.append(dict(visual=visual['name'], bone=bone['name'],
                            world=world.tolist(), optical_depth=float(optical[2]),
                            pixel=([float(fx*optical[0]/optical[2]+cx),
                                    float(fy*optical[1]/optical[2]+cy)]
                                   if optical[2] > 0 else None)))
                item['comparisons'].append(dict(render_seq=rendered['seq'],
                    camera=rendered['camera'], scene_s=rendered['scene_s'],
                    translation_delta_m=float(np.linalg.norm(render_xyz-original_xyz)),
                    rotation_delta_rad=float(np.arccos(np.clip(
                        (np.trace(render_r.T@original_r)-1)/2, -1, 1))),
                    actor_projections=projections))
        output.append(item)
    Path(args.output).write_text(json.dumps(output, indent=2, allow_nan=False))
    print(json.dumps(dict(candidates=len(output), exact_matches=sum(bool(x['render_matches'])
          for x in output), output=args.output)))


if __name__ == '__main__':
    main()
