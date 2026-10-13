"""Short-lived image proof for passing an already observed search viewpoint.

No coverage cells are mutated here. Full-cell coverage remains the manager's
nine ground samples, actual processed camera images and line-of-sight checks.
"""
from search_observation import in_view


def remember_transit_views(task, camera, visible):
    if task is None or task.get('purpose') != 'coverage_view':
        return
    sample, x, y, yaw, height, pitch = camera
    if sample < task['created']:
        return
    book = task.setdefault('transit_images', {})
    for index in range(task['index'], len(task['look_points'])):
        look = task['look_points'][index]
        state = book.get(index)
        if state is not None and sample <= state['last']:
            continue
        if not (in_view(x, y, yaw, *look, height, pitch) and visible(x, y, *look)):
            if state is not None:
                state['first'] = None
            continue
        if state is None:
            state = book[index] = dict(first=sample, last=sample, observed_at=None)
        elif state['first'] is None or sample-state['last'] > .7:
            state['first'] = sample
        state['last'] = sample
        if sample-state['first'] >= .2-1e-6:
            state['observed_at'] = sample


def can_pass_imaged_view(task, now):
    if task.get('purpose') != 'coverage_view' or task['index'] >= len(task['cells'])-1:
        return False
    state = task.get('transit_images', {}).get(task['index'])
    sample = None if state is None else state['observed_at']
    return sample is not None and 0. <= now-sample <= 2.
