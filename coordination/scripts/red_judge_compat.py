"""Adapt a development judge copy to the organizer's red-coordinate matching rule.

Source: user relayed organizer clarification on 2026-10-03. This does not
claim an upstream release implements it. Neither aircraft nor perception
receive actor truth: matching runs exclusively inside the judge process.
"""
import ast


def adapt_red_judge(source):
    tree = ast.parse(source)
    callbacks = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)
                 and n.name in ('actor_info_callback', 'actor_info1_callback', 'actor_info2_callback')}
    if len(callbacks) != 3:
        raise ValueError('Unsupported judge red callbacks')
    lines = source.splitlines(keepends=True)
    for name, node in sorted(callbacks.items(), key=lambda item: item[1].lineno, reverse=True):
        if name == 'actor_info_callback':
            replacement = '''def actor_info_callback(msg):
    if getattr(msg, 'cls', '') == 'red':
        _process_actor_detection(msg, _matched_red_actor_ids(msg))
    else:
        _process_actor_detection(msg, actor_id_dict.get(getattr(msg, 'cls', ''), []))
'''
        else:
            replacement = 'def %s(msg):\n    _process_actor_detection(msg, _matched_red_actor_ids(msg))\n' % name
        lines[node.lineno-1:node.end_lineno] = [replacement]
    adapted = ''.join(lines)
    helper = '''
def _matched_red_actor_ids(msg):
    # One red report matches either remaining red truth; it must not reset the
    # other person's independently accumulated confirmation window.
    if getattr(msg, 'cls', '') != 'red':
        return []
    candidates = [(i, (msg.x-actors_pos[i].x)**2 + (msg.y-actors_pos[i].y)**2)
                  for i in (4, 5) if i in left_actors and actors_pos[i] is not None]
    if not candidates:
        return []
    actor_id, distance_sq = min(candidates, key=lambda row: (row[1], row[0]))
    return [actor_id] if distance_sq < err_threshold**2 else []

'''
    adapted = helper + adapted
    marker = '    actor_red2_sub = rospy.Subscriber("/actor_red2_info",ActorInfo,actor_info2_callback,queue_size=1)'
    if adapted.count(marker) != 1:
        raise ValueError('Unsupported judge subscriber declaration')
    adapted = adapted.replace(marker, marker + '\n    actor_red_sub = rospy.Subscriber("/actor_red_info",ActorInfo,actor_info_callback,queue_size=30)')
    ast.parse(adapted)
    return adapted
