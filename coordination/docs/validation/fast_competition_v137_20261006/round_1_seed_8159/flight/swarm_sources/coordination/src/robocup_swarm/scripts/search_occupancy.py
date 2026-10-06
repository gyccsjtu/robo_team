"""Keep auction occupancy aligned with verified task-authority releases."""
from swarm_task import STATE_ASSIGNED, STATE_FREE
from task_authority import task_key


def apply_intent_cancellation(grid, event, run_id, current_locks, current_pending):
    """Cancel only an unexecuted auction claim, never an occupied task lock."""
    if not isinstance(event, dict) or event.get('event') != 'TASK_INTENT_CANCELLED':
        return False
    details = event.get('details')
    if not isinstance(details, dict):
        return False
    key = details.get('key')
    if not isinstance(key, list):
        return False
    try:
        if any(task_key(task) == tuple(key) for task in current_pending.values()):
            return False
    except (ValueError, TypeError, KeyError, AttributeError):
        return False
    released = dict(event, event='TASK_RELEASED')
    return apply_authority_release(grid, released, run_id, current_locks)


def apply_authority_release(grid, event, run_id, current_locks):
    if (not isinstance(event, dict) or event.get('schema_version') != 2 or event.get('run_id') != run_id
            or event.get('event') != 'TASK_RELEASED'):
        return False
    details = event.get('details')
    if not isinstance(details, dict) or not isinstance(event.get('uav_id'), str) or not event['uav_id']:
        return False
    key = details.get('key')
    if (not isinstance(key, list) or len(key) != 3 or key[0] != 'search'
            or any(type(value) is not int for value in key[1:])):
        return False
    if tuple(key) in current_locks:
        return False  # A later grant may already have reacquired the same cell.
    cell = grid.cells.get(tuple(key[1:]))
    if cell is None or cell.state != STATE_ASSIGNED or cell.owner != event.get('uav_id'):
        return False
    cell.state, cell.owner, cell.lease_until = STATE_FREE, None, 0.
    return True
