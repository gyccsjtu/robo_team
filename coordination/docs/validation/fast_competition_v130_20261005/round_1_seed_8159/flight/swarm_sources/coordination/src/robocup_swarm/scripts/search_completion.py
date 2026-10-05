"""Pure evidence gate for search exhaustion; lack of assignments is not success."""
import json


def parse_actor_list(payload):
    try:
        actors = json.loads(payload)
        if (not isinstance(actors, list) or any(type(a) is not int or not 0 <= a < 6 for a in actors)
                or len(set(actors)) != len(actors)):
            return None
        return actors
    except (ValueError, TypeError):
        return None


def completion_state(unfinished_cells, official_seen, official_left, unconfirmed_targets):
    if official_seen and not official_left and not unconfirmed_targets:
        return 'OFFICIAL_COMPLETION_CONFIRMED'
    if unfinished_cells:
        return 'SEARCH_OR_WAIT_FOR_ROUTE'
    if not official_seen:
        return 'OFFICIAL_EVIDENCE_MISSING'
    if official_left or unconfirmed_targets:
        return 'TARGETS_REMAIN'
    return 'TARGETS_REMAIN'
