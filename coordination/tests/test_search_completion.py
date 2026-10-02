from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / 'src/robocup_swarm/scripts'))
from search_completion import completion_state, parse_actor_list


class CompletionTests(unittest.TestCase):
    def test_no_assignment_with_unfinished_cells_is_not_complete(self):
        self.assertEqual(completion_state(True, True, [1], []), 'SEARCH_OR_WAIT_FOR_ROUTE')

    def test_empty_tracker_without_official_feedback_is_missing_evidence(self):
        self.assertEqual(completion_state(False, False, [], []), 'OFFICIAL_EVIDENCE_MISSING')

    def test_known_remaining_target_prevents_completion(self):
        self.assertEqual(completion_state(False, True, [1], []), 'TARGETS_REMAIN')
        self.assertEqual(completion_state(False, True, [], ['t1']), 'TARGETS_REMAIN')

    def test_complete_requires_positive_official_evidence(self):
        self.assertEqual(completion_state(False, True, [], []), 'OFFICIAL_COMPLETION_CONFIRMED')
        self.assertEqual(completion_state(True, True, [], []), 'OFFICIAL_COMPLETION_CONFIRMED')

    def test_malformed_actor_list_cannot_become_an_empty_result(self):
        for payload in ('garbage', '', '[true]', '[6]', '[1,1]', '{}', '["1"]'):
            self.assertIsNone(parse_actor_list(payload))
        self.assertEqual(parse_actor_list('[]'), [])
        self.assertEqual(parse_actor_list('[0, 2, 5]'), [0, 2, 5])
