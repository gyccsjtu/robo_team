"""Offline tests for the one-shot ROS master graph audit."""

import contextlib
import io
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parent))
import audit_match_graph as audit


def valid_graph():
    judge_topics = ["/actor_%s_info" % tag for tag in audit.ACTOR_TAGS]
    publishers = [[topic, ["/yolo_target_bridge"]] for topic in judge_topics]
    publishers.append(["/swarm/target_states", ["/yolo_target_bridge"]])
    subscribers = [[topic, ["/score_cal"]] for topic in judge_topics]
    subscribers.extend([
        ["/left_actors", ["/actor_0", "/actor_5"]],
        ["/find_actor_0", ["/actor_0"]],
    ])
    topic_types = [[topic, audit.ACTOR_INFO_TYPE] for topic in judge_topics]
    return [publishers, subscribers, []], topic_types


class GraphAuditTests(unittest.TestCase):
    def test_gimbal_bearing_aid_cannot_publish_flight_or_formal_positions(self):
        state, types = valid_graph()
        for topic in ('/swarm/gimbal_candidate','/swarm/gimbal_observation',
                      '/typhoon_h480_5/mavros/mount_control/command'):
            state[0].append([topic,['/gimbal_auxiliary']])
        self.assertEqual(audit.audit_graph(state,types),[])
        state[0].append(['/typhoon_h480_5/mavros/setpoint_position/local',['/gimbal_auxiliary']])
        self.assertTrue(any('non-auxiliary' in f for f in audit.audit_graph(state,types)))
        state[1].append(['/swarm/gimbal_candidate',['/yolo_target_bridge']])
        self.assertTrue(any('discovery-only' in f for f in audit.audit_graph(state,types)))

    def test_yaw_only_search_hints_cannot_feed_formal_reports(self):
        state, types = valid_graph()
        state[0].append(['/swarm/search_view_hint', ['/swarm_manager']])
        state[1].append(['/swarm/search_view_hint', ['/swarm_agent_0']])
        self.assertEqual(audit.audit_graph(state, types), [])
        for topic in ('/swarm/search_view_hint', '/swarm/visual_candidate_missed',
                      '/perception/fast_tracking_view'):
            with self.subTest(topic=topic):
                state[1].append([topic, ['/yolo_target_bridge']])
                self.assertTrue(any(topic in f and 'discovery-only' in f
                                    for f in audit.audit_graph(state, types)))
                state[1].pop()
        state[0][-1][1].append('/unexpected_hint_publisher')
        self.assertTrue(any('must be published only' in f
                            for f in audit.audit_graph(state, types)))

    def test_exploration_is_not_an_official_report_input(self):
        state, types = valid_graph()
        state[0].append(['/swarm/discovery_states',['/swarm_manager']])
        state[1].append(['/swarm/discovery_states',['/swarm_agent_0']])
        self.assertEqual(audit.audit_graph(state,types),[])
        state[1].append(['/swarm/discovery_observation',['/yolo_target_bridge']])
        self.assertTrue(any('discovery-only' in f for f in audit.audit_graph(state,types)))

    def test_valid_graph_including_official_actor_feedback(self):
        self.assertEqual(audit.audit_graph(*valid_graph()), [])

    def test_missing_and_wrong_type_are_rejected(self):
        state, types = valid_graph()
        types[0][1] = "robocup_swarm/ActorInfo"
        types.pop()
        failures = audit.audit_graph(state, types)
        self.assertTrue(any("/actor_green_info type" in item for item in failures))
        self.assertTrue(any("/actor_red2_info type" in item for item in failures))

    def test_foreign_and_duplicate_publishers_are_rejected(self):
        state, types = valid_graph()
        state[0][0][1].append("/official_target")
        state[0][-1][1] = ["/target_sim"]
        failures = audit.audit_graph(state, types)
        self.assertTrue(any("/actor_green_info publishers" in item for item in failures))
        self.assertTrue(any("/swarm/target_states publishers" in item for item in failures))

    def test_missing_judge_subscriber_is_rejected(self):
        state, types = valid_graph()
        state[1][1][1] = ["/swarm_manager"]
        failures = audit.audit_graph(state, types)
        self.assertTrue(any("/actor_blue_info has no /score_cal" in item for item in failures))

    def test_forbidden_feedback_and_truth_apply_to_every_other_node(self):
        state, types = valid_graph()
        state[1].extend([
            ["/score", ["/score_diag"]],
            ["/time_usage", ["/swarm_manager"]],
            ["/gazebo/model_states", ["/gazebo_truth_bridge"]],
            ["/typhoon_h480_0/ground_truth/pose", ["/nav/debug"]],
            ["/actor_1/pose", ["/actor_6"]],
        ])
        failures = audit.audit_graph(state, types)
        self.assertEqual(len([item for item in failures if "forbidden topic" in item]), 5)
        self.assertTrue(any("/actor_6 subscribes" in item for item in failures))

    def test_exact_official_node_exemption_only(self):
        state, types = valid_graph()
        state[1].extend([
            ["/left_actors", ["/actor_0", "/team/actor_0", "/score_cal"]],
            ["/scoreboard", ["/swarm_agent_0"]],
        ])
        failures = audit.audit_graph(state, types)
        self.assertEqual(failures, [
            "/team/actor_0 subscribes to forbidden topic /left_actors"])

    def test_team_cannot_publish_actor_commands_or_replace_environment_service(self):
        state, types = valid_graph()
        state[0].extend([['/actor_0/cmd_motion', ['/actor_0', '/team_bad']],
                         ['/gazebo/set_model_state', ['/swarm_agent_0']]])
        state[2].append(['/gazebo/reset_world', ['/team_bad']])
        failures = audit.audit_graph(state, types)
        self.assertEqual(len(failures), 3)
        self.assertTrue(any('/team_bad publishes' in f for f in failures))
        self.assertTrue(any('/team_bad provides' in f for f in failures))

    def test_malformed_master_response_is_error(self):
        with self.assertRaises(audit.GraphError):
            audit.audit_graph([[], []], [])
        with self.assertRaises(audit.GraphError):
            audit.audit_graph([["bad"], [], []], [])
        with self.assertRaises(audit.GraphError):
            audit.audit_graph(*([[], [], []], [["/actor_green_info", "A"],
                                              ["/actor_green_info", "B"]]))

    def test_master_calls_are_only_state_and_types(self):
        state, types = valid_graph()

        class FakeMaster:
            def __init__(self):
                self.calls = []

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return None

            def getSystemState(self, caller):
                self.calls.append(("state", caller))
                return [1, "ok", state]

            def getTopicTypes(self, caller):
                self.calls.append(("types", caller))
                return [1, "ok", types]

        fake = FakeMaster()
        with patch.object(audit.xmlrpc.client, "ServerProxy", return_value=fake) as proxy:
            self.assertEqual(audit.fetch_graph("http://localhost:11311", 3), (state, types))
        self.assertEqual(fake.calls, [
            ("state", "/audit_match_graph"), ("types", "/audit_match_graph")])
        self.assertIsInstance(proxy.call_args.kwargs["transport"], audit.TimeoutTransport)

    def test_cli_status_distinguishes_violation_from_master_error(self):
        state, types = valid_graph()
        output, errors = io.StringIO(), io.StringIO()
        with patch.object(audit, "fetch_graph", return_value=(state, types)), \
                contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
            self.assertEqual(audit.main(["--timeout", "2"]), 0)
        self.assertIn("PASS", output.getvalue())
        state[0][0][1] = []
        with patch.object(audit, "fetch_graph", return_value=(state, types)), \
                contextlib.redirect_stderr(errors):
            self.assertEqual(audit.main([]), 1)
        with patch.object(audit, "fetch_graph", side_effect=audit.GraphError("down")), \
                contextlib.redirect_stderr(errors):
            self.assertEqual(audit.main([]), 2)


if __name__ == "__main__":
    unittest.main()
