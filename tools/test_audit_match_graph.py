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


def clock_graph(nodes, topic="/clock", publishers=("/gazebo",),
                type_name="rosgraph_msgs/Clock"):
    state, types = valid_graph()
    state[0].append([topic, list(publishers)])
    state[1].append([topic, list(nodes)])
    types.append([topic, type_name])
    return state, types


class GraphAuditTests(unittest.TestCase):
    def test_automatic_team_clock_subscriptions_require_permission(self):
        nodes = ["/swarm_manager", "/swarm_agent_0", "/yolo_target_bridge",
                 "/perception_real_123", "/fast_visual_yaw_123",
                 "/typhoon_h480_0_communication", "/record_tracking_images_123",
                 "/unknown_node"]
        state, types = clock_graph(nodes)
        self.assertEqual(audit.audit_graph(state, types), [
            "%s subscribes to forbidden simulation clock /clock" % node
            for node in sorted(nodes)])
        self.assertEqual(audit.audit_graph(state, types, allow_clock=True), [])

    def test_only_exact_native_environment_clock_nodes_are_exempt(self):
        nodes = ["/gazebo", "/gazebo_gui", "/score_cal"]
        nodes.extend("/actor_%d" % i for i in range(6))
        nodes.extend("/typhoon_h480_%d/mavros" % i for i in range(6))
        state, types = clock_graph(nodes)
        self.assertEqual(audit.audit_graph(state, types), [])
        lookalikes = ["/team/actor0", "/team/actor_0", "/actor_6", "/gazebo/team",
                      "/typhoon_h480_6/mavros", "/typhoon_h480_0/mavros/plugin"]
        state[1][-1][1].extend(lookalikes)
        self.assertEqual(audit.audit_graph(state, types), [
            "%s subscribes to forbidden simulation clock /clock" % node
            for node in sorted(lookalikes)])

    def test_clock_aliases_cannot_use_canonical_clock_permission(self):
        state, types = clock_graph(["/swarm_agent_0", "/gazebo_gui"],
                                   topic="/renamed_clock", publishers=("/clock_bridge",))
        for allowed in (False, True):
            with self.subTest(allow_clock=allowed):
                failures = audit.audit_graph(state, types, allow_clock=allowed)
                self.assertEqual(len(failures), 3)
                self.assertTrue(all("forbidden simulation clock alias /renamed_clock" in item
                                    for item in failures))
                self.assertTrue(any("/clock_bridge publishes" in item for item in failures))

    def test_clock_alias_publishers_are_rejected_without_subscribers(self):
        state, types = clock_graph([], topic="/team/time",
                                   publishers=("/team_clock",))
        self.assertEqual(audit.audit_graph(state, types, allow_clock=True), [
            "/team_clock publishes forbidden simulation clock alias /team/time (rosgraph_msgs/Clock)"])

    def test_canonical_clock_requires_correct_type_and_unique_gazebo_source(self):
        for clock_type in ("std_msgs/Float64", None):
            with self.subTest(type_name=clock_type):
                state, types = clock_graph(["/gazebo_gui"], type_name=clock_type)
                if clock_type is None:
                    types.pop()
                failures = audit.audit_graph(state, types, allow_clock=True)
                self.assertEqual(failures, [
                    "/clock type is %s; expected rosgraph_msgs/Clock" % (
                        clock_type or "missing")])
        for publishers in ((), ("/team_clock",), ("/gazebo", "/team_clock")):
            with self.subTest(publishers=publishers):
                state, types = clock_graph(["/gazebo_gui"], publishers=publishers)
                failures = audit.audit_graph(state, types, allow_clock=True)
                self.assertIn("/clock publishers are %s; expected only /gazebo" % (
                    sorted(publishers)), failures)
                if "/team_clock" in publishers:
                    self.assertIn("/team_clock publishes forbidden simulation clock /clock",
                                  failures)

    def test_clock_permission_does_not_allow_truth_or_fake_clock_publishers(self):
        state, types = clock_graph(["/swarm_manager"], publishers=("/gazebo", "/team_clock"))
        state[1].append(["/gazebo/model_states", ["/swarm_manager"]])
        failures = audit.audit_graph(state, types, allow_clock=True)
        self.assertIn("/swarm_manager subscribes to forbidden topic /gazebo/model_states",
                      failures)
        self.assertIn("/team_clock publishes forbidden simulation clock /clock", failures)
        self.assertFalse(any("subscribes to forbidden simulation clock /clock" in item
                             for item in failures))

    def test_foreign_canonical_clock_publisher_is_rejected_without_subscribers(self):
        state, types = clock_graph([], publishers=("/swarm_manager",))
        self.assertEqual(audit.audit_graph(state, types, allow_clock=True), [
            "/swarm_manager publishes forbidden simulation clock /clock"])

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

    @patch.dict(audit.os.environ, {"CLOCK_PERMISSION": "unconfirmed",
                                  "CLOCK_PERMISSION_REFERENCE": ""})
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

    def test_cli_unconfirmed_and_forbidden_clock_permission_reject_team_clock(self):
        state, types = clock_graph(["/swarm_manager"])
        for arguments in ([], ["--clock-permission", "forbidden"]):
            with self.subTest(arguments=arguments), \
                    patch.dict(audit.os.environ, {}, clear=True), \
                    patch.object(audit, "fetch_graph", return_value=(state, types)), \
                    contextlib.redirect_stderr(io.StringIO()) as errors:
                self.assertEqual(audit.main(arguments), 1)
                self.assertIn("forbidden simulation clock /clock", errors.getvalue())

    def test_cli_allowed_clock_permission_requires_reference_before_master_query(self):
        for environment, arguments in (
                ({}, ["--clock-permission", "allowed"]),
                ({"CLOCK_PERMISSION": "allowed", "CLOCK_PERMISSION_REFERENCE": "  "}, [])):
            with self.subTest(environment=environment), \
                    patch.dict(audit.os.environ, environment, clear=True), \
                    patch.object(audit, "fetch_graph") as fetch, \
                    contextlib.redirect_stderr(io.StringIO()) as errors:
                self.assertEqual(audit.main(arguments), 2)
                fetch.assert_not_called()
                self.assertIn("requires a nonempty", errors.getvalue())

    def test_cli_permission_and_reference_accept_explicit_arguments_or_environment(self):
        state, types = clock_graph(["/swarm_manager"])
        reference = "teacher confirmation dated 2026-10-13"
        for environment, arguments in (
                ({}, ["--clock-permission", "allowed", "--clock-permission-reference", reference]),
                ({"CLOCK_PERMISSION": "allowed", "CLOCK_PERMISSION_REFERENCE": reference}, [])):
            with self.subTest(environment=environment), \
                    patch.dict(audit.os.environ, environment, clear=True), \
                    patch.object(audit, "fetch_graph", return_value=(state, types)), \
                    contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(audit.main(arguments), 0)
                self.assertIn("clock_permission=allowed", output.getvalue())
                self.assertIn(reference, output.getvalue())
                self.assertIn("Clock aliases and foreign clock publishers forbidden", output.getvalue())
                self.assertIn("master registration snapshot only", output.getvalue())

    def test_cli_unconfirmed_snapshot_without_clock_retains_pass_with_policy_boundary(self):
        with patch.dict(audit.os.environ, {}, clear=True), \
                patch.object(audit, "fetch_graph", return_value=valid_graph()), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(audit.main([]), 0)
        self.assertIn("clock_permission=unconfirmed", output.getvalue())
        self.assertIn("team /clock subscriptions forbidden", output.getvalue())

    def test_cli_rejects_invalid_environment_permission_before_master_query(self):
        with patch.dict(audit.os.environ, {"CLOCK_PERMISSION": "yes"}, clear=True), \
                patch.object(audit, "fetch_graph") as fetch, \
                contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(audit.main([]), 2)
            fetch.assert_not_called()
        self.assertIn("invalid CLOCK_PERMISSION", errors.getvalue())

    def test_cli_explicit_permission_overrides_environment(self):
        state, types = clock_graph(["/swarm_manager"])
        with patch.dict(audit.os.environ, {"CLOCK_PERMISSION": "allowed",
                                          "CLOCK_PERMISSION_REFERENCE": "teacher confirmation"},
                        clear=True), \
                patch.object(audit, "fetch_graph", return_value=(state, types)), \
                contextlib.redirect_stderr(io.StringIO()) as errors:
            self.assertEqual(audit.main(["--clock-permission", "forbidden"]), 1)
        self.assertIn("forbidden simulation clock /clock", errors.getvalue())


if __name__ == "__main__":
    unittest.main()
