#!/usr/bin/env python3
"""Audit the formal match's ROS master graph without subscribing to any topic.

The master reports registrations even before the first ActorInfo message.  Two
bounded XML-RPC calls replace serial ``rostopic info``/``rosnode info`` calls.
This program never queries Gazebo services or reads judge feedback payloads.
"""

import argparse
import http.client
import math
import os
import re
import socket
import sys
import time
import xmlrpc.client


CALLER_ID = "/audit_match_graph"
ACTOR_TAGS = ("green", "blue", "brown", "white", "red1", "red2")
ACTOR_INFO_TYPE = "ros_actor_cmd_pose_plugin_msgs/ActorInfo"
BRIDGE_NODE = "/yolo_target_bridge"
JUDGE_NODE = "/score_cal"
TARGET_TOPIC = "/swarm/target_states"
OFFICIAL_NODE = re.compile(r"^/actor_[0-5]$")
CLOCK_TOPIC = "/clock"
CLOCK_TYPE = "rosgraph_msgs/Clock"
OFFICIAL_CLOCK_NODE = re.compile(
    r"^/(?:gazebo|gazebo_gui|score_cal|actor_[0-5]|typhoon_h480_[0-5]/mavros"
    r"|typhoon_h480_[0-5]_communication)$"
)
CLOCK_PERMISSIONS = ("unconfirmed", "allowed", "forbidden")
FORBIDDEN_TOPIC = re.compile(
    r"^/(?:score|time_usage|left_actors|find_actor_\d+)$"
    r"|^/gazebo/(?:model|link)_states$"
    r"|^/actor_\d+/pose$"
    r"|(?:^|/)ground_truth(?:/|$)"
)


class GraphError(Exception):
    """The master response cannot be audited safely."""


class TimeoutTransport(xmlrpc.client.Transport):
    """Apply a finite timeout to both newly opened and reused master sockets."""

    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self.timeout
        if connection.sock is not None:
            connection.sock.settimeout(self.timeout)
        return connection


def _master_value(response, operation):
    if not isinstance(response, (list, tuple)) or len(response) != 3:
        raise GraphError("%s returned an invalid XML-RPC response" % operation)
    code, message, value = response
    if code != 1:
        raise GraphError("%s failed (%s): %s" % (operation, code, message))
    return value


def fetch_graph(master_uri, timeout=8.0):
    """Return master graph registration data within bounded socket waits."""
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    if not master_uri.startswith("http://"):
        raise GraphError("ROS_MASTER_URI must use the ROS master's http:// XML-RPC endpoint")
    deadline = time.monotonic() + timeout
    transport = TimeoutTransport(timeout)
    try:
        with xmlrpc.client.ServerProxy(master_uri, transport=transport,
                                       allow_none=False) as master:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise GraphError("graph audit deadline expired before getSystemState")
            transport.timeout = remaining
            state = _master_value(master.getSystemState(CALLER_ID), "getSystemState")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise GraphError("graph audit deadline expired before getTopicTypes")
            transport.timeout = remaining
            topic_types = _master_value(master.getTopicTypes(CALLER_ID), "getTopicTypes")
    except (OSError, socket.timeout, http.client.HTTPException,
            xmlrpc.client.Error, ValueError) as exc:
        raise GraphError("ROS master graph request failed: %s" % exc) from exc
    return state, topic_types


def _pairs(values, label):
    if not isinstance(values, (list, tuple)):
        raise GraphError("%s is not a list" % label)
    for entry in values:
        if (not isinstance(entry, (list, tuple)) or len(entry) != 2
                or not isinstance(entry[0], str) or not entry[0].startswith("/")):
            raise GraphError("%s has an invalid topic entry: %r" % (label, entry))
        yield entry


def _connections(entries, label):
    result = {}
    for topic, nodes in _pairs(entries, label):
        if (not isinstance(nodes, (list, tuple)) or any(
                not isinstance(node, str) or not node.startswith("/") for node in nodes)):
            raise GraphError("%s has invalid nodes for %s" % (label, topic))
        result.setdefault(topic, set()).update(nodes)
    return result


def _topic_types(entries):
    result = {}
    for topic, type_name in _pairs(entries, "topic types"):
        if not isinstance(type_name, str):
            raise GraphError("invalid message type for %s" % topic)
        if topic in result and result[topic] != type_name:
            raise GraphError("conflicting registered message types for %s" % topic)
        result[topic] = type_name
    return result


def audit_graph(state, topic_types, allow_clock=False, allow_judge_pending=False):
    """Return precise rule/registration failures from a master snapshot."""
    if not isinstance(state, (list, tuple)) or len(state) != 3:
        raise GraphError("getSystemState must contain publishers, subscribers and services")
    publishers = _connections(state[0], "publishers")
    subscribers = _connections(state[1], "subscribers")
    types = _topic_types(topic_types)
    failures = []

    for tag in ACTOR_TAGS:
        topic = "/actor_%s_info" % tag
        if types.get(topic) != ACTOR_INFO_TYPE:
            failures.append("%s type is %s; expected %s" % (
                topic, types.get(topic, "missing"), ACTOR_INFO_TYPE))
        if publishers.get(topic, set()) != {BRIDGE_NODE}:
            failures.append("%s publishers are %s; expected only %s" % (
                topic, sorted(publishers.get(topic, ())), BRIDGE_NODE))
        if not allow_judge_pending and JUDGE_NODE not in subscribers.get(topic, ()):
            failures.append("%s has no %s subscriber" % (topic, JUDGE_NODE))

    if publishers.get(TARGET_TOPIC, set()) != {BRIDGE_NODE}:
        failures.append("%s publishers are %s; expected only %s" % (
            TARGET_TOPIC, sorted(publishers.get(TARGET_TOPIC, ())), BRIDGE_NODE))

    discovery_state = '/swarm/discovery_states'
    if (discovery_state in publishers and
            publishers[discovery_state] != {'/swarm_manager'}):
        failures.append('%s must be published only by /swarm_manager' % discovery_state)
    hint_topic = '/swarm/search_view_hint'
    if hint_topic in publishers and publishers[hint_topic] != {'/swarm_manager'}:
        failures.append('%s must be published only by /swarm_manager' % hint_topic)
    for topic in ('/swarm/discovery_observation', discovery_state,
                  '/swarm/visual_candidate', '/swarm/visual_candidate_missed',
                  hint_topic, '/perception/fast_tracking_view'):
        if BRIDGE_NODE in subscribers.get(topic, set()):
            failures.append('%s subscribes to discovery-only topic %s' % (BRIDGE_NODE, topic))

    # rospy/roscpp automatically register /clock when /use_sim_time is true.
    # Framework-created subscriptions still require explicit rule permission.
    # A Clock message on another name cannot use canonical /clock permission.
    clock_topics = {CLOCK_TOPIC} | {
        topic for topic, type_name in types.items() if type_name == CLOCK_TYPE
    }
    for topic in sorted(clock_topics):
        if topic == CLOCK_TOPIC:
            if subscribers.get(topic):
                if types.get(topic) != CLOCK_TYPE:
                    failures.append("%s type is %s; expected %s" % (
                        topic, types.get(topic, "missing"), CLOCK_TYPE))
                if publishers.get(topic, set()) != {"/gazebo"}:
                    failures.append("%s publishers are %s; expected only /gazebo" % (
                        topic, sorted(publishers.get(topic, ()))))
            for node in sorted(subscribers.get(topic, ())):
                if not OFFICIAL_CLOCK_NODE.fullmatch(node) and allow_clock is not True:
                    failures.append("%s subscribes to forbidden simulation clock %s" % (
                        node, topic))
            for node in sorted(publishers.get(topic, ())):
                if node != "/gazebo":
                    failures.append("%s publishes forbidden simulation clock %s" % (
                        node, topic))
        else:
            for node in sorted(subscribers.get(topic, ())):
                failures.append("%s subscribes to forbidden simulation clock alias %s (%s)" % (
                    node, topic, CLOCK_TYPE))
            for node in sorted(publishers.get(topic, ())):
                failures.append("%s publishes forbidden simulation clock alias %s (%s)" % (
                    node, topic, CLOCK_TYPE))

    for topic, nodes in sorted(subscribers.items()):
        if FORBIDDEN_TOPIC.search(topic):
            for node in sorted(nodes):
                if node != JUDGE_NODE and not OFFICIAL_NODE.fullmatch(node):
                    failures.append("%s subscribes to forbidden topic %s" % (node, topic))
    for topic, nodes in sorted(publishers.items()):
        actor = re.fullmatch(r'/actor_([0-5])/cmd_motion', topic)
        if actor:
            allowed = '/actor_' + actor.group(1)
            for node in sorted(nodes):
                if node != allowed:
                    failures.append('%s publishes forbidden environment command %s' % (node, topic))
        elif re.fullmatch(r'/gazebo/(set_model_state|set_link_state)', topic):
            for node in sorted(nodes):
                if node != '/gazebo':
                    failures.append('%s publishes forbidden environment command %s' % (node, topic))
    # Master registrations expose service providers, not callers. This catches
    # replaced environment services; it cannot certify absence of client calls.
    services = _connections(state[2], 'services')
    for service, nodes in services.items():
        if service.startswith('/gazebo/'):
            for node in sorted(nodes):
                if node != '/gazebo':
                    failures.append('%s provides forbidden environment service %s' % (node, service))
    return failures


def positive_seconds(value):
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("timeout must be numeric") from exc
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("timeout must be finite and positive")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=positive_seconds, default=8.0,
                        metavar="SECONDS", help="ROS master socket deadline (default: 8)")
    parser.add_argument("--clock-permission", choices=CLOCK_PERMISSIONS,
                        default=os.environ.get("CLOCK_PERMISSION", "unconfirmed"),
                        help="explicit rule permission for team /clock subscriptions (default: CLOCK_PERMISSION or unconfirmed)")
    parser.add_argument("--clock-permission-reference",
                        default=os.environ.get("CLOCK_PERMISSION_REFERENCE", ""),
                        help="rule or teacher confirmation reference; required when clock permission is allowed")
    parser.add_argument("--allow-judge-pending", action="store_true",
                        help="official judge starts after first takeoff; defer subscriber check")
    args = parser.parse_args(argv)
    # argparse does not validate choices supplied by an environment default.
    if args.clock_permission not in CLOCK_PERMISSIONS:
        print("[graph-audit] ERROR invalid CLOCK_PERMISSION: %s" % args.clock_permission,
              file=sys.stderr)
        return 2
    reference = args.clock_permission_reference.strip()
    if args.clock_permission == "allowed" and not reference:
        print("[graph-audit] ERROR allowed /clock permission requires a nonempty rule or teacher confirmation reference",
              file=sys.stderr)
        return 2
    master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    try:
        state, topic_types = fetch_graph(master_uri, args.timeout)
        failures = audit_graph(state, topic_types,
                               allow_clock=args.clock_permission == "allowed",
                               allow_judge_pending=args.allow_judge_pending)
    except GraphError as exc:
        print("[graph-audit] ERROR %s" % exc, file=sys.stderr)
        return 2
    if failures:
        for failure in failures:
            print("[graph-audit] FAIL %s" % failure, file=sys.stderr)
        return 1
    print("[graph-audit] PASS six ActorInfo channels, target source, judge subscription %s, truth/feedback subscriptions and environment command publishers; clock_permission=%s%s; team /clock subscriptions %s; Clock aliases and foreign clock publishers forbidden (master registration snapshot only; other topic authorization, service clients/files and future subscriptions not certified)" % (
        "deferred" if args.allow_judge_pending else "confirmed",
        args.clock_permission,
        "; clock_permission_reference=%s" % reference if reference else "",
        "explicitly authorized" if args.clock_permission == "allowed" else "forbidden",
    ))
    return 0


if __name__ == "__main__":
    sys.exit(main())
