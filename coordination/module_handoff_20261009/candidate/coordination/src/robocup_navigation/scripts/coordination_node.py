#!/usr/bin/env python3
"""ROS wrapper for the coordination core -- DEVELOPMENT HARNESS, not六机 E2E.

Scope and honesty (interface v0 §5, AGENTS.md):
  * This node is WorkBuddy's transport/plumbing only. It never reinterprets
    clearance, release, timeout or route-version semantics.
  * Without the official multi-UAV launch it is a *dev harness*. A TODO skeleton
    must never be recorded as "six-UAV E2E passed".
  * The single-UAV node's ~goal topic is NOT a ROUTE_GRANT: it must never be
    turned into a flight command without the executor gate authorising it.
  * The coordinator is single-writer: ROS callbacks only enqueue; ONE timer
    thread drives the core. Do not let callbacks touch the core directly.

Transport: JSON strings over std_msgs/String. Message payloads must already
conform to schema v1 (see docs/coordination_interface_v0.md); the adapter stamps
only run_id/seq/sim_s/source_id.

Wiring summary (topics):
  sub  ~<uav>/coordination/vehicle_state   (JSON)  -> VEHICLE_STATE
  sub  ~<uav>/coordination/command_ack     (JSON)  -> COMMAND_ACK
  sub  /coordination/route_offer           (JSON)  -> ROUTE_OFFER   [planner]
  sub  /coordination/resource_clear        (JSON)  -> RESOURCE_CLEAR[verifier]
  sub  /coordination/cancel_task           (JSON)  -> CANCEL_TASK
  sub  /coordination/target_report         (JSON)  -> TARGET_REPORT [perception]
  pub  /coordination/commands              (JSON)  every core output, full stream
  pub  /coordination/events                (JSON)  coord_events mirror
  pub  ~<uav>/coordination/grant           (JSON)  this UAV's ROUTE_GRANT / stop

TODO(official): once the competition multi-UAV launch is frozen, replace the
dev-harness bring-up with it and enable the per-UAV isolation checks (controller
flock, ROS node names, control-topic conflict detection, TF, spawn transforms,
MAVLink id/port, model allow-list) before claiming six-UAV E2E.
"""
import collections
import json
import os
import signal
import sys
import threading
import time

import rospy
from std_msgs.msg import String
from robocup_navigation.srv import SetCoordinationLimits, SetCoordinationLimitsResponse
from robocup_navigation.msg import CoordinationStatus

from robocup_navigation.coordination_adapter import CoreBus, GateFanout, PhysicalMonitor
from robocup_navigation.coordination.protocol import CoordinationError

# 队友栅格订阅 (2026-10)
try:
    from robocup_navigation.team_grid_subscriber import TeamGridSubscriber
    _HAVE_TEAM_GRID = True
except ImportError:
    _HAVE_TEAM_GRID = False

# Dev-harness limits. 支持部分参数覆盖，core.py 会自动填充默认值并验证范围
DEV_LIMITS = dict(min_separation_m=1.0, arrival_tolerance_m=0.15, mission_timeout_s=180.0,
                  deadlock_timeout_s=60.0, max_monitor_gap_s=1.1, state_timeout_s=3.0,
                  lease_s=3.0, stop_speed_mps=0.05, required_clearance_m=0.6,
                  tracking_bound_m=0.2, position_tolerance_m=0.05, nominal_speed_mps=0.3)

# 示例：不同场景的参数配置（可扩展）
SCENARIO_CONFIGS = {
    'indoor_small': dict(min_separation_m=0.8, mission_timeout_s=120.0),
    'indoor_large': dict(min_separation_m=1.2, mission_timeout_s=300.0),
    'outdoor': dict(min_separation_m=2.0, mission_timeout_s=600.0),
}


class CoordinationHarness:
    def __init__(self):
        run_id = rospy.get_param("~run_id", "run_%d" % int(time.time()))
        self.fleet = rospy.get_param("~fleet_ids", ["uav_%d" % i for i in range(1, 7)])
        test_case = rospy.get_param("~test_case_id", "dev_harness")
        
        # 支持场景配置 + 部分参数覆盖
        scenario = rospy.get_param("~scenario", None)
        user_limits = rospy.get_param("~limits", {})
        
        if scenario and scenario in SCENARIO_CONFIGS:
            # 场景配置 + 用户覆盖
            limits = {**SCENARIO_CONFIGS[scenario], **user_limits}
        elif user_limits:
            # 只有用户覆盖
            limits = user_limits
        else:
            # 默认 DEV_LIMITS
            limits = DEV_LIMITS
        
        rospy.loginfo("[coordination] 使用参数: scenario=%s, limits=%s", scenario, limits)
        log_dir = rospy.get_param("~log_dir", os.path.join(os.getcwd(), "coord_logs", run_id))
        os.makedirs(log_dir, exist_ok=True)
        self.run_id = run_id
        self.tick_hz = float(rospy.get_param("~tick_hz", 10.0))
        # Read-only introspection of the core's PUBLIC state, for
        # diagnosing why scheduling produces nothing. It never changes
        # the core and is off unless explicitly enabled.
        self.debug_core = bool(rospy.get_param("~debug_core", False))
        self._last_dbg = -1.0

        # run_id must be globally unique (interface v0 §1); refuse to reuse a log.
        self.events_path = os.path.join(log_dir, "coord_events.jsonl")
        self.commands_path = os.path.join(log_dir, "commands.jsonl")
        self.events_file = open(self.events_path, "x", buffering=1, encoding="utf-8")
        self.commands_file = open(self.commands_path, "x", buffering=1, encoding="utf-8")

        tasks = rospy.get_param("~tasks", {})       # {task_id: {target_id, xyz}}
        self.tasks = tasks
        roles = {u: ["VEHICLE_STATE", "COMMAND_ACK"] for u in self.fleet}
        roles.update(planner=["ROUTE_OFFER"], verifier=["RESOURCE_CLEAR"],
                     clock=["TICK"], perception=["TARGET_REPORT"])
        # The core measures its mission/deadlock budgets from start_sim_s. With
        # a simulator that has been running for a while, the first message can
        # arrive at sim_s ~ 2400 s; seeding the core with 0.0 makes
        # `now - progress` exceed deadlock_timeout_s on the very first TICK, so
        # the core halts and never schedules anything. Seed from the live clock.
        try:
            rospy.wait_for_message("/clock", rospy.AnyMsg, timeout=10.0)
        except Exception:  # noqa: BLE001 - a wall-clock-only master still works
            pass
        start_sim_s = rospy.Time.now().to_sec()
        start_wall_s = time.monotonic()
        rospy.loginfo("[coordination] start_sim_s=%.3f start_wall_s=%.3f",
                      start_sim_s, start_wall_s)
        # 传递 team_grid_provider 给 CoreBus
        team_provider = self._team_subscriber if hasattr(self, '_team_subscriber') else None
        self.bus = CoreBus(run_id, self.fleet, tasks, limits, roles, test_case,
                           start_sim_s=start_sim_s, start_wall_s=start_wall_s,
                           team_grid_provider=team_provider)
        self.fan = GateFanout(run_id, self.fleet, limits["position_tolerance_m"],
                              max(1.0, limits["lease_s"]))
        self.monitor = PhysicalMonitor(self.fleet, limits["min_separation_m"],
                                       limits["arrival_tolerance_m"],
                                       limits["max_monitor_gap_s"],
                                       start_sim_s=start_sim_s)

        # 队友栅格订阅 (2026-10): 协同避让
        self._team_subscriber = None
        if _HAVE_TEAM_GRID:
            try:
                # 从节点名称提取 uav_id，如 "uav_1_coord" -> 0
                node_name = rospy.get_name()  # 如 "/uav_1/coordination_node"
                parts = node_name.strip("/").split("/")
                uav_id = 0
                for p in parts:
                    if p.startswith("uav_"):
                        uav_id = int(p.split("_")[1]) - 1
                        break
                fleet_ids = list(range(6))  # 6架无人机
                self._team_subscriber = TeamGridSubscriber(uav_id=uav_id, fleet_ids=fleet_ids)
                rospy.loginfo("[team_grid] 协同层订阅队友栅格: uav_id=%d", uav_id)
            except Exception as e:
                rospy.logwarn("[team_grid] 初始化失败: %s", e)

        # Dev-harness placeholders for the executor fan-out.  TODO(executor):
        # replace with the LOCAL map revision and each UAV's MEASURED world_xyz.
        self._positions = {u: [0.0, 0.0, 0.0] for u in self.fleet}
        # Only UAVs that have actually reported may be monitored:/n        # the zero defaults are placeholders, and sampling them made
        # the whole fleet look co-located (separation 0 -> COLLISION).
        self._reported = set()
        self._arrival_seen = set()
        self._map_revision = 1

        # Single-writer queue: callbacks enqueue, the timer drains.
        self._q = collections.deque()
        self._lock = threading.Lock()

        self.cmd_pub = rospy.Publisher("/coordination/commands", String, queue_size=200)
        self.event_pub = rospy.Publisher("/coordination/events", String, queue_size=200)

        self._subs = []
        for u in self.fleet:
            self._subs.append(rospy.Subscriber("/%s/coordination/vehicle_state" % u, String,
                                               self._in(u, "VEHICLE_STATE"), queue_size=50))
            self._subs.append(rospy.Subscriber("/%s/coordination/command_ack" % u, String,
                                               self._in(u, "COMMAND_ACK"), queue_size=50))
        self._subs.append(rospy.Subscriber("/coordination/route_offer", String,
                                           self._in("planner", "ROUTE_OFFER"), queue_size=50))
        self._subs.append(rospy.Subscriber("/coordination/resource_clear", String,
                                           self._in("verifier", "RESOURCE_CLEAR"), queue_size=50))
        self._subs.append(rospy.Subscriber("/coordination/cancel_task", String,
                                           self._in("planner", "CANCEL_TASK"), queue_size=50))
        self._subs.append(rospy.Subscriber("/coordination/target_report", String,
                                           self._in("perception", "TARGET_REPORT"), queue_size=50))
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.tick_hz), self._tick)

        rospy.loginfo("[coordination] dev harness up run_id=%s fleet=%s log=%s",
                      run_id, self.fleet, log_dir)

        # Without this the process is killed mid-run and the core never emits
        # RUN_FINISHED, so the verdict can only ABSTAIN.
        rospy.on_shutdown(self.finish)
        for _sig in (signal.SIGTERM, signal.SIGINT):
            try:
                signal.signal(_sig, lambda *_a: (self.finish(), rospy.signal_shutdown("signal")))
            except Exception:  # noqa: BLE001
                pass
        
        # 动态调参服务 (国家一等奖标准：支持运行时参数调整)
        self._limits_service = rospy.Service("~set_limits", SetCoordinationLimits, self._set_limits_srv)
        self._status_pub = rospy.Publisher("~status", CoordinationStatus, queue_size=10)
        rospy.loginfo("[coordination] 动态调参服务已启用")

    def _set_limits_srv(self, req):
        """动态调整协同参数"""
        try:
            from robocup_navigation.coordination.core import _validate_limits, PARAM_RANGES
            
            # 验证新参数
            new_limits = _validate_limits(req.limits)
            
            # 更新 core 的 limits
            self.bus.core.limits = new_limits
            self.bus.core.limits = new_limits  # 确保更新
            
            rospy.loginfo("[coordination] 参数已更新: %s", new_limits)
            return SetCoordinationLimitsResponse(success=True, message="参数更新成功")
        except Exception as e:
            rospy.logerr("[coordination] 参数更新失败: %s", e)
            return SetCoordinationLimitsResponse(success=False, message=str(e))

    def _in(self, source_id, kind):
        def cb(msg):
            try:
                data = json.loads(msg.data)
            except ValueError as exc:
                rospy.logerr("[coordination] bad JSON for %s/%s: %s", source_id, kind, exc)
                return
            if kind == "VEHICLE_STATE" and isinstance(data.get("xyz"), list):
                with self._lock:
                    self._positions[data.get("uav_id", source_id)] = list(data["xyz"])
                    self._reported.add(data.get("uav_id", source_id))
            with self._lock:
                self._q.append((source_id, kind, data))
        return cb

    def _tick(self, _evt=None):
        self.process_queue()
        # TICK drives deadline checks; also covers gaps when no packet arrives.
        self._submit("clock", "TICK", {})
        self._sample_monitor()

        # 队友栅格障碍检测 (2026-10): 检测队友附近是否有障碍物
        self._check_team_obstacles()

    def _check_team_obstacles(self):
        """检测队友位置附近是否有障碍物，用于协同避让"""
        if not self._team_subscriber:
            return

        # 处理ROS回调
        self._team_subscriber.spin_once()

        # 获取合并后的7m栅格
        team_grid = self._team_subscriber.get_combined_grid(max_age_s=10.0)
        if team_grid is None:
            return

        # 检查每个队友的位置是否有障碍物
        import numpy as np
        team_grid = np.array(team_grid)
        CELL_7M = 7.0
        X0, Y0 = -52.0, -52.0

        with self._lock:
            positions = dict(self._positions)

        obstacles_detected = []
        for uav, pos in positions.items():
            if len(pos) < 2:
                continue
            wx, wy = pos[0], pos[1]

            # 将世界坐标转换为7m栅格坐标
            gx = int((wx - X0) / CELL_7M)
            gy = int((wy - Y0) / CELL_7M)

            # 检查3x3邻域内是否有障碍物
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= ny < team_grid.shape[0] and 0 <= nx < team_grid.shape[1]:
                        if team_grid[ny, nx] > 0:  # OCCUPIED
                            obstacles_detected.append((uav, wx, wy))
                            break

        # 记录警告（可以触发更复杂的避让逻辑）
        if obstacles_detected:
            rospy.logwarn_throttle(5.0, "[team_grid] 队友附近有障碍物: %s", obstacles_detected)

    def _sample_monitor(self):
        """Feed the independent monitor from measured positions only."""
        with self._lock:
            if len(self._reported) < len(self.fleet):
                return  # never turn a placeholder into a measurement
            positions = {u: list(p) for u, p in self._positions.items()
                         if u in self._reported}
        targets = {}
        try:
            snap = self.bus.core.snapshot()
            for tid, t in (snap.get("tasks") or {}).items():
                owner = t.get("owner") or t.get("owner_uav_id")
                if not owner or tid not in self.tasks:
                    continue
                # "Arrival error" is the error AT ARRIVAL. Feeding the target on
                # every sample makes the run's maximum distance-to-target (i.e.
                # the transit distance) the "arrival error", so a target farther
                # away than arrival_tolerance_m can never pass. Record it once,
                # at the sample where the task completes.
                if t.get("complete") and tid not in self._arrival_seen:
                    self._arrival_seen.add(tid)
                    targets[owner] = list(self.tasks[tid]["xyz"])
        except Exception:  # noqa: BLE001 - monitoring must never break the loop
            targets = {}
        self.monitor.sample(rospy.Time.now().to_sec(), positions, targets, progressed=True)
        self._debug_core(positions)

    def _debug_core(self, positions):
        if not self.debug_core:
            return
        now = rospy.Time.now().to_sec()
        if now - self._last_dbg < 8.0:
            return
        self._last_dbg = now
        try:
            core = self.bus.core
            rospy.loginfo("[coord-debug] halted=%s finished=%s offers=%d vehicles=%d "
                          "tasks=%s", core.halted, core.finished, len(core.offers),
                          len(core.vehicles),
                          {k: v["status"] for k, v in core.tasks.items()})
            rospy.loginfo("[coord-debug] now=%.2f progress=%.2f started=%.2f deadlock_t=%.1f",
                          core.now, core.progress, core.started,
                          core.limits["deadlock_timeout_s"])
            for (u, tid), o in list(core.offers.items()):
                rospy.loginfo("[coord-debug] %s/%s valid_until=%.1f now=%.1f safe=%s "
                              "conflict=%s stopped=%s fresh=%s", u, tid,
                              o["valid_until_sim_s"], core.now,
                              core._offer_safe(o), core._conflict(o["points"], u),
                              core._stopped(u), core._fresh(u))
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("[coord-debug] failed: %s", exc)

    def finish(self, *_args):
        """Let the core close the run so the verdict has a RUN_FINISHED."""
        if getattr(self, "_finished", False):
            return
        self._finished = True
        try:
            self._sample_monitor()
            self.bus.finish(self.monitor.report())
            self._flush_events()
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("[coordination] finish failed: %s", exc)
        rospy.loginfo("[coordination] run finished; log=%s", self.log_dir)

    def process_queue(self):
        while True:
            with self._lock:
                if not self._q:
                    return
                item = self._q.popleft()
            self._submit(*item)

    def _submit(self, source_id, kind, data):
        sim_s = rospy.Time.now().to_sec()
        wall_s = time.monotonic()
        try:
            outputs = self.bus.submit(source_id, kind, data, sim_s, wall_s)
        except CoordinationError as exc:
            rospy.logwarn("[coordination] core rejected %s from %s: %s", kind, source_id, exc)
            self._flush_events()
            return
        with self._lock:
            positions = dict(self._positions)
        for m in outputs:
            payload = json.dumps(m, sort_keys=True, allow_nan=False)
            self.commands_file.write(payload + "\n")
            self.cmd_pub.publish(String(payload))
            # Fan-out needs the LOCAL map revision and each UAV's MEASURED
            # world_xyz.  The placeholders below keep the harness runnable; they
            # are NOT flight-worthy and Gazebo truth must never reach the core.
            self.fan.deliver([m], sim_s, wall_s, self._map_revision,
                             lambda u: positions.get(u, [0.0, 0.0, 0.0]))
        self._flush_events()

    def _flush_events(self):
        for e in self.bus.drain_events():
            line = json.dumps(e, sort_keys=True, allow_nan=False)
            self.events_file.write(line + "\n")
            self.event_pub.publish(String(line))


def main():
    rospy.init_node("coordination", anonymous=False)
    CoordinationHarness()
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
