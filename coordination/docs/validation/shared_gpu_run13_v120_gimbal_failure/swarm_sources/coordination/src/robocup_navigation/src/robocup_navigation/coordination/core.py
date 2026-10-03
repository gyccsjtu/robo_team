"""Single-writer fleet coordinator: assignment, fencing and spatial reservations.

Routes are reserved in their entirety until verified clear. This deliberately
does not extrapolate a future release from an arrival estimate. Transport,
map construction, ROS and physical safety monitors belong to adapters.
"""
import copy
import hashlib
import json
import math

from .geometry import matching, route_distance, route_length, route_distance_with_time, estimate_route_times
from .protocol import CoordinationError, number, string, validate, xyz


DEFAULT_KEYS = {
    'min_separation_m', 'arrival_tolerance_m', 'mission_timeout_s',
    'deadlock_timeout_s', 'max_monitor_gap_s', 'state_timeout_s',
    'lease_s', 'stop_speed_mps', 'required_clearance_m',
    'tracking_bound_m', 'position_tolerance_m', 'nominal_speed_mps',
}

# 参数默认值（RoboCup 室内场景）
DEFAULT_LIMITS = {
    'min_separation_m': 1.0,
    'arrival_tolerance_m': 0.15,
    'mission_timeout_s': 180.0,
    'deadlock_timeout_s': 60.0,
    'max_monitor_gap_s': 1.1,
    'state_timeout_s': 3.0,
    'lease_s': 3.0,
    'stop_speed_mps': 0.05,
    'required_clearance_m': 0.6,
    'tracking_bound_m': 0.2,
    'position_tolerance_m': 0.05,
    'nominal_speed_mps': 0.3,
}

# 参数合理范围验证 (min, max)
PARAM_RANGES = {
    'min_separation_m': (0.3, 5.0),
    'arrival_tolerance_m': (0.05, 1.0),
    'mission_timeout_s': (10.0, 600.0),
    'deadlock_timeout_s': (5.0, 300.0),
    'max_monitor_gap_s': (0.1, 10.0),
    'state_timeout_s': (0.5, 30.0),
    'lease_s': (0.5, 30.0),
    'stop_speed_mps': (0.01, 0.5),
    'required_clearance_m': (0.1, 3.0),
    'tracking_bound_m': (0.05, 2.0),
    'position_tolerance_m': (0.01, 0.5),
    'nominal_speed_mps': (0.05, 2.0),
}


def _validate_limits(limits):
    """验证参数：支持部分覆盖，自动填充默认值，验证范围"""
    # 合并默认参数和用户提供的参数
    merged = {**DEFAULT_LIMITS, **limits}
    
    # 检查是否包含所有必需键
    if not set(merged.keys()) >= DEFAULT_KEYS:
        missing = DEFAULT_KEYS - set(merged.keys())
        raise CoordinationError(f'CONFIG_LIMITS_MISSING: {missing}')
    
    # 验证每个参数
    for key, value in merged.items():
        if not number(value) or value <= 0:
            raise CoordinationError(f'CONFIG_LIMITS_VALUE: {key}={value} must be > 0')
        if key in PARAM_RANGES:
            min_val, max_val = PARAM_RANGES[key]
            if not min_val <= value <= max_val:
                raise CoordinationError(f'CONFIG_LIMITS_RANGE: {key}={value} not in [{min_val}, {max_val}]')
    
    return merged


class Coordinator:
    """Pure deterministic core. Call receive(message, received_wall_s) serially.

    fleet: logical UAV ids, at most six.
    tasks: {task_id: {'target_id': str, 'xyz': [x,y,z]}}.
    roles: {source_id: [permitted input kinds]}; vehicle reports and ACKs
    additionally require source_id == uav_id. Source authentication is external.
    Inputs, outputs and public snapshots are copied to prevent alias mutation.
    """

    def __init__(self, run_id, fleet, tasks, limits, roles, test_case_id,
                 start_sim_s=0., start_wall_s=0., team_grid_provider=None):
        """初始化协同器
        
        Args:
            team_grid_provider: 可选的队友栅格提供者，用于协同决策
                               需要实现 get_team_grid() 方法返回合并后的栅格
        """
        if not string(run_id) or not string(test_case_id) or not number(start_sim_s) or not number(start_wall_s):
            raise CoordinationError('CONFIG_RUN')
        if not isinstance(fleet, (list, tuple)) or not 1 <= len(fleet) <= 6 or len(set(fleet)) != len(fleet):
            raise CoordinationError('CONFIG_FLEET')
        if any(u not in ['uav_%d' % i for i in range(1, 7)] for u in fleet):
            raise CoordinationError('CONFIG_UAV_ID')
        # 使用新的验证函数：支持部分覆盖 + 范围验证
        validated_limits = _validate_limits(limits)
        if limits['position_tolerance_m'] > limits['tracking_bound_m']:
            raise CoordinationError('CONFIG_START_OUTSIDE_TRACKING_BOUND')
        if not tasks or any(not string(k) or not isinstance(v, dict) or
                            not {'target_id', 'xyz'}.issubset(set(v.keys())) or not string(v['target_id']) or
                            not xyz(v['xyz']) for k, v in tasks.items()):
            raise CoordinationError('CONFIG_TASKS')
        if len({v['target_id'] for v in tasks.values()}) != len(tasks):
            raise CoordinationError('CONFIG_DUPLICATE_TARGET')
        if not isinstance(roles, dict) or any(not string(k) or not isinstance(v, (list, tuple))
                                             for k, v in roles.items()):
            raise CoordinationError('CONFIG_ROLES')
        self.run_id, self.fleet = run_id, tuple(sorted(fleet))
        self.limits, self.roles = validated_limits, copy.deepcopy(roles)
        self.team_grid_provider = team_grid_provider  # 队友栅格提供者
        self.tasks = {k: dict(copy.deepcopy(v), status='PENDING', epoch=0, owner=None,
                              stopped=False, stop_seq=None, lease=0., version=0,
                              reservation=None, command=None, complete=False, cancel=False,
                              priority=v.get('priority', 50))  # 默认优先级50，0-100越高越优先
                      for k, v in tasks.items()}
        
        # 死锁预防：记录等待图
        self._wait_graph = {}  # {uav_id: waiting_for_task_id}
        self._deadlock_detection_enabled = True
        self._conflict_history = {}  # 冲突历史，用于预测性避让
        self.vehicles, self.offers, self.reservations, self.commands = {}, {}, {}, {}
        self.retired = set()
        self.sequences, self.observations = {}, {}
        self.now, self.wall, self.started = start_sim_s, start_wall_s, start_sim_s
        self.progress = start_sim_s
        self.halted = self.finished = False
        self.failures, self.outbox, self.events = [], [], []
        self.output_seq = self.event_seq = self.counter = 0
        digest = hashlib.sha256(json.dumps(dict(fleet=fleet, tasks=tasks, limits=limits, roles=roles),
                                           sort_keys=True).encode()).hexdigest()
        self._event('RUN_STARTED', details=dict(fleet_ids=list(self.fleet),
                    required_task_ids=sorted(tasks), test_case_id=test_case_id,
                    config_digest=digest, limits=copy.deepcopy(limits)))

    def _message(self, source, seq, kind, data):
        return dict(schema_version=1, run_id=self.run_id, source_id=source,
                    seq=seq, sim_s=self.now, kind=kind, data=copy.deepcopy(data))

    def _emit(self, kind, **data):
        self.output_seq += 1
        msg = self._message('coordinator_commands', self.output_seq, kind, data)
        self.outbox.append(msg)
        return msg

    def _event(self, event, uav=None, task=None, reason='OK', details=None):
        self.event_seq += 1
        self.events.append(self._message('coordinator', self.event_seq, 'EVENT',
                           dict(event=event, uav_id=uav, task_id=task,
                                reason=reason, details=details or {})))

    def drain_outputs(self):
        result, self.outbox = copy.deepcopy(self.outbox), []
        return result

    def drain_events(self):
        result, self.events = copy.deepcopy(self.events), []
        return result

    def snapshot(self):
        return copy.deepcopy(dict(tasks=self.tasks, reservations=self.reservations,
                                  halted=self.halted, failures=self.failures))

    def _fresh(self, u):
        s = self.vehicles.get(u)
        return s is not None and self.wall - s['wall'] <= self.limits['state_timeout_s'] and \
            0 <= self.now - s['sim'] <= self.limits['state_timeout_s']

    def _stopped(self, u):
        return self._fresh(u) and math.dist(self.vehicles[u]['velocity_xyz'], (0, 0, 0)) <= self.limits['stop_speed_mps']

    def _lock(self, tid, state, reason):
        t = self.tasks[tid]
        data = dict(task_id=tid, target_id=t['target_id'], owner_uav_id=t['owner'],
                    epoch=t['epoch'], state=state, lease_until_sim_s=t['lease'], reason=reason)
        self._emit('TARGET_LOCK', **data)
        self._event('LOCK_CHANGED', t['owner'], tid, reason, data)

    def _reservation(self, rid, state):
        r = self.reservations[rid]
        r['state'] = state
        data = {k: r[k] for k in ('reservation_id', 'uav_id', 'task_id', 'epoch', 'route_version',
                                 'resource_id', 'enter_after_sim_s', 'expected_exit_sim_s')}
        data['state'] = state
        self._emit('RESERVATION', **data)
        self._event('RESERVATION_CHANGED', r['uav_id'], r['task_id'], details=data)

    def _revoke(self, tid, reason, quarantine=False):
        t = self.tasks[tid]
        if t['owner'] is None or t['status'] in ('COMPLETED', 'CANCELLED'):
            return
        if t['status'] in ('REVOKING', 'QUARANTINED'):
            return
        t['status'], t['stopped'], t['stop_seq'] = ('QUARANTINED' if quarantine else 'REVOKING'), False, None
        self._lock(tid, t['status'], reason)
        if t['reservation']:
            self._reservation(t['reservation'], 'QUARANTINED')
        self.counter += 1
        cid = 'stop-%d' % self.counter
        self.commands[cid] = dict(task=tid, kind='CANCEL_REQUEST', epoch=t['epoch'],
                                  version=t['version'], uav=t['owner'], ack=None)
        t['command'] = cid
        self._emit('CANCEL_REQUEST', command_id=cid, uav_id=t['owner'], task_id=tid,
                   epoch=t['epoch'], route_version=t['version'], reason=reason)

    def _violation(self, reason):
        if reason not in self.failures:
            self.failures.append(reason)
            self._event('SAFETY_VIOLATION', reason=reason)
        self.halted = True
        for tid in self.tasks:
            self._revoke(tid, reason, quarantine=True)

    def receive(self, message, received_wall_s):
        """Malformed/unauthorized/out-of-order input raises without mutation.

        Semantic rejections consume a valid sequence (caller must not retry it
        with changed data). Duplicate delivery has no effects or outputs.
        """
        canonical = validate(message)
        m = copy.deepcopy(message)
        if self.finished:
            raise CoordinationError('RUN_FINISHED')
        if m['run_id'] != self.run_id:
            raise CoordinationError('RUN_MISMATCH')
        src, kind, d = m['source_id'], m['kind'], m['data']
        if kind not in self.roles.get(src, ()):
            raise CoordinationError('SOURCE_NOT_AUTHORIZED')
        if 'uav_id' in d and d['uav_id'] not in self.fleet:
            raise CoordinationError('UNKNOWN_UAV')
        if kind in ('VEHICLE_STATE', 'COMMAND_ACK') and src != d['uav_id']:
            raise CoordinationError('UAV_IDENTITY_MISMATCH')
        prev = self.sequences.get(src)
        if prev and m['seq'] == prev[0]:
            if canonical != prev[1]:
                raise CoordinationError('SEQ_CONTENT_CONFLICT')
            return []
        if m['seq'] != (prev[0] + 1 if prev else 1):
            raise CoordinationError('SEQ_GAP_OR_OLD')
        if not number(received_wall_s) or received_wall_s < self.wall:
            raise CoordinationError('RECEIVE_CLOCK_INVALID')
        if prev and m['sim_s'] < prev[2]:
            # Delayed source stamps are not a global clock rewind. Reject them.
            raise CoordinationError('SOURCE_TIME_REWOUND')
        if kind == 'TICK' and m['sim_s'] < self.now:
            self.halted = True
            for tid in self.tasks:
                self._revoke(tid, 'CLOCK_REWOUND', quarantine=True)
            raise CoordinationError('CLOCK_REWOUND_NEW_RUN_REQUIRED')
        if m['sim_s'] < self.now - self.limits['state_timeout_s']:
            raise CoordinationError('MESSAGE_STALE')
        self.sequences[src] = (m['seq'], canonical, m['sim_s'])
        self.now, self.wall = max(self.now, m['sim_s']), received_wall_s
        offset = len(self.outbox)
        # Expiry precedes renewal: a late heartbeat cannot resurrect an epoch.
        self._watchdogs()
        if kind == 'VEHICLE_STATE':
            self._state(d, m)
        elif kind == 'TARGET_REPORT':
            self._target(d)
        elif kind == 'ROUTE_OFFER':
            if d['task_id'] not in self.tasks:
                raise CoordinationError('UNKNOWN_TASK')
            self.offers[d['uav_id'], d['task_id']] = d
        elif kind == 'COMMAND_ACK':
            self._ack(d)
        elif kind == 'RESOURCE_CLEAR':
            self._clear(d)
        elif kind == 'CANCEL_TASK':
            if d['task_id'] not in self.tasks:
                raise CoordinationError('UNKNOWN_TASK')
            t = self.tasks[d['task_id']]
            t['cancel'] = True
            if t['owner'] is None:
                t['status'] = 'CANCELLED'
            else:
                self._revoke(d['task_id'], d['reason'])
        self._watchdogs()
        if kind == 'TICK' and not self.halted:
            self._schedule()
        return copy.deepcopy(self.outbox[offset:])

    def _state(self, d, m):
        u = d['uav_id']
        old = self.vehicles.get(u)
        self.vehicles[u] = dict(d, wall=self.wall, sim=m['sim_s'], seq=m['seq'])
        active = next(((tid, t) for tid, t in self.tasks.items() if t['owner'] == u and
                       t['status'] not in ('COMPLETED', 'CANCELLED')), None)
        if active:
            tid, t = active
            if d['health'] != 'OK':
                self._revoke(tid, 'VEHICLE_UNHEALTHY', quarantine=True)
            elif d['observed_epoch'] == t['epoch'] and d['route_version'] == t['version'] and \
                    d['active_command_id'] == t['command'] and t['status'] in ('ASSIGNED', 'EXECUTING'):
                t['lease'] = self.now + self.limits['lease_s']
                self._lock(tid, 'HELD', 'RENEWED')
                r = self.reservations[t['reservation']]
                if route_distance(r['points'], [d['xyz'], d['xyz']]) > self.limits['tracking_bound_m']:
                    self._violation('AUTHORIZATION_CONFLICT')
                if d['map_revision'] != r['map_revision']:
                    self._revoke(tid, 'MAP_CHANGED')
            elif d['mode'] == 'EXECUTING' and d['active_command_id'] != t['command']:
                # During revocation, old movement is possible and stays reserved.
                if t['status'] not in ('REVOKING', 'QUARANTINED'):
                    self._violation('STALE_COMMAND_EXECUTED')
            if old and t['status'] == 'EXECUTING':
                # Count monotone distance along the route, not straight-line
                # distance to goal (a valid detour initially moves away).
                points = self.reservations[t['reservation']]['points']
                best, walked = None, 0.
                for a, b in zip(points, points[1:]):
                    delta = [b[i] - a[i] for i in range(3)]
                    length = math.dist(a, b)
                    fraction = max(0., min(1., sum((d['xyz'][i] - a[i]) * delta[i] for i in range(3)) /
                                          (length * length))) if length else 0.
                    q = [a[i] + fraction * delta[i] for i in range(3)]
                    item = (math.dist(d['xyz'], q), walked + fraction * length)
                    best = item if best is None or item < best else best
                    walked += length
                if best[1] - t.get('route_progress', 0.) > self.limits['position_tolerance_m']:
                    t['route_progress'], self.progress = best[1], self.now
        elif d['mode'] == 'EXECUTING':
            self._violation('STALE_COMMAND_EXECUTED')
        for other in self.fleet:
            if other != u and self._fresh(other) and math.dist(d['xyz'], self.vehicles[other]['xyz']) < self.limits['min_separation_m']:
                self._violation('SEPARATION_BREACH')

    def _target(self, d):
        key = d['target_id'], d['observation_id']
        if key in self.observations:
            if self.observations[key] != d:
                raise CoordinationError('OBSERVATION_CONFLICT')
            return
        self.observations[key] = d
        if d['confidence'] < 1.:
            return  # v1 consumes confirmed reports; upstream owns fusion.
        for tid, t in self.tasks.items():
            if t['target_id'] == d['target_id'] and t['status'] != 'COMPLETED':
                moved = math.dist(t['xyz'], d['xyz']) > self.limits['arrival_tolerance_m']
                t['xyz'] = d['xyz']
                if moved and t['owner']:
                    self._revoke(tid, 'TARGET_MOVED')

    def _offer_safe(self, offer):
        u, tid = offer['uav_id'], offer['task_id']
        if not self._stopped(u):
            return False
        s, t, l = self.vehicles[u], self.tasks[tid], self.limits
        return (s['health'] == 'OK' and s['mode'] in ('IDLE', 'HOLDING', 'LANDED') and
                offer['valid_until_sim_s'] > self.now and offer['map_revision'] == s['map_revision'] and
                offer['static_safe'] and offer['grid_safe'] and
                offer['clearance_m'] >= l['required_clearance_m'] and
                offer['tracking_bound_m'] <= l['tracking_bound_m'] and
                math.dist(offer['points'][0], s['xyz']) <= l['position_tolerance_m'] and
                math.dist(offer['points'][-1], t['xyz']) <= l['arrival_tolerance_m'])

    def _conflict(self, points, owner):
        margin = self.limits['min_separation_m'] + 2 * self.limits['tracking_bound_m']
        for r in self.reservations.values():
            if r['state'] != 'RELEASED' and r['uav_id'] != owner and route_distance(points, r['points']) <= margin:
                return True
        for u, s in self.vehicles.items():
            if u != owner and route_distance(points, [s['xyz'], s['xyz']]) <= margin:
                return True
        return False

    def safe_waiting_points(self, offer):
        """Filter trusted planner candidates; synthesize wait point if needed."""
        from .protocol import fields, SPECS
        fields(offer, SPECS['ROUTE_OFFER'])
        if offer['uav_id'] not in self.fleet or not self._fresh(offer['uav_id']):
            return []
        current = self.vehicles[offer['uav_id']]
        
        # 先过滤外部提供的等待点
        external = [p for p in offer['waiting_points']
                    if p['static_safe'] and p['grid_safe'] and p['connector_safe'] and
                    p['outside_bottleneck'] and p['map_revision'] == current['map_revision'] and
                    p['valid_until_sim_s'] > self.now and
                    p['clearance_m'] >= self.limits['required_clearance_m'] and
                    p['tracking_bound_m'] <= self.limits['tracking_bound_m'] and
                    not self._conflict([current['xyz'], p['xyz']], offer['uav_id'])]
        
        if external:
            return copy.deepcopy(external)
        
        # 如果没有外部等待点，尝试自动生成
        return self._generate_waiting_points(offer, current)

    def _generate_waiting_points(self, offer, current):
        """自动生成等待点：当路径冲突时，在当前位置附近寻找安全等待位置"""
        # 计算冲突区域：在路径的哪个位置与他人冲突
        own_points = offer['points']
        conflict_points = []
        
        for r in self.reservations.values():
            if r['state'] == 'RELEASED' or r['uav_id'] == offer['uav_id']:
                continue
            dist = route_distance(own_points, r['points'])
            if dist < self.limits['min_separation_m'] + 2 * self.limits['tracking_bound_m']:
                # 找到冲突区域，记录冲突点
                conflict_points.extend(r['points'])
        
        if not conflict_points:
            return []
        
        # 在当前位置的各个方向生成候选等待点
        candidates = []
        current_xyz = current['xyz']
        
        # 6个方向：前后左右上下
        directions = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        wait_distance = self.limits['min_separation_m'] * 1.5  # 等待距离
        
        for dx, dy, dz in directions:
            wait_point = [
                current_xyz[0] + dx * wait_distance,
                current_xyz[1] + dy * wait_distance,
                current_xyz[2] + dz * wait_distance
            ]
            
            # 检查等待点是否安全（不与任何 reservation 冲突）
            if not self._conflict([current_xyz, wait_point], offer['uav_id']):
                candidates.append({
                    'point_id': f'auto_wait_{len(candidates)}',
                    'xyz': wait_point,
                    'map_revision': current['map_revision'],
                    'valid_until_sim_s': self.now + self.limits['lease_s'] * 2,
                    'clearance_m': self.limits['required_clearance_m'],
                    'tracking_bound_m': self.limits['tracking_bound_m'],
                    'static_safe': True,
                    'grid_safe': True,
                    'connector_safe': True,
                    'outside_bottleneck': True,
                })
        
        return copy.deepcopy(candidates[:3])  # 最多返回3个候选

    def _compute_spacetime_conflict_penalty(self, points, owner_uav):
        """计算路径的时空冲突惩罚成本
        
        使用时空冲突检测，如果路径与其他无人机路径在时间上重叠且空间接近，
        则给予较高的惩罚成本，降低该路径的优先级
        """
        penalty = 0.0
        speed = self.limits['nominal_speed_mps']
        
        # 估计自己路径的时间
        times = estimate_route_times(points, speed, self.now)
        
        for r in self.reservations.values():
            if r['state'] == 'RELEASED' or r['uav_id'] == owner_uav:
                continue
            
            # 估计对方路径的时间
            other_points = r['points']
            other_times = estimate_route_times(other_points, speed, r['enter_after_sim_s'])
            
            # 时空冲突检测
            min_dist, time_overlap = route_distance_with_time(
                points, other_points, times, other_times, speed
            )
            
            # 空间距离阈值
            threshold = self.limits['min_separation_m'] + 2 * self.limits['tracking_bound_m']
            
            if min_dist < threshold and time_overlap > 0:
                # 冲突：距离越近、时间重叠越多，惩罚越高
                conflict_factor = (threshold - min_dist) / threshold  # 0-1，越近越大
                penalty += time_overlap * conflict_factor * 10.0
        
        return penalty

    def _compute_team_bonus(self, points, team_grid_provider):
        """计算路径使用队友已知区域的奖励
        
        如果路径经过队友已经探索过的无障碍区域，给予奖励（降低 cost）
        这鼓励利用队友的地图信息选择更安全的路径
        """
        if team_grid_provider is None:
            return 0.0
        
        team_grid = team_grid_provider.get_team_grid()
        if team_grid is None:
            return 0.0
        
        # 计算路径上有多少个点位于队友已探索的无障碍区域
        bonus = 0.0
        for point in points:
            # 检查该点是否在队友已知的安全区域内
            # team_grid 是 7m 分辨率的栅格
            grid_x = int((point[0] + 52) / 7)
            grid_y = int((point[1] + 52) / 7)
            
            if 0 <= grid_x < 26 and 0 <= grid_y < 14:
                # 队友已知该区域（不为 Unknown）
                if team_grid[grid_y, grid_x] == 0:  # FREE
                    bonus += 0.5  # 每个安全点给予奖励
        
        return -bonus  # 负奖励 = 降低总成本

    def _detect_deadlock(self):
        """检测是否形成死锁（循环等待）"""
        for tid, t in self.tasks.items():
            if t['status'] == 'PENDING' and t['owner'] is None:
                wait_time = self.now - t.get('epoch', 0)
                if wait_time > self.limits['deadlock_timeout_s']:
                    self._event('DEADLOCK_SUSPECTED', task=tid, 
                               reason=f'wait_time={wait_time:.1f}s > {self.limits["deadlock_timeout_s"]}s')
                    return True
        return False

    def _resolve_deadlock(self):
        """解决死锁：提升等待最久的任务优先级"""
        self._event('DEADLOCK_RESOLUTION', reason='Initiating deadlock resolution')
        max_wait_time, oldest_task = 0, None
        for tid, t in self.tasks.items():
            if t['status'] == 'PENDING' and t['owner'] is None:
                wait_time = self.now - t.get('epoch', 0)
                if wait_time > max_wait_time:
                    max_wait_time, oldest_task = wait_time, tid
        if oldest_task:
            self.tasks[oldest_task]['priority'] = min(100, self.tasks[oldest_task]['priority'] + 20)
            self._event('DEADLOCK_RESOLVED', task=oldest_task, 
                       reason=f'priority_boosted_to_{self.tasks[oldest_task]["priority"]}')

    def _schedule(self):
        # An unobserved/lost aircraft could be anywhere: no new motion grants.
        if any(t['status'] in ('REVOKING', 'QUARANTINED') for t in self.tasks.values()):
            return
        if not all(u in self.retired or (self._fresh(u) and self.vehicles[u]['health'] == 'OK') for u in self.fleet):
            return
        busy = {t['owner'] for t in self.tasks.values() if t['owner'] is not None}
        
        # 任务优先级调度：先按优先级降序，再按等待时间升序
        pending_with_priority = [
            (tid, self.tasks[tid].get('priority', 50), self.tasks[tid].get('epoch', 0))
            for tid, t in self.tasks.items() if t['status'] == 'PENDING'
        ]
        # 排序：(优先级降序, 等待时间升序)
        pending_with_priority.sort(key=lambda x: (-x[1], x[2]))
        pending = [x[0] for x in pending_with_priority]
        
        available = [u for u in self.fleet if u not in busy and u not in self.retired]
        
        # 死锁预防：检测是否形成循环等待
        if self._deadlock_detection_enabled and self._detect_deadlock():
            self._resolve_deadlock()
        
        # 改进的成本函数：多目标优化
        # 成本 = 飞行时间 + 任务紧急度惩罚 + 负载均衡惩罚 + 时空冲突惩罚
        costs = {}
        for (u, tid), o in self.offers.items():
            if u not in available or tid not in pending or not self._offer_safe(o):
                continue
            
            # 1. 基础飞行时间成本
            base_cost = route_length(o['points']) / self.limits['nominal_speed_mps']
            
            # 2. 任务紧急度惩罚（任务等待越久越优先）
            task = self.tasks[tid]
            wait_time = self.now - task.get('epoch', 0)
            urgency_penalty = wait_time * 0.1  # 等待时间加权
            
            # 3. 负载均衡惩罚（避免某些无人机过于繁忙）
            # 计算每架无人机当前任务数
            current_load = sum(1 for t in self.tasks.values() if t['owner'] == u and t['status'] in ('ASSIGNED', 'EXECUTING'))
            load_penalty = current_load * 2.0  # 当前任务数加权
            
            # 4. 时空冲突惩罚（如果路径与其他无人机时空接近）
            conflict_penalty = self._compute_spacetime_conflict_penalty(o['points'], u)
            
            # 5. 队友地图奖励（如果路径经过队友已探索的无障碍区域）
            team_bonus = self._compute_team_bonus(o['points'], self.team_grid_provider)
            
            costs[u, tid] = base_cost + urgency_penalty + load_penalty + conflict_penalty + team_bonus
        for u, tid in matching(available, pending, costs):
            offer = self.offers[u, tid]
            if self._conflict(offer['points'], u):
                continue  # stay in current safe state; retry on a future TICK
            t = self.tasks[tid]
            t.update(owner=u, epoch=t['epoch'] + 1, version=t['version'] + 1,
                     status='ASSIGNED', stopped=False, stop_seq=None, complete=False,
                     lease=self.now + self.limits['lease_s'], route_progress=0.)
            self.counter += 1
            rid, cid = 'route-%d' % self.counter, 'fly-%d' % self.counter
            t['reservation'], t['command'] = rid, cid
            self.reservations[rid] = dict(reservation_id=rid, uav_id=u, task_id=tid,
                epoch=t['epoch'], route_version=t['version'], resource_id=rid,
                enter_after_sim_s=self.now,
                expected_exit_sim_s=self.now + costs[u, tid], points=copy.deepcopy(offer['points']),
                map_revision=offer['map_revision'], valid_until_sim_s=offer['valid_until_sim_s'])
            self.commands[cid] = dict(task=tid, kind='ROUTE_GRANT', epoch=t['epoch'],
                                      version=t['version'], uav=u, ack=None)
            self._lock(tid, 'HELD', 'ASSIGNED')
            self._emit('TASK_ASSIGN', task_id=tid, target_id=t['target_id'], uav_id=u,
                       epoch=t['epoch'], lease_until_sim_s=t['lease'])
            self._event('TASK_ASSIGNED', u, tid)
            self._reservation(rid, 'RESERVED')
            msg = self._emit('ROUTE_GRANT', command_id=cid, task_id=tid, target_id=t['target_id'],
                uav_id=u, epoch=t['epoch'], route_version=t['version'], offer_id=offer['offer_id'],
                map_revision=offer['map_revision'], frame_id='world_enu', points=offer['points'],
                reservation_ids=[rid], valid_until_sim_s=offer['valid_until_sim_s'])
            self._event('ROUTE_GRANTED', u, tid, details=msg['data'])

    def _ack(self, d):
        c = self.commands.get(d['command_id'])
        if not c or (d['uav_id'], d['task_id'], d['epoch'], d['route_version']) != \
                (c['uav'], c['task'], c['epoch'], c['version']):
            raise CoordinationError('ACK_FENCE_MISMATCH')
        t = self.tasks[c['task']]
        if d['command_id'] != t['command']:
            raise CoordinationError('ACK_SUPERSEDED')
        status = d['status']
        if c['ack'] == status:
            return
        transitions = {None: {'ACCEPTED', 'REJECTED'}, 'ACCEPTED': {'STARTED', 'STOPPED', 'REJECTED'},
                       'STARTED': {'STOPPED', 'COMPLETED', 'REJECTED'}}
        if status not in transitions.get(c['ack'], set()):
            raise CoordinationError('ACK_ORDER')
        if status == 'COMPLETED' and c['kind'] != 'ROUTE_GRANT':
            raise CoordinationError('STOP_COMMAND_CANNOT_COMPLETE_TASK')
        if status in ('STOPPED', 'COMPLETED'):
            s = self.vehicles.get(c['uav'])
            if not self._stopped(c['uav']) or s['mode'] not in ('HOLDING', 'LANDED', 'IDLE') or s['observed_epoch'] != c['epoch'] or \
                    s['route_version'] != c['version'] or s['active_command_id'] != d['command_id']:
                raise CoordinationError('ACK_NO_STOP_EVIDENCE')
            if status == 'COMPLETED' and math.dist(s['xyz'], t['xyz']) > self.limits['arrival_tolerance_m']:
                self._violation('ARRIVAL_ERROR')
                return
            t['stopped'], t['stop_seq'] = True, s['seq']
            if status == 'COMPLETED':
                t['complete'], t['status'] = True, 'COMPLETED'
                self.progress = self.now
                self._event('TASK_COMPLETED', c['uav'], c['task'])
            elif t['status'] not in ('REVOKING', 'QUARANTINED'):
                t['status'] = 'REVOKING'
                self._lock(c['task'], 'REVOKING', 'STOPPED')
        c['ack'] = status
        if status == 'STARTED' and c['kind'] == 'ROUTE_GRANT':
            t['status'] = 'EXECUTING'
            self._reservation(t['reservation'], 'OCCUPIED')
        if status == 'REJECTED':
            self._revoke(c['task'], 'COMMAND_REJECTED', quarantine=True)
        self._event('COMMAND_ACKED', c['uav'], c['task'], d['reason'], d)

    def _clear(self, d):
        r = self.reservations.get(d['reservation_id'])
        if not r or (d['uav_id'], d['epoch'], d['route_version']) != \
                (r['uav_id'], r['epoch'], r['route_version']):
            raise CoordinationError('CLEAR_FENCE_MISMATCH')
        if r['state'] == 'RELEASED':
            return
        t, u = self.tasks[r['task_id']], r['uav_id']
        s = self.vehicles.get(u)
        if not t['stopped'] or not self._stopped(u) or s['mode'] not in ('HOLDING', 'LANDED', 'IDLE') or s['seq'] <= t['stop_seq'] or \
                s['observed_epoch'] != r['epoch'] or s['route_version'] != r['route_version'] or \
                d['map_revision'] != s['map_revision'] or \
                math.dist(d['xyz'], s['xyz']) > self.limits['position_tolerance_m']:
            raise CoordinationError('CLEAR_NO_FRESH_EVIDENCE')
        # Must be physically OUTSIDE even if landed: a ground-level reservation
        # cannot be released merely because motors are disarmed.
        if route_distance(r['points'], [s['xyz'], s['xyz']]) <= \
                self.limits['min_separation_m'] + 2 * self.limits['tracking_bound_m']:
            raise CoordinationError('RESOURCE_STILL_OCCUPIED')
        self._reservation(r['reservation_id'], 'RELEASED')
        self._lock(r['task_id'], 'RELEASED', 'VERIFIED_CLEAR')
        if s['health'] != 'OK' and s['mode'] == 'LANDED' and not s['armed']:
            # Explicit stopped + clear + grounded evidence permits task
            # takeover. This aircraft cannot be reactivated in this run.
            self.retired.add(u)
        t.update(owner=None, reservation=None, command=None,
                 status='COMPLETED' if t['complete'] else ('CANCELLED' if t['cancel'] else 'PENDING'))
        self.progress = self.now
        self.offers = {k: v for k, v in self.offers.items() if k[1] != r['task_id']}

    def _watchdogs(self):
        for tid, t in self.tasks.items():
            if t['owner'] and t['status'] in ('ASSIGNED', 'EXECUTING'):
                r = self.reservations[t['reservation']]
                if not self._fresh(t['owner']):
                    self._revoke(tid, 'HEARTBEAT_TIMEOUT', quarantine=True)
                elif self.now >= t['lease']:
                    self._revoke(tid, 'LEASE_EXPIRED', quarantine=True)
                elif self.now >= r['valid_until_sim_s']:
                    self._revoke(tid, 'ROUTE_EXPIRED')
        incomplete = any(t['status'] != 'COMPLETED' for t in self.tasks.values())
        if incomplete and self.now - self.started > self.limits['mission_timeout_s']:
            self._violation('MISSION_TIMEOUT')
        elif incomplete and self.now - self.progress > self.limits['deadlock_timeout_s']:
            self._violation('DEADLOCK')

    def finish(self, monitor_report=None):
        """Close log only after no new grants are desired. Never invent evidence.

        No monitor report => ABSTAIN; caller must supply independent coverage.
        This does not stop a real aircraft: adapter owns shutdown/landing.
        """
        if self.finished:
            raise CoordinationError('RUN_FINISHED')
        from .verdict import evaluate_terminal
        self.halted = True
        landed = all(self._fresh(u) and self.vehicles[u]['mode'] == 'LANDED' and
                     not self.vehicles[u]['armed'] for u in self.fleet)
        details = dict(outcome='ABSTAIN', failures=list(self.failures),
                       completed_task_ids=sorted(k for k, t in self.tasks.items() if t['complete']),
                       all_landed_disarmed=landed, evidence_complete=monitor_report is not None,
                       monitor_report=copy.deepcopy(monitor_report))
        ok, reason = evaluate_terminal(self.limits, self.started, self.now, list(self.tasks),
                                       len(self.fleet), details)
        if any(r['state'] != 'RELEASED' for r in self.reservations.values()) and ok is True:
            ok, reason = None, 'RESERVATIONS_NOT_CLEARED'
            details['evidence_complete'] = False
        details['outcome'] = 'PASS' if ok is True else ('FAIL' if ok is False else 'ABSTAIN')
        self._event('RUN_FINISHED', reason=reason, details=details)
        self.finished = True
        return ok, reason
