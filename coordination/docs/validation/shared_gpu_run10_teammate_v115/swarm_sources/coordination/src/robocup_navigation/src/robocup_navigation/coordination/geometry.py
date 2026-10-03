"""Continuous 3-D distance checks for conservative route reservations."""
import math


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def segment_distance(p, q, r, s):
    """Minimum distance between two CLOSED segments, including degeneracies."""
    u, v, w = sub(q, p), sub(s, r), sub(p, r)
    a, b, c, d, e = dot(u, u), dot(u, v), dot(v, v), dot(u, w), dot(v, w)
    if a <= 1e-20 and c <= 1e-20:
        return math.dist(p, r)
    if a <= 1e-20:
        x, y = 0., max(0., min(1., e / c))
    elif c <= 1e-20:
        x, y = max(0., min(1., -d / a)), 0.
    else:
        den = a * c - b * b
        x = max(0., min(1., (b * e - c * d) / den)) if den > 1e-20 else 0.
        y = (b * x + e) / c
        if y < 0.:
            y, x = 0., max(0., min(1., -d / a))
        elif y > 1.:
            y, x = 1., max(0., min(1., (b - d) / a))
    return math.sqrt(max(0., sum((w[i] + x * u[i] - y * v[i]) ** 2 for i in range(3))))


def route_distance(a, b):
    """两条路径之间的最小空间距离（无时间维度）"""
    return min(segment_distance(p, q, r, s)
               for p, q in zip(a, a[1:]) for r, s in zip(b, b[1:]))


def route_distance_with_time(path_a, path_b, time_a, time_b, speed_mps):
    """时空冲突检测：检查两条路径是否在时间+空间上冲突
    
    Args:
        path_a, path_b: 路径点列表 [[x,y,z], ...]
        time_a, time_b: 到达时间列表 [t0, t1, t2, ...] (sim_s)
        speed_mps: 飞行速度 m/s
    
    Returns:
        (min_distance, time_overlap): 最小空间距离和时间重叠
        如果 time_overlap > 0 且 min_distance < threshold，则冲突
    """
    min_distance = float('inf')
    time_overlap = 0.0
    
    # 路径段之间的空间距离
    for i, (p, q) in enumerate(zip(path_a, path_a[1:])):
        for j, (r, s) in enumerate(zip(path_b, path_b[1:])):
            dist = segment_distance(p, q, r, s)
            if dist < min_distance:
                min_distance = dist
            
            # 检查时间重叠
            t_a_start, t_a_end = time_a[i], time_a[i + 1]
            t_b_start, t_b_end = time_b[j], time_b[j + 1]
            
            # 计算时间重叠区间
            overlap_start = max(t_a_start, t_b_start)
            overlap_end = min(t_a_end, t_b_end)
            
            if overlap_end > overlap_start:
                # 时间重叠段的空间距离
                if dist < speed_mps * 2:  # 阈值：2秒内的距离
                    time_overlap += overlap_end - overlap_start
    
    return min_distance, time_overlap


def estimate_route_times(path_points, speed_mps, start_time=0.0):
    """估计路径上每个点的到达时间
    
    Args:
        path_points: 路径点列表
        speed_mps: 速度 m/s
        start_time: 起始时间
    
    Returns:
        到达时间列表 [t0, t1, t2, ...]
    """
    times = [start_time]
    for i in range(1, len(path_points)):
        dist = math.dist(path_points[i - 1], path_points[i])
        times.append(times[-1] + dist / speed_mps)
    return times


def route_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def matching(vehicles, tasks, costs):
    """Max-cardinality then min-cost matching, deterministic, at most six UAVs.

    DP over the vehicle bitmask is O(tasks * 2**vehicles * vehicles).
    Missing cost edges are infeasible, never replaced with straight-line cost.
    """
    vehicles, tasks = sorted(vehicles), sorted(tasks)
    states = {0: (0., ())}
    for task in tasks:
        nxt = dict(states)
        for mask, (cost, pairs) in states.items():
            for i, vehicle in enumerate(vehicles):
                if mask & (1 << i) or (vehicle, task) not in costs:
                    continue
                key = mask | (1 << i)
                candidate = (cost + costs[vehicle, task], pairs + ((vehicle, task),))
                if key not in nxt or candidate < nxt[key]:
                    nxt[key] = candidate
        states = nxt
    return list(min(states.items(), key=lambda item:
                    (-bin(item[0]).count('1'), item[1][0], item[1][1]))[1][1])
