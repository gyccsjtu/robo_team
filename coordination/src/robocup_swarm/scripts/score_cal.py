#coding: utf-8
import rospy
import sys
from std_msgs.msg import Int16,String, Time, Float32
from robocup_swarm.msg import ActorInfo
from gazebo_msgs.srv import DeleteModel,GetModelState
import time
import os
try:
    import cv2
except ImportError:
    cv2 = None


uav_type = (sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-')
            else 'typhoon_h480')
actor_num = 6
uav_num = 6
err_threshold = 1
coordx_bias = 3
coordy_bias = 9
actor_id_dict = {'green':[0], 'blue':[1], 'brown':[2], 'white':[3], 'red':[4,5]}

MAX_UAV_ALTITUDE = 6.0
DETECTION_INTERVAL = 1.0
DETECTION_DURATION = 15.0
MISSION_TIMEOUT = 600.0
DEFAULT_UAV_LOSS_PENALTY = 100.0
UAV_MISS_LIMIT = 3
#跟踪分数相关设置
TRACKING_DURATION = 20.0
TRACKING_DISTANCE = 10.0
uav_positions = [None] * uav_num
tracking_start_time = [None] * actor_num
tracked_actors = set()



left_actors = []
find_actors = []
actors_pos = [None] * actor_num
count_flag = [False] * actor_num
topic_arrive_time = [0.0] * actor_num
find_time = [0.0] * actor_num


find_actor_pub = []
mission_finished = False
uav_loss_count = 0
uav_loss_penalty = DEFAULT_UAV_LOSS_PENALTY
# 2026-10-05 复盘统计：score_cal reset 触发与 streak 长度分布（聚合写 stderr）
_STATS = {
    "last_t": 0.0,
    "cb_calls": 0,
    "reset_total": 0,
    "reset_by_distance": 0,
    "reset_by_discontinuous": 0,
    "reset_by_no_pos": 0,
    "streak_samples": 0,
    "streak_len_sum": 0.0,
    "streak_len_max": 0.0,
    "streak_lt1": 0, "streak_1to5": 0, "streak_5to10": 0,
    "streak_10to15": 0, "streak_ge15": 0,
    # 2026-10-06: 裁判误差判据（<1m）直方图 —— 上报坐标 vs Gazebo 真值距离。
    # ge1 是"越界即清零"的直接计数，dist_max 是全程最远误差。
    "dist_n": 0, "dist_max": 0.0,
    "dist_lt03": 0, "dist_03to05": 0, "dist_05to08": 0,
    "dist_08to1": 0, "dist_ge1": 0,
    "streak_last_start": None,
    "_last_reset_cause": None,
}
sensor_cost = 0.0
target_finish = 0
find_finish = 0
score = 0.0
time_usage = 0.0
start_time = 0.0



def _now():
    return rospy.get_time()


def _publish_score():
    if 'score_pub' not in globals():
        return
    value = int(round(max(-32768, min(32767, score))))
    score_pub.publish(value)


def _progress_score():
    track_finish = len(tracked_actors)

    return (
        find_finish * 50
        + track_finish * 80
        + target_finish * 100
        - sensor_cost * 3e-3
        - uav_loss_count * uav_loss_penalty
    )



def _emit_final(reason):
    """2026-10-06: [FINAL] 单行汇总（_finish 与 ROS shutdown 异常路径共用）。
    dist 字段是裁判误差判据（<1m）的直接证据：n=样本数 max=最远 ge1=越界次数。"""
    _ss = _STATS
    print(
        "[FINAL] score={} find_finish={} uav_loss={} "
        "cb={} reset={} (dist/disc/no_pos={}/{}/{}) "
        "buckets(lt1/1-5/5-10/10-15/ge15)={}/{}/{}/{}/{} "
        "dist(n/max/ge1)={}/{:.2f}m/{} "
        "streak_max={:.1f}s reason={}".format(
            score,
            find_finish,
            uav_loss_count,
            _ss.get("cb_calls", 0),
            _ss.get("reset_total", 0),
            _ss.get("reset_by_distance", 0),
            _ss.get("reset_by_discontinuous", 0),
            _ss.get("reset_by_no_pos", 0),
            _ss.get("streak_lt1", 0),
            _ss.get("streak_1to5", 0),
            _ss.get("streak_5to10", 0),
            _ss.get("streak_10to15", 0),
            _ss.get("streak_ge15", 0),
            _ss.get("dist_n", 0),
            _ss.get("dist_max", 0.0),
            _ss.get("dist_ge1", 0),
            _ss.get("streak_len_max", 0.0),
            reason,
        )
    )


def _finish(reason, final_score=None):
    global score, mission_finished
    if mission_finished:
        return
    mission_finished = True
    # 2026-10-05 复盘：进程退出前强制打出最后一段聚合
    try:
        _stats_dump(_now(), force=True)
    except Exception:
        pass
    if final_score is not None:
        score = final_score
    print(reason)
    print('score:', score)
    # 2026-10-06: Ctrl-C / SIGTERM 时也保证能从 03_judge.log 末尾 grep 到最终值
    try:
        _stats_dump(_now(), force=True)
        _emit_final(reason)
        sys.stdout.flush()
    except Exception:
        pass
    _publish_score()
    signal_shutdown = getattr(rospy, 'signal_shutdown', None)
    if signal_shutdown:
        signal_shutdown(reason)


def _reset_detection(actor_id):
    count_flag[actor_id] = False
    find_time[actor_id] = 0.0
    topic_arrive_time[actor_id] = 0.0
    # 2026-10-05 复盘统计：reset 触发频率（按原因拆分）—— 判断 EMA 双轨制是否生效
    _STATS["reset_total"] += 1
    if _STATS.get("_last_reset_cause"):
        _STATS["reset_by_" + _STATS["_last_reset_cause"]] += 1
    _STATS["streak_samples"] += 1
    cur = _STATS["streak_last_start"]
    if cur is not None:
        dur = _now() - cur
        _STATS["streak_len_sum"] += dur
        if dur > _STATS["streak_len_max"]:
            _STATS["streak_len_max"] = dur
        # 桶：<1s / 1-5 / 5-10 / 10-15 / >=15
        if   dur < 1.0:  _STATS["streak_lt1"]   += 1
        elif dur < 5.0:  _STATS["streak_1to5"]  += 1
        elif dur < 10.0: _STATS["streak_5to10"] += 1
        elif dur < 15.0: _STATS["streak_10to15"] += 1
        else:            _STATS["streak_ge15"]  += 1
    _STATS["streak_last_start"] = -1.0  # 表示本轮未在 streak


def _stats_dump(now, force=False):
    """2026-10-05 复盘统计：score_cal 的 reset/streak 聚合一览"""
    if not force and (now - _STATS["last_t"] < 10.0 or _STATS["last_t"] == 0.0):
        if _STATS["last_t"] == 0.0:
            _STATS["last_t"] = now
        return
    s = _STATS["streak_samples"]
    avg = (_STATS["streak_len_sum"] / s) if s > 0 else 0.0
    sys.stderr.write(
        "[SCORE_STATS] win=%.0fs cb=%d reset=%d far_dist=%d discontinuous=%d "
        "streak_avg=%.2fs max=%.2fs buckets(lt1/1-5/5-10/10-15/ge15)=%d/%d/%d/%d/%d "
        "dist(n/max/ge1)=%d/%.2fm/%d\n" % (
            now - _STATS["last_t"],
            _STATS["cb_calls"], _STATS["reset_total"],
            _STATS["reset_by_distance"], _STATS["reset_by_discontinuous"],
            avg, _STATS["streak_len_max"],
            _STATS["streak_lt1"], _STATS["streak_1to5"], _STATS["streak_5to10"],
            _STATS["streak_10to15"], _STATS["streak_ge15"],
            _STATS["dist_n"], _STATS["dist_max"], _STATS["dist_ge1"]))
    sys.stderr.flush()
    _STATS.update({"cb_calls": 0, "reset_total": 0,
                   "reset_by_distance": 0, "reset_by_discontinuous": 0,
                   "streak_samples": 0, "streak_len_sum": 0.0, "streak_len_max": 0.0,
                   "streak_lt1": 0, "streak_1to5": 0, "streak_5to10": 0,
                   "streak_10to15": 0, "streak_ge15": 0,
                   "dist_n": 0, "dist_max": 0.0,
                   "dist_lt03": 0, "dist_03to05": 0, "dist_05to08": 0,
                   "dist_08to1": 0, "dist_ge1": 0,
                   "last_t": now, "streak_last_start": None})


def _delete_actor(actor_id):
    global target_finish, score
    try:
        response = del_model('actor_' + str(actor_id))
        if hasattr(response, 'success') and not response.success:
            print('Unable to remove actor_%d: %s' % (actor_id, getattr(response, 'status_message', 'unknown error')))
            return False
    except Exception as exc:
        print('Unable to remove actor_%d: %s' % (actor_id, exc))
        return False
    if actor_id in left_actors:
        left_actors.remove(actor_id)
    target_finish = actor_num - len(left_actors)
    score = _progress_score()
    return True


def _process_actor_detection(msg, actor_ids):
    global find_finish, score, time_usage
    if mission_finished or not getattr(msg, 'cls', None):
        return
    now = _now()
    _STATS["cb_calls"] += 1
    _STATS["_last_reset_cause"] = None
    for actor_id in actor_ids:
        if actor_id not in left_actors:
            continue
        position = actors_pos[actor_id]
        previous = topic_arrive_time[actor_id]
        topic_arrive_time[actor_id] = now
        if position is None:
            _STATS["_last_reset_cause"] = "no_pos"
            _reset_detection(actor_id)
            continue
        distance_sq = ((msg.x - position.x) ** 2 + (msg.y - position.y) ** 2)
        continuous = previous == 0.0 or now - previous <= DETECTION_INTERVAL
        # 2026-10-06: 裁判误差判据（<1m）直方图 —— 上报坐标 vs Gazebo 真值，
        # 含被 reset 的样本（越界样本正是"计时清零"的直接原因，必须统计）
        _dist = distance_sq ** 0.5
        _STATS["dist_n"] += 1
        if _dist > _STATS["dist_max"]:
            _STATS["dist_max"] = _dist
        if   _dist < 0.3:  _STATS["dist_lt03"]   += 1
        elif _dist < 0.5:  _STATS["dist_03to05"] += 1
        elif _dist < 0.8:  _STATS["dist_05to08"] += 1
        elif _dist < 1.0:  _STATS["dist_08to1"]  += 1
        else:              _STATS["dist_ge1"]    += 1
        if distance_sq >= err_threshold ** 2 or not continuous:
            # 拆分 reset 原因：distance vs discontinuous（用于判断 EMA 双轨制是否生效）
            # 2026-10-06 修：旧三元式 "discontinuous" if continuous else "distance"
            # 标签写反 —— 走到此处且 continuous=True 时失败原因必是 distance。
            _STATS["_last_reset_cause"] = ("distance" if distance_sq >= err_threshold ** 2
                                           else "discontinuous")
            # 2026-10-06 B 项诊断打印：每帧距离误差值（判断外推补偿是过冲还是欠补）
            # （v10 复盘：此打印此前只在 XTDrone/robocup 版有，coordination 版漏合并）
            if continuous:
                sys.stderr.write(
                    "[RESET_DBG] actor=%s dist=%.2fm msg=(%.2f,%.2f) true=(%.2f,%.2f) t=%.2fs\n"
                    % (actor_id, distance_sq**0.5, msg.x, msg.y, position.x, position.y, now))
            _reset_detection(actor_id)
            continue
        # 通过校验：开始或继续累积 streak
        if _STATS["streak_last_start"] is None:
            _STATS["streak_last_start"] = now
        if not count_flag[actor_id]:
            count_flag[actor_id] = True
            find_time[actor_id] = now
            if actor_id in find_actors:
                find_actors.remove(actor_id)
            find_finish = actor_num - len(find_actors)
            if actor_id < len(find_actor_pub):
                find_actor_pub[actor_id].publish(now)
            score = _progress_score()
            print('find actor_' + str(actor_id))
            _publish_score()
            continue
        if now - find_time[actor_id] < DETECTION_DURATION:
            continue
        if not _delete_actor(actor_id):
            _reset_detection(actor_id)
            continue
        _reset_detection(actor_id)
        print('actor_%d is OK' % actor_id)
        print('Time usage:', time_usage)
        if target_finish == actor_num:
            elapsed = max(0.0, now - start_time)
            score = (2580.0 - elapsed - sensor_cost * 3e-3
                     - uav_loss_count * uav_loss_penalty)
            _finish('Mission finished', score)
        else:
            print('score:', score)
            _publish_score()
    # 2026-10-05 复盘：每回调聚合一次 _STATS（10s 窗口）
    _stats_dump(now)


def actor_info_callback(msg):
    ids = actor_id_dict.get(getattr(msg, 'cls', ''), [])
    _process_actor_detection(msg, ids)

def actor_info1_callback(msg):
    _process_actor_detection(msg, [5] if getattr(msg, 'cls', '') == 'red' else [])

def actor_info2_callback(msg):
    _process_actor_detection(msg, [4] if getattr(msg, 'cls', '') == 'red' else [])
    
def _update_tracking(now):
    global score

    for actor_id in list(left_actors):

        actor_pos = actors_pos[actor_id]

        if actor_pos is None:
            tracking_start_time[actor_id] = None
            continue

        in_tracking_range = False

        for uav_id in range(uav_num):

            if uav_lost[uav_id]:
                continue

            uav_pos = uav_positions[uav_id]

            if uav_pos is None:
                continue

            dx = uav_pos.x - actor_pos.x
            dy = uav_pos.y - actor_pos.y

            distance_sq = dx * dx + dy * dy

            if distance_sq <= TRACKING_DISTANCE ** 2:
                in_tracking_range = True
                break

        # 当前至少有一架无人机在跟踪范围内
        if in_tracking_range:

            # 本轮连续跟踪开始
            if tracking_start_time[actor_id] is None:
                tracking_start_time[actor_id] = now

            # 连续跟踪达到20秒
            elif (
                now - tracking_start_time[actor_id]
                >= TRACKING_DURATION
            ):

                # 同一目标跟踪分只记一次
                if actor_id not in tracked_actors:

                    tracked_actors.add(actor_id)

                    print(
                        'actor_%d tracking success' % actor_id
                    )

                    score = _progress_score()
                    _publish_score()

        else:
            # 本轮连续跟踪失败，重新计时
            # 但不会取消已经获得的跟踪分
            tracking_start_time[actor_id] = None




if __name__ == "__main__":
    left_actors = list(range(actor_num))
    find_actors = list(range(actor_num))
    actors_pos = [None] * actor_num
    count_flag = [False] * actor_num
    topic_arrive_time = [0.0] * actor_num
    tracking_start_time = [None] * actor_num
    tracked_actors = set()

    find_time = [0.0] * actor_num
    find_actor_pub = []
    mission_finished = False
    uav_loss_count = 0
    uav_loss_penalty = DEFAULT_UAV_LOSS_PENALTY
    uav_seen = [False] * uav_num
    uav_lost = [False] * uav_num
    uav_miss_count = [0] * uav_num
    rospy.init_node('score_cal')
    time.sleep(1)
    start_time = rospy.get_time()
    del_model = rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState) 
    score_pub = rospy.Publisher("/score",Int16,queue_size=1) 
    time_usage_pub = rospy.Publisher("/time_usage",Int16,queue_size=1) 
    left_actors_pub = rospy.Publisher("/left_actors",String,queue_size=1)
    actor_blue_sub = rospy.Subscriber("/actor_blue_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_green_sub = rospy.Subscriber("/actor_green_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_white_sub = rospy.Subscriber("/actor_white_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_brown_sub = rospy.Subscriber("/actor_brown_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_red1_sub = rospy.Subscriber("/actor_red1_info",ActorInfo,actor_info1_callback,queue_size=1)
    actor_red2_sub = rospy.Subscriber("/actor_red2_info",ActorInfo,actor_info2_callback,queue_size=1)
    for i in range(actor_num):
        find_actor_pub.append(rospy.Publisher("/find_actor_%d"%i, Float32, queue_size=5))


    mono_cam = int(rospy.get_param('~mono_cam', 1))
    stereo_cam = int(rospy.get_param('~stereo_cam', 0))
    laser1d = int(rospy.get_param('~laser1d', 0))
    laser2d = int(rospy.get_param('~laser2d', 1))
    laser3d = int(rospy.get_param('~laser3d', 0))
    gimbal = int(rospy.get_param('~gimbal', 1))
    bino_cam = int(rospy.get_param('~bino_cam', 0))
    sensor_cost = (mono_cam * 5e2 + stereo_cam * 1e3 + laser1d * 2e2
                   + laser2d * 1e3 + laser3d * 2e3 + gimbal * 2e2
                   + bino_cam * 1e3)


    uav_loss_penalty = float(rospy.get_param('~uav_loss_penalty', DEFAULT_UAV_LOSS_PENALTY))
    target_finish = 0
    find_finish = 0
    score = _progress_score()
    time_usage = 0.0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        for i in range(uav_num):
            try:
                response = get_model_state(uav_type + '_' + str(i), 'ground_plane')
                success = getattr(response, 'success', True)
                if success:
                    uav_pos_tmp = response.pose.position

                    # 保存该无人机当前位置
                    uav_positions[i] = uav_pos_tmp

                    uav_seen[i] = True
                    uav_miss_count[i] = 0

                    if uav_pos_tmp.z > MAX_UAV_ALTITUDE:
                        score = 0
                        _finish(
                            'Warning: UAV %d is higher than 6 meters' % i,
                            0
                        )
                        break
                elif uav_seen[i] and not uav_lost[i]:
                    uav_miss_count[i] += 1
                    if uav_miss_count[i] >= UAV_MISS_LIMIT:
                        uav_lost[i] = True
                        uav_loss_count += 1
                        uav_positions[i] = None
                        score = _progress_score()
                        print('UAV %d lost; penalty %.1f' % (i, uav_loss_penalty))
            except Exception as exc:
                if uav_seen[i] and not uav_lost[i]:
                    uav_miss_count[i] += 1
                    if uav_miss_count[i] >= UAV_MISS_LIMIT:
                        uav_lost[i] = True
                        uav_loss_count += 1
                        score = _progress_score()
                        print('UAV %d lost (%s); penalty %.1f' % (i, exc, uav_loss_penalty))
        if mission_finished:
            break
        for i in list(left_actors):
            try:
                response = get_model_state('actor_' + str(i), 'ground_plane')
                if not getattr(response, 'success', True):
                    continue
                actors_pos_tmp = response.pose.position
                # 2026-10-05 国家一等奖修：旧版要求 x^2+y^2 != 0 ⇒ 原点附近 spawn
                # 的 actor 永远进不了 actors_pos ⇒ 永远 position is None ⇒
                # _process_actor_detection 直接 _reset_detection 清零，无法 delete。
                # 修复：只要返回成功（x,y 是任意值）就记录，包括原点 actor。
                actors_pos[i] = actors_pos_tmp
            except Exception:
                continue
        # 更新本轮时间
        now = rospy.get_time()
        time_usage = now - start_time

        # 根据 UAV 和目标真实位置进行连续跟踪判定
        _update_tracking(now)

        if time_usage > MISSION_TIMEOUT:
            _finish('Time out, mission failed', score)
            break

        score = _progress_score()
        _publish_score()
        time_usage_pub.publish(int(time_usage))
        left_actors_pub.publish(str(left_actors))
        if cv2 is not None and (os.name == 'nt' or os.environ.get('DISPLAY')):
            try:
                background = cv2.imread(os.path.join(os.path.dirname(__file__), "white_background.png"))
                if background is not None:
                    cv2.putText(background,"Score: "+str(score) ,(60,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
                    cv2.putText(background,"Time usage: "+str(time_usage) ,(360,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
                    cv2.putText(background,"Left targets: "+str(left_actors) ,(760,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
                    cv2.imshow("Multi-UAV search simulation competition judgment system",background)
                    cv2.waitKey(1)
            except Exception:
                pass
        # 2026-10-06: hard_cap SIGINT 场景下 rate.sleep 抛 ROSInterruptException，
        # 旧版直接裸奔退出 ⇒ 03_judge.log 只剩 traceback、复盘统计全丢（v9 即如此）。
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            try:
                _stats_dump(_now(), force=True)
                _emit_final('ros_shutdown')
                sys.stdout.flush()
            except Exception:
                pass
            raise
