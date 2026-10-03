"""Per-UAV authoritative map snapshot + revision (interface v0 §2, clarif. §2).

Two rules drive this module:

  * ATOMICITY.  Security-relevant proofs, the planner and the executor gate must
    all reference the SAME counter and the SAME snapshot.  Callers therefore use
    :meth:`snapshot` (which returns the grid copy and its revision together) and
    must never read the counter separately.
  * WHAT BUMPs THE REVISION.  Any safety-relevant content change
    (FREE/OCCUPIED/UNKNOWN transitions), window recenter/reset, or geometry
    change.  Rewriting identical content keeps the revision; refreshing an
    observation timestamp is not a content change.

Simulation states: FREE=0, OCCUPIED=1, UNKNOWN=2 (unknown is NOT free).

队友栅格订阅 (2026-10):
    订阅 /uav{0..5}/occupancy_coarse 话题，合并队友的7m降采样栅格
    用于多机协同建图
"""
import threading

try:
    from robocup_navigation.team_grid_subscriber import TeamGridSubscriber
    _HAVE_TEAM_GRID = True
except ImportError:
    _HAVE_TEAM_GRID = False

FREE, OCCUPIED, UNKNOWN = 0, 1, 2

VALID_STATES = (FREE, OCCUPIED, UNKNOWN)


class MapRevisionError(ValueError):
    pass


class MapRevisionProvider:
    """Owns one UAV's occupancy grid and its content revision."""

    def __init__(self, initial=None, uav_id=None, fleet_ids=None):
        self._lock = threading.RLock()
        self._cells = {}
        self._rev = 0
        
        # 队友栅格订阅 (可选)
        self._team_subscriber = None
        if _HAVE_TEAM_GRID and uav_id is not None and fleet_ids is not None:
            self._team_subscriber = TeamGridSubscriber(uav_id, fleet_ids)
        
        if initial:
            self.apply(initial)

    def spin_once(self):
        """处理 ROS 回调（需要在主循环中调用）"""
        if self._team_subscriber:
            self._team_subscriber.spin_once()

    def get_team_grid(self):
        """获取队友合并后的栅格"""
        if self._team_subscriber:
            return self._team_subscriber.get_combined_grid()
        return None

    def has_team_data(self):
        """检查是否收到队友数据"""
        if self._team_subscriber:
            return self._team_subscriber.has_team_data()
        return False

    def shutdown(self):
        """关闭订阅器"""
        if self._team_subscriber:
            self._team_subscriber.shutdown()

    # -- writers ------------------------------------------------------------
    def apply(self, cell_states):
        """Write {cell_index: state}.  Bumps revision only on real change.

        Returns (revision, changed_count).
        """
        for index, state in cell_states.items():
            if state not in VALID_STATES:
                raise MapRevisionError("bad state %r at %r" % (state, index))
        with self._lock:
            changed = 0
            for index, state in cell_states.items():
                if self._cells.get(index) != state:
                    self._cells[index] = state
                    changed += 1
            if changed:
                self._rev += 1
            return self._rev, changed

    def clear(self):
        """Clear every cell (e.g. a fresh observation window).  Returns revision."""
        with self._lock:
            if self._cells:
                self._cells = {}
                self._rev += 1
            return self._rev

    def recenter(self):
        """Window recenter / geometry change: always safety relevant."""
        with self._lock:
            self._rev += 1
            return self._rev

    def reset(self, cells, geometry_changed=True):
        """Full replacement (restart / reset).  Bumps on content or geometry."""
        with self._lock:
            if geometry_changed or self._cells != dict(cells):
                self._cells = dict(cells)
                self._rev += 1
            return self._rev

    # -- reader -------------------------------------------------------------
    def snapshot(self):
        """Atomic read: returns (cells_copy, revision).  Use this, not .revision."""
        with self._lock:
            return dict(self._cells), self._rev

    @property
    def revision(self):
        """Informational only -- never pair it with a separately read grid."""
        with self._lock:
            return self._rev
