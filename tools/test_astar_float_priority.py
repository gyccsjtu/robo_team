"""Long empty-map legs must not exhaust the patrol budget on equal-cost ties."""
import heapq
import math
import random
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'coordination/src/robocup_navigation/src'))
from robocup_navigation.astar import GridMap,plan

class AstarPriorityTests(unittest.TestCase):
    def test_recorded_empty_east_corner_reachable_with_original_budget(self):
        grid=GridMap(420,260,.5,(-55.,-65.),bytes(420*260))
        for start in ((0.,0.),(5.,0.)):
            for goal in ((139.1060413763,58.5),(139.1765936283,58.)):
                with self.subTest(start=start,goal=goal):
                    result=plan(grid,start,goal,connectivity=8,max_expansions=2500)
                    self.assertTrue(result.success,result.reason)
                    dx=abs(result.start_cell[0]-result.goal_cell[0])
                    dy=abs(result.start_cell[1]-result.goal_cell[1])
                    self.assertAlmostEqual(result.path_length_m,
                        (max(dx,dy)-min(dx,dy)+math.sqrt(2)*min(dx,dy))*.5)
                    self.assertLessEqual(result.expanded_nodes,max(dx,dy)+2)

    def test_matches_independent_shortest_path_and_keeps_corner_protection(self):
        rng=random.Random(734)
        for connectivity in (4,8):
            for _ in range(12):
                cells=bytearray(int(rng.random()<.2) for _ in range(13*11))
                cells[0]=cells[-1]=0
                grid=GridMap(13,11,.5,(0.,0.),cells)
                # Independent uniform-cost search, with explicit side checks.
                heap=[(0.,(0,0))];best={(0,0):0.};expected=math.inf
                while heap:
                    cost,cell=heapq.heappop(heap)
                    if cost>best[cell]:continue
                    if cell==(12,10):expected=cost;break
                    for dx in (-1,0,1):
                        for dy in (-1,0,1):
                            if (dx==dy==0) or (connectivity==4 and dx and dy):continue
                            nxt=(cell[0]+dx,cell[1]+dy)
                            if not grid.is_free(nxt):continue
                            if dx and dy and (not grid.is_free((cell[0]+dx,cell[1])) or
                                              not grid.is_free((cell[0],cell[1]+dy))):continue
                            value=cost+(.5*math.sqrt(2) if dx and dy else .5)
                            if value<best.get(nxt,math.inf):
                                best[nxt]=value;heapq.heappush(heap,(value,nxt))
                result=plan(grid,(.25,.25),(6.25,5.25),connectivity=connectivity)
                self.assertEqual(result.success,math.isfinite(expected))
                if result.success:self.assertAlmostEqual(result.path_length_m,expected,places=10)

    def test_true_work_limit_and_occupied_points_still_fail_closed(self):
        grid=GridMap(420,260,.5,(-55.,-65.),bytes(420*260))
        self.assertEqual(plan(grid,(0.,0.),(139.,58.),max_expansions=1).reason,'EXPANSION_LIMIT')
        cells=bytearray(grid.cells);cells[130*420+110]=1
        occupied=GridMap(420,260,.5,(-55.,-65.),cells)
        self.assertEqual(plan(occupied,(0.,0.),(139.,58.)).reason,'START_OCCUPIED')

if __name__=='__main__':unittest.main()
