#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" OccupancyGrid 降采样器：1m -> 7m

把你们的 1m 分辨率占据栅格降采样到 7m 粗网格，供队友的 swarm_task 使用。

设计原则：
  - 保守降采样：只要原 7x7 区域内有一个障碍格，目标格就是障碍
  - 世界坐标对齐：保持与源 grid 相同的坐标原点

用法：
    from grid_downsampler import OccupancyDownsampler
    downsampler = OccupancyDownsampler()
    
    # 输入你们的 1m grid
    downsampler.set_fine_grid(hits_bytearray, x0=-52.0, y0=-52.0, cell=1.0)
    
    # 输出 7m 粗网格
    coarse_hits, nx, ny = downsampler.get_coarse_grid(cell_size=7.0)
"""

import math


class OccupancyDownsampler(object):
    """1m -> 7m 降采样器"""

    def __init__(self):
        self.fine_hits = None
        self.fine_nx = 0
        self.fine_ny = 0
        self.x0 = 0.0
        self.y0 = 0.0
        self.fine_cell = 1.0

    def set_fine_grid(self, hits, x0=-52.0, y0=-52.0, cell=1.0):
        """设置细栅格 (1m 分辨率)
        
        Args:
            hits: bytearray，1m栅格的障碍标记
            x0, y0: 栅格世界坐标原点
            cell: 栅格分辨率 (默认 1.0m)
        """
        self.fine_hits = hits
        self.x0 = x0
        self.y0 = y0
        self.fine_cell = cell
        
        # 计算细栅格尺寸
        # occupancy_online.py: X0=-52, X1=132, Y0=-52, Y1=52, CELL=1.0
        self.fine_nx = int((132.0 - (-52.0)) / cell) + 1  # 185
        self.fine_ny = int((52.0 - (-52.0)) / cell) + 1   # 105

    def get_coarse_grid(self, cell_size=7.0):
        """获取降采样后的粗栅格 (7m 分辨率)
        
        Args:
            cell_size: 目标分辨率，默认 7.0m (与 GRID_SIZE_M 对齐)
            
        Returns:
            coarse_hits: bytearray，7m栅格的障碍标记
            nx: 粗栅格 x 方向数量
            ny: 粗栅格 y 方向数量
        """
        if self.fine_hits is None:
            raise ValueError("请先调用 set_fine_grid() 设置细栅格")

        # 计算粗栅格尺寸
        x1 = self.x0 + (self.fine_nx - 1) * self.fine_cell
        y1 = self.y0 + (self.fine_ny - 1) * self.fine_cell
        
        nx = int(math.ceil((x1 - self.x0) / cell_size)) + 1
        ny = int(math.ceil((y1 - self.y0) / cell_size)) + 1
        
        coarse_hits = bytearray(nx * ny)
        
        # 降采样：每个粗格子对应 7x7 个细格子
        # 保守策略：只要有一个细格子是障碍，粗格子就是障碍
        samples_per_side = int(cell_size / self.fine_cell)  # 7 / 1 = 7
        
        for ix in range(nx):
            for iy in range(ny):
                # 粗格子覆盖的细格子范围
                fx0 = ix * samples_per_side
                fy0 = iy * samples_per_side
                fx1 = min(fx0 + samples_per_side, self.fine_nx)
                fy1 = min(fy0 + samples_per_side, self.fine_ny)
                
                # 扫描该区域，取最大值
                blocked = 0
                for fxi in range(fx0, fx1):
                    for fyi in range(fy0, fy1):
                        if fxi < self.fine_nx and fyi < self.fine_ny:
                            idx = fyi * self.fine_nx + fxi
                            if idx < len(self.fine_hits) and self.fine_hits[idx]:
                                blocked = 1
                                break
                    if blocked:
                        break
                
                coarse_hits[iy * nx + ix] = blocked
        
        return coarse_hits, nx, ny

    def world_to_coarse_cell(self, x, y, cell_size=7.0):
        """世界坐标转粗栅格索引
        
        Args:
            x, y: 世界坐标 (米)
            cell_size: 粗栅格分辨率
            
        Returns:
            (ix, iy) 栅格索引，或 None 如果超出范围
        """
        ix = int(math.floor((x - self.x0) / cell_size))
        iy = int(math.floor((y - self.y0) / cell_size))
        
        nx = int(math.ceil((132.0 - (-52.0)) / cell_size)) + 1
        ny = int(math.ceil((52.0 - (-52.0)) / cell_size)) + 1
        
        if 0 <= ix < nx and 0 <= iy < ny:
            return (ix, iy)
        return None


def downsample_fine_to_coarse(fine_hits, fine_nx, fine_ny, 
                              x0=-52.0, y0=-52.0, fine_cell=1.0,
                              coarse_cell=7.0):
    """ standalone 降采样函数
    
    Args:
        fine_hits: bytearray，1m栅格障碍标记
        fine_nx, fine_ny: 细栅格尺寸
        x0, y0: 世界坐标原点
        fine_cell: 细栅格分辨率
        coarse_cell: 粗栅格分辨率
        
    Returns:
        coarse_hits, nx, ny
    """
    ds = OccupancyDownsampler()
    ds.fine_hits = fine_hits
    ds.fine_nx = fine_nx
    ds.fine_ny = fine_ny
    ds.x0 = x0
    ds.y0 = y0
    ds.fine_cell = fine_cell
    return ds.get_coarse_grid(coarse_cell)


# ------------------------------------------------------------------ 自检
def selftest():
    print("=" * 60)
    print("OccupancyDownsampler 自检")
    print("=" * 60)
    
    # 用例1：全部空的场景
    print("\n用例1: 全空场景")
    fine = bytearray(185 * 105)  # 1m栅格全0
    coarse, nx, ny = downsample_fine_to_coarse(fine, 185, 105)
    print(f"  细栅格: 185x105, 障碍格: {sum(fine)}")
    print(f"  粗栅格: {nx}x{ny}, 障碍格: {sum(coarse)}")
    assert sum(coarse) == 0, "全空应该输出全空"
    print("  ✓ 通过")
    
    # 用例2：中心一个障碍
    print("\n用例2: 中心一个1m障碍")
    fine = bytearray(185 * 105)
    cx, cy = 40, 52  # 大概在中心附近 (40, 52) -> 对应 world (40-52, 52-52) = (-12, 0)
    idx = cy * 185 + cx
    fine[idx] = 1
    coarse, nx, ny = downsample_fine_to_coarse(fine, 185, 105)
    print(f"  细栅格障碍: ({cx}, {cy})")
    print(f"  粗栅格: {nx}x{ny}, 障碍格: {sum(coarse)}")
    assert sum(coarse) > 0, "中心障碍应该被检测到"
    print("  ✓ 通过")
    
    # 用例3：7x7区域全障碍 -> 降采样后应该是1个粗格子
    print("\n用例3: 7x7区域全障碍")
    fine = bytearray(185 * 105)
    start_x, start_y = 35, 49  # 起始位置
    for fx in range(start_x, start_x + 7):
        for fy in range(start_y, start_y + 7):
            if fx < 185 and fy < 105:
                fine[fy * 185 + fx] = 1
    coarse, nx, ny = downsample_fine_to_coarse(fine, 185, 105)
    print(f"  细栅格: {start_x}~{start_x+6}, {start_y}~{start_y+6} 全障碍")
    print(f"  粗栅格: {nx}x{ny}, 障碍格: {sum(coarse)}")
    # 7x7应该正好落在一个7m格子里
    assert sum(coarse) >= 1, "7x7区域应该产生至少1个障碍格"
    print("  ✓ 通过")
    
    # 用例4：验证保守性 - 边缘有部分障碍
    print("\n用例4: 7x7边缘部分障碍 (保守性验证)")
    fine = bytearray(185 * 105)
    # 只填满7x7区域的左上角3x3
    for fx in range(start_x, start_x + 3):
        for fy in range(start_y, start_y + 3):
            if fx < 185 and fy < 105:
                fine[fy * 185 + fx] = 1
    coarse, nx, ny = downsample_fine_to_coarse(fine, 185, 105)
    print(f"  细栅格: {start_x}~{start_x+2}, {start_y}~{start_y+2} 障碍")
    print(f"  粗栅格: {nx}x{ny}, 障碍格: {sum(coarse)}")
    # 即使只有部分障碍，目标粗格子仍然是障碍
    assert sum(coarse) >= 1, "部分障碍也应该被检测到"
    print("  ✓ 通过 (保守性验证)")
    
    print("\n" + "=" * 60)
    print("自检全部通过!")
    print("=" * 60)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="OccupancyGrid 降采样器")
    parser.add_argument("--selftest", action="store_true", help="运行自检")
    args = parser.parse_args()
    
    if args.selftest:
        selftest()
    else:
        print(__doc__)
