#!/bin/bash
# 多张随机图批量真飞 —— 连续验证「不同随机布局下两级避障均可工作」
#
# 每张图一条 A* 零穿墙航线，起飞点均为 (0,-3)，高度 5.5m。
# 用法：bash fly_multi.sh
set -u
HERE="$HOME/robocup_real"

RUNS="try2:112:22 try3:112:22 try5:126:-12 try6:120:-30"

for r in $RUNS; do
  IFS=: read -r name gx gy <<< "$r"
  echo ""
  echo "############################################################"
  echo "############  $name   ->  ($gx, $gy)  ############"
  echo "############################################################"
  GEN="$HERE/_genvm/$name" X=0.0 Y=-3.0 GX="$gx" GY="$gy" \
    bash "$HERE/fly_rand.sh" 2>&1 | tail -42
  echo "### $name 结束 rc=$?"
  sleep 3
done

echo ""
echo "######################## 全部完成 ########################"
pkill -9 -f '[g]zserver' 2>/dev/null; pkill -9 -f '[b]in/px4' 2>/dev/null
pkill -9 -f '[m]avros_node' 2>/dev/null; pkill -9 -f '[r]osmaster' 2>/dev/null
echo "栈已清理"
