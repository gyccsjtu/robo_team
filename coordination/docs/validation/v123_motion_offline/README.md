# v1.23 运动过滤修正：离线证据，未执行的新代码

当前只有一轮六机运行：`codex_city_teammate_v122b_20261004/flight`，
启动提交1004277、视觉v2.9。此检查点来自该轮中途，不是另一轮仿真，
不是完整600秒结果。开发裁判当时已消除白色和两个红色（3/6）。

这里保存423项核心测试、蓝色运动闸门回放、对应CSV字节前缀与SHA、
原始相机图和只读模型位置。`extra_static_person.xml`显示地图额外的
静态蓝衣casual_female；它不是actor_0..5。模型坐标只用于此离线诊断。

- 本轮该静态人物附近361个逐轨、逐原图样本：旧运动闸门93个通过，新闸门6个通过。
- 旧v1.15附近101个样本：27→14；其他蓝色样本78个：63→56。
  “其他”缺少独立蓝色真值，不能当作全部正确目标。
- 相机原图、模型查询有约0.2秒时间差；图片支持识别静态场景，不能作为同步精度验收。
- 候选新代码允许有效授权中的绿色同帧人体停住后继续上报；当前帧必须通过
  人体与绿色衣服核验。缺框、旧帧、STOP、过期不能制造连续15秒证据。
- **新代码尚未进入当前飞行快照；没有六目标完成或无碰撞证明。**

从仓库根目录复现两份运动统计（纯Python，无ROS/Gazebo/CUDA）：

```bash
python3 coordination/scripts/replay_image_motion.py \
  --algorithm coordination/docs/validation/v123_motion_offline/algorithm_checkpoint \
  --annotation-xy -26.902 13.6572 --output /tmp/current_motion_replay.json
python3 coordination/scripts/replay_image_motion.py \
  --algorithm coordination/docs/validation/v123_motion_offline/algorithm_v115_checkpoint \
  --annotation-xy -26.902 13.6572 --output /tmp/old_motion_replay.json
```

输出文件须不存在。该回放通过距离不超过3米的近邻将原始检测关联到日志轨迹，
不是完整YOLO或裁判回放。脚本不发布话题、不查询真值，不修改输入文件。
源代码语义见视觉v2.10与v2.11文档；原图去重、schema2、任务授权v2.2保持。
