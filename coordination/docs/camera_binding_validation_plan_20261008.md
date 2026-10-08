# 同图相机与渲染绑定验证计划

接续ec0ff07。用户授权继续，当前没有仿真进程。只启动一次新预算，复用validate_fast_city.py --single-seed 8159，40Hz、共享GPU、原裁判/人物/传感器/高度/内存。六机控制算法不变。

## 本次新候选

ec0ff07已保存同图header与实际相机历史样本。本次补只读SystemPlugin render_binding_observer.cc，在PostRender读取真实渲染相机姿态、原始像素FNV-1a64、尺寸/格式、场景时间、各人物visual的世界姿态与选定骨骼位置。用相同原始像素校验值关联PNG，而不是只靠接近的时间猜同帧。RGB/BGR按记录的格式转换；不匹配时必须记缺证。骨骼derived和node_world均保留，不预认其中之一与ModelState相同。

入口通过可选RENDER_BINDING_PLUGIN载入，缺省行为不变。观察器没有发布话题、人物指令或飞控输入。不能使用camera_actor_probe开关：旧probe会命令人物移动，本轮不启用。原render_observer保持。

观察器编译使用g++ -shared -fPIC -O2，以及pkg-config --cflags --libs gazebo OGRE和/usr/include/OGRE各子目录-I；保存源文件及so的SHA。构建位于/root/robo_team_build/render_binding_20261008，不改官方runtime。

## 要回答的问题及收尾

1. 新PNG证据包含完整绑定，服务响应名是否与配置相机相符？
2. PNG能否按原始像素校验值匹配渲染相机，渲染姿态是否与投影矩阵对应？
3. 画面人形能否由渲染骨骼位置解释，而物理ModelState是否相同？

取得明确同图反例即可主动结束并记录真实原因，不空等600秒。若校验值/骨骼取证缺失，先修取证链，不将缺证称根因。不原样重复第二轮。上限600秒；6/6、建筑接触、基础服务故障按既有入口收尾。五色精度与原图观察器尽早启动；实际裁判/接触/六机运动/高度/源码SHA另行统计，短轮0/6不作退化比较。
