# 原图与相机绑定取证补丁

## 问题与本次计划

坏原图中的人形与该图记录的六个官方人物位置不相容，两个正常原图对照则基本相容，详见 `n12_camera_identity_followup_20261008.md`。现有记录缺原始image header和所选相机服务采样，无法区分图像来源错误、渲染状态问题与非评分人形。先补实际数据，不猜全局偏移、不调YOLO门限，不启动新仿真。

## 已实现

- on_img在同一图像缓存锁内保存原图header.frame_id和header.seq；检测线程读取图像、原图时间、header、接收时间及年龄时使用同一把锁。
- 相机历史采样保留GetLinkState响应的link_name、reference_frame、原始link位置/四元数、服务调用前后ROS时间。原有五个采样成员位置不变，追加第六个内部诊断成员。
- pose_at_stamp增加可选evidence参数，记录实际选择的前后采样、旋转插值alpha、实际投影使用的相机位置和旋转矩阵。原插值计算和拒绝条件不变，旧五成员样本兼容。诊断不重新对齐、猜测或修正位置。
- 既有通过/失败PNG索引增加camera_binding及effective_conf；仍沿用原限流和总量上限，只有保存入选原图才将这份绑定证据落盘。没有新增逐帧大文件。

## 记录兼容

外层EvidenceCapture schema_version=1保留，新增可选camera_binding对象，其schema_version=1。旧索引没有该字段，表示绑定证据缺失，不能推断正确；旧读取器可继续读取原字段。task、route、visual和共享推理接口没有修改。

camera_binding包含image_header、image_topic、configured_link、camera_info_topic、camera_offset_link、image_recv_wall、image_receive_age_s、intrinsics、size、image_s、first、second、rotation_alpha、camera_xyz和camera_rotation。first/second里的service包含before_s、after_s、link_name、reference_frame、link_xyz和link_quaternion_xyzw。camera_rotation按行展开，是投影实际使用的旋转；service里的四元数是各服务采样的原始link四元数，不能混用。旧debug cam中的四元数来自另一次当前服务调用，不作为同图姿态证据。

image_receive_age_s是接收回调时的ROS时间减原图时间，image_recv_wall是墙钟，两种时间不可相减。service的前后时间给出调用区间，sample_s是原有中点估计，不能冒称Gazebo提供了准确的姿态采集时间。header只是实际记录，尚未按frame_id拒绝图像，也未证明渲染姿态正确。

## 离线验证

相机相关30项、EvidenceCapture 6项通过，0失败0跳过；perception_real.py编译检查及git diff --check通过。新增五项实际源码函数检查：原始header复制、实际服务采样区间及身份、选择正确历史括号及矩阵、拒绝不制造诊断、旧样本兼容。带/不带诊断返回相同位置和旋转，JSON可序列化。

最初将两个不同测试根复用同一个unittest discover上下文产生导入错误，已分开执行；不是生产服务故障。尚无本补丁实体执行证明，不能称63米坐标问题已修好。

## 下一步

有目的的下一次实体验证使用统一新快照，在既有通过/失败原图流核对header与配置link、前后样本与实际投影矩阵，并同步只读取人物骨骼/模型证据。人物真值不进入控制。先取得唯一根因，再作定位或模型修复；不原样重复旧轮。
