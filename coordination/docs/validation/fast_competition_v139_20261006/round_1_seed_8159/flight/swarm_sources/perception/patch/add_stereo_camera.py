#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 typhoon_h480 装上官方规格的双目相机（left / right 两路图像话题）。

为什么必须做这一步
------------------
本仓库的感知节点订阅 ``/<uav>/stereo_camera/left/image_raw``。
PX4 原厂的 typhoon_h480 只有云台相机（cgo3），**没有** stereo_camera
⇒ 不打这个 patch，话题根本不存在，表现是"节点在跑、但一条检测都没有"。

🔴 为什么不用 `model://stereo_camera` 直接 include（踩过的坑）
-------------------------------------------------------------
XTDrone 官方的 ``stereo_camera`` 模型用的是
``<sensor type="multicamera">`` + ``libgazebo_ros_multicamera.so``。
在本机（gazebo 9 + ROS Noetic）实测：插件能加载、**话题也 advertise 出来了**
（``/…/stereo_camera/left|right/image_raw`` 都在 ``rostopic list`` 里），
但**永远收不到图像帧** —— 这正是最难查的一类故障（话题在，所以看着像"对"）。

所以本脚本改用**两只独立单目** ``<sensor type="camera">`` +
``libgazebo_ros_camera.so``（机上的 cgo3、拍数据集用的相机都是这条路，验证过能出图），
**光学参数与基线严格照抄官方 stereo_camera，一个数都不改**：

    752x480, hfov 1.5708 (90deg), fx=fy=376, cx=376, cy=240,
    clip 0.1~300, 30Hz, 基线 0.12 m (y = ±0.06), 畸变系数同平台

话题名保持官方约定，所以对下游（感知节点 / 数据集）完全透明::

    /<uav>/stereo_camera/left/image_raw
    /<uav>/stereo_camera/right/image_raw

改哪些文件
----------
    <PX4>/Tools/sitl_gazebo/models/<model>/<model>.sdf        Gazebo 实际加载
    <PX4>/Tools/sitl_gazebo/models/<model>/<model>.sdf.jinja  防 PX4 重新编译时丢失

用法
----
    python3 add_stereo_camera.py --check              # 只检查状态，不改动
    python3 add_stereo_camera.py                      # 打 patch（自动备份）
    python3 add_stereo_camera.py --px4-root ~/PX4_Firmware --model typhoon_h480
    python3 add_stereo_camera.py --revert             # 从 .bak 回滚

打完必做：重启 PX4 SITL 后**看有没有帧**（不是只看话题在不在）::

    rostopic hz /typhoon_h480_0/stereo_camera/left/image_raw     # 应 ≈30 Hz
"""
import argparse
import os
import shutil
import sys

BAK_SUFFIX = ".bak_robocup_stereo"

GOOD_MARK = "stereo_camera_left"        # 已经装好单目等价实现
BAD_MARK = "stereo_camera_joint"        # 旧版 include multicamera 的残留（收不到图）

# 官方 stereo_camera 的光学参数（照抄，勿改）
SENSOR_TMPL = """
      <!-- ===== RoboCup 感知用：平台双目相机 {side} 目（等价实现）=====
           官方 stereo_camera 是 <sensor type="multicamera"> + libgazebo_ros_multicamera.so，
           在本机 gazebo9/Noetic 下能加载、话题也在，但永远收不到图像帧；
           故改用单目 camera + libgazebo_ros_camera.so（本机验证可出图）。
           光学参数与基线严格沿用官方 model://stereo_camera，未做任何修改：
           752x480, hfov 1.5708(90deg), fx=fy=376, cx=376, cy=240, 基线 0.12m, 30Hz
           安装位姿为本队自定：机身前方 0.12m、下沉 0.05m、左右各偏 0.06m。 ===== -->
      <sensor name="stereo_camera_{side}" type="camera">
        <pose>{x} {y} -0.05 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>1.5708</horizontal_fov>
          <image>
            <width>752</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>300</far>
          </clip>
        </camera>
        <plugin name="stereo_{side}_ctrl" filename="libgazebo_ros_camera.so">
          <cameraName>stereo_camera/{side}</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>stereo_camera_{side}_frame</frameName>
          <hackBaseline>{baseline}</hackBaseline>
          <Fx>376.0</Fx>
          <Fy>376.0</Fy>
          <Cx>376.0</Cx>
          <Cy>240.0</Cy>
          <distortionK1>-0.1</distortionK1>
          <distortionK2>0.01</distortionK2>
          <distortionK3>0.0</distortionK3>
          <distortionT1>5e-5</distortionT1>
          <distortionT2>-1e-4</distortionT2>
        </plugin>
      </sensor>
"""

SENSORS = (SENSOR_TMPL.format(side="left", x="0.12", y="0.06", baseline="0.0")
           + SENSOR_TMPL.format(side="right", x="0.12", y="-0.06", baseline="0.12"))

CANDIDATE_ROOTS = [
    "~/PX4_Firmware/Tools/sitl_gazebo/models",
    "/data/PX4_Firmware/Tools/sitl_gazebo/models",
    "~/XTDrone/sitl_config/models",
    "/data/XTDrone/sitl_config/models",
]


def find_model_dirs(model, px4_root=None):
    roots = []
    if px4_root:
        roots.append(os.path.join(px4_root, "Tools", "sitl_gazebo", "models"))
        roots.append(px4_root)
    roots += CANDIDATE_ROOTS

    found, seen = [], set()
    for r in roots:
        d = os.path.join(os.path.expanduser(r), model)
        if not os.path.isdir(d):
            continue
        real = os.path.realpath(d)
        if real in seen:            # 软链导致的同一目录只算一次
            continue
        seen.add(real)
        found.append(d)
    return found


def state_of(path):
    if not os.path.isfile(path):
        return "MISSING"
    src = open(path, encoding="utf-8", errors="ignore").read()
    if GOOD_MARK in src:
        n = src.count('name="stereo_camera_') + src.count("name='stereo_camera_")
        return "INSTALLED (%d sensor)" % n
    if BAD_MARK in src:
        return "BAD_INCLUDE"        # 旧的多目 include：话题在，但没有帧
    return "NOT_INSTALLED"


def show(dirs, model):
    print("=" * 74)
    for d in dirs:
        kind = "PX4" if "PX4_Firmware" in d else "XTDrone(副本)"
        print("[%s] %s" % (kind, d))
        for ext in (".sdf", ".sdf.jinja"):
            name = model + ext
            st = state_of(os.path.join(d, name))
            note = ""
            if st == "BAD_INCLUDE":
                note = "   <-- 坏方案！话题在但收不到帧，需重打"
            elif st == "INSTALLED (%d sensor)" % 2:
                note = "   <-- 正常"
            elif st == "MISSING":
                note = "   (此目录无此文件，正常)"
            print("   %-26s %s%s" % (name, st, note))
        mc = os.path.join(d, "model.config")
        if os.path.isfile(mc):
            src = open(mc, encoding="utf-8", errors="ignore").read()
            import re
            m = re.search(r"<sdf[^>]*>([^<]+)</sdf>", src)
            print("   %-26s model.config -> %s" % (model + "/model.config",
                                                   m.group(1) if m else "?"))
    print("=" * 74)


def strip_bad_include(src):
    """移除旧版 <include>model://stereo_camera</include> 及其 fixed joint。"""
    import re
    n0 = len(src)
    src = re.sub(r"[ \t]*<!--[^>]*?stereo_camera[^>]*?-->\s*", "", src, flags=re.S)
    src = re.sub(r"[ \t]*<include>\s*<uri>\s*model://stereo_camera\s*</uri>.*?</include>\s*",
                 "", src, flags=re.S)
    src = re.sub(r"[ \t]*<joint name=\"stereo_camera_joint\".*?</joint>\s*",
                 "", src, flags=re.S)
    return src, (n0 != len(src))


def patch_one(path, model, dry=False):
    st = state_of(path)
    if st == "MISSING":
        return "MISSING"
    if st.startswith("INSTALLED"):
        return "ALREADY"

    src = open(path, encoding="utf-8").read()
    src, cleaned = strip_bad_include(src)
    if dry:
        return "WOULD_PATCH%s" % (" (+clean bad include)" if cleaned else "")

    bak = path + BAK_SUFFIX
    if not os.path.isfile(bak):
        shutil.copy2(path, bak)

    i = src.find("<link name='base_link'>")
    if i < 0:
        i = src.find('<link name="base_link">')
    if i < 0:
        return "NO_BASE_LINK"
    j = src.find("</link>", i)
    if j < 0:
        return "NO_BASE_LINK_END"

    out = src[:j] + SENSORS + src[j:]
    with open(path, "w", encoding="utf-8") as f:
        f.write(out)

    chk = open(path, encoding="utf-8").read()
    ok = (GOOD_MARK in chk and "stereo_camera_right" in chk and BAD_MARK not in chk)
    return "OK (%d -> %d bytes)" % (len(src), len(chk)) if ok else "FAIL_VERIFY"


def revert_one(path):
    bak = path + BAK_SUFFIX
    if not os.path.isfile(bak):
        return "NO_BACKUP"
    shutil.copy2(bak, path)
    return "REVERTED"


def main():
    ap = argparse.ArgumentParser(
        description="给 typhoon_h480 装官方规格双目相机（两只单目传感器）")
    ap.add_argument("--model", default="typhoon_h480", help="机体模型名（默认 typhoon_h480）")
    ap.add_argument("--px4-root", default=os.environ.get("ROBOCUP_PX4_ROOT"),
                    help="PX4_Firmware 根目录（默认自动探测）")
    ap.add_argument("--check", action="store_true", help="只检查，不改动")
    ap.add_argument("--revert", action="store_true", help="从 .bak 回滚")
    args = ap.parse_args()

    dirs = find_model_dirs(args.model, args.px4_root)
    if not dirs:
        print("[!] 没找到 %s 模型目录。" % args.model)
        print("    用 --px4-root 指定 PX4_Firmware 路径，例如：")
        print("      python3 add_stereo_camera.py --px4-root ~/PX4_Firmware")
        return 1

    show(dirs, args.model)
    if args.check:
        return 0

    rc = 0
    for d in dirs:
        for ext in (".sdf", ".sdf.jinja"):
            p = os.path.join(d, args.model + ext)
            if not os.path.isfile(p):
                continue
            r = revert_one(p) if args.revert else patch_one(p, args.model)
            print("%-44s %s" % (os.path.relpath(p, os.path.expanduser("~")), r))
            if r.startswith("FAIL") or r.startswith("NO_"):
                rc = 1

    if not args.revert:
        print()
        print("下一步（务必验证『有帧』而不只是『话题在』）：")
        print("  重启 PX4 SITL，然后：")
        print("    rostopic hz /%s_0/stereo_camera/left/image_raw     # 应 ≈30 Hz" % args.model)
        print("  只看到 `rostopic list` 里有话题、但 hz 一直是 no new messages，")
        print("  说明装成了 multicamera 版本 —— 那种方案在本机收不到帧。")
    return rc


if __name__ == "__main__":
    sys.exit(main())
