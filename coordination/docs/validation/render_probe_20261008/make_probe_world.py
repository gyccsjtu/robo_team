#!/usr/bin/env python3
"""Build a probe world: the real city scene + one static camera looking at actor_0.

The camera copies the official cgo3 sensor parameters exactly (640x360,
horizontal_fov 2.0, clip 0.05/15000, update_rate 10) so the frame rate we measure
transfers to the real pipeline. No aircraft, no PX4, no coordination stack.
"""
import os

SRC = ("/root/robocup_runs/wb_v141_validation3_20261008T104200Z/round_1_seed_8159"
       "/flight/connectivity.world")
OUT = "/root/render_ab"

# actor_0 sits at (53, -3). Put the camera 18 m east of it, 5 m up, yawed 180 deg
# to look back west and pitched down ~12.5 deg to frame a 1.75 m person.
CX, CY, CZ = 71.0, -3.0, 5.0
PITCH = 0.218
YAW = 3.14159

PROBE = '''
    <model name="cam_probe">
      <static>true</static>
      <pose>%.3f %.3f %.3f 0 %.5f %.5f</pose>
      <link name="probe_link">
        <sensor name="probe_camera" type="camera">
          <pose>0 0 0 0 0 0</pose>
          <camera>
            <horizontal_fov>2.0</horizontal_fov>
            <image>
              <format>R8G8B8</format>
              <width>640</width>
              <height>360</height>
            </image>
            <clip>
              <near>0.05</near>
              <far>15000</far>
            </clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <visualize>false</visualize>
          <plugin name="probe_cam" filename="libgazebo_ros_camera.so">
            <robotNamespace>/probe</robotNamespace>
            <cameraName>camera</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>probe_link</frameName>
          </plugin>
        </sensor>
      </link>
    </model>
''' % (CX, CY, CZ, PITCH, YAW)

os.makedirs(OUT, exist_ok=True)
s = open(SRC, encoding="utf-8").read()
i = s.rfind("</world>")
if i < 0:
    raise SystemExit("no </world> in source")
dst = os.path.join(OUT, "probe.world")
with open(dst, "w", encoding="utf-8") as fh:
    fh.write(s[:i] + PROBE + "\n  " + s[i:])
print("wrote %s (%d bytes, source %d)" % (dst, os.path.getsize(dst), len(s)))
print("camera at (%.2f, %.2f, %.2f) pitch=%.3f yaw=%.3f looking at actor_0 (53, -3)"
      % (CX, CY, CZ, PITCH, YAW))
