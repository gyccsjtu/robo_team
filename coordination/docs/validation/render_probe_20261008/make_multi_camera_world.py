#!/usr/bin/env python3
"""Add N-1 extra cameras to probe.world so we can measure how frame rate and
image age scale with the number of cameras (the single-camera probe runs at
6.76 Hz / 0.152 s; the six-aircraft round ran at 2.5 Hz / 0.786 s).

All cameras share the same pose, so the only variable is the count. The first
keeps the topic /probe/camera/image_raw so measure.py works unchanged.
"""
import os
import sys

N = int(sys.argv[1]) if len(sys.argv) > 1 else 6
SRC = "/root/render_ab/probe.world"
DST = "/root/render_ab/probe_cam%d.world" % N

CX, CY, CZ = 71.0, -3.0, 5.0
PITCH, YAW = 0.218, 3.14159


def block(i):
    return '''
    <model name="cam_probe_%d">
      <static>true</static>
      <pose>%.3f %.3f %.3f 0 %.5f %.5f</pose>
      <link name="probe_link">
        <sensor name="probe_camera" type="camera">
          <pose>0 0 0 0 0 0</pose>
          <camera>
            <horizontal_fov>2.0</horizontal_fov>
            <image><format>R8G8B8</format><width>640</width><height>360</height></image>
            <clip><near>0.05</near><far>15000</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <visualize>false</visualize>
          <plugin name="probe_cam" filename="libgazebo_ros_camera.so">
            <robotNamespace>/probe%d</robotNamespace>
            <cameraName>camera</cameraName>
            <imageTopicName>image_raw</imageTopicName>
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
            <frameName>probe_link</frameName>
          </plugin>
        </sensor>
      </link>
    </model>
''' % (i, CX, CY, CZ, PITCH, YAW, i)


s = open(SRC, encoding="utf-8").read()
extra = "".join(block(i) for i in range(2, N + 1))
i = s.rfind("</world>")
with open(DST, "w", encoding="utf-8") as fh:
    fh.write(s[:i] + extra + "\n  " + s[i:])
print("wrote %s : %d cameras total (first keeps /probe/camera)" % (DST, N))
