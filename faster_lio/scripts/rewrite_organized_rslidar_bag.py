#!/usr/bin/env python3
"""Rewrite an organized RoboSense PointCloud2 so it carries the official ring field.

This does not invent hardware per-point timestamps. Those exist only inside
rslidar_sdk when it is compiled with POINT_TYPE=XYZIRT (README section 1.2,
CHANGELOG v1.2.0). The existing XYZI bags never recorded them.

Ring assignment follows the vendor organized layout in
rs_driver/src/rs_driver/driver/lidar_driver_impl.hpp: after decode, the
cloud is stored row-major with height = LASER_NUM, so row i is laser
channel i. Helios-16 bags in this project are height=16, width=1800.

Per-point time is left unset (0). Faster-LIO / FAST-LIO2 then apply the
published spinning-lidar undistortion used when the last point time is 0:
offset from yaw at a constant 10 Hz scan rate (Bai et al., RA-L 2022,
VelodyneHandler / HesaiHandler fallback; same model as FAST-LIO2).
"""

from __future__ import annotations

import argparse
import sys

import rosbag
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header


def organized_rings(msg):
    if msg.height < 2 or msg.width < 2:
        raise ValueError(
            "cloud is not organized (height=%s width=%s); cannot recover ring "
            "from the official rslidar_sdk row-major layout" % (msg.height, msg.width)
        )
    xyz = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=False))
    if len(xyz) != msg.height * msg.width:
        raise ValueError(
            "point count %s != height*width %s" % (len(xyz), msg.height * msg.width)
        )
    out = []
    for row in range(msg.height):
        for col in range(msg.width):
            x, y, z, intensity = xyz[row * msg.width + col]
            out.append((x, y, z, intensity, row))
    return out


def rewrite_cloud(msg):
    points = organized_rings(msg)
    header = Header()
    header.seq = msg.header.seq
    header.stamp = msg.header.stamp
    header.frame_id = msg.header.frame_id
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
    ]
    rewritten = pc2.create_cloud(header, fields, points)
    rewritten.height = msg.height
    rewritten.width = msg.width
    rewritten.is_dense = msg.is_dense
    rewritten.row_step = rewritten.point_step * rewritten.width
    return rewritten


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag")
    parser.add_argument("output_bag")
    parser.add_argument("--cloud-topic", default="/rslidar_points")
    args = parser.parse_args()

    rewritten = 0
    with rosbag.Bag(args.input_bag, "r") as src, rosbag.Bag(args.output_bag, "w") as dst:
        for topic, msg, t in src.read_messages():
            if topic == args.cloud_topic:
                msg = rewrite_cloud(msg)
                rewritten += 1
            dst.write(topic, msg, t)
    if rewritten == 0:
        print("no messages on %s" % args.cloud_topic, file=sys.stderr)
        return 1
    print("rewrote %s clouds from %s -> %s" % (rewritten, args.input_bag, args.output_bag))
    return 0


if __name__ == "__main__":
    sys.exit(main())
