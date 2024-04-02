#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class FilteredPointCloudPublisher:
    def __init__(self):
        rospy.init_node('ground_detection_seg', anonymous=True)
        rospy.loginfo("初始化篩選點雲發布節點")

        self.filter_z_min = rospy.get_param('~filter_z_min', -0.5)
        self.filter_z_max = rospy.get_param('~filter_z_max', 3.0)

        self.min_special_color_count = rospy.get_param('~min_special_color_count', 2)


        self.topic_of_3d_pcss_map = rospy.get_param('~topic_of_3d_pcss_map', '/topic_of_3d_pcss_map')
        self.topic_obstacle_3d_map = rospy.get_param('~topic_obstacle_3d_map', '/topic_obstacle_3d_map')

        rospy.loginfo(f"訂閱主題: {self.topic_of_3d_pcss_map}")
        rospy.loginfo(f"發布主題: {self.topic_obstacle_3d_map}")

        self.subscriber = rospy.Subscriber(self.topic_of_3d_pcss_map, PointCloud2, self.callback)
        self.obstacle_3d_map_pub = rospy.Publisher(self.topic_obstacle_3d_map, PointCloud2, queue_size=10)

        rospy.loginfo("訂閱者和發布者設置完成")

        self.colors_to_remove = [(80, 50, 50), (120, 120, 80), (255, 173, 0)]

    def callback(self, cloud_msg):
        filtered_points = []
        special_color_points = {}
        for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
            x, y, z, rgb = point
            r, g, b, a = self.unpack_rgba(rgb)

            if (r, g, b) == (120, 120, 120):
                key = (round(x, 2), round(y, 2))
                special_color_points.setdefault(key, []).append((x, y, z, rgb))
            elif self.filter_z_min <= z <= self.filter_z_max and (r, g, b) not in self.colors_to_remove:
                rgb_packed = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                filtered_points.append([x, y, z, rgb_packed])

        for points in special_color_points.values():
            if len(points) >= self.min_special_color_count:
                for x, y, z, rgb in points:
                    r, g, b, a = self.unpack_rgba(rgb)
                    rgb_packed = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    filtered_points.append([x, y, z, rgb_packed])

        self.publish_filtered_cloud(filtered_points, cloud_msg.header)

    def unpack_rgba(self, rgb):
        b = rgb & 0xFF
        g = (rgb >> 8) & 0xFF
        r = (rgb >> 16) & 0xFF
        a = (rgb >> 24) & 0xFF
        return r, g, b, a

    def publish_filtered_cloud(self, points, header):
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')] + \
                 [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
        cloud_filtered_msg = pc2.create_cloud(header, fields, points)
        self.obstacle_3d_map_pub.publish(cloud_filtered_msg)

if __name__ == '__main__':
    try:
        node = FilteredPointCloudPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
