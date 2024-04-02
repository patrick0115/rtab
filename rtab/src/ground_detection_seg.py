#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import struct

class FilteredPointCloudPublisher:
    def __init__(self):
        rospy.init_node('ground_detection_seg', anonymous=True)
        rospy.loginfo("初始化篩選點雲發布節點")

        self.filter_z_min = rospy.get_param('~filter_z_min', -0.5)
        self.filter_z_max = rospy.get_param('~filter_z_max', 3.0)

        topic_of_3d_pcss_map = rospy.get_param('~topic_of_3d_pcss_map', '/topic_of_3d_pcss_map')
        topic_obstacle_3d_map = rospy.get_param('~topic_obstacle_3d_map', '/topic_obstacle_3d_map')

        rospy.loginfo(f"訂閱主題: {topic_of_3d_pcss_map}")
        rospy.loginfo(f"發布主題: {topic_obstacle_3d_map}")

        self.subscriber = rospy.Subscriber(topic_of_3d_pcss_map, PointCloud2, self.callback)
        self.obstacle_3d_map_pub = rospy.Publisher(topic_obstacle_3d_map, PointCloud2, queue_size=10)

        rospy.loginfo("訂閱者和發布者設置完成")

    def callback(self, cloud_msg):
        filtered_points = []
        for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
            x, y, z, rgb = point
            r, g, b, a = self.unpack_rgba(rgb)
            if self.filter_z_min <= z <= self.filter_z_max and not self.is_specific_color(r, g, b):
                rgb_packed = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                filtered_points.append([x, y, z, rgb_packed])

        self.publish_filtered_cloud(filtered_points, cloud_msg.header)

    def is_specific_color(self, r, g, b):
        colors_to_remove = [(80, 50, 50), (120, 120, 80), (255, 173, 0)]
        return (r, g, b) in colors_to_remove

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
