#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import struct
import std_msgs.msg
def convert_to_ros_point_cloud(pcd, frame_id="map"):
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
 
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]

    header = std_msgs.msg.Header()
    header.frame_id = frame_id
    # unique_colors = set() 

    cloud_data = []

    for point, color in zip(points, colors):

        r, g, b = (int(c * 255) for c in color)
    
        # rgb_tuple = (r, g, b)
        # unique_colors.add(rgb_tuple)

        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
        cloud_data.append(point.tolist() + [rgb])

    # for color in unique_colors:
    #     print(color)
    ros_cloud = pc2.create_cloud(header, fields, cloud_data)
    return ros_cloud

def load_and_publish_pcd():
    rospy.init_node('pcd_publisher')

    # 获取 launch 文件中定义的参数
    pcd_file_path_of_3d_pcss_map = rospy.get_param('~pcd_file_path_of_3d_pcss_map')
    topic_of_3d_pcss_map = rospy.get_param('~topic_of_3d_pcss_map')
    frame_id = rospy.get_param('~frame_id', 'map')

    rospy.loginfo(f"pcd_file_path_of_3d_pcss_map: {pcd_file_path_of_3d_pcss_map}")
    rospy.loginfo(f"topic_of_3d_pcss_map: {topic_of_3d_pcss_map}")
    rospy.loginfo(f"frame_id: {frame_id}")

    if not pcd_file_path_of_3d_pcss_map:
        rospy.logerr("PCD 文件路徑未設定。請在 launch 文件中指定 'pcd_file_path_of_3d_pcss_map' 參數。")
        return

    if not topic_of_3d_pcss_map:
        rospy.logerr("ROS 主題未設定。請在 launch 文件中指定 'topic_of_3d_pcss_map' 參數。")
        return

    try:
        pcd_publisher = rospy.Publisher(topic_of_3d_pcss_map, PointCloud2, queue_size=10)
        rospy.loginfo(f"正在發布點雲到主題: {topic_of_3d_pcss_map}")
    except rospy.ROSException as e:
        rospy.logerr(f"創建 ROS 發布者時出錯: {e}")
        return

    rate = rospy.Rate(10000)  # 每秒发布一次

    # 读取 PCD 文件
    try:
        pcd = o3d.io.read_point_cloud(pcd_file_path_of_3d_pcss_map)
        rospy.loginfo(f"PCD 文件讀取成功: {pcd_file_path_of_3d_pcss_map}")
    except Exception as e:
        rospy.logerr(f"讀取 PCD 文件時出錯: {e}")
        return

    while not rospy.is_shutdown():
        ros_cloud = convert_to_ros_point_cloud(pcd, frame_id)
        pcd_publisher.publish(ros_cloud)
        rate.sleep()

if __name__ == '__main__':
    load_and_publish_pcd()