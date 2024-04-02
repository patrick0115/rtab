#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
import pcl
from pcl import PointCloud
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from std_msgs.msg import Header

def pcl_to_ros(pcl_array):
    """ 將PCL點雲數據轉換為ROS消息 """
    ros_msg = PointCloud2()
    ros_msg.header = Header(frame_id="map")
    ros_msg = pc2.create_cloud_xyz32(ros_msg.header, pcl_array.to_array())
    return ros_msg

def ros_to_pcl(ros_cloud):
    """ 將ROS點雲消息轉換為PCL對象 """
    points_list = []

    for data in pc2.read_points(ros_cloud, skip_nans=True):
        points_list.append([data[0], data[1], data[2]])

    pcl_data = PointCloud()
    pcl_data.from_list(points_list)

    return pcl_data

def cloud_callback(ros_cloud):
    """ 點雲回調函數，處理接收到的點雲數據 """
    # 將ROS點雲轉換為PCL格式
    pcl_cloud = ros_to_pcl(ros_cloud)

    # 體素格化降采樣
    # vox = pcl_cloud.make_voxel_grid_filter()
    # LEAF_SIZE = 0.01
    # vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # cloud_filtered = vox.filter()

    # 統計分析移除雜訊
    outlier_filter = pcl_cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    outlier_filter.set_std_dev_mul_thresh(1.0)
    cloud_filtered = outlier_filter.filter()

    # 將處理後的PCL點雲轉換回ROS消息
    ros_cloud_filtered = pcl_to_ros(cloud_filtered)

    # 發布處理後的點雲
    pub.publish(ros_cloud_filtered)

if __name__ == '__main__':
    # 初始化ROS節點
    rospy.init_node('point_cloud_processing_node', anonymous=True)

    # 讀取參數
    input_topic = rospy.get_param('~input_topic', '/rtabmap/cloud_map')
    output_topic = rospy.get_param('~output_topic', '/processed_cloud_map')
    
    # 印出參數
    rospy.loginfo('Input topic: %s', input_topic)
    rospy.loginfo('Output topic: %s', output_topic)

    pub = rospy.Publisher("/processed_cloud_map", PointCloud2, queue_size=1)
    rospy.Subscriber("/rtabmap/cloud_map", PointCloud2, cloud_callback)

    rospy.spin()
