#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class GroundDetectionNode(object):
    def __init__(self):
        self.topic_of_3d_pcss_map = rospy.get_param('~topic_of_3d_pcss_map', '/input_point_cloud')
        self.topic_of_ground = rospy.get_param('~topic_of_ground', '/ground')
        self.topic_obstacle_3d_map = rospy.get_param('~topic_obstacle_3d_map', '/obstacle_3d_map')
        self.voxel_grid_leaf_size = rospy.get_param('~voxel_grid_leaf_size', 0.1)
        self.max_distance = rospy.get_param('~max_distance', 0.1)

        rospy.loginfo("參數設定如下：")
        rospy.loginfo(f"Input Point Cloud Topic: {self.topic_of_3d_pcss_map}")
        rospy.loginfo(f"Ground Point Cloud Topic: {self.topic_of_ground}")
        rospy.loginfo(f"Obstacle Point Cloud Topic: {self.topic_obstacle_3d_map}")
        rospy.loginfo(f"Voxel Grid Leaf Size: {self.voxel_grid_leaf_size}")
        rospy.loginfo(f"Max Distance for RANSAC: {self.max_distance}")

        self.ground_pub = rospy.Publisher(self.topic_of_ground, PointCloud2, queue_size=10)
        self.obstacle_3d_map_pub = rospy.Publisher(self.topic_obstacle_3d_map, PointCloud2, queue_size=10)
        rospy.Subscriber(self.topic_of_3d_pcss_map, PointCloud2, self.pcl_callback)

    def pcl_callback(self, pcl_msg):
        rospy.loginfo("接收到點雲數據...")
        pcl_cloud = self.point_cloud2_to_xyz_array(pcl_msg)
        
        rospy.loginfo("進行下採樣...")
        pcl_cloud_downsampled = self.voxel_grid_downsampling(pcl_cloud, self.voxel_grid_leaf_size)
        
        rospy.loginfo("進行RANSAC地面分割...")
        ground_indices, obstacle_indices = self.segment_ground_plane(pcl_cloud_downsampled, self.max_distance)
        
        ground_pcl = pcl_cloud_downsampled.extract(ground_indices, negative=False)
        obstacle_pcl = pcl_cloud_downsampled.extract(obstacle_indices, negative=False)
        
        rospy.loginfo("準備發布分割後的點雲數據...")
        ground_ros_msg = self.xyz_array_to_point_cloud2(ground_pcl.to_array(), pcl_msg.header.frame_id)
        obstacle_ros_msg = self.xyz_array_to_point_cloud2(obstacle_pcl.to_array(), pcl_msg.header.frame_id)
        
        self.ground_pub.publish(ground_ros_msg)
        self.obstacle_3d_map_pub.publish(obstacle_ros_msg)
        rospy.loginfo("點雲數據發布完成.")

    def point_cloud2_to_xyz_array(self, cloud_msg):
        points_list = []
        for data in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            points_list.append([data[0], data[1], data[2]])
        pcl_cloud = pcl.PointCloud(np.array(points_list, dtype=np.float32))
        return pcl_cloud

    def xyz_array_to_point_cloud2(self, cloud_array, frame_id):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        return pc2.create_cloud_xyz32(header, cloud_array)

    def voxel_grid_downsampling(self, pcl_cloud, leaf_size):
        sor = pcl_cloud.make_voxel_grid_filter()
        sor.set_leaf_size(leaf_size, leaf_size, leaf_size)
        return sor.filter()

    def segment_ground_plane(self, pcl_cloud, max_distance):
        seg = pcl_cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(max_distance)
        inliers, coefficients = seg.segment()
        if inliers:
            return inliers, [index for index in range(pcl_cloud.size) if index not in inliers]
        else:
            return [], []

if __name__ == '__main__':
    rospy.init_node('ground_detection_node', anonymous=True)
    node = GroundDetectionNode()
    rospy.spin()
