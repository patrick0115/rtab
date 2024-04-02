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
        self.normal_search_radius = rospy.get_param('~normal_search_radius', 0.05)
        self.angle_threshold = rospy.get_param('~angle_threshold', np.pi/6)
        self.voxel_size = rospy.get_param('~voxel_size', 0.01)

        rospy.loginfo(f"Input Point Cloud Topic: {self.topic_of_3d_pcss_map}")
        rospy.loginfo(f"Ground Point Cloud Topic: {self.topic_of_ground}")
        rospy.loginfo(f"Obstacle Point Cloud Topic: {self.topic_obstacle_3d_map}")
        rospy.loginfo(f"Normal Search Radius: {self.normal_search_radius}")
        rospy.loginfo(f"Angle Threshold: {self.angle_threshold}")
        rospy.loginfo(f"Voxel Size: {self.voxel_size}")

        self.ground_pub = rospy.Publisher(self.topic_of_ground, PointCloud2, queue_size=10)
        self.obstacle_3d_map_pub = rospy.Publisher(self.topic_obstacle_3d_map, PointCloud2, queue_size=10)
        rospy.Subscriber(self.topic_of_3d_pcss_map, PointCloud2, self.pcl_callback)

    def pcl_callback(self, pcl_msg):
        rospy.loginfo("接收到點雲數據...")
        pcl_cloud = self.point_cloud2_to_pcl(pcl_msg)
        rospy.loginfo("點雲轉換完成.")

        rospy.loginfo("開始Voxel Grid下採樣...")
        pcl_cloud_filtered = self.apply_voxel_grid_filter(pcl_cloud, self.voxel_size)
        rospy.loginfo("Voxel Grid下採樣完成，點雲數量減少.")

        rospy.loginfo("開始進行地面和障礙物分割...")
        ground_indices, obstacle_indices = self.filter_ground_plane(pcl_cloud_filtered, self.normal_search_radius, self.angle_threshold)
        rospy.loginfo("地面和障礙物分割完成.")

        ground_pcl = pcl_cloud_filtered.extract(ground_indices, negative=False)
        obstacle_pcl = pcl_cloud_filtered.extract(obstacle_indices, negative=False)

        rospy.loginfo(f"地面點雲數量: {len(ground_indices)}")
        rospy.loginfo(f"障礙物點雲數量: {len(obstacle_indices)}")

        ground_ros_msg = self.pcl_to_point_cloud2(ground_pcl, pcl_msg.header.frame_id)
        obstacle_ros_msg = self.pcl_to_point_cloud2(obstacle_pcl, pcl_msg.header.frame_id)
        self.ground_pub.publish(ground_ros_msg)
        self.obstacle_3d_map_pub.publish(obstacle_ros_msg)
        rospy.loginfo("點雲數據發布完成.")

    def point_cloud2_to_pcl(self, cloud_msg):
        points_list = []
        for data in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            points_list.append([data[0], data[1], data[2]])
        pcl_cloud = pcl.PointCloud()
        pcl_cloud.from_list(points_list)
        return pcl_cloud

    def apply_voxel_grid_filter(self, pcl_cloud, voxel_size):
        vg = pcl_cloud.make_voxel_grid_filter()
        vg.set_leaf_size(voxel_size, voxel_size, voxel_size)
        return vg.filter()

    def pcl_to_point_cloud2(self, pcl_cloud, frame_id):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        return pc2.create_cloud_xyz32(header, pcl_cloud.to_array())

    def filter_ground_plane(self, pcl_cloud, normal_search_radius, angle_threshold):
        ne = pcl_cloud.make_NormalEstimation()
        tree = pcl_cloud.make_kdtree()
        ne.set_SearchMethod(tree)
        ne.set_RadiusSearch(normal_search_radius)
        normals = ne.compute()

        ground_indices = []
        obstacle_indices = []
        for i, normal in enumerate(normals):
            angle = np.arccos(normal[2])  # Calculate the angle with the vertical
            if angle <= angle_threshold:
                ground_indices.append(i)
            else:
                obstacle_indices.append(i)
        return ground_indices, obstacle_indices

if __name__ == '__main__':
    rospy.init_node('ground_detection_node', anonymous=True)
    node = GroundDetectionNode()
    rospy.spin()
