#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
import struct
import std_msgs.msg

class PointCloudToOccupancyGrid:
    def __init__(self):
        rospy.init_node('pointcloud_to_occupancygrid', anonymous=True)
        rospy.loginfo("初始化點雲到佔用網格轉換節點")

        # 從參數服務器獲取參數
        self.resolution = rospy.get_param('resolution', 0.05)

        self.filter_z_min = rospy.get_param('filter_z_min', -0.5)
        self.filter_z_max = rospy.get_param('filter_z_max', 3.01)
        # 設置訂閱者和發布者
        subscribe_topic = rospy.get_param('subscribe_topic', '/map_3d')
        publish_grid_topic = rospy.get_param('publish_grid_topic', '/map')
        publish_filtered_3d_map_topic = rospy.get_param('publish_filtered_3d_map_topic', '/filtered_3d_map')

        # 印出參數
        rospy.loginfo(f"解析度 (resolution): {self.resolution}")
        rospy.loginfo(f"最小高度篩選 (filter_z_min): {self.filter_z_min}")
        rospy.loginfo(f"最大高度篩選 (filter_z_max): {self.filter_z_max}")
        rospy.loginfo(f"訂閱點雲主題 (subscribe_topic): {subscribe_topic}")
        rospy.loginfo(f"發布佔用網格主題 (publish_grid_topic): {publish_grid_topic}")
        rospy.loginfo(f"發布篩選後點雲主題 (publish_filtered_3d_map_topic): {publish_filtered_3d_map_topic}")
        rospy.loginfo("參數設定完成")

        self.cloud_subscriber = rospy.Subscriber(subscribe_topic, PointCloud2, self.cloud_callback)
        self.grid_publisher = rospy.Publisher(publish_grid_topic, OccupancyGrid, queue_size=10)
        self.filtered_map_publisher = rospy.Publisher(publish_filtered_3d_map_topic, PointCloud2, queue_size=50)

        rospy.loginfo("訂閱者和發布者設置完成")



    def cloud_callback(self, cloud_msg):
        pcl_cloud = pcl.PointCloud()
        point_list = []
        filtered_point_list = []
        unique_colors = set()  # 儲存不重複顏色的集合


        for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z", "rgb")):
            x, y, z, rgb = point        
            r, g, b, a = self.unpack_rgba(rgb)
    
            # 將RGB顏色轉換為整數元組，並加入集合中
            color = (r, g, b)
            unique_colors.add(color)
            if self.filter_z_min <= z <= self.filter_z_max:     
                if not self.is_specific_color(r, g, b):
                    point_list.append([x, y, z])
                    filtered_point_list.append([x, y, z, r, g, b])

        pcl_cloud.from_list(point_list)
        occupancy_grid = self.create_occupancy_grid(pcl_cloud)

        # 打印所有不重複的顏色
        # for color in unique_colors:
        #     rospy.loginfo(f"Unique Color: {color}")

        # 發布佔用網格地圖
        self.grid_publisher.publish(occupancy_grid)

        # 重新發布篩選後的點雲
        self.publish_filtered_cloud(filtered_point_list)

    def create_occupancy_grid(self, cloud):
    # 計算點雲的邊界
        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = float('-inf'), float('-inf')
        for point in cloud:
            x, y = point[0], point[1]
            min_x, min_y = min(min_x, x), min(min_y, y)
            max_x, max_y = max(max_x, x), max(max_y, y)

        # 計算中心點
        center_x, center_y = (max_x + min_x) / 2, (max_y + min_y) / 2

        # 計算佔用網格的尺寸
        self.width = int((max_x - min_x) / self.resolution) + 1
        self.height = int((max_y - min_y) / self.resolution) + 1

        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = rospy.Time.now() 
        grid.info.map_load_time = rospy.Time.now() 
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        # 設置原點為點雲的中心點，並考慮網格尺寸
        grid.info.origin.position.x = center_x - (self.width * self.resolution / 2)
        grid.info.origin.position.y = center_y - (self.height * self.resolution / 2)
        grid.info.origin.position.z = -1
        grid.info.origin.orientation.w = 1



        # 初始化網格
        grid_data = np.zeros((self.height, self.width))
        for point in cloud:
            x, y = point[0], point[1]
            grid_x = int((x - grid.info.origin.position.x) / self.resolution)
            grid_y = int((y - grid.info.origin.position.y) / self.resolution)
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                grid_data[grid_y, grid_x] = 100  # 佔用

        # 轉換為 1D 數組
        grid.data = grid_data.ravel().astype(int).tolist()
        return grid
    def is_specific_color(self, r, g, b):
        # 定義要刪除的顏色列表，每個顏色是一個 (r, g, b) 元組
        colors_to_remove = [
            (80, 50, 50),  # floor
            (120, 120, 80), # ceiling
            (255, 173, 0),#light
            # 可以繼續添加其他顏色
        ]
        return (r, g, b) in colors_to_remove

    def unpack_rgba(self, rgb):
        # 將浮點數格式的 RGBA 數據轉換為整數
        b = rgb & 0xFF
        g = (rgb >> 8) & 0xFF
        r = (rgb >> 16) & 0xFF
        a = (rgb >> 24) & 0xFF
        # print(r,g,b)
        return r, g, b, a
        # 解包 RGBA 數值
        # r, g, b, a = (rgba_int >> 24) & 0xff, (rgba_int >> 16) & 0xff, (rgba_int >> 8) & 0xff, rgba_int & 0xff
        # return r, g, b, a

    def publish_filtered_cloud(self, points):
        formatted_points = []
        for point in points:
            x, y, z, r, g, b = point
            # 將 r, g, b 值合成一個整數
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]  # 注意顏色順序和填充
            formatted_points.append([x, y, z, rgb])

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            # 注意這裡使用的是 UINT32 來存儲合成後的 RGB 值
            PointField('rgb', 12, PointField.UINT32, 1)
        ]
        cloud_msg = pc2.create_cloud(header, fields, formatted_points)
        self.filtered_map_publisher.publish(cloud_msg)


if __name__ == '__main__':
    try:
        converter = PointCloudToOccupancyGrid()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
