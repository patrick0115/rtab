#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
import std_msgs.msg

class PointCloudToOccupancyGrid:
    def __init__(self):
        rospy.init_node('map3d_to_map2d', anonymous=True)
        rospy.loginfo("初始化點雲到佔用網格轉換節點")

        # 從參數服務器獲取參數
        self.resolution = rospy.get_param('~resolution', 0.02)
        # self.z_threshold = rospy.get_param('~z_threshold', 0.1)  # 垂直方向上的閾值
        self.point_threshold = rospy.get_param('~point_threshold', 50) 
        self.subscribe_topic = rospy.get_param('~subscribe_topic', '/map_3d')
        self.publish_grid_topic = rospy.get_param('~publish_grid_topic', '/map')

        # 設置訂閱者和發布者
        self.cloud_subscriber = rospy.Subscriber(self.subscribe_topic, PointCloud2, self.cloud_callback)
        self.grid_publisher = rospy.Publisher(self.publish_grid_topic, OccupancyGrid, queue_size=10)

        rospy.loginfo("訂閱者和發布者設置完成")

    def cloud_callback(self, cloud_msg):
        rospy.loginfo("收到點雲數據")

        point_list = [point for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y"))]
        # point_list = [point for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))]

        # 轉換點雲為佔用網格
        occupancy_grid = self.create_occupancy_grid(point_list)
        rospy.loginfo("佔用網格創建完成")

        # 發布佔用網格地圖
        self.grid_publisher.publish(occupancy_grid)

    def create_occupancy_grid(self, points):
        # 計算點雲的邊界
        min_x = min(points, key=lambda point: point[0])[0]
        min_y = min(points, key=lambda point: point[1])[1]
        max_x = max(points, key=lambda point: point[0])[0]
        max_y = max(points, key=lambda point: point[1])[1]

        # 計算佔用網格的尺寸
        width = int((max_x - min_x) / self.resolution) + 1
        height = int((max_y - min_y) / self.resolution) + 1

        grid = OccupancyGrid()
        grid.header = std_msgs.msg.Header(frame_id="map", stamp=rospy.Time.now())
        grid.info.resolution = self.resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = min_x - (self.resolution / 2)
        grid.info.origin.position.y = min_y - (self.resolution / 2)
        grid.info.origin.position.z = 0
        grid.info.origin.orientation.w = 1

        # 初始化網格數據
        grid_data = -1 * np.ones((height, width), dtype=int)  # 初始化所有值為-1

        # 創建一個空的3D Numpy數組來計算每個單元的點數
        point_counts = np.zeros((height, width))
        occupied_cells = 0
        for x, y in points:
            # if abs(z) <= self.z_threshold:  # 只考慮Z軸閾值內的點
            grid_x = int((x - min_x) / self.resolution)
            grid_y = int((y - min_y) / self.resolution)
            point_counts[grid_y, grid_x] += 1

        # 根據點的數量更新網格數據
        for x in range(width):
            for y in range(height):
                if point_counts[y, x] >= self.point_threshold:
                    grid_data[y, x] = 100  # 標記
                    occupied_cells += 1

        rospy.loginfo(f"總共有 {occupied_cells} 個格被標記為佔用")

        grid.data = list(grid_data.ravel())

        return grid

if __name__ == '__main__':
    try:
        PointCloudToOccupancyGrid()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
