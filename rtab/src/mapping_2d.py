#!/usr/bin/env python
import rospy
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid

class PointCloudToOccupancyGrid:
    def __init__(self):
        rospy.init_node('pointcloud_to_occupancygrid', anonymous=True)
        self.cloud_subscriber = rospy.Subscriber("/merged_cloud", PointCloud2, self.cloud_callback)
        self.grid_publisher = rospy.Publisher("/converted_occupancy_grid", OccupancyGrid, queue_size=10)
        self.resolution = 0.05  # 網格解析度 (m)
        self.width = 2000  # 網格寬度 (格數)
        self.height = 3000  # 網格高度 (格數)
        self.filter_z_min = 0.02  # 最小高度 (m)
        self.filter_z_max = 0.6  # 最大高度 (m)

    def cloud_callback(self, cloud_msg):
        pcl_cloud = pcl.PointCloud()
        point_list = []

        for point in pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            point_list.append([point[0], point[1], point[2]])
        pcl_cloud.from_list(point_list)

        # 高度篩選
        filt = pcl_cloud.make_passthrough_filter()
        filt.set_filter_field_name("z")
        filt.set_filter_limits(self.filter_z_min, self.filter_z_max)
        cloud_filtered = filt.filter()

        # 轉換到佔用網格
        occupancy_grid = self.create_occupancy_grid(cloud_filtered)

        # 發布佔用網格地圖
        self.grid_publisher.publish(occupancy_grid)

    def create_occupancy_grid(self, cloud):
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = -self.width * self.resolution / 2
        grid.info.origin.position.y = -self.height * self.resolution / 2
        grid.info.origin.position.z = 0
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

if __name__ == '__main__':
    try:
        converter = PointCloudToOccupancyGrid()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
