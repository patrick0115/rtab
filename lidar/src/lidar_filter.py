#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan

class ScanFilter:
    def __init__(self):
        """
        初始化ScanFilter節點，從ROS參數服務器讀取配置。
        """
        # 初始化節點
        rospy.init_node('scan_filter', anonymous=True)

        # 從參數服務器讀取參數
        input_topic = rospy.get_param('~input_topic', '/scan')
        output_topic = rospy.get_param('~output_topic', '/filtered_scan')
        self.filter_distance = rospy.get_param('~filter_distance', 1.0)
        
        # 日誌輸出參數信息
        rospy.loginfo(f"輸入主題: {input_topic}")
        rospy.loginfo(f"輸出主題: {output_topic}")
        rospy.loginfo(f"濾波距離: {self.filter_distance} 公尺")

        # 創建發布者
        self.pub = rospy.Publisher(output_topic, LaserScan, queue_size=10)

        # 訂閱輸入主題
        rospy.Subscriber(input_topic, LaserScan, self.callback)

    def callback(self, scan):
        """
        處理接收到的LaserScan數據。
        :param scan: 原始LaserScan數據
        """
        filtered_scan = LaserScan()
        filtered_scan.header = scan.header
        filtered_scan.angle_min = scan.angle_min
        filtered_scan.angle_max = scan.angle_max
        filtered_scan.angle_increment = scan.angle_increment
        filtered_scan.time_increment = scan.time_increment
        filtered_scan.scan_time = scan.scan_time
        filtered_scan.range_min = scan.range_min
        filtered_scan.range_max = scan.range_max

        # 過濾距離
        filtered_scan.ranges = [
            r if r >= self.filter_distance else float('Inf') for r in scan.ranges
        ]
        filtered_scan.intensities = scan.intensities

        # 發布過濾後的數據
        self.pub.publish(filtered_scan)

if __name__ == '__main__':
    scan_filter = ScanFilter()
    rospy.spin()
