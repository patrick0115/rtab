#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import struct
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

class PointCloudMerger:
    def __init__(self):
        rospy.init_node('point_cloud_merger', anonymous=True)
        self.pub = rospy.Publisher('merged_cloud', PointCloud2, queue_size=10)
        self.merged_cloud = []
        rospy.Subscriber('colored_cloud', PointCloud2, self.callback)

    def callback(self, data):
        # 檢查點雲數據中是否包含RGB字段
        fields = data.fields
        has_rgb = any(f.name == 'rgb' for f in fields)
        if not has_rgb:
            rospy.logwarn("点云数据不包含RGB信息")
            return

        # 根据是否有RGB信息读取点云数据
        field_names = ("x", "y", "z", "rgb") if has_rgb else ("x", "y", "z")
        incoming_cloud = list(pc2.read_points(data, skip_nans=True, field_names=field_names))

        for point in incoming_cloud:
            if len(point) == 4:  # 如果点包含RGB信息
                x, y, z, rgb = point
            else:
                x, y, z = point
                rgb = 0  # 如果没有RGB信息，则设置为0

            self.merged_cloud.append((x, y, z, rgb))

        # 创建点云消息并发布
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
        if has_rgb:
            fields.append(PointField('rgb', 12, PointField.UINT32, 1))
        
        header = data.header
        merged_cloud_msg = pc2.create_cloud(header, fields, self.merged_cloud)
        self.pub.publish(merged_cloud_msg)

    # def convert_rgba_float_to_int(self, rgba_float):
    #     # 提取 RGBA 浮點數的每個組件
    #     b = int((rgba_float) % 256)
    #     g = int((rgba_float / 256) % 256)
    #     r = int((rgba_float / (256*256)) % 256)
    #     a = int((rgba_float / (256*256*256)) % 256)
        
    #     # 重新打包為整數
    #     rgba_int = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
    #     return rgba_int

if __name__ == '__main__':
    try:
        merger = PointCloudMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
