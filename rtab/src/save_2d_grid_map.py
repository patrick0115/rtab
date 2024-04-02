#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty, EmptyResponse  # 使用标准空服务
import numpy as np
import cv2
import os

map_data = None

def map_callback(data):
    global map_data
    map_data = data
    # rospy.loginfo("Map data received.")

def handle_save_map(req):
    global map_data
    if map_data is None:
        rospy.logwarn("No map data available to save.")
        return EmptyResponse()
    
    try:
        map_path = rospy.get_param('~map_path', '/home/lab606/rtab/src/rtab/map/map.pgm')  # 从参数服务器读取map_path参数
        if not map_path.endswith('.pgm'):
            map_path += '.pgm'

        # 确保目录存在
        directory = os.path.dirname(map_path)
        if not os.path.exists(directory):
            os.makedirs(directory)

        # 将ROS的地图消息转换为OpenCV图像
        width, height = map_data.info.width, map_data.info.height
        map_img = np.array(map_data.data).reshape((height, width))
        map_img = np.clip(map_img, 0, 100)  # 限制值在0到100之间
        map_img = (100 - map_img) * 2.55  # 0表示占用，100表示空闲
        map_img = map_img.astype(np.uint8)
        map_img = cv2.flip(map_img, 0)
        # 保存图像
        cv2.imwrite(map_path, map_img)
        rospy.loginfo("Map image saved as %s", map_path)

        # 保存YAML文件
        yaml_path = map_path.replace('.pgm', '.yaml')
        with open(yaml_path, "w") as yaml_file:
            yaml_file.write("image: {}\n".format(os.path.basename(map_path)))
            yaml_file.write("resolution: {:.3f}\n".format(map_data.info.resolution))
            yaml_file.write("origin: [{:.3f}, {:.3f}, {:.3f}]\n".format(map_data.info.origin.position.x, map_data.info.origin.position.y, 0))
            yaml_file.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        rospy.loginfo("Map metadata saved as %s", yaml_path)

        return EmptyResponse()

        return EmptyResponse()
    except Exception as e:
        rospy.logerr("Failed to save map: %s", str(e))
        return EmptyResponse()

def save_map_server():
    rospy.init_node('save_2d_grid_map')
    
    topic_name = rospy.get_param('~occupancy_grid_topic', '/converted_occupancy_grid')
    rospy.Subscriber(topic_name, OccupancyGrid, map_callback)
    s = rospy.Service('save_grid_map', Empty, handle_save_map)  
    rospy.loginfo("Ready to save 2D grid map. Waiting for service call /save_grid_map service call.")
    rospy.spin()

if __name__ == '__main__':
    save_map_server()
