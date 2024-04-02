#!/usr/bin/env python
# coding: utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.callback)

    def callback(self, data):
        try:
            # 將 ROS 的影像消息轉換為 OpenCV 的影像格式
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        # 儲存影像為 PNG 格式
        cv2.imwrite('/home/lab606/epth_image.png', cv_image)
        print("影像已儲存為 depth_image.png")

if __name__ == '__main__':
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
