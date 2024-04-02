#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
import tf
import std_msgs.msg
import struct
import message_filters  # 導入 message_filters

class PCSSNode:
    def __init__(self):
        # 初始化 ROS 節點
        rospy.init_node('pcss_node', anonymous=True)
        self.bridge = CvBridge()
        self.tf_listener = tf.TransformListener()

        self.perform_transform = rospy.get_param('~perform_transform', True)

        # 讀取攝影機校正和外部參數
        camera_cal_file = rospy.get_param('~camera_cal_file', '/home/lab606/rtab/src/calibration/script/cal_file/cam_cal/realsense_640_480/ost.txt')
        camera_ext = rospy.get_param('~camera_ext', '/home/lab606/rtab/src/calibration/script/cal_file/lidar_cam_cal/realsense_640_480/rvecs_tvecs.txt')
        self.rvecs, self.tvecs = self.load_rvecs_tvecs(camera_ext)
        self.camera_matrix, self.dist_coeffs = self.read_camera_params(camera_cal_file)

        # 使用 message_filters 創建訂閱者
        image_topic = rospy.get_param('~image_topic', '/segmented_image')
        
        # cloud_topic = rospy.get_param('~cloud_topic', '/rtabmap/odom_local_map')
        cloud_topic = rospy.get_param('~cloud_topic', '/rtabmap/cloud_map')
        image_sub = message_filters.Subscriber(image_topic, Image)
        cloud_sub = message_filters.Subscriber(cloud_topic, PointCloud2)

        # 使用時間同步器對影像和點雲進行同步
        ts = message_filters.ApproximateTimeSynchronizer([image_sub, cloud_sub], 10, 0.01)
        ts.registerCallback(self.sync_callback)

        self.colored_cloud_pub = rospy.Publisher('colored_cloud', PointCloud2, queue_size=10)
        self.image_pub = rospy.Publisher('projected_image', Image, queue_size=10)

        rospy.loginfo("PCSSNode initialized successfully")

    # 同步回調函數
    def sync_callback(self, image_msg, cloud_msg):
        # rospy.loginfo("Received synchronized image and cloud messages")

        # rospy.loginfo("---------------------")
        # cloud_timestamp = cloud_msg.header.stamp
        # rospy.loginfo("點雲時間戳記: %s", str(cloud_timestamp))
        # img_timestamp = image_msg.header.stamp
        # rospy.loginfo("影像時間戳記: %s", str(img_timestamp))
        # time_diff = img_timestamp - cloud_timestamp
        # rospy.loginfo("時間差: %s", str(time_diff.to_sec()) + " 秒")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_image_w_point = np.copy(cv_image)

            # 處理點雲數據
            cloud_points = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)

            if self.perform_transform:
                self.tf_listener.waitForTransform(image_msg.header.frame_id, cloud_msg.header.frame_id, rospy.Time(0), rospy.Duration(5.0))
                (trans, rot) = self.tf_listener.lookupTransform(image_msg.header.frame_id, cloud_msg.header.frame_id, rospy.Time(0))
                transformed_points = [tf.transformations.quaternion_matrix(rot).dot(np.array([point[0], point[1], point[2], 1]))[:3] + trans for point in cloud_points]
                transform_matrix = tf.transformations.quaternion_matrix(rot)
                transform_matrix[:3, 3] = trans
                inverse_transform_matrix = np.linalg.inv(transform_matrix)
            else:
                transformed_points = np.array(list(cloud_points))
            filtered_points = [point for point in transformed_points if point[2] > 0]
            if filtered_points:
                xyz_points = np.array(filtered_points)
                projected_points, _ = cv2.projectPoints(xyz_points.reshape(-1, 1, 3), self.rvecs, self.tvecs, self.camera_matrix, self.dist_coeffs)
                projected_points = np.squeeze(projected_points, axis=1)
         
   
            colored_cloud = []

            for i, point in enumerate(projected_points):
                x, y = int(point[0]), int(point[1])
                if 0 <= x < cv_image.shape[1] and 0 <= y < cv_image.shape[0]:     
                    color = cv_image[y, x]
                    r, g, b = color[2], color[1], color[0]
                    r, g, b = [np.uint8(x) for x in [r, g, b]]
                    rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0] 
                    colored_point = list(xyz_points[i]) + [rgba]
                    colored_cloud.append(colored_point)
                    cv2.circle(cv_image_w_point, (x, y), 3, (0, 255, 255), -1)
                    
            if colored_cloud:
          
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "map"
                fields = [
                    PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    PointField('rgba', 12, PointField.UINT32, 1)
                ]

                if self.perform_transform:
                    for i in range(len(colored_cloud)):    
                        xyz = np.array(colored_cloud[i][:3] + [1])
                        xyz = inverse_transform_matrix.dot(xyz)[:3]
                        colored_cloud[i][:3] = xyz.tolist()
                if not colored_cloud:
                    rospy.logwarn("彩色點雲為空，不進行發布")
                    return
                else:
                    pass

                cloud_msg = pc2.create_cloud(header, fields, colored_cloud)
                self.colored_cloud_pub.publish(cloud_msg)

                projected_image_msg = self.bridge.cv2_to_imgmsg(cv_image_w_point, "bgr8")
                projected_image_msg.header.stamp = cloud_msg.header.stamp
                projected_image_msg.header.frame_id = cloud_msg.header.frame_id
                self.image_pub.publish(projected_image_msg)
        except Exception as e:
            rospy.logerr("Error in sync_callback: %s", e)

    def read_camera_params(self,file_path):
        # 初始化空的列表來存儲camera matrix和distortion的值
        camera_matrix_list = []
        distortion_list = []

        # 打開和讀取txt檔
        with open(file_path, "r") as f:
            lines = f.readlines()

        # 解析每一行
        in_camera_matrix_section = False
        in_distortion_section = False

        for line in lines:
            line = line.strip()  # 去掉首尾的空白字符

            # 判斷是否進入特定的section
            if line == "camera matrix":
                in_camera_matrix_section = True
                continue
            elif line == "distortion":
                in_distortion_section = True
                continue

            # 讀取數據並加入到對應的列表中
            if in_camera_matrix_section and line:
                row = list(map(float, line.split()))
                camera_matrix_list.append(row)

                # 如果camera matrix已經有3行了，那就退出這個section
                if len(camera_matrix_list) == 3:
                    in_camera_matrix_section = False

            if in_distortion_section and line:
                distortion_list = list(map(float, line.split()))

                # 退出這個section
                in_distortion_section = False

        # 將列表轉換為numpy.ndarray
        camera_matrix_np = np.array(camera_matrix_list)
        distortion_np = np.array([distortion_list])

        print("讀取完畢")
        return camera_matrix_np, distortion_np


    def load_rvecs_tvecs(self, filepath):
        data = np.loadtxt(filepath)
        rvecs = data[:3].reshape(-1, 1)
        tvecs = data[3:].reshape(-1, 1)
        return rvecs, tvecs

if __name__ == '__main__':
    try:
        node = PCSSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
