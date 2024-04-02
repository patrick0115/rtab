#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import open3d as o3d
import numpy as np
import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import datetime

latest_point_cloud = None
latest_image = None
bridge = CvBridge()


def extract_unique_colors(point_cloud_msg):
    # 確保點雲消息包含顏色信息
    if 'rgb' not in [field.name for field in point_cloud_msg.fields]:
        # rospy.logwarn("點雲不包含RGB顏色信息。")
        pass
        return []

    # 指定所需的欄位名稱
    field_names = ("x", "y", "z", "rgb")

    # 將 PointCloud2 消息轉換為數組
    incoming_cloud = list(pc2.read_points(point_cloud_msg, skip_nans=True, field_names=field_names))

    # 提取RGB顏色
    colors = np.array([point[3] for point in incoming_cloud])

    # 將RGB顏色從浮點數轉換為單獨的R, G, B值
    colors.dtype = np.uint32
    r = np.bitwise_and(colors, 0x00FF0000) >> 16
    g = np.bitwise_and(colors, 0x0000FF00) >> 8
    b = np.bitwise_and(colors, 0x000000FF)

    # 合並R, G, B值並歸一化
    colors = np.stack([r, g, b], axis=1) 

    # 獲取不重複的顏色
    unique_colors = np.unique(colors, axis=0)
    return unique_colors


def image_callback(msg):
    global latest_image
    # rospy.loginfo("接收到新的圖片消息！")
    latest_image = msg

def point_cloud_callback(msg):
    unique_colors = extract_unique_colors(msg)
    # print("不重複的顏色數量:", len(unique_colors))
    # print("不重複的顏色（RGB）:", unique_colors)
    global latest_point_cloud
    # rospy.loginfo("接收到新的點雲消息！")
    latest_point_cloud = msg

def handle_trigger(request):
    global latest_point_cloud, latest_image
    folder_path = rospy.get_param('~pcd_folder_path', '/home/lab606/rtab/src/calibration/raw_data/pcd')
    image_folder_path = rospy.get_param('~image_folder_path', '/home/lab606/rtab/src/calibration/raw_data/img')
    save_images = rospy.get_param('~save_images', True)  # 新增參數，預設為True

    # 獲取檔名前綴
    filename_prefix = rospy.get_param('~filename_prefix', 'data')

    # 使用前綴和當前時間構建檔案名稱
    current_time = datetime.datetime.now().strftime("%m%d-%H%M%S")
    file_name = f"{filename_prefix}_{current_time}"

    if latest_point_cloud is not None:
        output_file_path = os.path.join(folder_path, f"{file_name}.pcd")
        if save_point_cloud(latest_point_cloud, output_file_path):
            rospy.loginfo("點雲已保存: " + output_file_path)
        else:
            rospy.logwarn("保存點雲時發生錯誤。")

    if save_images and latest_image:
        image_file_path = os.path.join(image_folder_path, f"{file_name}.png")
        if save_image(latest_image, image_file_path):
            rospy.loginfo(f"圖片已保存到: {image_file_path}")
        else:
            rospy.logwarn("保存圖片時發生錯誤。")
    elif not save_images:
        rospy.loginfo("設定為不保存圖像。")
    else:
        rospy.logwarn("未接收到圖片消息，無法保存。")
    return EmptyResponse()


def save_point_cloud(point_cloud_msg, output_file_path):
    try:
        # 檢查點雲是否包含顏色信息
        field_names = [field.name for field in point_cloud_msg.fields]
        
        has_color = 'rgb' in field_names or 'rgba' in field_names
        # 讀取點雲數據
        cloud = pc2.read_points(point_cloud_msg, field_names=field_names, skip_nans=True)
        cloud_array = np.array(list(cloud))

        # 創建 open3d 點雲對象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud_array[:, :3])

        if has_color:
            # 如果有顏色信息，則處理並保存顏色
            field_names = ("x", "y", "z", "rgb" if 'rgb' in field_names else "rgba")
            cloud = pc2.read_points(point_cloud_msg, field_names=field_names, skip_nans=True)
            cloud_array = np.array(list(cloud))
            
            colors = cloud_array[:, 3]
            if 'rgb' in field_names:
                rospy.loginfo("顏色格式為rgb:")
                colors_uint32 = np.asarray(colors, dtype=np.uint32)
                r = (colors_uint32 >> 16) & 0x0000FF
                g = (colors_uint32 >> 8) & 0x0000FF
                b = colors_uint32 & 0x0000FF
                colors = np.stack([r, g, b], axis=-1) / 255.0
            else:  # RGBA
                colors = np.asarray(colors, dtype=np.float32).view(np.uint32)
                r = np.bitwise_and(colors, 0x00FF0000) >> 16
                g = np.bitwise_and(colors, 0x0000FF00) >> 8
                b = np.bitwise_and(colors, 0x000000FF)
                colors = np.stack([r, g, b], axis=-1) / 255.0

            pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            pass

        # 保存點雲
        o3d.io.write_point_cloud(output_file_path, pcd)
        rospy.loginfo("點雲已保存為PCD檔：%s" % output_file_path)
        return True
    except Exception as e:
        rospy.logerr("保存點雲時出錯：%s" % str(e))
        return False



def save_image(image_msg, output_file_path):
    try:
        # 確保收到的消息是圖片消息
        if not isinstance(image_msg, Image):
            rospy.logerr("嘗試保存的不是圖片消息。")
            return False

        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imwrite(output_file_path, cv_image)
        rospy.loginfo("圖片已保存為PNG檔：%s" % output_file_path)
        return True
    except CvBridgeError as e:
        rospy.logerr("保存圖片時出錯：%s" % str(e))
        return False
        
if __name__ == '__main__':
    rospy.init_node('point_cloud_and_image_node')
    point_cloud_topic = rospy.get_param('~point_cloud_topic', '/rtabmap/cloud_map')
    image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
    rospy.Subscriber(point_cloud_topic, PointCloud2, point_cloud_callback)
    rospy.Subscriber(image_topic, Image, image_callback)
    trigger_service = rospy.Service('save_data', Empty, handle_trigger)
    rospy.loginfo("正在訂閱點雲數據topic: %s 和圖像數據topic: %s，等待 rosservice call /save_data 被調用，存單一幀的影像和點雲數據", point_cloud_topic, image_topic)
    rospy.spin()