#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospkg
import sys
rospack = rospkg.RosPack()
package_path = rospack.get_path('seg')
sys.path.append(package_path)
import torch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from torchvision import transforms
from model.net.bisenetv2 import BiSeNetV2
from tools.palette import get_palette
import math
import numpy as np


class ImageSegmentationNode:
    def __init__(self):
        rospy.init_node('image_segmentation_node', anonymous=True)
        self.input_topic = rospy.get_param('input_topic', '/camera_topic')
        self.output_topic = rospy.get_param('output_topic', 'segmented_image_topic')
        self.model_path = rospy.get_param('model_path', '/path/to/your/model.pth')
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.output_class = 11
        self.net = self.load_model(self.model_path, self.device , self.output_class)
        if not self.net:
            rospy.logerr("無法加載模型，節點將關閉。")
            sys.exit(1)
        self.transform = self.get_transform()
        self.bridge = CvBridge()
        self.palette = get_palette()[0]
        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=10)
  
        rospy.Subscriber(self.input_topic, Image, self.callback)


    def load_model(self, model_path, device='cpu',output_class=11):
        rospy.loginfo("加載模型中...")
        net = BiSeNetV2(150, output_class = output_class)
        net.aux_mode = 'pred'
        try:
            net.load_state_dict(torch.load(model_path, map_location=device))
            net.eval()
            net.to(device)
            rospy.loginfo("模型加載完成。")
            return net
        except Exception as e:
            rospy.logerr(f"加載模型時出錯: {e}")
            return None

    def get_transform(self):
        mean = [0.49343230, 0.46819794, 0.43106043]
        std = [0.25680755, 0.25506608, 0.27422913]
        return transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean, std)
        ])

    def preprocess_image(self, cv_image):
        try:
            im = self.transform(cv_image).unsqueeze(0).to(self.device)
            return im
        except Exception as e:
            rospy.logerr(f"圖像預處理過程中出錯: {e}")
            return None

    def segment_image(self, im):
        if self.output_class==10:
            seg_class =  [0, 3, 5, 8, 12, 14, 43, 82, 100, 144 ]
        if self.output_class==11:
            seg_class =  [0, 3, 5, 8, 12, 14, 43,59, 82, 100, 144 ]
        try:
            out = self.net(im)[0]
            adjustment_value = 1
            out[:, 1, :, :] += adjustment_value
            adjustment_value = 3
            out[:, 7, :, :] += adjustment_value
            # adjustment_value = 6
            # out[:, 0, :, :] += adjustment_value
            out = torch.tensor(seg_class).to(out.device)[torch.argmax(out, dim=1)]
            out = out.squeeze().cpu().numpy()

            out = self.palette[out]
            return out
        except Exception as e:
            rospy.logerr(f"圖像分割過程中出錯: {e}")
            return None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            im = self.preprocess_image(cv_image)
            if im is not None:
                segmented_img = self.segment_image(im)
                if segmented_img is not None:
                    segmented_image_msg = self.bridge.cv2_to_imgmsg(segmented_img.astype(np.uint8), encoding="bgr8")
                    segmented_image_msg.header = data.header
                    self.pub.publish(segmented_image_msg)
        except Exception as e:
            rospy.logerr(f"處理回調中的圖像時出錯: {e}")


if __name__ == '__main__':
    node = ImageSegmentationNode()
    rospy.spin()
