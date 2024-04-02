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


def load_model(model_path, device='cpu'):
    rospy.loginfo("加載模型中...")
    net = BiSeNetV2(150,output_class=10)
    net.aux_mode = 'pred'
    try:
        net.load_state_dict(torch.load(model_path, map_location=device))
        net.eval()
        net.to(device)
        rospy.loginfo("模型加載完成。")
        return net
    except Exception as e:
        rospy.logerr("加載模型時出錯: {}".format(e))
        return None


def get_transform():
    rospy.loginfo("配置圖像轉換。")
    mean = [0.49343230, 0.46819794, 0.43106043]
    std = [0.25680755, 0.25506608, 0.27422913]
    return transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(mean, std)
    ])
def preprocess_image(cv_image, transform, device):
    # rospy.loginfo("圖像預處理中...")
    try:
        im = transform(cv_image).unsqueeze(0).to(device)
        org_size = im.size()[2:]
        new_size = [math.ceil(el / 32) * 32 for el in org_size]
        im = torch.nn.functional.interpolate(im, size=new_size, mode='bilinear', align_corners=False)
        # rospy.loginfo("圖像預處理完成。")
        return im, org_size
    except Exception as e:
        rospy.logerr("圖像預處理過程中出錯: {}".format(e))
        return None, None



def segment_image(net, im, org_size, palette):
    # rospy.loginfo("執行圖像分割...")
    try:
    
        out = net(im)[0]
        out = torch.nn.functional.interpolate(out, size=org_size, mode='bilinear', align_corners=False)
        out = out.argmax(dim=1).squeeze().detach().cpu().numpy()
        # rospy.loginfo(f"out.shape: {out.shape}")
        
        out = palette[out]
        # rospy.loginfo(f"out.shape: {out.shape}")
        
        return out
    except Exception as e:
        rospy.logerr("圖像分割過程中出錯: {}".format(e))
        return None


def callback(data, args):
    # rospy.loginfo("開始處理新圖像。")
    bridge, pub, palette, transform, net, device = args
    
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        im, org_size = preprocess_image(cv_image, transform, device)
        if im is not None and org_size is not None:
            segmented_img = segment_image(net, im, org_size, palette)
            if segmented_img is not None:
                segmented_image_msg = bridge.cv2_to_imgmsg(segmented_img.astype(np.uint8), encoding="bgr8")
                segmented_image_msg.header = data.header
                pub.publish(segmented_image_msg)
                # rospy.loginfo("圖像處理和發布完成。")
    except Exception as e:
        rospy.logerr("處理回調中的圖像時出錯: {}".format(e))

if __name__ == '__main__':
    rospy.init_node('image_segmentation_node')

    input_topic = rospy.get_param('input_topic', '/camera_topic')
    output_topic = rospy.get_param('output_topic', 'segmented_image_topic')
    model_path = rospy.get_param('model_path', '/path/to/your/model.pth') 

    bridge = CvBridge()
    palette = get_palette()[0]

    device = 'cuda'  # 或其他您想要使用的設備
    net = load_model(model_path, device=device)
    transform = get_transform()

    rospy.loginfo(f"輸入Topic: {input_topic}")
    rospy.loginfo(f"輸出Topic: {output_topic}")
    rospy.loginfo(f"模型路徑: {model_path}")
    rospy.loginfo(f"設備: {device}")

    pub = rospy.Publisher(output_topic, Image, queue_size=1)
    callback_args = (bridge, pub, palette, transform, net, device)  # 新增了 device 參數
    rospy.Subscriber(input_topic, Image, callback, callback_args)

    rospy.loginfo("ROS節點初始化完成，開始運行。")
    rospy.spin()