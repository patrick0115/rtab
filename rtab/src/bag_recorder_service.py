#!/usr/bin/env python3
import rospy
import subprocess
import os
import signal
from std_srvs.srv import Empty, EmptyResponse

class BagRecorder:
    def __init__(self):
        # 設定bag檔儲存位置
        self.bag_file_directory = rospy.get_param('~bag_file_directory', '/path/to/save/bags/')
        self.record_process = None
        self.bag_file_path = os.path.join(self.bag_file_directory, 'bag.bag')

        self.topics_to_record = rospy.get_param('~topics_to_record', '/topic1 /topic2')

        # 創建service
        rospy.Service('record_bag', Empty, self.handle_record_bag)

    def handle_record_bag(self, req):
        if self.record_process is None:

            exclude_pattern = ".*(theora|compressed|compressedDepth).*"
            
            # bag_file_path = os.path.join(self.bag_file_directory, 'bag.bag')



            # 判斷是否錄製所有topics
            if self.topics_to_record == "all":
                # 錄製所有topics，但排除特定關鍵字的topics
                self.record_process = subprocess.Popen(['rosbag', 'record', '-a', '-o', self.bag_file_path, '-x', exclude_pattern])
            else:
                # 將要錄製的topics轉換成列表
                topics_list = self.topics_to_record.split(' ')
                self.record_process = subprocess.Popen(['rosbag', 'record', '-o', self.bag_file_path] + topics_list)

            rospy.loginfo("Started recording bag file.")

        else:
            # 停止錄製
            self.record_process.send_signal(subprocess.signal.SIGINT)
            self.record_process = None
            rospy.loginfo(f"Bag檔案儲存於: {self.bag_file_path}")

        return EmptyResponse()

if __name__ == '__main__':    
    
    rospy.init_node('bag_recorder_service')
    br = BagRecorder()
    rospy.loginfo("此服務允許您透過 'rosservice call /record_bag' 命令來控制ROS bag的錄製。")
    rospy.loginfo("執行此命令將開始錄製指定的topics，再次執行將停止錄製並保存bag檔案。")
    rospy.spin()
