#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

def handle_odom(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

def odom_listener():
    rospy.init_node('odom_to_tf_broadcaster')
    rospy.Subscriber('/odom_robot', Odometry, handle_odom)
    rospy.spin()

if __name__ == '__main__':
    odom_listener()
