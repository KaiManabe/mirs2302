#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose():
    rospy.init_node('initial_pose_publisher', anonymous=True)
    initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
    rate = rospy.Rate(1)

    initial_pose_msg = PoseWithCovarianceStamped()
    initial_pose_msg.header.stamp = rospy.Time.now()
    initial_pose_msg.header.frame_id = 'map'
    initial_pose_msg.pose.pose.position.x = 0.0
    initial_pose_msg.pose.pose.position.y = 0.0
    initial_pose_msg.pose.pose.position.z = 0.0
    initial_pose_msg.pose.pose.orientation.x = 0.0
    initial_pose_msg.pose.pose.orientation.y = 0.0
    initial_pose_msg.pose.pose.orientation.z = 0.0
    initial_pose_msg.pose.pose.orientation.w = 1.0

    while not rospy.is_shutdown():
        initial_pose_pub.publish(initial_pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass
