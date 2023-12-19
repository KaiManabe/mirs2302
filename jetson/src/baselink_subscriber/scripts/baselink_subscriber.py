#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf
import tf2_geometry_msgs
import threading
import time
from geometry_msgs.msg import Twist
import datetime


class velocity_controller():
    def __init__(self):
        self.p = 0.2
        self.i = 0.0
        self.d = 0.0
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        #ノードの初期化
        rospy.init_node("baselink_subscriber")
        
        #base_linkの絶対座標を取得するやつ　の　初期化
        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)
        
        #cmd_velをpubするやつ　の　初期化
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        
        
        t = threading.Thread(target = self.get_robot_pose)
        t.setDaemon(True)
        t.start()

    
    
    def stop(self):
        self.v_x = 0.0
        self.v_theta = 0.0
    

    def get_robot_pose(self):
        while not rospy.is_shutdown():
            robot_pose = tf2_geometry_msgs.PoseStamped()
            robot_pose.header.frame_id = "base_link"
            robot_pose.header.stamp = rospy.Time(0)

            robot_pose.pose.orientation.w = 1.0

            try:
                global_pose = self.tf_buffer.transform(robot_pose, "map")
            except:
                print("[WARN][baselink_subscriber.py] : mapからbase_linkのtfが見つかりませんでした。continueします")
                rospy.sleep(0.5)
                continue
            
            euler_rotation = \
            tf.transformations.euler_from_quaternion((
                global_pose.pose.orientation.x,
                global_pose.pose.orientation.y,
                global_pose.pose.orientation.z,
                global_pose.pose.orientation.w))
            
            self.robot_x = global_pose.pose.position.x
            self.robot_y = global_pose.pose.position.y
            self.robot_theta = euler_rotation[2]
            
            rospy.sleep(0.05)


if __name__ == "__main__":
        vc = velocity_controller()
        
        
    