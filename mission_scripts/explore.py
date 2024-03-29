#!/usr/bin/env python

import rospy, time
from rospy import timer
from std_msgs.msg import String
import std_msgs.msg
from nav_msgs.msg import Path
from math import pi, sin, cos, atan2, sqrt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, Pose, PointStamped, Point
from geometry_msgs.msg import PoseStamped
from exploration.msg import Frontier
import time
import subprocess
import tf 
from tf.transformations import quaternion_from_euler
import math

WP_SIZE = 0.5

return_flag = 0
m = 0


class Server:

    def __init__(self, current_time=None):

        global mission_timer, mission_duration, start
        mission_timer = time.time()        
        self.waypoint = False
        self.frontier_received = False
        self.xbee_return_flag = False
        self.local_heading_ref = 0
        self.heading_ref = 0
        self.x_c = 0
        self.y_c = 0
        self.z_c = 0
        self.x_r = 0
        self.y_r = 0
        self.z_r = 0
        self.d = 0

        #############################################################################################

        mission_duration = 10000

        #############################################################################################
        
        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        
        self.dsp_publisher = rospy.Publisher('/dsp/set_goal', Point, queue_size= 1)
        self.dsp_msg = Point() 

        self.vel_publisher = rospy.Publisher('/hummingbird/command/pose', PoseStamped, queue_size = 1)
        self.vel_msg = PoseStamped()

        self.safety_publisher = rospy.Publisher('/hummingbird/safety_land', String, queue_size= 10)
        self.safety_msg = String()

        self.frontier_subscriber = rospy.Subscriber('/frontier/point', Point, self.frontier_callback)
        
        rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, self.read_callback)

        rospy.Subscriber('/hummingbird/return',  String, self.return_to_base)

        self.waypoint_subscriber = rospy.Subscriber('/dsp/path', Path, self.waypoint_callback)

        self.f_x = 0
        self.f_y = 0
        self.f_z = 0

    def frontier_callback(self, msg):

        self.f_x = msg.x
        self.f_y = msg.y
        self.f_z = msg.z
        self.frontier_received = True

    def return_to_base(self, msg):

        self.xbee_return_flag = True

    def waypoint_callback(self, msg):

        self.wp_index = 0
        self.path = msg
        self.x_r = msg.poses[self.wp_index].pose.position.x
        self.y_r = msg.poses[self.wp_index].pose.position.y
        self.z_r = msg.poses[self.wp_index].pose.position.z
        self.waypoint = True

    def explore(self, current_time=None):
 
        while (sqrt((self.x_c - self.x_r) * (self.x_c - self.x_r)  + (self.y_c - self.y_r) * (self.y_c - self.y_r) + (self.z_c - self.z_r) * (self.z_c - self.z_r)) < WP_SIZE):
            if(self.wp_index < len(self.path.poses) - 2):
                self.wp_index = self.wp_index + 1
            else:
                break
            self.x_r = self.path.poses[self.wp_index].pose.position.x
            self.y_r = self.path.poses[self.wp_index].pose.position.y
            self.z_r = self.path.poses[self.wp_index].pose.position.z

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'

        self.heading_ref = math.atan2((self.y_r - self.y_c), (self.x_r - self.x_c))
        
        self.vel_msg.pose.position.x = self.x_r
        self.vel_msg.pose.position.y = self.y_r
        self.vel_msg.pose.position.z = self.z_r
        self.vel_msg.pose.orientation.x = 0
        self.vel_msg.pose.orientation.y = 0 
        self.vel_msg.pose.orientation.z = quaternion_from_euler(0, 0, self.heading_ref)[2]
        self.vel_msg.pose.orientation.w = quaternion_from_euler(0, 0, self.heading_ref)[3]

        self.vel_publisher.publish(self.vel_msg)
        print('exploring....')

        return

    def read_callback(self, msg):
        
        self.x_c = msg.pose.pose.position.x
        self.y_c = msg.pose.pose.position.y
        self.z_c = msg.pose.pose.position.z

        start = time.time()

        self.d = ((self.x_c)*(self.x_c) + (self.y_c)*(self.y_c) + (self.z_c)*(self.z_c)) 


        if ((start - mission_timer) < mission_duration):

            print(start - mission_timer)

            if (self.xbee_return_flag == False):

                if (self.frontier_received):
                    self.dsp_msg.x = self.f_x
                    self.dsp_msg.y = self.f_y
                    self.dsp_msg.z = self.f_z
                    self.dsp_publisher.publish(self.dsp_msg)
                
            else:
                self.frontier_received = False
                self.dsp_msg.x = 0
                self.dsp_msg.y = 0
                self.dsp_msg.z = 1
                self.dsp_publisher.publish(self.dsp_msg)
                print("Xbee : returning to base....")

        else:
            self.return_flag = True                                
            self.dsp_msg.x = 0
            self.dsp_msg.y = 0
            self.dsp_msg.z = 1
            self.dsp_publisher.publish(self.dsp_msg)
            print("autonomous returning to base....")
        
        if ((start - mission_timer) > mission_duration) and (self.d < 1.5):
            
            self.safety_publisher.publish(self.safety_msg)
            print("safety landing....")

        if (self.waypoint):
            self.explore()

        end = time.time()

        return


if __name__ == "__main__":
    try:
        rospy.init_node('global exploration', anonymous=True)
        rospy.loginfo("Starting...")

        server = Server()

        while not rospy.is_shutdown():
            rospy.spin()

        rospy.loginfo("Reading finished...")
    except rospy.ROSInterruptException:
        pass
