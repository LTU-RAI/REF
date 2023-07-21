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

import math

WP_SIZE = 0.5

return_flag = 0
m = 0


class Server:


    
    def __init__(self, current_time=None):

        global mission_timer, mission_duration, start
        mission_timer = time.time()
        
        self.globalfrontier = False
        self.localfrontier = False
        self.waypoint = False
        self.global_re_pos = False
        self.local_exploration = False
        self.return_flag = False
        self.local_frontier = False
        self.local_heading_ref = 0
        self.global_heading_ref = 0
        self.cmd_x = 0
        self.cmd_y = 0
        self.cmd_z = 1
        self.x_c = 0
        self.y_c = 0
        self.z_c = 0
        self.x_r = 0
        self.y_r = 0
        self.z_r = 0
        self.d = 0

        #############################################################################################

        mission_duration = 100000

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

        self.cmd_subscriber = rospy.Subscriber('/cmd/reference', PoseStamped, self.cmd_callback)

        self.frontier_subscriber = rospy.Subscriber('/frontier/pose', PoseStamped, self.global_frontier_callback)
        
        self.local_frontier_subscriber = rospy.Subscriber('/cmd_frontier', Frontier, self.local_frontier_callback)

        rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, self.read_callback)

        rospy.Subscriber('/hummingbird/return',  String, self.return_to_base)

        self.waypoint_subscriber = rospy.Subscriber('/dsp/path', Path, self.waypoint_callback)

        self.f_x = 0
        self.f_y = 0
        self.f_z = 0

    def global_frontier_callback(self, msg):

        self.f_x = msg.pose.position.x
        self.f_y = msg.pose.position.y
        self.f_z = msg.pose.position.z
        self.global_re_pos = True
        self.local_frontier = False

    def local_frontier_callback(self, msg):

        self.f_x = msg.point.x
        self.f_y = msg.point.y
        self.f_z = msg.point.z
        self.local_frontier = True
        self.global_re_pos = False


    def cmd_callback(self, msg):

        self.cmd_x = msg.pose.position.x
        self.cmd_y = msg.pose.position.y
        self.cmd_z = msg.pose.position.z
        self.local_exploration = True

    def return_to_base(self, msg):

        self.return_flag = True
        self.local_exploration = False
        self.global_re_pos = False

    def waypoint_callback(self, msg):

        self.wp_index = 0
        self.path = msg
        self.x_r = msg.poses[self.wp_index].pose.position.x
        self.y_r = msg.poses[self.wp_index].pose.position.y
        self.z_r = msg.poses[self.wp_index].pose.position.z
        self.waypoint = True

    def explore(self, current_time=None):
 
        if (self.local_frontier) and (self.global_re_pos == False) and (self.return_flag == False):

            self.local_heading_ref = math.atan2((self.cmd_y - self.y_c), (self.cmd_x - self.x_c))
            self.vel_msg.pose.position.x = self.cmd_x
            self.vel_msg.pose.position.y = self.cmd_y
            self.vel_msg.pose.position.z = self.cmd_z
            self.vel_msg.pose.orientation.x = 0
            self.vel_msg.pose.orientation.y = 0
            self.vel_msg.pose.orientation.z = self.local_heading_ref
            self.vel_msg.pose.orientation.w = 0
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now
            header.frame_id = 'world'
            self.vel_publisher.publish(self.vel_msg)
            print('local_ref')


        if (self.global_re_pos) or (self.return_flag) or (self.local_frontier == False):

            while (sqrt((self.x_c - self.x_r) * (self.x_c - self.x_r)  + (self.y_c - self.y_r) * (self.y_c - self.y_r) + (self.z_c - self.z_r) * (self.z_c - self.z_r)) < WP_SIZE):
                if(self.wp_index < len(self.path.poses) - 1):
                    self.wp_index = self.wp_index + 1
                else:
                    break
                self.x_r = self.path.poses[self.wp_index].pose.position.x
                self.y_r = self.path.poses[self.wp_index].pose.position.y
                self.z_r = self.path.poses[self.wp_index].pose.position.z

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'

            self.global_heading_ref = math.atan2((self.y_r - self.y_c), (self.x_r - self.x_c))

            self.vel_msg.pose.position.x = self.x_r
            self.vel_msg.pose.position.y = self.y_r
            self.vel_msg.pose.position.z = self.z_r
            self.vel_msg.pose.orientation.x = 0
            self.vel_msg.pose.orientation.y = 0
            self.vel_msg.pose.orientation.z = self.global_heading_ref
            self.vel_msg.pose.orientation.w = 0

            self.vel_publisher.publish(self.vel_msg)
            print('global_ref')
        else:
            if (self.local_frontier) and (self.return_flag == False):
                self.local_heading_ref = math.atan2((self.cmd_y - self.y_c), (self.cmd_x - self.x_c))
                self.vel_msg.pose.position.x = self.cmd_x
                self.vel_msg.pose.position.y = self.cmd_y
                self.vel_msg.pose.position.z = self.cmd_z
                self.vel_msg.pose.orientation.x = 0
                self.vel_msg.pose.orientation.y = 0
                self.vel_msg.pose.orientation.z = self.local_heading_ref
                self.vel_msg.pose.orientation.w = 0
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now
                header.frame_id = 'world'
                self.vel_publisher.publish(self.vel_msg)
                print('local_ref')

        return

    def read_callback(self, msg):
        
        self.x_c = msg.pose.pose.position.x
        self.y_c = msg.pose.pose.position.y
        self.z_c = msg.pose.pose.position.z

        start = time.time()

        self.d = ((self.x_c)*(self.x_c) + (self.y_c)*(self.y_c) + (self.z_c)*(self.z_c)) 


        if ((start - mission_timer) < mission_duration):

            print(start - mission_timer)

            if (self.global_re_pos):
                self.dsp_msg.x = self.f_x
                self.dsp_msg.y = self.f_y
                self.dsp_msg.z = self.f_z
                self.dsp_publisher.publish(self.dsp_msg)

            if (self.return_flag):
                self.dsp_msg.x = 0
                self.dsp_msg.y = 0
                self.dsp_msg.z = 1
                self.dsp_publisher.publish(self.dsp_msg)
                print("Xbee : returning to base....")

        else:
            self.return_flag = True                                
            self.dsp_msg.x = 0
            self.dsp_msg.y = 0
            self.dsp_msg.z = 0.8
            self.dsp_publisher.publish(self.dsp_msg)
            print("returning to base....")
        
        if (start - mission_timer) > mission_duration and (self.d < 1.5):
            
            self.safety_publisher.publish(self.safety_msg)
            print("safety landing....")

        self.explore()

        end = time.time()

        return


if __name__ == "__main__":
    try:
        rospy.init_node('explore', anonymous=True)
        rospy.loginfo("Starting...")

        server = Server()

        while not rospy.is_shutdown():
            rospy.spin()

        rospy.loginfo("Reading finished...")
    except rospy.ROSInterruptException:
        pass
