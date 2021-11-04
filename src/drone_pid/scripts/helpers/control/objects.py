#! /usr/bin/env python

import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

class Control:
    def __init__(self):
        self.goal = 0.0  # [angle]
        self.ctrl_c = False
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._move_msg = Twist()
        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self._takeoff_msg = Empty()
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self._land_msg = Empty()

    def stop(self):
        rospy.loginfo("Stopping...")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 0.0
        self._pub_cmd_vel.publish(self._move_msg)

    def takeoff(self):
        rospy.loginfo('Taking off...')
        i=0
        while not i == 3:
            self._pub_takeoff.publish(self._takeoff_msg)
            time.sleep(1)
            i += 1
    
    def land(self):
        rospy.loginfo('Landing...')
        i=0
        while not i == 3:
            self._pub_land.publish(self._land_msg)
            time.sleep(1)
            i += 1

    def turn(self):
        rospy.loginfo("Turning...")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 1.0
        self._pub_cmd_vel.publish(self._move_msg)

    def move_forward(self):
        rospy.loginfo("Moving forward...")
        self._move_msg.linear.x = 1.0
        self._move_msg.angular.z = 0.0
        self._pub_cmd_vel.publish(self._move_msg)