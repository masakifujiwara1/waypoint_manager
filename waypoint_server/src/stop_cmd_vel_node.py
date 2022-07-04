#!/usr/bin/env python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Twist
import rospy


class stop_cmd_vel_node():
    def __init__(self):
        rospy.init_node('stop_cmd_vel_node', anonymous=True)
        self.vel_pub = rospy.Publisher("stop_vel", Twist, queue_size=10)
        self.vel = Twist()

    def loop(self):
        self.vel.linear.x = 0.0
        self.vel_pub.publish(self.vel)


if __name__ == '__main__':
    rospy.loginfo('stop_cmd_vel_node started')
    rg = stop_cmd_vel_node()
    DURATION = 1
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        rg.loop()
        r.sleep()
