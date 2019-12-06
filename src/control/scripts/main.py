#!/usr/bin/env python2
#coding: utf-8

import rospy
from common.msg import HeadTask, BodyTask
from common.msg import PlayerInfo

if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
