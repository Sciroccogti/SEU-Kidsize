#!/usr/bin/env python2
#coding: utf-8

import rospy
import math
from common.msg import HeadTask, BodyTask
from common.msg import PlayerInfo, ImageResult

def ImageResUpdate(res):
    pass

if __name__ == '__main__':
    rospy.init_node('strategy', anonymous=True)
    bodyTaskPublisher = rospy.Publisher("/task/body", BodyTask, queue_size=1)
    headTaskPublisher = rospy.Publisher("/task/head", HeadTask, queue_size=1)
    imgResSubscriber = rospy.Subscriber('/result/vision/imgproc', ImageResult, ImageResUpdate)
    rate = rospy.Rate(20)
    i = 0.0
    while not rospy.is_shutdown():
        htask = HeadTask()
        htask.pitch = -45.0
        htask.yaw = math.sin(i)*100.0
        headTaskPublisher.publish(htask)
        i = i+0.04
        btask = BodyTask()
        btask.type = BodyTask.TASK_WALK
        btask.step = 0.02
        bodyTaskPublisher.publish(btask)
        rate.sleep()
