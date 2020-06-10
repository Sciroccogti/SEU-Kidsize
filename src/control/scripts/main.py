#!/usr/bin/env python2
#coding: utf-8

import rospy
import math
from common.msg import HeadTask, BodyTask, LedTask
from common.msg import PlayerInfo, ImageResult
from common.msg import HeadAngles, ImuData, GcInfo

imgResult = ImageResult()
headAngles = HeadAngles()
imuData = ImuData()
gcData = GcInfo()

def ImageResUpdate(msg):
    global imgResult
    imgResult = msg

def HeadUpdate(msg):
    global headAngles
    headAngles = msg

def ImuUpdate(msg):
    global imuData
    imuData = msg

def GcUpdate(msg):
    global gcData
    gcData = msg

searchBallTable = [(-90, 0), (-45, 0), (0, 0), (45, 0), (90, 0),
    (90, 30), (45, 30), (0, 30), (-45, 30), (-90, 30),
    (-70, 55), (-35, 55), (0, 55), (35, 55), (70, 55)]
searchPostTable = [(-90, 0), (-45, 0), (0, 0), (45, 0), (90, 0),
    (90, 30), (45, 30), (0, 30), (-45, 30), (-90, 30)]

if __name__ == '__main__':
    rospy.init_node('strategy', anonymous=True)
    bodyTaskPublisher = rospy.Publisher("/task/body", BodyTask, queue_size=1)
    headTaskPublisher = rospy.Publisher("/task/head", HeadTask, queue_size=1)
    ledTaskPublisher = rospy.Publisher("/task/led", LedTask, queue_size=1)
    imgResSubscriber = rospy.Subscriber('/result/vision/imgproc', ImageResult, ImageResUpdate)
    headSubscriber = rospy.Subscriber('/sensor/head', HeadAngles, HeadUpdate)
    imuSubscriber = rospy.Subscriber('/sensor/imu', ImuData, ImuUpdate)
    gcSubscriber = rospy.Subscriber('/sensor/gctrl', GcInfo, GcUpdate)
    rate = rospy.Rate(20)
    lstatus = True
    i = 0
    j = 0
    while not rospy.is_shutdown():
        htask = HeadTask()
        btask = BodyTask()
        btask.type = BodyTask.TASK_WALK
        btask.count = 2
        btask.step = 0.0
        if imgResult.has_ball:
            x = imgResult.ball.x
            y = imgResult.ball.y
            htask.yaw = searchBallTable[i%len(searchBallTable)][0]
            htask.pitch = searchBallTable[i%len(searchBallTable)][1]
        else:
            if j%15 == 0:
                i = i+1
            htask.yaw = searchBallTable[i%len(searchBallTable)][0]
            htask.pitch = searchBallTable[i%len(searchBallTable)][1]
            j = j+1
            
        bodyTaskPublisher.publish(btask) 
        headTaskPublisher.publish(htask)           
        rate.sleep()
