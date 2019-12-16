#!/usr/bin/env python2
#coding: utf-8

import rospy
import math
from common.msg import HeadTask, BodyTask, LedTask
from common.msg import PlayerInfo, ImageResult
from common.msg import HeadAngles, ImuData

imgResult = ImageResult()
headAngles = HeadAngles()

def ImageResUpdate(msg):
    global imgResult
    imgResult = msg

def HeadUpdate(msg):
    global headAngles
    headAngles = msg

if __name__ == '__main__':
    rospy.init_node('strategy', anonymous=True)
    bodyTaskPublisher = rospy.Publisher("/task/body", BodyTask, queue_size=1)
    headTaskPublisher = rospy.Publisher("/task/head", HeadTask, queue_size=1)
    ledTaskPublisher = rospy.Publisher("/task/led", LedTask, queue_size=1)
    imgResSubscriber = rospy.Subscriber('/result/vision/imgproc', ImageResult, ImageResUpdate)
    headSubscriber = rospy.Subscriber('/sensor/head', HeadAngles, HeadUpdate)
    rate = rospy.Rate(20)
    lstatus = True
    i = 0.0
    while not rospy.is_shutdown():
        htask = HeadTask()
        htask.pitch = 60.0
        btask = BodyTask()
        if imgResult.has_ball:
            x = imgResult.ball.x
            y = imgResult.ball.y
            print("head: {}, ball: {}, {}".format(headAngles.pitch, x, y))
            htask.pitch = headAngles.pitch
            if headAngles.pitch > 45:
                if y<330:
                    btask.type = BodyTask.TASK_WALK
                    btask.count = 2
                    if x<200:
                        btask.lateral = 0.02
                    elif x>440:
                        btask.lateral = -0.02
                    else:
                        btask.step = 0.02
                else:
                    if x<200:
                        btask.type = BodyTask.TASK_WALK
                        btask.lateral = 0.02
                        btask.count = 2
                    elif x<290:
                        btask.type = BodyTask.TASK_ACT
                        btask.actname = 'left_kick'
                    elif x<350:
                        btask.type = BodyTask.TASK_WALK
                        btask.lateral = -0.02
                        btask.lateral = 2
                    elif x<440:
                        btask.type = BodyTask.TASK_ACT
                        btask.actname = 'right_kick'
                    else:
                        btask.type = BodyTask.TASK_WALK
                        btask.lateral = -0.02
                        btask.count = 2
            else:
                btask.type = BodyTask.TASK_WALK
                btask.count = 2
                if x<200:
                    btask.lateral = 0.02
                elif x>440:
                    btask.lateral = -0.02
                else:
                    btask.step = 0.02
        else:
            htask.pitch = math.fabs(math.sin(i)*60.0)
            print(htask.pitch)
            i = i+0.05
        print("body task: {} {} {}".format(btask.type, btask.step, btask.lateral))
        bodyTaskPublisher.publish(btask) 
        headTaskPublisher.publish(htask)           
        rate.sleep()
