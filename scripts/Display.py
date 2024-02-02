#!/usr/bin/env python

import rospy
import time
import math
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point

NewTarget = Point()

## odomCallback for new target position
def odomCallback(data):
    NewTarget.x = data.x
    NewTarget.y = data.y
    NewTarget.z = data.z

## Initializing Position
old_Position = Point()
old_Position.x = 0
old_Position.y = 0
old_Position.z = 0

## Read position funtion

def ReadPosition():
    rospy.loginfo('I am waiting for new target')
    ## subsribe to newTargetPosition
    rospy.Subscriber("newTargetPosition", Point, odomCallback)
    ## wait fo mmessage
    NewTarget = rospy.wait_for_message("newTargetPosition", Point)

    ## Main Loop
    while not ((old_Position.x == NewTarget.x) and (old_Position.y == NewTarget.y)):
        ## Get the state Parameter
        state = rospy.get_param('state')
        ## Normal state
        if state == 1:
            rospy.loginfo('we are in normal state')
            TimetoGetPosition = rospy.get_param('TimetoGetPosition')

            rospy.loginfo(' Target posiiton Achieved')
            time.sleep(TimetoGetPosition)

        ## Sleep State
        if state == 2:
            rospy.loginfo('we are in sleep state')
            TimetoGetPosition = rospy.get_param('TimetoGetPosition')
            time.sleep(TimetoGetPosition)
            rospy.loginfo('Home posiiton is achieved and sleeping')
            Timeforsleeping = rospy.get_param('TimeforSleeping')
            time.sleep('TimeforSleeping')

        ## Play State

        if state == 3:
            rospy.loginfo('in play state')
            TimetoGetPosition = rospy.get_param('TimetoGetPosition')
            time.sleep(TimetoGetPosition)
            rospy.loginfo('person location is achieved and waiting for new pointing gesture')
            WaitingForANewPointingGesture = rospy.get_param('WaitingForANewPointingGesture')
            time.sleep(WaitingForANewPointingGesture)
            time.sleep(TimetoGetPosition)
            rospy.loginfo(' The pointed location is achieved')
            time.sleep(TimetoGetPosition)
            rospy.loginfo('I came back and waiting for a new pointing gesture')
            time.sleep(WaitingForANewPointingGesture)

## Main function

if __name__ == "__main__":
    ## initializinf ros node
    rospy.init_node('display')
    ReadPosition()

