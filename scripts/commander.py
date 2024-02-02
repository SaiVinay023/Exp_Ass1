#!/usr/bin/env python

import rospy
import time
import math
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point

def UserAction():

    while True:
        comm = String()
        comm = random.choice(['play', 'sleep'])
        speech = rospy.Publisher("command",String,queue_size=10)
        speech.publish(comm)

        if comm == 'play':
            PersonPosition = Point()
            PersonPosition.x = random.randrange(1,11,1)
            PersonPosition.y = random.randrange(1,11,1)
            PersonPosition.z = 0

            pos = rospy.Publisher("PersonPosition", Point, queue_size = 10)
            pos.publish(PersonPosition)
            PointingGesture = Point()
            PointingGesture.x = random.randrange(1,11,1)
            PointingGesture.y = random.randrange(1,11,1)
            PointingGesture.z = 0
            point = rospy.Publisher("PointingGesture", Point, queue_size = 10)
            point.publish(PointingGesture)

        time.sleep(5)

if __name__ == '__main__':

    rospy.init_node('commander')
    UserAction()