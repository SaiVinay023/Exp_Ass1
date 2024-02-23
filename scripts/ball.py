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
        speech = rospy.Publisher("BallPosition",String,queue_size=10)
        speech.publish(comm)

            ## Added Ball position
            BallPosition = Point()
            BallPosition.x = random.randrange(1,11,1)
            BallPosition.y = random.randrange(1,11,1)
            BallPosition.z = 0

            pos = rospy.Publisher("BallPosition", Point, queue_size = 10)
            pos.publish(BallPosition)


        time.sleep(5)

if __name__ == '__main__':

    rospy.init_node('ball')
    UserAction()