#!/usr/bin/env python

import rospy
import time
import math
import random
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import Point

pose = Point()

def GenerateRandomPosition():

    pose.x = random.randrange(1,11,1)
    pose.y = random.randrange(1,11,1)
    pose.z = 0
    return pose

userdata = String()

def commcallback(data):
    userdata = data.data

PersonPosition = Point()


def PersonPositioncallback(pose):
    PersonPosition.x = pose.x
    PersonPosition.y = pose.y
    PersonPosition.z = 0

PointingGesture = Point()

def BallPositioncallback(pose):
    BallPosition.x = pose.x
    BallPosition.y = pose.y
    BallPosition.z = 0

BallPosition = Point()

def PointingGesturecallback(pose2):
    PointingGesture.x = pose2.x
    PointingGesture.y = pose2.y
    PointingGesture.z = 0

 ## Normal state definition
class Normal(smach.State):

    ## Initialization
    def __init__(self):
        smach.State.__init__(self, outcomes = [ 'outcome1', 'outcome2'])
        self.RandomPose = Point()

    ## Execution
    def execute(self, userdata):
        ## set parameter serice state = 1
        rospy.set_param('state', 1)

        ## Main Loop
        while True:
            ## subscribe to command Topic
            rospy.Subscriber("command", String, commcallback)
            usercommand = rospy.wait_for_message("command", String)

            targ = rospy.Publisher("/newTargetPosition", Point, queue_size=10)
            ## Generate random target Posiiton
            self.RandomPose = GenerateRandomPosition()
            rospy.loginfo('x target is %s', self.RandomPose.x)
            rospy.loginfo('y target is %s', self.RandomPose.y)
            ## Publish the random position in topic
            targ.publish(self.RandomPose)
            rospy.loginfo('command received is %s', usercommand.data)
            TimetoGetPosition = rospy.get_param("/TimetoGetPosition")
            ## sleep for a while
            time.sleep(TimetoGetPosition)

            ## choose the outcome
            if usercommand.data == "play" :
                return 'outcome2'
            if usercommand.data == "sleep" :
                return 'outcome1'

class Sleep(smach.State):
    ## Initialization
    def __init__(self):
        ## 1 outcome defined as noral
        smach.State.__init__(self, outcomes=['outcome1'])
        self.home = Point()

    ## Execution

    def execute(self, userdata):
        ## set paraeter service state to 2
        rospy.set_param('state',2)
        ## Define the home position
        self.home.x = 1
        self.home.y = 1
        self.home.z = 0
        rospy.loginfo('x target = x home %s', self.home.x)
        rospy.loginfo('y target = y home %s', self.home.y)
        targ = rospy.Publisher("/newTargetPosition", Point, queue_size=10)
        ## publlish the home position
        targ.publish(self.home)
        TimeforSleeping = rospy.get_param("/TimeforSleeping")
        ## sleep for a while
        time.sleep(14)

        return 'outcome1'
## pLay state definition
class Play(smach.State):
    ## Initialization
    def __init__(self):
        ## 1 outcome defined : Normal
        smach.State.__init__(self, outcomes= ['outcome2'])
        self.location = Point()
        self.PointingGesture = Point()
    ## Execution
    def execute(self, userdata):
        ## set parameter service state =3
        rospy.set_param('state', 3)
        ## subscribe to PersonPosition topic
        rospy.Subscriber("PersonPostion", Point, PersonPositioncallback)
        position = rospy.wait_for_message("PersonPosition", Point)
        rospy.loginfo('Location person x = %s', position.x)
        rospy.loginfo('Location Person y =%s', position.y)
        pos = rospy.Publisher("/newTargetPosition", Point, queue_size=10)
        ## publish Person Position for displaying
        pos.publish(PersonPosition)
        TimetoGetPosition = rospy.get_param("/TimetoGetPosition")
        ## wait for a while
        time.sleep(TimetoGetPosition)
        WaitingForANewPointingGesture = rospy.get_param("WaitForANewpointingGesture")
        ## wait for a new pointing Gesture
        time.sleep(WaitingForANewPointingGesture)
        ## subsribe to Pointing Gesture Topic
        rospy.Subscriber("PointingGesture", Point, PointingGesturecallback)
        point = rospy.wait_for_message("PointingGesture", Point)
        rospy.loginfo('PointingGesture x = %s', point.x)
        rospy.loginfo('PointingGesture y = %s', point.y)
        ## wait for desired location and coming back to person position
        time.sleep(TimetoGetPosition)
        ## wait for a new Pointing gesture
        time.sleep(WaitingForANewPointingGesture)

        ## return to Normal state
        return 'outcome2'

    class Random(smach.State):

    def __init__(self):
        ## 1 outcome defined as noral
        smach.State.__init__(self, outcomes=['outcome3'])
        self.RandomPose = Point()
        self.home = Point()
    def execute(self, userdata):

        rospy.set_param('state', 3)

        ## Main Loop
        while True:
            ## subscribe to ball positon Topic
            rospy.Subscriber("ballposition", String, commcallback)
            usercommand = rospy.wait_for_message("ballposition", String)
            self.home.x = 1
            self.home.y = 1
            self.home.z = 0
            rospy.loginfo('x target = x home %s', self.home.x)
            rospy.loginfo('y target = y home %s', self.home.y)
            targ = rospy.Publisher("/newTargetPosition", Point, queue_size=10)
        ## publlish the home position
            targ.publish(self.home)
            TimeforSleeping = rospy.get_param("/TimeforSleeping")
        ## sleep for a while
            time.sleep(14)

            ## Generate random target Posiiton
            self.RandomPose = GenerateRandomPosition()
            rospy.loginfo('x target is %s', self.RandomPose.x)
            rospy.loginfo('y target is %s', self.RandomPose.y)
            ## Publish the random position in topic
            targ.publish(self.RandomPose)
            rospy.loginfo('command received is %s', usercommand.data)
            TimetoGetPosition = rospy.get_param("/TimetoGetPosition")
            ## sleep for a while
            time.sleep(TimetoGetPosition)


## Main function

def main():
    ## Initialize the Ros Node
    rospy.init_node('state_machine')

    ## create SMACH state Machine
    sm = smach.StateMachine(outcomes= ['outcome4'])

    ## open the container
    with sm:
        ## Add states to the container
        smach.StateMachine.add('NORMAL', Normal(),transitions={'outcome1':'SLEEP', 'outcome2': 'PLAY'} )
        smach.StateMachine.add('SLEEP', Sleep(), transitions={'outcome1': 'NORMAL'})
        smach.StateMachine.add('PLAY', Play(), transitions={'outcome2':'NORMAL'})
        smach.StateMachine.add('Random',  Random(), transitions={'outcome3': 'RANDOM'})
    ## Create and start server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Exwcute smach plan
    outcome = sm.execute()
    # wait for Ctrl-c to stop the Application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

