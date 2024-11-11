#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import LinkStates
from cartesian_state_msgs.msg import PoseTwist
import numpy as np

pub = rospy.Publisher("/ee_state", PoseTwist, queue_size=10)

class nokov_state_pub():
    def __init__(self):
        rospy.init_node('nokov_state_pub')
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
        self.state = PoseTwist()


    def callback(self, msg):
        self.state.pose = msg.pose[19]
        self.state.twist = msg.twist[19]
        #print(self.state)
        pub.publish(self.state)

#def nokov_state_pub():

if __name__ == '__main__':
    nokov_state_pub()        
    rospy.spin()
