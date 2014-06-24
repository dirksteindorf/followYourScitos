#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String

def CommandCallback(data):
    rospy.loginfo(data.data)

    if data.data == "scitos here":
        answerToScitos.publish("heard you")

    if data.data == "follow me":
        answerToScitos.publish("searching")
        time.sleep(2)
        answerToScitos.publish("marker found")

    if data.data == "stop following":
        answerToScitos.publish("stopping")
        time.sleep(3)
        answerToScitos.publish("done")

if __name__ == '__main__':
    rospy.init_node('pseudoKuka2')

    rospy.Subscriber('scitosChatter2', String, CommandCallback)
    answerToScitos = rospy.Publisher("kuka2Chatter", String)
    
    rospy.spin()
